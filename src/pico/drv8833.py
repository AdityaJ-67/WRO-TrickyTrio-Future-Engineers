"""
DRV8833 single-motor driver -- Pico 2 W / MicroPython.

RULE 11.5 IS ENFORCED BY THE API SHAPE, NOT BY DISCIPLINE
---------------------------------------------------------
"Differential/skid steering = disqualification." This class drives ONE
motor and exposes ONE speed. There is no channel argument anywhere in the
public interface, so there is no way to write code that produces different
thrust on two sides -- not because we remembered not to, but because the
function to do it does not exist.

If you parallel OUT3/OUT4 with OUT1/OUT2 for current headroom (conflict C5),
tie IN3/IN4 to IN1/IN2 in COPPER. Do not add a second PWM pair here.

WIRING
------
  IN1/AIN1  -> GP8   (PWM)
  IN2/AIN2  -> GP9   (PWM)
  nSLEEP    -> GP7   (added; NOT in the blueprint -- conflict C6)
  nFAULT    -> GP6   (added; input, open-drain, needs a pull-up)
  VM        -> 7.4 V LiPo direct
  OUT1/OUT2 -> N20 motor

WHY nSLEEP WAS ADDED
--------------------
Without it, "stop the motor" means "write 0 to two PWM registers and hope".
If the loop is wedged, or a PWM register is corrupted, or the RP2350 is
mid-reset, the last duty cycle keeps being emitted by the PWM hardware --
the motor keeps running while the CPU is dead. nSLEEP is a single GPIO that
puts the H-bridge outputs into high-Z regardless of what the IN pins say.
That is the difference between a software stop and an actual stop, and it
is what makes the EMERGENCY state trustworthy. It costs one wire.

DECAY MODE
----------
We drive SLOW DECAY (a.k.a. brake-mode PWM): the "off" phase of each PWM
cycle shorts the motor rather than letting it freewheel.
  forward:  IN1 = 100%, IN2 = PWM(1 - duty)
  reverse:  IN2 = 100%, IN1 = PWM(1 - duty)
Slow decay gives markedly better low-speed control and a much more linear
duty->speed curve, which matters because our parking manoeuvre lives at
~200 mm/s where fast decay is all stiction and no motion. The cost is
slightly higher ripple current, which the bulk capacitance absorbs.

DIRECTION REVERSAL
------------------
Commanding a hard reversal at speed dumps the motor's back-EMF straight
into the bridge and the shared 7.4 V rail -- a current spike big enough to
brown out the Pi 5 through the Mini560. So `set_speed()` NEVER reverses
directly: it brakes, waits for the ENCODER to report near-zero velocity,
and only then applies the opposite direction. The state machine for that
lives here (`_pending_dir`) so no caller can forget it.
"""

from machine import Pin, PWM

# 20 kHz: above the audible band (a 2 kHz whine during a technical
# inspection is a bad look) and comfortably inside the DRV8833's spec.
# Higher would raise switching losses in a driver we are already asking for
# ~1.5 A continuous.
PWM_FREQ_HZ = 20000

_MODE_COAST = 0
_MODE_BRAKE = 1
_MODE_DRIVE = 2


class DRV8833(object):

    def __init__(self, in1_pin, in2_pin, sleep_pin=None, fault_pin=None,
                 max_pct=75, ramp_pct_s=250):
        self._in1 = PWM(Pin(in1_pin))
        self._in2 = PWM(Pin(in2_pin))
        self._in1.freq(PWM_FREQ_HZ)
        self._in2.freq(PWM_FREQ_HZ)
        self._in1.duty_u16(0)
        self._in2.duty_u16(0)

        # Start ASLEEP. The vehicle is placed on the mat switched off and
        # powered on into a waiting state (rules 9.6/9.10/9.11) -- the motor
        # must be electrically incapable of turning until we explicitly arm.
        self._sleep = Pin(sleep_pin, Pin.OUT, value=0) if sleep_pin is not None else None
        self._fault = Pin(fault_pin, Pin.IN, Pin.PULL_UP) if fault_pin is not None else None

        self.max_pct = max_pct
        self.ramp_pct_s = ramp_pct_s

        self._target_pct = 0          # signed request, -100..100
        self._applied_pct = 0         # signed, after ramping
        self._mode = _MODE_COAST
        self._pending_dir = 0         # non-zero => waiting for zero-crossing
        self._pending_pct = 0         # magnitude to apply once we get there
        self._awake = False
        self.fault_latched = False

    # -- power / arming ----------------------------------------------------

    def enable(self):
        """Wake the bridge. Called ONLY when the Pico enters ARMED."""
        self._applied_pct = 0
        self._target_pct = 0
        self._write(0, _MODE_BRAKE)
        if self._sleep is not None:
            self._sleep.value(1)
        self._awake = True

    def disable(self):
        """Hardware kill: outputs high-Z, immediately, regardless of PWM
        state. This is the emergency path and it must not depend on anything
        else in this file being correct."""
        self._target_pct = 0
        self._applied_pct = 0
        self._pending_dir = 0
        self._in1.duty_u16(0)
        self._in2.duty_u16(0)
        self._mode = _MODE_COAST
        if self._sleep is not None:
            self._sleep.value(0)
        self._awake = False

    @property
    def enabled(self):
        return self._awake

    def configure(self, max_pct=None, ramp_pct_s=None):
        if max_pct is not None:
            self.max_pct = _clamp(max_pct, 0, 100)
        if ramp_pct_s is not None:
            self.ramp_pct_s = _clamp(ramp_pct_s, 20, 2000)

    # -- fault monitoring --------------------------------------------------

    def fault(self):
        """nFAULT is active LOW and open-drain: over-current, over-temp, or
        under-voltage lockout. We LOG it and let the mission continue --
        the DRV8833 already protected itself in hardware, and stopping the
        car because the driver briefly got warm loses more than it saves.
        A latched flag makes it visible in the journal afterwards."""
        if self._fault is None:
            return False
        f = (self._fault.value() == 0)
        if f:
            self.fault_latched = True
        return f

    # -- command path ------------------------------------------------------

    def set_speed_pct(self, pct):
        """Signed percentage: + forward, - reverse. Both bridge halves always
        receive the same magnitude and direction -- see the module docstring.

        Reversal is deferred, not immediate: see `_pending_dir`.
        """
        pct = _clamp(int(pct), -self.max_pct, self.max_pct)
        cur_dir = _sign(self._applied_pct)
        new_dir = _sign(pct)
        if cur_dir != 0 and new_dir != 0 and cur_dir != new_dir:
            # Reversal requested while moving: brake first, remember the
            # intent, and let update() release it once we have actually
            # stopped. The alternative -- trusting the caller to sequence
            # this -- fails the first time someone writes a parking routine
            # at midnight.
            self._pending_dir = new_dir
            self._pending_pct = abs(pct)
            self._target_pct = 0
        else:
            self._pending_dir = 0
            self._pending_pct = 0
            self._target_pct = pct

    def brake(self):
        """Active short across the windings. Used for the finish-section hold
        (rule 9.24.2 wants us genuinely stopped, and a coasting car on a
        smooth mat keeps creeping) and for every emergency stop."""
        self._target_pct = 0
        self._pending_dir = 0
        self._mode = _MODE_BRAKE

    def coast(self):
        self._target_pct = 0
        self._pending_dir = 0
        self._mode = _MODE_COAST

    def update(self, dt_ms, speed_mm_s=0):
        """Advance the ramp one tick. `speed_mm_s` comes from the encoder and
        is what releases a deferred reversal.

        Ramping is a POWER decision, not a comfort one: a hard 0->70% step on
        the shared 7.4 V pack drops the rail far enough to reset the Pi 5
        through the Mini560. Ramping over ~400 ms keeps the dip inside what
        the bulk capacitance can cover.
        """
        if self._pending_dir != 0:
            # Release the reversal only once the wheel has genuinely stopped.
            # 30 mm/s is below the encoder's useful resolution at our tick
            # rate, i.e. "as stopped as we can measure".
            if abs(speed_mm_s) < 30:
                self._target_pct = self._pending_dir * self._pending_pct
                self._pending_dir = 0
                self._pending_pct = 0
                self._mode = _MODE_DRIVE
            else:
                self._mode = _MODE_BRAKE
                self._target_pct = 0

        step = (self.ramp_pct_s * dt_ms) // 1000
        if step < 1:
            step = 1
        err = self._target_pct - self._applied_pct
        if err > step:
            self._applied_pct += step
        elif err < -step:
            self._applied_pct -= step
        else:
            self._applied_pct = self._target_pct

        if self._applied_pct == 0 and self._mode != _MODE_COAST:
            self._write(0, _MODE_BRAKE)
        elif self._applied_pct == 0:
            self._write(0, _MODE_COAST)
        else:
            self._mode = _MODE_DRIVE
            self._write(self._applied_pct, _MODE_DRIVE)

    @property
    def applied_pct(self):
        return self._applied_pct

    # -- internals ---------------------------------------------------------

    def _write(self, pct, mode):
        if not self._awake:
            # Belt and braces: even if something calls us while asleep, emit
            # zeros so waking up later cannot resurrect a stale duty cycle.
            self._in1.duty_u16(0)
            self._in2.duty_u16(0)
            return

        if mode == _MODE_COAST:
            self._in1.duty_u16(0)
            self._in2.duty_u16(0)
            return
        if mode == _MODE_BRAKE or pct == 0:
            # Both HIGH = both low-side... (DRV8833: IN1=IN2=1 -> brake/short)
            self._in1.duty_u16(65535)
            self._in2.duty_u16(65535)
            return

        duty = (abs(pct) * 65535) // 100
        inv = 65535 - duty            # slow-decay: modulate the OTHER pin
        if pct > 0:
            self._in1.duty_u16(65535)
            self._in2.duty_u16(inv)
        else:
            self._in2.duty_u16(65535)
            self._in1.duty_u16(inv)


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _sign(v):
    return 0 if v == 0 else (1 if v > 0 else -1)
