"""
MG90S steering servo on GP22 -- 50 Hz PWM, Pico 2 W / MicroPython.

RULE CONTEXT (11.3 / 11.5)
--------------------------
This is the vehicle's ONLY steering actuator. Ackermann-style, one servo.
Differential thrust is forbidden and is not implementable anywhere in this
codebase -- see drv8833.py, which has no per-channel API.

DESIGN NOTES
------------
* Angles are carried in CENTI-DEGREES relative to the calibrated centre,
  signed, POSITIVE = RIGHT. Centre is therefore exactly 0, which means a
  config that fails to load leaves the wheels straight rather than at some
  arbitrary "90" that only makes sense with the right trim.

* Three independent limits, in order:
    1. steer_left_cdeg / steer_right_cdeg  -- software travel limit
    2. servo_left_us / servo_right_us      -- pulse-width limit
    3. slew rate                           -- how fast we may move
  Any one of them alone is enough to protect the linkage. All three are
  applied because they fail differently: a bad config trips (1), a bad
  interpolation trips (2), and a control-loop oscillation trips (3).

* SLEW LIMITING IS NOT COSMETIC. An MG90S asked for a 40 deg step draws
  its stall current for the whole transit, and that current comes off the
  same Mini560 5 V rail that feeds the Pi 5. Rate-limiting the servo is
  part of the brown-out story, not just a smoothness tweak.

* We never detach/idle the servo mid-round. A 9 g servo holding against
  the steering rack's return spring draws far less than it does moving,
  and an un-driven servo wanders -- rule 9.24.2 wants us STOPPED and
  stable in the finish section, which includes not drifting the wheels.
"""

from machine import Pin, PWM

SERVO_FREQ_HZ = 50            # standard analogue servo frame
_PERIOD_US = 1000000 // SERVO_FREQ_HZ   # 20000 us

# Absolute pulse-width sanity floor/ceiling. Config values are clamped into
# this window before anything reaches the PWM peripheral, so a corrupted
# MSG_CONFIG cannot command a pulse the servo will try to track past its
# mechanical stop.
_US_ABS_MIN = 700
_US_ABS_MAX = 2300


class Servo(object):

    def __init__(self, pin_num, centre_us=1500, left_us=1050, right_us=1950,
                 left_cdeg=-2800, right_cdeg=2800, slew_cdeg_s=60000):
        self._pwm = PWM(Pin(pin_num))
        self._pwm.freq(SERVO_FREQ_HZ)
        self.centre_us = centre_us
        self.left_us = left_us
        self.right_us = right_us
        self.left_cdeg = left_cdeg
        self.right_cdeg = right_cdeg
        self.slew_cdeg_s = slew_cdeg_s
        self._cmd_cdeg = 0            # what the controller asked for
        self._out_cdeg = 0            # what we have actually slewed to
        self._last_us = None
        self.apply(0)                 # wheels straight the instant we power up

    # -- configuration -----------------------------------------------------

    def configure(self, centre_us=None, left_us=None, right_us=None,
                  left_cdeg=None, right_cdeg=None, slew_cdeg_s=None):
        """Applied from MSG_CONFIG at arm time. Every value is sanity-clamped
        here rather than trusted, because this data crossed a UART."""
        if centre_us is not None:
            self.centre_us = _clamp(centre_us, _US_ABS_MIN, _US_ABS_MAX)
        if left_us is not None:
            self.left_us = _clamp(left_us, _US_ABS_MIN, _US_ABS_MAX)
        if right_us is not None:
            self.right_us = _clamp(right_us, _US_ABS_MIN, _US_ABS_MAX)
        if left_cdeg is not None:
            self.left_cdeg = _clamp(left_cdeg, -9000, 0)
        if right_cdeg is not None:
            self.right_cdeg = _clamp(right_cdeg, 0, 9000)
        if slew_cdeg_s is not None:
            self.slew_cdeg_s = _clamp(slew_cdeg_s, 1000, 200000)
        # Re-emit at the new trim immediately; otherwise the wheels stay at
        # the old centre until the next command, which during the arming
        # sequence looks exactly like a dead servo.
        self.apply(self._cmd_cdeg)

    # -- command path ------------------------------------------------------

    def set_angle_cdeg(self, cdeg):
        """Store the request. Nothing moves until update() runs -- keeping
        the slew integration on the loop's fixed timestep is what makes the
        rate limit a real rate and not 'however often commands happened to
        arrive'."""
        self._cmd_cdeg = _clamp(int(cdeg), self.left_cdeg, self.right_cdeg)

    def update(self, dt_ms):
        """Advance the slew limiter by dt_ms and drive the PWM. Call once per
        control tick."""
        step = (self.slew_cdeg_s * dt_ms) // 1000
        if step < 1:
            step = 1
        err = self._cmd_cdeg - self._out_cdeg
        if err > step:
            self._out_cdeg += step
        elif err < -step:
            self._out_cdeg -= step
        else:
            self._out_cdeg = self._cmd_cdeg
        self._write_us(self._cdeg_to_us(self._out_cdeg))

    def apply(self, cdeg):
        """Bypass the slew limiter. Boot and emergency paths only: on an
        e-stop we want the wheels centred NOW, and the extra current for one
        transit is irrelevant next to the collision we are avoiding."""
        self._cmd_cdeg = _clamp(int(cdeg), self.left_cdeg, self.right_cdeg)
        self._out_cdeg = self._cmd_cdeg
        self._write_us(self._cdeg_to_us(self._out_cdeg))

    # -- introspection (telemetry / debugging) -----------------------------

    @property
    def commanded_cdeg(self):
        return self._cmd_cdeg

    @property
    def actual_cdeg(self):
        """What the slew limiter has actually reached. If this lags the
        command persistently, the controller is asking for more steering
        rate than the servo can deliver and your gains are too hot."""
        return self._out_cdeg

    @property
    def pulse_us(self):
        return self._last_us

    # -- internals ---------------------------------------------------------

    def _cdeg_to_us(self, cdeg):
        # Two-sided linear interpolation from the centre. NOT a single
        # left->right lerp: real steering linkages are asymmetric, and a
        # single span silently biases one direction. Two spans let the
        # calibration capture the asymmetry.
        if cdeg >= 0:
            span_cdeg = self.right_cdeg if self.right_cdeg else 1
            us = self.centre_us + (self.right_us - self.centre_us) * cdeg // span_cdeg
        else:
            span_cdeg = self.left_cdeg if self.left_cdeg else -1
            us = self.centre_us + (self.left_us - self.centre_us) * cdeg // span_cdeg
        lo = min(self.left_us, self.right_us)
        hi = max(self.left_us, self.right_us)
        return _clamp(us, max(lo, _US_ABS_MIN), min(hi, _US_ABS_MAX))

    def _write_us(self, us):
        if us == self._last_us:
            return                    # no-op writes are free to skip
        self._last_us = us
        # duty_u16 is 0..65535 over the 20 ms frame.
        self._pwm.duty_u16((us * 65535) // _PERIOD_US)

    def deinit(self):
        self._pwm.deinit()


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)
