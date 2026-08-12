"""
=============================================================================
 WRO 2026 Future Engineers -- Pico 2 W "Brainstem"
 Real-time controller: mux sequencing, sensor sampling, actuation, safety.
=============================================================================

 WHAT THIS BOARD IS FOR
 ----------------------
 Everything on this board is something that must happen at a predictable
 time, or must happen even if Linux is busy. It sequences the I2C mux,
 samples seven devices on a fixed schedule, counts encoder edges in a
 hardware ISR, synthesises the servo and motor PWM, and -- critically --
 runs safety interlocks that can stop the vehicle WITHOUT asking the Pi.

 What it deliberately does NOT do: decide anything about the mission. It
 does not know what a lap is, which way a red pillar should be passed, or
 that parking exists. Those are macro decisions and they live on the Pi 5.
 The one place that boundary blurs is line CLASSIFICATION (see the colour
 sensor slot) -- classification is a cheap micro-decision on high-rate data,
 while what a crossing MEANS stays on the Pi.

 WHY MICROPYTHON AND NOT THE C/C++ SDK
 -------------------------------------
 We measured the actual duty cycle before choosing:
   * encoder edges:      ~1400 IRQ/s  (300 rpm out, ~1:20 box, 7 PPR, x2)
   * I2C devices/tick:   2.25 average (see SENSOR_SCHEDULE below)
   * I2C bus time:       ~0.4 ms per VL53L0X result read at 400 kHz
   * budget per tick:    10.0 ms at 100 Hz
 The loop runs at roughly a quarter of its budget in MicroPython on a
 150 MHz RP2350. C would buy headroom we have no use for, and cost us the
 ability to poke a sensor from a REPL at 1 a.m. the night before the round.
 That is the whole argument. If the loop ever exceeds budget the tick
 overrun counter in telemetry will say so, loudly, before it matters.

 THE RADIO (RULE 11.10, CONFLICT C1)
 -----------------------------------
 PREFERRED: flash the NON-W MicroPython build (RPI_PICO2) onto this Pico
 2 W. The CYW43 driver is then absent from the firmware entirely -- the
 radio is never initialised, and there is no code path that could turn it
 on. That is a much stronger claim to make to a judge than "we called
 active(False)".
 This is exactly why the status LED is on GP17 and the battery sense is on
 GP26: the onboard LED and VSYS sense both hang off the CYW43, and we
 refuse to depend on a chip we are contractually obliged to keep asleep.
 If a W build IS flashed, _disable_radios() below powers the interfaces
 down and, if it cannot prove they are down, raises FAULT_RADIO_ON and the
 vehicle REFUSES TO ARM.

 LOOP STRUCTURE
 --------------
 Fixed 100 Hz timestep. There is no sleep() anywhere in the loop: the pacing
 gap is spent servicing the UART receiver, which is useful work. A sleep
 would idle the CPU while frames queue up in a 32-byte FIFO.

=============================================================================
"""

import time
import machine
from machine import Pin, I2C, UART, ADC, WDT

# Deployed flat onto the Pico by tools/deploy_pico.sh, so these are top-level
# imports here but package imports on the Pi. See drivers/__init__.py.
try:
    from protocol import (
        FrameParser, encode_frame, MSG_TELEMETRY, MSG_COMMAND, MSG_PING,
        MSG_EVENT, MSG_CONFIG, MSG_ACK, encode_telemetry, decode_command,
        decode_config, encode_event, decode_ping, encode_ack, seq_is_newer,
        RANGE_INVALID, ACK_OK, ACK_PARTIAL,
        TOF_FC, TOF_FL, TOF_FR, TOF_DIAG, TOF_REAR,
        PICO_ST_BOOT, PICO_ST_SELFTEST, PICO_ST_FAULT, PICO_ST_IDLE,
        PICO_ST_ARMED, PICO_ST_RUN, PICO_ST_LINKLOSS, PICO_ST_ESTOP,
        FLAG_ESTOP_LATCHED, FLAG_TOUCH_L, FLAG_TOUCH_R, FLAG_START_LATCHED,
        FLAG_ARMED, FLAG_LINK_OK, FLAG_WDT_RESET, FLAG_LOW_BATTERY,
        FAULT_MUX, FAULT_IMU, FAULT_COLOUR, FAULT_TOF_ANY, FAULT_MOTOR,
        FAULT_I2C_BUS, FAULT_RADIO_ON, FAULT_CONFIG,
        CMDF_ARM, CMDF_BRAKE, CMDF_ALLOW_REVERSE, CMDF_CLEAR_ESTOP,
        CMDF_ZERO_HEADING, CMDF_RESET_ODOM, CMDF_PARKING_PROF,
        EV_BOOT, EV_SELFTEST_PASS, EV_SELFTEST_FAIL, EV_STATE_CHANGE,
        EV_ESTOP, EV_ESTOP_CLEARED, EV_TOF_DEGRADED, EV_TOF_RECOVERED,
        EV_LINK_LOST, EV_LINK_RESTORED, EV_START_PRESSED, EV_CONFIG_APPLIED,
        EV_MOTOR_FAULT, EV_LOW_BATTERY, EV_CMD_REJECTED,
        ESTOP_CAUSE_TOUCH_L, ESTOP_CAUSE_TOUCH_R, ESTOP_CAUSE_FRONT_TOF,
        ESTOP_CAUSE_REAR_TOF, ESTOP_CAUSE_PI_REQUEST, ESTOP_CAUSE_MOTOR_FAULT,
        CFG_SERVO_CENTRE_US, CFG_SERVO_LEFT_US, CFG_SERVO_RIGHT_US,
        CFG_STEER_LEFT_CDEG, CFG_STEER_RIGHT_CDEG, CFG_SERVO_SLEW_CDEG_S,
        CFG_UM_PER_COUNT, CFG_FRONT_STOP_MM, CFG_REAR_STOP_MM,
        CFG_MOTOR_RAMP_PCT_S, CFG_MOTOR_MAX_PCT, CFG_SPEED_KP_X100,
        CFG_SPEED_KI_X100, CFG_LINK_TIMEOUT_MS, CFG_LOW_BATT_MV,
        CFG_VBAT_DIV_X100,
    )
    from tca9548a import TCA9548A, MuxError
    from vl53l0x import VL53L0X
    from bno085 import BNO085
    from tcs34725 import TCS34725, LineClassifier
    from drv8833 import DRV8833
    from servo import Servo
    from encoder import Encoder
except ImportError:                       # running from the repo, off-target
    from common.protocol import *         # noqa -- dev convenience only
    from pico.drivers.tca9548a import TCA9548A, MuxError
    from pico.drivers.vl53l0x import VL53L0X
    from pico.drivers.bno085 import BNO085
    from pico.drivers.tcs34725 import TCS34725, LineClassifier
    from pico.drivers.drv8833 import DRV8833
    from pico.drivers.servo import Servo
    from pico.drivers.encoder import Encoder


# =============================================================================
# PIN MAP -- code addresses GPIO numbers, never physical pin numbers.
# Physical pins are given only so this table can be checked against the PCB.
# =============================================================================

PIN_UART_TX   = 0    # phys 1  -> Pi 5 header pin 10 (GPIO15, RXD0)
PIN_UART_RX   = 1    # phys 2  <- Pi 5 header pin  8 (GPIO14, TXD0)
PIN_I2C_SDA   = 4    # phys 6  -- 4.7k pull-up to 3V3
PIN_I2C_SCL   = 5    # phys 7  -- 4.7k pull-up to 3V3
PIN_DRV_FAULT = 6    # phys 9  -- ADDED (not in blueprint): nFAULT, open-drain
PIN_DRV_SLEEP = 7    # phys 10 -- ADDED: nSLEEP, the hardware motor kill
PIN_MOTOR_IN1 = 8    # phys 11
PIN_MOTOR_IN2 = 9    # phys 12
PIN_ENC_A     = 12   # phys 16 -- hardware IRQ, both edges
PIN_ENC_B     = 13   # phys 17 -- quadrature direction
PIN_TOUCH_L   = 14   # phys 19 -- ADDED: rear-left bumper
PIN_TOUCH_R   = 15   # phys 20 -- ADDED: rear-right bumper
PIN_START_BTN = 16   # phys 21 -- ADDED: the ONE start button (rule 9.11)
PIN_STATUS_LED= 17   # phys 22 -- ADDED: external LED (NOT the CYW43 one)
PIN_COLOUR_LED= 18   # phys 24 -- ADDED: TCS34725 illumination control
PIN_SERVO     = 22   # phys 29
PIN_VBAT_ADC  = 26   # phys 31 -- ADDED, optional: 100k/33k divider off 7.4 V

# GP23/24/25/29 are NOT used: on a Pico 2 W they belong to the CYW43.
# GP20/21/27/28 are free for future expansion.

UART_ID = 0
UART_BAUD = 460800   # 6.6% utilised at 50 Hz both ways -- ample headroom
I2C_ID = 0
I2C_FREQ = 400000    # fast mode; the VL53L0X and BNO085 both support it

# =============================================================================
# MUX MAP -- a NAMED dictionary, never index arithmetic.
# The channel order is not contiguous by sensor type (ToFs on 0,3,4,5,6 with
# colour and IMU interleaved on 1 and 2) because it follows the PCB routing.
# Any loop that assumed "ToF n lives on channel n" would silently read the
# IMU as a range. Lives in firmware rather than config.json because this is
# WIRING, not a tunable: changing it requires a soldering iron.
# =============================================================================

MUX_ADDR = 0x70
MUX_CHANNELS = {
    "tof_front_centre": 0,
    "colour":           1,
    "imu":              2,
    "tof_front_left":   3,
    "tof_front_right":  4,
    "tof_front_diag":   5,
    "tof_rear":         6,    # CONFLICT C2 resolved: 5th ToF goes here
    # channel 7 reserved for diagnostics
}

# Expected I2C address on each channel. The self-test checks this: "something
# ACKed" is not the same as "the right device is there" -- on this bus a
# mis-selected channel would also ACK at 0x29.
MUX_EXPECT_ADDR = {
    "tof_front_centre": 0x29, "colour": 0x29, "imu": 0x4A,
    "tof_front_left": 0x29, "tof_front_right": 0x29,
    "tof_front_diag": 0x29, "tof_rear": 0x29,
}

TOF_ORDER = ("tof_front_centre", "tof_front_left", "tof_front_right",
             "tof_front_diag", "tof_rear")
TOF_INDEX = {"tof_front_centre": TOF_FC, "tof_front_left": TOF_FL,
             "tof_front_right": TOF_FR, "tof_front_diag": TOF_DIAG,
             "tof_rear": TOF_REAR}

# =============================================================================
# SENSOR SCHEDULE
# =============================================================================
# (name, period_ticks, phase_ticks, priority)  at LOOP_HZ = 100.
#
# Rates were chosen from what each measurement is FOR, not from what the
# sensor can do:
#
#  front-centre 50 Hz : it arms the emergency interlock. Interlock latency is
#                       one sample period (20 ms) plus the brake ramp -- far
#                       from the "microseconds" a bumper gives us, but ~10x
#                       faster than any camera round-trip, which is the
#                       comparison that matters for this rule.
#  IMU          50 Hz : closes every 90 deg turn. Under-sampling yaw during a
#                       ~1.2 s corner is directly an overshoot.
#  colour       50 Hz : THE tightest constraint on this table. The line is
#                       20 mm wide; at 700 mm/s the sensor is over it for
#                       ~28 ms. At 25 Hz we would miss crossings outright,
#                       and a missed crossing is a missed lap.
#  side ToFs    25 Hz : wall-following input. At 700 mm/s that is 28 mm of
#                       travel between samples -- well inside the lateral
#                       error we can correct.
#  diag/rear  12.5 Hz : the diagonal is a corner-confirmation hint and the
#                       rear only matters while parking (where we are doing
#                       220 mm/s, so 12.5 Hz is 18 mm/sample).
#
# Average load: 0.5+0.5+0.5+0.25+0.25+0.125+0.125 = 2.25 devices/tick.
# Cap of MAX_DEVICES_PER_TICK keeps a phase collision from blowing the
# budget; the lowest-priority due sensor simply waits one tick.
SENSOR_SCHEDULE = (
    # name,               period, phase, priority (0 = most important)
    ("tof_front_centre",       2,     0,  0),
    ("imu",                    2,     1,  0),
    ("colour",                 2,     0,  1),
    ("tof_front_left",         4,     1,  2),
    ("tof_front_right",        4,     3,  2),
    ("tof_front_diag",         8,     5,  3),
    ("tof_rear",               8,     7,  3),
)
MAX_DEVICES_PER_TICK = 3

LOOP_HZ = 100
TICK_US = 1000000 // LOOP_HZ
TELEMETRY_EVERY_N_TICKS = 2          # 50 Hz

# --- hard safety envelope (firmware floors; the Pi cannot weaken these) ----
HARD_FRONT_STOP_MM = 110
HARD_REAR_STOP_MM = 90
HARD_STOP_MAX_MM = 800               # ceiling on what the Pi may request
DEFAULT_LINK_TIMEOUT_MS = 300
DEFAULT_LINK_DECEL_MS = 250
WDT_TIMEOUT_MS = 400                 # 4 missed ticks

TOF_INVALID_TOLERANCE = 8            # hold last valid for N cycles
TOF_REINIT_AFTER = 40                # then attempt a software re-init
TOF_REINIT_MAX = 3                   # then give up on that sensor, keep going

TOUCH_DEBOUNCE_MS = 20


# =============================================================================
# Status LED patterns -- (period_ticks, on_ticks) or a custom blink list.
# At 2 a.m. with no laptop attached, this LED is the entire UI. Each state
# gets a visually distinct rhythm, not just a different rate.
# =============================================================================
LED_PATTERNS = {
    PICO_ST_BOOT:     (100, 50),     # 1 Hz square -- "thinking"
    PICO_ST_SELFTEST: (20, 10),      # 5 Hz square -- "testing"
    PICO_ST_FAULT:    (20, 16),      # mostly ON, short gaps -- "broken"
    PICO_ST_IDLE:     (200, 20),     # short wink each 2 s -- "healthy, waiting"
    PICO_ST_ARMED:    (100, 80),     # mostly on -- "armed, press start"
    PICO_ST_RUN:      (0, 1),        # solid
    PICO_ST_LINKLOSS: (10, 5),       # 10 Hz -- "panicking politely"
    PICO_ST_ESTOP:    (6, 3),        # ~17 Hz frantic -- "stopped, help"
}

# Legal transitions. Every state change goes through _set_state(), which
# refuses illegal moves and logs them. No hidden state in flag variables:
# if you want to know what the Brainstem is doing, there is exactly one
# integer to look at.
TRANSITIONS = {
    PICO_ST_BOOT:     (PICO_ST_SELFTEST,),
    PICO_ST_SELFTEST: (PICO_ST_IDLE, PICO_ST_FAULT),
    PICO_ST_FAULT:    (PICO_ST_SELFTEST,),
    PICO_ST_IDLE:     (PICO_ST_ARMED, PICO_ST_FAULT, PICO_ST_ESTOP),
    PICO_ST_ARMED:    (PICO_ST_RUN, PICO_ST_IDLE, PICO_ST_ESTOP,
                       PICO_ST_LINKLOSS),
    PICO_ST_RUN:      (PICO_ST_IDLE, PICO_ST_ESTOP, PICO_ST_LINKLOSS),
    PICO_ST_LINKLOSS: (PICO_ST_RUN, PICO_ST_IDLE, PICO_ST_ESTOP),
    PICO_ST_ESTOP:    (PICO_ST_IDLE,),      # only when stationary + requested
}


# =============================================================================
def _sat16(v):
    """Saturate to int16. Used on every value packed into telemetry: an
    unhandled struct.error inside the control loop would stop the car far
    more effectively than any interlock, and always at the worst moment."""
    return -32768 if v < -32768 else (32767 if v > 32767 else v)


def _disable_radios():
    """Rule 11.10. Returns True only if we can PROVE no radio is up.

    On a non-W firmware build `import network` fails, which is the outcome we
    want and the strongest one available: there is no radio driver in the
    image at all. On a W build we power the interfaces down and verify.
    """
    try:
        import network
    except ImportError:
        return True                   # best case: no radio driver exists
    ok = True
    try:
        for iface_id in (network.STA_IF, network.AP_IF):
            w = network.WLAN(iface_id)
            w.active(False)
            if w.active():
                ok = False
        try:
            w.deinit()                # not present on every port/version
        except Exception:
            pass
    except Exception:
        ok = False
    try:
        import bluetooth
        ble = bluetooth.BLE()
        ble.active(False)
        if ble.active():
            ok = False
    except ImportError:
        pass
    except Exception:
        ok = False
    return ok


# =============================================================================
class Link(object):
    """Framed UART master-slave endpoint. Non-blocking in both directions."""

    def __init__(self, uart):
        self.uart = uart
        self.parser = FrameParser()
        self.tx_seq = 0
        self.rx_cmd_seq = None
        self.last_cmd_ms = 0
        self.cmd_count = 0
        self.cmd_rejected = 0
        self.lost_frames = 0

    def service_rx(self):
        """Drain whatever the UART has. Returns a list of (type, seq, payload).
        Called both from the tick body AND from the inter-tick pacing gap, so
        the 32-byte hardware FIFO never has time to overflow."""
        n = self.uart.any()
        if not n:
            return ()
        data = self.uart.read(n)
        if not data:
            return ()
        return self.parser.feed(data)

    def send(self, mtype, payload=b""):
        """Fire and forget. We never block on a full TX buffer: dropping a
        telemetry frame is free (the next one is 20 ms away), whereas
        blocking the control loop to deliver one is not."""
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        try:
            self.uart.write(encode_frame(mtype, self.tx_seq, payload))
        except OSError:
            pass


# =============================================================================
class SensorHub(object):
    """Owns the mux and every device behind it, plus the recovery ladder.

    All I2C in the firmware goes through here, which is what makes the
    "one channel at a time" invariant checkable by reading one file.
    """

    def __init__(self, i2c, colour_led_pin):
        self.i2c = i2c
        self.mux = TCA9548A(i2c, MUX_ADDR)
        self.tofs = {}
        for name in TOF_ORDER:
            self.tofs[name] = VL53L0X(i2c, name, MUX_CHANNELS[name])
        self.imu = BNO085(i2c)
        self.colour = TCS34725(i2c, led_pin=colour_led_pin)
        self.classifier = LineClassifier()

        self.tof_mm = [RANGE_INVALID] * 5
        self.tof_valid_bits = 0
        self.tof_degraded_bits = 0
        self.tof_hold_cycles = [0] * 5
        self.tof_reinit_cycles = [0] * 5

        self.colour_sample = None
        self.colour_class = 0
        self.colour_clear = 0

        self.fault_bits = 0
        self.i2c_error_streak = 0
        self.events = []              # (event_id, arg) queued for the Pi
        self._tick = 0
        self._parking_profile = False

    # -- boot self-test ----------------------------------------------------

    def self_test(self):
        """Probe every mux channel, confirm each device ID, initialise.

        Runs ONCE, before the start button, with the results reported on the
        LED and over the link. Rule 9.9 forbids calibration at the start;
        this is not calibration, it is a presence check -- and it is the
        difference between finding a dead sensor in the pit and finding it
        halfway round lap 2.
        """
        self.fault_bits = 0
        report = []

        if not self.mux.probe():
            self.fault_bits |= FAULT_MUX
            report.append("MUX 0x70 NOT FOUND -- nothing else can be read")
            return False, report

        # Prove the mux actually SWITCHES, not merely that it ACKs. A mux that
        # answers but never latches returns the same sensor five times, which
        # looks like a working car until the first corner.
        self.mux.select(3)
        rb = self.mux.read_control()
        if rb != 0x08:
            self.fault_bits |= FAULT_MUX
            report.append("MUX control readback %r, expected 0x08" % rb)
            return False, report

        for name in TOF_ORDER:
            ch = MUX_CHANNELS[name]
            tof = self.tofs[name]
            try:
                with self.mux.channel(ch) as bus:
                    found = bus.scan()
                    if MUX_EXPECT_ADDR[name] not in found:
                        tof.degraded = True
                        report.append("%s: no 0x29 on ch%d (saw %s)"
                                      % (name, ch, [hex(a) for a in found]))
                        continue
                    if tof.init(long_range=True, timing_budget_us=33000):
                        report.append("%s: OK (ch%d)" % (name, ch))
                    else:
                        tof.degraded = True
                        report.append("%s: INIT FAILED on ch%d" % (name, ch))
            except MuxError as e:
                tof.degraded = True
                report.append("%s: mux error %s" % (name, e))

        degraded = [n for n in TOF_ORDER if self.tofs[n].degraded]
        for n in degraded:
            self.tof_degraded_bits |= 1 << TOF_INDEX[n]
        if degraded:
            self.fault_bits |= FAULT_TOF_ANY

        # IMU
        ch = MUX_CHANNELS["imu"]
        try:
            with self.mux.channel(ch) as bus:
                found = bus.scan()
                if 0x4A not in found and 0x4B not in found:
                    self.fault_bits |= FAULT_IMU
                    report.append("imu: nothing at 0x4A/0x4B on ch%d (saw %s)"
                                  % (ch, [hex(a) for a in found]))
                elif self.imu.init():
                    report.append("imu: OK, accuracy=%d" % self.imu.accuracy)
                else:
                    self.fault_bits |= FAULT_IMU
                    report.append("imu: init/settle FAILED -- hold the car "
                                  "still and retry")
        except MuxError as e:
            self.fault_bits |= FAULT_IMU
            report.append("imu: mux error %s" % e)

        # Colour
        ch = MUX_CHANNELS["colour"]
        try:
            with self.mux.channel(ch) as bus:
                if 0x29 not in bus.scan():
                    self.fault_bits |= FAULT_COLOUR
                    report.append("colour: nothing at 0x29 on ch%d" % ch)
                elif self.colour.init():
                    report.append("colour: OK id=0x%02X" % self.colour.device_id)
                else:
                    self.fault_bits |= FAULT_COLOUR
                    report.append("colour: init FAILED (id=%r)"
                                  % self.colour.device_id)
        except MuxError as e:
            self.fault_bits |= FAULT_COLOUR
            report.append("colour: mux error %s" % e)

        # ARMING POLICY. Deliberately not "everything must be perfect":
        #   * the mux, the IMU and the colour sensor are MANDATORY -- without
        #     them we cannot turn, cannot count laps, and cannot steer at all.
        #   * front-centre ToF is MANDATORY -- it arms the interlock.
        #   * we tolerate ONE dead side/rear ToF. Wall-following degrades to
        #     single-wall mode, which we can actually do; refusing to run
        #     would guarantee zero points instead of most of them.
        fatal = bool(self.fault_bits & (FAULT_MUX | FAULT_IMU | FAULT_COLOUR))
        if self.tofs["tof_front_centre"].degraded:
            fatal = True
            report.append("FATAL: front-centre ToF is the interlock sensor")
        if len(degraded) > 1:
            fatal = True
            report.append("FATAL: %d ToFs degraded; only 1 is tolerable"
                          % len(degraded))
        return (not fatal), report

    # -- scheduled sampling ------------------------------------------------

    def set_parking_profile(self, on):
        """Re-prioritise for reversing: the rear ToF becomes as important as
        the front one, and the front-diagonal (a corner hint) becomes
        irrelevant because we are not looking for corners while parking."""
        self._parking_profile = on

    def _due(self, tick):
        due = []
        for name, period, phase, prio in SENSOR_SCHEDULE:
            p, ph = period, phase
            if self._parking_profile:
                if name == "tof_rear":
                    p, ph = 2, 1
                elif name == "tof_front_diag":
                    p = 16
            if (tick % p) == ph:
                due.append((prio, name))
        due.sort()
        return due[:MAX_DEVICES_PER_TICK]

    def service(self, tick, now_ms):
        """Sample whatever is due this tick. Never raises; every failure path
        ends in 'mark it and carry on'."""
        self._tick = tick
        for _prio, name in self._due(tick):
            try:
                if name == "imu":
                    with self.mux.channel(MUX_CHANNELS["imu"]):
                        self.imu.poll(now_ms)
                elif name == "colour":
                    with self.mux.channel(MUX_CHANNELS["colour"]) as _bus:
                        s = self.colour.read_raw()
                    if s is not None:
                        self.colour_sample = s
                        self.colour_clear = s[0]
                        self.colour_class = self.classifier.classify(s)
                else:
                    self._service_tof(name, now_ms)
                self.i2c_error_streak = 0
            except MuxError:
                # Concurrent access or a failed channel switch. Skip this
                # sample; the schedule will come back around in 20-80 ms.
                self.i2c_error_streak += 1
            except OSError:
                self.i2c_error_streak += 1

        if self.i2c_error_streak > 20:
            # A sustained error storm means the bus itself is wedged -- almost
            # always a downstream device holding SDA. Deselecting every mux
            # channel physically disconnects the culprit, which is the only
            # lever we have without XSHUT wiring.
            self.fault_bits |= FAULT_I2C_BUS
            self.mux.recover_bus()
            self.i2c_error_streak = 0

    def _service_tof(self, name, now_ms):
        idx = TOF_INDEX[name]
        tof = self.tofs[name]
        if tof.degraded:
            self.tof_valid_bits &= ~(1 << idx)
            return

        with self.mux.channel(MUX_CHANNELS[name]):
            mm = tof.read_mm(now_ms)

            if mm is not None:
                self.tof_mm[idx] = mm
                self.tof_valid_bits |= 1 << idx
                self.tof_hold_cycles[idx] = 0
                self.tof_reinit_cycles[idx] = 0
                return

            # No fresh valid sample. Rung 1 of the recovery ladder: keep
            # serving the last good value for a bounded number of cycles.
            # Bounded is the whole point -- an unbounded hold means the car
            # confidently steers on a range it measured a second ago.
            self.tof_hold_cycles[idx] += 1
            self.tof_reinit_cycles[idx] += 1

            if self.tof_hold_cycles[idx] > TOF_INVALID_TOLERANCE:
                self.tof_valid_bits &= ~(1 << idx)
                self.tof_mm[idx] = RANGE_INVALID

            # Rung 2: software re-init through the mux.
            if self.tof_reinit_cycles[idx] >= TOF_REINIT_AFTER:
                self.tof_reinit_cycles[idx] = 0
                if tof.attempt_recovery(TOF_REINIT_MAX):
                    self.events.append((EV_TOF_RECOVERED, idx))
                elif tof.degraded:
                    # Rung 3: give up on this one, tell the Pi, keep driving.
                    self.tof_degraded_bits |= 1 << idx
                    self.fault_bits |= FAULT_TOF_ANY
                    self.events.append((EV_TOF_DEGRADED, idx))

    def tof(self, idx):
        """Last known range, or None. The ONLY way the interlocks read a ToF:
        it enforces the valid mask so no code path can mistake 'no data' for
        'zero millimetres'."""
        if not (self.tof_valid_bits & (1 << idx)):
            return None
        v = self.tof_mm[idx]
        return None if v == RANGE_INVALID else v


# =============================================================================
class Brainstem(object):

    def __init__(self):
        self.boot_ms = time.ticks_ms()
        try:
            self.wdt_reset = (machine.reset_cause() == machine.WDT_RESET)
        except AttributeError:
            self.wdt_reset = False

        self.state = PICO_ST_BOOT
        self.fault_bits = 0
        self.radio_ok = _disable_radios()
        if not self.radio_ok:
            self.fault_bits |= FAULT_RADIO_ON

        # --- I/O ---
        self.led = Pin(PIN_STATUS_LED, Pin.OUT, value=0)
        self.colour_led = Pin(PIN_COLOUR_LED, Pin.OUT, value=0)
        self.uart = UART(UART_ID, baudrate=UART_BAUD,
                         tx=Pin(PIN_UART_TX), rx=Pin(PIN_UART_RX),
                         timeout=0, timeout_char=0, rxbuf=512, txbuf=512)
        self.i2c = I2C(I2C_ID, sda=Pin(PIN_I2C_SDA), scl=Pin(PIN_I2C_SCL),
                       freq=I2C_FREQ)
        self.link = Link(self.uart)
        self.sensors = SensorHub(self.i2c, self.colour_led)

        self.motor = DRV8833(PIN_MOTOR_IN1, PIN_MOTOR_IN2,
                             sleep_pin=PIN_DRV_SLEEP, fault_pin=PIN_DRV_FAULT)
        self.servo = Servo(PIN_SERVO)
        self.encoder = Encoder(PIN_ENC_A, PIN_ENC_B)

        # Inputs. All active-low with internal pull-ups: a broken wire then
        # reads as "not pressed" for the button (safe: we never auto-start)
        # and as "not touched" for the bumpers. The bumper failure direction
        # is the uncomfortable one, which is exactly why the bumpers are a
        # SECONDARY interlock behind the ToF, never the primary one.
        self.touch_l = Pin(PIN_TOUCH_L, Pin.IN, Pin.PULL_UP)
        self.touch_r = Pin(PIN_TOUCH_R, Pin.IN, Pin.PULL_UP)
        self.start_btn = Pin(PIN_START_BTN, Pin.IN, Pin.PULL_UP)

        self._touch_l_ms = 0
        self._touch_r_ms = 0
        self.touch_l_hit = False
        self.touch_r_hit = False
        self.touch_l.irq(trigger=Pin.IRQ_FALLING, handler=self._isr_touch_l)
        self.touch_r.irq(trigger=Pin.IRQ_FALLING, handler=self._isr_touch_r)

        self._start_edge_ms = 0
        self._start_pending = False   # MUST exist before the IRQ is armed
        self.start_latched = False    # rule 9.11: ONE start, latched forever
        self.start_btn.irq(trigger=Pin.IRQ_FALLING, handler=self._isr_start)

        self.vbat_adc = ADC(PIN_VBAT_ADC) if PIN_VBAT_ADC is not None else None
        self.vbat_mv = 0
        self.vbat_div_x100 = 403
        self.low_batt_mv = 6800
        self._low_batt_reported = False

        # --- command state ---
        self.cmd_steer_cdeg = 0
        self.cmd_speed_mm_s = 0
        self.cmd_flags = 0
        self.pi_state = 0
        self.front_stop_mm = HARD_FRONT_STOP_MM
        self.rear_stop_mm = HARD_REAR_STOP_MM
        self.link_timeout_ms = DEFAULT_LINK_TIMEOUT_MS
        self.link_decel_ms = DEFAULT_LINK_DECEL_MS
        self.last_cmd_ms = 0
        self.config_applied = False
        self.cmd_seq_echo = 0

        # --- speed controller (PI on encoder velocity) ---
        # Lives on the Pico because it needs the encoder at tick rate and has
        # nothing to do with the mission. The Pi asks for mm/s, not for PWM.
        self.speed_kp_x100 = 45
        self.speed_ki_x100 = 12
        self._speed_i = 0
        self._motor_pct = 0

        # --- emergency ---
        self.estop_latched = False
        self.estop_cause = 0
        self._linkloss_start_ms = 0
        self._link_was_ok = True

        self.tick = 0
        self.overruns = 0
        self.max_tick_us = 0

        self.wdt = None               # started after the self-test passes

    # -- ISRs (minimal: timestamp and set a flag; nothing else) ------------

    def _isr_touch_l(self, pin):
        t = time.ticks_ms()
        if time.ticks_diff(t, self._touch_l_ms) > TOUCH_DEBOUNCE_MS:
            self._touch_l_ms = t
            self.touch_l_hit = True

    def _isr_touch_r(self, pin):
        t = time.ticks_ms()
        if time.ticks_diff(t, self._touch_r_ms) > TOUCH_DEBOUNCE_MS:
            self._touch_r_ms = t
            self.touch_r_hit = True

    def _isr_start(self, pin):
        t = time.ticks_ms()
        if time.ticks_diff(t, self._start_edge_ms) > 50:
            self._start_edge_ms = t
            self._start_pending = True

    # -- state machine -----------------------------------------------------

    def _set_state(self, new):
        if new == self.state:
            return True
        allowed = TRANSITIONS.get(self.state, ())
        if new not in allowed:
            # Refuse and report. An illegal transition is a logic bug, and the
            # safe response is to stay where we are -- states are ordered by
            # how much authority they grant, and skipping one always grants
            # more than intended.
            self._emit(EV_CMD_REJECTED, (self.state << 8) | new)
            return False
        old = self.state
        self.state = new
        self._emit(EV_STATE_CHANGE, (old << 8) | new)
        return True

    def _emit(self, event_id, arg=0):
        self.link.send(MSG_EVENT, encode_event(time.ticks_ms(), event_id, arg))

    # -- emergency ---------------------------------------------------------

    def trigger_estop(self, cause):
        """Latched. Kills the H-bridge in HARDWARE (nSLEEP low), not just in
        PWM registers -- so the motor is stopped even if this loop dies in
        the next microsecond."""
        if self.estop_latched:
            return
        self.estop_latched = True
        self.estop_cause = cause
        self.motor.brake()
        self.motor.update(10, 0)
        self.motor.disable()
        self.servo.apply(0)
        self._speed_i = 0
        self._emit(EV_ESTOP, cause)
        self._set_state(PICO_ST_ESTOP)

    def _check_interlocks(self, now_ms):
        """Runs EVERY tick, in every state, before any command is applied.
        This is the Pico's unilateral authority and it does not consult the
        Pi, the mission state, or anything that arrived over a wire."""
        # Bumpers: rear-mounted, collision detection ONLY. Rule 9.24.7 ends
        # the round if we touch a parking-lot limitation, so these must never
        # be used to feel for the parking walls -- and they are not: nothing
        # in the parking path reads them. If one fires, we have already hit
        # something and the only correct response is to stop.
        if self.touch_l_hit:
            self.touch_l_hit = False
            self.trigger_estop(ESTOP_CAUSE_TOUCH_L)
            return
        if self.touch_r_hit:
            self.touch_r_hit = False
            self.trigger_estop(ESTOP_CAUSE_TOUCH_R)
            return

        if self.motor.fault():
            self._emit(EV_MOTOR_FAULT, 0)
            self.fault_bits |= FAULT_MOTOR
            # NOT an e-stop: the DRV8833 already protected itself in hardware
            # and usually recovers within a millisecond. Log it, keep racing.

        if self.state not in (PICO_ST_RUN, PICO_ST_LINKLOSS):
            return

        # Direction-aware ranging interlocks. The front sensor is irrelevant
        # while reversing into a parking bay and the rear is irrelevant while
        # driving forward; applying both unconditionally would make the
        # parking manoeuvre impossible.
        moving_fwd = self._motor_pct > 0 or self.encoder.speed_mm_s > 40
        moving_rev = self._motor_pct < 0 or self.encoder.speed_mm_s < -40

        if moving_fwd:
            d = self.sensors.tof(TOF_FC)
            if d is not None and d < self.front_stop_mm:
                self.trigger_estop(ESTOP_CAUSE_FRONT_TOF)
                return
        if moving_rev:
            d = self.sensors.tof(TOF_REAR)
            if d is not None and d < self.rear_stop_mm:
                self.trigger_estop(ESTOP_CAUSE_REAR_TOF)
                return

    # -- link --------------------------------------------------------------

    def _handle_frames(self, frames, now_ms):
        for mtype, seq, payload in frames:
            if mtype == MSG_COMMAND:
                self._handle_command(seq, payload, now_ms)
            elif mtype == MSG_PING:
                self.link.send(MSG_ACK, encode_ack(MSG_PING, seq, ACK_OK,
                                                   decode_ping(payload)))
            elif mtype == MSG_CONFIG:
                n = self._handle_config(payload)
                self.link.send(MSG_ACK,
                               encode_ack(MSG_CONFIG, seq,
                                          ACK_OK if n else ACK_PARTIAL, n))
                self._emit(EV_CONFIG_APPLIED, n)

    def _handle_command(self, seq, payload, now_ms):
        if len(payload) < 14:
            self.link.cmd_rejected += 1
            return
        # Replay / reorder protection. A stale command is not merely useless:
        # applying an old steering angle after a newer one is a real, if
        # brief, wrong-way steer.
        if self.link.rx_cmd_seq is not None and \
                not seq_is_newer(seq, self.link.rx_cmd_seq):
            self.link.cmd_rejected += 1
            self._emit(EV_CMD_REJECTED, seq)
            return
        if self.link.rx_cmd_seq is not None:
            self.link.lost_frames += (seq - self.link.rx_cmd_seq - 1) & 0xFF
        self.link.rx_cmd_seq = seq
        self.link.cmd_count += 1
        self.last_cmd_ms = now_ms
        self.cmd_seq_echo = seq

        c = decode_command(payload)
        self.cmd_flags = c["flags"]
        self.pi_state = c["pi_state"]

        # --- clamp everything that crossed the wire ---
        self.cmd_steer_cdeg = c["steer_cdeg"]     # servo.py clamps to travel

        spd = c["speed_mm_s"]
        if spd < 0 and not (self.cmd_flags & CMDF_ALLOW_REVERSE):
            # Two independent bits must agree before we reverse. A single
            # corrupted sign bit that survives CRC must not be able to send
            # the car backwards mid-lap.
            spd = 0
        self.cmd_speed_mm_s = spd

        # ONE-SIDED clamp: the Pi may only ask us to stop EARLIER.
        req_f = c["front_stop_mm"]
        self.front_stop_mm = min(max(req_f, HARD_FRONT_STOP_MM),
                                 HARD_STOP_MAX_MM)
        req_r = c["rear_stop_mm"]
        self.rear_stop_mm = min(max(req_r, HARD_REAR_STOP_MM), HARD_STOP_MAX_MM)

        if self.cmd_flags & CMDF_ZERO_HEADING:
            self.sensors.imu.zero_heading()
        if self.cmd_flags & CMDF_RESET_ODOM:
            self.encoder.reset()
        self.sensors.set_parking_profile(bool(self.cmd_flags & CMDF_PARKING_PROF))

        # E-stop may ONLY be cleared while genuinely stationary. Otherwise a
        # Pi bug could clear a collision stop while the car is still rolling
        # into whatever it hit.
        if (self.cmd_flags & CMDF_CLEAR_ESTOP) and self.estop_latched:
            if abs(self.encoder.speed_mm_s) < 20:
                self.estop_latched = False
                self.estop_cause = 0
                self._emit(EV_ESTOP_CLEARED, 0)
                self._set_state(PICO_ST_IDLE)

        # Arming. CMDF_ARM must be held CONTINUOUSLY -- it is a dead-man
        # switch, not a latch. If the Pi stops asserting it we disarm.
        if self.cmd_flags & CMDF_ARM:
            if self.state == PICO_ST_IDLE and self._may_arm():
                if self._set_state(PICO_ST_ARMED):
                    self.motor.enable()
        else:
            if self.state in (PICO_ST_ARMED, PICO_ST_RUN, PICO_ST_LINKLOSS):
                self._disarm()

    def _may_arm(self):
        """Rule 11.10 is enforced here as well as at boot: if a radio is up we
        do not arm, full stop. Also refuses without a config -- driving on
        firmware defaults would mean driving on an uncalibrated servo centre."""
        if not self.radio_ok:
            return False
        if not self.config_applied:
            self.fault_bits |= FAULT_CONFIG
            return False
        return True

    def _disarm(self):
        self.motor.brake()
        self.motor.update(10, self.encoder.speed_mm_s)
        self.motor.disable()
        self.servo.apply(0)
        self._speed_i = 0
        self._motor_pct = 0
        self._set_state(PICO_ST_IDLE)

    def _handle_config(self, payload):
        applied = 0
        for key, val in decode_config(payload):
            if key == CFG_SERVO_CENTRE_US:
                self.servo.configure(centre_us=val)
            elif key == CFG_SERVO_LEFT_US:
                self.servo.configure(left_us=val)
            elif key == CFG_SERVO_RIGHT_US:
                self.servo.configure(right_us=val)
            elif key == CFG_STEER_LEFT_CDEG:
                self.servo.configure(left_cdeg=val)
            elif key == CFG_STEER_RIGHT_CDEG:
                self.servo.configure(right_cdeg=val)
            elif key == CFG_SERVO_SLEW_CDEG_S:
                self.servo.configure(slew_cdeg_s=val)
            elif key == CFG_UM_PER_COUNT:
                self.encoder.configure(um_per_count=val)
            elif key == CFG_FRONT_STOP_MM:
                self.front_stop_mm = min(max(val, HARD_FRONT_STOP_MM),
                                         HARD_STOP_MAX_MM)
            elif key == CFG_REAR_STOP_MM:
                self.rear_stop_mm = min(max(val, HARD_REAR_STOP_MM),
                                        HARD_STOP_MAX_MM)
            elif key == CFG_MOTOR_RAMP_PCT_S:
                self.motor.configure(ramp_pct_s=val)
            elif key == CFG_MOTOR_MAX_PCT:
                self.motor.configure(max_pct=val)
            elif key == CFG_SPEED_KP_X100:
                self.speed_kp_x100 = val
            elif key == CFG_SPEED_KI_X100:
                self.speed_ki_x100 = val
            elif key == CFG_LINK_TIMEOUT_MS:
                self.link_timeout_ms = min(max(val, 100), 1000)
            elif key == CFG_LOW_BATT_MV:
                self.low_batt_mv = val
            elif key == CFG_VBAT_DIV_X100:
                self.vbat_div_x100 = val
            else:
                continue              # unknown key: ignore, do not fail
            applied += 1
        if applied:
            self.config_applied = True
            self.fault_bits &= ~FAULT_CONFIG
        return applied

    def _check_link(self, now_ms):
        """Fail-safe on Pi silence: decelerate to a CONTROLLED stop.

        Not a coast (we would keep travelling into whatever is ahead), and
        not a latched e-stop (a 350 ms hiccup should cost us a metre, not the
        round). If the link comes back we resume -- but only on a fresh,
        sequence-valid command, so a stuck buffer replaying old bytes cannot
        wake us up.
        """
        if self.state not in (PICO_ST_ARMED, PICO_ST_RUN, PICO_ST_LINKLOSS):
            return
        age = time.ticks_diff(now_ms, self.last_cmd_ms)
        if age > self.link_timeout_ms:
            if self.state != PICO_ST_LINKLOSS:
                self._linkloss_start_ms = now_ms
                self._set_state(PICO_ST_LINKLOSS)
                self._emit(EV_LINK_LOST, age)
            # Ramp the SETPOINT down over link_decel_ms; the motor's own ramp
            # then smooths the PWM. Steering is HELD, not centred: snapping
            # the wheels straight mid-corner would send us into the outer wall
            # while we are still moving.
            t = time.ticks_diff(now_ms, self._linkloss_start_ms)
            if t >= self.link_decel_ms:
                self.cmd_speed_mm_s = 0
                self.motor.brake()
            else:
                frac = (self.link_decel_ms - t) * 100 // self.link_decel_ms
                self.cmd_speed_mm_s = self.cmd_speed_mm_s * frac // 100
        elif self.state == PICO_ST_LINKLOSS:
            self._set_state(PICO_ST_RUN)
            self._emit(EV_LINK_RESTORED, age)

    # -- actuation ---------------------------------------------------------

    def _drive(self, dt_ms):
        """Closed-loop speed control. The Pi commands mm/s; converting that to
        PWM needs the encoder at tick rate, so it belongs here."""
        if self.state not in (PICO_ST_RUN, PICO_ST_LINKLOSS):
            self.servo.update(dt_ms)
            self.motor.update(dt_ms, self.encoder.speed_mm_s)
            return

        self.servo.set_angle_cdeg(self.cmd_steer_cdeg)
        self.servo.update(dt_ms)

        target = self.cmd_speed_mm_s
        if self.cmd_flags & CMDF_BRAKE:
            target = 0
            self.motor.brake()

        if target == 0:
            self._speed_i = 0
            self._motor_pct = 0
            self.motor.set_speed_pct(0)
        else:
            err = target - self.encoder.speed_mm_s
            self._speed_i += err * dt_ms // 1000
            # Anti-windup: clamp the integral to what it could contribute at
            # full authority. Without this, a stalled wheel (say, against a
            # wall) winds the integrator up and the car leaps the instant it
            # comes free.
            i_limit = 100 * 100 // max(self.speed_ki_x100, 1)
            self._speed_i = max(-i_limit, min(i_limit, self._speed_i))
            pct = (self.speed_kp_x100 * err +
                   self.speed_ki_x100 * self._speed_i) // 100 // 10
            # Feed-forward from the target so the PI only has to correct,
            # not to generate, the operating point. Roughly 10% PWM per
            # 100 mm/s -- MEASURE and replace with your own measured slope.
            pct += target // 10
            self._motor_pct = max(-100, min(100, pct))
            self.motor.set_speed_pct(self._motor_pct)

        self.motor.update(dt_ms, self.encoder.speed_mm_s)

    # -- housekeeping ------------------------------------------------------

    def _service_led(self):
        pat = LED_PATTERNS.get(self.state, (100, 50))
        period, on = pat
        if period == 0:
            self.led.value(1)
        else:
            self.led.value(1 if (self.tick % period) < on else 0)

    def _service_battery(self):
        if self.vbat_adc is None or self.vbat_div_x100 <= 0:
            self.vbat_mv = 0
            return
        raw = self.vbat_adc.read_u16()
        # 3.3 V reference, 16-bit scaled reading, then undo the divider.
        mv = (raw * 3300) // 65535
        self.vbat_mv = mv * self.vbat_div_x100 // 100
        if self.vbat_mv < self.low_batt_mv and not self._low_batt_reported:
            self._low_batt_reported = True
            self._emit(EV_LOW_BATTERY, self.vbat_mv)
            # WARNING ONLY. We never cut the motors mid-round for a tired
            # pack: finishing slowly always beats stopping on lap 2.

    def _send_telemetry(self, now_ms):
        flags = 0
        if self.estop_latched:
            flags |= FLAG_ESTOP_LATCHED
        if self.touch_l.value() == 0:
            flags |= FLAG_TOUCH_L
        if self.touch_r.value() == 0:
            flags |= FLAG_TOUCH_R
        if self.start_latched:
            flags |= FLAG_START_LATCHED
        if self.state in (PICO_ST_ARMED, PICO_ST_RUN, PICO_ST_LINKLOSS):
            flags |= FLAG_ARMED
        if self.state != PICO_ST_LINKLOSS:
            flags |= FLAG_LINK_OK
        if self.wdt_reset:
            flags |= FLAG_WDT_RESET
        if self.vbat_mv and self.vbat_mv < self.low_batt_mv:
            flags |= FLAG_LOW_BATTERY

        s = self.sensors
        # Saturate before packing. yaw_rate in centi-deg/s overflows int16 at
        # 327 deg/s, which a spin-out or a dropped car reaches easily -- and
        # an unhandled struct.error here would kill the control loop at the
        # exact moment the car most needs it. Saturating loses precision in a
        # situation where precision is already meaningless.
        payload = encode_telemetry(
            now_ms, s.tof_mm, s.tof_valid_bits, s.tof_degraded_bits,
            _sat16(int(s.imu.heading_deg * 100)),
            _sat16(int(s.imu.yaw_rate_dps * 100)),
            self.encoder.counts, _sat16(self.encoder.speed_mm_s),
            s.colour_clear, s.colour_class, self.state, flags,
            self.cmd_seq_echo, self.vbat_mv,
            self.fault_bits | s.fault_bits)
        self.link.send(MSG_TELEMETRY, payload)

        while s.events:
            eid, arg = s.events.pop(0)
            self._emit(eid, arg)

    # -- boot + main loop --------------------------------------------------

    def boot(self):
        print("\n=== WRO Brainstem boot ===")
        print("watchdog reset:", self.wdt_reset)
        print("radios down   :", self.radio_ok)
        self._emit(EV_BOOT, 1 if self.wdt_reset else 0)

        self._set_state(PICO_ST_SELFTEST)
        ok, report = self.sensors.self_test()
        for line in report:
            print("  " + line)
        self.fault_bits |= self.sensors.fault_bits
        if not self.radio_ok:
            print("  RADIO STILL UP -- rule 11.10; refusing to arm")
            ok = False

        if ok:
            print("SELF-TEST PASS")
            self._emit(EV_SELFTEST_PASS, 0)
            self._set_state(PICO_ST_IDLE)
        else:
            print("SELF-TEST FAIL bits=0x%02X" % self.fault_bits)
            self._emit(EV_SELFTEST_FAIL, self.fault_bits)
            self._set_state(PICO_ST_FAULT)

        # Watchdog starts only AFTER the self-test: init sequences legitimately
        # take hundreds of milliseconds and would trip it. From here on, any
        # tick that fails to complete within 400 ms resets the board -- and
        # FLAG_WDT_RESET tells the Pi it happened.
        self.wdt = WDT(timeout=WDT_TIMEOUT_MS)
        return ok

    def run(self):
        next_tick = time.ticks_add(time.ticks_us(), TICK_US)
        last_ms = time.ticks_ms()
        self._start_pending = False

        while True:
            t0 = time.ticks_us()
            now_ms = time.ticks_ms()
            dt_ms = time.ticks_diff(now_ms, last_ms)
            if dt_ms <= 0:
                dt_ms = 1000 // LOOP_HZ
            last_ms = now_ms

            # 1. Inputs that must never be missed.
            if self._start_pending:
                self._start_pending = False
                if not self.start_latched:
                    # Rule 9.11: ONE start trigger, and it is latched forever.
                    # There is no code path that un-latches it -- a second
                    # press mid-round does nothing at all.
                    self.start_latched = True
                    self._emit(EV_START_PRESSED, 0)
                    if self.state == PICO_ST_ARMED:
                        self._set_state(PICO_ST_RUN)

            frames = self.link.service_rx()
            if frames:
                self._handle_frames(frames, now_ms)

            # 2. Sensors (mux-sequenced, bounded devices per tick).
            self.sensors.service(self.tick, now_ms)
            self.encoder.update(dt_ms)

            # 3. Safety FIRST, before any commanded value is applied.
            self._check_interlocks(now_ms)
            self._check_link(now_ms)

            # 4. Actuate.
            self._drive(dt_ms)

            # 5. Report.
            if (self.tick % TELEMETRY_EVERY_N_TICKS) == 0:
                self._send_telemetry(now_ms)
            if (self.tick % 50) == 0:
                self._service_battery()
            self._service_led()

            self.wdt.feed()
            self.tick += 1

            used = time.ticks_diff(time.ticks_us(), t0)
            if used > self.max_tick_us:
                self.max_tick_us = used

            # 6. Pace the loop. NOT a sleep: we spend the slack draining the
            # UART, which keeps the 512-byte RX buffer from ever backing up
            # and gives command frames sub-tick latency instead of waiting
            # for the next tick boundary.
            if time.ticks_diff(next_tick, time.ticks_us()) <= 0:
                self.overruns += 1
                next_tick = time.ticks_add(time.ticks_us(), TICK_US)
            else:
                while time.ticks_diff(next_tick, time.ticks_us()) > 0:
                    f = self.link.service_rx()
                    if f:
                        self._handle_frames(f, time.ticks_ms())
                next_tick = time.ticks_add(next_tick, TICK_US)


# =============================================================================
def main():
    bs = Brainstem()
    bs.boot()
    # We enter the loop even after a failed self-test. The FAULT state grants
    # no motor authority, but it keeps telemetry flowing so the Pi can display
    # exactly WHICH sensor is missing -- which is the whole point of failing
    # loudly before the round instead of quietly during it.
    bs.run()


if __name__ == "__main__":
    main()
