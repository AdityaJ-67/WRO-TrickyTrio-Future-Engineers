"""
VL53L0X time-of-flight ranger -- MicroPython, behind a TCA9548A.

Five of these, all at address 0x29, on mux channels 0, 3, 4, 5 and 6.
The caller is responsible for selecting the channel; this driver only ever
talks to 0x29 and has no idea which physical sensor it is. That is
deliberate -- it keeps the mux discipline in exactly one file
(tca9548a.py) instead of smeared across the sensor layer.

-----------------------------------------------------------------------------
 THE BLACK-WALL PROBLEM  (the single most important fact about this sensor
 on this track)
-----------------------------------------------------------------------------
 The VL53L0X emits at 940 nm and measures returned photons. The WRO track
 walls are BLACK and only 100 mm tall. Black paint absorbs most of that IR,
 so the return signal is a small fraction of what the datasheet's 2 m spec
 assumes. Expect roughly 600-900 mm of usable range against these walls,
 not 2000 mm.

 Consequences that shape the code above this driver:
   * On a 1000 mm lane, the FAR wall may simply not answer. "No return" is
     reported as an out-of-range status, NOT as a large distance -- so the
     wall-follower must tolerate a missing side and fall back to
     single-wall following. It does; see pi/control.py.
   * We therefore run LONG RANGE settings: signal-rate limit lowered to
     0.10 MCPS (from the 0.25 default) and a 33 ms timing budget. Both trade
     ambient-light immunity and update rate for sensitivity to weak returns,
     which is exactly the trade this track demands.
   * We do NOT lengthen the VCSEL pulse periods, the third part of ST's
     usual long-range preset. Changing them invalidates the timing-budget
     arithmetic in a way that must be recomputed in a specific order, and a
     subtly wrong budget produces plausible-but-wrong ranges -- the worst
     failure mode we have. Signal-rate limit plus timing budget got us the
     range we needed on the practice mat.
     MEASURE: record your own black-wall max range here after bench testing;
     the wall-follow config key `far_wall_saturation_mm` must match it.

-----------------------------------------------------------------------------
 NO XSHUT -> SOFTWARE RECOVERY IS THE ONLY RECOVERY
-----------------------------------------------------------------------------
 The PCB leaves every XSHUT floating (internal pull-up). Acceptable, because
 the mux already solves the address collision -- but it means a sensor that
 wedges cannot be hardware-reset. The recovery ladder is built in from the
 start rather than bolted on after the first mid-round freeze:

     invalid reading      -> hold last valid value (bounded cycles)
     N invalid in a row   -> deselect the mux channel, re-run init()
     M failed re-inits    -> mark permanently DEGRADED, tell the Pi,
                             and KEEP DRIVING on the remaining sensors

 The last rung matters most. A car that finishes on four of five ToFs
 scores; a car that stops to mourn one scores nothing.

-----------------------------------------------------------------------------
 CONTINUOUS MODE, NEVER SINGLE-SHOT
-----------------------------------------------------------------------------
 In single-shot you start a measurement and wait ~33 ms for it. That is
 three whole control ticks of blocking, per sensor. In back-to-back
 continuous mode the sensor ranges on its own schedule and read_mm() just
 collects whatever is ready -- a ~0.4 ms I2C transaction with no waiting.
 The cost is that a reading may be up to one integration period old, which
 we account for by timestamping every sample.
"""

import time

_ADDR = 0x29

# --- registers (ST API names, abbreviated) ---------------------------------
_SYSRANGE_START = 0x00
_SYSTEM_SEQUENCE_CONFIG = 0x01
_SYSTEM_INTERMEASUREMENT_PERIOD = 0x04
_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A
_SYSTEM_INTERRUPT_CLEAR = 0x0B
_RESULT_INTERRUPT_STATUS = 0x13
_RESULT_RANGE_STATUS = 0x14
_MSRC_CONFIG_CONTROL = 0x60
_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44
_PRE_RANGE_CONFIG_VCSEL_PERIOD = 0x50
_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x51
_MSRC_CONFIG_TIMEOUT_MACROP = 0x46
_FINAL_RANGE_CONFIG_VCSEL_PERIOD = 0x70
_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI = 0x71
_IDENTIFICATION_MODEL_ID = 0xC0
_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0
_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F
_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E
_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6

_MODEL_ID_EXPECTED = 0xEE

# Range status codes we accept. The VL53L0X packs its status into bits 3:6 of
# RESULT_RANGE_STATUS. 0x0B (11) is the only "measurement valid" code; every
# other value means sigma failure, signal failure, phase failure or hardware
# fail -- all of which we must treat as NO DATA, never as a distance.
_RANGE_VALID = 11

RANGE_INVALID = 0xFFFF          # mirrors protocol.RANGE_INVALID

# ST's reference tuning table. Undocumented magic from the vendor API; every
# working VL53L0X driver ships this verbatim. Do not "clean it up".
_TUNING = (
    (0xFF, 0x01), (0x00, 0x00), (0xFF, 0x00), (0x09, 0x00), (0x10, 0x00),
    (0x11, 0x00), (0x24, 0x01), (0x25, 0xFF), (0x75, 0x00), (0xFF, 0x01),
    (0x4E, 0x2C), (0x48, 0x00), (0x30, 0x20), (0xFF, 0x00), (0x30, 0x09),
    (0x54, 0x00), (0x31, 0x04), (0x32, 0x03), (0x40, 0x83), (0x46, 0x25),
    (0x60, 0x00), (0x27, 0x00), (0x50, 0x06), (0x51, 0x00), (0x52, 0x96),
    (0x56, 0x08), (0x57, 0x30), (0x61, 0x00), (0x62, 0x00), (0x64, 0x00),
    (0x65, 0x00), (0x66, 0xA0), (0xFF, 0x01), (0x22, 0x32), (0x47, 0x14),
    (0x49, 0xFF), (0x4A, 0x00), (0xFF, 0x00), (0x7A, 0x0A), (0x7B, 0x00),
    (0x78, 0x21), (0xFF, 0x01), (0x23, 0x34), (0x42, 0x00), (0x44, 0xFF),
    (0x45, 0x26), (0x46, 0x05), (0x40, 0x40), (0x0E, 0x06), (0x20, 0x1A),
    (0x43, 0x40), (0xFF, 0x00), (0x34, 0x03), (0x35, 0x44), (0xFF, 0x01),
    (0x31, 0x04), (0x4B, 0x09), (0x4C, 0x05), (0x4D, 0x04), (0xFF, 0x00),
    (0x44, 0x00), (0x45, 0x20), (0x47, 0x08), (0x48, 0x28), (0x67, 0x00),
    (0x70, 0x04), (0x71, 0x01), (0x72, 0xFE), (0x76, 0x00), (0x77, 0x00),
    (0xFF, 0x01), (0x0D, 0x01), (0xFF, 0x00), (0x80, 0x01), (0x01, 0xF8),
    (0xFF, 0x01), (0x8E, 0x01), (0x00, 0x01), (0xFF, 0x00), (0x80, 0x00),
)

# Fixed overheads from ST's API, in microseconds. Used by the timing-budget
# solver below.
_START_OVERHEAD = 1910
_END_OVERHEAD = 960
_MSRC_OVERHEAD = 660
_TCC_OVERHEAD = 590
_DSS_OVERHEAD = 690
_PRE_RANGE_OVERHEAD = 660
_FINAL_RANGE_OVERHEAD = 550
_MIN_TIMING_BUDGET_US = 20000


class VL53L0X(object):
    """One physical sensor. `name` and `mux_channel` are carried purely so
    that log lines and fault events can say WHICH sensor without the caller
    having to remember."""

    def __init__(self, i2c, name="tof", mux_channel=-1):
        self.i2c = i2c
        self.name = name
        self.mux_channel = mux_channel

        self.ok = False
        self.degraded = False           # permanently given up on
        self.init_attempts = 0          # total, including the boot init
        self.recovery_attempts = 0      # re-inits after the boot init only

        self.last_mm = RANGE_INVALID
        self.last_valid_mm = RANGE_INVALID
        self.last_valid_ms = 0
        self.invalid_streak = 0
        self.read_count = 0
        self.error_count = 0
        self.range_status = 0

        self._stop_variable = 0
        self._buf = bytearray(12)       # RESULT_RANGE_STATUS block
        self._b2 = bytearray(2)
        self._b1 = bytearray(1)

    # -- register access ---------------------------------------------------

    def _w8(self, reg, val):
        self._b1[0] = val
        self.i2c.writeto_mem(_ADDR, reg, self._b1)

    def _r8(self, reg):
        self.i2c.readfrom_mem_into(_ADDR, reg, self._b1)
        return self._b1[0]

    def _w16(self, reg, val):
        self._b2[0] = (val >> 8) & 0xFF
        self._b2[1] = val & 0xFF
        self.i2c.writeto_mem(_ADDR, reg, self._b2)

    def _r16(self, reg):
        self.i2c.readfrom_mem_into(_ADDR, reg, self._b2)
        return (self._b2[0] << 8) | self._b2[1]

    # -- initialisation ----------------------------------------------------

    def init(self, long_range=True, timing_budget_us=33000):
        """Full ST init sequence. Boot path and recovery path only -- it
        blocks for a few milliseconds and must never run inside the control
        loop. Returns True on success.

        Every step is wrapped: a sensor that vanishes mid-init must leave us
        with ok=False, not with a half-configured device that returns
        confident nonsense.
        """
        self.init_attempts += 1
        try:
            if self._r8(_IDENTIFICATION_MODEL_ID) != _MODEL_ID_EXPECTED:
                self.ok = False
                return False

            # --- data init: 2.8 V mode, then the vendor's magic preamble ---
            self._w8(0x89, self._r8(0x89) | 0x01)
            self._w8(0x88, 0x00)
            self._w8(0x80, 0x01)
            self._w8(0xFF, 0x01)
            self._w8(0x00, 0x00)
            # stop_variable must be captured here and replayed on every
            # start_continuous(); it is per-device state, not a constant.
            self._stop_variable = self._r8(0x91)
            self._w8(0x00, 0x01)
            self._w8(0xFF, 0x00)
            self._w8(0x80, 0x00)

            # Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks.
            self._w8(_MSRC_CONFIG_CONTROL, self._r8(_MSRC_CONFIG_CONTROL) | 0x12)

            # Signal-rate limit, 9.7 fixed point MCPS.
            # 0.10 for long range (black walls), 0.25 is the ST default.
            limit_mcps = 0.10 if long_range else 0.25
            self._w16(_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                      int(limit_mcps * (1 << 7)))

            self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xFF)

            # --- reference SPAD management ---
            spad_count, is_aperture = self._get_spad_info()
            if spad_count is None:
                self.ok = False
                return False
            self.i2c.readfrom_mem_into(_ADDR, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                                       self._buf)  # first 6 bytes are the map
            ref_map = bytearray(self._buf[0:6])

            self._w8(0xFF, 0x01)
            self._w8(_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00)
            self._w8(_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C)
            self._w8(0xFF, 0x00)
            self._w8(_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)

            first_spad = 12 if is_aperture else 0
            enabled = 0
            for i in range(48):
                byte_i, bit_i = i // 8, i % 8
                if i < first_spad or enabled == spad_count:
                    ref_map[byte_i] &= ~(1 << bit_i)
                elif (ref_map[byte_i] >> bit_i) & 0x01:
                    enabled += 1
            self.i2c.writeto_mem(_ADDR, _GLOBAL_CONFIG_SPAD_ENABLES_REF_0,
                                 bytes(ref_map))

            # --- load reference tuning ---
            for reg, val in _TUNING:
                self._w8(reg, val)

            # --- interrupt config: new-sample-ready, active low ---
            self._w8(_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04)
            self._w8(0x84, self._r8(0x84) & ~0x10)
            self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)

            # --- timing budget ---
            self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)
            self.set_timing_budget(timing_budget_us)

            # --- reference calibration (VHV then phase) ---
            self._w8(_SYSTEM_SEQUENCE_CONFIG, 0x01)
            if not self._perform_ref_calibration(0x40):
                self.ok = False
                return False
            self._w8(_SYSTEM_SEQUENCE_CONFIG, 0x02)
            if not self._perform_ref_calibration(0x00):
                self.ok = False
                return False
            self._w8(_SYSTEM_SEQUENCE_CONFIG, 0xE8)

            self.start_continuous()
            self.ok = True
            self.degraded = False
            self.invalid_streak = 0
            return True

        except OSError:
            self.error_count += 1
            self.ok = False
            return False

    def _get_spad_info(self):
        try:
            self._w8(0x80, 0x01)
            self._w8(0xFF, 0x01)
            self._w8(0x00, 0x00)
            self._w8(0xFF, 0x06)
            self._w8(0x83, self._r8(0x83) | 0x04)
            self._w8(0xFF, 0x07)
            self._w8(0x81, 0x01)
            self._w8(0x80, 0x01)
            self._w8(0x94, 0x6B)
            self._w8(0x83, 0x00)

            # Bounded wait. An unbounded `while` here is how a single flaky
            # sensor hangs the whole boot -- and a car that never finishes
            # booting cannot even report which sensor failed.
            deadline = time.ticks_add(time.ticks_ms(), 100)
            while self._r8(0x83) == 0x00:
                if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                    return None, None
            self._w8(0x83, 0x01)
            tmp = self._r8(0x92)
            count = tmp & 0x1F
            is_aperture = bool((tmp >> 7) & 0x01)

            self._w8(0x81, 0x00)
            self._w8(0xFF, 0x06)
            self._w8(0x83, self._r8(0x83) & ~0x04)
            self._w8(0xFF, 0x01)
            self._w8(0x00, 0x01)
            self._w8(0xFF, 0x00)
            self._w8(0x80, 0x00)
            return count, is_aperture
        except OSError:
            return None, None

    def _perform_ref_calibration(self, vhv_init_byte):
        self._w8(_SYSRANGE_START, 0x01 | vhv_init_byte)
        deadline = time.ticks_add(time.ticks_ms(), 100)
        while (self._r8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
            if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                return False
        self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        self._w8(_SYSRANGE_START, 0x00)
        return True

    # -- timing budget -----------------------------------------------------
    # Longer budget = more photons integrated = more range on black walls,
    # at the cost of update rate. 33 ms is our operating point: it holds
    # ~30 Hz per sensor, which is faster than our schedule polls any single
    # ToF anyway, so the budget is free in wall-clock terms.

    def set_timing_budget(self, budget_us):
        if budget_us < _MIN_TIMING_BUDGET_US:
            budget_us = _MIN_TIMING_BUDGET_US
        try:
            enables = self._get_sequence_step_enables()
            timeouts = self._get_sequence_step_timeouts(enables)

            used = _START_OVERHEAD + _END_OVERHEAD
            if enables["tcc"]:
                used += timeouts["msrc_dss_tcc_us"] + _TCC_OVERHEAD
            if enables["dss"]:
                used += 2 * (timeouts["msrc_dss_tcc_us"] + _DSS_OVERHEAD)
            elif enables["msrc"]:
                used += timeouts["msrc_dss_tcc_us"] + _MSRC_OVERHEAD
            if enables["pre_range"]:
                used += timeouts["pre_range_us"] + _PRE_RANGE_OVERHEAD
            if not enables["final_range"]:
                return True

            used += _FINAL_RANGE_OVERHEAD
            if used > budget_us:
                return False          # requested budget is unachievable

            final_us = budget_us - used
            final_mclks = _timeout_us_to_mclks(
                final_us, timeouts["final_range_vcsel_period_pclks"])
            if enables["pre_range"]:
                final_mclks += timeouts["pre_range_mclks"]
            self._w16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                      _encode_timeout(final_mclks))
            return True
        except OSError:
            self.error_count += 1
            return False

    def _get_sequence_step_enables(self):
        s = self._r8(_SYSTEM_SEQUENCE_CONFIG)
        return {
            "tcc": bool((s >> 4) & 0x1),
            "dss": bool((s >> 3) & 0x1),
            "msrc": bool((s >> 2) & 0x1),
            "pre_range": bool((s >> 6) & 0x1),
            "final_range": bool((s >> 7) & 0x1),
        }

    def _get_sequence_step_timeouts(self, enables):
        pre_vcsel = ((self._r8(_PRE_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1)
        msrc_mclks = self._r8(_MSRC_CONFIG_TIMEOUT_MACROP) + 1
        pre_mclks = _decode_timeout(
            self._r16(_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        final_vcsel = ((self._r8(_FINAL_RANGE_CONFIG_VCSEL_PERIOD) + 1) << 1)
        final_mclks = _decode_timeout(
            self._r16(_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI))
        if enables["pre_range"]:
            final_mclks -= pre_mclks
        return {
            "msrc_dss_tcc_us": _timeout_mclks_to_us(msrc_mclks, pre_vcsel),
            "pre_range_us": _timeout_mclks_to_us(pre_mclks, pre_vcsel),
            "pre_range_mclks": pre_mclks,
            "final_range_vcsel_period_pclks": final_vcsel,
            "final_range_us": _timeout_mclks_to_us(final_mclks, final_vcsel),
        }

    # -- ranging -----------------------------------------------------------

    def start_continuous(self, period_ms=0):
        """period_ms = 0 means back-to-back (as fast as the timing budget
        allows). We use back-to-back so that whenever our schedule gets round
        to this sensor, the freshest possible sample is already waiting."""
        self._w8(0x80, 0x01)
        self._w8(0xFF, 0x01)
        self._w8(0x00, 0x00)
        self._w8(0x91, self._stop_variable)
        self._w8(0x00, 0x01)
        self._w8(0xFF, 0x00)
        self._w8(0x80, 0x00)
        if period_ms:
            self._w16(_SYSTEM_INTERMEASUREMENT_PERIOD, period_ms)
            self._w8(_SYSRANGE_START, 0x04)
        else:
            self._w8(_SYSRANGE_START, 0x02)

    def stop_continuous(self):
        try:
            self._w8(_SYSRANGE_START, 0x01)
            self._w8(0xFF, 0x01)
            self._w8(0x00, 0x00)
            self._w8(0x91, 0x00)
            self._w8(0x00, 0x01)
            self._w8(0xFF, 0x00)
        except OSError:
            pass

    def read_mm(self, now_ms):
        """Non-blocking. Returns distance in mm, or None if there is no fresh
        valid sample. NEVER returns 0 for 'no data' -- 0 mm is also the most
        dangerous possible reading, and a caller that forgets to check would
        e-stop or, worse, believe it.

        Updates the invalid streak used by the recovery ladder.
        """
        if self.degraded:
            return None
        try:
            if (self._r8(_RESULT_INTERRUPT_STATUS) & 0x07) == 0:
                return None                     # not ready; not an error
            self.i2c.readfrom_mem_into(_ADDR, _RESULT_RANGE_STATUS, self._buf)
            self._w8(_SYSTEM_INTERRUPT_CLEAR, 0x01)
        except OSError:
            self.error_count += 1
            self.invalid_streak += 1
            self.ok = False
            return None

        self.read_count += 1
        self.range_status = (self._buf[0] & 0x78) >> 3
        mm = (self._buf[10] << 8) | self._buf[11]

        # Status 11 is the ONLY valid code. 8190/8191 are the sensor's
        # explicit "nothing came back" values and appear constantly against
        # the black walls -- that is the sensor working correctly and telling
        # us the truth, so we record it as missing data, not as a fault.
        if self.range_status != _RANGE_VALID or mm >= 8000 or mm == 0:
            self.invalid_streak += 1
            self.last_mm = RANGE_INVALID
            return None

        self.invalid_streak = 0
        self.ok = True
        self.last_mm = mm
        self.last_valid_mm = mm
        self.last_valid_ms = now_ms
        return mm

    # -- recovery ----------------------------------------------------------

    def attempt_recovery(self, max_attempts=3):
        """Called by the scheduler after too many consecutive invalid reads.
        The mux channel is already selected by the caller.

        There is no XSHUT to toggle, so this is a pure software re-init: stop
        continuous mode, run the full init sequence again. It genuinely works
        for the common failure (a sensor stuck mid-conversion); it cannot fix
        a sensor whose I2C state machine has hung holding SDA -- for that the
        mux deselect in TCA9548A.recover_bus() at least saves the rest of the
        bus.

        Returns True if recovered. On final failure the sensor is marked
        degraded and we never touch it again this round; the Pi is told, and
        the mission plans without it.
        """
        if self.degraded:
            return False
        if self.recovery_attempts >= max_attempts:
            self.degraded = True
            self.ok = False
            return False
        self.recovery_attempts += 1
        self.stop_continuous()
        if self.init():
            return True
        if self.recovery_attempts >= max_attempts:
            self.degraded = True
            self.ok = False
        return False


# ---------------------------------------------------------------------------
# Timeout encode/decode helpers (ST API arithmetic, kept module-level so they
# can be unit-tested off-target)
# ---------------------------------------------------------------------------

def _decode_timeout(reg_val):
    return ((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1


def _encode_timeout(mclks):
    if mclks <= 0:
        return 0
    ls_byte = mclks - 1
    ms_byte = 0
    while ls_byte > 0xFF:
        ls_byte >>= 1
        ms_byte += 1
    return ((ms_byte << 8) | (ls_byte & 0xFF))


def _calc_macro_period_ns(vcsel_period_pclks):
    return ((2304 * vcsel_period_pclks * 1655) + 500) // 1000


def _timeout_mclks_to_us(mclks, vcsel_period_pclks):
    macro_ns = _calc_macro_period_ns(vcsel_period_pclks)
    return ((mclks * macro_ns) + (macro_ns // 2)) // 1000


def _timeout_us_to_mclks(us, vcsel_period_pclks):
    macro_ns = _calc_macro_period_ns(vcsel_period_pclks)
    return ((us * 1000) + (macro_ns // 2)) // macro_ns
