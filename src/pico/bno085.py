"""
BNO085 9-DoF IMU (SH-2 over I2C) -- mux channel 2, address 0x4A.
Supplies the fused absolute heading that closes every 90 degree turn.

WHY THE FUSED ROTATION VECTOR AND NOT INTEGRATED GYRO
-----------------------------------------------------
Integrating a raw gyro over a 180 s round accumulates bias into tens of
degrees of heading error. Our entire cornering strategy is "turn until the
IMU says we have turned 90 degrees" -- with drifting heading, lap 3's
corners are systematically wrong and the car ends up diagonal in a 1000 mm
lane. The BNO085's on-chip fusion runs accel + gyro + mag and returns an
absolute rotation vector whose yaw does not walk. We use ONLY that.

Yaw RATE is obtained by differentiating the fused yaw rather than by
enabling a second gyro report. Two reasons: the rotation vector is already
fused and smooth, so its derivative is much cleaner than raw gyro; and a
second report stream would compete for I2C time in a schedule that is
already carrying seven devices.

STARTUP STABILITY
-----------------
The BNO085 needs a quiet start. Its fusion converges during the first
second or so, and if the car is being carried, jostled, or is vibrating
from a spinning motor, the converged frame is tilted. Two consequences,
both implemented:

  1. `init()` waits for the reset-complete handshake and then requires
     N consecutive rotation-vector reports with an accuracy status of at
     least MEDIUM before declaring the IMU healthy. The boot self-test
     fails if that does not happen -- better to refuse to arm on the bench
     than to corner on a bad frame.

  2. The heading REFERENCE is zeroed at the moment the start button is
     pressed (CMDF_ZERO_HEADING), not at boot. Between boot and the start
     button the car may be picked up and re-placed by a judge; whatever
     absolute frame the chip settled on at boot is meaningless by then.
     We zero in SOFTWARE (a stored offset) rather than using the chip's
     tare command, so it is instant, reversible, and visible in the log.

SH-2 / SHTP IN ONE PARAGRAPH
----------------------------
Every transfer is prefixed by a 4-byte SHTP header: length LSB, length MSB
(bit 15 = continuation, not part of the length), channel, sequence. Length
INCLUDES the 4 header bytes. Reads are two transactions: read the header to
learn the length, then read the whole packet (the device re-sends the
header). Sensor reports arrive on channel 3, prefixed by a 5-byte timebase
report (0xFB). Feature enables are written to channel 2.
"""

import time
import math

_ADDR_DEFAULT = 0x4A            # 0x4B if the address pin is strapped high

_CH_COMMAND = 0
_CH_EXECUTABLE = 1
_CH_CONTROL = 2
_CH_REPORTS = 3

_REPORT_SET_FEATURE = 0xFD
_REPORT_PRODUCT_ID_REQ = 0xF9
_REPORT_PRODUCT_ID_RESP = 0xF8
_REPORT_BASE_TIMESTAMP = 0xFB

_SENSOR_ROTATION_VECTOR = 0x05

_Q14 = 1.0 / (1 << 14)          # rotation vector components are Q14

# Accuracy status field in the rotation-vector report, bits 1:0.
ACC_UNRELIABLE, ACC_LOW, ACC_MEDIUM, ACC_HIGH = 0, 1, 2, 3


class BNO085(object):

    def __init__(self, i2c, address=_ADDR_DEFAULT):
        self.i2c = i2c
        self.address = address
        self.ok = False

        self.yaw_deg = 0.0          # raw fused yaw, -180..180
        self.yaw_rate_dps = 0.0
        self.accuracy = ACC_UNRELIABLE
        self.last_report_ms = 0
        self.report_count = 0
        self.error_count = 0

        self._offset_deg = 0.0      # software zero, set at start-button press
        self._seq = [0] * 6         # outbound sequence number per channel
        # 384 bytes covers the ~272-byte boot advertisement in one read.
        # Sized once, at construction: no allocation in the control loop.
        self._buf = bytearray(384)
        self._hdr = bytearray(4)
        self._prev_yaw = None
        self._prev_ms = 0

    # -- SHTP transport ----------------------------------------------------

    def _send(self, channel, payload):
        n = len(payload) + 4
        pkt = bytearray(n)
        pkt[0] = n & 0xFF
        pkt[1] = (n >> 8) & 0xFF
        pkt[2] = channel
        pkt[3] = self._seq[channel]
        self._seq[channel] = (self._seq[channel] + 1) & 0xFF
        pkt[4:] = payload
        self.i2c.writeto(self.address, pkt)

    def _read_packet(self):
        """Returns (channel, memoryview_of_payload) or (None, None).

        Never blocks and never loops unboundedly: at most one header read and
        one body read per call. A wedged IMU costs us one tick, not the round.
        """
        try:
            self.i2c.readfrom_into(self.address, self._hdr)
        except OSError:
            self.error_count += 1
            return None, None

        length = (self._hdr[0] | (self._hdr[1] << 8)) & 0x7FFF
        if length == 0 or length == 0x7FFF:
            return None, None           # nothing pending
        if length <= 4:
            return None, None
        channel = self._hdr[2]

        if length > len(self._buf):
            # Oversized (only the boot advertisement gets close). Drain it in
            # buffer-sized bites so the device does not keep re-offering the
            # same packet forever, then report nothing.
            remaining = length
            while remaining > 0:
                take = min(remaining, len(self._buf))
                try:
                    self.i2c.readfrom_into(self.address,
                                           memoryview(self._buf)[0:take])
                except OSError:
                    self.error_count += 1
                    return None, None
                remaining -= take
            return None, None

        try:
            mv = memoryview(self._buf)[0:length]
            self.i2c.readfrom_into(self.address, mv)
        except OSError:
            self.error_count += 1
            return None, None
        return channel, memoryview(self._buf)[4:length]

    # -- lifecycle ---------------------------------------------------------

    def init(self, report_interval_us=10000, settle_reports=8,
             min_accuracy=ACC_MEDIUM, timeout_ms=3000):
        """Boot path only; blocking waits here are deliberate and bounded.

        `settle_reports` consecutive reports at >= `min_accuracy` before we
        call the IMU healthy -- this is the 'stable startup' the BNO085
        needs, made explicit and testable rather than left to a sleep().
        """
        self.ok = False
        deadline = time.ticks_add(time.ticks_ms(), timeout_ms)

        # Drain the boot advertisement and any unsolicited reset-complete.
        for _ in range(12):
            ch, _pl = self._read_packet()
            if ch is None:
                break
            time.sleep_ms(2)

        # Product ID request doubles as a liveness probe: if the chip answers
        # this, SHTP framing is working and the address is right.
        got_id = False
        try:
            self._send(_CH_CONTROL, bytes([_REPORT_PRODUCT_ID_REQ, 0]))
        except OSError:
            return False
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            ch, pl = self._read_packet()
            if ch == _CH_CONTROL and pl is not None and len(pl) > 0 and \
                    pl[0] == _REPORT_PRODUCT_ID_RESP:
                got_id = True
                break
            time.sleep_ms(2)
        if not got_id:
            return False

        if not self.enable_rotation_vector(report_interval_us):
            return False

        # Wait for the fusion to converge.
        good = 0
        while time.ticks_diff(deadline, time.ticks_ms()) > 0:
            if self.poll(time.ticks_ms()):
                good = good + 1 if self.accuracy >= min_accuracy else 0
                if good >= settle_reports:
                    self.ok = True
                    return True
            time.sleep_ms(5)
        return False

    def enable_rotation_vector(self, interval_us=10000):
        """Set Feature Command for the rotation vector.
        interval_us = 10000 -> 100 Hz, which is faster than we sample it. We
        over-provision on purpose: the report we read is then always fresh,
        so turn-completion decisions are never made on a stale angle."""
        payload = bytearray(17)
        payload[0] = _REPORT_SET_FEATURE
        payload[1] = _SENSOR_ROTATION_VECTOR
        payload[2] = 0                       # feature flags
        payload[3] = 0                       # change sensitivity LSB
        payload[4] = 0                       # change sensitivity MSB
        payload[5] = interval_us & 0xFF
        payload[6] = (interval_us >> 8) & 0xFF
        payload[7] = (interval_us >> 16) & 0xFF
        payload[8] = (interval_us >> 24) & 0xFF
        # bytes 9..12 batch interval = 0, 13..16 sensor-specific = 0
        try:
            self._send(_CH_CONTROL, payload)
            return True
        except OSError:
            self.error_count += 1
            return False

    # -- runtime -----------------------------------------------------------

    def poll(self, now_ms):
        """Read at most one packet. Returns True if a rotation vector was
        decoded. Call once per scheduled IMU slot."""
        ch, pl = self._read_packet()
        if ch != _CH_REPORTS or pl is None or len(pl) < 5:
            return False
        if pl[0] != _REPORT_BASE_TIMESTAMP:
            return False

        # Reports follow the 5-byte timebase header, packed back to back.
        off = 5
        got = False
        n = len(pl)
        while off + 4 <= n:
            rid = pl[off]
            if rid == _SENSOR_ROTATION_VECTOR:
                if off + 14 > n:
                    break
                status = pl[off + 2] & 0x03
                qi = _i16(pl, off + 4) * _Q14
                qj = _i16(pl, off + 6) * _Q14
                qk = _i16(pl, off + 8) * _Q14
                qr = _i16(pl, off + 10) * _Q14
                self.accuracy = status
                self._update_yaw(qi, qj, qk, qr, now_ms)
                got = True
                off += 14
            else:
                # Unknown report in the batch: we cannot know its length, so
                # stop parsing this packet rather than guessing an offset and
                # decoding garbage as a heading.
                break
        return got

    def _update_yaw(self, qi, qj, qk, qr, now_ms):
        # Standard quaternion -> yaw. Roll and pitch are not computed: the car
        # stays flat on a 3 m mat, and every degree of maths we do not do is a
        # degree that cannot be wrong.
        siny = 2.0 * (qr * qk + qi * qj)
        cosy = 1.0 - 2.0 * (qj * qj + qk * qk)
        yaw = math.degrees(math.atan2(siny, cosy))

        if self._prev_yaw is not None:
            dt = time.ticks_diff(now_ms, self._prev_ms)
            if 0 < dt < 200:
                d = _wrap180(yaw - self._prev_yaw)
                inst = d * 1000.0 / dt
                # Light low-pass. The fused yaw is smooth, but differentiating
                # anything at 100 Hz amplifies its last digit; the turn
                # controller's D term is the consumer and it hates noise.
                self.yaw_rate_dps += 0.35 * (inst - self.yaw_rate_dps)
        self._prev_yaw = yaw
        self._prev_ms = now_ms
        self.yaw_deg = yaw
        self.last_report_ms = now_ms
        self.report_count += 1

    # -- heading reference -------------------------------------------------

    def zero_heading(self):
        """Latch the current fused yaw as zero. Called EXACTLY once per round,
        at the start-button press."""
        self._offset_deg = self.yaw_deg

    @property
    def heading_deg(self):
        """Yaw relative to the start-button reference, wrapped to +/-180.
        POSITIVE = counter-clockwise (right-hand rule about the vertical
        axis). Every turn target in the mission logic is expressed in this
        frame, so this sign convention is load-bearing -- verify it on the
        bench by rotating the car left and watching the number rise."""
        return _wrap180(self.yaw_deg - self._offset_deg)

    def is_stale(self, now_ms, max_age_ms=200):
        return time.ticks_diff(now_ms, self.last_report_ms) > max_age_ms


def _i16(buf, off):
    v = buf[off] | (buf[off + 1] << 8)
    return v - 65536 if v & 0x8000 else v


def _wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a
