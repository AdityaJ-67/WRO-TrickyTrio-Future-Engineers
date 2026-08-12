"""
TCS34725 RGB colour sensor -- mux channel 1, address 0x29.
Downward-facing, fixed height above the mat. Lap counting + direction latch.

Note the address: 0x29, the SAME address as all five VL53L0X sensors. That
is the second reason the TCA9548A is on this board. It has its own channel
and never coexists on the bus with a ToF.

WHY RATIOS, NEVER RAW RGB
-------------------------
This is the sensor most likely to lose you the round, and always for the
same reason: raw thresholds calibrated in the pit under fluorescent light
meet stage spotlights at the venue, every channel doubles, and the
classifier reports "orange" for the white mat.

So we never threshold on R, G or B. We threshold on R/C and B/C -- each
channel normalised by the CLEAR channel. Illumination scales all four
channels together, so it cancels in the ratio. What survives is the surface
reflectance, which is what the rules actually specify.

The white mat is the built-in reference: on white, R/C and B/C sit near
their neutral values, and a coloured line is a *departure* from that. We
learn the white baseline during calibration (tools/calibrate.py colour) and
store it in config, so classification is "how far from white, and in which
direction", not "is this number big".

INTEGRATION TIME
----------------
24 ms (ATIME 0xF6). The line is 20 mm wide; at our 700 mm/s cruise the
sensor is over it for ~28 ms. A longer integration would smear the line
into the mat and we would miss crossings; a much shorter one starves the
ADC and the ratios get noisy. 24 ms with a 50 Hz sampling slot gives us
1-2 clean samples per crossing, which is why colour gets a high-priority
slot in the Pico's sensor schedule despite being "just" a lap counter.

Gain 4x: enough headroom on a white mat under stage lighting without
clipping the clear channel. Clipping is fatal here -- a saturated C makes
every ratio wrong in the same direction.

THE LED
-------
The breakout's illumination LED is driven from GP18 rather than left tied
on. Deterministic, known illumination is the entire premise of the ratio
method; ambient-only would make the readings depend on where the judges are
standing. It is also switchable, which lets the calibration tool measure
the ambient contribution and subtract it.
"""

import time

_ADDR = 0x29

_CMD_BIT = 0x80
_AUTO_INC = 0x20                 # auto-increment for multi-byte reads

_REG_ENABLE = 0x00
_REG_ATIME = 0x01
_REG_CONTROL = 0x0F
_REG_ID = 0x12
_REG_STATUS = 0x13
_REG_CDATAL = 0x14               # C, R, G, B -- 8 bytes, auto-increment

_ENABLE_PON = 0x01
_ENABLE_AEN = 0x02

_STATUS_AVALID = 0x01

# Valid device IDs. 0x44 = TCS34725, 0x4D = TCS34727. CJMCU boards ship
# either; both behave identically for our purposes. The self-test accepts
# both and rejects anything else, because "something ACKed at 0x29" is not
# the same as "the colour sensor is present" -- on this bus it could be a
# mis-selected ToF.
_VALID_IDS = (0x44, 0x4D)

GAIN_1X, GAIN_4X, GAIN_16X, GAIN_60X = 0x00, 0x01, 0x02, 0x03


class TCS34725(object):

    def __init__(self, i2c, led_pin=None, atime=0xF6, gain=GAIN_4X):
        self.i2c = i2c
        self.atime = atime
        self.gain = gain
        self.ok = False
        self.device_id = None
        self._led = led_pin          # machine.Pin or None
        self._buf8 = bytearray(8)    # pre-allocated; no allocation in-loop
        self._buf1 = bytearray(1)

    # -- bus helpers -------------------------------------------------------

    def _w8(self, reg, val):
        self.i2c.writeto_mem(_ADDR, _CMD_BIT | reg, bytes([val]))

    def _r8(self, reg):
        self.i2c.readfrom_mem_into(_ADDR, _CMD_BIT | reg, self._buf1)
        return self._buf1[0]

    # -- lifecycle ---------------------------------------------------------

    def init(self):
        """Boot-time only. Blocking sleeps here are fine and deliberate --
        this runs before the control loop exists."""
        try:
            self.device_id = self._r8(_REG_ID)
            if self.device_id not in _VALID_IDS:
                self.ok = False
                return False
            self._w8(_REG_ENABLE, _ENABLE_PON)
            time.sleep_ms(3)                    # datasheet: 2.4 ms warm-up
            self._w8(_REG_ATIME, self.atime)
            self._w8(_REG_CONTROL, self.gain)
            self._w8(_REG_ENABLE, _ENABLE_PON | _ENABLE_AEN)
            time.sleep_ms(30)                   # one full integration cycle
            self.led(True)
            self.ok = True
            return True
        except OSError:
            self.ok = False
            return False

    def led(self, on):
        if self._led is not None:
            self._led.value(1 if on else 0)

    # -- reading -----------------------------------------------------------

    def read_raw(self):
        """Returns (c, r, g, b) or None. Non-blocking: if the ADC has not
        finished integrating we return None and the caller keeps its previous
        value. We never spin waiting for AVALID -- that is a 24 ms stall in a
        10 ms control loop."""
        try:
            if not (self._r8(_REG_STATUS) & _STATUS_AVALID):
                return None
            self.i2c.readfrom_mem_into(_ADDR, _CMD_BIT | _AUTO_INC | _REG_CDATAL,
                                       self._buf8)
        except OSError:
            self.ok = False
            return None
        b = self._buf8
        return (b[0] | (b[1] << 8), b[2] | (b[3] << 8),
                b[4] | (b[5] << 8), b[6] | (b[7] << 8))


class LineClassifier(object):
    """Turns (c, r, g, b) into orange / blue / none.

    Lives next to the driver rather than on the Pi because the DECISION is
    cheap and the DATA is high-rate: shipping raw RGB at 50 Hz and
    classifying on the Pi would work, but it puts a lap-critical edge
    detection behind a UART and a Linux scheduler. The Pico classifies; the
    Pi decides what a crossing MEANS (which lap, which direction). That
    split matches the division of labour: micro-decision on the Pico, macro
    on the Pi.

    All thresholds arrive from config.json via MSG_CONFIG-adjacent plumbing
    in main.py -- nothing here is tuned in source.
    """

    # Matches protocol.COLOUR_* -- duplicated as plain ints so this driver
    # has no import dependency on the protocol module.
    NONE, ORANGE, BLUE, OTHER = 0, 1, 2, 3

    def __init__(self, clear_min=400, orange_rc_min=0.42, orange_bc_max=0.24,
                 blue_bc_min=0.36, blue_rc_max=0.26):
        self.clear_min = clear_min
        self.orange_rc_min = orange_rc_min
        self.orange_bc_max = orange_bc_max
        self.blue_bc_min = blue_bc_min
        self.blue_rc_max = blue_rc_max
        self.last_rc = 0.0
        self.last_bc = 0.0

    def configure(self, **kw):
        for k, v in kw.items():
            if hasattr(self, k) and v is not None:
                setattr(self, k, v)

    def classify(self, sample):
        if sample is None:
            return self.NONE
        c, r, g, b = sample
        # A dark or lifted sensor produces tiny, noisy values whose ratios are
        # meaningless. Refusing to classify is strictly better than emitting
        # a confident wrong answer into the lap counter.
        if c < self.clear_min:
            return self.NONE
        rc = r / c
        bc = b / c
        self.last_rc = rc
        self.last_bc = bc
        # Orange and blue are tested against BOTH a positive and a negative
        # criterion. A single-sided test ("is red high") fires on the orange
        # line and on the red pillar's reflection on the mat; requiring blue
        # to also be LOW rejects that.
        if rc >= self.orange_rc_min and bc <= self.orange_bc_max:
            return self.ORANGE
        if bc >= self.blue_bc_min and rc <= self.blue_rc_max:
            return self.BLUE
        return self.NONE
