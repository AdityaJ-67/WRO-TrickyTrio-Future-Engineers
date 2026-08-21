"""TCS34725 colour sensor on channel 1 of the TCA9548A.  MicroPython.

Reads raw RGBC and classifies it. Reads and reports only - no navigation,
no decision making.

On the Pico:  mpremote run colour.py   (needs tcs34725.py on the board)
On a laptop:  python3 -c "import colour; colour.selftest()"
"""

# --- 1. Imports -------------------------------------------------------------
try:
    from machine import I2C, Pin        # type: ignore
    from time import sleep_ms           # type: ignore
except ImportError:                     # laptop: classification is still testable
    I2C = Pin = None

    def sleep_ms(ms):
        pass

# Separate block on purpose - see the note in distance.py.
try:
    from tcs34725 import TCS34725       # type: ignore  - third-party driver
except ImportError:
    TCS34725 = None

# --- 2. Configuration -------------------------------------------------------
I2C_ID = 0                  # GP4/GP5 are I2C0
SDA_PIN = 4
SCL_PIN = 5
I2C_FREQ = 400_000

TCA_ADDRESS = 0x70
COLOUR_CHANNEL = 1          # channel 2 is the IMU, 0/3/4/5 are the VL53L0X sensors

INTEGRATION_TIME_MS = 50    # longer = more light gathered = better in the dark,
GAIN = 4                    # but both push the clear channel towards saturation

READ_INTERVAL_MS = 200
RETRY_MS = 500

# --- Colour thresholds - calibrate these on the real mat --------------------
# Red, green and blue are compared as a FRACTION of clear, so the classification
# survives the sensor sitting closer to or further from the surface.
BLACK_MAX_CLEAR = 300       # below this there is barely any light coming back
WHITE_MIN_CLEAR = 2000      # above this, and balanced, means a bright surface
WHITE_MAX_SPREAD = 0.10     # how equal r/g/b must be to count as white
RED_MIN_RATIO = 0.40        # red must be at least this share of clear
GREEN_MIN_RATIO = 0.40
DOMINANCE = 1.4             # winning channel must beat the others by this factor

# --- 3. I2C initialization --------------------------------------------------
i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ) if I2C else None


# --- 4. Multiplexer ---------------------------------------------------------
def select_channel(channel):
    """Point the TCA9548A at one channel - one bit per channel, one at a time."""
    i2c.writeto(TCA_ADDRESS, bytes([1 << channel]))


# --- 5. Sensor initialization -----------------------------------------------
def setup_sensor():
    """Bring the TCS34725 up. Returns the sensor, or None if it is not there."""
    if TCS34725 is None:
        print("TCS34725 driver missing - copy tcs34725.py to the board")
        return None
    try:
        select_channel(COLOUR_CHANNEL)
        sensor = TCS34725(i2c)
        # Driver forks name these differently, so only apply what exists.
        if hasattr(sensor, "integration_time"):
            sensor.integration_time(INTEGRATION_TIME_MS)
        if hasattr(sensor, "gain"):
            sensor.gain(GAIN)
        print("TCS34725 ready on channel", COLOUR_CHANNEL)
        return sensor
    except Exception as error:
        print("TCS34725 init failed -", error)
        return None


# --- 6. Reading and classification ------------------------------------------
def read_rgbc(sensor):
    """(red, green, blue, clear) raw counts, or None if the read failed."""
    try:
        select_channel(COLOUR_CHANNEL)
        return sensor.read(raw=True)    # driver variants: .read(raw=True) / .color_raw
    except Exception:
        return None


def classify(red, green, blue, clear):
    """Raw counts -> "RED" / "GREEN" / "BLACK" / "WHITE" / "UNKNOWN".

    Darkness is judged on clear alone. Everything else is judged on each
    channel's share of clear, which is what makes the result independent of how
    bright the surface happens to be lit.
    """
    if clear < BLACK_MAX_CLEAR:
        return "BLACK"                  # also covers clear == 0, so no divide by zero

    red_ratio = red / clear
    green_ratio = green / clear
    blue_ratio = blue / clear

    if (clear >= WHITE_MIN_CLEAR
            and max(red_ratio, green_ratio, blue_ratio)
            - min(red_ratio, green_ratio, blue_ratio) < WHITE_MAX_SPREAD):
        return "WHITE"                  # bright and roughly equal in all three

    if (red_ratio >= RED_MIN_RATIO
            and red_ratio > green_ratio * DOMINANCE
            and red_ratio > blue_ratio * DOMINANCE):
        return "RED"

    if (green_ratio >= GREEN_MIN_RATIO
            and green_ratio > red_ratio * DOMINANCE
            and green_ratio > blue_ratio * DOMINANCE):
        return "GREEN"

    return "UNKNOWN"


# --- 7. Main loop -----------------------------------------------------------
def main():
    sensor = None

    while True:
        # Re-initialise whenever we have no working sensor, so a knocked-out
        # cable recovers on its own rather than ending the run.
        if sensor is None:
            sensor = setup_sensor()
            if sensor is None:
                print("COLOUR SENSOR ERROR")
                sleep_ms(RETRY_MS)
                continue

        reading = read_rgbc(sensor)
        if reading is None:
            print("COLOUR SENSOR ERROR")
            sensor = None               # force a fresh init next time round
            sleep_ms(RETRY_MS)
            continue

        red, green, blue, clear = reading
        print("R: %d" % red)
        print("G: %d" % green)
        print("B: %d" % blue)
        print("Clear: %d" % clear)
        print("Colour: %s" % classify(red, green, blue, clear))
        print("-" * 20)

        sleep_ms(READ_INTERVAL_MS)


def selftest():
    global i2c

    class _FakeI2C:             # test double, not part of the module
        def __init__(self):
            self.written = []

        def writeto(self, address, data):
            self.written.append((address, data))

    class _FakeSensor:
        def __init__(self, reading):
            self.reading = reading

        def read(self, raw=False):
            if self.reading is None:
                raise OSError("sensor gone")
            return self.reading

    i2c = _FakeI2C()

    # the colour sensor lives on channel 1 alone
    select_channel(COLOUR_CHANNEL)
    assert i2c.written == [(TCA_ADDRESS, b"\x02")], i2c.written

    # black mat, and the darkness test must come before any division
    assert classify(10, 10, 10, 25) == "BLACK"
    assert classify(0, 0, 0, 0) == "BLACK"

    # bright and balanced
    assert classify(900, 900, 900, 3000) == "WHITE"

    # strongly coloured surfaces
    assert classify(1500, 400, 300, 2400) == "RED"
    assert classify(300, 1200, 400, 2100) == "GREEN"

    # a blue surface is neither red nor green, and must not be guessed at
    assert classify(300, 400, 900, 1500) == "UNKNOWN"

    # same surface under half the light classifies the same way - that is the
    # whole point of working in ratios rather than raw counts
    assert classify(750, 200, 150, 1200) == "RED"

    # a dead sensor reports None instead of taking the loop down
    assert read_rgbc(_FakeSensor(None)) is None
    assert read_rgbc(_FakeSensor((1500, 400, 300, 2400))) == (1500, 400, 300, 2400)

    print("selftest ok  red sample ->", classify(1500, 400, 300, 2400),
          " same at half brightness ->", classify(750, 200, 150, 1200))


if __name__ == "__main__":
    main()
