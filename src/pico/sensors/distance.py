r"""Four VL53L0X distance sensors behind one TCA9548A multiplexer.  MicroPython.

Layout, looking down on the robot:

            left \  | front |  / right
             -45deg |  0deg |  +45deg
                 [ camera ]
                 [  robot ]
                    rear
                   180deg

The left and right sensors sit either side of the camera at 45 degrees, so they
see the forward diagonals - they are NOT side-facing. A reading of D on one of
them means roughly D x 0.71 of clearance both ahead and to that side.

Reads and reports only - no navigation, no avoidance, no motor control.

On the Pico:  mpremote run distance.py   (needs vl53l0x.py on the board)
On a laptop:  python3 -c "import distance; distance.selftest()"
"""

# --- 1. Imports -------------------------------------------------------------
try:
    from machine import I2C, Pin        # type: ignore
    from time import sleep_ms           # type: ignore
except ImportError:                     # laptop: the mux logic is still testable
    I2C = Pin = None

    def sleep_ms(ms):
        pass

# Separate block on purpose. A missing driver file must cost us this sensor and
# nothing else - sharing the try above would take I2C down with it, and every
# other sensor on the bus with that.
try:
    from vl53l0x import VL53L0X         # type: ignore  - third-party driver
except ImportError:
    VL53L0X = None

# --- 2. Configuration -------------------------------------------------------
I2C_ID = 0                  # GP4/GP5 are I2C0
SDA_PIN = 4
SCL_PIN = 5
I2C_FREQ = 400_000

TCA_ADDRESS = 0x70

# (name, mux channel, angle in degrees: negative = left of straight ahead)
# A tuple, not a dict: MicroPython does not guarantee dict ordering, and the
# printout should always read front, left, right, rear in that order.
# Channels 1 and 2 belong to the colour sensor and the IMU - left alone.
SENSORS = (
    ("front", 0, 0),
    ("left", 3, -45),
    ("right", 4, 45),
    ("rear", 5, 180),
)

READ_INTERVAL_MS = 100

# --- 3. I2C initialization --------------------------------------------------
i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ) if I2C else None


# --- 4. Multiplexer ---------------------------------------------------------
def select_channel(channel):
    """Point the TCA9548A at one channel.

    The control register is a single byte where each bit is one channel, so
    channel 3 means bit 3, i.e. 0b00001000. Exactly one bit at a time keeps one
    sensor on the bus, which is what lets four chips that all answer to 0x29
    share it.
    """
    i2c.writeto(TCA_ADDRESS, bytes([1 << channel]))


# --- 5. Sensor initialization -----------------------------------------------
def setup_sensors():
    """Bring up every sensor. A failure here is recorded, not raised."""
    if VL53L0X is None:
        print("VL53L0X driver missing - copy vl53l0x.py to the board")
        return {name: None for name, _, _ in SENSORS}

    sensors = {}
    for name, channel, angle in SENSORS:
        try:
            select_channel(channel)
            sensor = VL53L0X(i2c)
            if hasattr(sensor, "start"):    # some driver versions need this
                sensor.start()
            sensors[name] = sensor
            print("%-5s ready  channel %d  facing %+d deg" % (name, channel, angle))
        except Exception as error:
            sensors[name] = None
            print("%-5s FAILED channel %d  -" % (name, channel), error)
    return sensors


# --- 6. Distance reading ----------------------------------------------------
def read_distance(sensor, channel):
    """Distance in mm, or None if this sensor cannot be read right now.

    Everything is caught: a sensor that browns out, is unplugged mid-run, or
    holds the bus must not take the other three down with it.
    """
    if sensor is None:
        return None
    try:
        select_channel(channel)
        return sensor.read()            # driver variants: .read() / .ping() / .range
    except Exception:
        return None


def read_all(sensors):
    """One sweep of all four, keyed by position name. None means that one failed."""
    return {name: read_distance(sensors[name], channel)
            for name, channel, _ in SENSORS}


def as_mm(distance):
    return "ERROR" if distance is None else "%d mm" % distance


# --- 7. Main loop -----------------------------------------------------------
def main():
    sensors = setup_sensors()
    print("-" * 26)

    while True:
        readings = read_all(sensors)

        # Named by position, so whatever consumes these later reads plainly.
        front_distance = readings["front"]
        left_distance = readings["left"]
        right_distance = readings["right"]
        rear_distance = readings["rear"]

        print("Front : %s" % as_mm(front_distance))
        print("Left  : %s" % as_mm(left_distance))
        print("Right : %s" % as_mm(right_distance))
        print("Rear  : %s" % as_mm(rear_distance))
        print("-" * 26)

        sleep_ms(READ_INTERVAL_MS)


def selftest():
    global i2c

    class _FakeI2C:             # test double, not part of the module
        def __init__(self):
            self.written = []

        def writeto(self, address, data):
            self.written.append((address, data))

    class _FakeSensor:
        def __init__(self, value):
            self.value = value

        def read(self):
            if self.value is None:
                raise OSError("sensor gone")
            return self.value

    i2c = _FakeI2C()

    # one bit per channel, and only ever one bit at a time
    for name, channel, angle in SENSORS:
        select_channel(channel)
    masks = [data[0] for _, data in i2c.written]
    assert masks == [0x01, 0x08, 0x10, 0x20], masks
    assert all(address == TCA_ADDRESS for address, _ in i2c.written)
    assert all(bin(mask).count("1") == 1 for mask in masks)

    # channels 1 and 2 are never touched - they belong to the colour sensor and IMU
    assert not any(mask & 0b0000_0110 for mask in masks)

    # the four positions are the ones the main loop unpacks, and the diagonals
    # are mirrored either side of straight ahead
    angles = {name: angle for name, _, angle in SENSORS}
    assert set(angles) == {"front", "left", "right", "rear"}, angles
    assert angles["front"] == 0 and angles["left"] == -angles["right"]

    # a good sensor reads, a dead one returns None instead of raising
    assert read_distance(_FakeSensor(421), 0) == 421
    assert read_distance(_FakeSensor(None), 0) is None
    assert read_distance(None, 0) is None

    # one dead sensor must not lose the other three
    sensors = {"front": _FakeSensor(421), "left": _FakeSensor(None),
               "right": _FakeSensor(205), "rear": None}
    readings = read_all(sensors)
    assert readings == {"front": 421, "left": None, "right": 205, "rear": None}, readings
    assert as_mm(readings["front"]) == "421 mm"
    assert as_mm(readings["left"]) == "ERROR"

    print("selftest ok  masks:", [hex(m) for m in masks], " angles:", angles)


if __name__ == "__main__":
    main()
