"""Sensor Manager: every sensor reading gathered into one dictionary.

Collects only. No navigation, no motor control, no decisions.

Everything here delegates to the existing modules - distance.py, imu.py,
colour.py and encoder.py own their own hardware, and this file only calls them.

On the Pico:  mpremote run sensorManager.py
On a laptop:  PYTHONPATH=.. python3 -c "import sensorManager; sensorManager.selftest()"
              (encoder.py lives one folder up; on the Pico everything sits at
               the root, so the plain imports below work as written)
"""

try:
    from machine import Pin             # type: ignore
    from time import sleep_ms           # type: ignore
except ImportError:                     # laptop: state assembly is still testable
    Pin = None

    def sleep_ms(ms):
        pass

import colour
import distance
import encoder
import imu

PRINT_INTERVAL_MS = 100

# Handles to the live sensors. setup() fills these in once; a value of None
# means that sensor is unavailable and its readings come back as None.
tof_sensors = {}
imu_sensor = None
colour_sensor = None

# Encoder bookkeeping. The pulse counter itself lives in encoder.py so there is
# only ever one of it.
_last_count = 0
_last_ms = 0


def setup():
    """Initialise every sensor exactly once. Safe to call on a partly built robot."""
    global tof_sensors, imu_sensor, colour_sensor, _last_count, _last_ms

    # One bus, one object. Each module builds its own I2C at import time, which
    # is right when you run it standalone on the bench, but here all three share
    # the same physical pins - so hand them a single bus to talk over.
    if distance.i2c is not None:
        imu.i2c = colour.i2c = distance.i2c

    tof_sensors = distance.setup_sensors()
    imu_sensor = imu.setup_imu()
    colour_sensor = colour.setup_sensor()

    # encoder.py attaches its interrupt inside its own main loop, which blocks,
    # so the handler is attached here instead. It still increments the counter
    # that lives in encoder.py, so there is one count and one set of maths.
    # ponytail: lift the IRQ setup into an encoder.setup() when that module is
    # next opened, and delete these six lines.
    if Pin:
        channel_a = Pin(encoder.ENCODER_A_PIN, Pin.IN, Pin.PULL_UP)
        channel_b = Pin(encoder.ENCODER_B_PIN, Pin.IN, Pin.PULL_UP)

        def on_pulse(pin):
            if channel_b.value():
                encoder.pulse_count -= encoder.DIRECTION_SIGN
            else:
                encoder.pulse_count += encoder.DIRECTION_SIGN

        channel_a.irq(trigger=Pin.IRQ_RISING, handler=on_pulse)

    _last_count = encoder.pulse_count
    _last_ms = encoder.ticks_ms()


def get_robot_state():
    """Every sensor value, right now, in one dictionary.

    Any sensor that cannot be read contributes None rather than raising, so one
    dead device never costs you the rest of the readings.
    """
    global _last_count, _last_ms

    # --- ToF ---
    ranges = distance.read_all(tof_sensors)

    # --- IMU ---
    orientation = imu.read_orientation(imu_sensor) if imu_sensor else None
    heading, pitch, roll = orientation if orientation else (None, None, None)

    # --- Colour ---
    reading = colour.read_rgbc(colour_sensor) if colour_sensor else None
    if reading:
        red, green, blue, clear = reading
        floor_colour = colour.classify(red, green, blue, clear)
        rgb = (red, green, blue)
    else:
        floor_colour, rgb = None, None

    # --- Encoder ---
    count = encoder.pulse_count                 # single read, so it cannot tear
    now_ms = encoder.ticks_ms()
    delta_ms = encoder.ticks_diff(now_ms, _last_ms)
    _, distance_travelled, speed = encoder.measure(
        count, count - _last_count, delta_ms)
    _last_count, _last_ms = count, now_ms

    return {
        "front_distance": ranges["front"],
        "left_distance": ranges["left"],
        "right_distance": ranges["right"],
        "rear_distance": ranges["rear"],

        "heading": heading,
        "pitch": pitch,
        "roll": roll,

        "distance_travelled": distance_travelled,
        "speed": speed,

        "floor_colour": floor_colour,
        "rgb": rgb,
    }


def main():
    setup()
    print("-" * 40)

    while True:
        state = get_robot_state()
        print("front %s  left %s  right %s  rear %s mm"
              % (state["front_distance"], state["left_distance"],
                 state["right_distance"], state["rear_distance"]))
        print("heading %s  pitch %s  roll %s"
              % (state["heading"], state["pitch"], state["roll"]))
        print("travelled %.1f cm  speed %.1f cm/s"
              % (state["distance_travelled"], state["speed"]))
        print("floor %s  rgb %s" % (state["floor_colour"], state["rgb"]))
        print("-" * 40)
        sleep_ms(PRINT_INTERVAL_MS)


EXPECTED_KEYS = ("front_distance", "left_distance", "right_distance",
                 "rear_distance", "heading", "pitch", "roll",
                 "distance_travelled", "speed", "floor_colour", "rgb")


def selftest():
    global tof_sensors, imu_sensor, colour_sensor, _last_count, _last_ms

    class _FakeI2C:             # test double, not part of the module
        def writeto(self, address, data):
            pass

    class _FakeToF:
        def __init__(self, value):
            self.value = value

        def read(self):
            if self.value is None:
                raise OSError("gone")
            return self.value

    class _FakeIMU:
        quaternion = (0, 0, 0.7071068, 0.7071068)   # a quarter turn

    class _FakeColour:
        def read(self, raw=False):
            return (1500, 400, 300, 2400)           # a red surface

    distance.i2c = imu.i2c = colour.i2c = _FakeI2C()
    tof_sensors = {"front": _FakeToF(421), "left": _FakeToF(180),
                   "right": _FakeToF(205), "rear": _FakeToF(None)}
    imu_sensor = _FakeIMU()
    colour_sensor = _FakeColour()
    encoder.pulse_count = 0
    _last_count, _last_ms = 0, encoder.ticks_ms()

    state = get_robot_state()

    # every documented key is present, and nothing extra crept in
    assert set(state) == set(EXPECTED_KEYS), sorted(state)

    # readings arrive under their position names
    assert state["front_distance"] == 421
    assert state["left_distance"] == 180
    assert state["right_distance"] == 205
    # the dead rear sensor is None, and did not stop the other three
    assert state["rear_distance"] is None

    assert abs(state["heading"] - 90) < 0.01, state["heading"]
    assert state["floor_colour"] == "RED"
    assert state["rgb"] == (1500, 400, 300)
    assert state["distance_travelled"] == 0 and state["speed"] == 0

    # a wheel turn between calls shows up as travel
    encoder.pulse_count = encoder.PULSES_PER_WHEEL_REV
    moved = get_robot_state()
    assert abs(moved["distance_travelled"]
               - encoder.WHEEL_CIRCUMFERENCE_MM / 10) < 1e-9, moved

    # with nothing initialised at all, the shape still holds - no exceptions
    tof_sensors = {"front": None, "left": None, "right": None, "rear": None}
    imu_sensor = colour_sensor = None
    blank = get_robot_state()
    assert set(blank) == set(EXPECTED_KEYS)
    assert blank["front_distance"] is None and blank["heading"] is None
    assert blank["floor_colour"] is None and blank["rgb"] is None

    print("selftest ok  keys:", len(EXPECTED_KEYS),
          " heading %.1f" % state["heading"], " floor", state["floor_colour"])


if __name__ == "__main__":
    main()
