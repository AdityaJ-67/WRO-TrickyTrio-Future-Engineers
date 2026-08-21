"""BNO085 orientation on channel 2 of the TCA9548A.  MicroPython.

Reports heading, pitch and roll. Reads only - no navigation, no PID, no motors.

The driver hands back a quaternion (the rotation vector report), which this
module converts to Euler angles itself. Every BNO08x driver exposes the
quaternion; only some expose Euler angles, so converting here keeps the module
working whichever port you install.

On the Pico:  mpremote run imu.py   (needs a bno08x driver on the board)
On a laptop:  python3 -c "import imu; imu.selftest()"
"""

# --- 1. Imports -------------------------------------------------------------
from math import asin, atan2, degrees

try:
    from machine import I2C, Pin        # type: ignore
    from time import sleep_ms           # type: ignore
except ImportError:                     # laptop: the maths below is still testable
    I2C = Pin = None

    def sleep_ms(ms):
        pass

# Separate block on purpose - see the note in distance.py.
try:
    from bno08x import BNO08X_I2C, BNO_REPORT_ROTATION_VECTOR   # type: ignore
except ImportError:
    BNO08X_I2C = BNO_REPORT_ROTATION_VECTOR = None

# --- 2. Configuration -------------------------------------------------------
I2C_ID = 0                  # GP4/GP5 are I2C0
SDA_PIN = 4
SCL_PIN = 5
I2C_FREQ = 400_000          # drop to 100_000 if the BNO085 stretches the clock

TCA_ADDRESS = 0x70
IMU_CHANNEL = 2             # channels 0, 3, 4, 5 are the VL53L0X sensors

READ_INTERVAL_MS = 50       # 20 Hz is plenty to watch on a console
RETRY_MS = 500              # wait between reconnection attempts

# --- 3. I2C initialization --------------------------------------------------
i2c = I2C(I2C_ID, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ) if I2C else None


# --- 4. Multiplexer ---------------------------------------------------------
def select_channel(channel):
    """Point the TCA9548A at one channel - one bit per channel, one at a time."""
    i2c.writeto(TCA_ADDRESS, bytes([1 << channel]))


# --- 5. IMU initialization --------------------------------------------------
def setup_imu():
    """Bring the BNO085 up. Returns the sensor, or None if it is not there."""
    if BNO08X_I2C is None:
        print("BNO085 driver missing - copy bno08x.py to the board")
        return None
    try:
        select_channel(IMU_CHANNEL)
        bno = BNO08X_I2C(i2c)
        bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
        print("BNO085 ready on channel", IMU_CHANNEL)
        return bno
    except Exception as error:
        print("BNO085 init failed -", error)
        return None


# --- 6. Orientation reading -------------------------------------------------
def quaternion_to_euler(i, j, k, real):
    """Rotation vector -> (heading 0-360, pitch, roll) in degrees.

    Heading is wrapped to compass style so it never reads -170 one frame and
    +190 the next. Pitch is clamped before asin because floating point rounding
    can push the argument a hair past 1.0 and raise a domain error.
    """
    yaw = atan2(2 * (real * k + i * j), 1 - 2 * (j * j + k * k))
    pitch = asin(max(-1.0, min(1.0, 2 * (real * j - k * i))))
    roll = atan2(2 * (real * i + j * k), 1 - 2 * (i * i + j * j))
    return degrees(yaw) % 360, degrees(pitch), degrees(roll)


def read_orientation(bno):
    """(heading, pitch, roll) in degrees, or None if the read failed."""
    try:
        select_channel(IMU_CHANNEL)
        i, j, k, real = bno.quaternion
        return quaternion_to_euler(i, j, k, real)
    except Exception:
        return None


# --- 7. Main loop -----------------------------------------------------------
def main():
    bno = None

    while True:
        # Re-initialise whenever we have no working sensor, so a knocked-out
        # cable recovers on its own rather than ending the run.
        if bno is None:
            bno = setup_imu()
            if bno is None:
                print("IMU ERROR")
                sleep_ms(RETRY_MS)
                continue

        orientation = read_orientation(bno)
        if orientation is None:
            print("IMU ERROR")
            bno = None                  # force a fresh init next time round
            sleep_ms(RETRY_MS)
            continue

        heading, pitch, roll = orientation
        print("Heading : %.1f°" % heading)
        print("Pitch   : %.1f°" % pitch)
        print("Roll    : %.1f°" % roll)
        print("-" * 20)

        sleep_ms(READ_INTERVAL_MS)


def selftest():
    global i2c

    class _FakeI2C:             # test double, not part of the module
        def __init__(self):
            self.written = []

        def writeto(self, address, data):
            self.written.append((address, data))

    class _FakeIMU:
        def __init__(self, quaternion):
            self._quaternion = quaternion

        @property
        def quaternion(self):
            if self._quaternion is None:
                raise OSError("imu gone")
            return self._quaternion

    i2c = _FakeI2C()

    # the IMU lives on channel 2 alone, and must not disturb the ToF channels
    select_channel(IMU_CHANNEL)
    assert i2c.written == [(TCA_ADDRESS, b"\x04")], i2c.written

    def close(actual, expected, tolerance=0.01):
        return abs(actual - expected) < tolerance

    # level and facing zero
    heading, pitch, roll = quaternion_to_euler(0, 0, 0, 1)
    assert close(heading, 0) and close(pitch, 0) and close(roll, 0)

    # a quarter turn about the vertical axis is 90 degrees of heading
    heading, pitch, roll = quaternion_to_euler(0, 0, 0.7071068, 0.7071068)
    assert close(heading, 90), heading
    assert close(pitch, 0) and close(roll, 0)

    # half a turn, and heading wraps to 0-360 rather than going negative
    assert close(quaternion_to_euler(0, 0, 1, 0)[0], 180)
    assert close(quaternion_to_euler(0, 0, -0.7071068, 0.7071068)[0], 270)

    # nose up is pitch, leaning over is roll, neither leaks into heading
    _, pitch, _ = quaternion_to_euler(0, 0.2588190, 0, 0.9659258)
    assert close(pitch, 30), pitch
    _, _, roll = quaternion_to_euler(0.2588190, 0, 0, 0.9659258)
    assert close(roll, 30), roll

    # straight up would push asin past 1.0 on rounding - must not raise
    quaternion_to_euler(0, 0.7071068, 0, 0.7071068)

    # a dead IMU reports None instead of taking the loop down
    assert read_orientation(_FakeIMU(None)) is None
    assert read_orientation(_FakeIMU((0, 0, 0, 1))) is not None

    print("selftest ok  90deg quaternion ->  heading %.1f  pitch %.1f  roll %.1f"
          % quaternion_to_euler(0, 0, 0.7071068, 0.7071068))


if __name__ == "__main__":
    main()
