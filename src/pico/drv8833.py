"""Drive motor control through a DRV8833 on the Pico 2 W.  MicroPython.

Reads "speed,steering" lines from the Pi over UART and drives one motor forward.
The steering value is parsed and ignored - this module only drives.

Forward only. No reverse, no braking.

On the Pico:  run this file from Thonny.
On a laptop:  python3 -c "import drv8833; drv8833.selftest()"   (maths only)
"""

try:
    from machine import PWM, Pin, UART  # type: ignore
except ImportError:          # laptop: the conversion below is still testable
    PWM = Pin = UART = None

# --- Motor ------------------------------------------------------------------
AIN1_PIN = 8                # physical pin 11. PWM here = forward
AIN2_PIN = 9                # physical pin 12. Held low for forward

FREQ = 20_000               # 20 kHz: above hearing, so the motor does not whine

# Drop this to 40 or so for the first bench test, then raise it once the robot
# behaves. Nothing else needs changing.
MAX_SPEED = 100

# --- UART (same link as uart_test_pi.py) ------------------------------------
BAUD = 115200


def speed_to_duty(speed):
    """Speed 0-100 -> (clamped speed, PWM duty).

    Negative speeds clamp to 0, not to reverse: reverse is deliberately not
    implemented, so a bad value has to mean stop rather than something surprising.
    """
    speed = max(0, min(MAX_SPEED, speed))
    return speed, int(speed * 65535 / 100)


def main():
    ain1 = PWM(Pin(AIN1_PIN))
    ain1.freq(FREQ)
    ain2 = Pin(AIN2_PIN, Pin.OUT)

    # AIN2 low the whole time. Which of the two inputs carries the PWM is what
    # sets direction on a DRV8833 - there is no separate direction pin.
    ain2.value(0)
    ain1.duty_u16(0)
    uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1))
    print("Motor ready. Max speed:", MAX_SPEED)

    buffer = b""
    while True:
        if uart.any():
            buffer += uart.read()

            # A read can hold half a line or several lines, so drain whole ones.
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)

                try:
                    values = [int(v) for v in line.decode().strip().split(",")]
                    speed = values[0]            # values[1] is steering, ignored here
                except (ValueError, IndexError, UnicodeError):
                    print("Ignored:", line)
                    continue

                speed, duty = speed_to_duty(speed)
                ain1.duty_u16(duty)

                print("Received Speed:", speed)
                print("PWM Duty:", duty)
                print("Motor Running" if speed else "Motor Stopped")

        if len(buffer) > 200:
            buffer = b""


def selftest():
    assert speed_to_duty(0) == (0, 0)
    assert speed_to_duty(100) == (100, 65535)

    # reverse is not implemented, so anything negative must mean stop
    assert speed_to_duty(-1) == (0, 0)
    assert speed_to_duty(-100) == (0, 0)

    # out of range clamps instead of overflowing the duty register
    assert speed_to_duty(255) == speed_to_duty(MAX_SPEED)
    assert speed_to_duty(MAX_SPEED)[1] <= 65535

    # faster in always means a bigger duty out
    duties = [speed_to_duty(s)[1] for s in range(0, 101, 10)]
    assert duties == sorted(duties) and len(set(duties)) == len(duties)

    print("selftest ok  30% ->", speed_to_duty(30)[1],
          " 50% ->", speed_to_duty(50)[1], " 100% ->", speed_to_duty(100)[1])


if __name__ == "__main__":
    main()
