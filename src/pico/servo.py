"""Steering servo control on the Pico 2 W.  MicroPython.

Reads "speed,steering" lines from the Pi over UART and moves an MG90S.
The speed value is parsed and ignored - this module only steers.

On the Pico:  run this file from Thonny.
On a laptop:  python3 -c "import servo; servo.selftest()"   (maths only)
"""

try:
    from machine import PWM, Pin, UART  # type: ignore
except ImportError:          # laptop: the conversion below is still testable
    PWM = Pin = UART = None

# --- Servo ------------------------------------------------------------------
SERVO_PIN = 22             # physical pin 29
FREQ = 50                   # 50 Hz -> one pulse every 20000 us, what MG90S expects
PERIOD_US = 1_000_000 // FREQ

CENTRE_US = 1500            # CALIBRATE: raise/lower until the wheels point straight
US_PER_DEGREE = 10.5        # MG90S sweeps ~180 deg over 500-2400 us
STEER_DIRECTION = 1         # set to -1 if the servo turns the wrong way

MAX_STEER = 30              # degrees each side
MIN_US, MAX_US = 1000, 2000  # hard limit, so a bad CENTRE_US cannot drive the
                             # servo into the steering linkage and stall it

# --- UART (same link as uart_test_pi.py) ------------------------------------
BAUD = 115200


def steering_to_duty(angle):
    """Steering angle in degrees -> (clamped angle, PWM duty).

    Straight is CENTRE_US. Every degree adds US_PER_DEGREE microseconds of pulse
    width, then the pulse is turned into duty_u16's 0-65535 scale as the
    fraction of the 20 ms frame that the pulse occupies.
    """
    angle = max(-MAX_STEER, min(MAX_STEER, angle))
    pulse_us = CENTRE_US + angle * US_PER_DEGREE * STEER_DIRECTION
    pulse_us = max(MIN_US, min(MAX_US, pulse_us))
    duty = int(pulse_us * 65535 / PERIOD_US)
    return angle, duty


def main():
    servo = PWM(Pin(SERVO_PIN))
    servo.freq(FREQ)
    uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1))

    angle, duty = steering_to_duty(0)       # start centred
    servo.duty_u16(duty)
    print("Steering ready. Centre duty:", duty)

    buffer = b""
    while True:
        if uart.any():
            buffer += uart.read()

            # A read can hold half a line or several lines, so drain whole ones.
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)

                try:
                    values = [int(v) for v in line.decode().strip().split(",")]
                    steering = values[1]         # values[0] is speed, ignored here
                except (ValueError, IndexError, UnicodeError):
                    print("Ignored:", line)
                    continue

                angle, duty = steering_to_duty(steering)
                servo.duty_u16(duty)

                print("Received Steering:", steering)
                print("Servo Angle:", angle)
                print("PWM Duty:", duty)

        if len(buffer) > 200:
            buffer = b""


def selftest():
    centre_duty = int(CENTRE_US * 65535 / PERIOD_US)
    assert steering_to_duty(0) == (0, centre_duty), steering_to_duty(0)

    # right is a longer pulse than centre, left is shorter
    assert steering_to_duty(30)[1] > centre_duty > steering_to_duty(-30)[1]

    # out of range values clamp instead of running off the end
    assert steering_to_duty(90) == steering_to_duty(30)
    assert steering_to_duty(-90) == steering_to_duty(-30)
    assert steering_to_duty(90)[0] == 30

    # the hard limit holds even if someone miscalibrates the centre badly
    for duty in (steering_to_duty(a)[1] for a in (-30, 0, 30)):
        assert int(MIN_US * 65535 / PERIOD_US) <= duty <= int(MAX_US * 65535 / PERIOD_US)

    # symmetric about centre, so equal steer each way gives equal travel
    assert centre_duty - steering_to_duty(-20)[1] == steering_to_duty(20)[1] - centre_duty

    print("selftest ok  centre:", centre_duty,
          " -30:", steering_to_duty(-30)[1], " +30:", steering_to_duty(30)[1])


if __name__ == "__main__":
    main()
