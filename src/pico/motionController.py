"""Motion Controller: one call to set how fast and which way the robot goes.

    move(speed, steering)   speed 0-100, steering -30 to +30 degrees
    stop()                  motor off, wheels centred

Coordination only. The clamping and the duty-cycle maths already live in
drv8833.py and servo.py, and this module calls them rather than repeating them.
No PID, no acceleration ramps, no autonomy.

On the Pico:  mpremote run motionController.py
On a laptop:  python3 -c "import motionController; motionController.selftest()"
"""

try:
    from machine import PWM, Pin        # type: ignore
    from time import sleep_ms           # type: ignore
except ImportError:                     # laptop: the coordination is still testable
    PWM = Pin = None

    def sleep_ms(ms):
        pass

import drv8833
import servo

# Hardware handles, filled in by setup().
motor_pwm = None
steering_pwm = None

STEP_MS = 1500      # how long the test loop holds each command


def setup():
    """Create the two PWM outputs. Call once before move()."""
    global motor_pwm, steering_pwm

    motor_pwm = PWM(Pin(drv8833.AIN1_PIN))
    motor_pwm.freq(drv8833.FREQ)

    # AIN2 stays low: on a DRV8833 the input carrying the PWM is the direction,
    # so holding this one down is what makes AIN1 mean forward.
    Pin(drv8833.AIN2_PIN, Pin.OUT).value(0)

    steering_pwm = PWM(Pin(servo.SERVO_PIN))
    steering_pwm.freq(servo.FREQ)

    stop()
    print("Motion ready. Speed 0-%d, steering +/-%d deg"
          % (drv8833.MAX_SPEED, servo.MAX_STEER))


def move(speed, steering):
    """Drive at speed (0-100) while steering (-30 to +30 degrees).

    Both values are clamped by the modules that own them, and the clamped
    values are returned so the caller can see what actually got used.
    """
    speed, motor_duty = drv8833.speed_to_duty(speed)
    angle, servo_duty = servo.steering_to_duty(steering)

    motor_pwm.duty_u16(motor_duty)
    steering_pwm.duty_u16(servo_duty)
    return speed, angle


def stop():
    """Motor off, wheels straight."""
    return move(0, 0)


def main():
    setup()

    for speed, steering in ((30, 0), (50, -20), (50, 20)):
        actual_speed, actual_angle = move(speed, steering)
        print("move(%d, %d) -> speed %d, steering %d deg"
              % (speed, steering, actual_speed, actual_angle))
        sleep_ms(STEP_MS)

    stop()
    print("stop() -> speed 0, steering 0 deg")


def selftest():
    global motor_pwm, steering_pwm

    class _FakePWM:             # test double, not part of the module
        def __init__(self):
            self.duty = None

        def duty_u16(self, value):
            self.duty = value

    motor_pwm, steering_pwm = _FakePWM(), _FakePWM()

    # in-range values pass straight through
    assert move(30, 0) == (30, 0)
    assert move(50, -20) == (50, -20)
    assert move(50, 20) == (50, 20)

    # and the duties actually reach the hardware, not just the return value
    move(50, 20)
    assert motor_pwm.duty == drv8833.speed_to_duty(50)[1]
    assert steering_pwm.duty == servo.steering_to_duty(20)[1]

    # clamping is delegated, so the limits are whatever those modules say
    assert move(999, 0)[0] == drv8833.MAX_SPEED
    assert move(-50, 0)[0] == 0                     # no reverse: negative means stop
    assert move(0, 90)[1] == servo.MAX_STEER
    assert move(0, -90)[1] == -servo.MAX_STEER

    # stop is exactly move(0, 0) - motor off and wheels centred
    assert stop() == (0, 0)
    assert motor_pwm.duty == 0
    assert steering_pwm.duty == servo.steering_to_duty(0)[1]

    # a hard-over command followed by stop must leave nothing latched on
    move(100, 30)
    stop()
    assert motor_pwm.duty == 0

    print("selftest ok  move(50,-20) duties: motor %d, servo %d"
          % (drv8833.speed_to_duty(50)[1], servo.steering_to_duty(-20)[1]))


if __name__ == "__main__":
    main()
