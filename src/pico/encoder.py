"""Wheel encoder on the GA12-N20 motor.  MicroPython, Pico 2 W.

Counts quadrature pulses on an interrupt and reports rotations, distance and
speed. Reads only - nothing here drives the motor.

On the Pico:  run this file from Thonny, then turn the wheel by hand.
On a laptop:  python3 -c "import encoder; encoder.selftest()"   (maths only)
"""

from math import pi

try:
    import micropython  # type: ignore
    from machine import Pin  # type: ignore
    from time import ticks_diff, ticks_ms  # type: ignore
    micropython.alloc_emergency_exception_buf(100)   # readable errors from an ISR
except ImportError:          # laptop: the maths below is still testable
    Pin = None
    from time import monotonic

    def ticks_ms():
        return int(monotonic() * 1000)

    def ticks_diff(a, b):
        return a - b

# --- Configuration ----------------------------------------------------------
ENCODER_A_PIN = 12          # physical pin 16
ENCODER_B_PIN = 13          # physical pin 17

# GA12-N20 encoders are 7 pulses per turn of the MOTOR shaft, then the gearbox
# divides that down. CHECK YOUR GEARBOX - N20s ship as 1:30 up to 1:298 and the
# ratio is the single biggest source of wrong distances.
PULSES_PER_MOTOR_REV = 7
GEAR_RATIO = 100
PULSES_PER_WHEEL_REV = PULSES_PER_MOTOR_REV * GEAR_RATIO

WHEEL_DIAMETER_MM = 33.0    # MEASURE yours across the tyre, not the hub
WHEEL_CIRCUMFERENCE_MM = pi * WHEEL_DIAMETER_MM

DIRECTION_SIGN = 1          # set to -1 if forward counts down
PRINT_INTERVAL_MS = 500

pulse_count = 0             # written by the interrupt, read by the main loop


def measure(count, delta_count, delta_ms):
    """Pulse counts -> (rotations, distance in cm, speed in cm/s)."""
    rotations = count / PULSES_PER_WHEEL_REV
    distance_cm = rotations * WHEEL_CIRCUMFERENCE_MM / 10
    moved_cm = delta_count / PULSES_PER_WHEEL_REV * WHEEL_CIRCUMFERENCE_MM / 10
    speed_cm_s = moved_cm * 1000 / delta_ms if delta_ms else 0.0
    return rotations, distance_cm, speed_cm_s


def main():
    channel_a = Pin(ENCODER_A_PIN, Pin.IN, Pin.PULL_UP)
    channel_b = Pin(ENCODER_B_PIN, Pin.IN, Pin.PULL_UP)

    # Fire on one edge of A only, and read B inside to get direction. B leads or
    # lags A by a quarter cycle depending on which way the wheel turns, so its
    # level at the instant A rises is the direction.
    def on_pulse(pin):
        global pulse_count
        if channel_b.value():
            pulse_count -= DIRECTION_SIGN
        else:
            pulse_count += DIRECTION_SIGN

    channel_a.irq(trigger=Pin.IRQ_RISING, handler=on_pulse)

    print("Counting.", PULSES_PER_WHEEL_REV, "pulses per wheel turn,",
          "%.1f mm circumference" % WHEEL_CIRCUMFERENCE_MM)

    last_count = 0
    last_ms = ticks_ms()

    while True:
        now_ms = ticks_ms()
        delta_ms = ticks_diff(now_ms, last_ms)
        if delta_ms < PRINT_INTERVAL_MS:
            continue

        count = pulse_count                 # single read, so it cannot tear
        rotations, distance_cm, speed_cm_s = measure(
            count, count - last_count, delta_ms)
        last_count, last_ms = count, now_ms

        print("Pulses:", count,
              " Rotations: %.2f" % rotations,
              " Distance: %.1f cm" % distance_cm,
              " Speed: %.1f cm/s" % speed_cm_s)


def selftest():
    circumference_cm = WHEEL_CIRCUMFERENCE_MM / 10

    # exactly one wheel turn
    rotations, distance, _ = measure(PULSES_PER_WHEEL_REV, 0, 500)
    assert abs(rotations - 1.0) < 1e-9, rotations
    assert abs(distance - circumference_cm) < 1e-9, distance

    # ten turns is ten times the distance
    assert abs(measure(10 * PULSES_PER_WHEEL_REV, 0, 500)[1]
               - 10 * circumference_cm) < 1e-9

    # one wheel turn in one second == circumference per second
    _, _, speed = measure(0, PULSES_PER_WHEEL_REV, 1000)
    assert abs(speed - circumference_cm) < 1e-9, speed
    # half the time, twice the speed
    _, _, faster = measure(0, PULSES_PER_WHEEL_REV, 500)
    assert abs(faster - 2 * speed) < 1e-9, faster

    # stationary, and a zero interval must not divide by zero
    assert measure(0, 0, 500) == (0.0, 0.0, 0.0)
    assert measure(100, 0, 0)[2] == 0.0

    # rolling backwards reads negative, not as a huge positive
    assert measure(-PULSES_PER_WHEEL_REV, 0, 500)[1] < 0

    print("selftest ok  1 turn = %.1f cm over %d pulses"
          % (circumference_cm, PULSES_PER_WHEEL_REV))


if __name__ == "__main__":
    main()
