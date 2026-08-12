"""
Raw TCS34725 dump -- run from the Pico REPL during practice.

    >>> import tools_colour_dump as d
    >>> d.run()

Prints c, r, g, b and the derived r/c and b/c ratios at 10 Hz. Slide the
sensor over the white mat, then an orange line, then a blue one, and read
off the ratio clusters.

WHY THIS IS A SEPARATE MANUAL STEP (and not automated into calibrate.py):
the lap counter is load-bearing -- a misclassified line is a missed lap, and
a missed lap is the round. Ratio thresholds get looked at by a human before
they are written into config.json. The rest of calibration is automated; this
one deliberately is not.

Set the thresholds MIDWAY between the clusters, not at their edges. If white
sits at r/c = 0.33 and orange at r/c = 0.51, use 0.42 -- the widest margin
available is the one that survives a lighting change at the venue.

Rule 9.9: practice tool. It is never imported by main.py.
"""

import time
from machine import Pin, I2C

from tca9548a import TCA9548A
from tcs34725 import TCS34725

# Must match pico/main.py. Duplicated rather than imported because main.py
# starts a control loop and a watchdog on import, which is the last thing you
# want while poking at a sensor from the REPL.
PIN_I2C_SDA, PIN_I2C_SCL = 4, 5
PIN_COLOUR_LED = 18
MUX_ADDR = 0x70
COLOUR_CHANNEL = 1


def run(seconds=120, hz=10):
    i2c = I2C(0, sda=Pin(PIN_I2C_SDA), scl=Pin(PIN_I2C_SCL), freq=400000)
    mux = TCA9548A(i2c, MUX_ADDR)
    if not mux.probe():
        print("mux not found at 0x%02X" % MUX_ADDR)
        return
    led = Pin(PIN_COLOUR_LED, Pin.OUT, value=0)
    sensor = TCS34725(i2c, led_pin=led)

    with mux.channel(COLOUR_CHANNEL):
        if not sensor.init():
            print("TCS34725 init failed (id=%r)" % sensor.device_id)
            return

    print("     c     r     g     b     r/c    g/c    b/c")
    end = time.ticks_add(time.ticks_ms(), int(seconds * 1000))
    period = 1000 // hz
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        with mux.channel(COLOUR_CHANNEL):
            s = sensor.read_raw()
        if s is not None:
            c, r, g, b = s
            if c > 0:
                print("%6d%6d%6d%6d   %.3f  %.3f  %.3f"
                      % (c, r, g, b, r / c, g / c, b / c))
            else:
                print("%6d%6d%6d%6d   (clear=0: LED off or sensor lifted?)"
                      % (c, r, g, b))
        time.sleep_ms(period)
    sensor.led(False)
