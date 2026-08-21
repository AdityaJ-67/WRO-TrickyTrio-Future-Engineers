"""Bench test: prove the Pi can talk to the Pico. Not the flight program - see main.py.

UART receiver: Pico 2 W <- Raspberry Pi 3.  MicroPython.

Reads "speed,steering\n" lines and prints them. Bad lines are ignored.
Save this on the Pico as main.py so it runs on power-up.

Wiring: Pi GPIO14 (TX) -> Pico GP1 (RX), Pi GPIO15 (RX) -> Pico GP0 (TX),
and a shared GND. Both boards are 3.3 V, so no level shifter.
"""

from machine import UART, Pin  # type: ignore  - MicroPython built-in, only exists on the Pico

BAUD = 115200

uart = UART(0, baudrate=BAUD, tx=Pin(0), rx=Pin(1))

print("Listening at", BAUD, "baud")

buffer = b""

while True:
    if uart.any():
        buffer += uart.read()

        # A read can arrive split across chunks or several lines at once, so
        # work through whatever complete lines the buffer now holds.
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)

            try:
                values = [int(v) for v in line.decode().strip().split(",")]
                speed = values[0]
                steering = values[1]
                # values[2:] is where state and flags will arrive later -
                # nothing reads them yet, and extra fields do not break this
            except (ValueError, IndexError, UnicodeError):
                print("Ignored:", line)
                continue

            print("Received Speed:", speed)
            print("Received Steering:", steering)

    # Guard against a flood of noise on an unplugged RX pin filling memory
    if len(buffer) > 200:
        buffer = b""
