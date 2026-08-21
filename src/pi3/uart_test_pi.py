"""UART sender: Raspberry Pi 3 -> Pico 2 W.

Sends one line per second, "speed,steering\n", and prints each one first.
Nothing is read back - this only proves the Pi can talk to the Pico.

Run:  python uart_test_pi.py
      python uart_test_pi.py --selftest    (checks the wire format, no hardware)
Ctrl-C to stop.

Needs pyserial:  pip install pyserial
"""

import sys
import time

PORT = "/dev/serial0"   # Pi 3: GPIO14/15. See the wiring notes in the chat.
BAUD = 115200

# speed 0-100, steering negative=left, positive=right, 0=centre
COMMANDS = [(40, 0), (40, -20), (40, 20), (0, 0)]


def parse(line):
    """Same three lines as the Pico's parser, kept here so the wire format has
    a test. The two boards cannot share a file, so this is the contract copy."""
    values = [int(v) for v in line.strip().split(",")]
    return values[0], values[1], values[2:]      # speed, steering, spare fields


def main():
    import serial   # imported here so --selftest works without pyserial

    try:
        pico = serial.Serial(PORT, BAUD, timeout=1)
    except serial.SerialException as error:
        sys.exit(f"Could not open {PORT}: {error}")

    time.sleep(2)       # let the port settle before the first line
    print(f"Sending on {PORT} at {BAUD} baud. Ctrl-C to stop.")

    try:
        while True:
            for speed, steering in COMMANDS:
                message = f"{speed},{steering}\n"
                print("Sending:", message.strip())
                pico.write(message.encode())
                time.sleep(1)
    except serial.SerialException as error:
        sys.exit(f"Serial link lost: {error}")
    except KeyboardInterrupt:
        print("\nStopped")
    finally:
        pico.close()


def selftest():
    assert parse("40,-20\n") == (40, -20, [])
    assert parse("0,0") == (0, 0, [])
    # extra fields parse fine and land in the spare list, so adding
    # state/flags later needs no change on either board
    assert parse("40,-20,2\n") == (40, -20, [2])
    assert parse("40,-20,2,1\n") == (40, -20, [2, 1])

    for junk in ("", "hello", "40", "40,", "40,abc", "40;-20"):
        try:
            parse(junk)
        except (ValueError, IndexError):
            continue                    # exactly what the Pico ignores
        raise AssertionError(f"{junk!r} should not have parsed")

    print("selftest ok")


if __name__ == "__main__":
    selftest() if "--selftest" in sys.argv else main()
