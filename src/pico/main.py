"""Pico 2 W flight program: receive commands, drive, report state.

    UART "45,23"  ->  motionController.move(45, 23)
    sensorManager.get_robot_state()  ->  UART "S,480,230,260,900,91.3"

Runs automatically on power-up, because MicroPython executes main.py at boot.

The loop deliberately runs at two speeds. Commands are read and applied every
single pass, because a command that waits is a robot that has already driven
somewhere else. A full sensor sweep costs about 130 ms - mostly the four
VL53L0X measurements - so it happens on its own slower interval and never
delays the motor.

If the Pi goes quiet, the watchdog stops the robot. That is the whole reason
this file exists rather than the Pi writing to the motor directly.

On the Pico:  save as main.py, it runs at boot
On a laptop:  python3 -c "import main; main.selftest()"   (protocol only)
"""

try:
    from machine import Pin, UART       # type: ignore
    from time import sleep_ms, ticks_diff, ticks_ms    # type: ignore
except ImportError:                     # laptop: the protocol is still testable
    Pin = UART = None
    from time import monotonic

    def sleep_ms(ms):
        pass

    def ticks_ms():
        return int(monotonic() * 1000)

    def ticks_diff(a, b):
        return a - b

import motionController
import sensorManager

# --- Configuration ----------------------------------------------------------
UART_ID = 0                 # GP0 = TX, GP1 = RX
BAUD = 115200

LOOP_MS = 20                # 50 Hz: commands are applied within 20 ms of arrival
SENSOR_INTERVAL_MS = 150    # a full sweep costs ~130 ms, so do not ask for more
COMMAND_TIMEOUT_MS = 500    # no command for this long -> stop

STATE_PREFIX = "S"
MAX_BUFFER = 200            # guard against noise on a floating RX pin
UNKNOWN = -1                # a distance the sensor could not measure


def parse_command(line):
    """b"45,23" -> (45, 23), or None if the line is junk.

    Extra fields are accepted and ignored, so the Pi can add state or flags
    later without this needing a change.
    """
    try:
        values = [int(value) for value in line.decode().strip().split(",")]
        return values[0], values[1]
    except (ValueError, IndexError, UnicodeError):
        return None


def state_line(state):
    """robot_state -> the line the Pi parses.

    Distances are integer millimetres, with -1 for a sensor that had no
    reading. Heading is appended only when it is known - the Pi treats a
    missing field as unknown, so there is no fake value to misread.
    """
    def mm(value):
        return UNKNOWN if value is None else int(value)

    line = "%s,%d,%d,%d,%d" % (STATE_PREFIX,
                               mm(state["front_distance"]), mm(state["left_distance"]),
                               mm(state["right_distance"]), mm(state["rear_distance"]))
    if state["heading"] is not None:
        line += ",%.1f" % state["heading"]
    return line + "\n"


def main():
    motionController.setup()        # centres the steering and stops the motor
    sensorManager.setup()
    uart = UART(UART_ID, baudrate=BAUD, tx=Pin(0), rx=Pin(1))
    print("Pico ready. Waiting for commands.")

    buffer = b""
    last_command_ms = ticks_ms()
    last_sensor_ms = ticks_ms()
    stopped = True

    while True:
        now = ticks_ms()

        # --- 1. read every command that has arrived, apply only the newest ---
        if uart.any():
            buffer += uart.read()
        command = None
        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            parsed = parse_command(line)
            if parsed:
                command = parsed          # a queued command is already stale
        if len(buffer) > MAX_BUFFER:
            buffer = b""

        if command:
            motionController.move(command[0], command[1])
            last_command_ms = now
            stopped = False

        # --- 2. watchdog: silence from the Pi must not mean "carry on" ---
        elif not stopped and ticks_diff(now, last_command_ms) > COMMAND_TIMEOUT_MS:
            motionController.stop()
            stopped = True
            print("No command for %d ms, stopped" % COMMAND_TIMEOUT_MS)

        # --- 3. sensors, on their own slower clock ---
        if ticks_diff(now, last_sensor_ms) >= SENSOR_INTERVAL_MS:
            last_sensor_ms = now
            try:
                uart.write(state_line(sensorManager.get_robot_state()).encode())
            except Exception as error:
                # A sensor fault must never stop us listening for commands.
                print("Sensor sweep failed:", error)

        sleep_ms(LOOP_MS)


def selftest():
    # --- commands from the Pi ---
    assert parse_command(b"45,23") == (45, 23)
    assert parse_command(b"0,0\n") == (0, 0)
    assert parse_command(b"40,-20\r\n") == (40, -20)
    assert parse_command(b"45,23,2,1") == (45, 23)      # extra fields ignored
    for junk in (b"", b"hello", b"45", b"45,", b"45,abc", b"\n"):
        assert parse_command(junk) is None, junk

    # --- state lines to the Pi ---
    full = {"front_distance": 480, "left_distance": 230, "right_distance": 260,
            "rear_distance": 900, "heading": 91.3}
    assert state_line(full) == "S,480,230,260,900,91.3\n", state_line(full)

    # a failed sensor is -1, never a plausible-looking distance
    broken = dict(full, left_distance=None)
    assert state_line(broken) == "S,480,-1,260,900,91.3\n", state_line(broken)

    # an unknown heading is omitted, not sent as a fake value
    no_heading = dict(full, heading=None)
    assert state_line(no_heading) == "S,480,230,260,900\n", state_line(no_heading)

    # floats from the sensors must not leak into the line
    floaty = dict(full, front_distance=480.7)
    assert state_line(floaty) == "S,480,230,260,900,91.3\n", state_line(floaty)

    # --- the two boards must agree on the wire format ---
    import importlib.util
    import sys
    from pathlib import Path
    pi3 = Path(__file__).resolve().parent.parent / "pi3"
    if (pi3 / "main.py").is_file():
        # Loaded by path under another name: both boards call their file main.py.
        sys.path.insert(0, str(pi3))
        sys.path.insert(0, str(pi3 / "camera_vision"))
        spec = importlib.util.spec_from_file_location("pi_main", pi3 / "main.py")
        pi_main = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pi_main)

        round_trip = pi_main.parse_state(state_line(full))
        assert round_trip["front_distance"] == 480
        assert round_trip["left_distance"] == 230
        assert round_trip["rear_distance"] == 900
        assert round_trip["heading"] == 91.3

        assert pi_main.parse_state(state_line(broken))["left_distance"] is None
        assert pi_main.parse_state(state_line(no_heading))["heading"] is None

        # and the Pi's command format must be one this file can read
        assert parse_command(b"%d,%d\n" % (45, 23)) == (45, 23)
        assert parse_command(pi_main.STOP_COMMAND.encode()) == (0, 0)
        print("selftest ok  wire format verified against the Pi's own parser")
    else:
        print("selftest ok  (Pi side not present, wire format unverified)")


if __name__ == "__main__":
    main()
