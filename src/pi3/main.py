"""Pi 3 coordinator: the only file that knows the modules exist.

    frame -> vision -> pillars ─┐
                                ├─> navigation -> requested command ─┐
    UART  -> robot_state ───────┘                                    │
                                                                     v
                                              state machine -> FINAL command
                                                                     │
                                                          UART ──────┘-> Pico

No navigation logic, no steering maths, no state transitions live here. This
file captures, converts, forwards, and sends - nothing else.

Run:  python main.py                 on the Pi, with the Pico connected
      python main.py --dry-run       laptop: webcam + decisions, no UART
      python main.py --show          draw the camera view (costs frame rate)
      python main.py --debug         one status block per frame
      python main.py --start-now     skip the start countdown
      python main.py --selftest      no camera, no UART
"""

import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent / "camera_vision"))
import cv2                              # noqa: E402
import mission_manager                  # noqa: E402
import navigation_engine                # noqa: E402
import state_machine                    # noqa: E402
import vision_test                      # noqa: E402

# --- Configuration ----------------------------------------------------------
UART_PORT = "/dev/serial0"      # Pi 3: GPIO14/15. Must resolve to ttyAMA0.
UART_BAUD = 115200              # uart_test_pi.py keeps its own copy on purpose:
                                # it is a standalone bench tool, not a dependency
# State line from the Pico: "S,<front>,<left>,<right>,<rear>,<heading>" in mm
# and degrees. A negative distance means that sensor had no reading. Extra
# fields may be appended later without breaking this parser.
STATE_PREFIX = "S"
STOP_COMMAND = "0,0\n"
STOP_REPEATS = 3                # a stop is worth sending more than once
RECONNECT_INTERVAL_S = 1.0      # how often to retry a dropped UART link
LINK_SETTLE_S = 2.0
START_DELAY_S = 3.0             # countdown after launch, so the robot can be
                                # placed. Replace with a real start button on
                                # the Pico when one is wired.
SLOW_FRAME_WARN_S = 0.20        # a frame slower than this risks tripping the
                                # Pico's 500 ms command watchdog
CM_TO_MM = 10
FPS_SMOOTHING = 0.9

DEBUG = False


# --- Conversions ------------------------------------------------------------
def pillars_from_vision(seen):
    """vision_test.detect() output -> the pillar list navigation expects.

    Only the coloured pillars come through; a wall is not something you pass on
    a side, so navigation never sees it.
    """
    pillars = []
    for colour, info in seen.items():
        if colour not in navigation_engine.PASS_TARGET:
            continue
        width, height = info["box"][2], info["box"][3]
        pillars.append({
            "colour": colour,
            "x": info["cx"],
            "area": width * height,
            "distance": info["distance"] * CM_TO_MM,    # vision works in cm
        })
    return pillars


def parse_state(line):
    """A state line from the Pico -> robot_state dict, or None if it is junk."""
    try:
        fields = line.strip().split(",")
        if fields[0] != STATE_PREFIX:
            return None
        front, left, right, rear = (int(value) for value in fields[1:5])
        heading = float(fields[5]) if len(fields) > 5 else None
    except (ValueError, IndexError):
        return None

    def known(value):
        return None if value < 0 else value

    return {"front_distance": known(front), "left_distance": known(left),
            "right_distance": known(right), "rear_distance": known(rear),
            "heading": heading}


# --- UART -------------------------------------------------------------------
def open_link(dry_run):
    """Open the Pico link, or None if we are running without one."""
    if dry_run:
        return None
    import serial
    link = serial.Serial(UART_PORT, UART_BAUD, timeout=0)
    time.sleep(LINK_SETTLE_S)       # let the port settle before the first command
    return link


def read_state(link, previous):
    """Newest state the Pico has sent, or the previous one if it sent nothing.

    Drains the buffer rather than reading a single line, so a slow frame never
    leaves us acting on stale distances.
    """
    if link is None:
        return previous
    state = previous
    while link.in_waiting:
        parsed = parse_state(link.readline().decode(errors="ignore"))
        if parsed:
            state = parsed
    return state


# --- Debug ------------------------------------------------------------------
def debug_block(behaviour, navigation, robot_state, pillars, fps):
    """One compact status block per frame - the only printing in the file."""
    pillar = pillars[0]["colour"] if pillars else "NONE"
    heading = robot_state.get("heading")
    front = robot_state.get("front_distance")
    print("Mission: %s\n"
          "State: %s\n"
          "Detected Pillar: %s\n"
          "Front Distance: %s\n"
          "Heading: %s\n"
          "Speed: %d\n"
          "Steering: %+d\n"
          "Reason: %s\n"
          "FPS: %.1f\n%s"
          % (behaviour["mission"], behaviour["state"], pillar,
             "%d mm" % front if front is not None else "UNKNOWN",
             "%.1f deg" % heading if heading is not None else "UNKNOWN",
             behaviour["speed"], behaviour["steering"],
             navigation["reason"], fps, "-" * 40))


def overlay(frame, seen, behaviour):
    for colour, info in seen.items():
        vision_test.draw(frame, colour, info)
    cv2.putText(frame, "%s  speed %d  steer %+d"
                % (behaviour["state"], behaviour["speed"], behaviour["steering"]),
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)


# --- Main loop --------------------------------------------------------------
def main(dry_run=False, show=False, debug=False, start_delay=START_DELAY_S):
    camera = cv2.VideoCapture(vision_test.CAMERA_INDEX)
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, vision_test.FRAME_WIDTH)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, vision_test.FRAME_HEIGHT)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not camera.isOpened():
        sys.exit("Could not open camera %s" % vision_test.CAMERA_INDEX)

    import serial                       # for the exception type, even in dry run
    try:
        link = open_link(dry_run)
    except serial.SerialException as error:
        sys.exit("Could not open %s: %s\nUse --dry-run to test without the Pico."
                 % (UART_PORT, error))

    navigation_engine.reset()
    state_machine.DEBUG = debug     # one flag, so state changes print with everything else
    machine = state_machine.StateMachine()

    # The robot must not drive off the moment the program launches. Until a
    # start button exists on the Pico, this countdown is the start signal.
    for remaining in range(int(start_delay), 0, -1):
        print("Starting in %d..." % remaining)
        time.sleep(1)
    machine.start()

    robot_state = {}
    mission = None
    next_reconnect = 0.0
    last_frame_at = time.monotonic()
    fps = 0.0
    print("Running%s. Ctrl-C to stop." % (" (dry run, no UART)" if dry_run else ""))

    try:
        while True:
            # 1. capture. A dropped frame is normal - skip it and try again.
            captured, frame = camera.read()
            if not captured:
                continue

            # Keep navigation's idea of the frame width honest, whatever the
            # camera gave us - every pillar offset is measured against it.
            width = frame.shape[1]
            if navigation_engine.CAMERA_WIDTH != width:
                navigation_engine.CAMERA_WIDTH = width
                navigation_engine.CAMERA_CENTRE_X = width // 2

            # 2. see what is there, in the shape the rest of the stack wants
            seen, _, walls = vision_test.detect(frame)
            # The wall is not a pillar, but the state machine uses it to see a
            # corner coming before the front ToF confirms it.
            vision = {"pillars": pillars_from_vision(seen),
                      "wall_distance": (seen["WALL"]["distance"] * CM_TO_MM
                                        if "WALL" in seen else None),
                      "parking": seen.get("PARKING"),
                      "walls": walls}

            # 3-7. everything below can fail if the Pico link drops
            try:
                # 3. what the robot itself reports
                robot_state = read_state(link, robot_state)

                # 4. which round we are running. Asked every frame, not once at
                # startup, so swapping the constant for automatic detection
                # later needs no change here.
                selected = mission_manager.current_mission()
                if selected["mission"] != mission:
                    mission = selected["mission"]
                    machine.mission = mission
                    print("Mission: %s (%s)" % (mission, selected["reason"]))

                # 5. navigation decides HOW to drive. It is told the current
                # behaviour so it can pick a steering law - follow the pillar,
                # track the lane, turn the corner, or aim at the parking slot.
                navigation = navigation_engine.compute_navigation(
                    vision, robot_state, machine.state.value, mission)

                # 6. the state machine decides WHAT we are doing, and returns
                # the final command - navigation's request, restrained by state
                behaviour = machine.update(vision, robot_state, navigation)

                # 7. one command, one place. Always the state machine's.
                if link:
                    link.write(("%d,%d\n" % (behaviour["speed"],
                                              behaviour["steering"])).encode())

            except (OSError, serial.SerialException) as error:
                # The link went away mid-run. Keep looping and keep retrying:
                # stopping the program would leave the robot uncommanded.
                if link:
                    link.close()
                link = None
                print("UART lost (%s), reconnecting..." % error)

            if link is None and not dry_run and time.monotonic() >= next_reconnect:
                next_reconnect = time.monotonic() + RECONNECT_INTERVAL_S
                try:
                    link = open_link(False)
                    print("UART reconnected")
                except serial.SerialException:
                    pass

            now = time.monotonic()
            frame_time = now - last_frame_at
            if frame_time > SLOW_FRAME_WARN_S:
                # The Pico stops the robot if it hears nothing for 500 ms, and we
                # only send once per frame. Say so rather than stuttering silently.
                print("Slow frame: %.0f ms - watchdog trips above %d ms"
                      % (frame_time * 1000, 500))
            fps = FPS_SMOOTHING * fps + (1 - FPS_SMOOTHING) / max(frame_time, 1e-6)
            last_frame_at = now

            if debug:
                debug_block(behaviour, navigation, robot_state,
                            vision["pillars"], fps)
            if show:
                overlay(frame, seen, behaviour)
                cv2.imshow("main", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        print("\nStopping")
    finally:
        # Whatever happened, the robot must not keep driving.
        if link:
            for _ in range(STOP_REPEATS):
                link.write(STOP_COMMAND.encode())
            link.close()
        camera.release()
        cv2.destroyAllWindows()


def selftest():
    state_machine.DEBUG = False

    # --- vision output converts to pillars ---
    seen = {
        "RED": {"cx": 410, "cy": 300, "box": (380, 250, 60, 140), "distance": 60.3},
        "GREEN": {"cx": 170, "cy": 320, "box": (150, 280, 40, 90), "distance": 140.0},
        "WALL": {"cx": 320, "cy": 200, "box": (0, 180, 640, 56), "distance": 95.0},
    }
    pillars = pillars_from_vision(seen)

    # the wall is not a pillar and must never reach navigation
    assert {p["colour"] for p in pillars} == {"RED", "GREEN"}, pillars
    red = next(p for p in pillars if p["colour"] == "RED")
    assert red["x"] == 410 and red["area"] == 60 * 140
    assert red["distance"] == 603                   # cm -> mm
    assert pillars_from_vision({}) == []

    # --- state lines from the Pico ---
    state = parse_state("S,480,230,260,900,91.3\n")
    assert state["front_distance"] == 480 and state["heading"] == 91.3
    assert parse_state("S,480,-1,260,900,91.3")["left_distance"] is None
    assert parse_state("S,480,230,260,900")["heading"] is None
    assert parse_state("S,480,230,260,900,91.3,7,1")["front_distance"] == 480
    for junk in ("", "hello", "S", "S,480", "S,a,b,c,d", "45,23", "\n"):
        assert parse_state(junk) is None, junk
    assert read_state(None, {"front_distance": 500}) == {"front_distance": 500}

    # --- one frame end to end, with no hardware anywhere ---
    clock = [0.0]

    def one_frame(machine, pillars, robot_state, step=0.1, parking=None):
        """One pass of the real loop, in the real order, on a clock we control."""
        clock[0] += step
        vision = {"pillars": pillars, "wall_distance": None,
                  "parking": parking, "walls": None}
        mission = mission_manager.current_mission()["mission"]
        machine.mission = mission
        navigation = navigation_engine.compute_navigation(
            vision, robot_state, machine.state.value, mission)
        return machine.update(vision, robot_state, navigation, now=clock[0])

    clear = {"front_distance": 900, "heading": 0.0, "speed": 40.0,
             "left_distance": 600, "right_distance": 600}
    navigation_engine.reset()
    machine = state_machine.StateMachine()
    machine.start()
    machine.update(None, clear, {"speed": 0}, now=clock[0])             # -> INITIALISE
    clock[0] += state_machine.STATE_TIMEOUT_S[state_machine.State.INITIALISE]
    behaviour = machine.update(None, clear, {"speed": 0}, now=clock[0])
    assert behaviour["state"] == "FOLLOW_COURSE"

    for _ in range(12):
        behaviour = one_frame(machine, pillars, clear)
    # red is the nearer pillar, so we line up to pass it on the right
    assert behaviour["state"] in ("APPROACH_PILLAR", "PASS_PILLAR"), behaviour
    assert behaviour["steering"] > 0, behaviour
    assert behaviour["speed"] > 0

    # once it is gone we recentre rather than driving on permanently offset
    for _ in range(12):
        behaviour = one_frame(machine, [], clear)
    assert behaviour["state"] in ("RECENTER", "FOLLOW_COURSE"), behaviour

    # --- the state machine has the final word on the command ---
    # --- the parking markers reach navigation as a slot to aim at ---
    slot = {"markers": 2, "offset": 0.5}
    machine.state = state_machine.State.ALIGN_PARKING
    parking_frame = one_frame(machine, [], clear, parking=slot)
    assert parking_frame["state"] in ("ALIGN_PARKING", "ENTER_PARKING"), parking_frame
    assert parking_frame["steering"] > 0, parking_frame     # slot is to the right
    assert parking_frame["speed"] == navigation_engine.MISSION_CRUISE_SPEED[
        navigation_engine.MISSION_PARKING], parking_frame

    machine.state = state_machine.State.FINISHED
    vision = {"pillars": pillars, "wall_distance": None,
              "parking": None, "walls": None}
    navigation = navigation_engine.compute_navigation(vision, clear)
    behaviour = machine.update(vision, clear, navigation, now=clock[0])
    assert navigation["speed"] > 0, navigation          # navigation wants to drive
    assert (behaviour["speed"], behaviour["steering"]) == (0, 0), behaviour

    # --- with no state from the Pico at all, we crawl rather than cruise ---
    navigation_engine.reset()
    blind = navigation_engine.compute_navigation({"pillars": []}, {})
    assert blind["speed"] == navigation_engine.SLOW_SPEED, blind

    # --- the mission decides what the same frame means ---
    saved = mission_manager.MISSION
    try:
        red_ahead = [{"colour": "RED", "x": 415, "area": 8200, "distance": 600}]
        outcomes = {}
        for choice in mission_manager.Mission:
            mission_manager.MISSION = choice
            navigation_engine.reset()
            machine = state_machine.StateMachine(choice.value)
            machine.start()
            # through the real entry path, not by forcing a state
            one_frame(machine, [], clear)                      # -> INITIALISE
            one_frame(machine, [], clear,
                      step=state_machine.STATE_TIMEOUT_S[state_machine.State.INITIALISE])
            outcomes[choice.value] = one_frame(machine, red_ahead, clear)["state"]

        # the obstacle round works the pillar; the open round drives past it
        assert outcomes["OBSTACLE_CHALLENGE"] == "APPROACH_PILLAR", outcomes
        assert outcomes["OPEN_CHALLENGE"] == "FOLLOW_COURSE", outcomes
        assert outcomes["PARKING"] == "SEARCH_PARKING", outcomes
    finally:
        mission_manager.MISSION = saved

    print("selftest ok  pillars %s -> state %s, speed %d, steering %+d"
          % ([p["colour"] for p in pillars], behaviour["state"],
             behaviour["speed"], behaviour["steering"]))


if __name__ == "__main__":
    if "--selftest" in sys.argv:
        selftest()
    else:
        main(dry_run="--dry-run" in sys.argv,
             show="--show" in sys.argv,
             debug=DEBUG or "--debug" in sys.argv,
             start_delay=0 if "--start-now" in sys.argv else START_DELAY_S)
