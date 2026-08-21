# Raspberry Pi 3 Source

Python 3 with OpenCV. Everything here needs an operating system: the camera, the vision
pipeline, and the decisions about where to drive.

## Files

| File | What it does |
|---|---|
| `main.py` | The control loop. Coordinates the other modules and owns the serial link. Contains no decision logic of its own. |
| `camera_vision/vision_test.py` | Detects red and green traffic signs, the black wall and the magenta parking markers, and estimates the distance to each. |
| `navigation_engine.py` | Decides how to drive. Separate steering laws for lane following, sign passing, cornering and parking. |
| `state_machine.py` | Decides what the robot is currently doing, across twelve states and three missions. Has the final say on the command sent to the Pico. |
| `mission_manager.py` | Reports which challenge is active. |
| `uart_test_pi.py` | Bench tool for testing the serial link on its own. |

## The loop

Each camera frame runs the same seven steps in the same order: capture, detect, read the
robot's state from the Pico, check the mission, decide how to drive, decide what we are
doing, send one command.

The separation that makes this work is that the navigation engine decides *how* to drive
while the state machine decides *what* we are doing, and the state machine may only
restrain navigation's request rather than invent one of its own. Only `main.py` writes
to the serial port, so there is exactly one place a command can reach the wheels.

## Running it

```
python3 main.py                    competition
python3 main.py --dry-run --show   laptop: webcam and decisions, no Pico
python3 main.py --debug            one status block per frame
```

On a Raspberry Pi 3 the serial port needs moving off the mini UART first, otherwise it
corrupts bytes under processor load. See [`schemes/README.md`](../../schemes/README.md).

## Testing

```
python3 navigation_engine.py --selftest
python3 state_machine.py --selftest
```

Every module has one, and they all run with no hardware attached.
