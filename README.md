<h1 align="center">Tricky Trio — WRO 2026 Future Engineers</h1>

<p align="center">
  <b>Self-Driving Cars Challenge · World Robot Olympiad 2026</b><br>
  Raspberry Pi 3 vision and planning · Raspberry Pi Pico 2 W real-time control
</p>

<!-- TODO: add your country, city and a hero photo of the finished robot here.
     A single wide photo of the car on the mat reads far better than a logo. -->

---

## Table of Contents

1. [The Team](#1-the-team)
2. [The Challenge](#2-the-challenge)
3. [Repository Structure](#3-repository-structure)
4. [Design Philosophy](#4-design-philosophy)
5. [Mobility & Mechanical Design](#5-mobility--mechanical-design)
6. [Power & Sensor Architecture](#6-power--sensor-architecture)
7. [Software Architecture](#7-software-architecture)
8. [Obstacle Strategy](#8-obstacle-strategy)
9. [Parking Strategy](#9-parking-strategy)
10. [Systems Thinking: Constraints, Trade-offs and Failures](#10-systems-thinking-constraints-trade-offs-and-failures)
11. [Testing & Validation](#11-testing--validation)
12. [Build, Flash and Run](#12-build-flash-and-run)
13. [Calibration Guide](#13-calibration-guide)
14. [Engineering Journal](#14-engineering-journal)
15. [Videos](#15-videos)
16. [Acknowledgements](#16-acknowledgements)

---

## 1. The Team

<!-- TODO: replace with your real details and add t-photos/team.jpg -->

| | Name | Age | Role | Responsibilities |
|---|---|---|---|---|
| <img src="t-photos/member1.jpg" width="90"> | *Name* | *Age* | Software Lead | Vision pipeline, navigation engine, state machine |
| <img src="t-photos/member2.jpg" width="90"> | *Name* | *Age* | Hardware Lead | Chassis, drivetrain, PCB design, wiring |
| <img src="t-photos/member3.jpg" width="90"> | *Name* | *Age* | Systems & Testing | Sensor calibration, test protocol, documentation |

**Coach:** *Name*  ·  **Country:** *Country*  ·  **Team photos:** [`t-photos/`](t-photos/)

> *One paragraph on who you are, how the team formed, and what each person learned.
> Judges read this — it is the only part of the repository that is about people.*

---

## 2. The Challenge

WRO Future Engineers asks for a fully autonomous vehicle that drives three laps of a
track whose layout is randomised for every round. There are two rounds:

| Round | Task | Points |
|---|---|---|
| **Open Challenge** | Three laps of an empty track | 30 |
| **Obstacle Challenge** | Three laps avoiding red and green traffic signs, then park | 62 |
| **Documentation** | This repository and the engineering journal | 30 |
| | | **122 total** |

The rules that shaped almost every decision in this project:

- **Red signs must be passed on their right; green signs on their left.** Getting this
  backwards is worse than not seeing the sign at all.
- **The track layout is randomised each round.** Nothing can be hard-coded — no
  memorised turn sequence, no fixed timings.
- **The parking lot is 20 cm wide and 1.5 × the robot's length.** For our 15 cm car
  that is a **22.5 cm slot** — 7.5 cm of total slack. This single number drove our
  decision to keep the chassis as short as possible.
- **Two buttons only:** one to power on, one to start the program. No other interaction.

---

## 3. Repository Structure

```
├── README.md            This document
├── src/                 All source code
│   ├── pi3/             Raspberry Pi 3 — vision, navigation, planning
│   │   ├── main.py                  Control loop coordinator
│   │   ├── mission_manager.py       Which challenge is active
│   │   ├── navigation_engine.py     Steering and speed decisions
│   │   ├── state_machine.py         Competition behaviour states
│   │   ├── uart_test_pi.py          Bench tool: UART link test
│   │   └── camera_vision/
│   │       └── vision_test.py       Camera detection pipeline
│   └── pico/            Raspberry Pi Pico 2 W — real-time hardware control
│       ├── main.py                  Flight program: command in, state out
│       ├── motionController.py      move(speed, steering) / stop()
│       ├── servo.py                 MG90S steering
│       ├── drv8833.py               Drive motor
│       ├── encoder.py               Wheel odometry
│       ├── uart_echo.py             Bench tool: UART receive test
│       ├── deploy.sh                Copy code to the board
│       └── sensors/
│           ├── sensorManager.py     One dictionary of every reading
│           ├── distance.py          4 × VL53L0X via multiplexer
│           ├── imu.py               BNO085 orientation
│           └── colour.py            TCS34725 floor colour
├── schemes/             Wiring diagrams and the master connection list
├── models/              3D-printable and laser-cut part files
├── t-photos/            Team photos
├── v-photos/            Vehicle photos, all six sides
├── video/               Link to the driving demonstration
└── other/               Datasheets, engineering journal, test logs
```

**4,071 lines of Python** across 16 modules. Every module runs its own self-test on a
laptop with no hardware attached — see [Testing & Validation](#11-testing--validation).

---

## 4. Design Philosophy

Three rules we set before writing any code, and kept to:

**1. Two brains, split by timing requirement.**
Linux is not a real-time operating system. A filesystem sync or a Wi-Fi interrupt can
stall a Python loop for tens of milliseconds — long enough for a servo to jitter or a
wheel to spin unchecked. So anything with a deadline runs on the Pico; anything that
needs to think runs on the Pi. The cost is a UART link to keep in sync, which we
accepted in exchange for control loops that never get starved by the vision stack.

**2. Every module must be testable without the robot.**
Hardware is slow to test and easy to break. Every file in this repository has a
`selftest()` that runs on a laptop. MicroPython modules guard their `machine` imports
so the maths is importable off-board:

```python
try:
    from machine import PWM, Pin, UART
except ImportError:          # laptop: the conversion below is still testable
    PWM = Pin = UART = None
```

This let us debug the steering geometry, the quaternion conversion and the entire
state machine before the chassis existed.

**3. Decisions live in tables, not in nested conditionals.**
The whole competition behaviour is one readable table. A judge — or a teammate at
2 a.m. — can read what the robot does without tracing code paths.

---

## 5. Mobility & Mechanical Design

<!-- TODO: photos of the chassis, drivetrain and steering linkage go here.
     Add v-photos/ images and reference them. -->

### Chassis

| Property | Value | Why |
|---|---|---|
| Length | **15.0 cm** | Parking slot is 1.5 × length = 22.5 cm. Every centimetre of car costs 1.5 cm of slot, so short is safe. |
| Width | **10.5 cm** | Lane is 100 cm wide; narrow enough to pass a sign with margin either side. |
| Drive | Single motor, rear axle | Rule 11.3/11.5 require the drive wheels to be physically connected. One motor through a gearbox satisfies this and removes differential-speed error. |
| Steering | Ackermann, front axle, single servo | Required by the category focus on non-differential kinematics. |

### Drive Motor — GA12-N20

Chosen for three reasons, in order of weight:

1. **Built-in encoder.** The deciding factor. It lets us measure actual wheel rotation
   instead of assuming PWM duty maps to speed. Without odometry we cannot detect a
   stalled robot, and stall detection is what triggers our recovery behaviour.
2. **Integrated gearbox** in a package small enough for a 15 cm chassis.
3. **6 V operation** matching the rest of the low-voltage system.

**Torque and speed reasoning.** Theoretical top speed follows `v = πDN / 60` where D is
wheel diameter and N is motor RPM at the output shaft. For our 33 mm wheel at
300 RPM this gives ≈ 0.52 m/s. That number ignores load, rolling resistance and
battery sag, so we treat it as a ceiling, not a target.

<!-- TODO: fill in after bench testing. Do NOT estimate these.
| Measurement | Value |
|---|---|
| Measured top speed, unloaded | ___ m/s |
| Measured top speed, on the mat | ___ m/s |
| Minimum PWM that actually moves the robot | ___ % |
| Time to travel 1 m from standstill | ___ s |
-->

**Why not two motors, one per side?** Explicitly forbidden by rule 11.5, and it would
have introduced differential error that our single-encoder odometry cannot see.

### Steering — MG90S

<!-- TODO: photo of the steering linkage -->

Metal-geared, so the gear train survives the shock of clipping a wall — the failure
mode that kills plastic-geared servos. Driven at 50 Hz with a pulse width computed
from a calibrated centre:

```python
pulse_us = CENTRE_US + angle * US_PER_DEGREE * STEER_DIRECTION
```

Three deliberate safety features in that one line:

- `CENTRE_US` is calibrated per build, because no two linkages are neutral at 1500 µs.
- `STEER_DIRECTION` flips the sign if the servo is mounted mirrored — we fix mounting
  errors with a constant, never by negating at the call site, so the sign convention
  stays identical on both boards.
- The angle is clamped to ±30° **before** conversion, and the resulting pulse is clamped
  again to 1000–2000 µs. A runaway command therefore cannot drive the servo into the
  steering linkage and stall it. A stalled MG90S draws over an amp and destroys its own
  gears in about a minute.

**Iteration:** our first version clamped only the pulse width. Clamping the angle first
means the reported steering angle and the physical angle always agree, which matters
because the state machine logs the commanded angle for debugging.

---

## 6. Power & Sensor Architecture

**Wiring diagrams:** [`schemes/`](schemes/) — see [`schemes/README.md`](schemes/README.md)
for the complete master connection list, and `circuit_image.svg` for the schematic.

### Power Budget

| Consumer | Typical | Peak | Notes |
|---|---|---|---|
| Raspberry Pi 3 | ~700 mA | ~2.5 A | Peaks during vision processing |
| MG90S servo | ~200 mA | ~1.5 A | Peak at stall / full lock |
| GA12-N20 motor | ~150 mA | ~800 mA | Peak at stall |
| Pico 2 W + sensors | ~150 mA | ~200 mA | 6 I²C devices on 3V3 |
| **Total** | **~1.2 A** | **~5 A** | |

A single Mini560 buck converter feeds the 5 V rail. The peak figure is what matters:
the Pi browns out and reboots if the rail sags, and it sags exactly when the servo
slams to full lock — which is precisely when the robot is doing something difficult.
Mitigation: a 100 µF electrolytic across the DRV8833's VM/GND at the driver, and a
0.1 µF ceramic across the motor terminals.

### The 3.3 V / 5 V Boundary — A Fault We Caught in Review

Our first connection list put the **N20 encoder on the 5 V rail**. An N20 magnetic
encoder outputs at whatever voltage supplies it, so its A/B channels would have swung
to 5 V straight into GP12/GP13. **The RP2350 is not 5 V tolerant** (absolute maximum
3.3 V + 0.3 V). This would have worked on the bench and killed the microcontroller
days or weeks later — the worst kind of fault, because it looks like a software bug.

The encoder now runs from the Pico's 3V3 rail alongside the I²C sensors. It draws a
few milliamps. **Lesson recorded in our journal: trace the logic-level path of every
sensor output, not just its supply.**

### Sensor Selection and Placement

<!-- TODO: a top-down diagram showing sensor positions and angles. This is the single
     highest-value diagram in the repository for Criterion 2. -->

| Sensor | Qty | Placement | Why here |
|---|---|---|---|
| Pi Camera Module 3 | 1 | Front, elevated | Height gives earlier sight of signs; the only sensor that can tell red from green |
| VL53L0X ToF | 1 | Front, 0° | Corner confirmation and collision avoidance |
| VL53L0X ToF | 2 | **±45°**, flanking the camera | Lane position and corner geometry |
| VL53L0X ToF | 1 | Rear, 180° | Parking and reversing clearance |
| BNO085 IMU | 1 | Centre, flat | Heading for turn completion and straight-line hold |
| TCS34725 | 1 | Underside, 5–10 mm above mat | Corner line detection |

**Why ±45° and not 90°?** This is the placement decision we spent longest on. Sensors
pointing straight out to the sides tell you the lane width but see a corner only once
you are level with it — too late to plan a turn. At 45° they see the forward diagonals,
so an approaching inner wall shows up while there is still room to react. The cost is
that a reading of D millimetres is **not** lateral clearance; it is D·cos45 ≈ 0.71·D
ahead and the same to the side. That conversion is documented at the top of
[`distance.py`](src/pico/sensors/distance.py) because anything that treats a diagonal
reading as lateral clearance will drive into a wall.

Lane position falls out of comparing the two diagonals, with no need to know the lane
width at all:

```python
offset = (right_mm - left_mm) / (right_mm + left_mm)   # +1 hard right, -1 hard left
```

**Failure-point consideration:** a VL53L0X reports ≈ 8190 mm when it sees nothing. Fed
into that formula this looks like an enormously wide lane. Readings above 2000 mm are
therefore discarded rather than believed.

### Why a Multiplexer

Every VL53L0X ships with the same fixed I²C address, **0x29** — and so does the
TCS34725. Two on one bus and both answer at once; nothing in software can separate
them. There are two ways out: pulse each sensor's XSHUT pin at boot to bring them up
one at a time and reassign addresses, or switch the bus. We chose the **TCA9548A
multiplexer** because it needs no extra GPIO per sensor and no boot-time sequencing
that can fail silently.

| TCA channel | Device |
|---|---|
| 0 | VL53L0X front |
| 1 | TCS34725 colour |
| 2 | BNO085 IMU |
| 3 / 4 / 5 | VL53L0X left / right / rear |

The cost is that reads are sequential: switch channel, talk, switch, talk. A full sweep
of four ToF sensors costs ≈ 132 ms, which is why the Pico's loop applies commands every
20 ms but sweeps sensors only every 150 ms. A naive loop would have delayed every motor
command by the length of a sensor sweep.

---

## 7. Software Architecture

### The Loop

```
   ┌──────────────── Raspberry Pi 3 ────────────────┐   ┌──── Pico 2 W ────┐
   │                                                │   │                  │
   │  camera ─► vision ─► pillars, wall, slot ──┐   │   │  motionController│
   │                                            ▼   │   │    ├─ servo      │
   │  robot_state ─► mission ─► navigation ─────┐   │   │    └─ drv8833    │
   │                            (HOW to drive)  ▼   │   │                  │
   │                          state machine ────────┼───┼─► "45,23\n"      │
   │                          (WHAT we are doing)   │   │                  │
   │                                                │   │  sensorManager   │
   │  robot_state ◄─────────────────────────────────┼───┼── "S,480,230,…"  │
   └────────────────────────────────────────────────┘   └──────────────────┘
```

Each frame, in this exact order:

| # | Step | Module |
|---|---|---|
| 1 | Capture a frame | `main.py` |
| 2 | Detect signs, wall and parking markers | `vision_test.py` |
| 3 | Read the newest robot state from the Pico | `main.py` |
| 4 | Ask which challenge is active | `mission_manager.py` |
| 5 | Decide **how** to drive — speed and steering | `navigation_engine.py` |
| 6 | Decide **what** we are doing — and restrain the command | `state_machine.py` |
| 7 | Send the final command over UART | `main.py` |

**The separation that makes this work:** navigation decides *how*, the state machine
decides *what*. The state machine may only **restrain** navigation's request, never
invent one:

```python
STATE_SPEED_CAP = {
    State.FOLLOW_COURSE: None,      # take whatever navigation asked for
    State.APPROACH_PILLAR: None,    # avoiding IS navigation's job
    State.TURN_CORNER: 35,          # corners are tighter than the straights
    State.RECOVERY: 30,             # crawl while working out where we are
    State.FINISHED: 0,
}
```

Only `main.py` may write to the serial port, and it writes only the state machine's
output — so there is exactly one place a command can reach the wheels.

### The State Machine

**12 states · 17 events · 30 transitions · 3 missions.**

The critical design choice: **detecting** what happened is separate from **deciding**
what to do about it. `detect_events()` is the only function that knows about
millimetres and degrees; the transition table knows about behaviour and contains no
numbers at all.

```python
State.FOLLOW_COURSE: (
    (Event.RUN_COMPLETE,      State.SEARCH_PARKING),
    (Event.RECOVERY_REQUIRED, State.RECOVERY),
    (Event.CORNER_DETECTED,   State.TURN_CORNER),     # a wall beats a sign
    (Event.RED_PILLAR,        State.APPROACH_PILLAR),
    (Event.GREEN_PILLAR,      State.APPROACH_PILLAR),
),
```

Row order is priority — "a wall beats a sign" is a line you can point at, not an
emergent consequence of which `if` came first. When we improve corner detection, we
change how `CORNER_DETECTED` is produced and the behaviour table never learns that
anything changed.

Events fire whenever they are true, regardless of state; the table decides which
matter. In the Open Challenge the robot still *sees* a red sign — `RED_PILLAR` appears
in the event list — and correctly ignores it, because that mission has no
`APPROACH_PILLAR` state.

**States:** `WAIT_FOR_START · INITIALISE · FOLLOW_COURSE · APPROACH_PILLAR ·
PASS_PILLAR · RECENTER · TURN_CORNER · SEARCH_PARKING · ALIGN_PARKING ·
ENTER_PARKING · RECOVERY · FINISHED`

### Vision Pipeline

Cost per frame is **O(N)** in pixels and **O(C)** in contours, and N is fixed by the
processing width rather than the camera resolution. The pipeline measures **0.32 ms
per frame** on a development laptop, so the camera's frame rate is the ceiling, not
the code.

| Stage | Detail |
|---|---|
| ROI crop | Top 35 % discarded — only ceiling and lights live up there |
| Downscale | Detection runs at 320 px wide; drawing stays full resolution |
| HSV threshold | Red needs two ranges because hue wraps at 0 |
| Morphology | Open to kill speckle, close to fill glare holes |
| Contour filter | Area, aspect ratio **and fill ratio** |

**Why fill ratio matters.** Largest-contour-wins locks onto a red jacket in the
audience or a stripe of glare on the mat. A real traffic sign fills its bounding box;
scattered reflections do not. A candidate must pass area **and** aspect ratio
(0.15–2.00, signs are taller than wide) **and** fill ratio (> 0.45) before it is
believed. Our self-test includes a wide red stripe with the largest area in the frame
and asserts the detector skips it in favour of the actual sign.

**Distance from pixel height.** Signs are a known 10 cm tall, so
`distance = real_height × focal_px / pixel_height`. This gives us range from a single
camera with no stereo rig.

**Saturation and value floors are deliberately low** (S ≥ 90, V ≥ 40 for red). Our
first version used S ≥ 110, V ≥ 70 and worked perfectly with one sign — then silently
dropped the second one whenever it sat further from the lights. Two signs on a mat are
never lit equally.

---

## 8. Obstacle Strategy

Red on the right, green on the left. The elegant part is that this is **one line**, not
a tree of cases:

```python
PASS_TARGET = {COLOUR_RED: -0.5, COLOUR_GREEN: +0.5}
steering = (offset - PASS_TARGET[colour]) * STEERING_GAIN
```

Each colour has a **target position in the frame**. Passing a red sign on its right
means the sign ends up on your left — so you want it at −0.5, and you steer by however
far it is from there. Green mirrors it.

The behaviour this produces, without a single special case:

| Red sign at | Frame offset | Steering |
|---|---|---|
| Far left | −0.91 | **0°** — already clear |
| Left of centre | −0.53 | 0° — just cleared |
| **Centred** | **0.00** | **+15°** |
| Right of centre | +0.30 | +24° |
| Far right | +0.56 | +30° — worst case, hard over |

One safety clamp: a red sign may only ever produce right steering. Without it, a sign
already cleared to the left would produce a small correction back *toward* the thing
you just avoided — which is how you clip one.

### Choosing Between Two Signs

Both colours are often visible at once — a red one close and a green one further down
the track. The **nearer** sign is the one about to be hit, so that is the one we steer
around; the far one is handled on later frames once it becomes the near one.

```python
if all(p.get("distance") is not None for p in candidates):
    return min(candidates, key=lambda p: p["distance"])
return max(candidates, key=lambda p: p["area"])     # fallback: bigger is nearer
```

### The Four-Phase Pass

`FOLLOW_COURSE → APPROACH_PILLAR → PASS_PILLAR → RECENTER → FOLLOW_COURSE`

**`RECENTER` is the phase that stops the robot living permanently offset.** Our first
implementation steered around a sign and simply carried on, drifting a little further
from the lane centre with each obstacle until it clipped a wall three signs later.
Now, once the sign has been out of view for 0.5 s, steering ignores signs entirely and
tracks the lane centre until the offset is inside a dead zone.

Note the deliberate asymmetry: `PASS_PILLAR` has **no** route to `RECOVERY`. Losing
sight of a sign you are squeezing past is exactly what success looks like, so both
exits lead to recentring.

### Edge Cases We Handle

| Case | Handling |
|---|---|
| Sign flickers out for one frame | 0.5 s confirmation before the pass is declared over |
| Two signs in view | Nearest by measured distance wins; frame order is irrelevant |
| Sign appears mid-corner | Corner outranks sign — the table row order says so |
| Sign smaller than `MIN_PILLAR_AREA` | Treated as noise, not a sign |
| Sign jitters around frame centre | Dead zone of ±0.06 snaps it to exactly centred |
| Sensor returns `None` | Speed drops to slow rather than cruising blind |
| Nothing detected at all | Lane following from the ±45° diagonals |

---

## 9. Parking Strategy

The parking lot is bounded by **two magenta elements, 20 × 2 × 10 cm**. The slot is
20 cm wide and 1.5 × the robot's length — for our 15 cm car, **22.5 cm**.

### Magenta vs Red — A Collision We Had to Resolve

Magenta sits immediately below red on the hue circle. Our original upper red band
started at hue 165 and **swallowed the magenta markers entirely** — the robot saw the
parking lot as a giant traffic sign and tried to pass it on the right. Red now starts
at 172, which costs nothing because red's main band is 0–10.

```python
"RED":  [((0, 90, 40), (10, 255, 255)), ((172, 90, 40), (180, 255, 255))],
MAGENTA_RANGE = [((140, 70, 60), (170, 255, 255))]
```

Our self-test puts a red sign and both magenta markers in the same synthetic frame and
asserts neither colour steals the other.

### Aiming at the Gap, Not the Markers

`parking_gap()` returns the midpoint between the markers' **inner edges** — the slot
itself, not either marker. Verified in the self-test: true midpoint 310 px, reported
311 px.

**Both markers must be visible** before `PARKING_VISIBLE` fires. You cannot aim at a
gap you can only half see.

`SEARCH_PARKING → ALIGN_PARKING → ENTER_PARKING → FINISHED`

### Two Bugs This Sequence Taught Us

**1. Watching the wrong sensor.** Our first version stopped when the *rear* ToF read
close. But the camera faces forward, so the slot is only visible while driving at it —
the robot enters nose-first and the rear sensor is pointed back at the open mat it came
from. It never triggered, and the robot stopped on a timeout instead of on arrival.

**2. The stop was physically unreachable.** Navigation emergency-stops at 150 mm, but
`PARKED` triggered at 120 mm — so the robot froze before it could ever reach its own
trigger. Fixed with a **speed floor** for `ENTER_PARKING`, the same pattern we already
needed for corners:

```
t=2.6  ENTER_PARKING  speed 20  front 200 mm   past the emergency-stop line
t=3.1  ENTER_PARKING  speed 15  front 140 mm   creeping
t=3.6  FINISHED       speed  0  front  95 mm   reason: Parked
```

Before the fix that last line read `reason: Timed out`. The self-test now asserts
`PARKING_STOP_MM < STOP_ENTER_MM` so the ordering cannot silently break again.

---

## 10. Systems Thinking: Constraints, Trade-offs and Failures

### Constraints We Designed Around

| Constraint | Source | Consequence |
|---|---|---|
| Slot = 1.5 × robot length | Rule | Chassis kept to 15 cm; every cm costs 1.5 cm of slot |
| Drive wheels physically connected | Rule 11.3 / 11.5 | Single motor + gearbox, not one per side |
| Two buttons only | Rule 9.10 / 9.11 | No laptop interaction at the start line |
| Track randomised each round | Rule | Nothing hard-coded; all behaviour sensor-driven |
| VL53L0X and TCS34725 share address 0x29 | Hardware | Multiplexer required |
| RP2350 not 5 V tolerant | Hardware | All sensor logic on 3V3 |
| Linux is not real-time | Platform | Two-controller architecture |

### Key Trade-offs

| Decision | Chosen | Rejected | Reasoning |
|---|---|---|---|
| Compute | Pi 3 **+** Pico | Pi alone | Non-real-time OS would starve the control loop |
| Sensor addressing | TCA9548A mux | XSHUT re-addressing | No boot sequencing that can fail silently |
| Side sensors | ±45° diagonals | 90° lateral | Sees corners early enough to plan |
| Control law | Proportional | PID | Tunable at a competition by one person under pressure |
| Sign selection | Nearest | Largest contour | Correct when two signs are visible |
| Detection width | 320 px | Full 640 px | ~5× fewer pixels; camera becomes the bottleneck, not the code |

**Why no PID?** We can implement one. We chose not to. A PID controller has three
interacting constants, and a competition venue is the worst possible place to tune
three interacting constants under time pressure. Our steering has **one gain and one
rate limit**, both of which a team member can reason about between rounds. Determinism
and tunability beat theoretical optimality when you get two attempts.

### Risk Analysis

| Risk | Impact | Mitigation |
|---|---|---|
| Pi crashes mid-run | Robot drives on uncommanded | Pico watchdog: no command for 500 ms → `stop()` |
| UART disconnects | Total loss of control | Reconnect loop; robot stops, program keeps retrying |
| One sensor fails | Cascade failure | Every sensor returns `None`; one dead ToF costs one reading |
| Driver file missing on Pico | All sensors dead | Driver imports isolated; each reports itself by name |
| Voltage sag at full lock | Pi reboots mid-run | Decoupling at the driver; peak-current budget |
| Robot wedged against a wall | Run over | Stall detection via encoder → `RECOVERY` |
| Corner never completes | Deadlock | Timeout → `RECOVERY`; speed floor prevents the freeze |
| Mini-UART clock drift (Pi 3) | Corrupt bytes under load | `dtoverlay=disable-bt` forces the stable PL011 |

**The Pi 3 UART trap deserves its own note.** On a Pi 3, `/dev/serial0` defaults to the
mini-UART, whose baud rate is derived from the VPU core clock — which changes with CPU
load. It works perfectly at idle and corrupts bytes the moment the vision code loads
the processor. That is an intermittent fault that only appears when the robot is
working hardest. `enable_uart=1` and `dtoverlay=disable-bt` in `config.txt` move it to
the stable PL011; `ls -l /dev/serial0` must resolve to `ttyAMA0`, not `ttyS0`.

### Iteration Log

| # | Problem | Root cause | Fix |
|---|---|---|---|
| 1 | Only one of two signs detected | S/V floors too high; the dimmer sign fell below them | Floors lowered to S ≥ 90, V ≥ 40 |
| 2 | Robot clipped signs it had already passed | Counter-steer back toward a cleared sign | One-directional clamp per colour |
| 3 | Drifted further off-centre with each sign | No return-to-lane behaviour | `RECENTER` state added |
| 4 | Robot gave up mid-corner | Emergency stop fired at the wall being turned away from; stopped robot stops turning | Speed floor for `TURN_CORNER` |
| 5 | Parking finished on a timer | Stop watched the rear sensor during a nose-first entry | Front sensor, gated to parking states |
| 6 | Parking stop unreachable | Trigger sat inside the emergency-stop zone | Speed floor + threshold below it, assertion added |
| 7 | Encoder would have destroyed the Pico | 5 V logic into a 3.3 V pin | Moved to the 3V3 rail |
| 8 | Magenta read as red | Hue bands overlapped | Red band narrowed to 172+ |
| 9 | Mode flapped between cruise and slow | Single threshold with noisy readings | Hysteresis: separate enter/exit thresholds |
| 10 | Stall detection never fired | `x or now` treats timestamp 0.0 as unset | Explicit `is None` |

---

## 11. Testing & Validation

**Every module self-tests on a laptop with no hardware attached.**

```
$ python state_machine.py --selftest
selftest ok  12 states, 17 events, 30 transitions, 3 missions
```

| Module | What its self-test proves |
|---|---|
| `vision_test.py` | Both colours detected in one frame; noise and wrong-shaped blobs rejected; magenta not read as red; gap midpoint within 1 px |
| `navigation_engine.py` | Pass side never wrong anywhere in the frame; steering never exceeds the servo limit; rate limit holds; hysteresis in both directions |
| `state_machine.py` | Every state has a transition row; no duplicate events; no mission can deadlock; every substitute is reachable |
| `main.py` (Pi) | Vision output converts to the navigation format; the wall never reaches navigation; state machine has the final word |
| `main.py` (Pico) | **Wire format verified against the Pi's actual parser**, not a copy of it |
| `servo.py` | Clamping symmetric; pulse always inside safe limits |
| `drv8833.py` | Negative speed means stop, not reverse; duty monotonic |
| `encoder.py` | One turn = one circumference; reverse counts negative |
| `distance.py` | One bit per mux channel; IMU and colour channels never touched |
| `imu.py` | Quaternion → Euler correct at 0°, 90°, 180°; gimbal-lock input does not raise |
| `colour.py` | Same surface at half brightness classifies identically |

**Structural assertions** are the ones we value most, because they catch a half-finished
edit rather than a wrong number:

```python
assert set(TRANSITIONS) == set(State), "a state has no transition row"
assert PARKING_STOP_MM < navigation_engine.STOP_ENTER_MM
```

<!-- TODO — REPLACE WITH REAL MEASUREMENTS. Do not estimate these.
### Field Test Results
| Test | Runs | Success | Notes |
|---|---|---|---|
| Open Challenge, 3 laps | | | |
| Obstacle Challenge, 3 laps | | | |
| Parking, both directions | | | |
| Sign pass, red / green | | | |
-->

> **Current status:** the software stack is complete and passes 15 module self-tests.
> Field testing on the physical robot is in progress; measured results will be added
> here as they are collected.

---

## 12. Build, Flash and Run

### Raspberry Pi 3

```bash
sudo apt install python3-opencv python3-pip
pip install pyserial

# Pi 3 only — move the UART off the unstable mini-UART
sudo nano /boot/firmware/config.txt      # add: enable_uart=1
                                         #      dtoverlay=disable-bt
sudo systemctl disable hciuart
sudo raspi-config                        # Serial: login shell NO, hardware YES
sudo reboot
ls -l /dev/serial0                       # must point at ttyAMA0
```

### Raspberry Pi Pico 2 W

```bash
# 1. Flash MicroPython — the RP2350 build, not the RP2040 "Pico W" one
#    micropython.org/download/RPI_PICO2_W  → hold BOOTSEL, drag the .uf2

# 2. Third-party drivers into src/pico/drivers/
#    vl53l0x.py · bno08x.py · tcs34725.py

# 3. Deploy
cd src/pico && ./deploy.sh --drivers
```

### Run

```bash
cd src/pi3
python3 main.py                # competition
python3 main.py --dry-run --show   # laptop: webcam + decisions, no UART
python3 main.py --debug        # one status block per frame
```

---

## 13. Calibration Guide

Values that **must** be measured on your build — the software cannot guess them:

| Constant | File | How to measure |
|---|---|---|
| `GEAR_RATIO` | `encoder.py` | Mark the tyre, turn 10 revolutions, divide the pulse change by 10 |
| `WHEEL_DIAMETER_MM` | `encoder.py` | Calipers across the tyre **with the robot's weight on it** |
| `CENTRE_US` | `servo.py` | Send `40,0`, adjust until the wheels are dead straight |
| `MAX_STEER` | `servo.py` | Reduce until full lock no longer makes the servo strain |
| `CAMERA_HFOV_DEG` | `vision_test.py` | Place a sign at a measured 50 cm, adjust until the readout agrees |
| `ROI_TOP` | `vision_test.py` | Set once the camera is mounted |
| `COLOUR_RANGES` | `vision_test.py` | Press **M** for mask view; widen until both signs are solid white |
| `MAGENTA_RANGE` | `vision_test.py` | Same, on the real markers under competition lighting |

Direction flags — set by observation, never by rewiring: `STEER_DIRECTION` (servo turns
the wrong way) and `ENCODER_DIRECTION` (forward counts down). If the motor spins
backwards, swap the two motor wires rather than negating in software.

---

## 14. Engineering Journal

<!-- TODO: add the PDF to other/ and link it here -->
The full engineering journal — weekly progress, sketches, failed prototypes and test
logs — is in [`other/engineering-journal.pdf`](other/).

---

## 15. Videos

<!-- TODO: replace with real YouTube links. Each must show ≥30 s of autonomous driving. -->

| Round | Link |
|---|---|
| Open Challenge | *YouTube URL* |
| Obstacle Challenge | *YouTube URL* |

Also recorded in [`video/video.md`](video/).

---

## 16. Acknowledgements

<!-- TODO: your school, mentors, sponsors. -->

Open-source work we build on: **OpenCV**, **MicroPython**, and the MicroPython drivers
for VL53L0X, BNO08x and TCS34725. We also studied the public repositories of previous
WRO Future Engineers teams — the category's culture of publishing work openly is the
reason this repository exists in the form it does.

---

<p align="center">
  <b>Team Tricky Trio</b> · WRO 2026 Future Engineers<br>
  <i>Every constant in this repository is either measured or explained.</i>
</p>
