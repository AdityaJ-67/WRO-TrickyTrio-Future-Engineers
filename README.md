<h1 align="center">Tricky Trio &middot; WRO 2026 Future Engineers</h1>


<!-- TODO: add a wide hero photo of the finished robot on the mat -->

---

## Table of Contents

1. [The Team](#1-the-team)
2. [The Challenge](#2-the-challenge)
3. [Design Philosophy](#3-design-philosophy)
4. [Components](#4-components)
5. [Mobility and Mechanical Design](#5-mobility-and-mechanical-design)
6. [Power and Sensor Architecture](#6-power-and-sensor-architecture)
7. [Software Architecture](#7-software-architecture)
8. [Obstacle Strategy](#8-obstacle-strategy)
9. [Parking Strategy](#9-parking-strategy)
10. [Systems Thinking](#10-systems-thinking)
11. [Testing and Validation](#11-testing-and-validation)
12. [Engineering Journal](#12-engineering-journal)
13. [Videos](#13-videos)
14. [Acknowledgements](#14-acknowledgements)

---

## 1. The Team

<!-- TODO: replace the names and add the photos to t-photos/ -->

| | Who we are |
|---|---|
| <img src="t-photos/member1.jpg" width="120"> | **Name**<br><br>*One short paragraph. Who you are, what you were most interested in on this project, what you took on, and something about you that is not robotics. Judges read this section more carefully than any other, because it is the only part of the repository that is about people rather than parts.* |
| <img src="t-photos/member2.jpg" width="120"> | **Name**<br><br>*One short paragraph.* |
| <img src="t-photos/member3.jpg" width="120"> | **Name**<br><br>*One short paragraph.* |
| <img src="t-photos/coach.jpg" width="120"> | **Name** (Coach)<br><br>*One short paragraph on how the coach supported the team.* |

**Country:** *fill in* &nbsp;&nbsp;&middot;&nbsp;&nbsp; **Team photos:** [`t-photos/`](t-photos/)

### What We Learned

<!-- TODO: write this last, once the robot has run. Three or four paragraphs.
     Good things to cover:
       - the hardest problem you solved and how long it took
       - something you were confident about that turned out to be wrong
       - a skill each member has now that they did not have at the start
       - what you would do differently if you started again tomorrow
     This is where a judge decides whether the engineering process was real. -->

*To be written.*

---

## 2. The Challenge

WRO Future Engineers asks for a fully autonomous vehicle that drives three laps of a
track whose layout is randomised for every round. There are two rounds plus the
documentation you are reading now.

| Round | Task | Points |
|---|---|---|
| Open Challenge | Three laps of an empty track | 30 |
| Obstacle Challenge | Three laps avoiding red and green traffic signs, then park | 62 |
| Documentation | This repository and the engineering journal | 30 |
| | | **122 total** |

The rules that shaped almost every decision in this project:

- **Red signs must be passed on their right, green signs on their left.** Getting this
  backwards is worse than not seeing the sign at all.
- **The track layout is randomised each round.** Nothing can be hard coded. No
  memorised turn sequence, no fixed timings.
- **The parking lot is 20 cm wide and 1.5 times the robot's length.** For our 15 cm car
  that is a 22.5 cm slot, leaving 7.5 cm of total slack. This single number drove our
  decision to keep the chassis as short as possible.
- **Two buttons only.** One to power on, one to start the program. No other interaction
  is permitted.

---

## 3. Design Philosophy

Three rules we set before writing any code, and kept to.

**Two brains, split by timing requirement.**
Linux is not a real time operating system. A filesystem sync or a Wi-Fi interrupt can
stall a Python loop for tens of milliseconds, which is long enough for a servo to
jitter or a wheel to spin unchecked. So anything with a deadline runs on the Pico, and
anything that needs to think runs on the Pi. The cost is a UART link to keep the two in
sync, which we accepted in exchange for control loops that never get starved by the
vision stack.

**Every module must be testable without the robot.**
Hardware is slow to test and easy to break. Every file in this repository has a self
test that runs on a laptop with nothing plugged in. The MicroPython modules are written
so that their hardware imports can fail harmlessly off the board, leaving the maths
importable. This let us debug the steering geometry, the quaternion conversion and the
entire competition state machine before the chassis existed.

**Decisions live in tables, not in nested conditionals.**
The whole competition behaviour is one readable table of states and events. A judge, or
a teammate at two in the morning, can read what the robot does without tracing code
paths through a tree of if statements.

---

## 4. Components

<!-- TODO for every component below:
       1. drop a photo into v-photos/ (or other/) and fix the image path
       2. fill in the specification values
     Leave a row out rather than guessing a number. -->

### Raspberry Pi 3 Model B

<img src="v-photos/pi3.jpg" width="220">

Runs the camera, the vision pipeline, the navigation engine and the competition state
machine. Chosen because computer vision needs an operating system, a filesystem and
enough RAM to hold video frames, none of which a microcontroller has.

| Specification | Value |
|---|---|
| Processor | |
| RAM | |
| Operating voltage | |
| Typical current draw | |
| Peak current draw | |
| Operating system | |

### Raspberry Pi Pico 2 W

<img src="v-photos/pico.jpg" width="220">

Runs the motor, the servo, the encoder and all six sensors. Chosen for deterministic
timing: it does exactly one thing per loop and never pauses to do housekeeping.

| Specification | Value |
|---|---|
| Microcontroller | |
| Clock speed | |
| Operating voltage | |
| Logic level | |
| Typical current draw | |
| Flash memory | |

### Camera Module 3

<img src="v-photos/camera.jpg" width="220">

The only sensor that can tell red from green, and the only one that sees far enough
ahead to plan a manoeuvre rather than react to one.

| Specification | Value |
|---|---|
| Sensor | |
| Resolution used | |
| Frame rate used | |
| Horizontal field of view | |
| Mounting height above mat | |
| Mounting angle | |

### GA12-N20 Gear Motor with Encoder

<img src="v-photos/motor.jpg" width="220">

Single drive motor on the rear axle. The built in encoder was the deciding factor, as
explained in section 5.

| Specification | Value |
|---|---|
| Rated voltage | |
| No load speed | |
| Gear ratio | |
| Stall torque | |
| Stall current | |
| Encoder pulses per motor revolution | |
| Pulses per wheel revolution | |

### MG90S Servo

<img src="v-photos/servo.jpg" width="220">

Steers the front axle through an Ackermann linkage. Metal geared so the gear train
survives clipping a wall.

| Specification | Value |
|---|---|
| Operating voltage | |
| Stall torque | |
| Operating speed | |
| Gear material | |
| Pulse width range used | |
| Steering range achieved | |

### DRV8833 Motor Driver

<img src="v-photos/drv8833.jpg" width="220">

Dual H bridge driving the single motor. Chosen over the larger L298N for its smaller
footprint and much better efficiency at low voltage.

| Specification | Value |
|---|---|
| Motor supply voltage range | |
| Continuous current per channel | |
| Peak current per channel | |
| PWM frequency used | |
| Logic voltage | |

### VL53L0X Time of Flight Sensor

<img src="v-photos/vl53l0x.jpg" width="220">

Four of these, at 0 degrees, plus and minus 45 degrees, and 180 degrees. Placement
reasoning is in section 6.

| Specification | Value |
|---|---|
| Measuring range | |
| Accuracy | |
| Field of view | |
| Measurement time (timing budget) | |
| Operating voltage | |
| I2C address | |

### TCA9548A I2C Multiplexer

<img src="v-photos/tca9548a.jpg" width="220">

Lets six devices that share the same I2C address coexist on one bus. Without it the
four distance sensors and the colour sensor cannot be used together.

| Specification | Value |
|---|---|
| Number of channels | |
| Operating voltage | |
| I2C address | |
| Bus speed used | |

### BNO085 IMU

<img src="v-photos/bno085.jpg" width="220">

Provides heading, which is what tells us a 90 degree turn is complete. Runs sensor
fusion on its own processor rather than handing us raw accelerometer and gyroscope
values to filter ourselves.

| Specification | Value |
|---|---|
| Sensors on board | |
| Output report used | |
| Update rate | |
| Heading accuracy | |
| Operating voltage | |
| I2C address | |

### TCS34725 Colour Sensor

<img src="v-photos/tcs34725.jpg" width="220">

Reads the mat directly below the car. Its built in infrared filter is why it stays
stable while four infrared distance sensors are firing beside it.

| Specification | Value |
|---|---|
| Output channels | |
| Integration time used | |
| Gain used | |
| Mounting height above mat | |
| Operating voltage | |
| I2C address | |

### Mini560 Buck Converter

<img src="v-photos/mini560.jpg" width="220">

Steps the battery down to the 5 V rail that feeds the Pi, the Pico and the servo.

| Specification | Value |
|---|---|
| Input voltage range | |
| Output voltage | |
| Continuous output current | |
| Peak output current | |
| Efficiency | |

### Battery

<img src="v-photos/battery.jpg" width="220">

| Specification | Value |
|---|---|
| Chemistry | |
| Nominal voltage | |
| Capacity | |
| Maximum discharge current | |
| Mass | |
| Measured run time | |

### Chassis and Wheels

<img src="v-photos/chassis.jpg" width="220">

| Specification | Value |
|---|---|
| Overall length | 15.0 cm |
| Overall width | 10.5 cm |
| Overall height | |
| Total mass | |
| Wheel diameter | |
| Wheelbase | |
| Track width | |
| Ground clearance | |
| Turning radius | |

---

## 5. Mobility and Mechanical Design

### Chassis

| Property | Value | Why |
|---|---|---|
| Length | 15.0 cm | The parking slot is 1.5 times the robot length, so every centimetre of car costs 1.5 cm of slot. Short is safe. |
| Width | 10.5 cm | The lane is 100 cm wide. Narrow enough to pass a sign with margin either side. |
| Drive | Single motor, rear axle | Rules 11.3 and 11.5 require the drive wheels to be physically connected. One motor through a gearbox satisfies this and removes differential speed error. |
| Steering | Ackermann, front axle, one servo | Required by the category focus on kinematics other than differential drive. |

### Why This Motor

The GA12-N20 was chosen for three reasons, in order of weight.

**The built in encoder was the deciding factor.** It lets us measure actual wheel
rotation instead of assuming that PWM duty maps cleanly to speed. Without odometry we
cannot detect a stalled robot, and stall detection is what triggers our recovery
behaviour. A robot that is wedged against a wall and does not know it will sit there
until the round ends.

Second, the integrated gearbox fits a package small enough for a 15 cm chassis. Third,
it runs at 6 V, matching the rest of the low voltage system.

**Torque and speed reasoning.** Theoretical top speed follows the relation between
wheel circumference and output shaft revolutions per minute. For our wheel and motor
this gives a ceiling of roughly 0.5 m/s. That figure ignores load, rolling resistance
and battery sag, so we treat it as an upper bound rather than a target, and tune the
operating speed experimentally instead.

<!-- TODO: measure these on the bench and fill them in. Do not estimate.
| Measurement | Value |
|---|---|
| Measured top speed, wheels off the ground | |
| Measured top speed, on the mat | |
| Lowest PWM percentage that actually moves the robot | |
| Time to travel 1 m from standstill | |
| Measured turning radius at full lock | |
-->

**Why not two motors, one per side?** Explicitly forbidden by rule 11.5. It would also
have introduced differential error that our single encoder cannot see.

### Steering

The servo angle is converted to a pulse width from a calibrated centre position, with
three deliberate safety features.

The centre position is calibrated per build, because no two linkages are neutral at the
same pulse width. A direction constant flips the sign if the servo is mounted mirrored,
so mounting errors are fixed with a constant rather than by negating at the call site,
and the sign convention stays identical on both controllers. Most importantly, the
angle is clamped to plus or minus 30 degrees before conversion, and the resulting pulse
is clamped again to a safe range. A runaway command therefore cannot drive the servo
into the steering linkage and stall it. A stalled MG90S draws over an amp and destroys
its own gears in about a minute.

**Iteration.** Our first version clamped only the pulse width. Clamping the angle first
means the reported steering angle and the physical angle always agree, which matters
because the state machine logs the commanded angle when something goes wrong.

---

## 6. Power and Sensor Architecture

Wiring diagrams are in [`schemes/`](schemes/), with the complete master connection list
in [`schemes/README.md`](schemes/README.md).

### Power Budget

| Consumer | Typical | Peak | Notes |
|---|---|---|---|
| Raspberry Pi 3 | ~700 mA | ~2.5 A | Peaks during vision processing |
| MG90S servo | ~200 mA | ~1.5 A | Peak at stall or full lock |
| GA12-N20 motor | ~150 mA | ~800 mA | Peak at stall |
| Pico 2 W and sensors | ~150 mA | ~200 mA | Six I2C devices on the 3.3 V rail |
| **Total** | **~1.2 A** | **~5 A** | |

The peak figure is the one that matters. The Pi browns out and reboots if the 5 V rail
sags, and it sags exactly when the servo slams to full lock, which is precisely when
the robot is doing something difficult. Our mitigations are a 100 uF electrolytic
capacitor across the motor driver's supply pins at the driver itself, and a 0.1 uF
ceramic capacitor across the motor terminals to suppress brush noise.

### A Fault We Caught in Review

Our first connection list put the encoder on the 5 V rail. An N20 magnetic encoder
outputs at whatever voltage supplies it, so its two channels would have swung to 5 V
straight into microcontroller pins rated for 3.3 V. **The RP2350 is not 5 V tolerant.**
This would have worked on the bench and destroyed the microcontroller days or weeks
later, which is the worst kind of fault because it looks exactly like a software bug.

The encoder now runs from the Pico's 3.3 V rail alongside the I2C sensors, where it
draws a few milliamps. The lesson we recorded in our journal was to trace the logic
level path of every sensor output, not just its supply voltage.

### Sensor Placement

<!-- TODO: add a top down diagram showing sensor positions and angles.
     This is the single highest value image in the repository. -->

| Sensor | Quantity | Placement | Why here |
|---|---|---|---|
| Camera Module 3 | 1 | Front, elevated | Height gives earlier sight of signs, and it is the only sensor that distinguishes colour |
| VL53L0X | 1 | Front, 0 degrees | Corner confirmation and collision avoidance |
| VL53L0X | 2 | Plus and minus 45 degrees, flanking the camera | Lane position and corner geometry |
| VL53L0X | 1 | Rear, 180 degrees | Parking and reversing clearance |
| BNO085 | 1 | Centre, mounted flat | Heading for turn completion and straight line hold |
| TCS34725 | 1 | Underside, 5 to 10 mm above the mat | Corner line detection |

**Why 45 degrees and not 90?** This is the placement decision we spent longest on.
Sensors pointing straight out to the sides tell you the lane width, but they only see a
corner once you are already level with it, which is too late to plan a turn. At 45
degrees they watch the forward diagonals, so an approaching inner wall shows up while
there is still room to react.

The cost of that choice is that a reading is no longer lateral clearance. A reading of
D millimetres at 45 degrees means roughly 0.71 times D ahead and the same again to the
side. That conversion is documented at the top of the distance module, because anything
that treats a diagonal reading as lateral clearance will drive the robot into a wall.

Lane position then falls out of comparing the two diagonals against each other, which
needs no knowledge of the lane width at all. More room on the right means the car has
drifted left, so it steers right.

**A failure point we designed around.** A VL53L0X reports roughly 8190 mm when it sees
nothing at all. Fed into that comparison this looks like an enormously wide lane, so
readings above 2000 mm are discarded rather than believed.

### Why a Multiplexer

Every VL53L0X ships with the same fixed I2C address, and so does the colour sensor. Put
two on one bus and both answer at once, and nothing in software can separate them.

There are two ways out. One is to pulse each sensor's shutdown pin at boot to bring
them up one at a time and reassign addresses. The other is to switch the bus. We chose
the TCA9548A multiplexer because it needs no extra GPIO pin per sensor and no boot time
sequencing that can fail silently.

| Multiplexer channel | Device |
|---|---|
| 0 | VL53L0X front |
| 1 | TCS34725 colour |
| 2 | BNO085 IMU |
| 3 | VL53L0X left |
| 4 | VL53L0X right |
| 5 | VL53L0X rear |

The cost is that reads are sequential: switch channel, talk, switch channel, talk. A
full sweep of four distance sensors costs about 132 ms, which is why the Pico applies
commands every 20 ms but sweeps sensors only every 150 ms. A single speed loop would
have delayed every motor command by the length of a sensor sweep.

---

## 7. Software Architecture

```
   +---------------- Raspberry Pi 3 ----------------+   +---- Pico 2 W ----+
   |                                                |   |                  |
   |  camera --> vision --> signs, wall, slot --+   |   |  motionController|
   |                                            |   |   |    servo         |
   |  robot_state --> mission --> navigation ---+   |   |    motor driver  |
   |                             (HOW to drive) |   |   |                  |
   |                          state machine ----+---+---+--> "45,23"       |
   |                          (WHAT we are doing)   |   |                  |
   |                                                |   |  sensorManager   |
   |  robot_state <---------------------------------+---+-- "S,480,230..." |
   +------------------------------------------------+   +------------------+
```

Each frame runs these seven steps, in this exact order.

| Step | What happens | Module |
|---|---|---|
| 1 | Capture a frame | `main.py` |
| 2 | Detect signs, wall and parking markers | `vision_test.py` |
| 3 | Read the newest robot state from the Pico | `main.py` |
| 4 | Ask which challenge is active | `mission_manager.py` |
| 5 | Decide **how** to drive, meaning speed and steering | `navigation_engine.py` |
| 6 | Decide **what** we are doing, and restrain the command | `state_machine.py` |
| 7 | Send the final command over UART | `main.py` |

**The separation that makes this work.** Navigation decides how to drive. The state
machine decides what we are doing, and it may only restrain navigation's request, never
invent one of its own. Cornering caps the speed lower than the straights. Recovery
caps it lower still. Finishing caps it to zero. Approaching a sign caps nothing,
because avoiding a sign is precisely navigation's job and clipping its request would
make the robot worse at it.

Only the coordinator writes to the serial port, and it writes only the state machine's
output, so there is exactly one place in the whole system where a command can reach the
wheels.

### The State Machine

**Twelve states, seventeen events, thirty transitions, three missions.**

The critical design choice is that detecting what happened is separate from deciding
what to do about it. One function turns sensor readings into named events, and it is
the only place that knows about millimetres and degrees. A separate table maps each
state and event to the next state, and that table contains no numbers at all.

Row order within the table is priority. When a wall and a sign are both visible, the
wall wins, and that is a line a judge can point at rather than a consequence of which
if statement happened to come first.

Events fire whenever they are true, regardless of the current state, and the table
decides which ones matter. In the Open Challenge the robot still sees a red sign, and
the event appears in its debug output, but it is correctly ignored because that mission
has no sign handling state at all.

States: `WAIT_FOR_START`, `INITIALISE`, `FOLLOW_COURSE`, `APPROACH_PILLAR`,
`PASS_PILLAR`, `RECENTER`, `TURN_CORNER`, `SEARCH_PARKING`, `ALIGN_PARKING`,
`ENTER_PARKING`, `RECOVERY`, `FINISHED`.

### Vision Pipeline

Cost per frame is linear in the number of pixels and linear in the number of contours
found, and the pixel count is fixed by the processing width rather than the camera
resolution. The pipeline measures 0.32 ms per frame on a development laptop, which
means the camera frame rate is the ceiling rather than the code.

| Stage | Detail |
|---|---|
| Region of interest | Top 35 percent discarded, since only ceiling and lights live up there |
| Downscale | Detection runs at 320 pixels wide, drawing stays at full resolution |
| Colour threshold | Red needs two hue ranges because hue wraps around zero |
| Morphology | Open to remove speckle, close to fill glare holes |
| Contour filter | Area, aspect ratio and fill ratio together |

**Why the fill ratio matters.** Taking the largest contour locks onto a red jacket in
the audience or a stripe of glare on the mat. A real traffic sign fills its bounding
box, and scattered reflections do not. A candidate must pass area, aspect ratio and
fill ratio before it is believed. Our self test deliberately includes a wide red stripe
with the largest area in the frame and asserts that the detector skips it in favour of
the actual sign.

**Distance from a single camera.** Signs are a known 10 cm tall, so the ratio of real
height to pixel height gives range without a stereo rig.

**Saturation and brightness floors are deliberately low.** Our first version used much
higher floors and worked perfectly with one sign, then silently dropped the second one
whenever it sat further from the lights. Two signs on a mat are never lit equally.

---

## 8. Obstacle Strategy

Red on the right, green on the left. The elegant part is that this is one calculation
rather than a tree of cases.

Each colour is given a target position in the camera frame. Passing a red sign on its
right means the sign has to end up on the robot's left, so the target for red is a
position well to the left of centre, and green is the mirror image. The robot then
steers by however far the sign currently is from where it should be.

That single rule produces all of the behaviour below, with no special cases.

| Red sign position in frame | Steering response |
|---|---|
| Far left | None, already clear |
| Just left of centre | None, just cleared |
| Centred | Firm right |
| Right of centre | Harder right |
| Far right | Full lock right, the worst case |

One safety clamp is applied on top: a red sign may only ever produce right steering.
Without it, a sign already cleared to the left would produce a small correction back
towards the thing the robot just avoided, which is how you clip one.

### Choosing Between Two Signs

Both colours are often visible at once, typically a red one close and a green one
further down the track. The nearer sign is the one about to be hit, so that is the one
the robot steers around. The far one is handled on later frames, once it becomes the
near one. Where the vision module has measured a distance we rank by that. Where it has
not, we fall back to apparent size, since a nearer sign of the same real size looks
bigger.

### The Four Phase Pass

The robot moves through `FOLLOW_COURSE`, then `APPROACH_PILLAR`, then `PASS_PILLAR`,
then `RECENTER`, and back to `FOLLOW_COURSE`.

**The recentre phase is what stops the robot living permanently offset.** Our first
implementation steered around a sign and simply carried on, drifting a little further
from the lane centre with each obstacle until it clipped a wall three signs later. Now,
once the sign has been out of view for half a second, steering ignores signs entirely
and tracks the lane centre until the offset is back inside a dead zone.

There is a deliberate asymmetry worth noting. The passing phase has no route into
recovery. Losing sight of a sign you are squeezing past is exactly what success looks
like, so both of its exits lead to recentring instead.

### Edge Cases We Handle

| Case | Handling |
|---|---|
| Sign flickers out for a single frame | Half a second of confirmation before the pass is declared over |
| Two signs in view at once | Nearest by measured distance wins, and frame order is irrelevant |
| Sign appears in the middle of a corner | The corner outranks the sign, because the table row order says so |
| Blob smaller than the minimum area | Treated as noise, not as a sign |
| Sign jittering around the frame centre | A dead zone snaps it to exactly centred so the wheels do not twitch |
| A sensor returns no reading | Speed drops to slow rather than cruising blind |
| Nothing detected at all | Lane following from the two diagonal sensors |

---

## 9. Parking Strategy

The parking lot is bounded by two magenta elements measuring 20 by 2 by 10 cm. The slot
is 20 cm wide and 1.5 times the robot's length, which for our 15 cm car means 22.5 cm.

### Magenta Against Red

Magenta sits immediately below red on the hue circle. Our original upper red band
started low enough that it **swallowed the magenta markers entirely**, and the robot
saw the parking lot as a giant traffic sign and tried to pass it on the right. The red
band now starts higher, which costs nothing because red's main band sits at the other
end of the scale. Our self test puts a red sign and both magenta markers in the same
frame and asserts that neither colour steals the other.

### Aiming at the Gap

The vision module returns the midpoint between the markers' inner edges, which is the
slot itself rather than either marker. Verified in the self test as within one pixel of
the true midpoint.

Both markers must be visible before the robot commits, because you cannot aim at a gap
you can only half see. The sequence is `SEARCH_PARKING`, then `ALIGN_PARKING`, then
`ENTER_PARKING`, then `FINISHED`.

### Two Bugs This Sequence Taught Us

**We were watching the wrong sensor.** Our first version stopped when the rear distance
sensor read close. But the camera faces forward, so the slot is only visible while
driving at it, meaning the robot enters nose first and the rear sensor is pointed back
at the open mat it came from. It never triggered, and the robot stopped on a timeout
rather than on arrival.

**The stop was then physically unreachable.** Navigation performs an emergency stop at
150 mm, but the parking trigger sat at 120 mm, so the robot froze before it could ever
reach its own trigger. The fix was a minimum speed for the entry phase, the same
pattern we had already needed for corners, together with a threshold below the
emergency stop line. A self test assertion now enforces that ordering so it cannot
silently break again.

---

## 10. Systems Thinking

### Constraints We Designed Around

| Constraint | Source | Consequence |
|---|---|---|
| Slot is 1.5 times robot length | Rule | Chassis kept to 15 cm, since every cm costs 1.5 cm of slot |
| Drive wheels physically connected | Rules 11.3 and 11.5 | Single motor and gearbox, not one motor per side |
| Two buttons only | Rules 9.10 and 9.11 | No laptop interaction at the start line |
| Track randomised each round | Rule | Nothing hard coded, all behaviour sensor driven |
| Distance and colour sensors share one address | Hardware | Multiplexer required |
| RP2350 is not 5 V tolerant | Hardware | All sensor logic on the 3.3 V rail |
| Linux is not real time | Platform | Two controller architecture |

### Key Trade-offs

| Decision | Chosen | Rejected | Reasoning |
|---|---|---|---|
| Compute | Pi 3 and Pico together | Pi alone | A non real time operating system would starve the control loop |
| Sensor addressing | Multiplexer | Shutdown pin re-addressing | No boot sequencing that can fail silently |
| Side sensor angle | 45 degrees | 90 degrees | Sees corners early enough to plan a turn |
| Control law | Proportional | PID | Tunable at a competition by one person under pressure |
| Sign selection | Nearest | Largest contour | Correct when two signs are visible at once |
| Detection width | 320 pixels | Full resolution | Roughly five times fewer pixels, making the camera the bottleneck rather than the code |

**Why no PID controller?** We can implement one. We chose not to. A PID controller has
three interacting constants, and a competition venue is the worst possible place to
tune three interacting constants under time pressure. Our steering has one gain and one
rate limit, both of which a team member can reason about between rounds. Determinism
and tunability beat theoretical optimality when you only get two attempts.

### Risk Analysis

| Risk | Impact | Mitigation |
|---|---|---|
| Pi crashes mid run | Robot drives on uncommanded | Watchdog on the Pico stops the robot after 500 ms of silence |
| UART disconnects | Total loss of control | Robot stops, and the program keeps retrying the connection |
| One sensor fails | Cascade failure | Every sensor returns nothing rather than raising, so one dead sensor costs one reading |
| A driver file is missing on the Pico | All sensors dead | Driver imports are isolated, and each reports itself by name |
| Voltage sag at full lock | Pi reboots mid run | Decoupling at the driver and a peak current budget |
| Robot wedged against a wall | Run is over | Stall detection through the encoder triggers recovery |
| Corner never completes | Deadlock | Timeout into recovery, and a minimum speed that prevents the freeze |
| Serial clock drift on the Pi 3 | Corrupt bytes under load | Bluetooth disabled to force the stable hardware UART |

**The Pi 3 serial trap deserves its own note.** On a Pi 3 the default serial port is the
mini UART, whose baud rate is derived from a clock that changes with processor load. It
works perfectly when the machine is idle and corrupts bytes the moment the vision code
loads the processor. That is an intermittent fault that only appears when the robot is
working hardest, which makes it extremely hard to diagnose. Disabling Bluetooth moves
the port onto the stable hardware UART instead.

### Iteration Log

| Problem | Root cause | Fix |
|---|---|---|
| Only one of two signs detected | Saturation and brightness floors too high, so the dimmer sign fell below them | Floors lowered |
| Robot clipped signs it had already passed | Counter steering back towards a cleared sign | One directional clamp per colour |
| Drifted further off centre with each sign | No return to lane behaviour existed | Recentre state added |
| Robot gave up in the middle of a corner | Emergency stop fired at the wall being turned away from, and a stopped robot stops turning | Minimum speed for cornering |
| Parking finished on a timer | Stop condition watched the rear sensor during a nose first entry | Front sensor instead, gated to parking states |
| Parking stop unreachable | Trigger sat inside the emergency stop zone | Minimum speed plus a lower threshold, with an assertion |
| Encoder would have destroyed the Pico | 5 V logic driven into a 3.3 V pin | Moved to the 3.3 V rail |
| Magenta read as red | Hue bands overlapped | Red band narrowed |
| Speed flapped between cruise and slow | A single threshold with noisy readings | Hysteresis, with separate entry and exit thresholds |
| Stall detection never fired | A timestamp of zero was treated as unset | Explicit comparison against nothing |

---

## 11. Testing and Validation

**Every module self tests on a laptop with no hardware attached.** Fifteen of the
sixteen modules carry their own test, and all of them pass.

| Module | What its self test proves |
|---|---|
| Vision | Both colours detected in one frame, noise and wrong shaped blobs rejected, magenta not read as red, gap midpoint within one pixel |
| Navigation | Pass side never wrong anywhere in the frame, steering never exceeds the servo limit, rate limit holds, hysteresis works in both directions |
| State machine | Every state has a transition row, no duplicate events, no mission can deadlock, every substitute state is reachable |
| Coordinator (Pi) | Vision output converts to the navigation format, the wall never reaches navigation, the state machine has the final word |
| Flight program (Pico) | Wire format verified against the Pi's actual parser rather than a copy of it |
| Servo | Clamping is symmetric and the pulse always stays inside safe limits |
| Motor driver | Negative speed means stop rather than reverse, and duty is monotonic |
| Encoder | One wheel turn equals one circumference, and reverse counts negative |
| Distance | One multiplexer bit per channel, and the IMU and colour channels are never touched |
| IMU | Quaternion conversion correct at 0, 90 and 180 degrees, and a gimbal lock input does not raise |
| Colour | The same surface at half brightness classifies identically |

The assertions we value most are the structural ones, because they catch a half
finished edit rather than a wrong number. Every state must have a transition row. Every
mission must be able to reach an ending. The parking threshold must sit below the
emergency stop threshold. These fail on a laptop in milliseconds rather than on the mat
in front of a judge.

<!-- TODO: replace with real measurements once the robot has run.
### Field Test Results
| Test | Runs | Successes | Notes |
|---|---|---|---|
| Open Challenge, three laps | | | |
| Obstacle Challenge, three laps | | | |
| Parking, both driving directions | | | |
| Red sign pass | | | |
| Green sign pass | | | |
-->

> **Current status.** The software stack is complete and passes fifteen module self
> tests. Field testing on the physical robot is in progress, and measured results will
> be added here as they are collected.

---

## 12. Engineering Journal

<!-- TODO: add the PDF to other/ and link it here -->

The full engineering journal, covering weekly progress, sketches, failed prototypes and
test logs, is in [`other/`](other/).

---

## 13. Videos

<!-- TODO: replace with real links. Each video must show at least 30 seconds of
     autonomous driving, and one is required for each challenge. -->

| Round | Link |
|---|---|
| Open Challenge | *YouTube URL* |
| Obstacle Challenge | *YouTube URL* |

Also recorded in [`video/`](video/).

---

## 14. Acknowledgements

<!-- TODO: your school, mentors and sponsors -->

Open source work we build on: OpenCV, MicroPython, and the MicroPython drivers for the
VL53L0X, BNO08x and TCS34725 sensors. We also studied the public repositories of
previous WRO Future Engineers teams. The category's culture of publishing work openly
is the reason this repository exists in the form it does.

---

<p align="center">
  <b>Team Tricky Trio</b> &middot; WRO 2026 Future Engineers<br>
  <i>Every constant in this repository is either measured or explained.</i>
</p>
