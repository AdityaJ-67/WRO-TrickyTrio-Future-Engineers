# Mechanical Design and Mobility

Chassis design choices, drive and steering justification, torque and speed reasoning,
and the iterations that changed the design.

Covers rubric criterion 1. Part files and renders are in the subfolders of this
directory; this document explains why they are shaped the way they are.

## Contents

1. [Design Constraints](#design-constraints)
2. [Chassis](#chassis)
3. [Drive System](#drive-system)
4. [Torque and Speed Reasoning](#torque-and-speed-reasoning)
5. [Steering System](#steering-system)
6. [Mechanical Stability](#mechanical-stability)
7. [Trade-offs](#trade-offs)
8. [Iterations](#iterations)

## Design Constraints

Every mechanical decision on this robot traces back to one of these.

| Constraint | Source | Consequence |
|---|---|---|
| Parking slot is 1.5 times the robot's length | Rule | Length is the most expensive dimension on the robot. Every centimetre of car costs 1.5 cm of slot, and the slot is bounded by walls we must not touch. |
| Drive wheels must be physically connected | Rules 11.3 and 11.5 | One motor and a gearbox. One motor per side is explicitly forbidden. |
| Lane is 100 cm wide, signs are 5 cm square | Rule | Passing a sign in a 100 cm lane needs roughly 10.75 cm of lateral clearance from our centreline, so a narrow robot has more room for steering error. |
| Track randomised each round | Rule | The chassis must be symmetric in behaviour. It has to turn as tightly clockwise as anticlockwise. |
| Camera needs height | Physics | The higher the camera, the earlier a sign is visible. This fights against keeping the centre of gravity low. |

## Chassis

| Property | Value | Reasoning |
|---|---|---|
| Length | 15.0 cm | Sets the parking slot at 22.5 cm. Shorter would be better still, but the drivetrain, steering assembly and two boards have to fit. |
| Width | 10.5 cm | With 10.75 cm of clearance needed from the centreline, a wider car eats the margin for steering error. |
| Nose shape | Trapezoid | See below. This is the most deliberate shape decision on the robot. |
| Construction | 3D printed, four body sections | Each section fits the print bed and can be replaced without reprinting the whole car. |

### Why the nose is a trapezoid

The three front distance sensors need to look straight ahead and at plus and minus 45
degrees. There are two ways to achieve that.

The first is to mount all three on a flat front face and hold two of them at an angle
with printed wedges. The second is to shape the chassis so each sensor sits flat against
a face that already points the right way.

We chose the second. A sensor bolted flat to a flat surface aligns repeatably, every
time it is removed and refitted. A sensor held at an angle by a printed wedge depends on
the wedge printing accurately, on the screw torque, and on the part not creeping.

That repeatability matters more here than it would elsewhere, because the lane following
calculation compares the left and right readings **against each other**. A couple of
degrees of error on one side biases every lane position estimate in the same direction,
and a consistent bias is far harder to notice than random noise.

The cost is a more complex print and slightly less internal volume at the front.

## Drive System

### Motor selection

We compared three options.

| Option | Torque | Encoder | Size | Verdict |
|---|---|---|---|---|
| GA12-N20 with encoder | Adequate with gearbox | Built in | Fits | **Chosen** |
| N20 without encoder | Same | None | Fits | Rejected. No odometry means no stall detection. |
| TT gearmotor | Higher | None | Too large for a 15 cm chassis | Rejected on size. |

**The encoder was the deciding factor, not the torque.** Without odometry the robot
cannot tell the difference between driving forward and being held against a wall with
the wheels slipping. Stall detection is what triggers the recovery behaviour, and a
robot that gets wedged and does not know it will sit there until the round ends. We
would have accepted less torque to keep the encoder.

### Single motor, driven rear axle

Required by rules 11.3 and 11.5, but it also removes a problem. Two motors, one per
side, differ slightly in speed even at identical commands, and our single encoder cannot
see that difference. The robot would curve gently and the odometry would report a
straight line.

## Torque and Speed Reasoning

<!-- TODO: measure the values marked below. The formulas are correct; the numbers
     are what turn this section from theory into evidence. -->

### Top speed

Wheel speed follows from motor output speed and wheel size:

```
v = pi * D * N / 60
```

where `D` is wheel diameter in metres and `N` is output shaft revolutions per minute.

| Symbol | Value | Source |
|---|---|---|
| D, wheel diameter | *measure under load* | Rubber compresses, so the rolling diameter is smaller than the modelled one |
| N, output RPM | *from motor spec* | Depends on the gearbox ratio fitted |
| v, theoretical top speed | *calculate* | |
| v, measured on the mat | *measure* | Expect this to be lower |

The theoretical figure ignores load, rolling resistance and battery sag under current
draw, so we treat it as a ceiling rather than a target and tune the operating speed
experimentally.

### Torque required

The motor has to overcome rolling resistance and accelerate the robot's mass.

```
F_total   = F_rolling + F_acceleration
F_rolling = mu * m * g
F_accel   = m * a
T_wheel   = F_total * r
```

| Symbol | Meaning | Value |
|---|---|---|
| m | Robot mass | *weigh it* |
| mu | Rolling resistance coefficient | roughly 0.015 to 0.03, rubber on a smooth mat |
| g | Gravity | 9.81 m/s squared |
| a | Target acceleration | *choose, then verify* |
| r | Wheel radius | *measure under load* |
| T_wheel | Torque needed at the wheel | *calculate* |

Then compare against what the motor can supply:

| Check | Value |
|---|---|
| Motor stall torque at the output shaft | *from spec* |
| Torque required, from above | *calculate* |
| **Margin** | *stall divided by required* |

**What the margin should be.** A ratio near 1 means the motor barely moves the robot and
will stall on any bump or against a wall. A very large ratio means the gearbox is
over-geared and top speed suffers. We are looking for enough headroom that the robot
accelerates cleanly from rest and can push out of a light collision, without giving up
the speed needed to finish three laps in time.

### The gear ratio trade-off

Gear ratio trades torque against speed directly. A higher ratio gives more torque and a
lower top speed; a lower ratio does the reverse.

Because the course is flat and the robot is light, torque is not the binding constraint.
Speed is, since three laps have to fit inside the round. But dropping the ratio too far
means the robot cannot start cleanly from rest, and cannot recover from being nudged
against a wall.

<!-- TODO: record which ratio you fitted, and whether you tried another one. If you
     tested two ratios, that comparison is exactly the "iterations affecting
     performance" the rubric asks for. -->

## Steering System

### Ackermann geometry

When a car turns, the inside wheel follows a tighter circle than the outside wheel. If
both front wheels were held parallel, one of them would scrub sideways through every
corner.

Ackermann geometry angles the steering arms so the inside wheel turns through a larger
angle than the outside one, letting both roll cleanly around a shared turning centre.

**Two reasons it matters on this robot specifically.**

The corners are tight relative to the wheelbase, so the difference between the required
inside and outside angles is significant rather than a rounding error.

Our odometry comes from the **driven rear wheels**, and anything that makes the front
wheels scrub adds drag that the encoder cannot see. The robot would think it had
travelled further than it had.

### Servo selection

| Option | Verdict |
|---|---|
| MG90S, metal geared | **Chosen.** The gear train survives clipping a wall. |
| SG90, plastic geared | Rejected. Identical size and cheaper, but the nylon gears strip on impact, and hitting a wall during testing is certain. |
| Stepper with a rack | Rejected. Heavier, needs a driver, and open loop position is worse than a servo's closed loop. |

<!-- TODO: fill in once measured
| Measurement | Value |
|---|---|
| Maximum steering angle achieved | |
| Turning radius at full lock | |
| Servo current at full lock | |
-->

### Software protection for the mechanism

The steering angle is clamped to plus or minus 30 degrees in software **before** it is
converted to a pulse width, and the resulting pulse is clamped again to a safe range.

This exists to protect the mechanism, not the code. A runaway command cannot drive the
servo into the end of the steering linkage. A stalled MG90S draws over an amp and
destroys its own gear train in about a minute.

## Mechanical Stability

**Battery placement.** The battery is the heaviest single item, and it sits low and
central. Low keeps the centre of gravity down, which reduces how much the car leans when
cornering hard. Central keeps weight distribution predictable between the axles.

**Weight over the driven axle.** The rear wheels do all the driving, so they need
enough vertical load to grip. Too much weight forward and they slip under acceleration,
which shows up as odometry error rather than as an obvious symptom.

**Camera height versus centre of gravity.** These pull in opposite directions. A higher
camera sees signs earlier, which the navigation benefits from. A higher mass raises the
centre of gravity, which hurts cornering. The camera itself is light, so the compromise
is cheap, but the mount has to be rigid rather than tall and flexible.

**Why mount rigidity matters for the sensors.** We estimate how far away a sign is from
how tall it appears in the image. A mount that sags under its own weight, or vibrates
while the motor runs, changes that apparent height and reports the wrong distance.
Because the error is systematic rather than random, averaging does not remove it.

## Trade-offs

| Decision | Chosen | Rejected | Why |
|---|---|---|---|
| Chassis length | 15 cm | Longer, easier to package | Parking slot scales at 1.5 times length |
| Nose shape | Trapezoid | Flat front with angled brackets | Sensors align repeatably against flat faces |
| Drive | One motor, rear axle | One motor per side | Rules 11.3 and 11.5, and single encoder odometry |
| Steering | Ackermann, one servo | Parallel front wheels | Parallel wheels scrub in tight corners |
| Servo | MG90S metal gear | SG90 plastic gear | Plastic gears strip on wall impacts |
| Body | Four printed sections | One piece | Any section can be reprinted alone |
| Battery position | Low and central | Rear, easier to access | Centre of gravity and weight distribution |

## Iterations

<!-- TODO: this section is where criterion 1 is won or lost. The rubric asks
     specifically for "testing or iterations affecting performance". Record every
     version of a part, what was wrong with it, and what changed.

     For each iteration, record:
       - what the problem was, observed rather than predicted
       - what you measured
       - what you changed
       - what the measurement was afterwards

     Photograph the old part next to the new one and put it in
     v-photos/build-progress/. -->

| Part | Version | Problem observed | Change made | Result |
|---|---|---|---|---|
| | | | | |

Iterations already recorded elsewhere, which belong here too once the mechanical side
has been tested:

- The steering clamp order was changed so the angle is limited before conversion rather
  than after, so the reported angle and the physical angle always agree. Recorded in
  [the iteration log](../other/engineering-journal/iteration-log.md).
