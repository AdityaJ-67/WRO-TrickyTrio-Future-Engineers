# Test Logs

Dated records of what was tested, what the numbers were, and what changed as a result.

Fill a row in as soon as a test is run. Reconstructing this later from memory is both
harder and less honest.

---

## Software Self Tests

Every module carries a self test that runs on a laptop with no hardware attached.
Fifteen of the sixteen modules are covered.

| Module | What its test proves |
|---|---|
| Vision | Both colours detected in one frame, noise and wrong shaped blobs rejected, magenta not read as red, parking gap midpoint within one pixel |
| Navigation | Pass side never wrong anywhere in the frame, steering never exceeds the servo limit, rate limit holds, hysteresis works in both directions |
| State machine | Every state has a transition row, no duplicate events, no mission can deadlock |
| Coordinator | Vision output converts correctly, the state machine has the final word on the command |
| Flight program | Wire format verified against the Pi's actual parser rather than a copy of it |
| Servo | Clamping symmetric, pulse always inside safe limits |
| Motor driver | Negative speed means stop rather than reverse, duty monotonic |
| Encoder | One wheel turn equals one circumference, reverse counts negative |
| Distance | One multiplexer bit per channel, other channels never touched |
| IMU | Orientation conversion correct at 0, 90 and 180 degrees |
| Colour | The same surface at half brightness classifies identically |

Run them with `python3 <module>.py --selftest`.

---

## Bench Tests

Wheels off the ground, one subsystem at a time.

<!-- TODO: fill a row in each time you test. Copy the blank row. -->

| Date | Test | Result | Numbers | Action taken |
|---|---|---|---|---|
| | Servo centres straight | | Centre pulse: | |
| | Servo reaches both limits without straining | | Max angle: | |
| | Motor spins in the correct direction | | | |
| | Motor stops from every speed | | | |
| | Lowest speed that moves the robot | | Percent: | |
| | Encoder counts one turn correctly | | Pulses per turn: | |
| | Encoder counts down in reverse | | | |
| | All four distance sensors respond | | | |
| | IMU heading changes correctly when turned | | | |
| | Colour sensor distinguishes mat surfaces | | | |
| | UART survives five minutes under processor load | | | |

---

## Calibration Measurements

The values for this specific build. Every one of these has to be measured, not assumed.

<!-- TODO: fill these in. Each affects something downstream, noted in the last column. -->

| Value | Measured | Affects |
|---|---|---|
| Gear ratio | | Every distance the robot calculates |
| Wheel diameter, under load | | Every distance the robot calculates |
| Servo centre pulse width | | Whether the robot drives straight |
| Maximum usable steering angle | | Turning radius, and whether the servo strains |
| Camera field of view | | Distance estimates to signs and markers |
| Camera height above the mat | | How early signs are seen |
| Red hue range, on the mat | | Whether red signs are detected |
| Green hue range, on the mat | | Whether green signs are detected |
| Magenta hue range, on the mat | | Whether the parking slot is found |
| Battery run time under load | | How many practice runs per charge |

---

## Field Tests

Full runs on a mat.

<!-- TODO: one row per session. Record failures in as much detail as successes,
     they are the rows that tell you what to fix. -->

| Date | Round | Runs | Completed | What failed | What we changed |
|---|---|---|---|---|---|
| | Open | | | | |
| | Obstacle | | | | |
| | Parking | | | | |

---

## Reliability Runs

Once the robot completes a round, repeat it. A robot that works once is not the same as
a robot that works.

<!-- TODO -->

| Date | Round | Attempts | Successes | Success rate | Notes |
|---|---|---|---|---|---|
| | | | | | |
