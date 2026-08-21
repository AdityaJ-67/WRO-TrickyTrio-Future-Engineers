# Design Decisions

Why the robot is built the way it is.

## Constraints We Designed Around

Every one of these came from either the rulebook or the hardware, and each forced a
decision we would not otherwise have made.

| Constraint | Source | What it forced |
|---|---|---|
| Parking slot is 1.5 times the robot's length | Rule | Chassis kept to 15 cm. Every centimetre of car costs 1.5 cm of slot, so a longer robot parks in a proportionally longer space and gains nothing. |
| Drive wheels must be physically connected | Rules 11.3 and 11.5 | One motor through a gearbox rather than one motor per side. |
| One button to power on, one to start | Rules 9.10 and 9.11 | No laptop interaction at the start line. |
| Track layout randomised every round | Rule | Nothing hard coded. No memorised turn sequence, no fixed timings, all behaviour driven by what the sensors report. |
| Distance and colour sensors share I2C address 0x29 | Hardware | A multiplexer is unavoidable. |
| RP2350 is not 5 V tolerant | Hardware | Every sensor output has to be on the 3.3 V rail. |
| Linux is not a real time operating system | Platform | Control split across two boards. |

## Key Trade-offs

| Decision | What we chose | What we rejected | Reasoning |
|---|---|---|---|
| Compute | Pi 3 and Pico together | A single Pi doing everything | A non real time operating system can stall a control loop for tens of milliseconds. That is invisible when processing an image and disastrous when a servo is waiting for a pulse. |
| Sensor addressing | TCA9548A multiplexer | Reassigning addresses via the shutdown pins at boot | Address reassignment needs a boot sequence that can fail silently, and one extra GPIO per sensor. |
| Side sensor angle | 45 degrees | 90 degrees, straight out to the sides | At 90 degrees a corner only appears once you are level with it, which is too late to plan a turn. At 45 degrees the same sensors watch the forward diagonals. |
| Steering control law | Proportional, one gain | PID | A PID controller has three interacting constants. A competition venue is the worst possible place to tune three interacting constants under time pressure. |
| Which sign to act on | The nearest one | The largest contour in the frame | Largest is only correct by accident. When two signs are visible, the near one is the one about to be hit. |
| Detection resolution | 320 pixels wide | Full camera resolution | Roughly five times fewer pixels. The camera frame rate becomes the limit rather than the processing, which is the right way round. |
| Parking entry | Nose first | Reversing in | The camera faces forward, so the slot is only visible while driving towards it. Reversing in means backing in blind. |

## Why We Did Not Use PID

We can implement a PID controller. We chose not to, and it is worth explaining because
it looks like a gap.

A PID controller has three constants that interact with each other. Changing one
changes what the correct value of the other two would be. Tuning it properly needs
repeated tests in the conditions it will run in.

At a competition you get limited practice time on an unfamiliar table, under lighting
you did not choose, with two attempts that count. Our steering has **one gain and one
rate limit**. If the robot swings too wide we know which number to change and in which
direction, and any of the three of us can do it.

Determinism and tunability beat theoretical optimality when the tuning has to happen
under pressure.

## Risk Analysis

What could go wrong, how bad it would be, and what we did about it.

| Risk | Impact if unmitigated | Mitigation |
|---|---|---|
| Pi crashes mid run | Robot keeps driving on its last command until it hits something | The Pico stops the robot if no command arrives for 500 ms. This is the single most important reason the control system is split across two boards. |
| Serial cable comes loose | Same as above | Same watchdog. The Pi also keeps retrying the connection rather than exiting. |
| One sensor fails | A crash in the reading code takes down every other sensor with it | Every sensor returns nothing rather than raising. One dead sensor costs one reading. |
| A driver file is missing from the board | All six sensors silently dead | Driver imports are isolated from each other and from the I2C setup. Each reports itself by name at startup. |
| Voltage sag when the servo hits full lock | The Pi browns out and reboots mid run | Decoupling capacitor at the motor driver, and a power budget sized on peak rather than average draw. |
| Robot wedges against a wall | The round is over and nothing notices | Encoder detects that the robot is commanded to move but is not moving, and triggers recovery. |
| A corner never completes | The robot freezes facing a wall | The state times out into recovery, and a minimum speed prevents the freeze in the first place. |
| Serial clock drift on the Pi 3 | Corrupted bytes, but only under processor load | Bluetooth disabled so the port uses the stable hardware UART. |
| A sign is detected on the wrong side | Robot steers into the thing it is meant to avoid | The pass direction is clamped per colour, so a red sign can only ever produce right steering. |

### The Fault We Caught Before It Cost Us a Board

Our first connection list put the N20 encoder on the 5 V rail.

A magnetic encoder outputs at whatever voltage supplies it. On 5 V, its two channels
would have driven 5 V straight into GP12 and GP13, and the RP2350 has an absolute
maximum of 3.3 V plus 0.3 V on any GPIO pin.

The reason this one is worth recording is the failure mode. It would have worked on the
bench. The board would have died days or weeks later, and it would have looked exactly
like a software bug, because the symptom would have been sensor readings quietly going
wrong rather than anything obviously electrical.

The encoder now runs from the Pico's 3.3 V rail, where it draws a few milliamps. The
lesson we took from it was to trace the logic level path of every sensor output, not
just check that its supply voltage is correct.
