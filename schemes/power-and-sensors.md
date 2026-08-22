# Power and Sensor Architecture

Power budget, current draw reasoning, sensor selection and placement, calibration
method, and failure point analysis.

Covers rubric criterion 2. Wiring diagrams and the full connection list are in
[README.md](README.md) and the schematic files alongside it.

## Contents

1. [Power Architecture](#power-architecture)
2. [Current Draw Reasoning](#current-draw-reasoning)
3. [Sensor Selection](#sensor-selection)
4. [Sensor Placement](#sensor-placement)
5. [Calibration Method](#calibration-method)
6. [Failure Points](#failure-points)
7. [Iterations](#iterations)

## Power Architecture

```
Battery 7.4 V 2S LiPo
   |
   +---------------------------> DRV8833 VM (motor supply, unregulated)
   |
   +--> Mini560 buck --> 5 V rail --> Raspberry Pi 3
                                  --> Pico 2 W (VSYS)
                                  --> MG90S servo
                                        |
                          Pico 3V3 OUT --+--> TCA9548A multiplexer
                                         +--> 4x VL53L0X
                                         +--> BNO085 IMU
                                         +--> TCS34725 colour
                                         +--> N20 encoder
                                         +--> DRV8833 nSLEEP
```

### The one deliberate choice in this diagram

The motor supply taps the **raw battery**, in parallel with the buck converter's input,
rather than running off the regulated 5 V rail.

**What we gain.** Motor current never passes through the regulated logic rail. A motor
draws its largest current exactly when it is working hardest, and those spikes would
otherwise sag the voltage feeding the Pi, the Pico and every sensor. The Pi reboots if
its supply dips, and a reboot mid run ends the round.

**What it costs.** The battery sits above the motor's 6 V rating when freshly charged,
so running at 100 percent duty would overdrive it. This is why the PWM duty is limited
in software rather than left uncapped, and why `MAX_SPEED` exists as a constant rather
than being assumed to be 100.

**The alternative we rejected.** Running the motor from the regulated 5 V would have
protected the motor automatically, at the cost of putting every current spike through
the same regulator that feeds the Pi. We would rather software-limit the motor than
risk the brain rebooting.

### Three voltage domains

| Domain | Voltage | What runs on it | Why |
|---|---|---|---|
| Battery | 7.4 V nominal | Motor only | Isolates current spikes from everything else |
| Logic supply | 5 V regulated | Pi, Pico, servo | What these boards expect |
| Sensor logic | 3.3 V | Every sensor, the multiplexer, nSLEEP | The RP2350 is not 5 V tolerant |

**That last row is a hard rule, not a preference.** Any GPIO on the RP2350 has an
absolute maximum of 3.3 V plus 0.3 V. Every sensor output has to be inside that.

## Current Draw Reasoning

| Consumer | Typical | Peak | When the peak happens |
|---|---|---|---|
| Raspberry Pi 3 | ~700 mA | ~2.5 A | Vision processing on all four cores |
| MG90S servo | ~200 mA | ~1.5 A | Stall, or slamming to full lock |
| GA12-N20 motor | ~150 mA | ~800 mA | Stall, or accelerating from rest |
| Pico 2 W and six sensors | ~150 mA | ~200 mA | Continuous |
| **Total** | **~1.2 A** | **~5 A** | |

### Why the peak is the number that matters

The typical figure tells you battery life. The peak figure tells you whether the robot
works at all.

The three peaks are not independent. The servo hits full lock when the robot is
cornering or dodging a sign, which is exactly when the vision workload is highest and
the motor is working against a turn. They coincide, and they coincide at the worst
possible moment.

If the 5 V rail sags below the Pi's tolerance during that moment, the Pi reboots. The
Pico's watchdog then stops the robot after 500 ms, which is the correct behaviour but
still ends the run.

### Mitigations

| Measure | What it addresses |
|---|---|
| Motor on the unregulated rail | Removes the largest current spike from the regulated supply entirely |
| 100 uF electrolytic across the driver's VM and GND, at the driver | Supplies the instantaneous current a motor demands at switch-on, so it is not drawn through the wiring |
| 0.1 uF ceramic across the motor terminals | Suppresses brush noise, which otherwise couples into the encoder lines |
| Software speed limit | Keeps the motor away from stall current in normal operation |
| Ground plane on the PCB | Low impedance return path, so ground does not shift under load |

<!-- TODO: measure these. A power budget with measured numbers scores far better
     than one with estimates.
| Measurement | Value |
|---|---|
| Idle current, whole robot | |
| Current while driving straight | |
| Peak current, servo to full lock while accelerating | |
| 5 V rail voltage under that peak | |
| Battery run time, continuous driving | |
-->

## Sensor Selection

Each sensor exists because something else could not do its job.

### Camera Module 3

**Chosen because it is the only sensor that distinguishes colour.** The entire obstacle
challenge depends on telling red from green, and no distance sensor can do that.

It is also the only sensor with useful range for planning. A ToF sensor tells you
something is 400 mm away; the camera tells you it is a red sign 400 mm away and slightly
to the left, which is enough to plan a path rather than react.

**Rejected alternative:** a colour sensor pointed forward. Far cheaper, but it gives a
single reading with no idea where in the field of view the colour is.

### VL53L0X, four of them

**Chosen over ultrasonic.** Ultrasonic sensors are cheap and have good range, but they
have a wide beam, they are confused by soft or angled surfaces, and multiple units
interfere with each other. Four of them firing near each other would cross-talk badly.

The VL53L0X uses a narrow infrared laser and time of flight. Narrow beam means it
measures what it is pointed at rather than averaging a cone.

**Why four and not one.** One forward sensor tells you something is ahead. Four tell you
where you are in the lane, whether a corner is coming, and whether there is room behind
you to park. Position is a different question from proximity.

**Rejected alternative:** LIDAR. Better data, but larger, more expensive, and needs more
processing than a Pico can give it.

### BNO085 IMU

**Chosen because heading cannot be derived from anything else we have.** Wheel odometry
drifts as soon as a wheel slips, and it slips most during exactly the turns you most
want to measure.

**Chosen over a cheaper IMU** such as an MPU6050, because the BNO085 runs sensor fusion
on its own processor. A raw accelerometer and gyroscope would need a filter written and
tuned by us, running on the Pico, competing with everything else for time.

### TCS34725 colour sensor

**Chosen for redundancy and for the corner lines.** The mat marks corners with coloured
lines, and reading them directly under the robot is more reliable than trying to see
them at an angle in the camera.

**Its infrared filter is the reason we can use it at all.** Four infrared distance
sensors are firing a few centimetres away. A colour sensor without an IR filter would
read their light as a large fake contribution to every channel.

### TCA9548A multiplexer

**Chosen because it is unavoidable.** Every VL53L0X has fixed I2C address 0x29, and so
does the TCS34725. Two on a bus and both answer at once.

**Rejected alternative:** using the XSHUT pins to bring each sensor up one at a time and
reassign addresses at boot. That needs one GPIO per sensor and a boot sequence that can
fail silently, leaving two sensors on the same address with no error.

The multiplexer costs sequential reads instead of parallel ones, which is a real cost:
a full sweep of four ToF sensors takes about 132 ms. We handle that by running the
sensor sweep on a slower schedule than the control loop.

## Sensor Placement

<img src="sensor-placement.svg" width="620">

### Placement justified by field geometry

**The two 45 degree sensors are the placement decision we spent longest on.**

The field is a rectangular track with walls on both sides. What the robot needs to know
is where it sits across the lane, and when a corner is arriving.

A sensor at 90 degrees, pointing straight out to the side, measures lane position well.
But it only sees a corner once the robot is already level with it, which is far too late
to plan a turn at speed.

A sensor at 0 degrees sees the corner early but says nothing about lateral position.

At 45 degrees, one sensor does both. It watches the forward diagonal, so an approaching
inner wall appears while there is still room to react, and the comparison between the
left and right diagonals still gives lane position.

**The cost, written at the top of the distance module.** A reading of D millimetres at
45 degrees is **not** lateral clearance. It is roughly 0.71 times D ahead and 0.71 times
D to the side. Code that treats it as lateral clearance drives the robot into a wall.

### Lane position without knowing the lane width

```
offset = (right - left) / (right + left)
```

Dividing by the total is what makes this work regardless of lane width. Being one third
of the way across a 1 metre lane and one third across a 2 metre lane give the same
answer, so the robot does not need to be told the track dimensions.

### The colour sensor position

Mounted on the underside of the nose, below the camera mount, rather than under the
middle of the chassis.

**Field geometry reason.** Corner lines run across the track. A sensor at the front
crosses the line before the wheels do, which converts into reaction time at exactly the
moment it is needed. Under the middle of the car, the robot is already on the line when
it finds out.

### The IMU position

Set back behind the drive assembly.

**Why this is allowed to be a packaging decision.** Yaw is a property of the whole rigid
body, not of a point on it. The heading reading is identical wherever the sensor is
bolted, so the IMU can go wherever there is flat, rigid space. That freed the crowded
nose for the sensors whose position genuinely matters.

## Calibration Method

Each of these has to be measured on this specific robot. The software cannot derive
them.

### Camera field of view

**Why.** Distance to a sign is computed from its apparent height, and the conversion
depends on the camera's field of view.

**Method.** Place a sign at a measured 50 cm directly ahead. Run the vision module and
read the reported distance. Adjust `CAMERA_HFOV_DEG` until the readout matches. Verify
at 30 cm and 100 cm, since a single point can be matched by a wrong value.

### Colour thresholds

**Why.** The HSV ranges decide whether a sign is seen at all, and they depend on the
lighting in the room.

**Method.** Run the vision module and press **M** for the mask view. Point the camera at
each real object on the real mat. The object should appear solid white in the mask, with
nothing else white. Widen the range until it does, then narrow it until other things
stop appearing.

**Do this under competition lighting, not bench lighting.** This is the calibration most
likely to fail if done in the wrong room.

### Servo centre

**Why.** No two steering linkages sit neutral at the same pulse width.

**Method.** Send a command of zero steering. Look along the car. Adjust `CENTRE_US` until
the wheels are dead straight. Roughly 10 microseconds per degree of error.

### Maximum steering angle

**Why.** The software limit must be the mechanical limit, not the servo's limit.

**Method.** Command full lock in each direction. If the servo buzzes, strains or gets
warm, the linkage is hitting a stop before the servo reaches the commanded angle. Reduce
`MAX_STEER` until it is silent.

### Wheel diameter and gear ratio

**Why.** These two numbers scale every distance the robot calculates.

**Method for the ratio.** Mark the tyre, turn the wheel exactly ten full revolutions by
hand, and divide the pulse count change by ten. Ten turns rather than one so that eye
error on the mark is divided by ten too.

**Method for the diameter.** Measure across the tyre **with the robot's weight on it**,
since rubber compresses. Then roll the robot along a metre stick and compare against the
reported distance, adjusting until they agree.

### Colour sensor gain and integration time

**Why.** Set too high, the clear channel saturates and everything bright reads as white.

**Method.** Point at the brightest surface the robot will see. The clear value should
land well below its maximum. Reduce gain or integration time until it does. This is the
single most common cause of a colour sensor that "always says white".

<!-- TODO: record the measured values here once calibrated, so another team can
     reproduce the robot without repeating the whole process.
| Constant | Measured value | Date |
|---|---|---|
-->

## Failure Points

What can fail electrically, what it would look like, and what we did about it.

| Failure | Symptom | Mitigation |
|---|---|---|
| 5 V rail sags at peak load | Pi reboots mid run, which looks like a software crash | Motor on the unregulated rail, decoupling at the driver, peak-based power budget |
| Sensor supplied at 5 V drives a 3.3 V pin | Works on the bench, microcontroller dies days later | Every sensor output on the 3.3 V rail. See below. |
| One I2C device holds the bus | All six sensors stop responding | Every read is wrapped, and a failed sensor returns nothing rather than raising |
| A driver file missing from the board | All sensors dead with no obvious cause | Driver imports isolated from each other; each reports itself by name |
| Motor brush noise couples into encoder lines | Phantom counts under power, clean by hand | Capacitor across the motor terminals, encoder wiring routed away from motor leads |
| Serial clock drift on the Pi 3 | Corrupted bytes, but only under processor load | Bluetooth disabled to force the stable hardware UART |
| Loose connector | Intermittent sensor dropout | Sensor readings that go missing degrade the robot rather than stopping it |
| Battery sags as it discharges | Robot slows over a session, colour thresholds drift | Monitor run time, recalibrate colour after a voltage change |

### The 5 V fault we caught in review

Our first connection list put the N20 encoder on the 5 V rail.

A magnetic encoder outputs at whatever voltage supplies it. On 5 V, its two channels
would have driven 5 V straight into GP12 and GP13, against an absolute maximum of
3.3 V plus 0.3 V.

**Why this one is worth recording.** It would have worked. The board would have died
days or weeks later, and the symptom would have been sensors quietly reading wrong,
which looks exactly like a software bug. We would have spent that time debugging code.

The encoder now runs from the 3.3 V rail, where it draws a few milliamps.

**The rule we took from it:** trace the logic level path of every sensor output, not just
check that its supply voltage is correct.

### The nSLEEP trap

The DRV8833 has an nSLEEP pin that must be high or the driver stays asleep. It was
missing from our first connection list entirely.

The failure mode is that the motor does nothing while every print statement in the code
reads perfectly correct. It is tied to 3.3 V now.

## Iterations

<!-- TODO: record electrical changes here as they happen. -->

| Change | Reason | Result |
|---|---|---|
| Encoder moved from 5 V to 3.3 V | Would have destroyed the microcontroller | Caught before any hardware was damaged |
| nSLEEP added to the connection list | Driver would never have woken up | |
| Decoupling capacitor added at the driver | Peak current budget showed the rail could sag | |
| Red hue band narrowed from 165 to 172 | Magenta parking markers were read as red signs | Both colours now detected in the same frame |
| Saturation and value floors lowered | The dimmer of two signs was silently dropped | Both signs detected regardless of lighting |
