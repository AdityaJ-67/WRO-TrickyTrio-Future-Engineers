# Reproducing This Robot

Everything another team needs to build and run this robot from scratch.

Covers rubric criterion 5. If a step here is unclear, that is a bug in this document.

## Contents

1. [What you need](#what-you-need)
2. [Build the mechanics](#build-the-mechanics)
3. [Wire the electronics](#wire-the-electronics)
4. [Set up the Raspberry Pi](#set-up-the-raspberry-pi)
5. [Set up the Pico](#set-up-the-pico)
6. [Calibrate](#calibrate)
7. [Testing workflow](#testing-workflow)
8. [Version history](#version-history)

## What you need

### Electronics

| Part | Quantity | Notes |
|---|---|---|
| Raspberry Pi 3 Model B | 1 | With a micro SD card, 16 GB or more |
| Raspberry Pi Pico 2 W | 1 | The RP2350 board, not the older RP2040 Pico W |
| Camera Module 3 | 1 | Plus a **15 pin** ribbon cable for the Pi 3 |
| GA12-N20 gear motor with encoder | 1 | 6 V. Record the gear ratio you buy. |
| MG90S servo | 1 | Metal geared |
| DRV8833 motor driver | 1 | |
| VL53L0X distance sensor | 4 | |
| TCA9548A I2C multiplexer | 1 | |
| BNO085 IMU | 1 | |
| TCS34725 colour sensor | 1 | |
| Mini560 buck converter | 1 | 5 V output |
| 2S LiPo battery | 1 | 7.4 V, 1500 mAh, 35C |
| 4.7 kohm resistors | 2 | I2C pull-ups |
| 100 uF electrolytic capacitor | 1 | Across the motor driver supply |
| 0.1 uF ceramic capacitor | 1 | Across the motor terminals |

Full parts list with sourcing in
[`schemes/Main Electrical Components.xlsx`](schemes/Main%20Electrical%20Components.xlsx).

### Printed parts

All in [`models/`](models/), one folder per part with the CAD file and a render.

<!-- TODO: record your print settings so another team can match them.
| Setting | Value |
|---|---|
| Material | |
| Layer height | |
| Infill | |
| Supports | |
-->

### Tools and software

A 3D printer, a soldering iron, and calipers. On your computer: Python 3 with OpenCV and
pyserial, and `mpremote` for the Pico.

## Build the mechanics

1. Print the four body sections, the camera mount and the battery mount from
   [`models/body/`](models/body/).
2. Print or machine the steering assembly from
   [`models/front-steering/`](models/front-steering/) and the motor mount from
   [`models/rear-drive/`](models/rear-drive/).
3. Assemble the Ackermann linkage. Check both front wheels rotate freely and that the
   inside wheel turns through a larger angle than the outside one when steered.
4. Fit the motor and rear axle. Check for play between the motor shaft and the wheels,
   since any flex here becomes odometry error.
5. Mount the camera and, directly beneath it on the underside, the colour sensor.

Reasoning behind each part is in
[`models/mechanical-design.md`](models/mechanical-design.md).

## Wire the electronics

Follow the master connection list in [`schemes/README.md`](schemes/README.md) exactly.
The sensor layout is drawn in
[`schemes/sensor-placement.svg`](schemes/sensor-placement.svg).

**Three things that will cost you a board or a day if you get them wrong.**

Every sensor output must be on the **3.3 V** rail. The RP2350 is not 5 V tolerant, and a
sensor supplied at 5 V outputs 5 V logic. This kills the board days later and looks like
a software bug.

The DRV8833's **nSLEEP** pin must be high. Otherwise the motor does nothing while every
print statement reads correct.

**Grounds must be common.** The battery ground, the 5 V ground and the logic ground are
one net.

Full reasoning in [`schemes/power-and-sensors.md`](schemes/power-and-sensors.md).

## Set up the Raspberry Pi

```bash
sudo apt install python3-opencv python3-pip
pip install pyserial
```

Then move the serial port off the mini UART, which is a **Pi 3 specific** step:

```bash
sudo nano /boot/firmware/config.txt     # add: enable_uart=1
                                        #      dtoverlay=disable-bt
sudo systemctl disable hciuart
sudo raspi-config                       # Serial: login shell NO, hardware YES
sudo reboot
ls -l /dev/serial0                      # must point at ttyAMA0, not ttyS0
```

Skipping this gives a link that works at idle and corrupts bytes the moment the vision
code loads the processor.

## Set up the Pico

Flash MicroPython, using the **RP2350** build for the Pico 2 W. Hold BOOTSEL, plug in
USB, drag the `.uf2` file onto the drive that appears.

Then fetch the three sensor drivers into `src/pico/drivers/`:

<!-- TODO: record the exact driver sources you used, so another team gets the same
     versions. The modules import them as vl53l0x, tcs34725 and bno08x. -->

| Driver | Import name | Source |
|---|---|---|
| VL53L0X | `vl53l0x` | *record the URL and version* |
| TCS34725 | `tcs34725` | *record the URL and version* |
| BNO08x | `bno08x` | *record the URL and version* |

Then deploy:

```bash
cd src/pico
./deploy.sh --drivers
```

Everything lands flat at the board's root, because MicroPython's `import distance` does
not know about folders.

## Calibrate

Every one of these is specific to your build. Full method for each in
[`schemes/power-and-sensors.md`](schemes/power-and-sensors.md).

| Constant | File | Measure by |
|---|---|---|
| `GEAR_RATIO` | `encoder.py` | Turn the wheel ten revolutions, divide the pulse change by ten |
| `WHEEL_DIAMETER_MM` | `encoder.py` | Calipers, with the robot's weight on the wheel |
| `CENTRE_US` | `servo.py` | Adjust until the wheels are dead straight |
| `MAX_STEER` | `servo.py` | Reduce until the servo no longer strains at full lock |
| `CAMERA_HFOV_DEG` | `vision_test.py` | Sign at a measured 50 cm, adjust until the readout agrees |
| `ROI_TOP` | `vision_test.py` | Set once the camera is mounted |
| `COLOUR_RANGES` | `vision_test.py` | Mask view, on the real mat, under competition lighting |
| `MAGENTA_RANGE` | `vision_test.py` | Same, on the real parking markers |

Two direction flags are set by observation, not by rewiring. `STEER_DIRECTION` if the
servo turns the wrong way, `ENCODER_DIRECTION` if forward counts down. If the motor spins
backwards, swap the two motor wires.

## Testing workflow

Work up the levels. Do not skip.

### Level 1: self tests, no hardware

Every module carries one. They run on a laptop in milliseconds.

```bash
cd src/pi3
python3 camera_vision/vision_test.py --selftest
python3 navigation_engine.py --selftest
python3 state_machine.py --selftest
python3 mission_manager.py --selftest
python3 main.py --selftest

cd ../pico/sensors
PYTHONPATH=.. python3 -c "
import distance, imu, colour, sensorManager, servo, drv8833, encoder, motionController
for m in (distance, imu, colour, sensorManager, servo, drv8833, encoder, motionController):
    m.selftest()"
```

All fifteen should print `selftest ok`. **Run these after every change.**

### Level 2: vision on a laptop

```bash
python3 main.py --dry-run --show
```

Webcam and full decision logic, no robot. Hold coloured objects in front of the camera
and watch the detections and the chosen behaviour.

### Level 3: bench, wheels off the ground

One subsystem at a time, in this order. Set `MAX_SPEED = 40` first.

1. Servo centres straight, reaches both limits without straining
2. Motor spins the correct way, and **stops from every speed**
3. Encoder counts one turn correctly, counts down in reverse
4. All four distance sensors respond, and the right one changes when you block it
5. IMU heading changes correctly when the robot is turned
6. Colour sensor distinguishes the real mat surfaces
7. Serial link survives five minutes with the vision code loading the Pi

Record results in
[`other/engineering-journal/test-logs.md`](other/engineering-journal/test-logs.md).

### Level 4: field

Full runs on a mat. Log every attempt, including the failures, especially the failures.

## Version history

<!-- TODO: tag releases as milestones are reached, and record them here. The rubric
     asks for "versioning or release notes". A tag before each competition round is
     the minimum worth doing. -->

| Version | Date | What changed |
|---|---|---|
| | | |

To tag a version:

```bash
git tag -a v1.0 -m "First complete software stack, all self tests passing"
git push --tags
```
