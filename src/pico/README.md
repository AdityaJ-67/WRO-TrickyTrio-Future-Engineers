# Pico 2 W Source

MicroPython. Everything here has a deadline: the motor, the steering servo, the wheel
encoder and the six sensors.

## Files

| File | What it does |
|---|---|
| `main.py` | Flight program. Reads commands, drives the robot, reports sensor state. Runs automatically on power up. |
| `motionController.py` | One interface for movement: `move(speed, steering)` and `stop()`. |
| `servo.py` | MG90S steering, with clamped angle and pulse limits. |
| `drv8833.py` | Drive motor through the DRV8833. |
| `encoder.py` | Wheel odometry from the quadrature encoder, counted on an interrupt. |
| `sensors/sensorManager.py` | Gathers every sensor reading into one dictionary. |
| `sensors/distance.py` | Four VL53L0X distance sensors behind the multiplexer. |
| `sensors/imu.py` | BNO085 orientation. |
| `sensors/colour.py` | TCS34725 floor colour. |
| `uart_echo.py` | Bench tool for testing the serial link on its own. |
| `deploy.sh` | Copies everything to the board. |

## The loop

The flight program runs at two speeds on purpose. Commands are read and applied every
20 ms, because a command that waits is a robot that has already driven somewhere else.
A full sweep of the four distance sensors costs about 130 ms, so that happens on its own
slower schedule and never delays the motor.

If no command arrives for 500 ms, meaning the Pi has crashed or the cable has come
loose, the watchdog stops the robot. This is the main reason the control system is split
across two boards.

## Deploying

MicroPython has no package imports here, so every file lands flat at the board's root
regardless of the folders used in this repository.

```
./deploy.sh              copy the flight code
./deploy.sh --drivers    also copy the sensor drivers
```

<!-- TODO: the sensor drivers are not in this repository yet. The modules import
     vl53l0x, tcs34725 and bno08x by name. Without them the sensors will not read,
     though each module reports which file is missing and the robot keeps running. -->

## Testing

```
python3 -c "import servo; servo.selftest()"
```

Each module's self test runs on a laptop with no board attached.
