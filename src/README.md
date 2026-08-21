# Source Code

All software for the robot, split by the controller it runs on.

| Folder | Board | Language |
|---|---|---|
| [`pi3/`](pi3/) | Raspberry Pi 3 Model B | Python 3 with OpenCV |
| [`pico/`](pico/) | Raspberry Pi Pico 2 W | MicroPython |

## Why the split

The Raspberry Pi runs Linux, which is not a real time operating system. A background
process or a filesystem write can stall a Python loop for tens of milliseconds, which is
invisible when processing an image and disastrous when a servo is waiting for its next
pulse.

So the division is by timing requirement. Anything with a deadline runs on the Pico.
Anything that needs to think runs on the Pi. The two talk over a serial link: the Pi
sends a speed and steering command, the Pico sends back its sensor readings.

## Running the tests

Every module carries a self test that runs on a laptop with no hardware attached. The
MicroPython modules are written so their hardware imports fail harmlessly off the board,
which leaves the logic behind them testable.

```
cd src/pi3
python3 navigation_engine.py --selftest
python3 state_machine.py --selftest
```

Fifteen of the sixteen modules are covered. See each folder's README for detail.
