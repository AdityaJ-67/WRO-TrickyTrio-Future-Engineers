#!/bin/sh
# Copy the Pico code to the board.
#
# MicroPython has no package imports here - every module does `import distance`,
# `import servo` and so on - so everything lands FLAT at the board's root,
# regardless of the folders used in this repo.
#
#   ./deploy.sh              copy the flight code
#   ./deploy.sh --drivers    also copy the third-party drivers from ./drivers/
#
# Needs mpremote:  uv pip install mpremote
set -e

MPREMOTE="${MPREMOTE:-../../.venv/bin/mpremote}"
cd "$(dirname "$0")"

echo "Copying flight code..."
$MPREMOTE cp main.py motionController.py servo.py drv8833.py encoder.py :
$MPREMOTE cp sensors/sensorManager.py sensors/distance.py sensors/imu.py sensors/colour.py :

if [ "$1" = "--drivers" ]; then
    echo "Copying drivers..."
    # vl53l0x.py, bno08x.py and tcs34725.py are third party - fetch them once
    # into drivers/ and they get copied with everything else.
    $MPREMOTE cp drivers/*.py :
fi

echo
echo "On the board:"
$MPREMOTE ls
echo
echo "Missing a driver? Each sensor reports it by name at startup and the rest"
echo "of the robot keeps running. Fetch it into drivers/ and run --drivers."
