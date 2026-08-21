# Rear Drive Mount

<img src="rear_mount_image.png" width="400">

Holds the GA12-N20 gear motor rigidly in line with the rear axle.

**Why rigidity matters more than it looks.** The encoder counts revolutions of the
motor shaft, but what actually moves the robot is the wheel. Any flex or play between
the two shows up as a mismatch between the distance the robot calculates and the
distance it travels. Since our navigation uses that distance to judge how far it has
gone, a soft mount quietly degrades everything downstream.

| File | Format | Notes |
|---|---|---|
| `rear_motor_mount.step` | STEP | Neutral CAD format, opens in most packages |

<!-- TODO: fill in
| Property | Value |
|---|---|
| Material | |
| Layer height | |
| Infill | |
| Fasteners used | |
-->
