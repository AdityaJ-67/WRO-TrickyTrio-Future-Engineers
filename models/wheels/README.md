# Wheels

<img src="wheels_image.png" width="400">

Custom wheels, sized to keep the chassis low while leaving enough ground clearance for
the mat surface.

**Why the diameter is a measured value, not a design value.** Wheel diameter feeds
directly into every distance the robot calculates, because distance travelled is
revolutions multiplied by circumference. Rubber compresses under load, so the rolling
diameter is always slightly smaller than the modelled one. We measure it with the
robot's weight on the wheels and use that figure in the code, rather than taking the
number from the CAD file.

| File | Format | Notes |
|---|---|---|
| `wheels.f3d` | Fusion 360 | Editable source |

<!-- TODO: fill in
| Property | Value |
|---|---|
| Modelled diameter | |
| Measured rolling diameter, under load | |
| Material | |
| Tyre material | |
| Layer height | |
| Infill | |
-->
