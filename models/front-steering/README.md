# Front Steering Assembly

<img src="ackermann_image.png" width="400">

The Ackermann steering assembly. Holds both front wheels, the steering knuckles, and
the track rod that links them to the servo horn.

**Why Ackermann.** When a car turns, the inside wheel follows a tighter circle than the
outside wheel. Holding both front wheels parallel would force one of them to scrub
sideways through every corner, wasting energy, wearing the tyre and making the turn
unpredictable. Ackermann geometry angles the steering arms so the inside wheel turns
through a larger angle than the outside one, letting both roll cleanly around a shared
turning centre.

That matters here for two reasons. The corners on this course are tight, so the
difference between the two wheel angles is significant rather than negligible. And our
odometry comes from the driven rear wheels, so anything that makes the front wheels
scrub introduces error into how far the robot thinks it has travelled.

This part had the tightest tolerances of anything we designed, since the steering arm
geometry is what produces the angle difference.

| File | Format | Notes |
|---|---|---|
| `Ackerman System vinfinity.f3d` | Fusion 360 | Editable source |

<!-- TODO: fill in
| Property | Value |
|---|---|
| Material | |
| Layer height | |
| Infill | |
| Steering range achieved | |
-->
