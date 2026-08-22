# Models

**Design reasoning, torque and speed calculations, trade-offs and iterations are in
[mechanical-design.md](mechanical-design.md).**

CAD source files and renders for every part we designed and fabricated ourselves.
Organised by subsystem, one folder per part, each containing the model file and an
image of it.

| Folder | Part | Format |
|---|---|---|
| [`front-steering/`](front-steering/) | Ackermann steering assembly | Fusion 360 |
| [`rear-drive/`](rear-drive/) | Motor mount for the rear axle | STEP |
| [`wheels/`](wheels/) | Wheels | Fusion 360 |
| [`body/front_down/`](body/front_down/) | Front lower chassis section | STL |
| [`body/front_top/`](body/front_top/) | Front upper chassis section | STL |
| [`body/back_down/`](body/back_down/) | Rear lower chassis section | STL |
| [`body/back_top/`](body/back_top/) | Rear upper chassis section | STL |
| [`body/camera_mount/`](body/camera_mount/) | Camera mount | STL |
| [`body/battery_mount/`](body/battery_mount/) | Battery holder | STL |

The body is split into four printed sections so each fits the print bed and can be
reprinted on its own without redoing the whole car.

`.f3d` files are Fusion 360 sources and can be edited. `.stl` files are print ready
meshes. `.step` is a neutral CAD format that most packages can open.

See the main [README](../README.md) for how each part fits into the robot and why it is
shaped the way it is.
