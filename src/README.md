# `src/` — source code pointer

The WRO GitHub documentation requirement asks for a `/src` folder. This
project keeps its code at the repository root instead, in three folders that
match the import paths used in the code itself:

| Folder | Runs on | Contents |
|---|---|---|
| [`../pico/`](../pico/) | Raspberry Pi Pico 2 W (MicroPython) | 100 Hz control loop, safety interlocks, chip drivers |
| [`../pi/`](../pi/) | Raspberry Pi 5 (CPython 3.11) | Vision, mission state machine, planning, serial master |
| [`../common/`](../common/) | **both** | `protocol.py` — the wire format, deployed verbatim to each board |

The folders are not nested under `src/` on purpose: a judge reading
`from pi.control import WallFollower` in the source can find the file at the
path the import names, with no mental translation step.

Start here:

- [`../README.md`](../README.md) — architecture, wiring, power budget, trade-offs
- [`../config.md`](../config.md) — every tunable and how to calibrate it
- [`../common/protocol.py`](../common/protocol.py) — the Pi ↔ Pico contract
- [`../pi/main.py`](../pi/main.py) — mission state machine
- [`../pico/main.py`](../pico/main.py) — real-time loop and safety interlocks

