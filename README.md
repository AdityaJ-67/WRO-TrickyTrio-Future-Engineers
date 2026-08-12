# WRO-TrickyTrio-Future-Engineers

Members: Aditya Raj Juneja, Ishayu Datta, Sarjas Gauhar Singh


A self-driving model car for World Robot Olympiad 2026 Future Engineers, built by 
a three-person team, age 15, competing at WRO India in Hyderabad.

The car drives three laps of a randomised track using time-of-flight distance
sensors to stay centred, a downward-facing colour sensor to know when it has
turned a corner, and a camera to identify red and green pillars and pass them on
the correct side — then finds a parking space and reverse-parks into it. All
autonomously, with no wireless link, inside three minutes.


## How the car works

### The two challenges

**Open Challenge** — three laps of a rectangular track with randomly placed
walls and no obstacles. The gap between the inner and outer walls is either
600 mm or 1000 mm, decided per round, so nothing about the width can be assumed.

**Obstacle Challenge** — the same three laps, but with red and green pillars
placed on the straights. **Red means pass on the pillar's right; green means
pass on its left.** After three laps the car must find a parking bay marked by
two magenta posts and parallel park into it.

### The four sensing jobs

The principle the whole design follows: **no sensor is asked to do a job another
one does better.**

| Question | Sensor | Why that one |
| --- | --- | --- |
| *Where am I in the lane?* | 5 × VL53L0X laser distance | Measures the walls directly. Immune to lighting, and works at any track width without being told what the width is. |
| *Have I turned a corner?* | TCS34725 colour sensor, facing down | The corners are marked by 20 mm orange and blue lines painted on the floor. A colour read takes about a millisecond and the line under the car is never occluded. |
| *Which pillar is that, and which side do I pass it?* | Camera Module 3 | The only sensor that can tell red from green at a distance. |
| *Which way am I pointing?* | BNO085 IMU | The only thing that still means anything mid-corner, when the walls are not a useful lane reference. |

### Staying centred without knowing the track width

The two side-facing distance sensors give `front_left` and `front_right`. The
steering error is simply `front_left - front_right`, which is **zero in the middle of the lane no matter how
wide the lane is**. One controller handles both the 600 mm and the 1000 mm track
with nothing to configure and nothing to get wrong on the day.

When only one wall is in range — common on the wide track, where the black walls
absorb the sensor's infrared beyond about 900 mm — the car holds **half the
estimated lane width** from the wall it can see. Not a fixed standoff: 400 mm
centres the car on a 1000 mm lane but leaves 200 mm on the far side of a 600 mm
one, which was enough to put it into a wall in simulation. When neither is visible, which means it is
mid-corner looking at open space, it falls back to holding a compass heading
from the IMU.

### Counting laps

Not from a timer, and not by counting steering commands. The car counts physical
crossings of the painted corner lines: **four corners to a lap, twelve to
finish.** That counts the thing the rules actually define a lap by.

The colour sensor normalises each channel by the **clear** channel and then
classifies on the **ratios between them**, never on absolute brightness. That
removes the illumination level entirely: arena lighting is not classroom
lighting, but the ratio between channels is a property of the printed ink and
survives the move.

A crossing starts a debounce window, because a 20 mm line at driving speed is
under the sensor for about 40 ms and the car can re-cross the same paint while
turning. Without that window, one corner gets counted three times and the car
finishes its third lap somewhere in the middle of its second.

There is also a backstop: if the front sensor says a wall is imminent and no
line has fired, the car takes the corner anyway. A missed line is survivable; a
wall is not. Three guards stop that backstop from firing on things that are not
corners — it is suppressed briefly after a turn (coming out of one, the car is
still pointing near a wall), when the car is slewed more than 25° off its target
heading (that is our own drift, not a corner), and when the camera says the close
thing ahead is a pillar. Each of those was a crash in simulation before it
existed.

### Passing the pillars

The camera converts to HSV, keeps pixels whose hue falls in the red or green
band, groups them into blobs, and discards blobs of the wrong size, shape or
solidity — which removes reflections, floor markings, and the red jacket in the
crowd. What is left is a pillar, with a bearing in degrees and a rough distance
from how tall it appears.

**HSV rather than RGB** because HSV separates *which colour* from *how bright*,
so the thresholds can accept a wide brightness range while staying narrow on
hue. **Classical thresholding rather than a neural network** because it runs in
a few milliseconds with the same timing on every frame, and because we can
explain it completely in a judging interview.

Avoidance is expressed as a **shift of the lane-centering setpoint**, not as a
second steering controller. A pillar does not change *how* the car steers, only
*where in the lane it wants to be*, so the offset is handed to the same PID that
does ordinary wall following. Two controllers fighting over one servo is a bug
that only appears when both are active — exactly when you cannot afford it.

The offset is built from the geometry rather than guessed: half the 85 mm
no-move circle, plus half the car's width, plus a margin. Red gives a positive
(rightward) offset, green a negative one. The rules only penalise actually
*moving* a pillar out of its circle, but relying on that tolerance means every
small steering error costs points.

A camera detection is also cross-checked against the forward distance sensors
before the car acts on it. The camera says *what and where*; the ToF says *how
far*. A red blob at 800 mm with nothing in front of the car at 800 mm is a
reflection or a spectator, and swerving for it costs us the lane.

Once a pillar is closer than about 250 mm it has dropped below the camera's
view, and the car **commits** — it holds the steering it already has rather than
reacting to a pillar it can no longer see properly.

### Parking

Drive along the wall until a side sensor reads suddenly long — that gap is the
bay. Reverse on full lock until the car has swung about 40° into it, counter-
steer to bring the tail round, straighten against the IMU heading, and stop.

If it has not worked within 35 seconds, the car **stops cleanly wherever it is**.
That is a designed outcome, not a failure: a car still shuffling when the clock
expires risks pushing a boundary post out of place, and a knocked post costs
more than an unparked finish.

---

## Architecture

Two boards, split by what each is actually good at.

```
        ┌──────────────────────────┐         ┌──────────────────────────┐
        │   Raspberry Pi 5         │  UART   │   Pico 2 W               │
        │   Python 3 + OpenCV      │◄───────►│   MicroPython            │
        │                          │ 115200  │                          │
        │   • camera capture       │         │   • 5x ToF (via mux)     │
        │   • HSV pillar detection │         │   • colour sensor        │
        │   • state machine        │         │   • IMU (UART-RVC)       │
        │   • steering + speed PID │         │   • motor + servo        │
        │   • run logging          │         │   • encoder              │
        │                          │         │   • SAFETY CUTOFFS       │
        └──────────────────────────┘         └──────────────────────────┘
```

**Why split at all:** Linux is bad at microsecond-accurate PWM and interrupt
timing; a microcontroller cannot run OpenCV. Each board does what it is good at.

**Why the safety logic is on the Pico:** it is the board that already has the
distance reading in a variable. Waiting for the Pi's capture → threshold →
contour → decide → transmit cycle costs tens of milliseconds, during which the
car is still moving.

The Pico owns **no strategy at all**. It does not know what a lap is or what a
pillar is. It reads, reports, obeys, and stops. That is what keeps it small
enough to reason about completely.

They talk over a versioned, checksummed, human-readable text protocol,
documented in full at [docs/uart_protocol.md](docs/uart_protocol.md). The codec
is written once, in dependency-free Python that runs unmodified on **both**
CPython and MicroPython, and is copied to the Pico at flash time — so the two
boards cannot disagree about the wire format.

### The control loop

```
Pico, 100 Hz:   colour → one ToF → IMU → send telemetry → read commands → apply → safety
Pi,   ~30 Hz:   read telemetry → maybe a frame → decide → send → log
```

The Pico reads the colour sensor **every** iteration but only **one** distance
sensor, round-robin. That asymmetry is the most important timing decision in the
firmware: a ranging takes ~20 ms, so reading all four would give a 12 Hz loop,
at which the car would drive straight over the 40 ms corner lines without seeing
them. Reasoning in [docs/uart_protocol.md](docs/uart_protocol.md) §5.

---


### The two-rail power design

The most consequential electrical decision on the car, and a correction to our
original single-rail plan.

A Raspberry Pi 5 wants a 5 V / 5 A-capable supply and **brown-outs silently**
when its rail sags — no error, just a reboot. The MG90S servo draws a current
spike **every time the car steers**. Put both on one 2–3 A converter and the
Pi's supply dips at the exact moment the car is cornering. The symptom is "the
robot randomly freezes mid-run", it is intermittent, it gets worse as the
battery sags, and it looks exactly like a software bug.

So the car has **two separate buck converters**: a dedicated 5 V / 5 A unit
feeding only the Pi 5, and the Mini560 feeding the Pico, servo and sensors. One
shared ground, 100 µF of local decoupling at both the servo and the Pi's input,
and the motor running straight off the raw battery through the DRV8833 so the
biggest, spikiest load never touches a regulated rail
