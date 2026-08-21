# Iteration Log

Problems we hit, what actually caused them, and what we changed. Ordered roughly as
they happened.

The pattern worth noticing is how often the visible symptom pointed somewhere other
than the real cause.

---

**Only one of two traffic signs was detected**

*Symptom.* With a single sign on the mat, detection was perfect. With two, one of them
was frequently missed, seemingly at random.

*Cause.* Our saturation and brightness floors were set too high. Two signs on a mat are
never lit equally, and the one further from the lights fell below the threshold. It
looked random because it depended on where the robot happened to be standing.

*Fix.* Floors lowered substantially. A sign in shadow is much darker than one under the
lights, and a high floor silently discards it rather than reporting a problem.

---

**The robot clipped signs it had already passed**

*Symptom.* The robot would steer correctly around a sign, then swing back and catch it
with the rear of the chassis.

*Cause.* Once the sign was cleared to one side, the steering calculation produced a
small correction back towards it.

*Fix.* Clamped the pass direction per colour, so a red sign can only ever produce right
steering and a green one only left.

---

**The robot drifted further off centre with each sign**

*Symptom.* First sign passed cleanly. Second slightly awkwardly. Third ended with the
robot against a wall.

*Cause.* Nothing brought the robot back to the lane centre after a pass. Each avoidance
left it offset, and the offsets accumulated.

*Fix.* Added a dedicated recentre phase that ignores signs and tracks the lane centre
until the robot is back on line.

---

**The robot gave up in the middle of a corner**

*Symptom.* Robot enters a corner, stops facing the wall, waits, then abandons the run.

*Cause.* This one took a while. Turning towards a wall means the front distance sensor
keeps closing, which triggered the emergency stop. But a stopped robot stops turning, so
the corner never completed, so the state timed out into recovery, and recovery could not
clear either because the wall was still there. A chain of individually correct
behaviours producing a dead end.

*Fix.* A minimum speed for the cornering state, so the robot creeps through rather than
freezing. The wall ahead during a corner is the wall being turned away from.

---

**Parking finished on a timer rather than on arrival**

*Symptom.* The robot parked, but always after exactly the same number of seconds,
regardless of where it actually was.

*Cause.* The stop condition watched the rear distance sensor. But the camera faces
forward, so the slot is only visible while driving towards it, meaning the robot enters
nose first. The rear sensor was pointed back at the open mat it had come from and never
triggered.

*Fix.* Watch the front sensor instead, and only while parking, since a close wall
anywhere else on the track means something different.

---

**The parking stop was physically unreachable**

*Symptom.* After the previous fix, the robot stopped short of the slot.

*Cause.* Navigation performs an emergency stop at 150 mm, but the parking trigger sat at
120 mm. The robot froze before it could ever reach its own trigger.

*Fix.* A minimum speed for the entry phase, and a threshold set below the emergency stop
line. We also added an automatic check that enforces that ordering, so if either number
is changed carelessly in future the mistake is caught immediately rather than on the
mat.

---

**The encoder would have destroyed the microcontroller**

*Symptom.* None. This was caught reviewing the connection list, not from behaviour.

*Cause.* The encoder was on the 5 V rail, and a magnetic encoder outputs at whatever
voltage supplies it. That would have put 5 V into pins rated for 3.3 V.

*Fix.* Moved to the 3.3 V rail. Recorded in full in [design-decisions.md](design-decisions.md).

---

**Magenta parking markers were read as red traffic signs**

*Symptom.* The robot treated the parking lot as a giant traffic sign and tried to pass
it on the right.

*Cause.* Magenta sits immediately below red on the hue circle, and our red range started
low enough to swallow it entirely.

*Fix.* Narrowed the red range so it starts higher. This costs nothing, because red's
main band sits at the other end of the hue scale.

---

**Speed flapped between cruise and slow**

*Symptom.* Approaching a wall, the robot audibly surged and slowed several times a
second.

*Cause.* A single distance threshold with a slightly noisy sensor reading. Readings
either side of the threshold flipped the mode on alternate frames.

*Fix.* Hysteresis. Separate thresholds for entering and leaving the slow state, so a
reading hovering near the boundary no longer causes a change.

---

**Stall detection never fired**

*Symptom.* The robot could be held stationary indefinitely without noticing.

*Cause.* The timer that tracks how long the robot has been stalled treated a timestamp
of zero as "not set", so at the very start of a run it reset itself every frame.

*Fix.* An explicit check for whether the value was set, rather than relying on whether
it was non-zero. Worth recording because it would only ever have shown up in the first
seconds of a run.
