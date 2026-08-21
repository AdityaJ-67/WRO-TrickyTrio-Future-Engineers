# Obstacle Strategy

How the robot decides which way to go around a traffic sign.

## The Rule

Red signs are passed on their right. Green signs are passed on their left. Getting this
backwards is worse than not seeing the sign at all, because the robot then steers
deliberately into the thing it was meant to avoid.

## One Calculation, Not Nine Cases

The obvious way to write this is a set of cases: red on the left, red in the centre,
red on the right, then the same three for green. That is nine branches once you include
"no sign", and every one of them is a place to make a mistake.

Instead, each colour is given a **target position in the camera frame**.

Passing a red sign on its right means the sign has to end up on the robot's **left**, so
the target for red is a position well to the left of frame centre. Green is the mirror
image. The robot then steers by however far the sign currently is from where it should
be.

That single rule produces all of the behaviour below, with no special cases at all.

| Red sign position in frame | What the robot does |
|---|---|
| Far left | Nothing. It is already clear. |
| Just left of centre | Nothing. It has just cleared. |
| Dead centre | Firm right. This is the dangerous position, because the robot is aimed straight at it. |
| Right of centre | Harder right. |
| Far right | Full lock right. This is the worst case, because the sign is on the wrong side entirely. |

Note that a centred sign produces a strong correction while a sign far to one side
produces none. That is the opposite of what a naive "steer away from the obstacle"
rule would do, and it falls out of the target position idea without being written
anywhere as a special case.

## One Safety Clamp

A red sign may only ever produce right steering, and a green sign may only ever produce
left steering.

Without this, a sign already cleared to the left would produce a small correction back
towards the thing the robot just avoided. That is how you clip one.

## Choosing Between Two Signs

Both colours are often visible at once, typically a red one close and a green one
further down the track.

The **nearer** sign is the one about to be hit, so that is the one the robot steers
around. The far one is handled on later frames, once it has become the near one.

Where the vision module has measured a distance, we rank by that. Where it has not, we
fall back to apparent size, since a nearer sign of the same real size looks bigger. The
order signs happen to appear in the detection list makes no difference to the outcome.

## The Four Phase Pass

A pass is not a single decision. The robot moves through four phases:

`FOLLOW_COURSE` then `APPROACH_PILLAR` then `PASS_PILLAR` then `RECENTER` then back to
`FOLLOW_COURSE`.

**Approach** begins as soon as a sign is seen and is far enough away to line up gently.
Speed drops here, because a slower approach makes the correction smaller.

**Pass** begins once the sign is close, and commits to the path even as the sign drifts
towards the edge of the frame and eventually out of it.

**Recentre** is the phase that stops the robot living permanently offset, and it is the
one we did not have at first. Our early version steered around a sign and simply carried
on. Each obstacle left the robot a little further from the lane centre than the last,
and three signs later it clipped a wall. Now, once the sign has been out of view for
half a second, steering ignores signs entirely and tracks the lane centre until the
offset is back inside a dead zone.

There is a deliberate asymmetry in this sequence. The passing phase has **no route into
recovery**. Losing sight of a sign you are squeezing past is exactly what success looks
like, so both of its exits lead to recentring instead.

## Edge Cases

Each of these came from watching the robot behave oddly, not from planning ahead.

| Case | How it is handled |
|---|---|
| Sign flickers out for a single frame | Half a second of confirmation before the pass is declared over |
| Two signs in view at once | Nearest by measured distance wins, and frame order is irrelevant |
| Sign appears in the middle of a corner | The corner outranks the sign, because the transition table row order says so |
| Blob smaller than the minimum area | Treated as noise, not as a sign |
| Sign jittering around the frame centre | A dead zone snaps it to exactly centred, so the wheels do not twitch |
| A distance sensor returns no reading | Speed drops to slow rather than cruising blind |
| Nothing detected at all | Lane following from the two diagonal sensors |
| A red jacket in the audience | Fill ratio test rejects it. A real sign fills its bounding box; a person does not. |

## What Still Needs Field Testing

<!-- TODO: fill in once the robot has run on a real mat -->

- The steering gain, which controls how hard the robot corrects
- The distance at which the approach phase begins
- Whether half a second is the right confirmation delay for a lost sign
- Behaviour when two signs of the same colour appear in sequence
