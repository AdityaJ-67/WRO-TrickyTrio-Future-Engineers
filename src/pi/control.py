"""
Lateral and heading control for the Pi 5 "Brain".

ONE STEERING CONTROLLER. THAT IS THE WHOLE DESIGN.
--------------------------------------------------
Every driving behaviour in this vehicle -- straight-line wall following,
pillar avoidance, lane changes, the approach to the parking bay -- is the
SAME PD controller with a different lateral SETPOINT. There is deliberately
no second steering path anywhere in this codebase.

Why that matters beyond tidiness: two controllers means two sets of gains,
two failure modes, and a hand-off moment where the car is being steered by
neither of them properly. Pillar avoidance implemented as "a different way
of steering" is how a car swerves cleanly round a pillar and then snaps into
the wall when the avoidance releases. Implemented as "the same controller,
setpoint shifted 140 mm right", the transition is a ramp on one number and
the controller never notices anything happened.

The only exception is the 90 degree corner, which is a genuinely different
regime: there is no lateral reference to follow because the front wall IS
the wall we are following. TurnController owns that, it is closed on IMU yaw
alone, and the two are SEQUENCED, never blended -- see pi/main.py.

SIGN CONVENTIONS (get these wrong and the car drives into things)
-----------------------------------------------------------------
  steer     : centi-degrees, POSITIVE = RIGHT
  heading   : degrees, POSITIVE = counter-clockwise (LEFT), from the IMU
  lateral   : millimetres, POSITIVE = RIGHT of the lane centreline
  offset    : the setpoint, same frame as lateral
Because heading is CCW-positive and steer is CW-positive, the heading term
enters with a POSITIVE sign on (current - target): if we have drifted left
of our target heading, we steer right to come back.
"""

import math


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def wrap180(a):
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


# =============================================================================
class LaneEstimator:
    """Tracks the live lane width.

    In the Open Challenge the inner walls are placed randomly, so the lane is
    anywhere from 600 to 1000 mm and CHANGES between sections. A fixed
    assumption is wrong by up to 400 mm, which is most of a lane -- so both
    the single-wall follower and the corner trigger scale off this estimate.

    Only updated when BOTH walls answer. On black walls that is a minority of
    samples, which is exactly why the filter is slow: we want the good
    samples to accumulate, not to chase the dropouts.
    """

    def __init__(self, nominal_mm=1000, alpha=0.1, lo=600, hi=1000):
        self.width_mm = float(nominal_mm)
        self.alpha = alpha
        self.lo = lo
        self.hi = hi
        self.updates = 0

    def update(self, left_mm, right_mm, robot_width_mm):
        if left_mm is None or right_mm is None:
            return self.width_mm
        w = left_mm + right_mm + robot_width_mm
        # Reject physically impossible widths outright. A single bad pair
        # (say, the left sensor catching a pillar) would otherwise drag the
        # estimate for the next several seconds.
        if not (self.lo * 0.8 <= w <= self.hi * 1.25):
            return self.width_mm
        self.width_mm += self.alpha * (w - self.width_mm)
        self.updates += 1
        return self.width_mm

    @property
    def half(self):
        return self.width_mm * 0.5


# =============================================================================
class WallFollower:
    """PD lateral controller with an IMU heading-hold term.

    WHY PD AND NOT PID: our straights are ~3 m and last about 4 seconds at
    cruise. An integrator cannot converge usefully inside 4 s, but it can
    absolutely accumulate during the 1.2 s corner and then fire the car at
    the next wall as it unwinds. There is no steady-state error worth
    integrating away here -- the heading-hold term already removes the
    constant-drift case, which is what an I term would otherwise be for.

    WHY A HEADING TERM AT ALL: wall following on distance alone is a pure
    position loop with no direct angle feedback, so it converges by weaving
    -- the car crosses the centreline, over-corrects, crosses back. Adding
    the IMU heading error damps that into a clean approach. It also gives us
    something to steer on when BOTH walls drop out, which on black walls at
    a 1000 mm lane happens regularly.
    """

    def __init__(self, cfg):
        wf = cfg["wall_follow"]
        self.kp = float(wf["kp_cdeg_per_mm"])
        self.kd = float(wf["kd_cdeg_per_mm_s"])
        self.kh = float(wf["heading_hold_kp_cdeg_per_deg"])
        self.max_steer = int(wf["max_steer_cdeg"])
        self.far_sat_mm = float(wf["far_wall_saturation_mm"])
        self.d_alpha = self._alpha_from_hz(float(wf["d_lowpass_hz"]), 0.02)

        self._prev_err = None
        self._d_filt = 0.0
        self.last_error_mm = 0.0
        self.last_lateral_mm = 0.0
        self.last_source = "none"

    @staticmethod
    def _alpha_from_hz(hz, dt):
        tau = 1.0 / (2.0 * math.pi * max(hz, 0.1))
        return dt / (tau + dt)

    def reset(self):
        """Call on every state entry. Carrying a derivative across a state
        change means the first tick of the new state sees a step of tens of
        millimetres and kicks the servo."""
        self._prev_err = None
        self._d_filt = 0.0

    def usable(self, mm):
        """A range is usable only if it came back AND is inside the distance
        at which these black walls still return signal. Beyond
        far_wall_saturation_mm the sensor is reporting 'nothing came back',
        which is information about the SIGNAL, not about the distance."""
        return mm is not None and mm <= self.far_sat_mm

    def lateral_position(self, left_mm, right_mm, lane, robot_half_w):
        """Estimate our lateral offset from the lane centreline.

        Returns (position_mm, source). POSITIVE = right of centre.
        Three cases, in descending order of trust -- and crucially, the
        'neither wall' case returns None rather than a guess, so the caller
        can fall back to dead reckoning and FLAG it rather than steering on
        a fabricated position.
        """
        l_ok = self.usable(left_mm)
        r_ok = self.usable(right_mm)
        half = lane * 0.5

        if l_ok and r_ok:
            return (left_mm - right_mm) * 0.5, "both"
        if l_ok:
            # We know where the left wall is; centre is half a lane to its
            # right. If we are further from it than half a lane, we are right
            # of centre.
            return (left_mm + robot_half_w) - half, "left"
        if r_ok:
            return half - (right_mm + robot_half_w), "right"
        return None, "none"

    def compute(self, left_mm, right_mm, lane_mm, robot_half_w,
                offset_mm, heading_deg, target_heading_deg, dt_s):
        """Returns steer in centi-degrees, POSITIVE = right.

        `offset_mm` is the setpoint: 0 = lane centre, +140 = 140 mm right of
        centre. Pillar avoidance is nothing more than a ramped value here.
        """
        pos, source = self.lateral_position(left_mm, right_mm, lane_mm,
                                            robot_half_w)
        self.last_source = source

        if pos is None:
            # No lateral reference at all. Steer on heading only and let the
            # caller decide how long that is acceptable (see
            # safety.dead_reckoning_max_ms). We do NOT invent a position.
            self.last_lateral_mm = 0.0
            self.last_error_mm = 0.0
            self._prev_err = None
            h_err = wrap180(heading_deg - target_heading_deg)
            return int(clamp(self.kh * h_err, -self.max_steer, self.max_steer))

        self.last_lateral_mm = pos
        err = offset_mm - pos          # + => we must move right
        self.last_error_mm = err

        # Derivative on the MEASURED error, and specifically NOT on the
        # setpoint: every pillar-avoidance offset change is a setpoint step,
        # and differentiating it would spike the servo exactly when we are
        # closest to a pillar we must not touch.
        if self._prev_err is None or dt_s <= 0:
            d = 0.0
        else:
            raw_d = (err - self._prev_err) / dt_s
            self._d_filt += self.d_alpha * (raw_d - self._d_filt)
            d = self._d_filt
        self._prev_err = err

        # Heading error is CCW-positive; steer is CW-positive. Drifted left
        # of target => steer right.
        h_err = wrap180(heading_deg - target_heading_deg)

        steer = self.kp * err + self.kd * d + self.kh * h_err
        return int(clamp(steer, -self.max_steer, self.max_steer))


# =============================================================================
class TurnController:
    """90 degree corner, closed on IMU yaw. Never on time, never on encoder
    counts.

    Time-based turns are wrong the moment the battery sags or the mat grips
    differently. Encoder-based turns assume no wheel slip, which is exactly
    what a steered corner at speed does not give you. The IMU measures the
    thing we actually care about -- how far the car has rotated -- and it is
    the only sensor here that is immune to both.

    Completion requires BOTH a small angle error AND a settled yaw rate. On
    angle alone, the controller declares victory as it sweeps through the
    target at peak rate, hands back to the wall-follower mid-swing, and the
    car exits the corner pointing 8 degrees wrong.
    """

    def __init__(self, cfg):
        t = cfg["turn"]
        self.kp = float(t["kp_cdeg_per_deg"])
        self.kd = float(t["kd_cdeg_per_dps"])
        self.entry_steer = int(t["entry_steer_cdeg"])
        self.complete_err = float(t["complete_err_deg"])
        self.complete_rate = float(t["complete_rate_dps"])
        self.target_deg = float(t["target_deg"])
        self.timeout_ms = int(t["timeout_ms"])
        self.max_steer = int(cfg["wall_follow"]["max_steer_cdeg"])

        self.active = False
        self.target_heading = 0.0
        self.turn_sign = 0          # +1 = CCW (left), -1 = CW (right)
        self.start_ms = 0
        self.last_err = 0.0

    def begin(self, current_heading_deg, turn_sign, now_ms):
        self.active = True
        self.turn_sign = turn_sign
        self.target_heading = wrap180(current_heading_deg +
                                      turn_sign * self.target_deg)
        self.start_ms = now_ms
        self.last_err = self.target_deg * turn_sign

    def compute(self, heading_deg, yaw_rate_dps):
        """PD on heading error, plus a feed-forward lock into the corner.

        The feed-forward matters more than the gains: starting from a known
        near-correct steering angle rather than from zero removes the first
        ~150 ms of lag from every single corner, and over 12 corners that is
        most of a second.
        """
        err = wrap180(self.target_heading - heading_deg)   # + => need CCW
        self.last_err = err

        # u is the "rotate CCW" demand; steer is CW-positive, hence the sign
        # flip. The damping term is +kd*rate for the same reason: rotating
        # CCW fast means back off the left-hand steering.
        u = self.kp * err
        steer = -u + self.kd * yaw_rate_dps
        steer += -self.turn_sign * self.entry_steer * self._ff_taper(err)
        return int(clamp(steer, -self.max_steer, self.max_steer))

    def _ff_taper(self, err):
        """Fade the feed-forward out as we approach the target, so the PD term
        can actually settle instead of fighting a constant lock."""
        frac = abs(err) / max(self.target_deg, 1.0)
        return clamp(frac, 0.0, 1.0)

    def complete(self, yaw_rate_dps):
        return (abs(self.last_err) <= self.complete_err and
                abs(yaw_rate_dps) <= self.complete_rate)

    def timed_out(self, now_ms):
        return (now_ms - self.start_ms) > self.timeout_ms


# =============================================================================
class PillarPlanner:
    """Turns a believed pillar detection into a lateral SETPOINT.

    THE MAPPING, WRITTEN OUT IN FULL BECAUSE INVERTING IT IS THE SINGLE MOST
    COMMON FAILURE IN THIS EVENT:

        RED   pillar -> pass it on the pillar's RIGHT  -> the car keeps RIGHT
                     -> setpoint moves to POSITIVE lateral
        GREEN pillar -> pass it on the pillar's LEFT   -> the car keeps LEFT
                     -> setpoint moves to NEGATIVE lateral

    It is asserted in code (see `_side_for`) and printed at boot so a human
    can eyeball it before every round.

    We do not "hug the wall" by a fixed amount. We compute where the pillar
    actually IS laterally (from its bearing and range), then place ourselves
    a configured clearance to the correct side of THAT. A fixed wall-hug is
    wrong whenever the pillar is not on the centreline, which is most of the
    time.
    """

    RED, GREEN = "red", "green"

    def __init__(self, cfg):
        p = cfg["pillar"]
        self.clearance_mm = float(p["clearance_mm"])
        self.engage_mm = float(p["engage_range_mm"])
        self.commit_mm = float(p["commit_range_mm"])
        self.clear_mm = float(p["clear_range_mm"])
        self.ramp_mm = float(p["offset_ramp_mm"])
        self.max_bearing = float(p["max_bearing_deg"])
        self.pass_extra_mm = float(p["pass_extra_mm"])
        # 42.5 mm is the radius of the 85 mm no-move circle (rule 9.24.6).
        # Clearance is measured to the pillar CENTRE, so this is the part of
        # it that is not margin.
        self.no_move_radius_mm = float(
            cfg["field"]["pillar_move_circle_mm"]) * 0.5

    @staticmethod
    def _side_for(colour):
        """+1 = keep right, -1 = keep left. The one function whose sign the
        whole obstacle challenge depends on."""
        if colour == PillarPlanner.RED:
            return +1        # red  -> pass on its right -> we keep right
        if colour == PillarPlanner.GREEN:
            return -1        # green-> pass on its left  -> we keep left
        raise ValueError("unknown pillar colour: %r" % (colour,))

    def describe_mapping(self):
        return ("PILLAR MAPPING: RED -> keep RIGHT (offset +) | "
                "GREEN -> keep LEFT (offset -)")

    def lateral_of(self, det):
        """Where the pillar is, laterally, relative to our centreline."""
        return det.range_mm * math.sin(math.radians(det.bearing_deg))

    def offset_for(self, det, lane_mm, robot_half_w, wall_margin_mm=60.0):
        """Lateral setpoint in mm (POSITIVE = right of centre), already ramped
        by range and clamped to stay inside the lane.

        Returns 0.0 for a pillar that is too far, too far off-axis, or
        already behind us -- i.e. the planner's default is 'do nothing',
        which is the correct default for a detector that might be wrong.
        """
        if det is None:
            return 0.0
        if abs(det.bearing_deg) > self.max_bearing:
            return 0.0
        if det.range_mm > self.engage_mm:
            return 0.0

        side = self._side_for(det.colour)
        pillar_lat = self.lateral_of(det)

        # Clear the no-move circle by the configured margin, measured from
        # the pillar centre to our nearest edge.
        target = pillar_lat + side * (self.clearance_mm + robot_half_w)

        # Stay inside the lane. If the geometry demands more room than the
        # lane has, we take what the lane allows and accept a tighter pass --
        # clipping a wall is worse than nudging a sign, and both are worse
        # than a controlled tight pass.
        limit = lane_mm * 0.5 - robot_half_w - wall_margin_mm
        target = clamp(target, -limit, limit)

        # Ramp in with range. A step setpoint snaps the servo; a ramp makes
        # this a lane change.
        frac = clamp((self.engage_mm - det.range_mm) / max(self.ramp_mm, 1.0),
                     0.0, 1.0)
        return target * frac

    def committed(self, det):
        """Inside commit range the plan is FROZEN and a corner trigger must be
        deferred. See the last-pillar-before-a-corner handling in main.py --
        this flag is what makes that sequencing explicit instead of emergent.
        """
        return det is not None and det.range_mm <= self.commit_mm


# =============================================================================
class SpeedProfile:
    """Chooses a target speed. Trivial by design -- the interesting speed
    control (mm/s -> PWM) is a PI loop on the Pico, where the encoder is."""

    def __init__(self, cfg):
        d = cfg["drive"]
        self.cruise = int(d["cruise_mm_s"])
        self.turn = int(d["turn_mm_s"])
        self.approach = int(d["approach_pillar_mm_s"])
        self.parking = int(d["parking_mm_s"])
        self.reverse = int(d["reverse_mm_s"])
        self.search = int(d["search_mm_s"])
        self.min_move = int(d["min_move_mm_s"])

    def for_straight(self, front_mm, pillar_active, corner_trigger_mm):
        """Slow down as the front wall approaches, and whenever a pillar is
        being avoided. Both are places where steering authority matters more
        than speed, and where an overshoot is expensive."""
        v = self.approach if pillar_active else self.cruise
        if front_mm is not None:
            # Linear taper from 2.5x the corner trigger down to the trigger
            # itself. Arriving at a corner already slowed is worth more than
            # the fraction of a second it costs.
            hi = corner_trigger_mm * 2.5
            if front_mm < hi:
                frac = clamp((front_mm - corner_trigger_mm) /
                             max(hi - corner_trigger_mm, 1.0), 0.0, 1.0)
                v = int(self.turn + (v - self.turn) * frac)
        return max(v, self.min_move)
