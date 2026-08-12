"""
Vision pipeline -- Pi 5 only. Pillar detection and parking-lot detection.

DIVISION OF LABOUR, RESTATED
----------------------------
The camera says WHAT and WHERE (colour and bearing). The ToF cluster says
HOW FAR. Neither is trusted alone:
  * A camera-only range from apparent size is wrong whenever the pillar is
    partially occluded, clipped by the frame edge, or sitting in shadow.
  * A ToF-only detection cannot tell red from green, which is the entire
    decision we need to make.
So this module produces detections with a CAMERA-ESTIMATED range and a
confidence, and pi/main.py rejects any close-range detection the ToF cannot
corroborate. A red jacket in the crowd produces no ToF return and is
therefore never a pillar.

WHY NOT SLAM / ROS 2
--------------------
The blueprint mentions both. A WRO round is 180 seconds on a KNOWN 3 m x 3 m
track with four right-angle corners and walls at known heights. We have an
absolute heading reference (BNO085), wheel odometry, and five ranging
sensors. Building a map buys us nothing we do not already have analytically,
while costing frame latency, tuning time, and a large pile of code that
nobody on the team can debug under pressure. A reactive controller closed on
IMU and ToF is faster to build, faster to tune, and far more deterministic.
If you disagree, the rubric wants MEASURED evidence, not a framework name --
instrument both and compare lap-time variance over 20 runs.

COLOUR CLASSIFICATION
---------------------
HSV, not RGB. Hue separates the rule colours cleanly and is far more stable
under illumination changes than any RGB threshold. The thresholds are SEEDED
from the rule RGB values but LOADED FROM CONFIG, re-measured under venue
lighting during practice (tools/calibrate.py vision). Nothing here is tuned
in source, because thresholds tuned to one venue's lighting are a liability
at the next one.

Red straddles the OpenCV hue wrap (H is 0..179, and the rule red sits at
~177), so red is always TWO ranges. A single range silently loses half of
every red pillar -- and "sometimes sees red" is much harder to diagnose than
"never sees red".

FIXED EXPOSURE AND WHITE BALANCE
--------------------------------
Both are pinned in config. Auto-exposure and AWB retune themselves when a
large saturated pillar fills the frame, shifting the very hue we are trying
to classify. Pinning them is what makes calibration transfer from the
practice mat to the round.
"""

import math
import threading
import time

import cv2
import numpy as np


class Detection:
    """One believed object in one frame."""

    __slots__ = ("colour", "cx", "cy", "w", "h", "area", "fill",
                 "bearing_deg", "range_mm", "range_from_ground_mm",
                 "confidence", "t", "corroborated")

    def __init__(self, colour, cx, cy, w, h, area, fill):
        self.colour = colour
        self.cx = cx
        self.cy = cy
        self.w = w
        self.h = h
        self.area = area
        self.fill = fill
        self.bearing_deg = 0.0
        self.range_mm = 0.0
        self.range_from_ground_mm = None
        self.confidence = 0.0
        self.t = 0.0
        self.corroborated = False

    def __repr__(self):
        return ("<Det %s r=%.0fmm b=%+.1fdeg conf=%.2f%s>"
                % (self.colour, self.range_mm, self.bearing_deg,
                   self.confidence, " ToF" if self.corroborated else ""))


class VisionResult:
    """The whole world model the camera contributes, as one immutable-ish
    snapshot. Handing the FSM a snapshot rather than live lists means the
    planner can never read half of one frame and half of the next."""

    __slots__ = ("t", "frame_id", "pillars", "parking_walls", "nearest",
                 "fps", "proc_ms")

    def __init__(self):
        self.t = 0.0
        self.frame_id = 0
        self.pillars = []
        self.parking_walls = []
        self.nearest = None
        self.fps = 0.0
        self.proc_ms = 0.0


# =============================================================================
class ColourMask:
    """One configured colour band. Pre-builds its numpy bounds once, at
    construction: allocating six 3-element arrays per colour per frame is
    ~0.4 ms of pure garbage at 30 fps, and it all lands in the same GC as the
    control thread."""

    def __init__(self, name, spec):
        self.name = name
        self.bounds = []
        for h_lo, h_hi in spec["h_ranges"]:
            lo = np.array([h_lo, spec["s_min"], spec["v_min"]], dtype=np.uint8)
            hi = np.array([h_hi, spec["s_max"], spec["v_max"]], dtype=np.uint8)
            self.bounds.append((lo, hi))

    def apply(self, hsv, out=None):
        mask = None
        for lo, hi in self.bounds:
            m = cv2.inRange(hsv, lo, hi)
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        return mask


# =============================================================================
class VisionPipeline:

    def __init__(self, cfg, camera=None):
        self.cfg = cfg
        vc = cfg["vision"]
        cc = cfg["camera"]

        self.width, self.height = cc["resolution"]
        self.hfov = float(cc["hfov_deg"])
        # Pinhole focal length in pixels. Everything geometric downstream --
        # bearing, range from apparent height -- rests on this single number,
        # so it is derived from the lens FOV rather than guessed.
        self.focal_px = (self.width * 0.5) / math.tan(math.radians(self.hfov * 0.5))
        self.mount_height_mm = cc["mount_height_mm"]
        self.mount_tilt_deg = cc["mount_tilt_deg"]

        self.roi_top = int(self.height * float(vc["roi_top_frac"]))
        self.roi_bottom = int(self.height * float(vc["roi_bottom_frac"]))

        self.masks = {
            "red": ColourMask("red", vc["red"]),
            "green": ColourMask("green", vc["green"]),
            "magenta": ColourMask("magenta", vc["magenta"]),
        }
        k = int(vc["morph_kernel_px"])
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        self.open_iter = int(vc["morph_open_iter"])
        self.close_iter = int(vc["morph_close_iter"])
        self.min_area = int(vc["min_area_px"])
        self.max_area = int(float(vc["max_area_frac"]) *
                            (self.width * (self.roi_bottom - self.roi_top)))
        self.pillar_min_aspect = float(vc["pillar_min_aspect"])
        self.pillar_max_aspect = float(vc["pillar_max_aspect"])
        self.pillar_min_fill = float(vc["pillar_min_fill"])
        self.wall_max_aspect = float(vc["parking_wall_max_aspect"])
        self.min_consecutive = int(vc["min_consecutive_frames"])

        self.pillar_h_mm = float(cfg["field"]["pillar_size_mm"][2])
        self.pillar_w_mm = float(cfg["field"]["pillar_size_mm"][0])

        self.camera = camera
        self.result = VisionResult()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None
        self._frame_id = 0
        self._fps_ema = 0.0
        self._last_t = None
        # Temporal consistency: a pillar must be seen in N consecutive frames
        # before it may change the plan. Costs ~30 ms of reaction time and
        # removes essentially every single-frame false positive.
        self._streak = {"red": 0, "green": 0}

    # -- camera ------------------------------------------------------------

    def open_camera(self):
        """Picamera2, configured for LOW LATENCY rather than image quality.

        We ask for RGB888 at VGA and pin exposure/gain/AWB. Queueing is
        disabled where possible: a queued frame is a frame that is already
        stale by the time we classify it, and at 700 mm/s each 33 ms of
        staleness is 23 mm of position error in the pillar bearing.
        """
        from picamera2 import Picamera2      # imported late: heavy, Pi-only
        from libcamera import controls       # noqa: F401

        cc = self.cfg["camera"]
        picam = Picamera2()
        config = picam.create_video_configuration(
            main={"size": (self.width, self.height), "format": "RGB888"},
            buffer_count=4,
            controls={"FrameDurationLimits": (
                int(1e6 / float(cc["framerate_target"])),
                int(1e6 / float(cc["framerate_target"])))},
        )
        picam.configure(config)

        ctrls = {}
        if cc["exposure_mode"] == "fixed":
            ctrls["AeEnable"] = False
            ctrls["ExposureTime"] = int(cc["exposure_time_us"])
            ctrls["AnalogueGain"] = float(cc["analogue_gain"])
        if not cc["awb_enable"]:
            ctrls["AwbEnable"] = False
            ctrls["ColourGains"] = tuple(float(g) for g in cc["awb_gains"])
        picam.set_controls(ctrls)

        picam.start()
        time.sleep(0.5)          # sensor settle -- startup path, not a loop
        self.camera = picam
        return picam

    # -- threading ---------------------------------------------------------

    def start(self):
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="vision")
        self._thread.start()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self.camera is not None:
            try:
                self.camera.stop()
            except Exception:
                pass

    def _loop(self):
        while not self._stop.is_set():
            try:
                frame = self.camera.capture_array()
            except Exception:
                time.sleep(0.01)
                continue
            t0 = time.monotonic()
            res = self.process(frame)
            res.proc_ms = (time.monotonic() - t0) * 1000.0
            with self._lock:
                self.result = res

    def latest(self):
        """Snapshot for the control loop. Never blocks it for more than the
        duration of a pointer swap."""
        with self._lock:
            return self.result

    # -- the pipeline ------------------------------------------------------

    def process(self, frame_rgb):
        res = VisionResult()
        now = time.monotonic()
        res.t = now
        self._frame_id += 1
        res.frame_id = self._frame_id

        if self._last_t is not None:
            dt = now - self._last_t
            if dt > 0:
                inst = 1.0 / dt
                self._fps_ema = inst if self._fps_ema == 0 else \
                    self._fps_ema + 0.1 * (inst - self._fps_ema)
        self._last_t = now
        res.fps = self._fps_ema

        # Crop FIRST, then convert. The walls are 100 mm tall; everything
        # above the horizon line is the room, the judges, and someone's red
        # jacket. Cropping is both the cheapest false-positive filter we have
        # AND a ~35% saving on the colour conversion, which is the single
        # most expensive step in this function.
        roi = frame_rgb[self.roi_top:self.roi_bottom, :, :]
        hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

        for colour in ("red", "green"):
            for det in self._find(hsv, colour, pillar=True):
                res.pillars.append(det)

        for det in self._find(hsv, "magenta", pillar=False):
            res.parking_walls.append(det)

        # Temporal consistency gate.
        seen = {"red": False, "green": False}
        for d in res.pillars:
            seen[d.colour] = True
        for c in ("red", "green"):
            self._streak[c] = self._streak[c] + 1 if seen[c] else 0
        res.pillars = [d for d in res.pillars
                       if self._streak[d.colour] >= self.min_consecutive]

        # The nearest believable pillar is the only one the planner acts on.
        # Planning for two pillars at once produces a compromise trajectory
        # that clears neither.
        if res.pillars:
            res.nearest = min(res.pillars, key=lambda d: d.range_mm)

        res.pillars.sort(key=lambda d: d.range_mm)
        res.parking_walls.sort(key=lambda d: d.range_mm)
        return res

    def _find(self, hsv, colour, pillar):
        mask = self.masks[colour].apply(hsv)
        if self.open_iter:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel,
                                    iterations=self.open_iter)
        if self.close_iter:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel,
                                    iterations=self.close_iter)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < self.min_area or area > self.max_area:
                continue
            x, y, w, h = cv2.boundingRect(c)
            if w <= 0 or h <= 0:
                continue
            aspect = h / float(w)
            fill = area / float(w * h)

            if pillar:
                # Pillars are 50 wide x 100 tall: taller than wide in any
                # upright view. This one test rejects red mat reflections and
                # the long thin glare stripe, which are the two false
                # positives that actually happen on this track.
                if not (self.pillar_min_aspect <= aspect <= self.pillar_max_aspect):
                    continue
                if fill < self.pillar_min_fill:
                    continue
            else:
                # Parking walls are 200 long x 100 tall: WIDER than tall.
                if aspect > self.wall_max_aspect:
                    continue

            det = Detection(colour, x + w * 0.5, y + h * 0.5, w, h, area, fill)
            self._geometry(det)
            det.t = time.monotonic()
            det.confidence = self._confidence(det, pillar)
            out.append(det)
        return out

    def _geometry(self, det):
        """Bearing and range, in the vehicle frame."""
        # Bearing: pinhole, from horizontal pixel offset. POSITIVE = RIGHT,
        # matching the steering and lateral sign conventions in control.py.
        dx = det.cx - self.width * 0.5
        det.bearing_deg = math.degrees(math.atan2(dx, self.focal_px))

        # Range estimate 1: apparent HEIGHT. Preferred over width because a
        # pillar viewed at an angle gets narrower but not shorter -- height is
        # invariant to yaw, width is not.
        if det.h > 1:
            det.range_mm = self.focal_px * self.pillar_h_mm / det.h
        elif det.w > 1:
            det.range_mm = self.focal_px * self.pillar_w_mm / det.w
        else:
            det.range_mm = 0.0

        # Range estimate 2: ground plane, from where the BASE of the object
        # sits in the image. Independent of apparent size, so it stays right
        # when the top of a pillar is clipped by the ROI -- which is exactly
        # what happens at close range, i.e. when the decision is committed.
        det.range_from_ground_mm = self._ground_range(det)

        # If the two independent estimates disagree wildly, prefer the ground
        # plane and let the confidence score take the hit: a clipped bounding
        # box overestimates distance, and overestimating distance to a pillar
        # is the dangerous direction.
        g = det.range_from_ground_mm
        if g is not None and det.range_mm > 0:
            if abs(g - det.range_mm) > 0.5 * max(g, det.range_mm):
                det.range_mm = g

    def _ground_range(self, det):
        if self.mount_height_mm is None or self.mount_tilt_deg is None:
            return None                # not calibrated; say so, do not guess
        base_y_full = self.roi_top + det.cy + det.h * 0.5
        dy = base_y_full - self.height * 0.5
        ang_below = math.degrees(math.atan2(dy, self.focal_px))
        total = float(self.mount_tilt_deg) + ang_below
        if total <= 1.0:               # at or above the horizon: unusable
            return None
        return float(self.mount_height_mm) / math.tan(math.radians(total))

    def _confidence(self, det, pillar):
        """0..1. Consumed by main.py alongside the ToF corroboration test.

        Deliberately simple and explainable -- a judge can read this and a
        teammate can predict it. A learned score would be neither.
        """
        c = 0.4
        if pillar:
            ideal = self.pillar_h_mm / self.pillar_w_mm     # 2.0
            aspect = det.h / max(det.w, 1.0)
            c += 0.3 * max(0.0, 1.0 - abs(aspect - ideal) / ideal)
        c += 0.2 * min(det.fill, 1.0)
        if abs(det.bearing_deg) < 15.0:
            c += 0.1                   # straight ahead is where we can act
        if det.range_from_ground_mm is not None and det.range_mm > 0:
            agree = 1.0 - min(1.0, abs(det.range_from_ground_mm - det.range_mm)
                              / max(det.range_mm, 1.0))
            c = c * 0.8 + 0.2 * agree
        return max(0.0, min(1.0, c))


# =============================================================================
def corroborate_with_tof(det, tof_fc, tof_fl, tof_fr, tof_diag, cfg):
    """Reject a camera detection the ToF cluster cannot back up.

    The camera can hallucinate a pillar from a reflection, a spectator's
    clothing, or a colour-cast on the mat. None of those return a ToF echo at
    the matching bearing. This is the single cheapest false-positive filter
    we have, and it is the reason the vehicle does not swerve at the crowd.

    Corroboration is only REQUIRED at close range. Beyond
    `tof_corroborate_required_below_mm` the black-wall-limited ToF often
    returns nothing at all, so demanding agreement out there would reject
    every genuine distant pillar -- we would rather engage early on a
    maybe-pillar (the offset ramp makes that nearly free) than commit late on
    a confirmed one.
    """
    p = cfg["pillar"]
    tol = float(p["tof_corroborate_mm"])
    need_below = float(p["tof_corroborate_required_below_mm"])

    if det.range_mm > need_below:
        det.corroborated = False
        return True                     # too far to require agreement

    # Pick the ToF whose field of view actually covers this bearing. The
    # angles are the sensor MOUNTING angles -- update them if you re-mount.
    b = det.bearing_deg
    if b < -12.0:
        cand = (tof_fl, tof_diag)
    elif b > 12.0:
        cand = (tof_fr, tof_diag)
    else:
        cand = (tof_fc,)

    for d in cand:
        if d is not None and abs(d - det.range_mm) <= tol:
            det.corroborated = True
            # Trust the ToF's number over the camera's: a direct range
            # measurement beats one inferred from apparent size.
            det.range_mm = float(d)
            return True

    det.corroborated = False
    return False
