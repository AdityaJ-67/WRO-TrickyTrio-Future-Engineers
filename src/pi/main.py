

import argparse
import json
import os
import subprocess
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from common import protocol as P               # noqa: E402
from pi.link import PicoLink                   # noqa: E402
from pi.logger import Logger                   # noqa: E402
from pi.config import load_config, config_to_pico_pairs   # noqa: E402
from pi.control import (WallFollower, TurnController, PillarPlanner,   # noqa
                        LaneEstimator, SpeedProfile, clamp, wrap180)




IDLE           = "IDLE"
WAIT_START     = "WAIT_START"
DRIVE_STRAIGHT = "DRIVE_STRAIGHT"
TURN           = "TURN"
AVOID_PILLAR   = "AVOID_PILLAR"
SEARCH_PARKING = "SEARCH_PARKING"
PARK           = "PARK"
FINISHED       = "FINISHED"
EMERGENCY      = "EMERGENCY"

TRANSITIONS = {
    IDLE:           (WAIT_START, EMERGENCY),
    WAIT_START:     (DRIVE_STRAIGHT, EMERGENCY),
    DRIVE_STRAIGHT: (TURN, AVOID_PILLAR, SEARCH_PARKING, FINISHED, EMERGENCY),
    TURN:           (DRIVE_STRAIGHT, AVOID_PILLAR, FINISHED, EMERGENCY),
    AVOID_PILLAR:   (DRIVE_STRAIGHT, TURN, FINISHED, EMERGENCY),
    SEARCH_PARKING: (PARK, FINISHED, EMERGENCY),
    PARK:           (FINISHED, EMERGENCY),
    FINISHED:       (),         
    EMERGENCY:      (),          


PK_NAMES = {0: "ALIGN", 1: "FORWARD", 2: "CUT1", 3: "CUT2", 4: "SETTLE",
            5: "DONE"}



def check_radios():
    
    msgs = []
    ok = True

    boot_cfg = "/boot/firmware/config.txt"
    if not os.path.exists(boot_cfg):
        boot_cfg = "/boot/config.txt"
    try:
        with open(boot_cfg) as f:
            txt = f.read()
        for overlay in ("disable-wifi", "disable-bt"):
            if ("dtoverlay=" + overlay) not in txt:
                ok = False
                msgs.append("MISSING 'dtoverlay=%s' in %s" % (overlay, boot_cfg))
            else:
                msgs.append("OK  dtoverlay=%s present" % overlay)
    except OSError as e:
        ok = False
        msgs.append("could not read %s (%s)" % (boot_cfg, e))

    # Any live wireless interface is fatal, overlays or not.
    for iface in ("wlan0", "wlan1"):
        p = "/sys/class/net/%s/operstate" % iface
        if os.path.exists(p):
            try:
                with open(p) as f:
                    state = f.read().strip()
            except OSError:
                state = "unknown"
            if state != "down":
                ok = False
                msgs.append("FAIL %s is %s -- must be absent or down" %
                            (iface, state))
            else:
                msgs.append("OK  %s down" % iface)

    try:
        out = subprocess.run(["rfkill", "list"], capture_output=True,
                             text=True, timeout=3).stdout.lower()
        if "soft blocked: no" in out:
            msgs.append("WARN rfkill reports an unblocked radio; verify by "
                        "hand before the round")
    except Exception:
        pass    
    try:
        out = subprocess.run(["hciconfig"], capture_output=True, text=True,
                             timeout=3).stdout
        if "UP RUNNING" in out:
            ok = False
            msgs.append("FAIL a Bluetooth HCI device is UP")
    except Exception:
        pass

    return ok, msgs


def check_uart_released():
    for path in ("/boot/firmware/cmdline.txt", "/boot/cmdline.txt"):
        if os.path.exists(path):
            try:
                with open(path) as f:
                    txt = f.read()
            except OSError:
                continue
            if "console=serial0" in txt or "console=ttyAMA0" in txt:
                return False, "serial console still enabled in " + path
            return True, "serial console released (" + path + ")"
    return True, "cmdline.txt not found; assuming serial console released"


class Mission:

    def __init__(self, cfg, link, vision, log, args):
        self.cfg = cfg
        self.link = link
        self.vision = vision
        self.log = log
        self.args = args

        self.state = IDLE
        self.state_entered = time.monotonic()
        self.t_start = None             


        self.follower = WallFollower(cfg)
        self.turner = TurnController(cfg)
        self.planner = PillarPlanner(cfg)
        self.speeds = SpeedProfile(cfg)
        nominal = cfg["field"]["lane_width_obstacle_mm"]
        self.lane = LaneEstimator(
            nominal, float(cfg["wall_follow"]["lane_width_estimate_alpha"]),
            cfg["field"]["lane_width_open_min_mm"],
            cfg["field"]["lane_width_open_max_mm"])

        self.robot_half_w = float(cfg["robot"]["width_mm"]) * 0.5
        self.um_per_count = float(cfg["pico"]["um_per_count"])

      
        self.mode = cfg["challenge"]["mode"]
        self.crossings = 0
        self.laps = 0
        self.direction = None              
        self.turn_sign = 0                  
        self.first_line_colour = None
        self.target_heading = 0.0
        self.last_crossing_dist_mm = -1e9
        self.last_turn_dist_mm = -1e9
        self.pillars_seen = 0


        self.dist_mm = 0.0
        self.prev_t = None
        self.offset_target = 0.0
        self.offset_cmd = 0.0
        self.active_pillar = None
        self.pillar_pass_dist_mm = None
        self.pending_turn = False
        self.no_lateral_since = None
        self.prev_colour_class = 0

      
        self.park_phase = PK_ALIGN
        self.park_phase_t = 0.0
        self.park_phase_dist = 0.0
        self.park_entry_heading = 0.0
        self.park_side = 1                 
        self.search_started = None

        
        self.steer_cdeg = 0
        self.speed_mm_s = 0
        self.cmd_flags = 0

        self.finished_at = None
        self.emergency_reason = ""

   
    def transition(self, new, reason=""):
        if new == self.state:
            return True
        if new not in TRANSITIONS.get(self.state, ()):
            self.log.event("illegal_transition", frm=self.state, to=new,
                           reason=reason)
            return False
        dwell = time.monotonic() - self.state_entered
        self.log.event("state", frm=self.state, to=new, reason=reason,
                       dwell_s=round(dwell, 3), t_mission=self.mission_t(),
                       laps=self.laps, crossings=self.crossings)
        print("[%6.2f] %-14s -> %-14s  %s"
              % (self.mission_t(), self.state, new, reason))
        self.state = new
        self.state_entered = time.monotonic()
        self.follower.reset()         
        return True

    def mission_t(self):
        return 0.0 if self.t_start is None else time.monotonic() - self.t_start

   

    def _ranges(self, t):
   
        return (P.tof_valid(t, P.TOF_FC), P.tof_valid(t, P.TOF_FL),
                P.tof_valid(t, P.TOF_FR), P.tof_valid(t, P.TOF_DIAG),
                P.tof_valid(t, P.TOF_REAR))

    def _corner_trigger_mm(self):
       
        t = self.cfg["turn"]
        base = float(t["corner_trigger_mm"])
        nominal = float(self.cfg["field"]["lane_width_obstacle_mm"])
        scale = clamp(self.lane.width_mm / nominal,
                      float(t["corner_trigger_scale_min"]),
                      float(t["corner_trigger_scale_max"]))
        return base * scale

    def _update_lap_counting(self, t):
       
        c = t["colour_class"]
        prev = self.prev_colour_class
        self.prev_colour_class = c
        if c == P.COLOUR_NONE or prev != P.COLOUR_NONE:
            return                        
        if (self.dist_mm - self.last_crossing_dist_mm) < \
                float(self.cfg["lap"]["line_lockout_mm"]):
            return

        self.last_crossing_dist_mm = self.dist_mm
        self.crossings += 1
        colour = "orange" if c == P.COLOUR_ORANGE else "blue"

        if self.direction is None:
            self.first_line_colour = colour
            self.direction = self.cfg["lap"]["first_line_direction_map"][colour]
            self.turn_sign = -1 if self.direction == "clockwise" else +1
            self.log.event("direction_latched", first_line=colour,
                           direction=self.direction, turn_sign=self.turn_sign)
            print("  DIRECTION LATCHED: first line %s -> %s (turn_sign %+d)"
                  % (colour, self.direction, self.turn_sign))

        self.laps = self.crossings // int(self.cfg["field"]["corners_per_lap"])
        self.log.event("line_crossing", colour=colour, n=self.crossings,
                       laps=self.laps, dist_mm=round(self.dist_mm, 1),
                       clear=t["colour_clear"])

    def _select_pillar(self, vres, ranges):
       
        if vres is None or not vres.pillars:
            return None
        from pi.vision import corroborate_with_tof
        fc, fl, fr, diag, _rear = ranges
        ttl = float(self.cfg["vision"]["detection_ttl_ms"]) / 1000.0
        now = time.monotonic()
        best = None
        for det in vres.pillars:
            if (now - det.t) > ttl:
                continue             
            if det.range_mm > float(self.cfg["pillar"]["engage_range_mm"]):
                continue
            if abs(det.bearing_deg) > float(self.cfg["pillar"]["max_bearing_deg"]):
                continue
            if not corroborate_with_tof(det, fc, fl, fr, diag, self.cfg):
                continue
            if best is None or det.range_mm < best.range_mm:
                best = det
        return best

    

    def step(self, t):
        """One control step, called on every telemetry frame (50 Hz)."""
        now = time.monotonic()
        dt = 0.02 if self.prev_t is None else max(1e-3, now - self.prev_t)
        self.prev_t = now

        self.dist_mm = t["enc_counts"] * self.um_per_count / 1000.0
        fc, fl, fr, diag, rear = self._ranges(t)
        heading = t["yaw_deg"]
        rate = t["yaw_rate_dps"]
        vres = self.vision.latest() if self.vision else None

       
        if t["flags"] & P.FLAG_ESTOP_LATCHED and self.state not in (FINISHED,
                                                                    EMERGENCY):
            self.emergency_reason = "pico e-stop latched"
            self.transition(EMERGENCY, self.emergency_reason)

        if self.state not in (IDLE, WAIT_START, FINISHED, EMERGENCY):
            mt = self.mission_t()
            if mt > float(self.cfg["mission"]["round_limit_s"]):
                self.transition(FINISHED, "round time limit")
            elif (mt > float(self.cfg["mission"]["soft_deadline_s"])
                    and self.state in (SEARCH_PARKING, DRIVE_STRAIGHT,
                                       AVOID_PILLAR)
                    and self.laps >= int(self.cfg["field"]["laps_required"])):
            
                self.transition(FINISHED, "soft deadline; abandoning parking")

        if self.state not in (IDLE, WAIT_START):
            self._update_lap_counting(t)
            self.lane.update(fl, fr, self.robot_half_w * 2)

        
        handler = getattr(self, "_st_" + self.state.lower())
        handler(t, dt, fc, fl, fr, diag, rear, heading, rate, vres)

        self._log_row(t, fc, fl, fr, diag, rear, vres)
        return self.steer_cdeg, self.speed_mm_s, self.cmd_flags


    def _st_idle(self, t, dt, fc, fl, fr, diag, rear, heading, rate, vres):
       
        self.steer_cdeg = 0
        self.speed_mm_s = 0
        self.cmd_flags = 0

    def _st_wait_start(self, t, dt, fc, fl, fr, diag, rear, heading, rate, vres):
        self.steer_cdeg = 0
        self.speed_mm_s = 0
        self.cmd_flags = P.CMDF_ARM
        if t["flags"] & P.FLAG_START_LATCHED:
        
            self.cmd_flags |= P.CMDF_ZERO_HEADING | P.CMDF_RESET_ODOM
            self.t_start = time.monotonic()
            self.target_heading = 0.0
            self.last_crossing_dist_mm = -1e9
            self.log.event("start", mode=self.mode)
            self.transition(DRIVE_STRAIGHT, "start button")

    def _st_drive_straight(self, t, dt, fc, fl, fr, diag, rear,
                           heading, rate, vres):
        self.cmd_flags = P.CMDF_ARM

       
        det = self._select_pillar(vres, (fc, fl, fr, diag, rear)) \
            if self.mode == "obstacle" else None
        if det is not None:
            self.pillars_seen += 1
            self.active_pillar = det
            self.pillar_pass_dist_mm = None
            self.transition(AVOID_PILLAR,
                            "%s pillar at %.0fmm b=%+.1f"
                            % (det.colour, det.range_mm, det.bearing_deg))
            return

      
        need = int(self.cfg["lap"]["crossings_required"])
        if self.crossings >= need:
            if self.mode == "obstacle":
                self.search_started = time.monotonic()
                self.transition(SEARCH_PARKING, "3 laps complete")
                return
         
            run_on = self.cfg["finish"]["run_on_after_last_line_mm"]
            if run_on is None:
                
                if fc is not None and fc < self._corner_trigger_mm():
                    self.transition(FINISHED, "finish section (fallback: "
                                              "run_on not measured)")
                    return
            elif (self.dist_mm - self.last_crossing_dist_mm) >= float(run_on):
                self.transition(FINISHED, "finish section reached")
                return

        
        if self._corner_due(fc, diag):
            self._begin_turn(heading, "front %s < %.0f"
                             % (fc, self._corner_trigger_mm()))
            return

        self._follow(dt, fl, fr, heading, 0.0)
        self.speed_mm_s = self.speeds.for_straight(fc, False,
                                                   self._corner_trigger_mm())
        self._check_lateral_health(fl, fr)

    def _st_avoid_pillar(self, t, dt, fc, fl, fr, diag, rear,
                         heading, rate, vres):
        self.cmd_flags = P.CMDF_ARM
        det = self._select_pillar(vres, (fc, fl, fr, diag, rear))
        if det is not None and self.active_pillar is not None and \
                det.colour == self.active_pillar.colour:
            self.active_pillar = det

        committed = self.planner.committed(self.active_pillar) if det else False


        if self._corner_due(fc, diag):
            if committed:
                if not self.pending_turn:
                    self.pending_turn = True
                    self.log.event("turn_deferred",
                                   pillar=self.active_pillar.colour,
                                   pillar_mm=round(self.active_pillar.range_mm),
                                   front_mm=fc)
            else:
               
                self.active_pillar = None
                self.offset_target = 0.0
                self._begin_turn(heading, "corner before commit")
                return

       
        if det is None:
            if self.pillar_pass_dist_mm is None:
               
                self.pillar_pass_dist_mm = self.dist_mm + \
                    float(self.cfg["pillar"]["pass_extra_mm"])
                self.log.event("pillar_passed_visually",
                               hold_to_mm=round(self.pillar_pass_dist_mm))
            if self.dist_mm >= self.pillar_pass_dist_mm:
                self.active_pillar = None
                self.offset_target = 0.0
                if self.pending_turn or self._corner_due(fc, diag):
                    self.pending_turn = False
                    self._begin_turn(heading, "deferred turn after pillar")
                else:
                    self.transition(DRIVE_STRAIGHT, "pillar cleared")
                return
        else:
            self.pillar_pass_dist_mm = None

    
        target = self.planner.offset_for(self.active_pillar,
                                         self.lane.width_mm,
                                         self.robot_half_w)
  
        max_step = 400.0 * dt          # mm/s of setpoint travel
        self.offset_target += clamp(target - self.offset_target,
                                    -max_step, max_step)

        self._follow(dt, fl, fr, heading, self.offset_target)
        self.speed_mm_s = self.speeds.for_straight(fc, True,
                                                   self._corner_trigger_mm())

    def _st_turn(self, t, dt, fc, fl, fr, diag, rear, heading, rate, vres):
        self.cmd_flags = P.CMDF_ARM
        self.steer_cdeg = self.turner.compute(heading, rate)
        self.speed_mm_s = self.speeds.turn

        if self.turner.complete(rate):
            self.turner.active = False
            self.target_heading = self.turner.target_heading
            self.last_turn_dist_mm = self.dist_mm
            self.transition(DRIVE_STRAIGHT, "turn complete (err %.1f deg)"
                            % self.turner.last_err)
        elif self.turner.timed_out(int(time.monotonic() * 1000)):
    
            self.turner.active = False
            self.target_heading = self.turner.target_heading
            self.last_turn_dist_mm = self.dist_mm
            self.log.event("turn_timeout", err=self.turner.last_err)
            self.transition(DRIVE_STRAIGHT, "TURN TIMEOUT -- check the IMU")

    def _st_search_parking(self, t, dt, fc, fl, fr, diag, rear,
                           heading, rate, vres):
        self.cmd_flags = P.CMDF_ARM
        self._follow(dt, fl, fr, heading, 0.0)
        self.speed_mm_s = self.speeds.search

        if self._corner_due(fc, diag):
            self._begin_turn(heading, "corner while searching for the lot")
            return

        walls = vres.parking_walls if vres else []
        if len(walls) >= 2:
            
            near = sorted(walls, key=lambda d: d.range_mm)[:2]
            bearing = sum(d.bearing_deg for d in near) / 2.0
            rng = sum(d.range_mm for d in near) / 2.0
            self.park_phase = PK_ALIGN
            self.park_phase_t = time.monotonic()
            self.park_phase_dist = self.dist_mm
            self.park_entry_heading = heading
            self.park_side = 1 if bearing > 0 else -1
            self.log.event("parking_found", bearing=round(bearing, 1),
                           range_mm=round(rng), side=self.park_side)
            self.transition(PARK, "magenta lot at %.0fmm b=%+.1f"
                            % (rng, bearing))
            return

        if self.search_started and (time.monotonic() - self.search_started) > \
                float(self.cfg["parking"]["search_timeout_s"]):
           
            self.transition(FINISHED, "parking search timeout -- stopping safely")

    def _st_park(self, t, dt, fc, fl, fr, diag, rear, heading, rate, vres):
      
        self.cmd_flags = P.CMDF_ARM | P.CMDF_PARKING_PROF
        pk = self.cfg["parking"]
        phase_dist = self.dist_mm - self.park_phase_dist
        phase_t = time.monotonic() - self.park_phase_t

        if phase_t > float(pk["max_manoeuvre_s"]):
            self.log.event("park_timeout", phase=PK_NAMES[self.park_phase])
            self.transition(FINISHED, "park timeout -- stopping where we are")
            return

        if self.park_phase == PK_ALIGN:
          
            side = getattr(self, "park_side", 1)
            gap = float(pk["approach_side_gap_mm"])
            offset = side * max(0.0, self.lane.half - gap - self.robot_half_w)
            self._follow(dt, fl, fr, heading, offset)
            self.speed_mm_s = self.speeds.parking
            overshoot = pk["align_overshoot_mm"]
            if overshoot is None:
               
                overshoot = float(self.cfg["robot"]["wheelbase_mm"]) * 0.9
            if phase_dist >= overshoot:
                self._next_park_phase(PK_CUT1, heading)

        elif self.park_phase == PK_CUT1:
         
            self.cmd_flags |= P.CMDF_ALLOW_REVERSE
            side = getattr(self, "park_side", 1)
            self.steer_cdeg = int(side * float(pk["cut1_steer_cdeg"]))
            self.speed_mm_s = self.speeds.reverse
            turned = abs(wrap180(heading - self.park_entry_heading))
            if turned >= float(pk["cut1_target_yaw_deg"]):
                self._next_park_phase(PK_CUT2, heading)
            elif rear is not None and rear < float(pk["rear_min_mm"]):
              
                self._next_park_phase(PK_SETTLE, heading)

        elif self.park_phase == PK_CUT2:
         
            self.cmd_flags |= P.CMDF_ALLOW_REVERSE
            side = getattr(self, "park_side", 1)
            self.steer_cdeg = int(side * float(pk["cut2_steer_cdeg"]))
            self.speed_mm_s = self.speeds.reverse
            err = abs(wrap180(heading - self.park_entry_heading))
            if err <= float(pk["cut2_target_yaw_deg"]):
                self._next_park_phase(PK_SETTLE, heading)
            elif rear is not None and rear <= float(pk["rear_target_mm"]):
                self._next_park_phase(PK_SETTLE, heading)

        elif self.park_phase == PK_SETTLE:

            self.steer_cdeg = 0
            if phase_dist < float(pk["final_nudge_mm"]):
                self.speed_mm_s = self.speeds.min_move
            else:
                self.log.event("park_complete",
                               rear_mm=rear, heading_err=round(
                                   wrap180(heading - self.park_entry_heading), 2))
                self.transition(FINISHED, "parked")

    def _st_finished(self, t, dt, fc, fl, fr, diag, rear, heading, rate, vres):
        """TERMINAL. Rule 9.24.2: stop autonomously and stay stopped for 15 s.

        This state ignores ALL sensor input by design -- it does not read a
        range, does not look at the camera, and cannot be talked into moving
        again by anything. We hold an ACTIVE BRAKE (not a coast: a coasting
        car on a smooth mat keeps creeping) and centre the wheels, then
        disarm once the hold is comfortably past 15 s.
        """
        if self.finished_at is None:
            self.finished_at = time.monotonic()
            self.log.event("finished", t_mission=self.mission_t(),
                           laps=self.laps, crossings=self.crossings,
                           pillars_seen=self.pillars_seen)
        held = time.monotonic() - self.finished_at
        hold = float(self.cfg["finish"]["hold_stopped_s"]) + \
            float(self.cfg["finish"]["hold_extra_s"])
        self.steer_cdeg = 0
        self.speed_mm_s = 0
        
        self.cmd_flags = (P.CMDF_ARM | P.CMDF_BRAKE) if held < hold else 0

    def _st_emergency(self, t, dt, fc, fl, fr, diag, rear, heading, rate, vres):
        """TERMINAL. The Pico has already stopped the car in hardware; there
        is nothing useful left for us to command. We do NOT auto-clear the
        e-stop: something was hit, and a car that resumes after a collision
        turns a lost round into a damaged robot."""
        self.steer_cdeg = 0
        self.speed_mm_s = 0
        self.cmd_flags = 0


    def _follow(self, dt, fl, fr, heading, offset_mm):
        self.offset_cmd = offset_mm
        self.steer_cdeg = self.follower.compute(
            fl, fr, self.lane.width_mm, self.robot_half_w,
            offset_mm, heading, self.target_heading, dt)

    def _corner_due(self, fc, diag):
        if fc is None:
            return False
        if (self.dist_mm - self.last_turn_dist_mm) < \
                float(self.cfg["turn"]["min_interval_mm"]):
           
            return False
        trig = self._corner_trigger_mm()
        if fc < trig:
            return True
       
        return diag is not None and diag < trig * 0.8 and fc < trig * 1.3

    def _begin_turn(self, heading, reason):
        if self.turn_sign == 0:
          
            self.turn_sign = +1
            self.log.event("turn_sign_fallback")
        self.turner.begin(heading, self.turn_sign, int(time.monotonic() * 1000))
        self.transition(TURN, reason)

    def _next_park_phase(self, phase, heading):
        self.log.event("park_phase", frm=PK_NAMES[self.park_phase],
                       to=PK_NAMES[phase], heading=round(heading, 2))
        self.park_phase = phase
        self.park_phase_t = time.monotonic()
        self.park_phase_dist = self.dist_mm

    def _check_lateral_health(self, fl, fr):
    
        have = self.follower.usable(fl) or self.follower.usable(fr)
        if have:
            self.no_lateral_since = None
            return
        if self.no_lateral_since is None:
            self.no_lateral_since = time.monotonic()
            self.log.event("dead_reckoning_start")
            return
        elapsed_ms = (time.monotonic() - self.no_lateral_since) * 1000.0
        if elapsed_ms > float(self.cfg["safety"]["dead_reckoning_max_ms"]):
            self.transition(FINISHED, "no lateral reference for %.1f s"
                            % (elapsed_ms / 1000.0))



    def _log_row(self, t, fc, fl, fr, diag, rear, vres):
        det = self.active_pillar
        self.log.telemetry({
            "t": self.mission_t(), "pico_ms": t["t_ms"], "state": self.state,
            "pico_state": P.PICO_ST_NAMES.get(t["pico_state"]),
            "tof_fc": fc, "tof_fl": fl, "tof_fr": fr, "tof_diag": diag,
            "tof_rear": rear, "tof_valid": t["tof_valid_bits"],
            "tof_degraded": t["tof_degraded_bits"],
            "yaw": t["yaw_deg"], "yaw_rate": t["yaw_rate_dps"],
            "enc": t["enc_counts"], "speed": t["speed_mm_s"],
            "colour_class": t["colour_class"], "colour_clear": t["colour_clear"],
            "lane_est": self.lane.width_mm,
            "lateral": self.follower.last_lateral_mm,
            "lat_err": self.follower.last_error_mm,
            "lat_source": self.follower.last_source,
            "offset_target": self.offset_cmd,
            "steer_cmd": self.steer_cdeg, "speed_cmd": self.speed_mm_s,
            "front_stop": self.cfg["pico"]["front_stop_mm"],
            "pillar_colour": det.colour if det else "",
            "pillar_range": det.range_mm if det else "",
            "pillar_bearing": det.bearing_deg if det else "",
            "laps": self.laps, "crossings": self.crossings,
            "flags": t["flags"], "faults": t["fault_bits"],
            "vbat": t["vbat_mv"], "fps": vres.fps if vres else 0.0,
            "rtt": self.link.rtt_ms,
        })

def preflight(cfg, raw_cfg, problems, missing, link, vision, args):
    """Everything that must be true BEFORE the start button is pressed.

    Failures are reported here, on the console and on the Pico's status LED,
    never mid-round. That is the whole point: a missing sensor found in the
    pit costs a practice run; the same sensor found on lap 2 costs the round.
    """
    print("\n=== PRE-FLIGHT ===")
    ok = True

    print("config: %s (schema %s)" % (cfg.get("config_id"),
                                      cfg.get("schema_version")))
    for p in problems:
        ok = False
        print("  FAIL rule constant mismatch: " + p)
    for m in missing:
        ok = False
        print("  FAIL unmeasured required value: %s is still null" % m)
    if missing:
        print("       -> measure these on the mat and write them into "
              "config.json. We will NOT guess; see README 'Calibration'.")


    radios_ok, msgs = check_radios()
    for m in msgs:
        print("  " + m)
    if cfg["dev"]["allow_radio"] or cfg["dev"]["dashboard_enabled"]:
        radios_ok = False
        print("  FAIL config enables a radio/dashboard -- practice only "
              "(conflict C1, rule 11.10)")
    ok = ok and radios_ok

    uart_ok, uart_msg = check_uart_released()
    print("  " + ("OK  " if uart_ok else "FAIL ") + uart_msg)
    ok = ok and uart_ok


    print("waiting for Pico telemetry...")
    t = None
    deadline = time.monotonic() + 8.0
    while time.monotonic() < deadline:
        t = link.wait_telemetry(0.5)
        if t is not None:
            break
    if t is None:
        print("  FAIL no telemetry from the Pico. Check the UART cross-wiring "
              "(Pico GP0->Pi pin 10, GP1<-Pi pin 8) and the common ground.")
        return False
    print("  OK  telemetry, pico_state=%s" % P.PICO_ST_NAMES.get(t["pico_state"]))

    if t["pico_state"] == P.PICO_ST_FAULT:
        ok = False
        print("  FAIL Pico self-test failed, fault_bits=0x%02X" % t["fault_bits"])
        for bit, name in ((P.FAULT_MUX, "I2C mux"), (P.FAULT_IMU, "BNO085"),
                          (P.FAULT_COLOUR, "TCS34725"),
                          (P.FAULT_TOF_ANY, "one or more VL53L0X"),
                          (P.FAULT_RADIO_ON, "Pico radio still up"),
                          (P.FAULT_I2C_BUS, "I2C bus error storm")):
            if t["fault_bits"] & bit:
                print("       - " + name)
    if t["tof_degraded_bits"]:
        for i in range(5):
            if t["tof_degraded_bits"] & (1 << i):
                print("  WARN ToF %s is degraded; we will drive without it"
                      % P.TOF_NAMES[i])
    if t["flags"] & P.FLAG_WDT_RESET:
        print("  WARN the Pico rebooted from a WATCHDOG timeout. Something "
              "wedged its control loop -- investigate before the round.")


    pairs = config_to_pico_pairs(cfg)
    n_frames = link.send_config(pairs)
    time.sleep(0.3)
    applied = sum(1 for a in link.acks if a["acked_type"] == P.MSG_CONFIG)
    if applied < n_frames:
        ok = False
        print("  FAIL config push: %d/%d frames acknowledged" %
              (applied, n_frames))
    else:
        print("  OK  config pushed (%d keys in %d frames)" %
              (len(pairs), n_frames))


    for _ in range(10):
        link.send_ping()
        time.sleep(0.02)
    time.sleep(0.2)
    s = link.stats()
    print("  link: rtt=%s ms (max %.1f) crc_err=%d gaps=%d" %
          ("%.1f" % s["rtt_ms"] if s["rtt_ms"] else "?",
           s["rtt_max_ms"], s["crc_errors"], s["telemetry_gaps"]))
    if s["crc_errors"] > 5:
        ok = False
        print("  FAIL too many CRC errors already -- check the UART wiring, "
              "the ground, and that the baud rates match")
    
    if vision is not None:
        time.sleep(1.0)
        v = vision.latest()
        fps_min = float(cfg["mission"]["vision_hz_min"])
        if v.fps < fps_min:
            ok = False
            print("  FAIL camera at %.1f fps, need %.1f. Thermal throttling "
                  "or a bad ribbon -- it will not improve during the round."
                  % (v.fps, fps_min))
        else:
            print("  OK  camera %.1f fps, %.1f ms/frame" % (v.fps, v.proc_ms))

    print("mode: %s" % cfg["challenge"]["mode"])
    print(PillarPlanner(cfg).describe_mapping())
    print("=== PRE-FLIGHT %s ===\n" % ("PASS" if ok else "FAIL"))
    return ok



def main():
    ap = argparse.ArgumentParser(description="WRO 2026 Future Engineers -- Pi 5")
    ap.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        "config.json"))
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--mode", choices=("open", "obstacle"), default=None,
                    help="override config.challenge.mode (pit use only)")
    ap.add_argument("--no-camera", action="store_true",
                    help="bench testing without the CSI camera")
    ap.add_argument("--force", action="store_true",
                    help="arm despite pre-flight failures. PRACTICE ONLY -- "
                         "never at a competition; it can bypass the rule "
                         "11.10 radio check.")
    args = ap.parse_args()

    cfg, raw_cfg, problems, missing = load_config(args.config)
    if args.mode:
        cfg["challenge"]["mode"] = args.mode

    log = Logger(cfg)
    log.start(config_snapshot=raw_cfg)

    link = PicoLink(args.port, baud=460800)
    link.start()

    vision = None
    if not args.no_camera:
        from pi.vision import VisionPipeline
        vision = VisionPipeline(cfg)
        vision.open_camera()
        vision.start()

    ok = preflight(cfg, raw_cfg, problems, missing, link, vision, args)
    if not ok and not args.force:
        print("Refusing to arm. Fix the failures above, or re-run with "
              "--force for BENCH TESTING ONLY.")
        log.event("preflight_fail")
        log.close()
        link.close()
        if vision:
            vision.stop()
        return 2
    log.event("preflight_pass", forced=bool(args.force))

    mission = Mission(cfg, link, vision, log, args)
    mission.transition(WAIT_START, "preflight passed")
    print("ARMED. Waiting for the START button on the vehicle (rule 9.11).\n")

    front_stop = int(cfg["pico"]["front_stop_mm"])
    rear_stop = int(cfg["pico"]["rear_stop_mm"])
    last_ping = 0.0
    stale_since = None

    try:
        while True:
       
            t = link.wait_telemetry(timeout=0.2)
            if t is None:
             
                if stale_since is None:
                    stale_since = time.monotonic()
                    log.event("telemetry_stale")
                    print("!! telemetry stale -- the Pico is stopping itself")
                continue
            if stale_since is not None:
                log.event("telemetry_restored",
                          gap_s=round(time.monotonic() - stale_since, 3))
                stale_since = None

            for _ht, ev in link.drain_events():
                log.event("pico_event",
                          name=P.EVENT_NAMES.get(ev["event_id"],
                                                 ev["event_id"]),
                          arg=ev["arg"], pico_ms=ev["t_ms"])

            steer, speed, flags = mission.step(t)
            link.send_command(steer, speed, flags,
                              _state_code(mission.state), front_stop, rear_stop)

            now = time.monotonic()
            if now - last_ping > 1.0:
                last_ping = now
                link.send_ping()

            if mission.state in (FINISHED, EMERGENCY):
                held = time.monotonic() - (mission.finished_at or now)
                if mission.state == EMERGENCY or held > (
                        float(cfg["finish"]["hold_stopped_s"]) +
                        float(cfg["finish"]["hold_extra_s"]) + 2.0):
                    break

    except KeyboardInterrupt:
        print("\ninterrupted")
        log.event("keyboard_interrupt")
    finally:
      
        for _ in range(10):
            link.send_command(0, 0, 0, 0, front_stop, rear_stop)
            time.sleep(0.02)
        print("\n=== SUMMARY ===")
        print("state=%s laps=%d crossings=%d direction=%s pillars=%d t=%.1fs"
              % (mission.state, mission.laps, mission.crossings,
                 mission.direction, mission.pillars_seen, mission.mission_t()))
        print("link: %s" % link.stats())
        print("log:  %s (%d records, %d dropped)"
              % (log.run_dir, log.written, log.dropped))
        log.event("shutdown", state=mission.state, laps=mission.laps,
                  link=link.stats())
        log.close()
        link.close()
        if vision:
            vision.stop()
    return 0


_STATE_CODES = {IDLE: 0, WAIT_START: 1, DRIVE_STRAIGHT: 2, TURN: 3,
                AVOID_PILLAR: 4, SEARCH_PARKING: 5, PARK: 6, FINISHED: 7,
                EMERGENCY: 8}


def _state_code(state):

    return _STATE_CODES.get(state, 0)


if __name__ == "__main__":
    sys.exit(main())
