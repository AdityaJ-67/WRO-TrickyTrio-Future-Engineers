"""
Config loading, validation and the Pico config push.

Deliberately a separate module with NO heavy dependencies -- no pyserial, no
OpenCV, no picamera2. That means tools/test_control.py can validate the
config on any laptop, and a teammate can sanity-check config.json in the pit
on a machine that has never had the robot's dependencies installed:

    python3 -c "from pi.config import load_config; print(load_config('config.json')[2:])"

Config problems are the single most common cause of a car that "worked
yesterday". Making them checkable without hardware is worth one small file.
"""

import json

from common import protocol as P


# Hard-coded rule constants. config.json carries an identical copy purely so
# every number is visible in one place; if the two ever disagree we refuse to
# arm. A rule dimension is NOT a tunable, and a late-night "tweak" to one
# must not be able to silently change the mission logic.
FIELD_EXPECTED = {
    "track_outer_mm": 3000,
    "wall_height_mm": 100,
    "lane_width_obstacle_mm": 1000,
    "lane_width_open_min_mm": 600,
    "lane_width_open_max_mm": 1000,
    "pillar_size_mm": [50, 50, 100],
    "pillar_move_circle_mm": 85,
    "parking_wall_mm": [200, 20, 100],
    "parking_lot_width_mm": 200,
    "line_thickness_mm": 20,
    "start_zone_mm": [200, 500],
    "corners_per_lap": 4,
    "laps_required": 3,
}

# Values that CANNOT be guessed from a datasheet or a drawing. The pre-flight
# check refuses to arm while any of these is still null. Refusing loudly is
# strictly better than driving on a plausible-looking invention -- a guessed
# um_per_count silently scales every distance in the mission.
REQUIRED_MEASURED = [
    ("robot", "length_mm"),
    ("robot", "width_mm"),
    ("robot", "wheelbase_mm"),
    ("robot", "min_turn_radius_mm"),
    ("robot", "tof_front_offset_mm"),
    ("robot", "tof_rear_offset_mm"),
    ("robot", "tof_side_offset_mm"),
    ("camera", "mount_height_mm"),
    ("camera", "mount_tilt_deg"),
    ("pico", "um_per_count"),
]


def strip_docs(obj):
    """Drop the '_'-prefixed documentation keys.

    JSON has no comments, so config.json documents itself with sibling keys
    like "_cruise_mm_s". Stripping them here means the runtime config is
    exactly the tunables and nothing else, and a typo'd doc key can never be
    mistaken for a setting.
    """
    if isinstance(obj, dict):
        return {k: strip_docs(v) for k, v in obj.items()
                if not k.startswith("_")}
    if isinstance(obj, list):
        return [strip_docs(v) for v in obj]
    return obj


def load_config(path):
    """Returns (cfg, raw, problems, missing).

    `problems` are rule-constant mismatches; `missing` are unmeasured values.
    Both are returned rather than raised, so pre-flight can print EVERY
    failure at once. Fixing one problem per run, five runs in a row, is how
    you lose a practice session.
    """
    with open(path) as f:
        raw = json.load(f)
    cfg = strip_docs(raw)

    problems = []
    for k, expected in FIELD_EXPECTED.items():
        got = cfg["field"].get(k)
        if got != expected:
            problems.append("field.%s = %r but the rules say %r"
                            % (k, got, expected))

    missing = [".".join(p) for p in REQUIRED_MEASURED
               if cfg.get(p[0], {}).get(p[1]) is None]

    # Derived, never hard-coded: the lot is 1.5x the robot's ACTUAL length,
    # so it changes the moment somebody adds a bumper.
    if cfg["robot"]["length_mm"]:
        cfg["field"]["parking_lot_len_mm"] = (
            cfg["field"]["parking_lot_len_factor"] * cfg["robot"]["length_mm"])

    return cfg, raw, problems, missing


def config_to_pico_pairs(cfg):
    """The subset of config.json the Pico needs, as (key_id, int) pairs.

    Fixed-point integers only -- no floats on the wire (see the note in
    common/protocol.py). Anything the Pico does not recognise is ignored by
    the firmware rather than rejected, so a newer config against an older
    firmware degrades loudly (fewer keys acknowledged) instead of silently
    misparsing.
    """
    p = cfg["pico"]
    return [
        (P.CFG_SERVO_CENTRE_US,   int(p["servo_centre_us"])),
        (P.CFG_SERVO_LEFT_US,     int(p["servo_left_us"])),
        (P.CFG_SERVO_RIGHT_US,    int(p["servo_right_us"])),
        (P.CFG_STEER_LEFT_CDEG,   int(p["steer_left_cdeg"])),
        (P.CFG_STEER_RIGHT_CDEG,  int(p["steer_right_cdeg"])),
        (P.CFG_SERVO_SLEW_CDEG_S, int(p["servo_slew_cdeg_s"])),
        (P.CFG_UM_PER_COUNT,      int(p["um_per_count"] or 0)),
        (P.CFG_FRONT_STOP_MM,     int(p["front_stop_mm"])),
        (P.CFG_REAR_STOP_MM,      int(p["rear_stop_mm"])),
        (P.CFG_MOTOR_RAMP_PCT_S,  int(p["motor_ramp_pct_s"])),
        (P.CFG_MOTOR_MAX_PCT,     int(p["motor_max_pct"])),
        (P.CFG_SPEED_KP_X100,     int(p["speed_kp_x100"])),
        (P.CFG_SPEED_KI_X100,     int(p["speed_ki_x100"])),
        (P.CFG_LINK_TIMEOUT_MS,   int(p["link_timeout_ms"])),
        (P.CFG_LOW_BATT_MV,       int(p["low_batt_mv"])),
        (P.CFG_VBAT_DIV_X100,     int(p["vbat_div_x100"])),
    ]
