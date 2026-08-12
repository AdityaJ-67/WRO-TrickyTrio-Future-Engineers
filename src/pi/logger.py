"""
Structured logging that CANNOT block the control loop.

THE RULE THIS FILE EXISTS TO ENFORCE
------------------------------------
Logging is for the engineering journal, not for the round. It must never be
able to make the car worse. Every design decision here follows from that:

  * The control loop only ever does `queue.put_nowait()`. It never formats a
    string, never touches the filesystem, never waits on a lock held by a
    writer.
  * The queue is BOUNDED. If the SD card stalls -- and cheap cards do stall
    for tens of milliseconds -- the queue fills, and we DROP the oldest
    samples and count them. We never apply back-pressure to the loop. A log
    with a hole in it is a minor inconvenience; a control loop that missed
    three ticks is a wall strike.
  * A single background thread does all formatting and I/O, and flushes on a
    timer rather than per record.

WHAT WE LOG AND WHY
-------------------
Two streams, because they answer different questions:

  telemetry.csv  -- one row per control step: every raw sensor value, the
                    computed setpoint, the commanded output. This is what
                    you plot for the journal. CSV rather than JSON because
                    it is a third the size and pandas eats it directly.
  events.jsonl   -- state transitions, line crossings, pillar decisions,
                    faults. One JSON object per line, arbitrary fields. This
                    is what you READ at 2 a.m. to find out what the car
                    thought it was doing.

The dropped-record count is itself logged at shutdown. A journal plot with
silent holes in it is worse than one with a note saying "142 samples dropped
between t=61 and t=63".
"""

import json
import os
import queue
import threading
import time


TELEMETRY_COLUMNS = [
    "t", "pico_ms", "state", "pico_state",
    "tof_fc", "tof_fl", "tof_fr", "tof_diag", "tof_rear",
    "tof_valid", "tof_degraded",
    "yaw", "yaw_rate", "enc", "speed", "colour_class", "colour_clear",
    "lane_est", "lateral", "lat_err", "lat_source", "offset_target",
    "steer_cmd", "speed_cmd", "front_stop",
    "pillar_colour", "pillar_range", "pillar_bearing",
    "laps", "crossings", "flags", "faults", "vbat", "fps", "rtt",
]


class Logger:

    def __init__(self, cfg, run_id=None):
        lc = cfg["logging"]
        self.enabled = bool(lc["enabled"])
        self.dir = lc["dir"]
        self.flush_interval = float(lc["flush_interval_s"])
        self.q = queue.Queue(maxsize=int(lc["queue_max"]))
        self.dropped = 0
        self.written = 0
        self._stop = threading.Event()
        self._thread = None
        self._tel_f = None
        self._ev_f = None
        self.run_id = run_id or time.strftime("%Y%m%d-%H%M%S")
        self.run_dir = os.path.join(self.dir, self.run_id)

    # -- lifecycle ---------------------------------------------------------

    def start(self, config_snapshot=None):
        if not self.enabled:
            return
        os.makedirs(self.run_dir, exist_ok=True)
        self._tel_f = open(os.path.join(self.run_dir, "telemetry.csv"), "w",
                           buffering=1 << 16)
        self._tel_f.write(",".join(TELEMETRY_COLUMNS) + "\n")
        self._ev_f = open(os.path.join(self.run_dir, "events.jsonl"), "w",
                          buffering=1 << 16)
        if config_snapshot is not None:
            # The exact config this run used, frozen next to the data. Six
            # months later, "which thresholds produced this plot" is a
            # question you cannot answer any other way.
            with open(os.path.join(self.run_dir, "config.snapshot.json"),
                      "w") as f:
                json.dump(config_snapshot, f, indent=2, sort_keys=True)
        self._thread = threading.Thread(target=self._writer, daemon=True,
                                        name="logger")
        self._thread.start()
        self.event("logger_start", run_id=self.run_id)

    def close(self):
        if not self.enabled:
            return
        self.event("logger_stop", written=self.written, dropped=self.dropped)
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        for f in (self._tel_f, self._ev_f):
            if f:
                try:
                    f.flush()
                    os.fsync(f.fileno())
                    f.close()
                except Exception:
                    pass

    # -- producer side: called from the control loop, must never block -----

    def telemetry(self, row):
        """`row` is a dict keyed by TELEMETRY_COLUMNS. Missing keys become
        empty cells -- we would rather log a partial row than raise inside
        the control loop over a typo'd key name."""
        if not self.enabled:
            return
        self._put(("T", row))

    def event(self, kind, **fields):
        if not self.enabled:
            return
        fields["t"] = time.monotonic()
        fields["kind"] = kind
        self._put(("E", fields))

    def _put(self, item):
        try:
            self.q.put_nowait(item)
        except queue.Full:
            # Drop the OLDEST, keep the newest. When the disk is misbehaving
            # the recent past is what you need; the distant past is already
            # on disk.
            try:
                self.q.get_nowait()
                self.dropped += 1
                self.q.put_nowait(item)
            except (queue.Empty, queue.Full):
                self.dropped += 1

    # -- consumer side -----------------------------------------------------

    def _writer(self):
        last_flush = time.monotonic()
        while not self._stop.is_set() or not self.q.empty():
            try:
                kind, payload = self.q.get(timeout=0.1)
            except queue.Empty:
                kind = None
            if kind == "T":
                self._tel_f.write(",".join(
                    _fmt(payload.get(c)) for c in TELEMETRY_COLUMNS) + "\n")
                self.written += 1
            elif kind == "E":
                self._ev_f.write(json.dumps(payload, default=str) + "\n")
                self.written += 1
            now = time.monotonic()
            if now - last_flush >= self.flush_interval:
                last_flush = now
                try:
                    self._tel_f.flush()
                    self._ev_f.flush()
                except Exception:
                    pass


def _fmt(v):
    if v is None:
        return ""
    if isinstance(v, float):
        return "%.3f" % v
    if isinstance(v, str):
        return v.replace(",", ";")
    return str(v)
