"""
Serial master: the Pi 5's end of the Pico link.

THREADING MODEL
---------------
One dedicated reader thread does nothing but drain the serial port and parse
frames. It is the only thread that touches the port for reading. It is
deliberately tiny: read bytes, parse, update a snapshot, notify. No logic, no
disk, no OpenCV.

Why a thread and not a poll from the control loop: the Pico sends 41-byte
telemetry frames at 50 Hz. The kernel's serial buffer is generous, but if the
control loop ever stalls (a GC pause, a slow log flush, a vision frame that
took 60 ms) the frames pile up and we then process a burst of stale ones. A
reader thread keeps the port drained regardless of what the rest of Python is
doing, and the control loop consumes only the LATEST snapshot.

WHY THE CONTROL LOOP RUNS OFF TELEMETRY ARRIVAL
-----------------------------------------------
The Pi does not run its own control timer. It waits on the telemetry event
and computes one control step per telemetry frame, i.e. at exactly the
Pico's jitter-free 50 Hz. Two benefits:
  * We never fight the Linux scheduler for a periodic timer. Everything is
    paced by the microcontroller, which is the board that actually has a
    real-time clock discipline.
  * The steering controller runs at SENSOR rate (50 Hz), not at CAMERA rate
    (~30 fps). Vision changes the setpoint; it does not gate the loop. A
    dropped camera frame therefore cannot make the car stop steering.

TRUST
-----
Every field that crosses this link is treated as suspect. CRC failures,
sequence gaps and decode errors are counted and exposed -- they go in the log
and on the pre-flight display, because a link that is quietly losing 5% of
frames is a link that will lose 100% of them at the worst moment.
"""

import threading
import time

import serial

from common import protocol as P


class PicoLink:

    def __init__(self, port="/dev/ttyAMA0", baud=460800, timeout_s=0.05):
        self.port_name = port
        self.baud = baud
        self.ser = serial.Serial(port, baud, timeout=timeout_s,
                                 write_timeout=0.05)
        self.parser = P.FrameParser()

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self.telemetry_event = threading.Event()

        self.telemetry = None           # latest decoded dict
        self.telemetry_ms = 0.0         # host monotonic time of arrival
        self.telemetry_count = 0
        self.telemetry_gaps = 0
        self._last_telem_seq = None

        self.events = []                # (host_t, event dict) FIFO for the FSM
        self._events_lock = threading.Lock()

        self.tx_seq = 0
        self.tx_count = 0
        self.acks = []
        self.rtt_ms = None
        self.rtt_max_ms = 0.0
        self._ping_sent_ms = {}

        self._reader = threading.Thread(target=self._read_loop, daemon=True,
                                        name="pico-rx")

    # -- lifecycle ---------------------------------------------------------

    def start(self):
        # Flush whatever accumulated while the Pi was booting. Stale frames
        # from before we were listening describe a world that no longer
        # exists.
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass
        self._reader.start()

    def close(self):
        self._stop.set()
        try:
            self._reader.join(timeout=1.0)
        except RuntimeError:
            pass
        try:
            self.ser.close()
        except Exception:
            pass

    # -- reader thread -----------------------------------------------------

    def _read_loop(self):
        while not self._stop.is_set():
            try:
                n = self.ser.in_waiting
                data = self.ser.read(max(n, 1))
            except Exception:
                # A USB-serial adapter yanked mid-round, or the port went
                # away. Do not die: the control loop's own staleness check
                # will notice telemetry stopped and stop the car. Dying here
                # would leave nobody to notice.
                time.sleep(0.01)
                continue
            if not data:
                continue
            for mtype, seq, payload in self.parser.feed(data):
                self._dispatch(mtype, seq, payload)

    def _dispatch(self, mtype, seq, payload):
        now = time.monotonic()
        if mtype == P.MSG_TELEMETRY:
            try:
                t = P.decode_telemetry(payload)
            except Exception:
                return
            if self._last_telem_seq is not None:
                self.telemetry_gaps += P.seq_gap(self._last_telem_seq, seq)
            self._last_telem_seq = seq
            with self._lock:
                self.telemetry = t
                self.telemetry_ms = now
                self.telemetry_count += 1
            self.telemetry_event.set()
        elif mtype == P.MSG_EVENT:
            try:
                e = P.decode_event(payload)
            except Exception:
                return
            with self._events_lock:
                self.events.append((now, e))
                # Bounded: if the FSM ever stops draining these we drop the
                # OLDEST, because the newest event is the one describing the
                # situation we are currently in.
                if len(self.events) > 256:
                    del self.events[:len(self.events) - 256]
        elif mtype == P.MSG_ACK:
            try:
                a = P.decode_ack(payload)
            except Exception:
                return
            if a["acked_type"] == P.MSG_PING:
                sent = self._ping_sent_ms.pop(a["echo_t_ms"], None)
                if sent is not None:
                    self.rtt_ms = (now - sent) * 1000.0
                    self.rtt_max_ms = max(self.rtt_max_ms, self.rtt_ms)
            self.acks.append(a)
            if len(self.acks) > 32:
                del self.acks[:16]

    # -- consumer API ------------------------------------------------------

    def wait_telemetry(self, timeout=0.2):
        """Block until a fresh telemetry frame arrives. Returns the decoded
        dict, or None on timeout (which the caller MUST treat as link loss)."""
        if not self.telemetry_event.wait(timeout):
            return None
        self.telemetry_event.clear()
        with self._lock:
            return self.telemetry

    def latest(self):
        with self._lock:
            return self.telemetry

    def age_s(self):
        with self._lock:
            if self.telemetry is None:
                return float("inf")
            return time.monotonic() - self.telemetry_ms

    def drain_events(self):
        with self._events_lock:
            out = self.events
            self.events = []
        return out

    # -- transmit ----------------------------------------------------------

    def _send(self, mtype, payload):
        self.tx_seq = (self.tx_seq + 1) & 0xFF
        frame = P.encode_frame(mtype, self.tx_seq, payload)
        try:
            self.ser.write(frame)
        except Exception:
            # Never raise into the control loop over a write failure. If the
            # link is genuinely gone the Pico's own 300 ms timeout stops the
            # car -- which is precisely why that timeout exists on the Pico
            # and not here.
            return False
        self.tx_count += 1
        return True

    def send_command(self, steer_cdeg, speed_mm_s, flags, pi_state,
                     front_stop_mm, rear_stop_mm):
        """Also serves as the heartbeat. Deliberately: a separate keepalive
        could keep arriving while the command path was wedged, so the Pico
        would think a dead Pi was healthy. Tying liveness to the command
        stream means the only way to look alive is to still be steering."""
        t_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        return self._send(P.MSG_COMMAND,
                          P.encode_command(t_ms, steer_cdeg, speed_mm_s,
                                           flags, pi_state,
                                           front_stop_mm, rear_stop_mm))

    def send_ping(self):
        t_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF
        self._ping_sent_ms[t_ms] = time.monotonic()
        if len(self._ping_sent_ms) > 8:
            for k in list(self._ping_sent_ms)[:-4]:
                self._ping_sent_ms.pop(k, None)
        return self._send(P.MSG_PING, P.encode_ping(t_ms))

    def send_config(self, pairs):
        """Push tunables from config.json. Chunked to the protocol's 12-pair
        frame limit. Returns the number of frames sent -- the caller must
        wait for the matching ACK count before arming, because a car running
        on firmware defaults is a car with an uncalibrated servo centre."""
        pairs = list(pairs)
        sent = 0
        for i in range(0, len(pairs), P.CFG_MAX_PAIRS):
            chunk = pairs[i:i + P.CFG_MAX_PAIRS]
            if self._send(P.MSG_CONFIG, P.encode_config(chunk)):
                sent += 1
            time.sleep(0.01)     # setup path only, never in the control loop
        return sent

    # -- diagnostics -------------------------------------------------------

    def stats(self):
        return {
            "tx": self.tx_count,
            "rx_telemetry": self.telemetry_count,
            "telemetry_gaps": self.telemetry_gaps,
            "crc_errors": self.parser.frames_crc_err,
            "dropped_bytes": self.parser.frames_dropped_bytes,
            "rtt_ms": self.rtt_ms,
            "rtt_max_ms": self.rtt_max_ms,
        }
