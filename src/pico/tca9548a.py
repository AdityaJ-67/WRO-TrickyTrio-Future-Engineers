"""
TCA9548A 8-channel I2C multiplexer -- Pico 2 W / MicroPython.

WHY THIS PART EXISTS AT ALL
---------------------------
All five VL53L0X ToF sensors are hard-wired to I2C address 0x29 and the PCB
has NO XSHUT wiring, so the usual "bring them up one at a time and reassign
addresses" trick is unavailable. The mux resolves the collision in hardware:
exactly one downstream channel is connected to the upstream bus at a time,
so five identical addresses never coexist.

The TCS34725 also lives at 0x29. It sits on its own channel (1) for exactly
the same reason. That is not an accident of layout -- it is the mux earning
its place twice.

TWO INVARIANTS, BOTH ENFORCED HERE
----------------------------------
1. NEVER enable two channels at once.
   Electrically it is an address collision: two devices ACK the same address
   and the bus reads back garbage that is not obviously garbage.
   Also a POWER problem: the whole 3.3 V sensor rail hangs off the Pico's
   3V3 OUT pin, rated ~300 mA. Seven sensors plus the mux fit inside that
   budget ONLY because a deselected VL53L0X is idle. Two VCSELs firing
   simultaneously is how you brown out your own sensor rail and spend an
   evening debugging "random I2C failures".
   Enforcement: select() writes a ONE-HOT byte computed from a channel
   NUMBER. There is no API here that takes a bitmask, so multi-select is
   not merely discouraged, it is unrepresentable.

2. NEVER let two readers interleave a channel switch.
   A pre-empted "select ch3; read 0x29" becomes "select ch3; select ch4;
   read 0x29" and you get the front-right sensor's range labelled as
   front-left -- a silent, plausible, wrong number. That is the worst class
   of bug in this vehicle.
   Enforcement: the `channel()` context manager takes a lock. On the Pico we
   drive all I2C from the single control loop, so this lock is normally
   uncontended -- it is an ASSERTION that our single-threaded assumption
   still holds, and it costs microseconds. If someone later adds a REPL
   thread or an IRQ-driven read, this catches it instead of the field doing.
"""

import time

try:
    import _thread
    _HAVE_THREAD = True
except ImportError:
    _HAVE_THREAD = False


class MuxError(Exception):
    pass


class MuxBusy(MuxError):
    """Raised when a second caller tries to switch channels mid-transaction."""
    pass


class TCA9548A(object):

    def __init__(self, i2c, address=0x70):
        self.i2c = i2c
        self.address = address
        self._current = None          # last channel we believe is selected
        self._depth = 0               # re-entrancy / interleave detector
        self._lock = _thread.allocate_lock() if _HAVE_THREAD else None
        self.switch_count = 0
        self.error_count = 0
        # Cache the one-hot bytes so the hot path allocates nothing. In
        # MicroPython an allocation inside a 10 ms control tick can trigger a
        # GC pass at the worst possible moment.
        self._onehot = [bytes([1 << c]) for c in range(8)]
        self._zero = bytes([0x00])

    # -- low level ---------------------------------------------------------

    def probe(self):
        """True if the mux ACKs. Called by the boot self-test; if this fails
        nothing else on the vehicle can be read, so it is fatal."""
        try:
            return self.address in self.i2c.scan()
        except OSError:
            return False

    def _write_raw(self, payload):
        try:
            self.i2c.writeto(self.address, payload)
            return True
        except OSError:
            self.error_count += 1
            return False

    def read_control(self):
        """Read back the control register. Used by the self-test to prove the
        mux actually latched what we wrote -- a mux that ACKs but does not
        switch produces five identical readings from one sensor, which looks
        exactly like a working car right up until it does not."""
        try:
            return self.i2c.readfrom(self.address, 1)[0]
        except OSError:
            self.error_count += 1
            return None

    def select(self, ch):
        """Enable exactly one channel. `ch` is a NUMBER, never a mask."""
        if not 0 <= ch <= 7:
            raise ValueError("mux channel out of range: %r" % (ch,))
        if ch == self._current:
            return True               # already there; skip the bus traffic
        if self._write_raw(self._onehot[ch]):
            self._current = ch
            self.switch_count += 1
            return True
        # A failed switch must invalidate our cached state. Believing we are
        # on ch3 when the write failed means the next read is attributed to
        # whatever channel really is live.
        self._current = None
        return False

    def deselect_all(self):
        """Disconnect every channel. Used on fault recovery: it is the only
        way to force a hung downstream device to stop holding SDA low from
        the upstream bus's point of view."""
        if self._write_raw(self._zero):
            self._current = 0xFF      # sentinel: 'known, and it is none'
            return True
        self._current = None
        return False

    # -- the sanctioned access pattern -------------------------------------

    def channel(self, ch):
        """`with mux.channel(2): ...` -- the ONLY approved way to touch a
        downstream device. Returns a context manager."""
        return _ChannelCtx(self, ch)

    # -- recovery ----------------------------------------------------------

    def recover_bus(self):
        """Best-effort unwedge.

        With no XSHUT lines, a VL53L0X that has locked up mid-transfer while
        holding SDA low is normally unrecoverable without a power cycle. The
        mux gives us one lever the blueprint did not anticipate: deselecting
        the channel physically disconnects the offending device from the
        upstream bus, so the bus recovers even though the SENSOR has not.
        We then re-init that sensor through a fresh channel selection.

        Returns True if the mux still answers afterwards.
        """
        self.deselect_all()
        time.sleep_ms(5)              # boot/recovery path only, never in-loop
        self._current = None
        return self.probe()


class _ChannelCtx(object):
    __slots__ = ("mux", "ch", "prev")

    def __init__(self, mux, ch):
        self.mux = mux
        self.ch = ch
        self.prev = None

    def __enter__(self):
        m = self.mux
        if m._lock is not None and not m._lock.acquire(0):
            # Non-blocking acquire: in a real-time loop we would rather fail
            # fast and skip this sample than block and blow the tick budget.
            raise MuxBusy("mux busy: concurrent channel switch attempted")
        m._depth += 1
        if m._depth != 1:
            m._depth -= 1
            if m._lock is not None:
                m._lock.release()
            raise MuxBusy("nested mux channel selection")
        if not m.select(self.ch):
            m._depth -= 1
            if m._lock is not None:
                m._lock.release()
            raise MuxError("mux select ch%d failed" % self.ch)
        return m.i2c

    def __exit__(self, exc_type, exc, tb):
        m = self.mux
        m._depth -= 1
        if m._lock is not None:
            m._lock.release()
        # NOTE: we deliberately do NOT deselect on exit. Leaving the channel
        # live costs nothing (only one is ever on) and saves a bus
        # transaction per access; select() already short-circuits when the
        # requested channel is already current, which on our schedule saves
        # roughly 25% of all mux writes. Fault paths call deselect_all()
        # explicitly when isolation actually matters.
        return False
