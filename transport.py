"""One worker owns the port. Bytes and control events reach Qt in bounded batches.

Heartbeats are written by this thread every PING_PERIOD while the UI keeps calling
keepalive(True). A brief UI stall (dragging the window, a modal dialog) does not
starve the device; if the UI stops refreshing for UI_GRACE seconds the pings stop
and the firmware watchdog halts the motors as intended.
"""
from collections import deque
from threading import Event, Lock, Thread
from time import monotonic
import serial

PING_PERIOD = .2
UI_GRACE = 1.
HEARTBEAT = b"@PING\n"   # identity marks thread-generated heartbeats (not logged as TX)


class SerialLink(Thread):
    def __init__(self, options):
        super().__init__(daemon=True)
        self.options = options
        self.closed = Event()
        self.lock = Lock()
        self.writes = deque()
        self.ping_due = False
        self.heartbeat = False
        self.alive_at = 0.
        self.next_ping = 0.
        self.pings = 0
        self.events = deque()
        self.queued_bytes = 0
        self.dropped = 0
        self._modem_lines = None

    def set_modem_lines(self, dtr=None, rts=None):
        with self.lock:
            self._modem_lines = (dtr, rts)

    def send(self, data, urgent=False):
        with self.lock:
            if urgent:
                self.writes.clear()
                self.ping_due = False
            if len(self.writes) >= 64:
                return False
            self.writes.append(data)
            return True

    def ping(self):
        with self.lock:
            self.ping_due = True

    def keepalive(self, enabled):
        """Called from the UI timer. Enables thread-side heartbeats and proves the UI is alive."""
        with self.lock:
            if enabled and not self.heartbeat:
                self.next_ping = 0.
            self.heartbeat = enabled
            self.alive_at = monotonic()

    def next_write(self, now):
        with self.lock:
            if self.ping_due:
                self.ping_due = False
                return HEARTBEAT
            if self.writes:
                return self.writes.popleft()
            if self.heartbeat and now >= self.next_ping and now - self.alive_at <= UI_GRACE:
                self.next_ping = now + PING_PERIOD
                return HEARTBEAT
        return None

    def event(self, kind, data):
        with self.lock:
            if kind == "rx":
                if self.queued_bytes + len(data) > 2_000_000:
                    self.dropped += len(data)
                    if not self.events or self.events[-1][0] != "gap":
                        self.events.append(("gap", monotonic(), None))
                    return
                self.queued_bytes += len(data)
            self.events.append((kind, monotonic(), data))

    def take(self):
        with self.lock:
            batch, self.events = self.events, deque()
            lost, self.dropped = self.dropped, 0
            self.queued_bytes = 0
            return batch, lost

    def run(self):
        try:
            with serial.serial_for_url(**self.options, timeout=.012, write_timeout=.25) as port:
                self.event("open", port.port)
                while not self.closed.is_set():
                    with self.lock:
                        lines, self._modem_lines = self._modem_lines, None
                    if lines is not None:
                        try:
                            if lines[0] is not None:
                                port.dtr = bool(lines[0])
                            if lines[1] is not None:
                                port.rts = bool(lines[1])
                        except (AttributeError, OSError, serial.SerialException):
                            pass
                    data = self.next_write(monotonic())
                    if data is not None:
                        heartbeat = data is HEARTBEAT
                        if port.write(data) != len(data):
                            raise IOError("串口写入不完整")
                        if heartbeat:
                            self.pings += 1
                        else:
                            self.event("tx", data)
                    data = port.read(min(32768, port.in_waiting or 1))
                    if data:
                        self.event("rx", data)
        except (OSError, ValueError, serial.SerialException) as exc:
            self.event("error", str(exc))
        finally:
            self.event("closed", None)
