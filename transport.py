"""One worker owns the port. Bytes and control events reach Qt in bounded batches."""
from collections import deque
from threading import Event, Lock, Thread
from time import monotonic
import serial


class SerialLink(Thread):
    def __init__(self, options):
        super().__init__(daemon=True)
        self.options = options
        self.closed = Event()
        self.lock = Lock()
        self.writes = deque()
        self.ping_due = False
        self.events = deque()
        self.queued_bytes = 0
        self.dropped = 0

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
                        if self.ping_due:
                            data = b"@PING\n"
                            self.ping_due = False
                        else:
                            data = self.writes.popleft() if self.writes else None
                    if data is not None:
                        if port.write(data) != len(data):
                            raise IOError("串口写入不完整")
                        self.event("tx", data)
                    data = port.read(min(32768, port.in_waiting or 1))
                    if data:
                        self.event("rx", data)
        except (OSError, ValueError, serial.SerialException) as exc:
            self.event("error", str(exc))
        finally:
            self.event("closed", None)

