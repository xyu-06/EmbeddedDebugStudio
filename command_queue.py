"""Serialize protocol commands, including their group context and write deadlines."""
from collections import deque


class CommandQueue:
    def __init__(self):
        self.waiting = deque()
        self.pending = None
        self.batch = 0
        self.active_batch = None
        self.quiet_batches = set()

    def clear(self):
        self.waiting.clear()
        self.pending = None

    def append(self, commands, group, urgent=False, quiet=False):
        if urgent:
            self.clear()
        if len(self.waiting) + len(commands) > 64:
            return False
        self.batch += 1
        if quiet:
            self.quiet_batches = {b for b in self.quiet_batches if b > self.batch - 64}
            self.quiet_batches.add(self.batch)
        self.waiting.extend((cmd, group, urgent,self.batch) for cmd in commands)
        return True

    def active_is_quiet(self):
        return self.active_batch in self.quiet_batches

    def busy(self):
        return bool(self.pending or self.waiting)

    def next_command(self):
        if self.pending or not self.waiting:
            return None
        cmd, group, urgent, self.active_batch = self.waiting.popleft()
        self.pending = (cmd, group, None)
        return cmd, urgent

    def written(self, data, stamp):
        if self.pending and data == (self.pending[0]+"\n").encode():
            self.pending = (*self.pending[:2], stamp+.3)

    def timed_out(self, now):
        return self.pending and self.pending[2] is not None and now >= self.pending[2]
