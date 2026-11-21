"""Serialize protocol commands, including their group context and write deadlines."""
from collections import deque


class CommandQueue:
    def __init__(self):
        self.waiting = deque()
        self.pending = None
        self.batch = 0
        self.active_batch = None

    def clear(self):
        self.waiting.clear()
        self.pending = None

    def append(self, commands, group, urgent=False):
        if urgent:
            self.clear()
        if len(self.waiting) + len(commands) > 64:
            return False
        self.batch += 1
        self.waiting.extend((cmd, group, urgent,self.batch) for cmd in commands)
        return True

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
