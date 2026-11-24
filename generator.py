"""Repeatable setpoint waveforms. Values are absolute TGT commands; no GUI dependencies."""
import math

SHAPES = ("单次阶跃", "方波", "正弦", "三角波")
DISCRETE = (0, 1)


class TargetGenerator:
    """base: TGT before the test. amplitude: signed excursion. period: seconds.

    单次阶跃 holds base+amplitude for one period, returns to base and finishes.
    方波 alternates every half period. 正弦 and 三角波 swing around base and are
    sent at `rate` Hz; discrete shapes are only sent when the level changes.
    """

    def __init__(self, shape, base, amplitude, period, rate=10.):
        if shape not in range(len(SHAPES)):
            raise ValueError("未知波形")
        if not all(math.isfinite(v) for v in (base, amplitude, period, rate)):
            raise ValueError("发生器参数必须有限")
        if period <= 0 or rate <= 0:
            raise ValueError("周期和节拍必须大于零")
        self.shape, self.base, self.amplitude = shape, base, amplitude
        self.period, self.rate = period, rate
        self.start = None
        self.last_sent = None
        self.next_send = 0.

    def bounds(self):
        ends = [self.base, self.base + self.amplitude]
        if self.shape in (2, 3):
            ends.append(self.base - self.amplitude)
        return min(ends), max(ends)

    def value(self, t):
        """Setpoint at t seconds after start, or None once a single step is complete."""
        a, p = self.amplitude, self.period
        if self.shape == 0:
            return self.base + a if t < p else None
        phase = (t % p) / p
        if self.shape == 1:
            return self.base + (a if phase < .5 else 0.)
        if self.shape == 2:
            return self.base + a * math.sin(2 * math.pi * phase)
        # Triangle: base -> base+a -> base-a -> base, continuous slope.
        tri = 4 * phase if phase < .25 else 2 - 4 * phase if phase < .75 else 4 * phase - 4
        return self.base + a * tri

    def due(self, now):
        """Return the next value to send at monotonic time `now`, None if nothing is due,
        or StopIteration when a single step has completed and base was restored."""
        if self.start is None:
            self.start = now
        value = self.value(now - self.start)
        if value is None:
            if self.last_sent is not None and self.last_sent != self.base:
                return self.mark(self.base, now)
            return StopIteration
        if self.shape in DISCRETE:
            if self.last_sent is not None and abs(value - self.last_sent) < 1e-12:
                return None
            return self.mark(value, now)
        if now < self.next_send:
            return None
        return self.mark(value, now)

    def mark(self, value, now):
        self.last_sent = value
        self.next_send = now + 1. / self.rate
        return value
