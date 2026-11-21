"""Line protocol, simulated device and response analysis. No GUI dependencies."""
from collections import deque
import math

import numpy as np

LOOPS = ("ANG", "SPD", "YAW", "AUX")
KEYS = ("KP", "KI", "KD", "TGT")
DEFAULTS = dict(zip(KEYS, (2.0, 1.2, 0.15, 10.0)))
LIMIT = 1_000_000


def finite(text):
    value = float(text)
    if not math.isfinite(value):
        raise ValueError("数值必须有限")
    return value


def decode_line(line):
    line = line.strip()
    if not line:
        return "empty", None
    if line.startswith("#NAMES:"):
        names = [s.strip() for s in line[7:].split(",")]
        if 1 <= len(names) <= 4 and all(names):
            return "names", names
        raise ValueError("通道名数量应为 1 至 4")
    if line.startswith("#"):
        return "log", line
    for prefix in ("ACK", "ERR", "PARAM"):
        if line.startswith("@" + prefix + ":"):
            return prefix, line[len(prefix) + 2:].strip()
    # FireWater accepts an optional frame name followed by a colon.
    fields = line.split(",")
    if len(fields) > 4:
        raise ValueError("超过 4 通道")
    values = []
    for field in fields:
        value = finite(field.rsplit(":", 1)[-1])
        if abs(value) > 3.403e38:
            raise ValueError("超出单精度采样范围")
        values.append(value)
    return "data", values


class Lines:
    def __init__(self):
        self.buf = bytearray()
        self.skipping = False

    def feed(self, chunk):
        parts = chunk.split(b"\n")
        for i, part in enumerate(parts):
            complete = i < len(parts) - 1
            if not self.skipping:
                if len(self.buf) + len(part) > 2048:
                    self.buf.clear()
                    self.skipping = True
                    yield "bad", "超长行"
                else:
                    self.buf.extend(part)
            if complete:
                if not self.skipping:
                    try:
                        yield decode_line(self.buf.decode("utf-8"))
                    except (UnicodeError, ValueError, OverflowError) as exc:
                        yield "bad", str(exc)
                self.skipping = False
                self.buf.clear()


def parameter_pairs(payload, default_loop):
    result = {}
    for item in payload.split(","):
        lhs, value = item.strip().split("=", 1)
        group, key = lhs.split(".", 1) if "." in lhs else (default_loop, lhs)
        if group not in LOOPS or key not in KEYS:
            raise ValueError("未知参数")
        result[group + "." + key] = finite(value)
    return result


class Simulator:
    """Four independent loops. The selected loop supplies four scope channels."""
    dt = 0.01

    def __init__(self, model=0):
        self.model = model
        self.params = {g: dict(DEFAULTS) for g in LOOPS}
        self.running = False
        self.states = {g: [0., 0., 0., 0.] for g in LOOPS}
        self.delay = {g: deque([0.] * 15) for g in LOOPS}

    def command(self, line):
        op, sep, raw = line.removeprefix("@").partition("=")
        group, key = op.split(".", 1) if "." in op else ("ANG", op)
        if group not in LOOPS:
            return "@ERR:unknown loop"
        if key == "GET":
            data = ",".join(f"{group}.{k}={v:g}" for k, v in self.params[group].items())
            return "@PARAM:" + data
        if key in KEYS and sep:
            try:
                val = finite(raw)
                if abs(val) > LIMIT:
                    raise ValueError()
            except ValueError:
                return "@ERR:range"
            self.params[group][key] = val
        elif op in ("RUN", "STOP"):
            self.running = op == "RUN"
            if not self.running:
                for g, state in self.states.items():
                    state[2:] = (0., 0.)
                    self.delay[g] = deque([0.] * 15)
        elif key == "RST":
            self.states[group][2] = 0.
        else:
            return "@ERR:unknown command"
        return "@ACK:" + line.removeprefix("@")

    def step(self, selected):
        dt = self.dt
        for group in LOOPS:
            p = self.params[group]
            y, velocity, integral, _ = self.states[group]
            target = p["TGT"] if self.running else 0.
            error = target - y
            trial = integral + p["KI"] * error * dt
            raw = p["KP"] * error + trial - p["KD"] * velocity
            u = max(-100., min(100., raw)) if self.running else 0.
            if self.running and (abs(raw) <= 100 or (raw - u) * p["KI"] * error < 0):
                integral = trial
            drive = u
            if self.model == 2:
                self.delay[group].append(u)
                drive = self.delay[group].popleft()
            if self.model == 1:
                velocity += (7 * (drive - y) - 2.2 * velocity) * dt
            else:
                velocity = (drive - y) / 0.55
            y += velocity * dt
            self.states[group] = [y, velocity, integral, u]
        y, _, _, u = self.states[selected]
        target = self.params[selected]["TGT"] if self.running else 0.
        return [target, y, u, target - y]


def step_metrics(times, targets, actual, band=.02):
    """Most recent step, signed direction, target-referenced amplitude and band."""
    if len(times) < 3:
        return None
    changes = np.flatnonzero(~np.isclose(targets[1:], targets[:-1], rtol=1e-7, atol=1e-9)) + 1
    if not len(changes):
        return None
    k = changes[-1]
    t = times[k:] - times[k]
    y0 = actual[k - 1]
    target = targets[k]
    amp = target - y0
    if abs(amp) < 1e-9 or len(t) < 2:
        return None
    z = (actual[k:] - y0) / amp
    def crossing(level):
        hit = np.flatnonzero(z >= level)
        if not len(hit):
            return None
        i = hit[0]
        if i == 0:
            return 0.
        return t[i-1] + (level-z[i-1]) * (t[i]-t[i-1]) / (z[i]-z[i-1])
    low, high = crossing(.1), crossing(.9)
    outside = np.flatnonzero(abs(z - 1.) > band)
    last = outside[-1] + 1 if len(outside) else 0
    settle = t[last] if last < len(t) and t[-1] - t[last] >= .5 else None
    tail = actual[k:][t >= t[-1] - .5]
    err = target - actual[k:]
    absolute = np.abs(err)
    iae = np.sum(np.diff(t) * (absolute[:-1] + absolute[1:]) * .5)
    return (max(0., float(z.max() - 1) * 100),
            high - low if high is not None and low is not None else None,
            settle, target - float(tail.mean()) if t[-1] >= .5 else None,
            float(iae))


def match_ack(command, payload):
    key, sep, raw = command.lstrip("@").partition("=")
    received, has_value, value = payload.partition("=")
    if received != key or bool(sep) != bool(has_value):
        return False
    if not sep:
        return True
    try:
        return math.isclose(finite(raw), finite(value), abs_tol=0.00051, rel_tol=1e-6)
    except ValueError:
        return False

