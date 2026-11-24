"""Line protocol, simulated device and response analysis. No GUI dependencies."""
from collections import deque
import math

import numpy as np

LOOPS = ("ANG", "SPD", "YAW", "AUX")
KEYS = ("KP", "KI", "KD", "TGT")
DEFAULTS = dict(zip(KEYS, (2.0, 1.2, 0.15, 10.0)))
LIMIT = 1_000_000
MODELS = ("一阶惯性", "二阶振荡", "一阶含延迟", "倒立摆（平衡车直立）")


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
    """Four independent loops. The selected loop supplies four scope channels.

    Model 3 is an inverted pendulum (degrees): unstable without feedback, the
    D term acts on the measured angular rate like a gyro, and the device stops
    itself when the tilt exceeds FALL_DEG. Like the firmware example it
    advertises #CAP:PING on RUN and stops after HEARTBEAT_S without @PING once
    the host has started sending heartbeats.
    """
    dt = 0.01
    PENDULUM_G = 40.0      # 1/s^2, gravity / effective length
    PENDULUM_K = 30.0      # deg/s^2 per output unit
    PENDULUM_FRICTION = 0.5
    START_TILT = 1.0       # deg, released slightly off the setpoint
    FALL_DEG = 45.0
    HEARTBEAT_S = 0.5

    def __init__(self, model=0):
        self.model = model
        self.params = {g: dict(DEFAULTS) for g in LOOPS}
        self.running = False
        self.states = {g: [0., 0., 0., 0.] for g in LOOPS}
        self.delay = {g: deque([0.] * 15) for g in LOOPS}
        self.fallen = {g: False for g in LOOPS}
        self.time = 0.
        self.armed = False
        self.last_ping = 0.
        self.outbox = []

    @property
    def pendulum(self):
        return self.model == 3

    def drain(self):
        lines, self.outbox = self.outbox, []
        return lines

    def ping(self):
        if self.running:
            self.armed = True
            self.last_ping = self.time

    def halt(self, reason=None):
        self.running = self.armed = False
        for g, state in self.states.items():
            state[2:] = (0., 0.)
            self.delay[g] = deque([0.] * 15)
        if reason:
            self.outbox.append("#STOP:" + reason)

    def command(self, line):
        op, sep, raw = line.removeprefix("@").partition("=")
        group, key = op.split(".", 1) if "." in op else ("ANG", op)
        if op == "PING":
            self.ping()
            return ""
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
        elif op == "RUN":
            if self.pendulum:
                for g in LOOPS:
                    self.states[g] = [self.params[g]["TGT"] + self.START_TILT, 0., 0., 0.]
                    self.fallen[g] = False
            self.running = True
            self.armed = False
            self.last_ping = self.time
            return "#CAP:PING\n@ACK:RUN"
        elif op == "STOP":
            self.halt()
            if self.pendulum:
                for g in LOOPS:
                    self.states[g] = [0., 0., 0., 0.]
                    self.fallen[g] = False
        elif key == "RST":
            self.states[group][2] = 0.
        else:
            return "@ERR:unknown command"
        return "@ACK:" + line.removeprefix("@")

    def control(self, p, y, velocity, integral):
        target = p["TGT"] if self.running else 0.
        error = target - y
        trial = integral + p["KI"] * error * self.dt
        raw = p["KP"] * error + trial - p["KD"] * velocity
        u = max(-100., min(100., raw)) if self.running else 0.
        if self.running and (abs(raw) <= 100 or (raw - u) * p["KI"] * error < 0):
            integral = trial
        return u, integral

    def pendulum_step(self, group):
        p = self.params[group]
        y, velocity, integral, _ = self.states[group]
        if not self.running and not self.fallen[group]:
            self.states[group] = [0., 0., 0., 0.]
            return
        u, integral = self.control(p, y, velocity, integral)
        if self.fallen[group]:
            u = integral = 0.
        h = self.dt / 10
        for _ in range(10):
            accel = (self.PENDULUM_G * math.degrees(math.sin(math.radians(y)))
                     + self.PENDULUM_K * u - self.PENDULUM_FRICTION * velocity)
            velocity += accel * h
            y += velocity * h
        if abs(y) >= 90.:
            y, velocity = math.copysign(90., y), 0.
        self.states[group] = [y, velocity, integral, u]
        if self.running and abs(y - p["TGT"]) > self.FALL_DEG:
            self.fallen[group] = True

    def step(self, selected):
        dt = self.dt
        self.time += dt
        for group in LOOPS:
            if self.pendulum:
                self.pendulum_step(group)
                continue
            p = self.params[group]
            y, velocity, integral, _ = self.states[group]
            u, integral = self.control(p, y, velocity, integral)
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
        if self.running and self.pendulum and self.fallen[selected]:
            self.halt("fall")
        elif self.running and self.armed and self.time - self.last_ping >= self.HEARTBEAT_S - 1e-9:
            self.halt("heartbeat")
        return [target, y, u, target - y]


def step_metrics(times, targets, actual, band=.02):
    """Most recent step, signed direction, target-referenced amplitude and band.

    Missing samples (NaN) are ignored rather than poisoning every metric.
    """
    if len(times) < 3:
        return None
    changes = np.flatnonzero(~np.isclose(targets[1:], targets[:-1], rtol=1e-7, atol=1e-9)
                             & np.isfinite(targets[1:])) + 1
    if not len(changes):
        return None
    k = changes[-1]
    t = times[k:] - times[k]
    previous = actual[:k][np.isfinite(actual[:k])]
    if not len(previous):
        return None
    y0 = previous[-1]
    target = targets[k]
    amp = target - y0
    if abs(amp) < 1e-9 or len(t) < 2:
        return None
    z = (actual[k:] - y0) / amp
    valid = np.isfinite(z)
    if not valid.any():
        return None
    def crossing(level):
        hit = np.flatnonzero(valid & (z >= level))
        if not len(hit):
            return None
        i = hit[0]
        if i == 0 or not valid[i-1]:
            return float(t[i])
        return t[i-1] + (level-z[i-1]) * (t[i]-t[i-1]) / (z[i]-z[i-1])
    low, high = crossing(.1), crossing(.9)
    outside = np.flatnonzero(valid & (abs(z - 1.) > band))
    last = outside[-1] + 1 if len(outside) else 0
    settle = t[last] if last < len(t) and t[-1] - t[last] >= .5 else None
    tail = actual[k:][(t >= t[-1] - .5) & valid]
    absolute = np.abs(target - actual[k:])
    iae = np.nansum(np.diff(t) * (absolute[:-1] + absolute[1:]) * .5)
    return (max(0., float(np.nanmax(z) - 1) * 100),
            high - low if high is not None and low is not None else None,
            settle, target - float(tail.mean()) if t[-1] >= .5 and len(tail) else None,
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
