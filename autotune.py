"""Closed-loop PID auto-tuning engine. Pure Python + NumPy, no GUI dependencies.

Combines two stages using only standard setpoint/parameter commands (@KP, @KI, @KD,
@TGT, @RST) so existing STM32 firmware works without any protocol changes:
  1. One closed-loop proportional probe step estimates plant gain, dominant lag,
     apparent dead-time (FOPDT -> Skogestad SIMC) or underdamped 2nd-order poles.
  2. Hooke-Jeeves pattern search with physics-guided probing Directions evaluates
     real step responses under a dimensionless cost J (IAE + ITAE + tail error +
     overshoot + output/measurement chatter) and keeps the verified best gains.
"""
from dataclasses import dataclass
import math
import numpy as np

MODES = ("PI 整定（控速推荐）", "PID 整定（位置/姿态）", "PD 整定（直立/快响）")
STYLES = ("平稳无超调", "均衡推荐", "快速响应")
STRATEGIES = ("自动辨识 + 闭环寻优（推荐）", "从当前参数局部微调（平衡车/已有初值）")

MODE_KEYS = (("KP", "KI"), ("KP", "KI", "KD"), ("KP", "KD"))

# (w_iae, w_itae, w_tail, w_os, os_free, w_chatter, tau_factor, delay_factor, zeta_target)
STYLE_PROFILES = (
    (1.0, 1.6, 3.2, 4.8, 0.005, 0.35, 0.35, 2.0, 0.98),
    (1.3, 1.8, 3.0, 2.4, 0.020, 0.25, 0.20, 1.2, 0.88),
    (1.8, 1.5, 2.8, 1.1, 0.060, 0.15, 0.12, 0.8, 0.78),
)


@dataclass
class StepEvaluation:
    score: float
    overshoot_pct: float
    rise_s: float | None
    settle_s: float | None
    tail_error: float
    chatter: float
    saturated: bool = False
    reversed_sign: bool = False
    note: str = ""
    sat_ratio: float = 0.0
    sample_count: int = 0
    valid: bool = True
    stalled: bool = False


def evaluate_step(times, actual, base_y, goal_y, outputs=None, style=1, output_limit=None):
    """Score a step trajectory from base_y to goal_y. Lower score is better."""
    times = np.asarray(times, dtype=float)
    actual = np.asarray(actual, dtype=float)
    valid_mask = np.isfinite(times) & np.isfinite(actual)
    times, actual = times[valid_mask], actual[valid_mask]
    amp = goal_y - base_y
    scale = abs(amp)
    if len(times) < 6 or scale < 1e-6 or times[-1] - times[0] < 0.15:
        return StepEvaluation(
            1e6, 0., None, None, float("nan"), 0.,
            note="采样不足", sample_count=int(len(times)), valid=False,
        )

    t = times - times[0]
    duration = float(t[-1])
    z = (actual - base_y) / amp
    err_norm = np.abs(goal_y - actual) / scale
    dt = np.diff(t)

    niae = float(np.sum(dt * (err_norm[:-1] + err_norm[1:]) * 0.5) / duration)
    weighted = t * err_norm
    nitae = float(2.0 * np.sum(dt * (weighted[:-1] + weighted[1:]) * 0.5) / (duration * duration))

    tail_mask = t >= duration * 0.78
    if not np.any(tail_mask):
        tail_mask = slice(-max(2, len(t) // 5), None)
    tail_err = float(np.mean(err_norm[tail_mask]))
    signed_tail = float(np.mean(goal_y - actual[tail_mask]))

    overshoot = max(0., float(np.max(z) - 1.))
    undershoot = max(0., float(-np.min(z)))
    reversed_sign = float(np.mean(z[tail_mask])) < -0.08

    def crossing(level):
        hit = np.flatnonzero(z >= level)
        if not len(hit):
            return None
        i = int(hit[0])
        if i == 0 or z[i] == z[i - 1]:
            return float(t[i])
        return float(t[i - 1] + (level - z[i - 1]) * (t[i] - t[i - 1]) / (z[i] - z[i - 1]))

    low, high = crossing(0.1), crossing(0.9)
    rise = (high - low) if (low is not None and high is not None and high >= low) else None

    outside = np.flatnonzero(np.abs(z - 1.) > 0.02)
    last_out = int(outside[-1]) + 1 if len(outside) else 0
    settle = float(t[last_out]) if last_out < len(t) and duration - t[last_out] >= min(0.35, duration * 0.2) else None

    # Measure high-frequency ripple in the latter half + control output chatter if available.
    late = actual[t >= duration * 0.5]
    y_ripple = float(np.sum(np.abs(np.diff(late))) / scale) if len(late) >= 3 else 0.
    # A clean monotonic curve can change by at most ~1*scale in the second half; excess is oscillation/noise.
    y_chatter = max(0., y_ripple - 0.35) / max(1., len(late) / 25.)

    u_chatter = 0.
    saturated = False
    sat_ratio = 0.0
    if outputs is not None:
        u = np.asarray(outputs, dtype=float)
        u = u[np.isfinite(u)]
        if len(u) >= 6:
            u_span = max(float(np.ptp(u)), abs(float(np.mean(u))), 1.0)
            du = np.abs(np.diff(u))
            # Allow one initial kick + smooth decay (~2*u_span total variation); penalize excess buzz.
            excess_tv = max(0., float(np.sum(du)) / u_span - 2.5)
            u_chatter = excess_tv / max(1., len(u) / 25.)
            if output_limit is not None and float(output_limit) > 0:
                sat_ratio = float(np.mean(np.abs(u) >= 0.95 * float(output_limit)))
                if (sat_ratio > 0.25 and tail_err > 0.03) or sat_ratio >= 0.65:
                    saturated = True
            else:
                u_max = float(np.max(np.abs(u)))
                if u_max > 1e-3:
                    near_rail = float(np.mean(np.abs(u) >= 0.985 * u_max))
                    if near_rail > 0.82 and float(np.std(u[len(u) // 3:])) < 0.02 * u_max and tail_err > 0.03:
                        saturated = True
                        sat_ratio = near_rail

    chatter = y_chatter + 0.7 * u_chatter
    w_iae, w_itae, w_tail, w_os, os_free, w_chat, *_ = STYLE_PROFILES[style]
    score = (
        w_iae * niae
        + w_itae * nitae
        + w_tail * tail_err
        + w_os * max(0., overshoot - os_free)
        + 3.0 * max(0., overshoot - 0.25) ** 2
        + 1.8 * undershoot
        + w_chat * chatter
    )
    if reversed_sign:
        score += 50.
    tail_z = abs(float(np.mean(z[tail_mask])))
    no_response = bool((not reversed_sign) and tail_z < 0.03 and float(np.max(np.abs(z))) < 0.05)
    stalled = bool((saturated or sat_ratio >= 0.50) and not reversed_sign and tail_z < 0.05)
    is_valid = not reversed_sign and not stalled and not no_response
    note = (
        "反向响应" if reversed_sign
        else "输出触顶且反馈无响应" if stalled
        else "反馈无明显响应" if no_response
        else f"输出饱和({sat_ratio*100:.0f}%)" if saturated
        else ""
    )
    return StepEvaluation(
        score=float(score),
        overshoot_pct=overshoot * 100.,
        rise_s=rise,
        settle_s=settle,
        tail_error=signed_tail,
        chatter=float(chatter),
        saturated=saturated,
        reversed_sign=reversed_sign,
        note=note,
        sat_ratio=sat_ratio,
        sample_count=int(len(times)),
        valid=is_valid,
        stalled=stalled,
    )


def identify_from_p_step(times, actual, base_y, goal_y, kp_probe, mode=0, style=1, base_cmd=None):
    """Estimate gains from a closed-loop P-only step (KI=0, KD=0, KP=kp_probe).

    Uses command delta (goal_y - base_cmd) for closed-loop DC gain estimation so
    steady-state droop at a non-zero baseline (e.g. r:100->150, y:80->120) yields
    alpha = 40/50 = 0.8 instead of 40/(150-80) = 0.57.
    """
    times = np.asarray(times, dtype=float)
    actual = np.asarray(actual, dtype=float)
    valid = np.isfinite(times) & np.isfinite(actual)
    times, actual = times[valid], actual[valid]
    cmd_base = float(base_y if base_cmd is None else base_cmd)
    cmd_amp = goal_y - cmd_base
    if len(times) < 8 or abs(cmd_amp) < 1e-6 or abs(kp_probe) < 1e-6:
        return []
    t = times - times[0]
    duration = float(t[-1])
    if duration < 0.2:
        return []

    tail = actual[t >= duration * 0.75]
    if not len(tail):
        tail = actual[-3:]
    dy_inf = float(np.mean(tail) - base_y)
    alpha = dy_inf / cmd_amp
    if alpha <= 0.03:
        return []  # No forward response or wrong sign
    alpha_clamped = min(max(alpha, 0.05), 0.94)
    sign = 1.0 if kp_probe >= 0 else -1.0
    kp_abs = abs(kp_probe)
    k_plant = alpha_clamped / ((1.0 - alpha_clamped) * kp_abs)

    y_span = dy_inf
    yn = (actual - base_y) / y_span
    peak = float(np.max(yn))
    mp = max(0., peak - 1.)
    peak_idx = int(np.argmax(yn))
    tp = float(t[peak_idx])

    _, _, _, _, _, _, tau_fac, delay_fac, zeta_target = STYLE_PROFILES[style]
    candidates = []

    # Underdamped second-order branch when P-only step clearly rings/overshoots.
    if mp >= 0.08 and tp > 0.03 and mode in (1, 2):
        ln_mp = math.log(min(mp, 0.85))
        zeta_cl = -ln_mp / math.sqrt(math.pi ** 2 + ln_mp ** 2)
        wn_cl = math.pi / (tp * math.sqrt(max(0.05, 1.0 - zeta_cl ** 2)))
        a1 = 2.0 * zeta_cl * wn_cl
        b0 = alpha_clamped * (wn_cl ** 2) / kp_abs
        a0 = (1.0 - alpha_clamped) * (wn_cl ** 2)
        if b0 > 1e-6:
            speed = (0.85, 1.25, 1.65)[style]
            wc = max(0.8, wn_cl * speed)
            if mode == 1:  # PID pole placement
                p3 = wc * 0.9
                kp = max(0.05 * kp_abs, (wc * wc + 2.0 * zeta_target * wc * p3 - a0) / b0)
                ki = max(0., (wc * wc * p3) / b0)
                kd = max(0., (2.0 * zeta_target * wc + p3 - a1) / b0)
                candidates.append({"KP": sign * kp, "KI": sign * ki, "KD": sign * kd})
                # Also add a gentler damped variant
                wc2 = wc * 0.72
                p3_2 = wc2 * 0.85
                kp2 = max(0.05 * kp_abs, (wc2 * wc2 + 2.0 * min(1.0, zeta_target + 0.08) * wc2 * p3_2 - a0) / b0)
                ki2 = max(0., (wc2 * wc2 * p3_2) / b0)
                kd2 = max(0., (2.0 * min(1.0, zeta_target + 0.08) * wc2 + p3_2 - a1) / b0)
                candidates.append({"KP": sign * kp2, "KI": sign * ki2, "KD": sign * kd2})
                return candidates
            else:  # PD
                kp = max(0.05 * kp_abs, (wc * wc - a0) / b0)
                kd = max(0., (2.0 * zeta_target * wc - a1) / b0)
                candidates.append({"KP": sign * kp, "KI": 0., "KD": sign * kd})
                return candidates

    # FOPDT + Skogestad SIMC branch (works great for 1st-order, speed loops & dead-time plants).
    def interp_time(level):
        hit = np.flatnonzero(yn >= level)
        if not len(hit):
            return None
        i = int(hit[0])
        if i == 0 or yn[i] == yn[i - 1]:
            return float(t[i])
        return float(t[i - 1] + (level - yn[i - 1]) * (t[i] - t[i - 1]) / (yn[i] - yn[i - 1]))

    t10, t63 = interp_time(0.10), interp_time(0.632)
    if t10 is None or t63 is None or t63 <= t10:
        tau_cl = max(0.05, duration * 0.25)
        delay = 0.02
    else:
        tau_cl = max(0.02, (t63 - t10) / 0.8946)
        delay = max(0.01, t63 - tau_cl)
    tau = max(0.03, tau_cl / (1.0 - alpha_clamped))
    # If the P-only step had noticeable overshoot in PI mode, treat extra phase lag as effective delay.
    if mp > 0.05 and mode == 0:
        delay = max(delay, tp * 0.28)

    for mult in (1.0, 1.55):
        tau_c = max(delay * delay_fac * mult, tau * tau_fac * mult, 0.04)
        kc = tau / (k_plant * (tau_c + delay))
        ti = min(tau, 4.0 * (tau_c + delay))
        ki = kc / max(ti, 0.02) if mode in (0, 1) else 0.
        kd = (kc * min(delay * 0.5, tau * 0.18)) if mode in (1, 2) else 0.
        if mode == 2:
            ki = 0.
        if mode == 0:
            kd = 0.
        candidates.append({"KP": sign * kc, "KI": sign * ki, "KD": sign * kd})
    return candidates


class AutoTuner:
    """Stateful round-by-round optimizer producing candidate {KP, KI, KD} dicts."""

    def __init__(self, mode, style, strategy, initial, rules, base, target,
                 step_seconds=2.0, max_rounds=12, max_ratio=None, output_limit=None):
        if mode not in range(len(MODES)):
            raise ValueError("未知整定模式")
        if style not in range(len(STYLES)):
            raise ValueError("未知响应风格")
        if strategy not in range(len(STRATEGIES)):
            raise ValueError("未知整定策略")
        if not all(math.isfinite(v) for v in (base, target, step_seconds)) or abs(target - base) < 1e-4:
            raise ValueError("目标值与起始基准必须存在有效差值")
        if step_seconds < 0.4 or max_rounds < 3:
            raise ValueError("单轮时长至少 0.4 s，最大轮数至少 3 轮")

        self.mode, self.style, self.strategy = mode, style, strategy
        self.active_keys = MODE_KEYS[mode]
        self.base, self.target = float(base), float(target)
        self.step_seconds = float(step_seconds)
        self.max_rounds = int(max_rounds)
        self.max_ratio = float(max_ratio if max_ratio is not None else (1.35 if strategy == 1 else 2.5))
        self.output_limit = float(output_limit) if (output_limit is not None and float(output_limit) > 0) else None
        self.initial = {k: float(initial.get(k, 0.)) for k in ("KP", "KI", "KD")}

        # Respect user's parameter limits and keep gain signs consistent (default >= 0).
        sign = -1.0 if self.initial["KP"] < -1e-6 else 1.0
        self.sign = sign
        self.bounds = {}
        for k in ("KP", "KI", "KD"):
            lo, hi, _ = rules[k]
            if sign > 0 and lo < 0 <= hi:
                lo = 0.
            elif sign < 0 and lo <= 0 < hi:
                hi = 0.
            # In local fine-tuning mode, also constrain total excursion of active keys relative to baseline.
            if strategy == 1 and k in self.active_keys and abs(self.initial[k]) >= 1e-3:
                v0 = self.initial[k]
                env = sorted((v0 * 0.4, v0 * 2.5))
                lo = max(lo, env[0])
                hi = min(hi, env[1])
                if hi < lo:
                    lo = hi = min(max(v0, float(rules[k][0])), float(rules[k][1]))
            self.bounds[k] = (float(lo), float(hi))

        self.round = 0
        self.history = []  # list of (round_idx, params_dict, StepEvaluation, tag)
        self.best_params = None
        self.best_eval = None
        self.baseline_eval = None
        self.finished = False
        self.finish_reason = ""

        self._queue = []
        self._tried = set()
        self._probe_params = None
        self._probe_boost_used = False
        self._coord_anchor = None
        self._coord_index = 0
        self._coord_dir = 0
        self._coord_improved = False
        self._step_scale = 0.16 if strategy == 1 else 0.28
        self._setup_initial_queue()

    def _cap_step_change(self, candidate, ref):
        """Limit per-round multiplicative ratio relative to ref (and initial in local mode)."""
        if ref is None:
            return dict(candidate)
        capped = dict(candidate)
        ratio = max(1.05, self.max_ratio)
        for k in self.active_keys:
            r = ref.get(k, 0.)
            v = capped.get(k, 0.)
            if abs(r) >= 1e-3:
                lo_r, hi_r = sorted((r / ratio, r * ratio))
                v = min(hi_r, max(lo_r, v))
            elif self.strategy == 1:
                kp_ref = max(abs(ref.get("KP", 0.)), 0.2)
                max_seed = kp_ref * (0.35 if k == "KI" else 0.08 if k == "KD" else 0.5)
                if abs(v) > max_seed:
                    v = math.copysign(max_seed, v)
            capped[k] = v
        return self._clamp_params(capped)

    def _clamp_params(self, raw):
        out = {}
        for k in ("KP", "KI", "KD"):
            val = float(raw.get(k, 0.)) if k in self.active_keys else 0.
            lo, hi = self.bounds[k]
            out[k] = round(min(hi, max(lo, val)), 4)
        return out

    def _signature(self, params):
        return (round(params["KP"], 4), round(params["KI"], 4), round(params["KD"], 4))

    def _enqueue(self, raw, tag):
        p = self._clamp_params(raw)
        sig = self._signature(p)
        if sig not in self._tried:
            self._queue.append((p, tag))
            return True
        return False

    def _setup_initial_queue(self):
        # Round 1 baseline preserves all original confirmed device gains verbatim
        # (only bounded by hard safety rules) rather than truncating inactive gains.
        raw_base = {
            k: round(min(self.bounds[k][1], max(self.bounds[k][0], float(self.initial[k]))), 4)
            for k in ("KP", "KI", "KD")
        }
        has_initial = abs(raw_base["KP"]) > 1e-4 or abs(raw_base["KI"]) > 1e-4
        if self.strategy == 1:
            if not has_initial:
                raise ValueError("局部微调模式要求先设定可基本稳定运行的非零初始 KP/KI 参数")
            self._queue.append((raw_base, "当前基准"))
            return

        # Strategy 0: evaluate initial gains first (if non-zero) so we never regress,
        # then run a clean P-only probe (unless initial was already P-only!).
        if has_initial:
            self._queue.append((raw_base, "初始基准"))
            is_p_only = abs(raw_base["KI"]) < 1e-6 and abs(raw_base["KD"]) < 1e-6 and abs(raw_base["KP"]) > 1e-4
            if is_p_only:
                self._probe_params = self._signature(raw_base)
            else:
                probe_kp = max(0.5, min(abs(raw_base["KP"]) * 0.65, 5.0)) * self.sign
                probe = self._clamp_params({"KP": probe_kp, "KI": 0., "KD": 0.})
                if self._enqueue(probe, "阶跃辨识"):
                    self._probe_params = self._signature(probe)
        else:
            probe = self._clamp_params({"KP": 1.0 * self.sign, "KI": 0., "KD": 0.})
            self._enqueue(probe, "阶跃辨识")
            self._probe_params = self._signature(probe)

    def _preferred_direction(self, key):
        """Use physical features of best_eval to try the more promising direction first."""
        ev = self.best_eval
        if ev is None:
            return 1
        os_limit = STYLE_PROFILES[self.style][4] * 100. + 1.5
        if key == "KP":
            return -1 if ev.overshoot_pct > os_limit + 2.0 else 1
        if key == "KI":
            if abs(ev.tail_error) > 0.025 * abs(self.target - self.base):
                return 1
            return -1 if ev.overshoot_pct > os_limit + 3.0 else 1
        if key == "KD":
            if ev.chatter > 0.45:
                return -1
            return 1 if ev.overshoot_pct > os_limit else -1
        return 1

    def _perturb(self, base_params, key, direction):
        val = base_params[key]
        scale = self._step_scale
        if abs(val) < 1e-3:
            if direction < 0:
                return None
            # Seed a small positive magnitude proportional to KP when KI or KD starts at 0.
            kp_mag = max(abs(base_params["KP"]), 0.5)
            seed = kp_mag * (0.35 if key == "KI" else 0.06 if key == "KD" else 0.5)
            candidate = dict(base_params)
            candidate[key] = self.sign * seed
            return self._clamp_params(candidate)
        delta = val * scale * direction
        if abs(delta) < 0.0002:
            delta = math.copysign(0.0002, delta)
        candidate = dict(base_params)
        candidate[key] = val + delta
        clamped = self._cap_step_change(candidate, base_params)
        return None if self._signature(clamped) == self._signature(base_params) else clamped

    def _advance_search(self):
        """Generate the next Hooke-Jeeves / coordinate search candidate."""
        while self._step_scale >= 0.04:
            if self._coord_anchor is None:
                self._coord_anchor = dict(self.best_params)
                self._coord_index = 0
                self._coord_dir = 0
                self._coord_improved = False

            while self._coord_index < len(self.active_keys):
                key = self.active_keys[self._coord_index]
                pref = self._preferred_direction(key)
                if self._coord_dir == 0:
                    self._coord_dir = 1
                    cand = self._perturb(self.best_params, key, pref)
                    if cand is not None and self._signature(cand) not in self._tried:
                        return cand, f"微调 {key}{'+' if pref > 0 else '-'}"
                if self._coord_dir == 1:
                    self._coord_dir = 2
                    cand = self._perturb(self.best_params, key, -pref)
                    if cand is not None and self._signature(cand) not in self._tried:
                        return cand, f"微调 {key}{'-' if pref > 0 else '+'}"
                self._coord_index += 1
                self._coord_dir = 0

            # Completed a full pass over all active keys.
            if self._coord_improved:
                # Hooke-Jeeves pattern extrapolation along the winning diagonal!
                pattern = {}
                moved = False
                for k in ("KP", "KI", "KD"):
                    diff = self.best_params[k] - self._coord_anchor[k]
                    pattern[k] = self.best_params[k] + 0.65 * diff
                    if abs(diff) > 1e-4:
                        moved = True
                self._coord_anchor = None
                if moved:
                    cand = self._cap_step_change(pattern, self.best_params)
                    if self._signature(cand) not in self._tried:
                        return cand, "加速外推"
            else:
                self._step_scale *= 0.52
                self._coord_anchor = None

        return None, ""

    def next_candidate(self):
        """Return (round_number, params_dict, tag) or None when tuning is complete."""
        if self.finished:
            return None
        if self.round >= self.max_rounds:
            self.finished = True
            self.finish_reason = f"已完成 {self.round} 轮寻优"
            return None

        while self._queue:
            params, tag = self._queue.pop(0)
            sig = self._signature(params)
            if sig not in self._tried:
                self._tried.add(sig)
                self.round += 1
                self._current = (params, tag)
                return self.round, dict(params), tag

        if self.best_params is None:
            self.finished = True
            self.finish_reason = "无有效候选参数"
            return None

        params, tag = self._advance_search()
        if params is None:
            self.finished = True
            self.finish_reason = f"参数已收敛（共 {self.round} 轮）"
            return None

        self._tried.add(self._signature(params))
        self.round += 1
        self._current = (params, tag)
        return self.round, dict(params), tag

    def record_result(self, times, actual, base_y=None, outputs=None, aborted_reason=""):
        """Feed back the step trajectory for the candidate returned by next_candidate()."""
        params, tag = self._current
        if aborted_reason:
            ev = StepEvaluation(1e6, 0., None, None, float("nan"), 0., note=aborted_reason, valid=False)
            self.history.append((self.round, dict(params), ev, tag))
            self._step_scale *= 0.5
            if self.best_params is None and not self._queue:
                self.finished = True
                self.finish_reason = aborted_reason
            return ev

        actual_arr = np.asarray(actual, dtype=float)
        fin_a = actual_arr[np.isfinite(actual_arr)]
        y0 = float(fin_a[0]) if base_y is None and len(fin_a) else float(self.base if base_y is None else base_y)
        ev = evaluate_step(times, actual, y0, self.target, outputs=outputs, style=self.style, output_limit=self.output_limit)
        self.history.append((self.round, dict(params), ev, tag))

        if ev.valid and self.baseline_eval is None and tag in ("初始基准", "当前基准"):
            self.baseline_eval = ev

        if ev.stalled:
            self._probe_params = None
            self._queue.clear()
            self.finished = True
            self.finish_reason = "输出触顶且反馈无响应（已停机，请检查轮子受阻、供电或编码器反馈）"
            return ev

        if ev.reversed_sign and self.round <= 2 and self.best_params is None:
            self._probe_params = None
            self._queue.clear()
            self.finished = True
            self.finish_reason = "检测到反向响应，请检查反馈通道或参数正负号"
            return ev

        # If this was the P-only identification probe, synthesize SIMC / pole-placement candidates!
        if self._probe_params is not None and self._signature(params) == self._probe_params:
            self._probe_params = None
            identified = (
                identify_from_p_step(
                    times, actual, y0, self.target, params["KP"],
                    self.mode, self.style, base_cmd=self.base,
                )
                if ev.valid
                else []
            )
            if not identified:
                can_boost = (
                    (ev.valid or (not ev.stalled and not ev.reversed_sign and ev.sample_count >= 6))
                    and not self._probe_boost_used
                    and not ev.saturated
                    and ev.sat_ratio < 0.30
                    and ev.tail_error * (self.target - y0) > 0
                )
                if can_boost:
                    self._probe_boost_used = True
                    boosted = self._clamp_params({"KP": params["KP"] * 3.0, "KI": 0., "KD": 0.})
                    if self._enqueue(boosted, "放大辨识"):
                        self._probe_params = self._signature(boosted)
                elif self.best_params is None and not self._queue:
                    self.finished = True
                    self.finish_reason = ev.note or "输出无明显响应，已停止继续放大 KP"
            for idx, cand in enumerate(identified):
                self._enqueue(cand, "模型整定" if idx == 0 else "稳健候选")

        if ev.valid and (self.best_eval is None or ev.score < self.best_eval.score - 1e-6):
            first_or_better = self.best_eval is not None and ev.score < self.best_eval.score * 0.995
            self.best_eval = ev
            self.best_params = dict(params)
            if first_or_better:
                self._coord_improved = True
                if self._coord_dir == 1:
                    self._coord_index += 1
                    self._coord_dir = 0

        return ev

    def summary_text(self):
        if not self.best_params or not self.best_eval:
            return self.finish_reason or "未获得有效整定结果"
        p = self.best_params
        ev = self.best_eval
        parts = [f"KP={p['KP']:.4g}, KI={p['KI']:.4g}, KD={p['KD']:.4g}"]
        parts.append(f"评分 J={ev.score:.3f}（超调 {ev.overshoot_pct:.1f}%")
        if ev.settle_s is not None:
            parts[-1] += f"，调节 {ev.settle_s:.2f} s）"
        else:
            parts[-1] += "）"
        if self.baseline_eval and self.baseline_eval.score < 1e5:
            parts.append(f"整定前 J={self.baseline_eval.score:.3f}")
        parts.append(f"触顶占比 {ev.sat_ratio * 100:.0f}%")
        parts.append(f"有效采样 {ev.sample_count}")
        return " · ".join(parts)
