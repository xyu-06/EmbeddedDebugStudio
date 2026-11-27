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
    passed_gates: bool = True
    gate_reasons: tuple = ()
    max_pitch_dev: float | None = None
    brake_settle_s: float | None = None
    brake_rev_peak: float | None = None
    brake_lurch: float | None = None
    brake_sat_ratio: float = 0.0
    still_climbing: bool = False


def _detect_output_saturation(outputs, output_limit=None, tail_err=0.0):
    """Return (saturated, sat_ratio) detecting sustained rail clamping."""
    if outputs is None:
        return False, 0.0
    u = np.asarray(outputs, dtype=float)
    u = u[np.isfinite(u)]
    if len(u) < 6:
        return False, 0.0
    if output_limit is not None and float(output_limit) > 0:
        sat_ratio = float(np.mean(np.abs(u) >= 0.95 * float(output_limit)))
        return bool(sat_ratio >= 0.25), sat_ratio
    u_max = float(np.max(np.abs(u)))
    if u_max <= 1e-3:
        return False, 0.0
    near = np.abs(u) >= 0.985 * u_max
    du = np.abs(np.diff(u))
    flat_pair = near[:-1] & near[1:] & (du <= 0.01 * u_max)
    flat_mask = np.zeros(len(u), dtype=bool)
    flat_mask[:-1] |= flat_pair
    flat_mask[1:] |= flat_pair
    flat_ratio = float(np.mean(flat_mask))
    near_rail = float(np.mean(near))
    if flat_ratio >= 0.55 or (near_rail > 0.82 and float(np.std(u[len(u) // 3:])) < 0.02 * u_max and tail_err > 0.03):
        return True, max(flat_ratio, near_rail) if near_rail > 0.82 else flat_ratio
    return False, 0.0


def is_step_still_climbing(times, actual, base_y, min_samples=8):
    """Return True if a step curve is still rising strongly near the end of the window."""
    times = np.asarray(times, dtype=float)
    actual = np.asarray(actual, dtype=float)
    valid_mask = np.isfinite(times) & np.isfinite(actual)
    times, actual = times[valid_mask], actual[valid_mask]
    if len(times) < min_samples:
        return False
    t = times - times[0]
    duration = float(t[-1])
    if duration < 0.15:
        return False
    end_slice = actual[t >= 0.85 * duration]
    if not len(end_slice):
        end_slice = actual[-2:]
    dy_end = float(np.mean(end_slice) - base_y)
    if abs(dy_end) < 1e-6:
        return False
    yn = (actual - base_y) / dy_end
    peak_idx = int(np.argmax(yn))
    if float(yn[peak_idx]) >= 1.08 and peak_idx < len(yn) - 2:
        return False
    q3_mask = (t >= 0.65 * duration) & (t <= 0.80 * duration)
    y_q3 = float(np.mean(actual[q3_mask])) if np.any(q3_mask) else float(actual[max(0, (3 * len(actual)) // 4 - 1)])
    q4_mask = t >= 0.88 * duration
    y_q4 = float(np.mean(actual[q4_mask])) if np.any(q4_mask) else float(np.mean(actual[-2:]))
    frac_delta = (y_q4 - y_q3) / dy_end
    tail_mask = t >= 0.72 * duration
    frac_slope = 0.0
    if np.sum(tail_mask) >= 3:
        tt = t[tail_mask]
        yy = actual[tail_mask]
        dt_t = tt - np.mean(tt)
        denom = float(np.sum(dt_t * dt_t))
        if denom > 1e-9:
            slope = float(np.sum(dt_t * (yy - np.mean(yy))) / denom)
            frac_slope = slope * (duration * 0.25) / dy_end
    return bool(frac_delta > 0.10 or frac_slope > 0.10)


def evaluate_step(
    times, actual, base_y, goal_y, outputs=None, style=1, output_limit=None,
    cmd_base=None, ref_duration=None, pitch=None, pitch_ref=0.0,
    max_pitch_dev_deg=None, max_sat_ratio=0.58, max_tail_err=None,
    max_base_offset_ratio=None,
):
    """Score a step trajectory with unified command normalization and hard safety gates."""
    times = np.asarray(times, dtype=float)
    actual = np.asarray(actual, dtype=float)
    valid_mask = np.isfinite(times) & np.isfinite(actual)
    if outputs is not None:
        raw_out = np.asarray(outputs, dtype=float)
        outputs_arr = raw_out[valid_mask] if len(raw_out) == len(valid_mask) else raw_out[np.isfinite(raw_out)]
    else:
        outputs_arr = None
    if pitch is not None:
        raw_pitch = np.asarray(pitch, dtype=float)
        pitch_arr = raw_pitch[valid_mask] if len(raw_pitch) == len(valid_mask) else raw_pitch[np.isfinite(raw_pitch)]
    else:
        pitch_arr = None
    times, actual = times[valid_mask], actual[valid_mask]

    norm_base = float(base_y if cmd_base is None else cmd_base)
    cmd_amp = float(goal_y - norm_base)
    scale = abs(cmd_amp)
    if len(times) < 6 or scale < 1e-6 or times[-1] - times[0] < 0.15:
        return StepEvaluation(
            1e6, 0., None, None, float("nan"), 0.,
            note="采样不足", sample_count=int(len(times)), valid=False,
            passed_gates=False, gate_reasons=("采样不足",),
        )

    t = times - times[0]
    duration = float(t[-1])
    if ref_duration is not None and float(ref_duration) > 0:
        ref_dur = float(ref_duration)
        if duration < 0.75 * ref_dur:
            return StepEvaluation(
                1e6, 0., None, None, float("nan"), 0.,
                note="观测时长不足", sample_count=int(len(times)), valid=False,
                passed_gates=False, gate_reasons=("观测时长不足",),
            )
        if duration > ref_dur * 1.02:
            keep = t <= ref_dur + 1e-6
            if np.sum(keep) >= 6:
                t = t[keep]
                times = times[keep]
                actual = actual[keep]
                if outputs_arr is not None and len(outputs_arr) == len(keep):
                    outputs_arr = outputs_arr[keep]
                if pitch_arr is not None and len(pitch_arr) == len(keep):
                    pitch_arr = pitch_arr[keep]
                duration = float(t[-1])
        dur_norm = ref_dur
    else:
        dur_norm = duration

    z = (actual - norm_base) / cmd_amp
    local_amp = float(goal_y - base_y)
    z_rise = (actual - base_y) / local_amp if abs(local_amp) > 1e-6 else z
    err_norm = np.abs(goal_y - actual) / scale
    dt = np.diff(t)

    niae = float(np.sum(dt * (err_norm[:-1] + err_norm[1:]) * 0.5) / dur_norm)
    weighted = t * err_norm
    nitae = float(2.0 * np.sum(dt * (weighted[:-1] + weighted[1:]) * 0.5) / (dur_norm * dur_norm))

    tail_mask = t >= duration * 0.78
    if not np.any(tail_mask):
        tail_mask = slice(-max(2, len(t) // 5), None)
    tail_err = float(np.mean(err_norm[tail_mask]))
    signed_tail = float(np.mean(goal_y - actual[tail_mask]))

    overshoot = max(0., float(np.max(z) - 1.))
    sign_cmd = 1.0 if cmd_amp >= 0 else -1.0
    if cmd_base is None or abs(base_y - norm_base) <= 0.15 * scale:
        undershoot = max(0., float(np.max((norm_base - actual) * sign_cmd / scale)))
    else:
        undershoot = max(0., float(np.max((base_y - actual) * sign_cmd / scale)))
    reversed_sign = float(np.mean(z[tail_mask])) < -0.08

    def crossing(level):
        hit = np.flatnonzero(z_rise >= level)
        if not len(hit):
            return None
        i = int(hit[0])
        if i == 0 or z_rise[i] == z_rise[i - 1]:
            return float(t[i])
        return float(t[i - 1] + (level - z_rise[i - 1]) * (t[i] - t[i - 1]) / (z_rise[i] - z_rise[i - 1]))

    low, high = crossing(0.1), crossing(0.9)
    rise = (high - low) if (low is not None and high is not None and high >= low) else None

    outside = np.flatnonzero(err_norm > 0.02)
    last_out = int(outside[-1]) + 1 if len(outside) else 0
    settle = float(t[last_out]) if last_out < len(t) and duration - t[last_out] >= min(0.35, duration * 0.2) else None

    # Measure high-frequency ripple in the latter half + control output chatter if available.
    late = actual[t >= duration * 0.5]
    y_ripple = float(np.sum(np.abs(np.diff(late))) / scale) if len(late) >= 3 else 0.
    # A clean monotonic curve can change by at most ~1*scale in the second half; excess is oscillation/noise.
    y_chatter = max(0., y_ripple - 0.35) / max(1., len(late) / 25.)

    u_chatter = 0.
    if outputs_arr is not None:
        u = outputs_arr[np.isfinite(outputs_arr)]
        if len(u) >= 6:
            u_span = max(float(np.ptp(u)), abs(float(np.mean(u))), 1.0)
            du = np.abs(np.diff(u))
            # Allow one initial kick + smooth decay (~2*u_span total variation); penalize excess buzz.
            excess_tv = max(0., float(np.sum(du)) / u_span - 2.5)
            u_chatter = excess_tv / max(1., len(u) / 25.)
    saturated, sat_ratio = _detect_output_saturation(outputs_arr, output_limit=output_limit, tail_err=tail_err)

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
    climbing = is_step_still_climbing(times, actual, base_y)

    max_pitch_dev = None
    if pitch_arr is not None:
        p_fin = pitch_arr[np.isfinite(pitch_arr)]
        if len(p_fin):
            max_pitch_dev = float(np.max(np.abs(p_fin - float(pitch_ref))))

    gate_reasons = []
    if saturated or (max_sat_ratio is not None and sat_ratio >= float(max_sat_ratio)):
        gate_reasons.append(f"持续饱和({sat_ratio * 100:.0f}%)")
    if max_pitch_dev_deg is not None and max_pitch_dev is not None and max_pitch_dev > float(max_pitch_dev_deg):
        gate_reasons.append(f"倾角超限({max_pitch_dev:.1f}°)")
    if max_tail_err is not None and tail_err > float(max_tail_err):
        gate_reasons.append(f"尾差过大({tail_err * 100:.0f}%)")
    if max_base_offset_ratio is not None and cmd_base is not None:
        if abs(float(base_y) - norm_base) / scale > float(max_base_offset_ratio):
            gate_reasons.append("起点偏差过大")
    passed_gates = bool(is_valid and not gate_reasons)

    note = (
        "反向响应" if reversed_sign
        else "输出触顶且反馈无响应" if stalled
        else "反馈无明显响应" if no_response
        else " / ".join(gate_reasons) if gate_reasons
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
        passed_gates=passed_gates,
        gate_reasons=tuple(gate_reasons),
        max_pitch_dev=max_pitch_dev,
        still_climbing=climbing,
    )


def evaluate_step_and_brake(
    start_times, start_actual, brake_times, brake_actual,
    base_cmd, target_cmd, start_outputs=None, brake_outputs=None,
    start_pitch=None, brake_pitch=None, pitch_ref=0.0,
    style=1, output_limit=None, ref_duration=None, brake_duration=None,
    settle_band=5.0, settle_hold_s=0.20, max_pitch_dev_deg=12.0,
    max_sat_ratio=0.35, start_base_y=None,
):
    """Jointly evaluate start step (base -> target) and brake recovery (target -> base)."""
    s_act = np.asarray(start_actual, dtype=float)
    s_fin = s_act[np.isfinite(s_act)]
    start_y0 = float(s_fin[0]) if (start_base_y is None and len(s_fin)) else float(base_cmd if start_base_y is None else start_base_y)
    start_scale = abs(float(target_cmd) - float(base_cmd))
    start_max_tail_err = min(0.25, float(settle_band) / start_scale) if start_scale > 1e-6 else 0.25
    start_ev = evaluate_step(
        start_times, start_actual, start_y0, target_cmd,
        outputs=start_outputs, style=style, output_limit=output_limit,
        cmd_base=base_cmd, ref_duration=ref_duration,
        pitch=start_pitch, pitch_ref=pitch_ref,
        max_pitch_dev_deg=max_pitch_dev_deg, max_sat_ratio=max_sat_ratio,
        max_tail_err=start_max_tail_err,
    )

    bt = np.asarray(brake_times, dtype=float)
    by = np.asarray(brake_actual, dtype=float)
    b_valid = np.isfinite(bt) & np.isfinite(by)
    if brake_outputs is not None:
        raw_bu = np.asarray(brake_outputs, dtype=float)
        bu = raw_bu[b_valid] if len(raw_bu) == len(b_valid) else raw_bu[np.isfinite(raw_bu)]
    else:
        bu = None
    if brake_pitch is not None:
        raw_bp = np.asarray(brake_pitch, dtype=float)
        bp = raw_bp[b_valid] if len(raw_bp) == len(b_valid) else raw_bp[np.isfinite(raw_bp)]
    else:
        bp = None
    bt, by = bt[b_valid], by[b_valid]
    amp = float(target_cmd - base_cmd)
    scale = abs(amp)
    if len(bt) < 6 or scale < 1e-6 or bt[-1] - bt[0] < 0.4:
        return StepEvaluation(
            1e6, start_ev.overshoot_pct, start_ev.rise_s, None, float("nan"), start_ev.chatter,
            note="刹停采样不足", sample_count=start_ev.sample_count + int(len(bt)),
            valid=False, passed_gates=False, gate_reasons=("刹停采样不足",),
            max_pitch_dev=start_ev.max_pitch_dev,
        )

    brk_t = bt - bt[0]
    brk_dur = float(brk_t[-1])
    if brake_duration is not None and float(brake_duration) > 0:
        brk_lim = float(brake_duration)
        if brk_dur < 0.70 * brk_lim:
            return StepEvaluation(
                1e6, start_ev.overshoot_pct, start_ev.rise_s, None, float("nan"), start_ev.chatter,
                note="刹停观测时长不足", sample_count=start_ev.sample_count + int(len(brk_t)),
                valid=False, passed_gates=False, gate_reasons=("刹停观测时长不足",),
                max_pitch_dev=start_ev.max_pitch_dev,
            )
        if brk_dur > brk_lim * 1.02:
            keep = brk_t <= brk_lim + 1e-6
            if np.sum(keep) >= 6:
                brk_t = brk_t[keep]
                by = by[keep]
                if bu is not None and len(bu) == len(keep):
                    bu = bu[keep]
                if bp is not None and len(bp) == len(keep):
                    bp = bp[keep]
        brk_norm_dur = brk_lim
    else:
        brk_norm_dur = brk_dur

    direction = 1.0 if amp >= 0 else -1.0
    if len(s_fin):
        pre = s_fin[-min(3, len(s_fin)):]
        v0 = float(np.mean((pre - float(base_cmd)) * direction))
    else:
        v0 = float((by[0] - float(base_cmd)) * direction)

    signed_brk = (by - float(base_cmd)) * direction
    lurch_win = signed_brk[:max(2, len(signed_brk) // 2)]
    lurch = max(0.0, float(np.max(lurch_win) - v0))
    rev_peak = float(np.min(signed_brk))

    brake_settle_s = None
    band = max(1e-3, float(settle_band))
    for i in range(len(brk_t)):
        if abs(by[i] - float(base_cmd)) > band:
            continue
        ok = True
        held = False
        for j in range(i, len(brk_t)):
            if abs(by[j] - float(base_cmd)) > band:
                ok = False
                break
            if brk_t[j] - brk_t[i] >= float(settle_hold_s) - 0.005:
                held = True
                if any(abs(by[k] - float(base_cmd)) > band * 1.15 for k in range(j + 1, len(brk_t))):
                    ok = False
                break
        if ok and held:
            brake_settle_s = float(brk_t[i])
            break

    brk_tail_mask = brk_t >= float(brk_t[-1]) * 0.78
    if not np.any(brk_tail_mask):
        brk_tail_mask = slice(-max(2, len(brk_t) // 5), None)
    brake_tail_err = float(np.mean(np.abs(by[brk_tail_mask] - float(base_cmd))) / scale)
    brk_sat, brk_sat_ratio = _detect_output_saturation(bu, output_limit=output_limit, tail_err=brake_tail_err)

    brk_pitch_dev = None
    if bp is not None:
        bp_fin = bp[np.isfinite(bp)]
        if len(bp_fin):
            brk_pitch_dev = float(np.max(np.abs(bp_fin - float(pitch_ref))))
    pitch_vals = [v for v in (start_ev.max_pitch_dev, brk_pitch_dev) if v is not None]
    max_pitch_dev = max(pitch_vals) if pitch_vals else None
    sat_ratio = max(start_ev.sat_ratio, brk_sat_ratio)
    saturated = bool(start_ev.saturated or brk_sat)

    gate_reasons = list(start_ev.gate_reasons)
    if (saturated or (max_sat_ratio is not None and sat_ratio >= float(max_sat_ratio))) and not any("持续饱和" in r for r in gate_reasons):
        gate_reasons.append(f"持续饱和({sat_ratio * 100:.0f}%)")
    if max_pitch_dev_deg is not None and max_pitch_dev is not None and max_pitch_dev > float(max_pitch_dev_deg) and not any("倾角超限" in r for r in gate_reasons):
        gate_reasons.append(f"倾角超限({max_pitch_dev:.1f}°)")
    if brake_settle_s is None:
        gate_reasons.append("刹停未回稳")
    if max(0.0, -rev_peak) > 0.65 * scale:
        gate_reasons.append(f"反冲过大({rev_peak:.1f})")
    if brake_tail_err > 0.20:
        gate_reasons.append("刹停尾差过大")

    is_valid = bool(start_ev.valid)
    passed_gates = bool(is_valid and not gate_reasons)
    brake_settle_eff = brake_settle_s if brake_settle_s is not None else brk_norm_dur
    pitch_term = (0.25 * (max_pitch_dev / float(max_pitch_dev_deg))) if (max_pitch_dev is not None and max_pitch_dev_deg) else 0.0
    brake_score = (
        1.6 * (brake_settle_eff / brk_norm_dur)
        + 1.8 * max(0.0, -rev_peak) / scale
        + 0.6 * max(0.0, lurch) / scale
        + 2.0 * brake_tail_err
        + pitch_term
    )
    total_score = (0.45 * start_ev.score + 0.55 * brake_score) if is_valid else float(start_ev.score)
    note = start_ev.note if not is_valid else (" / ".join(gate_reasons) if gate_reasons else "")
    return StepEvaluation(
        score=float(total_score),
        overshoot_pct=start_ev.overshoot_pct,
        rise_s=start_ev.rise_s,
        settle_s=brake_settle_s,
        tail_error=float(np.mean(float(base_cmd) - by[brk_tail_mask])),
        chatter=start_ev.chatter,
        saturated=saturated,
        reversed_sign=start_ev.reversed_sign,
        note=note,
        sat_ratio=sat_ratio,
        sample_count=start_ev.sample_count + int(len(brk_t)),
        valid=is_valid,
        stalled=start_ev.stalled,
        passed_gates=passed_gates,
        gate_reasons=tuple(gate_reasons),
        max_pitch_dev=max_pitch_dev,
        brake_settle_s=brake_settle_s,
        brake_rev_peak=rev_peak,
        brake_lurch=lurch,
        brake_sat_ratio=brk_sat_ratio,
        still_climbing=start_ev.still_climbing,
    )


def identify_from_p_step(
    times, actual, base_y, goal_y, kp_probe, mode=0, style=1,
    base_cmd=None, outputs=None, output_limit=None,
):
    """Estimate gains from a closed-loop P-only step (KI=0, KD=0, KP=kp_probe).

    Uses command delta (goal_y - base_cmd) for closed-loop DC gain estimation so
    steady-state droop at a non-zero baseline (e.g. r:100->150, y:80->120) yields
    alpha = 40/50 = 0.8 instead of 40/(150-80) = 0.57.
    """
    times = np.asarray(times, dtype=float)
    actual = np.asarray(actual, dtype=float)
    valid = np.isfinite(times) & np.isfinite(actual)
    if outputs is not None:
        raw_out = np.asarray(outputs, dtype=float)
        outputs_arr = raw_out[valid] if len(raw_out) == len(valid) else raw_out[np.isfinite(raw_out)]
    else:
        outputs_arr = None
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
    tail_err_probe = abs(goal_y - float(np.mean(tail))) / abs(cmd_amp)
    sat_flag, sat_ratio = _detect_output_saturation(outputs_arr, output_limit=output_limit, tail_err=tail_err_probe)
    if sat_flag or sat_ratio >= 0.25:
        return []
    if is_step_still_climbing(times, actual, base_y):
        return []

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
        root_d = math.sqrt(max(1e-4, 1.0 - zeta_cl ** 2))
        wd = wn_cl * root_d
        phi = math.acos(min(0.999, max(-0.999, zeta_cl)))
        y_hat_2 = dy_inf * (1.0 - (np.exp(-zeta_cl * wn_cl * t) / root_d) * np.sin(wd * t + phi))
        nrmse_2 = float(np.sqrt(np.mean((y_hat_2 - (actual - base_y)) ** 2))) / max(abs(dy_inf), 1e-6)
        if nrmse_2 > 0.28:
            return []
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
    y_hat = dy_inf * (1.0 - np.exp(-np.maximum(0.0, t - delay) / tau_cl))
    nrmse = float(np.sqrt(np.mean((y_hat - (actual - base_y)) ** 2))) / max(abs(dy_inf), 1e-6)
    if nrmse > 0.22:
        return []
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

    def __init__(
        self, mode, style, strategy, initial, rules, base, target,
        step_seconds=2.0, max_rounds=12, max_ratio=None, output_limit=None,
        eval_brake=False, brake_seconds=None, reverify_winner=False,
        max_pitch_dev_deg=12.0, settle_band=5.0, max_sat_ratio=None,
    ):
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
        self.max_ratio = float(max_ratio if max_ratio is not None else (1.35 if strategy == 1 else 4.0))
        self.output_limit = float(output_limit) if (output_limit is not None and float(output_limit) > 0) else None
        self.eval_brake = bool(eval_brake)
        self.brake_seconds = float(brake_seconds) if (brake_seconds is not None and float(brake_seconds) > 0) else max(2.5, self.step_seconds * 1.4)
        self.reverify_winner = bool(reverify_winner)
        self.max_pitch_dev_deg = float(max_pitch_dev_deg) if (max_pitch_dev_deg is not None and float(max_pitch_dev_deg) > 0) else None
        self.settle_band = float(settle_band)
        self.max_sat_ratio = float(max_sat_ratio) if max_sat_ratio is not None else (0.35 if self.eval_brake else 0.55)
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
            if strategy == 1 and k in self.active_keys and abs(self.initial[k]) >= 1e-4:
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
        self.baseline_params = None
        self.finished = False
        self.finish_reason = ""
        self.winner_verified = False
        self.kept_baseline = False
        self.reverify_inconclusive = False
        self.candidate_ref = None

        self._queue = []
        self._reverify_queue = []
        self._tried = set()
        self._has_initial = False
        self._probe_params = None
        self._probe_boost_used = False
        self._model_targets = []
        self._reverify_stage = 0
        self._reverify_cand_eval = None
        self._reverify_base_eval = None
        self._reverify_base_retried = False
        self._pre_reverify_best = None
        self._coord_anchor = None
        self._coord_index = 0
        self._coord_dir = 0
        self._coord_improved = False
        self._step_scale = 0.16 if strategy == 1 else 0.28
        self._setup_initial_queue()

    def _cap_step_change(self, candidate, ref):
        """Limit per-round multiplicative ratio relative to ref."""
        if ref is None:
            return dict(candidate)
        capped = dict(candidate)
        ratio = max(1.05, self.max_ratio)
        for k in self.active_keys:
            r = ref.get(k, 0.)
            v = capped.get(k, 0.)
            if abs(r) >= 1e-4:
                lo_r, hi_r = sorted((r / ratio, r * ratio))
                v = min(hi_r, max(lo_r, v))
            elif self.strategy == 1 or self._has_initial:
                kp_ref = max(abs(ref.get("KP", 0.)), 0.02)
                if self.strategy == 1:
                    max_seed = kp_ref * (0.35 if k == "KI" else 0.08 if k == "KD" else 0.5)
                else:
                    max_seed = kp_ref * (1.2 if k == "KI" else 0.35 if k == "KD" else 0.5)
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
        if sig not in self._tried and all(self._signature(qp) != sig for qp, _ in self._queue):
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
        self._has_initial = has_initial
        self.baseline_params = dict(raw_base)
        if self.strategy == 1:
            if not has_initial:
                raise ValueError("局部微调模式要求先设定可基本稳定运行的非零初始 KP/KI 参数")
            self._queue.append((raw_base, "当前基准"))
            return

        # Strategy 0: evaluate initial gains first (if non-zero) so we never regress,
        # then run a bounded P-only probe (unless initial was already P-only!).
        if has_initial:
            self._queue.append((raw_base, "初始基准"))
            is_p_only = abs(raw_base["KI"]) < 1e-6 and abs(raw_base["KD"]) < 1e-6 and abs(raw_base["KP"]) > 1e-4
            if is_p_only:
                self._probe_params = self._signature(raw_base)
            else:
                raw_kp = abs(raw_base["KP"])
                if raw_kp > 1e-4:
                    probe_kp = min(raw_kp * 0.65, 5.0) * self.sign
                else:
                    probe_kp = min(max(abs(raw_base["KI"]) * 0.25, 0.02), 1.0) * self.sign
                probe_cand = {"KP": probe_kp, "KI": 0., "KD": 0.}
                if raw_kp > 1e-4:
                    probe_cand["KP"] = self._cap_step_change(probe_cand, raw_base)["KP"]
                probe = self._clamp_params(probe_cand)
                if self._enqueue(probe, "阶跃辨识"):
                    self._probe_params = self._signature(probe)
        else:
            probe = self._clamp_params({"KP": 1.0 * self.sign, "KI": 0., "KD": 0.})
            self._enqueue(probe, "阶跃辨识")
            self._probe_params = self._signature(probe)

    @staticmethod
    def _is_better(new_ev, old_ev):
        if new_ev is None or not new_ev.valid:
            return False
        if old_ev is None or not old_ev.valid:
            return True
        if new_ev.passed_gates != old_ev.passed_gates:
            return bool(new_ev.passed_gates)
        return new_ev.score < old_ev.score - 1e-6

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
        if abs(val) < 1e-4:
            if direction < 0:
                return None
            # Seed a small positive magnitude proportional to KP when KI or KD starts at 0.
            kp_mag = max(abs(base_params["KP"]), 0.02)
            seed = kp_mag * (0.35 if key == "KI" else 0.06 if key == "KD" else 0.5)
            candidate = dict(base_params)
            candidate[key] = self.sign * seed
            return self._cap_step_change(candidate, base_params)
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

    def _maybe_start_reverify(self):
        if (
            self.reverify_winner
            and self._reverify_stage == 0
            and self.best_params is not None
            and self.baseline_params is not None
            and self._signature(self.best_params) != self._signature(self.baseline_params)
            and self.best_eval is not None
            and self.best_eval.valid
            and self.best_eval.passed_gates
        ):
            self._reverify_stage = 1
            self._pre_reverify_best = (dict(self.best_params), self.best_eval)
            self.round += 1
            self._current = (dict(self.best_params), "复测候选")
            return self.round, dict(self.best_params), "复测候选"
        return None

    def next_candidate(self):
        """Return (round_number, params_dict, tag) or None when tuning is complete."""
        if self.finished:
            return None
        if self._reverify_queue:
            params, tag = self._reverify_queue.pop(0)
            self.round += 1
            self._current = (dict(params), tag)
            return self.round, dict(params), tag
        if self.round >= self.max_rounds:
            rev = self._maybe_start_reverify()
            if rev is not None:
                return rev
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
            rev = self._maybe_start_reverify()
            if rev is not None:
                return rev
            self.finished = True
            self.finish_reason = f"参数已收敛（共 {self.round} 轮）"
            return None

        self._tried.add(self._signature(params))
        self.round += 1
        self._current = (params, tag)
        return self.round, dict(params), tag

    def record_result(
        self, times, actual, base_y=None, outputs=None, aborted_reason="",
        brake_times=None, brake_actual=None, brake_outputs=None,
        start_pitch=None, brake_pitch=None, pitch_ref=0.0,
    ):
        """Feed back the step trajectory for the candidate returned by next_candidate()."""
        params, tag = self._current
        if aborted_reason:
            ev = StepEvaluation(
                1e6, 0., None, None, float("nan"), 0.,
                note=aborted_reason, valid=False, passed_gates=False, gate_reasons=(aborted_reason,),
            )
            self.history.append((self.round, dict(params), ev, tag))
            if self._reverify_stage in (1, 2):
                orig_p, orig_ev = self._pre_reverify_best
                self.best_params = dict(self.baseline_params)
                self.best_eval = self.baseline_eval or orig_ev
                self._reverify_stage = 3
                self.reverify_inconclusive = True
                self.candidate_ref = dict(orig_p)
                self.kept_baseline = True
                self.finished = True
                self.finish_reason = f"复测中止（{aborted_reason}），已保留原参数"
                return ev
            self._step_scale *= 0.5
            if self.best_params is None and not self._queue:
                self.finished = True
                self.finish_reason = aborted_reason
            return ev

        actual_arr = np.asarray(actual, dtype=float)
        fin_a = actual_arr[np.isfinite(actual_arr)]
        y0 = float(fin_a[0]) if base_y is None and len(fin_a) else float(self.base if base_y is None else base_y)
        if self.eval_brake and brake_times is not None and brake_actual is not None:
            ev = evaluate_step_and_brake(
                times, actual, brake_times, brake_actual,
                self.base, self.target,
                start_outputs=outputs, brake_outputs=brake_outputs,
                start_pitch=start_pitch, brake_pitch=brake_pitch, pitch_ref=pitch_ref,
                style=self.style, output_limit=self.output_limit,
                ref_duration=self.step_seconds, brake_duration=self.brake_seconds,
                settle_band=self.settle_band, max_pitch_dev_deg=self.max_pitch_dev_deg,
                max_sat_ratio=self.max_sat_ratio, start_base_y=y0,
            )
        else:
            ev = evaluate_step(
                times, actual, y0, self.target,
                outputs=outputs, style=self.style, output_limit=self.output_limit,
                cmd_base=self.base, ref_duration=self.step_seconds,
                pitch=start_pitch, pitch_ref=pitch_ref,
                max_pitch_dev_deg=(self.max_pitch_dev_deg if start_pitch is not None else None),
                max_sat_ratio=self.max_sat_ratio,
                max_tail_err=0.08,
            )
        self.history.append((self.round, dict(params), ev, tag))

        if self._reverify_stage == 1:
            self._reverify_cand_eval = ev
            self._reverify_stage = 2
            self._reverify_queue.append((dict(self.baseline_params), "复测原参"))
            return ev

        if self._reverify_stage == 2:
            self._reverify_base_eval = ev
            orig_p, orig_ev = self._pre_reverify_best
            re_cand = self._reverify_cand_eval
            re_base = ev
            base1 = self.baseline_eval
            if (not re_cand.valid) or (not re_cand.passed_gates):
                self._reverify_stage = 3
                keep_cand = False
                reason = "候选参数复测未通过门槛，已保留原参数"
            elif base1 is not None and base1.valid and base1.passed_gates:
                if re_base.valid and re_base.passed_gates:
                    self._reverify_stage = 3
                    cand_mean = 0.5 * (orig_ev.score + re_cand.score)
                    base_mean = 0.5 * (base1.score + re_base.score)
                    noise = max(abs(orig_ev.score - re_cand.score), abs(base1.score - re_base.score))
                    margin = max(0.03 * abs(base_mean), 0.5 * noise)
                    if cand_mean < base_mean - margin:
                        keep_cand = True
                        reason = "优胜参数复测通过"
                    else:
                        keep_cand = False
                        reason = "复测差距在重复波动范围内，已保留原参数"
                elif not self._reverify_base_retried:
                    self._reverify_base_retried = True
                    self._reverify_queue.append((dict(self.baseline_params), "补测原参"))
                    return ev
                else:
                    self._reverify_stage = 3
                    self.reverify_inconclusive = True
                    self.winner_verified = False
                    self.kept_baseline = True
                    self.candidate_ref = dict(orig_p)
                    self.best_params = dict(self.baseline_params)
                    self.best_eval = base1
                    self.finished = True
                    self.finish_reason = "候选通过，对照不足（原参数复测未通过，已暂缓自动写回并保留原参数）"
                    return ev
            else:
                self._reverify_stage = 3
                keep_cand = True
                reason = "优胜参数复测通过"

            if keep_cand:
                self.best_params = dict(orig_p)
                self.best_eval = re_cand if re_cand.score > orig_ev.score else orig_ev
                self.winner_verified = True
                self.kept_baseline = False
            else:
                self.best_params = dict(self.baseline_params)
                self.best_eval = re_base if (re_base.valid and re_base.passed_gates) else (base1 or re_base)
                self.winner_verified = False
                self.kept_baseline = True
            self.finished = True
            self.finish_reason = reason
            return ev

        if ev.valid and self.baseline_eval is None and tag in ("初始基准", "当前基准"):
            self.baseline_eval = ev
            self.baseline_params = dict(params)

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

        # If this was the P-only identification probe, synthesize bounded SIMC / pole-placement candidates!
        if self._probe_params is not None and self._signature(params) == self._probe_params:
            self._probe_params = None
            can_identify = ev.valid and (not ev.saturated) and (ev.sat_ratio < 0.25) and (not ev.still_climbing)
            identified = (
                identify_from_p_step(
                    times, actual, y0, self.target, params["KP"],
                    self.mode, self.style, base_cmd=self.base,
                    outputs=outputs, output_limit=self.output_limit,
                )
                if can_identify
                else []
            )
            if not identified:
                can_boost = (
                    (ev.valid or (not ev.stalled and not ev.reversed_sign and ev.sample_count >= 6))
                    and not self._probe_boost_used
                    and not ev.saturated
                    and ev.sat_ratio < 0.25
                    and not ev.still_climbing
                    and ev.tail_error * (self.target - y0) > 0
                )
                if can_boost:
                    self._probe_boost_used = True
                    boost_mult = min(3.0, self.max_ratio) if self._has_initial else 3.0
                    boosted_raw = {"KP": params["KP"] * boost_mult, "KI": 0., "KD": 0.}
                    if self._has_initial and self.baseline_params is not None:
                        boosted = self._cap_step_change(boosted_raw, self.baseline_params)
                    else:
                        boosted = self._clamp_params(boosted_raw)
                    if self._enqueue(boosted, "放大辨识"):
                        self._probe_params = self._signature(boosted)
                elif self.best_params is None and not self._queue:
                    self.finished = True
                    self.finish_reason = ev.note or "输出无明显响应，已停止继续放大 KP"
            if identified:
                if self._has_initial:
                    self._model_targets = [self._clamp_params(c) for c in identified]
                    ref_p = self.best_params or self.baseline_params
                    step0 = self._cap_step_change(self._model_targets[0], ref_p)
                    self._enqueue(step0, "模型整定")
                else:
                    for idx, cand in enumerate(identified):
                        self._enqueue(cand, "模型整定" if idx == 0 else "稳健候选")

        prev_best_eval = self.best_eval
        improved = self._is_better(ev, prev_best_eval)
        if tag == "模型整定" and self._model_targets:
            target_p = self._model_targets[0]
            if improved and self._signature(params) != self._signature(target_p):
                nxt_step = self._cap_step_change(target_p, params)
                if self._signature(nxt_step) != self._signature(params):
                    self._enqueue(nxt_step, "模型整定")
            elif not improved:
                self._model_targets = []

        if improved:
            first_or_better = (
                prev_best_eval is not None
                and prev_best_eval.valid
                and (
                    (ev.passed_gates and not prev_best_eval.passed_gates)
                    or ev.score < prev_best_eval.score * 0.995
                )
            )
            self.best_eval = ev
            self.best_params = dict(params)
            if first_or_better:
                self._coord_improved = True
                if self._coord_dir == 1:
                    self._coord_index += 1
                    self._coord_dir = 0

        return ev

    def has_qualified_winner(self):
        """Return True only if best_params is valid, conclusive, and passes required safety/acceptance gates."""
        if self.reverify_inconclusive:
            return False
        if self.best_params is None or self.best_eval is None or not self.best_eval.valid:
            return False
        strict_gates = bool(
            self.eval_brake
            or self.output_limit is not None
            or self.best_eval.max_pitch_dev is not None
        )
        if strict_gates and not self.best_eval.passed_gates:
            return False
        return True

    def summary_text(self):
        if not self.best_params or not self.best_eval:
            return self.finish_reason or "未获得有效整定结果"
        p = self.best_params
        ev = self.best_eval
        parts = [f"KP={p['KP']:.4g}, KI={p['KI']:.4g}, KD={p['KD']:.4g}"]
        if self.eval_brake:
            bset = f"{ev.brake_settle_s:.2f} s" if ev.brake_settle_s is not None else "未稳"
            rev = f"{ev.brake_rev_peak:+.1f}" if ev.brake_rev_peak is not None else "--"
            parts.append(f"评分 J={ev.score:.3f}（超调 {ev.overshoot_pct:.1f}%，刹停 {bset}，反冲 {rev}）")
        else:
            parts.append(f"评分 J={ev.score:.3f}（超调 {ev.overshoot_pct:.1f}%")
            if ev.settle_s is not None:
                parts[-1] += f"，调节 {ev.settle_s:.2f} s）"
            else:
                parts[-1] += "）"
        if self.baseline_eval and self.baseline_eval.score < 1e5:
            parts.append(f"整定前 J={self.baseline_eval.score:.3f}")
        parts.append(f"触顶占比 {ev.sat_ratio * 100:.0f}%")
        if self.winner_verified:
            parts.append("复测通过")
        elif self.reverify_inconclusive:
            parts.append("对照不足(已保留原参)")
        elif self.kept_baseline:
            parts.append("复测保留原参")
        parts.append(f"有效采样 {ev.sample_count}")
        return " · ".join(parts)
