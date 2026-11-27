"""Canonical channel semantics, roles, and engineering units shared across workbenches and codegen."""

CANONICAL_CHANNELS: dict[str, tuple[str, str, str]] = {
    # key: (default_unit, role, zh_label)
    "pitch_target": ("°", "target", "目标倾角"),
    "pitch": ("°", "actual", "实际倾角"),
    "gyro_pitch": ("°/s", "other", "俯仰角速度"),
    "pwm": ("PWM", "output", "电机输出"),
    "wheel_speed": ("RPM", "other", "轮速"),
    "roll": ("°", "other", "横滚角"),
    "yaw": ("°", "other", "航向角"),
    "ax_g": ("g", "other", "加速度X"),
    "ay_g": ("g", "other", "加速度Y"),
    "az_g": ("g", "other", "加速度Z"),
    "gx_dps": ("°/s", "other", "陀螺X"),
    "gy_dps": ("°/s", "other", "陀螺Y"),
    "gz_dps": ("°/s", "other", "陀螺Z"),
    "ax_raw": ("LSB", "other", "AX原始"),
    "ay_raw": ("LSB", "other", "AY原始"),
    "az_raw": ("LSB", "other", "AZ原始"),
    "gx_raw": ("LSB", "other", "GX原始"),
    "gy_raw": ("LSB", "other", "GY原始"),
    "gz_raw": ("LSB", "other", "GZ原始"),
    "seq": ("#", "other", "采样序号"),
    "mcu_ms": ("ms", "other", "MCU时间"),
    "target_rpm": ("RPM", "target", "目标转速"),
    "actual_rpm": ("RPM", "actual", "实际转速"),
    "error": ("", "error", "误差"),
    "pos_target": ("°", "target", "目标位置"),
    "pos_actual": ("°", "actual", "实际位置"),
    "drive_pwm": ("PWM", "output", "驱动PWM"),
    "pos_err": ("°", "error", "位置误差"),
}

_ALIASES: dict[str, str] = {
    "pitch_target": "pitch_target",
    "pitch_tgt": "pitch_target",
    "target_deg": "pitch_target",
    "tgt_deg": "pitch_target",
    "ang_tgt": "pitch_target",
    "pitch": "pitch",
    "pitch_deg": "pitch",
    "angle": "pitch",
    "gyro_pitch": "gyro_pitch",
    "gyro": "gyro_pitch",
    "gyro_dps": "gyro_pitch",
    "gx_dps": "gx_dps",
    "gy_dps": "gy_dps",
    "gz_dps": "gz_dps",
    "pwm": "pwm",
    "drive_pwm": "pwm",
    "pwm_out": "pwm",
    "wheel_speed": "wheel_speed",
    "speed": "wheel_speed",
    "speed_filtered": "wheel_speed",
    "enc_speed": "wheel_speed",
    "roll": "roll",
    "roll_deg": "roll",
    "yaw": "yaw",
    "yaw_deg": "yaw",
    "ax": "ax_g",
    "ax_g": "ax_g",
    "ay": "ay_g",
    "ay_g": "ay_g",
    "az": "az_g",
    "az_g": "az_g",
    "gx": "gx_dps",
    "gy": "gy_dps",
    "gz": "gz_dps",
    "ax_raw": "ax_raw",
    "ay_raw": "ay_raw",
    "az_raw": "az_raw",
    "gx_raw": "gx_raw",
    "gy_raw": "gy_raw",
    "gz_raw": "gz_raw",
    "seq": "seq",
    "frame_id": "seq",
    "sample_idx": "seq",
    "mcu_ms": "mcu_ms",
    "t_ms": "mcu_ms",
    "tick_ms": "mcu_ms",
    "timestamp_ms": "mcu_ms",
    "time_ms": "mcu_ms",
    "target_rpm": "target_rpm",
    "set_rpm": "target_rpm",
    "actual_rpm": "actual_rpm",
    "wheel_rpm": "actual_rpm",
    "pos_target": "pos_target",
    "pos_actual": "pos_actual",
}


def resolve_channel_semantics(names: list[str] | tuple[str, ...]) -> dict[str, int]:
    """Map canonical telemetry field names (pitch_target, pitch, gyro_pitch, pwm, wheel_speed, ...) to column indices."""
    resolved: dict[str, int] = {}
    for idx, raw in enumerate(names):
        tok = str(raw or "").strip().lower()
        canon = _ALIASES.get(tok)
        if canon and canon not in resolved:
            resolved[canon] = idx
    return resolved


def infer_channel_unit(name: str) -> str:
    """Return default engineering unit for a recognized channel name, or ''."""
    tok = str(name or "").strip().lower()
    canon = _ALIASES.get(tok, tok)
    entry = CANONICAL_CHANNELS.get(canon)
    if entry and entry[0]:
        return entry[0]
    if "rpm" in tok:
        return "RPM"
    if "pwm" in tok:
        return "PWM"
    if "deg" in tok or "pitch" in tok or "roll" in tok or "yaw" in tok or "angle" in tok or tok.startswith("ang_") or tok.endswith("_ang"):
        return "°"
    if tok.endswith("_ms"):
        return "ms"
    return ""
