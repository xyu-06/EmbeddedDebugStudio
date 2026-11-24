"""Signal diagnostics (FFT, noise, frame jitter, watchdog) and C code generator."""
from dataclasses import dataclass
import numpy as np
from PySide6 import QtCore as C, QtGui, QtWidgets as W

from engine import KEYS, LOOPS


@dataclass
class ChannelHealth:
    name: str
    samples: int
    mean: float
    std: float
    ptp: float
    dom_freq_hz: float | None
    dom_amp: float
    diagnosis: str


def dominant_fft(times, values):
    """Estimate dominant oscillation frequency (Hz) and peak amplitude on (times, values).

    Resamples onto a uniform grid when timestamps have USB-TTL batching jitter.
    Returns (freq_hz, amplitude) or (None, 0.0) if insufficient samples or flat.
    """
    t = np.asarray(times, dtype=float)
    y = np.asarray(values, dtype=float)
    valid = np.isfinite(t) & np.isfinite(y)
    t, y = t[valid], y[valid]
    if len(t) < 16:
        return None, 0.0
    span = float(t[-1] - t[0])
    if span <= 0.08:
        return None, 0.0

    # Interpolate onto uniform grid to handle minor serial timestamp batching
    n = min(4096, len(t))
    tu = np.linspace(float(t[0]), float(t[-1]), n)
    # Collapse duplicate timestamps before interp
    uniq_mask = np.r_[True, np.diff(t) > 1e-9]
    if np.sum(uniq_mask) < 12:
        return None, 0.0
    yu = np.interp(tu, t[uniq_mask], y[uniq_mask])
    dt = span / (n - 1)
    centered = yu - np.mean(yu)
    if float(np.ptp(centered)) < 1e-6:
        return None, 0.0

    window = np.hanning(n)
    spec = np.abs(np.fft.rfft(centered * window)) * (2.0 / np.sum(window))
    freqs = np.fft.rfftfreq(n, d=dt)
    # Ignore DC and sub-1.5-cycle drift bins
    min_bin = max(1, int(np.ceil(1.5 / span / max(freqs[1] - freqs[0], 1e-9))))
    if min_bin >= len(spec):
        return None, 0.0
    idx = min_bin + int(np.argmax(spec[min_bin:]))
    amp = float(spec[idx])
    if amp < 0.04 * float(np.std(centered)):
        return None, 0.0
    return float(freqs[idx]), amp


def classify_oscillation(freq_hz, amp, ptp):
    if freq_hz is None or ptp < 1e-4 or amp < 0.12 * ptp:
        return "平稳 / 无明显周期振荡"
    if freq_hz < 4.5:
        return f"低频周期波动 {freq_hz:.2f} Hz（检查 KP/KI 偏大或目标周期变化）"
    if freq_hz < 18.0:
        return f"中频振荡 {freq_hz:.1f} Hz（常见于相角裕度不足或 KD 偏小/偏大）"
    return f"高频抖动 {freq_hz:.1f} Hz（检查 KD 放大噪声、编码器量化台阶或机械共振）"


def analyze_frame_timing(times):
    """Return (fps_hz, mean_dt_ms, max_dt_ms, batch_ratio) from sample timestamps."""
    t = np.asarray(times, dtype=float)
    t = t[np.isfinite(t)]
    if len(t) < 4 or t[-1] <= t[0]:
        return 0.0, 0.0, 0.0, 0.0
    span = float(t[-1] - t[0])
    fps = (len(t) - 1) / span
    dts = np.diff(t) * 1000.0
    batch_ratio = float(np.mean(dts < 0.4))  # USB-TTL coalesced packets arrive <0.4ms apart
    return float(fps), float(np.mean(dts)), float(np.max(dts)), batch_ratio


def inspect_channels(data, count, names):
    """Compute ChannelHealth for active channels 0..count-1 on buffer slice `data`."""
    results = []
    if not len(data) or count <= 0:
        return results
    times = data[:, 0]
    for ch in range(min(4, count)):
        col = data[:, ch + 1]
        valid = col[np.isfinite(col)]
        if not len(valid):
            continue
        mean = float(np.mean(valid))
        std = float(np.std(valid))
        ptp = float(np.ptp(valid))
        freq, amp = dominant_fft(times, col)
        if len(valid) >= 20 and ptp == 0.0:
            diag = "数值恒定不变（静止零位，或检查传感器是否停止更新）"
        else:
            diag = classify_oscillation(freq, amp, ptp)
        results.append(ChannelHealth(names[ch], len(valid), mean, std, ptp, freq, amp, diag))
    return results


class RuntimeMonitor:
    """Passive non-intrusive watchdog for prolonged actuator saturation or frozen sensor."""

    def __init__(self):
        self.last_check = 0.0
        self.last_alert = ""
        self.alert_cooldown = 0.0

    def reset(self):
        self.last_check = 0.0
        self.last_alert = ""
        self.alert_cooldown = 0.0

    def check(self, now, data, actual_col, output_col, running):
        """Return a warning string when a new anomaly is detected, else None."""
        if now - self.last_check < 0.6:
            return None
        self.last_check = now
        if not running or output_col is None or actual_col is None or len(data) < 40:
            return None
        t_end = data[-1, 0]
        recent = data[data[:, 0] >= t_end - 2.6]
        if len(recent) < 25 or recent[-1, 0] - recent[0, 0] < 2.2:
            return None

        u = recent[:, output_col + 1]
        y = recent[:, actual_col + 1]
        u = u[np.isfinite(u)]
        y = y[np.isfinite(y)]
        if len(u) < 20 or len(y) < 20:
            return None

        alert = None
        # 1. Sensor stuck bit-for-bit identical while actuator output is actively driving!
        if float(np.ptp(y)) == 0.0 and float(np.mean(np.abs(u))) > 25.0:
            alert = "告警：控制输出非零但反馈值连续 2.5 s 毫无变化，请检查编码器/传感器接线或电机电源"
        else:
            # 2. Actuator pinned at ceiling for >2.5s while feedback stopped changing
            all_u = data[:, output_col + 1]
            all_u = all_u[np.isfinite(all_u)]
            u_ceil = max(float(np.max(np.abs(all_u))), 100.0)
            if u_ceil >= 80.0 and np.mean(np.abs(u) >= 0.985 * u_ceil) > 0.92 and float(np.std(u)) < 0.015 * u_ceil:
                if float(np.ptp(y[-10:])) < 0.02 * max(abs(float(np.mean(y))), 1.0):
                    alert = "提示：输出通道已连续满幅饱和超过 2.5 s，若达不到目标请降低 TGT 或检查供电/限幅"

        if alert and (alert != self.last_alert or now >= self.alert_cooldown):
            self.last_alert = alert
            self.alert_cooldown = now + 12.0
            return alert
        if not alert:
            self.last_alert = ""
        return None


def format_c_code(group, params_by_group, template_idx):
    """Generate ready-to-paste C definitions matching the user's STM32 balance car project."""
    p = params_by_group[group]
    ang = params_by_group["ANG"]
    spd = params_by_group["SPD"]
    yaw = params_by_group["YAW"]

    if template_idx == 0:
        # Matches User/pid_debug.c & User/pid.c
        return (
            f"// 粘贴至 User/pid_debug.c（{group} 环整定结果）\n"
            f"#define PID_DEBUG_DEFAULT_TARGET_RPM  {p['TGT']:.4f}f\n"
            f"#define PID_DEBUG_DEFAULT_KP          {p['KP']:.4f}f\n"
            f"#define PID_DEBUG_DEFAULT_KI          {p['KI']:.4f}f\n"
            f"#define PID_DEBUG_DEFAULT_KD          {p['KD']:.4f}f\n\n"
            f"// 若使用独立 User/pid.c：\n"
            f"#define PID_TARGET_RPM                {p['TGT']:.4f}f\n"
            f"#define PID_KP                        {p['KP']:.4f}f\n"
            f"#define PID_KI                        {p['KI']:.4f}f\n"
            f"#define PID_KD                        {p['KD']:.4f}f\n"
        )
    if template_idx == 1:
        # Matches User/balance.c
        return (
            "// 粘贴至 User/balance.c（直立环 ANG 参数）\n"
            f"#define BALANCE_TARGET_PITCH_DEG      {ang['TGT']:.4f}f   // 机械中值俯仰角（度）\n"
            f"#define BALANCE_KP                    {ang['KP']:.4f}f   // 直立比例系数（PWM / 度）\n"
            f"#define BALANCE_KD                    {ang['KD']:.4f}f   // 直立微分系数（PWM / (度/秒)）\n"
        )
    # Multi-loop struct / pid_link.c initializer
    lines = ["// 多环 PID 参数初始化（直立环 ANG / 速度环 SPD / 转向环 YAW / 辅助环 AUX）"]
    for g in LOOPS:
        gp = params_by_group[g]
        lines.append(
            f"pid_loops[PID_{g}].kp = {gp['KP']:.4f}f; "
            f"pid_loops[PID_{g}].ki = {gp['KI']:.4f}f; "
            f"pid_loops[PID_{g}].kd = {gp['KD']:.4f}f; "
            f"pid_loops[PID_{g}].target = {gp['TGT']:.4f}f;"
        )
    return "\n".join(lines) + "\n"


def c_export_dialog(w):
    w.save_draft()
    params = {}
    for g in LOOPS:
        params[g] = {k: w.confirmed.get(f"{g}.{k}", w.drafts[g][k]) for k in KEYS}

    dlg = W.QDialog(w)
    dlg.setWindowTitle("生成 C 语言参数代码")
    dlg.setMinimumSize(540, 360)
    outer = W.QVBoxLayout(dlg)

    top = W.QHBoxLayout()
    top.addWidget(W.QLabel("代码模板"))
    tpl = W.QComboBox()
    tpl.addItems([
        "模板 1：速度环宏定义（适配 User/pid_debug.c 与 pid.c）",
        "模板 2：平衡车直立环宏定义（适配 User/balance.c）",
        "模板 3：四环完整初始化语句（适配 pid_link / 串级工程）",
    ])
    if w.selected() == "ANG":
        tpl.setCurrentIndex(1)
    elif w.selected() == "SPD":
        tpl.setCurrentIndex(0)
    else:
        tpl.setCurrentIndex(2)
    top.addWidget(tpl, 1)
    outer.addLayout(top)

    editor = W.QPlainTextEdit()
    editor.setReadOnly(True)
    outer.addWidget(editor, 1)

    def refresh():
        editor.setPlainText(format_c_code(w.selected(), params, tpl.currentIndex()))

    tpl.currentIndexChanged.connect(refresh)
    refresh()

    btns = W.QHBoxLayout()
    note = W.QLabel("提示：优先导出设备已确认（绿色）参数，未读回项采用当前面板值。")
    note.setStyleSheet("color:#5b7083;font-size:9pt;")
    btns.addWidget(note, 1)
    copy_btn = W.QPushButton("复制到剪贴板")
    copy_btn.setObjectName("primary")
    close_btn = W.QPushButton("关闭")

    def do_copy():
        W.QApplication.clipboard().setText(editor.toPlainText())
        w.feedback("已复制 C 参数代码到剪贴板")
        dlg.accept()

    copy_btn.clicked.connect(do_copy)
    close_btn.clicked.connect(dlg.reject)
    btns.addWidget(copy_btn)
    btns.addWidget(close_btn)
    outer.addLayout(btns)
    dlg.exec()


def diagnostics_dialog(w):
    """Hardware & signal health check over the currently visible / recent window."""
    a = w.scope.data if w.scope.frozen else w.buffer.view(copy=True)
    count = w.frozen_count if w.scope.frozen else w.count
    if not len(a) or count <= 0:
        w.feedback("暂无采样数据，请先连接设备或打开仿真采集一段波形", True)
        return

    left, right = w.scope.viewRange()[0]
    seg = a[(a[:, 0] >= left) & (a[:, 0] <= right)]
    if len(seg) < 8:
        seg = a[-min(len(a), 500):]

    fps, mean_ms, max_ms, batch_ratio = analyze_frame_timing(seg[:, 0])
    health_rows = inspect_channels(seg, count, w.names())

    dlg = W.QDialog(w)
    dlg.setWindowTitle("硬件与信号体检报告")
    dlg.setMinimumSize(660, 420)
    outer = W.QVBoxLayout(dlg)

    jitter_tip = (
        f"检测到约 {batch_ratio*100:.0f}% 采样帧存在 USB-TTL 攒批到达现象，建议点击下方按钮改用“固定上报周期”消除横轴抖动。"
        if batch_ratio > 0.18 and w.time_mode.currentIndex() == 0
        else "帧间隔均匀，时间轴状态良好。"
    )
    summary = W.QLabel(
        f"<b>分析区间：</b>{seg[0,0]:.2f} s ～ {seg[-1,0]:.2f} s（共 {len(seg)} 帧）　"
        f"<b>实测平均帧率：</b>{fps:.1f} Hz（均值间隔 {mean_ms:.1f} ms，最大间隔 {max_ms:.1f} ms）<br>"
        f"<span style='color:#52687b'>{jitter_tip}</span>"
    )
    summary.setWordWrap(True)
    outer.addWidget(summary)

    table = W.QTableWidget(len(health_rows), 6)
    table.setHorizontalHeaderLabels(["通道", "均值（零偏/机械中值）", "标准差 σ（噪声）", "峰峰值", "主频 FFT", "诊断提示"])
    table.verticalHeader().setVisible(False)
    table.setEditTriggers(W.QAbstractItemView.EditTrigger.NoEditTriggers)
    table.setSelectionMode(W.QAbstractItemView.SelectionMode.NoSelection)
    for r, h in enumerate(health_rows):
        vals = [
            h.name,
            f"{h.mean:.4g}",
            f"{h.std:.4g}",
            f"{h.ptp:.4g}",
            f"{h.dom_freq_hz:.2f} Hz (幅 {h.dom_amp:.3g})" if h.dom_freq_hz else "—",
            h.diagnosis,
        ]
        for c_i, text in enumerate(vals):
            item = W.QTableWidgetItem(text)
            table.setItem(r, c_i, item)
    header = table.horizontalHeader()
    header.setSectionResizeMode(W.QHeaderView.ResizeMode.ResizeToContents)
    header.setSectionResizeMode(5, W.QHeaderView.ResizeMode.Stretch)
    outer.addWidget(table, 1)

    tip = W.QLabel(
        "用法建议：① 平衡车静止扶正时，俯仰角通道的“均值”即为机械中值角（可填入 BALANCE_TARGET_PITCH_DEG）；"
        "② 静止时“标准差 σ”反映陀螺仪/编码器底噪；③ 振荡时“主频 FFT”帮助区分低频KP振荡与高频KD噪声。"
    )
    tip.setWordWrap(True)
    tip.setStyleSheet("color:#5b7083;font-size:9pt;")
    outer.addWidget(tip)

    btns = W.QHBoxLayout()
    apply_rate_btn = W.QPushButton(f"采用实测帧率 ({max(1, round(fps))} Hz) 为固定周期")
    if fps < 1.0:
        apply_rate_btn.setEnabled(False)

    def apply_rate():
        w.rate.setValue(max(1, round(fps)))
        w.time_mode.setCurrentIndex(1)
        w.feedback(f"已将上报频率设为 {max(1, round(fps))} Hz 并切换为固定上报周期")
        dlg.accept()

    apply_rate_btn.clicked.connect(apply_rate)
    close_btn = W.QPushButton("关闭")
    close_btn.clicked.connect(dlg.accept)
    btns.addWidget(apply_rate_btn)
    btns.addStretch()
    btns.addWidget(close_btn)
    outer.addLayout(btns)
    dlg.exec()
