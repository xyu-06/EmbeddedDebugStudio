"""Qt dialog and real-time closed-loop execution controller for PID Auto-Tune."""
import re
from time import monotonic
import numpy as np
from PySide6 import QtCore as C, QtGui, QtWidgets as W

from autotune import MODE_KEYS, MODES, STYLES, STRATEGIES, AutoTuner, is_step_still_climbing
from engine import KEYS, LIMIT, LOOPS, MAX_CHANNELS
from layout_ui import combo, number, check
from telemetry_schema import resolve_channel_semantics


class AutoTuneController:
    """Drives AutoTuner over live serial or simulated telemetry inside Window.tick()."""

    IDLE, READ_BASELINE, WAIT_START, PREPARE, WAIT_BASE_ACK, SETTLE_BASE, WAIT_STEP_ACK, OBSERVE, WAIT_BRAKE_ACK, OBSERVE_BRAKE, FINALIZING = range(11)

    def __init__(self, window):
        self.ui = window
        self.engine = None
        self.state = self.IDLE
        self.group = "ANG"
        self.actual_col = 1
        self.output_col = None
        self.target_col = None
        self.pitch_col = None
        self.pitch_ref = 0.0
        self.eval_brake = False
        self.reverify_winner = False
        self._start_seg = None
        self._climbing_extended = False
        self.hold_target = True
        self.saved_before = None
        self.current_cand = None
        self.current_tag = ""
        self.phase_t0 = 0.
        self.wall_deadline = 0.
        self.base_y = 0.
        self.step_pre_t = 0.
        self.step_t0 = 0.
        self.step_t0_source = ""
        self.step_t0_resolved = False
        self.last_valid_actual_t = None
        self.last_valid_actual_wall = None
        self.last_best_ref = None
        self.last_history = []
        self.last_history_group = "SPD"
        self._pending_start = None
        self.finalize_mode = ""
        self.finalize_need_stop = False
        self.finalize_stop_acked = False
        self.finalize_target = None
        self.finalize_reason = ""
        self.finalize_summary = ""
        self.finalize_deadline = 0.
        # Remember dialog defaults across opens in the same session
        self.cfg = {
            "target": 50.0,
            "base": 0.0,
            "mode": 0,
            "style": 1,
            "strategy": 0,
            "step_s": 2.0,
            "rounds": 10,
            "hold": True,
            "output_limit": 0.0,
            "eval_brake": False,
            "reverify_winner": False,
        }

    def _find_pitch_col(self):
        count, _names, _roles, smap = self._channel_meta()
        col = smap.get("pitch")
        if col is not None and col < count and col != self.actual_col and col != self.output_col:
            return col
        return None

    def _channel_meta(self):
        w = self.ui
        count = max(0, min(int(w.count), MAX_CHANNELS))
        raw_names = [re.sub(r"\s*\[[^\]]*\]$", "", str(n or "")).strip() for n in w.names()[:count]]
        roles = list(getattr(w, "channel_roles", []))[:count]
        return count, raw_names, roles, resolve_channel_semantics(raw_names)

    @staticmethod
    def _has_target_token(text):
        low = str(text or "").lower()
        return ("目标" in low) or bool(re.search(r"(?:^|[_\W])(target|setpoint|sp|tgt|ref)(?:$|[_\W])", low))

    @staticmethod
    def _has_angle_token(text):
        low = str(text or "").lower()
        return any(t in low for t in ("pitch", "angle", "tilt", "倾角", "roll", "yaw"))

    def _find_speed_actual_col(self):
        count, names, _roles, smap = self._channel_meta()
        for key in ("actual_rpm", "wheel_speed"):
            if key in smap and smap[key] < count:
                return smap[key]
        speed_tags = ("wheel_speed", "actual_rpm", "wheel_rpm", "rpm", "speed", "vel", "轮速", "转速")
        for idx, raw in enumerate(names):
            low = raw.lower()
            if any(t in low for t in speed_tags) and not self._has_target_token(low):
                return idx
        return None

    def _find_speed_target_col(self):
        count, names, _roles, smap = self._channel_meta()
        if "target_rpm" in smap and smap["target_rpm"] < count:
            return smap["target_rpm"]
        speed_tags = ("wheel_speed", "rpm", "speed", "vel", "轮速", "转速")
        for idx, raw in enumerate(names):
            low = raw.lower()
            if any(t in low for t in speed_tags) and self._has_target_token(low):
                return idx
        return None

    def missing_speed_feedback_error(self, group=None):
        """Return an error string if SPD autotune is requested on angle-only telemetry lacking wheel speed."""
        w = self.ui
        grp = group or w.selected()
        if grp != "SPD" or w.sim or w.count <= 0:
            return ""
        spd_col = self._find_speed_actual_col()
        if spd_col is not None:
            return ""
        count, names, _roles, smap = self._channel_meta()
        pref_actual = w.channel_preferences[1] if hasattr(w, "channel_preferences") else None
        is_angle_stream = ("pitch" in smap) or any(
            ("pitch" in n.lower() or "angle" in n.lower() or "倾角" in n) for n in names
        )
        if is_angle_stream and (pref_actual is None or pref_actual == smap.get("pitch") or pref_actual >= count):
            return "当前通道仅包含姿态/倾角数据，缺少轮速 (RPM) 反馈通道，请先开启轮速上报后再整定 SPD"
        return ""

    def has_role_conflict(self, group=None):
        """Detect mismatch between manually selected actual channel and the active loop's expected feedback."""
        w = self.ui
        grp = group or w.selected()
        count, names, roles, _smap = self._channel_meta()
        selected_actual = w.actual_ch.currentData()
        if grp == "SPD":
            spd_col = self._find_speed_actual_col()
            if spd_col is None or selected_actual is None or selected_actual >= count or selected_actual == spd_col:
                return False, ""
            sel_name = names[selected_actual] if selected_actual < len(names) else f"CH{selected_actual+1}"
            spd_name = names[spd_col] if spd_col < len(names) else f"CH{spd_col+1}"
            return True, (
                f"⚠ 当前整定 SPD（速度环），但反馈通道选了 CH{selected_actual+1}({sel_name})，"
                f"建议改为轮速通道 CH{spd_col+1}({spd_name})。"
            )
        declared_actual = next((i for i, r in enumerate(roles) if r == "actual"), None)
        if declared_actual is None or selected_actual is None or declared_actual >= count:
            return False, ""
        if selected_actual != declared_actual:
            sel_name = names[selected_actual] if selected_actual < len(names) else f"CH{selected_actual+1}"
            dec_name = names[declared_actual] if declared_actual < len(names) else f"CH{declared_actual+1}"
            return True, (
                f"⚠ 当前反馈通道为 CH{selected_actual+1}({sel_name})，"
                f"但设备 #ROLES 声明 actual=CH{declared_actual+1}({dec_name})，请核对后再整定。"
            )
        return False, ""

    @property
    def active(self):
        return self.state not in (self.IDLE, self.FINALIZING)

    def _find_target_col(self, group=None):
        w = self.ui
        grp = group or self.group or w.selected()
        count, names, roles, _smap = self._channel_meta()
        if grp == "SPD":
            col = self._find_speed_target_col()
            if col is not None and col < count and col != self.actual_col:
                return col
            col = next(
                (i for i, r in enumerate(roles) if i != self.actual_col and r == "target" and not self._has_angle_token(names[i])),
                None,
            )
            if col is None:
                col = next(
                    (i for i, n in enumerate(names) if i != self.actual_col and self._has_target_token(n) and not self._has_angle_token(n)),
                    None,
                )
            return col if (col is not None and col < count and col != self.actual_col) else None
        col = next((i for i, r in enumerate(roles) if i != self.actual_col and r == "target"), None)
        if col is None:
            col = next((i for i, n in enumerate(names) if i != self.actual_col and self._has_target_token(n)), None)
        return col if (col is not None and col < count and col != self.actual_col) else None

    def detect_channels(self, sync_ui=True, group=None):
        """Return (actual_channel_index, optional_output_channel_index)."""
        w = self.ui
        grp = group or w.selected()
        target_tags = ("target", "目标", "setpoint", "sp", "tgt", "ref")
        actual_tags = ("actual", "实际", "feedback", "fb", "meas")
        output_tags = ("pwm", "输出", "output", "out", "duty", "占空")
        roles = list(getattr(w, "channel_roles", []))[:w.count]
        names = [n.lower() for n in w.names()[:w.count]]
        pref_actual = w.channel_preferences[1] if hasattr(w, "channel_preferences") else None

        actual = w.actual_ch.currentData()
        if pref_actual is not None and 0 <= pref_actual < w.count:
            actual = pref_actual
        elif grp == "SPD" and not w.sim:
            spd_col = self._find_speed_actual_col()
            if spd_col is not None:
                actual = spd_col
            else:
                role_actual = next((i for i, r in enumerate(roles) if r == "actual"), None)
                actual = role_actual if role_actual is not None else (actual if (actual is not None and actual < max(1, w.count)) else 0)
        else:
            role_actual = next((i for i, r in enumerate(roles) if r == "actual"), None)
            if role_actual is not None:
                actual = role_actual
            else:
                name_actual = next((i for i, n in enumerate(names) if any(t in n for t in actual_tags)), None)
                if name_actual is not None:
                    actual = name_actual
                elif actual is None or actual >= max(1, w.count) or actual >= MAX_CHANNELS:
                    if w.sim or w.count >= 3 or (w.count >= 2 and any(t in names[0] for t in target_tags)):
                        actual = 1
                    else:
                        actual = 0

        if sync_ui and actual is not None and actual != w.actual_ch.currentData():
            idx = w.actual_ch.findData(actual)
            if idx >= 0:
                with C.QSignalBlocker(w.actual_ch):
                    w.actual_ch.setCurrentIndex(idx)

        output = next((i for i, r in enumerate(roles) if i != actual and r == "output"), None)
        if output is None:
            output = next((i for i, n in enumerate(names) if i != actual and any(t in n for t in output_tags)), None)
        if output is None:
            if (w.sim or w.count >= 3) and actual != 2:
                output = 2
            elif w.count == 2 and w.right_channel == 1 - actual:
                output = 1 - actual
        return actual, output

    def open_dialog(self):
        w = self.ui
        if self.active:
            self.stop("已手动停止，采用当前最优参数", apply_best=True)
            return
        if not w.connected:
            w.feedback("请先连接串口或打开仿真后再启动自动整定", True)
            return

        group = w.selected()
        missing_spd_msg = self.missing_speed_feedback_error(group)
        if missing_spd_msg:
            w.feedback(missing_spd_msg, True)
            return
        actual_ch, output_ch = self.detect_channels(group=group)
        ch_names = w.names()
        actual_label = ch_names[actual_ch] if actual_ch < len(ch_names) else f"CH{actual_ch+1}"
        out_label = ch_names[output_ch] if output_ch is not None and output_ch < len(ch_names) else "未指定"

        dlg = W.QDialog(w)
        dlg.setWindowTitle(f"PID 自动整定（{group}）")
        dlg.setMinimumWidth(410)
        outer = W.QVBoxLayout(dlg)
        intro = W.QLabel(
            f"软件将自动下发阶跃目标并分析 <b>{actual_label}</b> 的响应波形（输出参考：{out_label}），"
            "通过模型辨识与多轮闭环评估自动寻优 KP / KI / KD。"
        )
        intro.setWordWrap(True)
        outer.addWidget(intro)

        form = W.QFormLayout()
        outer.addLayout(form)

        cur_tgt = w.fields["TGT"].value()
        has_nonzero_init = abs(w.fields["KP"].value()) > 1e-4 or abs(w.fields["KI"].value()) > 1e-4
        self.actual_col, self.output_col = self.detect_channels(group=group)
        is_balance_spd = (group == "SPD") and (not w.sim) and (self._find_pitch_col() is not None)
        default_tgt = cur_tgt if abs(cur_tgt) > 1e-3 else (30.0 if is_balance_spd else self.cfg["target"])
        target_spin = number(default_tgt, -LIMIT, LIMIT, 5.0, 3)
        base_spin = number(0.0 if is_balance_spd else self.cfg["base"], -LIMIT, LIMIT, 5.0, 3)
        if abs(default_tgt - base_spin.value()) < 1e-3:
            base_spin.setValue(0.0 if abs(default_tgt) > 1e-3 else -10.0)

        is_ang = group == "ANG"
        default_mode = 2 if (is_ang or (w.sim and w.sim.pendulum)) else 0 if is_balance_spd else 1 if w.sim and w.sim.model == 1 else self.cfg["mode"]
        default_strategy = 1 if (is_ang or (w.sim and w.sim.pendulum) or ((not w.sim) and has_nonzero_init)) else self.cfg["strategy"]
        mode_box = combo(list(MODES), default_mode)
        style_box = combo(list(STYLES), self.cfg["style"])
        strat_box = combo(list(STRATEGIES), default_strategy)
        if is_ang:
            strat_box.setCurrentIndex(1)
            strat_box.setEnabled(False)
            strat_box.setToolTip("直立环 (ANG) 属于开环不稳定对象，为防止倒地飞车已强制锁定为局部微调")
        default_step_s = 2.8 if is_balance_spd else self.cfg["step_s"]
        step_spin = number(default_step_s, 0.5, 30.0, 0.5, 2)
        rounds_spin = W.QSpinBox()
        rounds_spin.setRange(3, 25)
        rounds_spin.setValue(int(self.cfg["rounds"]))
        limit_val = float(self.cfg.get("output_limit", 0.0))
        if limit_val <= 0 and hasattr(w, "capture_pwm_limit"):
            limit_val = float(w.capture_pwm_limit.value())
        if limit_val <= 0 and is_balance_spd:
            limit_val = 950.0
        limit_spin = number(limit_val, 0.0, LIMIT, 50.0, 1)
        limit_spin.setSpecialValueText("自动估计")
        limit_spin.setToolTip("填写控制器真实输出上限（如平衡车 ±950），用于准确统计触顶饱和占比；0 为自动估计")
        hold_chk = check("整定完成后保持在目标值运行", False if is_balance_spd else self.cfg["hold"])
        brake_chk = check("联合评估起步 + 刹停 (0 → 目标 → 0，平衡车控速推荐)", is_balance_spd or self.cfg.get("eval_brake", False))
        reverify_chk = check("优胜组与原参数复测确认后再写回", is_balance_spd or self.cfg.get("reverify_winner", False))
        safety_chk = check("我已确认：车轮已架空离地或已加装机械限位，且知晓随时可按 Esc 急停", False)

        form.addRow("目标值（如控速 30）", target_spin)
        form.addRow("起始基准值", base_spin)
        form.addRow("控制器类型", mode_box)
        form.addRow("响应偏好", style_box)
        form.addRow("整定策略", strat_box)
        form.addRow("单轮观察时长 s", step_spin)
        form.addRow("最大寻优轮数", rounds_spin)
        form.addRow("输出限幅 ±", limit_spin)
        form.addRow(brake_chk)
        form.addRow(reverify_chk)
        form.addRow(hold_chk)
        form.addRow(safety_chk)

        mode_hint = W.QLabel()
        mode_hint.setWordWrap(True)
        mode_hint.setStyleSheet("color:#486075;font-size:9pt;padding-top:2px;")
        outer.addWidget(mode_hint)

        def _update_mode_hint():
            m_idx = mode_box.currentIndex()
            active_k = MODE_KEYS[m_idx]
            dropped = [k for k in ("KP", "KI", "KD") if k not in active_k and abs(w.fields[k].value()) > 1e-4]
            extra = f"（注意：第 1 轮按设备当前原参数运行，后续寻优候选将清零 {', '.join(dropped)}）" if dropped else ""
            readback_note = (
                " 启动前将自动 @GET 读回设备已确认参数作回滚基准。"
                if (not w.sim and any(f"{group}.{k}" not in w.confirmed for k in KEYS))
                else ""
            )
            mode_hint.setText(f"当前寻优调节项：{', '.join(active_k)}{extra}。{readback_note}")

        _update_mode_hint()
        mode_box.currentIndexChanged.connect(lambda _: _update_mode_hint())

        self.detect_channels(group=group)
        conflict, conflict_msg = self.has_role_conflict(group)
        conflict_chk = None
        if conflict:
            c_lbl = W.QLabel(conflict_msg)
            c_lbl.setWordWrap(True)
            c_lbl.setStyleSheet("color:#a22d38;font-size:9pt;font-weight:600;padding-top:2px;")
            outer.addWidget(c_lbl)
            conflict_chk = check("我已核对通道映射差异，确认仍按当前手选通道继续", False)
            outer.addWidget(conflict_chk)

        if is_ang:
            ang_warn = W.QLabel("⚠ 当前选中直立环 (ANG)：已强制锁定“从当前参数局部微调”（单轮变化 ≤ ±35%），请先确保现有初始 KP/KD 能稳住车身且目标值仅设小幅阶跃（如 0°→3°）。")
            ang_warn.setWordWrap(True)
            ang_warn.setStyleSheet("color:#a22d38;font-size:9pt;font-weight:600;padding-top:2px;")
            outer.addWidget(ang_warn)

        tip = W.QLabel(
            "提示：电机控速建议选“PI 整定 + 均衡推荐”；二阶位置/角度选“PID 整定”；"
            "若整定失败或被中断，软件会自动将设备恢复到整定前的原始参数。途中随时可按 Esc 急停。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#5b7083;font-size:9pt;padding-top:4px;")
        outer.addWidget(tip)

        buttons = W.QDialogButtonBox(W.QDialogButtonBox.StandardButton.Ok | W.QDialogButtonBox.StandardButton.Cancel)
        ok_btn = buttons.button(W.QDialogButtonBox.StandardButton.Ok)
        ok_btn.setText("开始自动整定")
        ok_btn.setEnabled(False)
        def _refresh_ok():
            ok_btn.setEnabled(safety_chk.isChecked() and (conflict_chk is None or conflict_chk.isChecked()))
        safety_chk.toggled.connect(lambda _: _refresh_ok())
        if conflict_chk is not None:
            conflict_chk.toggled.connect(lambda _: _refresh_ok())
        buttons.button(W.QDialogButtonBox.StandardButton.Cancel).setText("取消")
        buttons.accepted.connect(dlg.accept)
        buttons.rejected.connect(dlg.reject)
        outer.addWidget(buttons)

        if dlg.exec() != W.QDialog.DialogCode.Accepted:
            return

        self.cfg.update(
            target=target_spin.value(),
            base=base_spin.value(),
            mode=mode_box.currentIndex(),
            style=style_box.currentIndex(),
            strategy=strat_box.currentIndex(),
            step_s=step_spin.value(),
            rounds=rounds_spin.value(),
            hold=hold_chk.isChecked(),
            output_limit=limit_spin.value(),
            eval_brake=brake_chk.isChecked(),
            reverify_winner=reverify_chk.isChecked(),
        )
        self.start(
            mode=self.cfg["mode"],
            style=self.cfg["style"],
            strategy=self.cfg["strategy"],
            base=self.cfg["base"],
            target=self.cfg["target"],
            step_seconds=self.cfg["step_s"],
            max_rounds=self.cfg["rounds"],
            hold_target=self.cfg["hold"],
            output_limit=self.cfg["output_limit"],
            confirm_role=bool(conflict_chk and conflict_chk.isChecked()),
            eval_brake=self.cfg["eval_brake"],
            reverify_winner=self.cfg["reverify_winner"],
        )

    def start(
        self, mode, style, strategy, base, target, step_seconds=2.0,
        max_rounds=10, hold_target=True, output_limit=None, confirm_role=False,
        eval_brake=False, reverify_winner=False,
    ):
        w = self.ui
        if not w.connected:
            w.feedback("请先连接串口或打开仿真", True)
            return False
        group = w.selected()
        if group == "ANG" and strategy == 0:
            w.feedback("直立环 (ANG) 属于开环不稳定对象，仅允许使用【从当前参数局部微调】策略", True)
            return False
        missing_spd_msg = self.missing_speed_feedback_error(group)
        if missing_spd_msg:
            w.feedback(missing_spd_msg, True)
            return False
        tgt_lo, tgt_hi, _ = w.rules[group]["TGT"]
        if not (tgt_lo <= base <= tgt_hi and tgt_lo <= target <= tgt_hi):
            w.feedback(f"起始基准与目标值必须在 {group}.TGT 允许范围 {tgt_lo:g} 至 {tgt_hi:g} 内", True)
            return False

        self.detect_channels(group=group)
        conflict, conflict_msg = self.has_role_conflict(group)
        if conflict and not confirm_role:
            w.feedback(conflict_msg, True)
            return False

        w.save_draft()
        self.group = group
        self.hold_target = hold_target
        self.finalize_mode = ""
        self.finalize_need_stop = False
        self.finalize_stop_acked = False
        self.finalize_target = None
        self.finalize_reason = ""
        self.finalize_summary = ""

        need_readback = (not w.sim) and (
            any(f"{group}.{k}" not in w.confirmed for k in KEYS)
            or any(abs(w.fields[k].value() - w.confirmed[f"{group}.{k}"]) > 1e-4 for k in KEYS)
        )
        if need_readback:
            self._pending_start = {
                "mode": mode, "style": style, "strategy": strategy,
                "base": base, "target": target, "step_seconds": step_seconds,
                "max_rounds": max_rounds, "output_limit": output_limit,
                "eval_brake": bool(eval_brake), "reverify_winner": bool(reverify_winner),
            }
            self.engine = None
            self.saved_before = None
            self.state = self.READ_BASELINE
            self.wall_deadline = monotonic() + 3.0
            self.sync_ui()
            self._set_status("正在启动前读回设备当前参数 (@GET)…")
            if not w.commands.submit([self._cmd("GET")], quiet=True):
                self._pending_start = None
                self.state = self.IDLE
                self.sync_ui()
                w.feedback("命令队列繁忙，无法在启动前读回设备参数", True)
                return False
            return True

        initial = {
            k: float(w.confirmed[f"{group}.{k}"]) if (not w.sim and f"{group}.{k}" in w.confirmed) else float(w.fields[k].value())
            for k in KEYS
        }
        return self._launch_with_baseline(
            initial, mode, style, strategy, base, target, step_seconds, max_rounds, output_limit,
            eval_brake=eval_brake, reverify_winner=reverify_winner,
        )

    def _launch_with_baseline(
        self, baseline, mode, style, strategy, base, target,
        step_seconds, max_rounds, output_limit,
        eval_brake=False, reverify_winner=False,
    ):
        w = self.ui
        group = self.group
        try:
            self.engine = AutoTuner(
                mode, style, strategy, baseline, w.rules[group], base, target,
                step_seconds, max_rounds, output_limit=output_limit,
                eval_brake=eval_brake, reverify_winner=reverify_winner,
            )
        except ValueError as exc:
            self.state = self.IDLE
            self.sync_ui()
            w.feedback(str(exc), True)
            return False

        if w.generator:
            w.stop_generator("已切换至自动整定", restore=False)
        w.capture.cancel_capture("自动整定运行中")
        if w.scope.frozen:
            w.toggle_freeze()

        self.actual_col, self.output_col = self.detect_channels(group=group)
        self.target_col = self._find_target_col(group=group)
        self.pitch_col = self._find_pitch_col() if group == "SPD" else None
        self.pitch_ref = float(w.confirmed.get("ANG.TGT", w.drafts.get("ANG", {}).get("TGT", 0.0)))
        self.eval_brake = bool(eval_brake)
        self.reverify_winner = bool(reverify_winner)
        self._start_seg = None
        self._climbing_extended = False
        self.saved_before = {k: float(baseline[k]) for k in KEYS}
        self.last_best_ref = None
        self.last_history = []
        self.last_history_group = group

        self.state = self.WAIT_START
        self.wall_deadline = monotonic() + 3.0
        self.sync_ui()
        w.log(
            "系统",
            f"自动整定启动  {group} · {MODES[mode]} · {STYLES[style]} · 基准 {base:g} → 目标 {target:g} · 最长 {max_rounds} 轮",
        )
        # Put initial base setpoint & start device
        cmds = [self._cmd("TGT", base), "@RUN"]
        if not w.commands.submit(cmds, quiet=True):
            self.stop("命令队列繁忙，无法启动自动整定", apply_best=False)
            return False
        return True

    def _cmd(self, key, value=None):
        prefix = "@" if self.ui.address.currentIndex() else f"@{self.group}."
        return f"{prefix}{key}" if value is None else f"{prefix}{key}={value:.4f}"

    def _build_param_cmds(self, kp, ki, kd, tgt, reset=True):
        """Prefer atomic @<GRP>.SET=kp,ki,kd,tgt when firmware advertises #CAP:SET."""
        if getattr(self.ui, "set_capable", False):
            prefix = "@" if self.ui.address.currentIndex() else f"@{self.group}."
            cmds = [f"{prefix}SET={kp:.4f},{ki:.4f},{kd:.4f},{tgt:.4f}"]
            if reset:
                cmds.append(self._cmd("RST"))
            return cmds
        cmds = [self._cmd("KP", kp), self._cmd("KI", ki), self._cmd("KD", kd)]
        if reset:
            cmds.append(self._cmd("RST"))
        cmds.append(self._cmd("TGT", tgt))
        return cmds

    def _begin_finalize(self, mode, params_dict, reason, stop_motor=False, already_stopped=False, summary=""):
        w = self.ui
        self.finalize_mode = mode
        self.finalize_need_stop = bool(stop_motor and not already_stopped)
        self.finalize_stop_acked = bool(already_stopped)
        self.finalize_target = dict(params_dict) if params_dict is not None else None
        self.finalize_reason = reason or ""
        self.finalize_summary = summary

        if not w.connected or params_dict is None:
            self.state = self.IDLE
            self.sync_ui()
            if self.finalize_need_stop:
                msg = f"⚠ {self.finalize_reason} · 连接已断开，停机与设备参数均未确认（请立即检查实车并重连后点【读取】）"
            else:
                msg = f"⚠ {self.finalize_reason} · 设备参数未确认（请重连后点【读取】核对）"
            self._set_status(msg, summary)
            w.log("系统", "自动整定  " + msg)
            w.feedback(msg, True)
            return

        if self.finalize_need_stop:
            w.send_heartbeat = False
            if getattr(w, "link", None) is not None and hasattr(w.link, "keepalive"):
                w.link.keepalive(False)
            w.commands.clear()
        cmds = (["@STOP"] if self.finalize_need_stop else []) + self._build_param_cmds(
            params_dict["KP"], params_dict["KI"], params_dict["KD"], params_dict["TGT"], reset=True,
        )
        self.state = self.FINALIZING
        self.finalize_deadline = monotonic() + 3.5
        self.sync_ui()
        self._set_status("正在等待设备确认停机与参数写回…" if self.finalize_need_stop else "正在等待设备确认参数写回…")
        if not w.commands.submit(cmds, quiet=True):
            self.on_command_failed("写回指令未能入队")
            return
        if not w.commands.busy():
            self._finish_finalize()

    def _finish_finalize(self):
        w = self.ui
        if self.state != self.FINALIZING:
            return
        mode = self.finalize_mode
        reason = self.finalize_reason
        summary = self.finalize_summary
        p = self.finalize_target
        self.state = self.IDLE
        self.sync_ui()
        if mode == "winner" and p is not None:
            status = f"✓ {reason}"
            self._set_status(status, summary)
            w.mark(f"整定完成 KP={p['KP']:.4g},KI={p['KI']:.4g},KD={p['KD']:.4g}")
            w.feedback(f"自动整定完成 · {summary}")
        else:
            if self.finalize_stop_acked or not self.finalize_need_stop:
                status = f"{reason}（设备已确认停机，原参数已恢复）"
            else:
                status = f"⚠ {reason}（设备停机未获确认，参数恢复未确认）"
            self._set_status(status, summary)
            w.log("系统", "自动整定  " + status)
            w.feedback(status, True)

    def _abort_finalize_disconnected(self):
        w = self.ui
        if self.finalize_need_stop and self.finalize_stop_acked:
            msg = f"⚠ {self.finalize_reason} · 设备已停机，参数恢复未确认（请重连后点【读取】核对）"
        elif self.finalize_need_stop:
            msg = f"⚠ {self.finalize_reason} · 连接已断开，停机与设备参数均未确认（请检查实车）"
        else:
            msg = f"⚠ {self.finalize_reason} · 连接已断开，设备参数未确认（请重连后点【读取】核对）"
        self.state = self.IDLE
        self.sync_ui()
        self._set_status(msg, self.finalize_summary)
        w.log("系统", "自动整定  " + msg)
        w.feedback(msg, True)

    def on_command_acked(self, cmd):
        if self.state == self.FINALIZING:
            if cmd == "@STOP":
                self.finalize_stop_acked = True
            if not self.ui.commands.busy():
                self._finish_finalize()

    def on_command_failed(self, msg):
        w = self.ui
        if self.state == self.READ_BASELINE:
            self._pending_start = None
            self.state = self.IDLE
            self.sync_ui()
            err = f"启动前未能读回设备参数（{msg}），整定已取消"
            self._set_status("⚠ " + err)
            w.feedback(err, True)
            return
        if self.state == self.FINALIZING:
            if self.finalize_need_stop and self.finalize_stop_acked:
                status = f"⚠ 设备已停机，参数恢复未确认（{msg}；请重连后点【读取】核对）"
            elif self.finalize_need_stop:
                status = f"⚠ 命令失败，停机与设备参数均未确认（{msg}；请检查实车）"
            elif self.finalize_mode == "winner":
                status = f"⚠ 最优参数写回未确认（{msg}；请点【读取】核对设备参数）"
            else:
                status = f"⚠ 原参数恢复未确认（{msg}；请重连后点【读取】核对）"
            self.state = self.IDLE
            self.sync_ui()
            self._set_status(status, self.finalize_summary)
            w.log("系统", "自动整定  " + status)
            w.feedback(status, True)
            return
        if self.active:
            self._abort_with_stop(f"命令失败（{msg}）")

    def _restore_original(self, reason, send_to_device=True, stop_motor=True, already_stopped=False, summary=""):
        """Always roll back UI and device to saved_before when no verified winner exists."""
        w = self.ui
        if self.saved_before:
            for k in KEYS:
                w.drafts[self.group][k] = self.saved_before[k]
            w.load_fields()
        msg = reason or "已中止整定"
        if send_to_device and w.connected and self.saved_before:
            self._begin_finalize("restore", self.saved_before, msg, stop_motor=stop_motor, already_stopped=already_stopped, summary=summary)
            return
        self.state = self.IDLE
        self.sync_ui()
        note = msg + (" · 未连机或无设备基准，设备参数未确认" if self.saved_before is None or not w.connected else "")
        self._set_status(note, summary)
        w.log("系统", "自动整定  " + note)
        w.feedback(note, True)

    def _abort_with_stop(self, reason):
        """Abort safely on abnormal runtime faults: always @STOP + restore original params, never drive target."""
        w = self.ui
        best_ref = ""
        if self.engine and self.engine.best_params and self.engine.best_eval and self.engine.best_eval.valid:
            p = self.engine.best_params
            ev = self.engine.best_eval
            self.last_best_ref = {
                "group": self.group,
                "KP": float(p["KP"]),
                "KI": float(p["KI"]),
                "KD": float(p["KD"]),
                "score": float(ev.score),
            }
            best_ref = (
                f"此前已测最优参考（未自动下发）：KP={p['KP']:.4g}  KI={p['KI']:.4g}  KD={p['KD']:.4g}"
                f"（J={ev.score:.3f}，超调 {ev.overshoot_pct:.1f}%）"
            )
            w.log("整定", f"{reason} · {best_ref}")
        self._restore_original(
            reason, send_to_device=w.connected, stop_motor=True, already_stopped=False, summary=best_ref,
        )

    def sync_ui(self):
        w = self.ui
        running = self.state != self.IDLE
        if not running and self.engine and self.engine.history:
            self.last_history = list(self.engine.history)
            self.last_history_group = self.group
        w.autotune_btn.setText("停止整定（采用最优）" if self.active else "等待设备确认…" if running else "⚡ 自动整定 PID…")
        w.autotune_btn.setObjectName("danger" if running else "accent")
        w.autotune_btn.style().unpolish(w.autotune_btn)
        w.autotune_btn.style().polish(w.autotune_btn)
        w.autotune_btn.setEnabled((w.connected or self.active) and self.state != self.FINALIZING)
        for k in KEYS:
            w.fields[k].setEnabled(not running and (k != "TGT" or w.generator is None))
        w.group.setEnabled(not running and w.generator is None and w.address.currentIndex() != 1 and not (w.sim and w.writer))
        w.gen_btn.setEnabled((w.connected or w.generator is not None) and not running)
        w.undo_btn.setEnabled(bool(w.undo_history) and w.connected and not running)
        w.autotune_status.setVisible(running or bool(w.autotune_status.text()))
        if hasattr(w, "autotune_draft_btn"):
            show_draft = (not running) and (self.last_best_ref is not None)
            w.autotune_draft_btn.setVisible(show_draft)
            w.autotune_draft_btn.setEnabled(show_draft)
        if hasattr(w, "autotune_history_btn"):
            show_hist = (not running) and bool(self.last_history)
            w.autotune_history_btn.setVisible(show_hist)
            w.autotune_history_btn.setEnabled(show_hist)
        if hasattr(w, "update_module_badges"):
            w.update_module_badges()

    def apply_best_ref_to_draft(self, params=None, group=None):
        """Copy candidate or preserved best-so-far parameters into UI drafts ONLY (never sends to device)."""
        ref = params if params is not None else self.last_best_ref
        if not ref:
            return False
        w = self.ui
        grp = group or ref.get("group") or self.group
        if grp not in LOOPS:
            return False
        if w.selected() != grp:
            if w.address.currentIndex() == 1:
                w.feedback(f"该参考参数属于 {grp}，请先将【寻址方式】切回“分组命令”后再填入草稿", True)
                return False
            w.group.setCurrentIndex(LOOPS.index(grp))
        if w.selected() != grp:
            w.feedback(f"当前无法切换至参数组 {grp}，已取消填入草稿", True)
            return False
        for k in ("KP", "KI", "KD"):
            if k in ref:
                w.drafts[grp][k] = float(ref[k])
        w.load_fields()
        w.feedback(f"已将参考参数填入 {grp} 草稿（未下发），核对后请点【发送本组】")
        return True

    def history_rows(self):
        """Return structured per-round records from the latest Auto-Tune session."""
        hist = list(self.engine.history) if (self.engine and self.engine.history) else list(self.last_history)
        if not hist:
            return []
        strict_gates = bool(
            self.engine and (
                self.engine.eval_brake
                or self.engine.output_limit is not None
                or any(getattr(ev, "max_pitch_dev", None) is not None for _r, _p, ev, _t in hist)
            )
        )
        best_idx = None
        best_key = (True, float("inf"))
        for idx, (_rnd, _p, ev, _tag) in enumerate(hist):
            if ev.valid:
                passed = getattr(ev, "passed_gates", True)
                if strict_gates and not passed:
                    continue
                key = (not passed, float(ev.score))
                if key < best_key:
                    best_key = key
                    best_idx = idx
        rows = []
        for idx, (rnd, p, ev, tag) in enumerate(hist):
            note = str(ev.note or "")
            if not note:
                if ev.valid and ev.brake_settle_s is not None:
                    rev_txt = f"{ev.brake_rev_peak:+.1f}" if ev.brake_rev_peak is not None else "--"
                    note = f"刹停 {ev.brake_settle_s:.2f}s / 反冲 {rev_txt}"
                else:
                    note = "有效" if ev.valid else "无效"
            rows.append({
                "round": int(rnd),
                "tag": str(tag),
                "KP": float(p.get("KP", 0.0)),
                "KI": float(p.get("KI", 0.0)),
                "KD": float(p.get("KD", 0.0)),
                "score": float(ev.score),
                "overshoot_pct": float(ev.overshoot_pct),
                "settle_s": ev.settle_s,
                "sat_pct": float(ev.sat_ratio * 100.0),
                "note": note,
                "valid": bool(ev.valid),
                "is_best": idx == best_idx,
            })
        return rows

    def show_history_dialog(self):
        """Open round-by-round Auto-Tune scores dialog with double-click fill-to-draft."""
        rows = self.history_rows()
        if not rows:
            self.ui.feedback("暂无自动整定打分记录", True)
            return
        w = self.ui
        dlg = W.QDialog(w)
        dlg.setWindowTitle(f"自动整定各轮打分明细 · {self.last_history_group}")
        dlg.resize(780, 380)
        lay = W.QVBoxLayout(dlg)
        tip = W.QLabel("绿色高亮为本次最佳有效轮次；选中某轮并点击下方按钮（或双击该行）可将其参数回填至右侧草稿（不自动下发）。")
        tip.setWordWrap(True)
        lay.addWidget(tip)

        headers = ["轮次", "阶段", "KP", "KI", "KD", "评分 J", "超调 %", "调节 s", "饱和 %", "说明"]
        tbl = W.QTableWidget(len(rows), len(headers))
        tbl.setHorizontalHeaderLabels(headers)
        tbl.setEditTriggers(W.QAbstractItemView.EditTrigger.NoEditTriggers)
        tbl.setSelectionBehavior(W.QAbstractItemView.SelectionBehavior.SelectRows)
        tbl.setSelectionMode(W.QAbstractItemView.SelectionMode.SingleSelection)
        tbl.verticalHeader().setVisible(False)
        for r_i, item in enumerate(rows):
            vals = [
                f"R{item['round']:02d}" + (" ★" if item["is_best"] else ""),
                item["tag"],
                f"{item['KP']:.4g}",
                f"{item['KI']:.4g}",
                f"{item['KD']:.4g}",
                f"{item['score']:.3f}" if item["valid"] else "无效",
                f"{item['overshoot_pct']:.1f}",
                f"{item['settle_s']:.2f}" if item["settle_s"] is not None else "未稳",
                f"{item['sat_pct']:.0f}",
                item["note"],
            ]
            for c_i, txt in enumerate(vals):
                cell = W.QTableWidgetItem(txt)
                if item["is_best"]:
                    cell.setBackground(QtGui.QColor("#d9f3ea" if not w.dark.isChecked() else "#1c4b3d"))
                tbl.setItem(r_i, c_i, cell)
        tbl.resizeColumnsToContents()
        tbl.horizontalHeader().setStretchLastSection(True)
        best_row = next((i for i, r in enumerate(rows) if r["is_best"]), 0)
        tbl.selectRow(best_row)
        lay.addWidget(tbl, 1)

        btn_row = W.QHBoxLayout()
        fill_btn = W.QPushButton("填入选中轮次到草稿（不下发）")
        fill_btn.setObjectName("primary")
        close_btn = W.QPushButton("关闭")
        btn_row.addStretch()
        btn_row.addWidget(fill_btn)
        btn_row.addWidget(close_btn)
        lay.addLayout(btn_row)

        def _fill_selected():
            cur = tbl.currentRow()
            if 0 <= cur < len(rows):
                self.apply_best_ref_to_draft(rows[cur], group=self.last_history_group)
                dlg.accept()

        fill_btn.clicked.connect(_fill_selected)
        tbl.itemDoubleClicked.connect(lambda _: _fill_selected())
        close_btn.clicked.connect(dlg.reject)
        dlg.exec()

    def _set_status(self, line1, line2=""):
        text = line1 if not line2 else f"{line1}\n{line2}"
        self.ui.autotune_status.setText(text)
        self.ui.autotune_status.show()

    def _best_brief(self):
        if not self.engine or not self.engine.best_params or not (self.engine.best_eval and self.engine.best_eval.valid):
            return "尚未产生有效基准"
        p = self.engine.best_params
        ev = self.engine.best_eval
        return f"当前最优：KP={p['KP']:.4g}  KI={p['KI']:.4g}  KD={p['KD']:.4g}（J={ev.score:.3f}，超调 {ev.overshoot_pct:.1f}%）"

    def _preserve_unqualified_ref(self):
        if not self.engine:
            return ""
        cand_p = self.engine.candidate_ref
        if cand_p is not None and self.engine._pre_reverify_best is not None:
            ref_ev = self.engine._pre_reverify_best[1]
        else:
            cand_p = self.engine.best_params
            ref_ev = self.engine.best_eval
        if cand_p is None or ref_ev is None or not ref_ev.valid:
            return ""
        self.last_best_ref = {
            "group": self.group,
            "KP": float(cand_p["KP"]),
            "KI": float(cand_p["KI"]),
            "KD": float(cand_p["KD"]),
            "score": float(ref_ev.score),
        }
        return (
            f"未自动写回参考：KP={cand_p['KP']:.4g}  KI={cand_p['KI']:.4g}  KD={cand_p['KD']:.4g}"
            f"（J={ref_ev.score:.3f}，超调 {ref_ev.overshoot_pct:.1f}%）"
        )

    def _reject_reason_for_no_winner(self, fallback):
        if self.engine and (self.engine.reverify_inconclusive or self.engine.kept_baseline):
            return self.engine.finish_reason or fallback
        if self.engine and self.engine.best_eval is not None and self.engine.best_eval.valid and not self.engine.best_eval.passed_gates:
            return f"无候选通过安全验收门槛（{self.engine.best_eval.note or '未达标'}），已停机并恢复原参数"
        return fallback

    def on_device_stopped(self, reason):
        """Called when device reports #STOP:fall / #STOP:heartbeat or user hits emergency stop."""
        if self.state == self.IDLE:
            return
        if self.state == self.READ_BASELINE:
            self._pending_start = None
            self.state = self.IDLE
            self.sync_ui()
            self._set_status(f"因“{reason}”中止（启动前设备已停机）")
            return
        w = self.ui
        if reason == "急停":
            self._restore_original("已响应急停", send_to_device=w.connected, stop_motor=False, already_stopped=True)
            return
        if self.state in (self.OBSERVE, self.WAIT_BRAKE_ACK, self.OBSERVE_BRAKE, self.SETTLE_BASE, self.WAIT_STEP_ACK, self.WAIT_BASE_ACK) and self.current_cand and self.engine:
            self.engine.record_result([], [], aborted_reason=reason)
        if self.engine and self.engine.has_qualified_winner():
            p = self.engine.best_params
            base_tgt = self.engine.base
            if self.saved_before:
                entry = {}
                for k, old_v in self.saved_before.items():
                    new_v = base_tgt if k == "TGT" else p[k]
                    if abs(new_v - old_v) > 1e-5:
                        addr = f"{self.group}.{k}"
                        wire_key = k if w.address.currentIndex() else addr
                        entry[addr] = (wire_key, old_v)
                if entry:
                    w.commands.batch += 1
                    w.undo_history.append((w.commands.batch, entry))
            for k in ("KP", "KI", "KD"):
                w.drafts[self.group][k] = p[k]
            w.drafts[self.group]["TGT"] = base_tgt
            w.load_fields()
            target_dict = {"KP": p["KP"], "KI": p["KI"], "KD": p["KD"], "TGT": base_tgt}
            self._begin_finalize(
                "winner", target_dict,
                f"因“{reason}”中止 · 已将安全最优组同步写回设备（目标保持 {base_tgt:g}）",
                stop_motor=False, already_stopped=True, summary=self._best_brief(),
            )
        else:
            ref_txt = self._preserve_unqualified_ref()
            self._restore_original(
                self._reject_reason_for_no_winner(f"因“{reason}”中止（无合格最优组）"),
                send_to_device=w.connected, stop_motor=False, already_stopped=True, summary=ref_txt,
            )

    def stop(self, message="", apply_best=True):
        w = self.ui
        if self.state == self.IDLE:
            return
        if self.state == self.READ_BASELINE:
            self._pending_start = None
            self.state = self.IDLE
            self.sync_ui()
            self._set_status(message or "整定已取消")
            return
        if self.state == self.FINALIZING:
            return
        has_best = bool(apply_best and self.engine is not None and self.engine.has_qualified_winner() and w.connected)
        if has_best:
            self._apply_winner(message or "已停止并应用最优参数")
            return
        ref_txt = self._preserve_unqualified_ref() if apply_best else ""
        msg = self._reject_reason_for_no_winner(message or "自动整定已停止") if apply_best else (message or "自动整定已停止")
        self._restore_original(msg, send_to_device=w.connected, stop_motor=True, already_stopped=False, summary=ref_txt)

    def _apply_winner(self, reason):
        w = self.ui
        best = self.engine.best_params
        final_tgt = self.engine.target if self.hold_target else self.engine.base
        # Record pre-autotune values into undo_history so "撤销上次下发" reverts in 1 click!
        if self.saved_before:
            entry = {}
            for k, old_v in self.saved_before.items():
                new_v = final_tgt if k == "TGT" else best[k]
                if abs(new_v - old_v) > 1e-5:
                    addr = f"{self.group}.{k}"
                    wire_key = k if w.address.currentIndex() else addr
                    entry[addr] = (wire_key, old_v)
            if entry:
                w.commands.batch += 1
                w.undo_history.append((w.commands.batch, entry))

        for k in ("KP", "KI", "KD"):
            w.drafts[self.group][k] = best[k]
        w.drafts[self.group]["TGT"] = final_tgt
        w.load_fields()

        ch_names = w.names()
        actual_desc = (
            f"CH{self.actual_col+1}({ch_names[self.actual_col]})"
            if self.actual_col < len(ch_names)
            else f"CH{self.actual_col+1}"
        )
        time_src = "内置仿真步长" if w.sim else w.time_mode.currentText()
        summary = f"{self.engine.summary_text()} · 反馈 {actual_desc} · 时间 {time_src}"
        winner_full = {"KP": best["KP"], "KI": best["KI"], "KD": best["KD"], "TGT": final_tgt}
        self.last_best_ref = None
        if self.engine and self.engine.history:
            self.last_history = list(self.engine.history)
            self.last_history_group = self.group
        self._begin_finalize("winner", winner_full, reason, stop_motor=False, already_stopped=True, summary=summary)

    def _check_missing_feedback(self, rows, phase_start, now):
        """Return (trailing_missing, missing_s, timed_out) for actual_col within rows >= phase_start."""
        stall_timeout = min(max(0.80, self.engine.step_seconds * 0.45), 1.50)
        wall_stale_s = max(0.0, now - self.last_valid_actual_wall) if self.last_valid_actual_wall is not None else 0.0
        if not len(rows):
            ref_t = self.last_valid_actual_t if self.last_valid_actual_t is not None else phase_start
            missing_s = max(0.0, self.ui.latest - ref_t)
            empty_timed_out = (self.state in (self.SETTLE_BASE, self.OBSERVE, self.OBSERVE_BRAKE)) and (wall_stale_s >= stall_timeout)
            return 0, max(missing_s, wall_stale_s), empty_timed_out
        y_col = rows[:, self.actual_col + 1]
        fin_idx = np.flatnonzero(np.isfinite(y_col))
        if len(fin_idx):
            last_fin_t = float(rows[int(fin_idx[-1]), 0])
            if self.last_valid_actual_t is None or last_fin_t > self.last_valid_actual_t:
                self.last_valid_actual_t = last_fin_t
                self.last_valid_actual_wall = now
                wall_stale_s = 0.0
            trailing_missing = len(rows) - 1 - int(fin_idx[-1])
        else:
            trailing_missing = len(rows)
        ref_t = self.last_valid_actual_t if self.last_valid_actual_t is not None else phase_start
        missing_s = max(0.0, self.ui.latest - ref_t)
        missing_timeout = min(max(0.50, self.engine.step_seconds * 0.45), 1.50)
        timed_out = (trailing_missing >= 2 and missing_s >= missing_timeout) or (wall_stale_s >= stall_timeout)
        return trailing_missing, max(missing_s, wall_stale_s), timed_out

    def tick(self, now):
        if self.state == self.IDLE:
            return
        w = self.ui
        if self.state == self.FINALIZING:
            if not w.connected:
                self._abort_finalize_disconnected()
                return
            if w.commands.busy():
                if now > self.finalize_deadline:
                    self.on_command_failed("写回确认超时")
                return
            self._finish_finalize()
            return

        if not w.connected:
            self.stop("连接已断开", apply_best=False)
            return

        if self.state == self.READ_BASELINE:
            if w.commands.busy():
                if now > self.wall_deadline:
                    self.on_command_failed("读取超时")
                return
            if not all(f"{self.group}.{k}" in w.confirmed for k in KEYS):
                self.on_command_failed("未返回完整 4 项参数")
                return
            baseline = {k: float(w.confirmed[f"{self.group}.{k}"]) for k in KEYS}
            for k in KEYS:
                w.drafts[self.group][k] = baseline[k]
            w.load_fields()
            ps = self._pending_start
            self._pending_start = None
            self._launch_with_baseline(
                baseline, ps["mode"], ps["style"], ps["strategy"],
                ps["base"], ps["target"], ps["step_seconds"], ps["max_rounds"], ps["output_limit"],
                eval_brake=ps.get("eval_brake", False), reverify_winner=ps.get("reverify_winner", False),
            )
            return

        if self.state == self.WAIT_START:
            if w.commands.busy():
                return
            if w.frames == 0:
                if now > self.wall_deadline:
                    self.stop("未收到设备上报波形，请检查串口数据输出", apply_best=False)
                return
            target_tags = ("target", "目标", "setpoint", "sp", "tgt", "ref")
            if not w.sim and 1 <= w.count <= 2 and not any(w.ref_active):
                roles_now = list(getattr(w, "channel_roles", []))[:w.count]
                names_lower = [n.lower() for n in w.names()[:w.count]]
                has_tgt = ("target" in roles_now) or any(any(t in nl for t in target_tags) for nl in names_lower)
                if not has_tgt:
                    w.follow[0] = self.group
                    w.set_reference(0, "整定目标", self.engine.target)
            self.actual_col, self.output_col = self.detect_channels(group=self.group)
            self.target_col = self._find_target_col(group=self.group)
            self.pitch_col = self._find_pitch_col() if self.group == "SPD" else None
            ch_names = w.names()
            actual_desc = ch_names[self.actual_col] if self.actual_col < len(ch_names) else f"CH{self.actual_col+1}"
            out_desc = f"CH{self.output_col+1}({ch_names[self.output_col]})" if self.output_col is not None and self.output_col < len(ch_names) else "无"
            self.step_t0_source = f"目标列 CH{self.target_col+1} 跳变" if self.target_col is not None else "估计起点(ACK)"
            w.log("整定", f"检测通道：实际反馈=CH{self.actual_col+1}({actual_desc}) · 输出参考={out_desc} · 起点={self.step_t0_source}")
            self.state = self.PREPARE

        if self.state == self.PREPARE:
            nxt = self.engine.next_candidate()
            if nxt is None:
                if self.engine.has_qualified_winner():
                    self._apply_winner(self.engine.finish_reason or "整定完成")
                else:
                    ref_txt = self._preserve_unqualified_ref()
                    self._restore_original(
                        self._reject_reason_for_no_winner(self.engine.finish_reason or "未获得有效整定结果"),
                        send_to_device=w.connected, stop_motor=True, already_stopped=False, summary=ref_txt,
                    )
                return
            rnd, cand, tag = nxt
            self.current_cand, self.current_tag = cand, tag
            self._start_seg = None
            self._climbing_extended = False
            for k in ("KP", "KI", "KD"):
                with C.QSignalBlocker(w.fields[k]):
                    w.fields[k].setValue(cand[k])
            cmds = self._build_param_cmds(cand["KP"], cand["KI"], cand["KD"], self.engine.base, reset=True)
            if not w.commands.submit(cmds, quiet=True):
                self._abort_with_stop("命令队列已满，整定中止")
                return
            self.state = self.WAIT_BASE_ACK
            self._set_status(
                f"第 {rnd}/{self.engine.max_rounds} 轮 · {tag} · 准备回到基准 {self.engine.base:g}",
                self._best_brief(),
            )
            return

        if self.state == self.WAIT_BASE_ACK:
            if w.commands.busy():
                return
            self.phase_t0 = w.latest
            self.last_valid_actual_t = self.phase_t0
            self.last_valid_actual_wall = now
            self.wall_deadline = now + max(4.5, self.engine.step_seconds * 2.5)
            self.state = self.SETTLE_BASE
            return

        if self.state == self.SETTLE_BASE:
            elapsed = w.latest - self.phase_t0
            settle_cap = min(2.5, max(0.70, self.engine.step_seconds * 0.70))
            max_settle_wait = min(4.0, max(1.4, self.engine.step_seconds * 1.25))
            a = w.buffer.view()
            fresh_rows = a[a[:, 0] >= self.phase_t0] if len(a) else a
            fresh_fin = (
                fresh_rows[np.isfinite(fresh_rows[:, self.actual_col + 1]), self.actual_col + 1]
                if len(fresh_rows)
                else np.array([], dtype=float)
            )
            trailing_missing, missing_s, fb_timed_out = self._check_missing_feedback(fresh_rows, self.phase_t0, now)
            if fb_timed_out:
                if len(fresh_fin) < 3:
                    self._abort_with_stop("基准段未收到有效反馈，已停机并取消整定")
                else:
                    self._abort_with_stop(
                        f"基准段反馈通道 CH{self.actual_col+1} 持续缺失有效数据（已优先停机，未下发阶跃目标）"
                    )
                return
            near_base = flat = False
            feedback_live = (trailing_missing == 0 and missing_s <= 0.25)
            if feedback_live and elapsed >= 0.18 and len(fresh_fin) >= 3:
                recent = a[(a[:, 0] >= self.phase_t0) & (a[:, 0] >= w.latest - 0.15), self.actual_col + 1]
                recent = recent[np.isfinite(recent)]
                if len(recent) < 3:
                    recent = fresh_fin[-3:]
                amp = abs(self.engine.target - self.engine.base)
                mean_y = float(np.mean(recent))
                near_base = abs(mean_y - self.engine.base) <= max(0.06 * amp, 1e-3)
                near_base_strict = abs(mean_y - self.engine.base) <= max(0.15 * amp, 1e-3)
                flat = float(np.ptp(recent)) <= max(0.03 * amp, 1e-3)

            require_near_base = (abs(self.engine.base) <= 1e-6) or (abs(self.current_cand.get("KI", 0.0)) > 1e-6)
            ready = feedback_live and (
                (near_base and flat)
                or (elapsed >= settle_cap and flat and (near_base_strict or not require_near_base))
            )
            if not ready:
                if elapsed >= max_settle_wait or now > self.wall_deadline:
                    if len(fresh_fin) < 3 or trailing_missing >= 2 or missing_s >= 0.30:
                        self._abort_with_stop("基准段未收到有效反馈，已停机并取消整定")
                        return
                    if self.engine.round == 1:
                        self._abort_with_stop("起始基准未平稳，请待车轮停稳后再启动自动整定")
                        return
                    self.engine.record_result([], [], aborted_reason="基准段未平稳，本轮不可比较")
                    w.log("整定", f"R{self.engine.round:02d} 跳过：上一轮结束后尚未平稳")
                    self.state = self.PREPARE
                    return
                return

            if len(a) and len(fresh_fin):
                tail = a[(a[:, 0] >= self.phase_t0) & (a[:, 0] >= w.latest - 0.12), self.actual_col + 1]
                tail = tail[np.isfinite(tail)]
                self.base_y = float(np.mean(tail)) if len(tail) else float(np.mean(fresh_fin[-3:]))
            else:
                self.base_y = self.engine.base

            self.step_pre_t = w.latest
            if not w.commands.submit([self._cmd("RST"), self._cmd("TGT", self.engine.target)], quiet=True):
                self._abort_with_stop("下发阶跃目标失败")
                return
            self.state = self.WAIT_STEP_ACK
            return

        if self.state == self.WAIT_STEP_ACK:
            if w.commands.busy():
                return
            self.step_t0 = w.latest
            self.step_t0_resolved = self.target_col is None
            self.step_t0_source = "目标列待跳变" if self.target_col is not None else "估计起点(ACK)"
            if self.target_col is not None:
                a = w.buffer.view()
                sub = a[a[:, 0] >= self.step_pre_t]
                if len(sub):
                    step_amp = self.engine.target - self.engine.base
                    tgt_vals = sub[:, self.target_col + 1]
                    hits = np.flatnonzero(
                        np.isfinite(tgt_vals) & ((tgt_vals - self.engine.base) * np.sign(step_amp) >= 0.4 * abs(step_amp))
                    )
                    if len(hits):
                        self.step_t0 = float(sub[int(hits[0]), 0])
                        self.step_t0_resolved = True
                        self.step_t0_source = "目标列跳变起点"
            self.last_valid_actual_t = self.step_t0
            self.last_valid_actual_wall = now
            self.wall_deadline = now + max(5.0, self.engine.step_seconds * 3.0)
            self.state = self.OBSERVE
            return

        if self.state == self.OBSERVE:
            if now > self.wall_deadline:
                self._abort_with_stop("阶跃采样中断（已优先停机，避免失反馈继续运行）")
                return
            a = w.buffer.view()
            if not self.step_t0_resolved and self.target_col is not None and len(a):
                sub = a[a[:, 0] >= self.step_pre_t]
                if len(sub):
                    step_amp = self.engine.target - self.engine.base
                    tgt_vals = sub[:, self.target_col + 1]
                    hits = np.flatnonzero(
                        np.isfinite(tgt_vals) & ((tgt_vals - self.engine.base) * np.sign(step_amp) >= 0.4 * abs(step_amp))
                    )
                    if len(hits):
                        self.step_t0 = float(sub[int(hits[0]), 0])
                        self.step_t0_resolved = True
                        self.step_t0_source = "目标列跳变起点"

            elapsed = max(0., w.latest - self.step_t0)
            rnd = self.engine.round
            self._set_status(
                f"第 {rnd}/{self.engine.max_rounds} 轮 · {self.current_tag} · 阶跃评估 {min(elapsed, self.engine.step_seconds):.1f}/{self.engine.step_seconds:.1f} s",
                self._best_brief(),
            )

            # Mid-round safety guard against sustained motor stall or strong reversed feedback
            seg_live = a[a[:, 0] >= self.step_t0] if len(a) else a
            _, _, fb_timed_out = self._check_missing_feedback(seg_live, self.step_t0, now)
            if fb_timed_out:
                self._abort_with_stop(
                    f"反馈通道 CH{self.actual_col+1} 持续缺失有效数据（已优先停机，避免失反馈继续运行）"
                )
                return
            grace_s = min(max(0.45, self.engine.step_seconds * 0.25), 1.5)
            step_delta = self.engine.target - self.base_y
            if len(seg_live) >= 8 and abs(step_delta) > 1e-3 and elapsed >= grace_s:
                t_live = seg_live[:, 0]
                y_live = seg_live[:, self.actual_col + 1]
                after_mask = (t_live - self.step_t0) >= grace_s * 0.6
                fin_mask = after_mask & np.isfinite(y_live)
                if np.sum(fin_mask) >= 6:
                    z_pg = (y_live[fin_mask] - self.base_y) / step_delta
                    if float(np.mean(z_pg)) < -0.20 and self.engine.round <= 2 and self.engine.best_params is None:
                        self.engine.record_result(t_live, y_live, base_y=self.base_y, outputs=(seg_live[:, self.output_col + 1] if self.output_col is not None else None))
                        self.stop("检测到反向响应（已停机，请检查反馈方向或参数正负号）", apply_best=False)
                        return
                    if self.output_col is not None and elapsed >= grace_s + 0.40:
                        u_live = seg_live[:, self.output_col + 1]
                        u_fin = u_live[fin_mask & np.isfinite(u_live)]
                        if len(u_fin) >= 6:
                            lim = self.engine.output_limit
                            if lim is not None and lim > 0:
                                sat_r = float(np.mean(np.abs(u_fin) >= 0.95 * lim))
                            else:
                                u_max = float(np.max(np.abs(u_fin)))
                                sat_r = float(np.mean(np.abs(u_fin) >= 0.985 * u_max)) if (u_max > 1e-3 and float(np.std(u_fin)) < 0.02 * u_max) else 0.0
                            if sat_r >= 0.85 and abs(float(np.mean(z_pg))) < 0.04:
                                self.engine.record_result(t_live, y_live, base_y=self.base_y, outputs=u_live)
                                self.stop("输出触顶且反馈无响应（已停机，请检查轮子受阻或编码器反馈）", apply_best=False)
                                return

            if elapsed < self.engine.step_seconds:
                return

            seg = seg_live
            if len(seg) < 6:
                if now <= self.wall_deadline - 1.0:
                    return
                self._abort_with_stop("采样率过低（单轮少于 6 帧），已停机中止")
                return

            times = seg[:, 0]
            actual = seg[:, self.actual_col + 1]
            outputs = seg[:, self.output_col + 1] if self.output_col is not None else None
            pitch_arr = seg[:, self.pitch_col + 1] if self.pitch_col is not None else None
            finite_y = actual[np.isfinite(actual)]
            if len(finite_y) < 6:
                if len(finite_y) == 0:
                    self._abort_with_stop(f"反馈通道 CH{self.actual_col+1} 无有效数据（已优先停机，避免失反馈继续运行）")
                else:
                    self._abort_with_stop(f"反馈通道 CH{self.actual_col+1} 有效采样不足（仅 {len(finite_y)} 帧），已停机中止")
                return
            if (
                self.current_tag in ("阶跃辨识", "放大辨识")
                and elapsed < self.engine.step_seconds * 2.5
                and is_step_still_climbing(times, actual, self.base_y)
            ):
                if not self._climbing_extended:
                    self._climbing_extended = True
                    self.wall_deadline = max(self.wall_deadline, now + self.engine.step_seconds * 1.6)
                    w.log("整定", "探针曲线仍在爬升，自动延长采样以待稳态")
                return

            step_amp = abs(self.engine.target - self.base_y)
            if len(finite_y) >= 6 and step_amp > 1e-3:
                check_y = finite_y[1:] if (len(finite_y) >= 8 and abs(float(finite_y[0]) - self.base_y) <= 0.05 * step_amp) else finite_y
                if float(np.max(np.abs(check_y - self.engine.target))) <= 0.02 * step_amp:
                    self.stop(
                        f"检测到反馈通道 CH{self.actual_col+1} 与目标值完全重合（疑似误选目标通道），已中止并恢复原参数，请在左侧『阶跃与游标』将实际通道改为实测反馈通道",
                        apply_best=False,
                    )
                    return

            if self.eval_brake:
                self._start_seg = (times.copy(), actual.copy(), outputs.copy() if outputs is not None else None, pitch_arr.copy() if pitch_arr is not None else None)
                if not w.commands.submit([self._cmd("TGT", self.engine.base)], quiet=True):
                    self._abort_with_stop("下发刹停目标失败")
                    return
                self.state = self.WAIT_BRAKE_ACK
                self._set_status(
                    f"第 {rnd}/{self.engine.max_rounds} 轮 · {self.current_tag} · 刹停归零至 {self.engine.base:g}",
                    self._best_brief(),
                )
                return

            prev_best = self.engine.best_eval.score if (self.engine.best_eval and self.engine.best_eval.valid) else float("inf")
            prev_gates = getattr(self.engine.best_eval, "passed_gates", False) if self.engine.best_eval else False
            ev = self.engine.record_result(
                times, actual, base_y=self.base_y, outputs=outputs,
                start_pitch=pitch_arr, pitch_ref=self.pitch_ref,
            )
            self._handle_recorded_eval(ev, prev_best, prev_gates)
            return

        if self.state == self.WAIT_BRAKE_ACK:
            if w.commands.busy():
                return
            self.brake_t0 = w.latest
            self.last_valid_actual_t = self.brake_t0
            self.last_valid_actual_wall = now
            self.wall_deadline = now + max(5.0, self.engine.brake_seconds * 2.5)
            self.state = self.OBSERVE_BRAKE
            return

        if self.state == self.OBSERVE_BRAKE:
            if now > self.wall_deadline:
                self._abort_with_stop("刹停采样中断（已优先停机，避免失反馈继续运行）")
                return
            a = w.buffer.view()
            seg_brk = a[a[:, 0] >= self.brake_t0] if len(a) else a
            _, _, fb_timed_out = self._check_missing_feedback(seg_brk, self.brake_t0, now)
            if fb_timed_out:
                self._abort_with_stop(
                    f"刹停段反馈通道 CH{self.actual_col+1} 持续缺失有效数据（已优先停机）"
                )
                return
            elapsed_brk = max(0., w.latest - self.brake_t0)
            rnd = self.engine.round
            self._set_status(
                f"第 {rnd}/{self.engine.max_rounds} 轮 · {self.current_tag} · 刹停评估 {min(elapsed_brk, self.engine.brake_seconds):.1f}/{self.engine.brake_seconds:.1f} s",
                self._best_brief(),
            )
            if elapsed_brk < self.engine.brake_seconds:
                return
            if len(seg_brk) < 6:
                if now <= self.wall_deadline - 1.0:
                    return
                self._abort_with_stop("刹停段采样率过低（少于 6 帧），已停机中止")
                return
            brk_t = seg_brk[:, 0]
            brk_y = seg_brk[:, self.actual_col + 1]
            brk_u = seg_brk[:, self.output_col + 1] if self.output_col is not None else None
            brk_p = seg_brk[:, self.pitch_col + 1] if self.pitch_col is not None else None
            start_t, start_y, start_u, start_p = self._start_seg
            prev_best = self.engine.best_eval.score if (self.engine.best_eval and self.engine.best_eval.valid) else float("inf")
            prev_gates = getattr(self.engine.best_eval, "passed_gates", False) if self.engine.best_eval else False
            ev = self.engine.record_result(
                start_t, start_y, base_y=self.base_y, outputs=start_u,
                brake_times=brk_t, brake_actual=brk_y, brake_outputs=brk_u,
                start_pitch=start_p, brake_pitch=brk_p, pitch_ref=self.pitch_ref,
            )
            self._handle_recorded_eval(ev, prev_best, prev_gates)

    def _handle_recorded_eval(self, ev, prev_best, prev_gates):
        w = self.ui
        rnd = self.engine.round
        cand = self.current_cand
        self.last_history = list(self.engine.history)
        self.last_history_group = self.group

        if self.eval_brake and ev.brake_settle_s is not None:
            settle_txt = f"{ev.brake_settle_s:.2f} s"
        elif self.eval_brake:
            settle_txt = "未刹稳"
        else:
            settle_txt = f"{ev.settle_s:.2f} s" if ev.settle_s is not None else "未稳"
        extra_brk = f"  反冲 {ev.brake_rev_peak:+.1f}" if (self.eval_brake and ev.brake_rev_peak is not None) else ""
        note_txt = f" [{ev.note}]" if ev.note else ""
        w.log(
            "整定",
            f"R{rnd:02d} {self.current_tag} ({self.step_t0_source})  KP={cand['KP']:.4g} KI={cand['KI']:.4g} KD={cand['KD']:.4g} → "
            f"J={ev.score:.3f}  超调 {ev.overshoot_pct:.1f}%  调节 {settle_txt}{extra_brk}{note_txt}",
        )
        improved_rank = ev.valid and (
            (ev.passed_gates and not prev_gates)
            or (ev.passed_gates == prev_gates and ev.score < prev_best)
        )
        if improved_rank:
            w.mark(f"最优#{rnd} J={ev.score:.2f}")
        if ev.stalled or (ev.reversed_sign and self.engine.finished and self.engine.best_params is None):
            self.stop(self.engine.finish_reason, apply_best=False)
            return
        if not ev.valid and ev.sample_count < 6:
            self._abort_with_stop(f"反馈通道 CH{self.actual_col+1} 有效采样不足（{ev.sample_count} 帧），已停机中止")
            return
        if self.engine.finished:
            if self.engine.has_qualified_winner():
                self._apply_winner(self.engine.finish_reason)
            else:
                ref_txt = self._preserve_unqualified_ref()
                self._restore_original(
                    self._reject_reason_for_no_winner(self.engine.finish_reason),
                    send_to_device=w.connected, stop_motor=True, already_stopped=False, summary=ref_txt,
                )
            return
        if ev.saturated and rnd <= 2:
            w.log("整定", "提示：输出通道接近满幅饱和，若始终达不到目标请适当降低目标值或检查输出限幅")
        self.state = self.PREPARE
