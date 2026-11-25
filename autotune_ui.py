"""Qt dialog and real-time closed-loop execution controller for PID Auto-Tune."""
from time import monotonic
import numpy as np
from PySide6 import QtCore as C, QtWidgets as W

from autotune import MODES, STYLES, STRATEGIES, AutoTuner
from engine import KEYS, LIMIT, MAX_CHANNELS
from layout_ui import combo, number, check


class AutoTuneController:
    """Drives AutoTuner over live serial or simulated telemetry inside Window.tick()."""

    IDLE, WAIT_START, PREPARE, WAIT_BASE_ACK, SETTLE_BASE, WAIT_STEP_ACK, OBSERVE, APPLY_BEST = range(8)

    def __init__(self, window):
        self.ui = window
        self.engine = None
        self.state = self.IDLE
        self.group = "ANG"
        self.actual_col = 1
        self.output_col = None
        self.hold_target = True
        self.saved_before = None
        self.current_cand = None
        self.current_tag = ""
        self.phase_t0 = 0.
        self.wall_deadline = 0.
        self.base_y = 0.
        self.step_t0 = 0.
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
        }

    @property
    def active(self):
        return self.state != self.IDLE

    def detect_channels(self):
        """Return (actual_channel_index, optional_output_channel_index)."""
        w = self.ui
        actual = w.actual_ch.currentData()
        if actual is None or actual >= MAX_CHANNELS:
            if w.sim or w.count >= 4:
                actual = 1
            elif w.count >= 1:
                actual = 0
            else:
                actual = 1 if w.source.currentIndex() == 1 else 0
        output = None
        if w.sim or w.count >= 3:
            output = 2 if actual != 2 else None
        elif w.count == 2:
            other = 1 - actual
            names = [n.lower() for n in w.names()[:w.count]]
            if w.right_channel == other or any(tag in names[other] for tag in ("pwm", "输出", "out", "duty", "占空")):
                output = other
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
        actual_ch, output_ch = self.detect_channels()
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
        default_tgt = cur_tgt if abs(cur_tgt) > 1e-3 else self.cfg["target"]
        target_spin = number(default_tgt, -LIMIT, LIMIT, 5.0, 3)
        base_spin = number(self.cfg["base"], -LIMIT, LIMIT, 5.0, 3)
        if abs(default_tgt - base_spin.value()) < 1e-3:
            base_spin.setValue(0.0 if abs(default_tgt) > 1e-3 else -10.0)

        default_mode = 2 if (w.sim and w.sim.pendulum) else 1 if w.sim and w.sim.model == 1 else self.cfg["mode"]
        is_ang = group == "ANG"
        default_strategy = 1 if (is_ang or (w.sim and w.sim.pendulum)) else self.cfg["strategy"]
        mode_box = combo(list(MODES), default_mode)
        style_box = combo(list(STYLES), self.cfg["style"])
        strat_box = combo(list(STRATEGIES), default_strategy)
        if is_ang:
            strat_box.setCurrentIndex(1)
            strat_box.setEnabled(False)
            strat_box.setToolTip("直立环 (ANG) 属于开环不稳定对象，为防止倒地飞车已强制锁定为局部微调")
        step_spin = number(self.cfg["step_s"], 0.5, 30.0, 0.5, 2)
        rounds_spin = W.QSpinBox()
        rounds_spin.setRange(3, 25)
        rounds_spin.setValue(int(self.cfg["rounds"]))
        hold_chk = check("整定完成后保持在目标值运行", self.cfg["hold"])
        safety_chk = check("我已确认：车轮已架空离地或已加装机械限位，且知晓随时可按 Esc 急停", False)

        form.addRow("目标值（如控速 100）", target_spin)
        form.addRow("起始基准值", base_spin)
        form.addRow("控制器类型", mode_box)
        form.addRow("响应偏好", style_box)
        form.addRow("整定策略", strat_box)
        form.addRow("单轮观察时长 s", step_spin)
        form.addRow("最大寻优轮数", rounds_spin)
        form.addRow(hold_chk)
        form.addRow(safety_chk)

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
        safety_chk.toggled.connect(ok_btn.setEnabled)
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
        )

    def start(self, mode, style, strategy, base, target, step_seconds=2.0, max_rounds=10, hold_target=True):
        w = self.ui
        if not w.connected:
            w.feedback("请先连接串口或打开仿真", True)
            return False
        group = w.selected()
        if group == "ANG" and strategy == 0:
            w.feedback("直立环 (ANG) 属于开环不稳定对象，仅允许使用【从当前参数局部微调】策略", True)
            return False
        tgt_lo, tgt_hi, _ = w.rules[group]["TGT"]
        if not (tgt_lo <= base <= tgt_hi and tgt_lo <= target <= tgt_hi):
            w.feedback(f"起始基准与目标值必须在 {group}.TGT 允许范围 {tgt_lo:g} 至 {tgt_hi:g} 内", True)
            return False

        w.save_draft()
        initial = {k: w.fields[k].value() for k in ("KP", "KI", "KD")}
        try:
            self.engine = AutoTuner(mode, style, strategy, initial, w.rules[group], base, target, step_seconds, max_rounds)
        except ValueError as exc:
            w.feedback(str(exc), True)
            return False

        if w.generator:
            w.stop_generator("已切换至自动整定", restore=False)
        w.capture.cancel_capture("自动整定运行中")
        if w.scope.frozen:
            w.toggle_freeze()

        self.group = group
        self.hold_target = hold_target
        self.actual_col, self.output_col = self.detect_channels()
        self.saved_before = {
            k: w.confirmed.get(f"{group}.{k}", w.fields[k].value()) for k in KEYS
        }
        # Ensure a target reference line or target channel is active so step overlay renders nicely
        if not w.sim and w.count <= 2 and not any(w.ref_active):
            w.follow[0] = group
            w.set_reference(0, "整定目标", target)

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

    def _restore_original(self, reason, send_to_device=True):
        """Always roll back UI and device to saved_before when no verified winner exists."""
        w = self.ui
        self.state = self.IDLE
        if self.saved_before:
            for k in KEYS:
                w.drafts[self.group][k] = self.saved_before[k]
            w.load_fields()
            if send_to_device and w.connected:
                p = self.saved_before
                cmds = self._build_param_cmds(p["KP"], p["KI"], p["KD"], p["TGT"], reset=True)
                w.commands.submit(cmds, quiet=True)
        self.sync_ui()
        msg = reason or "已中止整定并恢复整定前参数"
        self._set_status(msg)
        w.log("系统", "自动整定  " + msg)
        w.feedback(msg, True)

    def sync_ui(self):
        w = self.ui
        running = self.active
        w.autotune_btn.setText("停止整定（采用最优）" if running else "⚡ 自动整定 PID…")
        w.autotune_btn.setObjectName("danger" if running else "accent")
        w.autotune_btn.style().unpolish(w.autotune_btn)
        w.autotune_btn.style().polish(w.autotune_btn)
        w.autotune_btn.setEnabled(w.connected or running)
        for k in KEYS:
            w.fields[k].setEnabled(not running and (k != "TGT" or w.generator is None))
        w.group.setEnabled(not running and w.generator is None and w.address.currentIndex() != 1 and not (w.sim and w.writer))
        w.gen_btn.setEnabled((w.connected or w.generator is not None) and not running)
        w.undo_btn.setEnabled(bool(w.undo_history) and w.connected and not running)
        w.autotune_status.setVisible(running or bool(w.autotune_status.text()))

    def _set_status(self, line1, line2=""):
        text = line1 if not line2 else f"{line1}\n{line2}"
        self.ui.autotune_status.setText(text)
        self.ui.autotune_status.show()

    def _best_brief(self):
        if not self.engine or not self.engine.best_params:
            return "尚未产生有效基准"
        p = self.engine.best_params
        ev = self.engine.best_eval
        return f"当前最优：KP={p['KP']:.4g}  KI={p['KI']:.4g}  KD={p['KD']:.4g}（J={ev.score:.3f}，超调 {ev.overshoot_pct:.1f}%）"

    def on_device_stopped(self, reason):
        """Called when device reports #STOP:fall / #STOP:heartbeat or user hits emergency stop."""
        if not self.active:
            return
        w = self.ui
        if reason == "急停":
            self._restore_original("已响应急停 · 已恢复整定前参数至设备", send_to_device=w.connected)
            return
        if self.state in (self.OBSERVE, self.SETTLE_BASE, self.WAIT_STEP_ACK, self.WAIT_BASE_ACK) and self.current_cand:
            self.engine.record_result([], [], aborted_reason=reason)
        if self.engine and self.engine.best_params:
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
            self.state = self.IDLE
            self.sync_ui()
            if w.connected:
                w.commands.submit(self._build_param_cmds(p["KP"], p["KI"], p["KD"], base_tgt, reset=True), quiet=True)
            self._set_status(f"因“{reason}”中止 · 已将安全最优组同步写回设备", self._best_brief())
            w.log("系统", f"自动整定因“{reason}”中止，已将安全最优组写回设备（未启动电机）")
        else:
            self._restore_original(f"因“{reason}”中止（无可用最优组）· 已恢复整定前参数", send_to_device=w.connected)

    def stop(self, message="", apply_best=True):
        w = self.ui
        if not self.active:
            return
        has_best = apply_best and self.engine is not None and self.engine.best_params is not None and w.connected
        if has_best:
            self._apply_winner(message or "已停止并应用最优参数")
            return
        self._restore_original(message or "自动整定已停止，已恢复整定前参数", send_to_device=w.connected)

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

        cmds = self._build_param_cmds(best["KP"], best["KI"], best["KD"], final_tgt, reset=True)
        self.state = self.IDLE
        self.sync_ui()
        summary = self.engine.summary_text()
        self._set_status(f"✓ {reason}", summary)
        if w.connected:
            w.commands.submit(cmds, quiet=True)
            w.mark(f"整定完成 KP={best['KP']:.4g},KI={best['KI']:.4g},KD={best['KD']:.4g}")
        w.feedback(f"自动整定完成 · {summary}")

    def tick(self, now):
        if not self.active:
            return
        w = self.ui
        if not w.connected:
            self.stop("连接已断开，整定停止", apply_best=False)
            return

        if self.state == self.WAIT_START:
            if w.commands.busy():
                return
            if w.frames == 0:
                if now > self.wall_deadline:
                    self.stop("未收到设备上报波形，请检查串口数据输出", apply_best=False)
                return
            self.actual_col, self.output_col = self.detect_channels()
            self.state = self.PREPARE

        if self.state == self.PREPARE:
            nxt = self.engine.next_candidate()
            if nxt is None:
                self._apply_winner(self.engine.finish_reason or "整定完成")
                return
            rnd, cand, tag = nxt
            self.current_cand, self.current_tag = cand, tag
            for k in ("KP", "KI", "KD"):
                with C.QSignalBlocker(w.fields[k]):
                    w.fields[k].setValue(cand[k])
            cmds = self._build_param_cmds(cand["KP"], cand["KI"], cand["KD"], self.engine.base, reset=True)
            if not w.commands.submit(cmds, quiet=True):
                self.stop("命令队列已满，整定中止", apply_best=True)
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
            self.wall_deadline = now + max(4.0, self.engine.step_seconds * 2.5)
            self.state = self.SETTLE_BASE
            return

        if self.state == self.SETTLE_BASE:
            if now > self.wall_deadline:
                self.stop("等待通道采样超时", apply_best=True)
                return
            elapsed = w.latest - self.phase_t0
            settle_cap = min(2.0, max(0.55, self.engine.step_seconds * 0.65))
            ready = elapsed >= settle_cap
            a = w.buffer.view()
            if not ready and elapsed >= 0.18 and len(a) >= 6:
                recent = a[a[:, 0] >= w.latest - 0.15, self.actual_col + 1]
                recent = recent[np.isfinite(recent)]
                amp = abs(self.engine.target - self.engine.base)
                if len(recent) >= 3:
                    near_base = abs(float(np.mean(recent)) - self.engine.base) <= max(0.06 * amp, 1e-3)
                    flat = float(np.ptp(recent)) <= max(0.025 * amp, 1e-3)
                    ready = near_base and flat
            if not ready:
                return

            if len(a):
                tail = a[a[:, 0] >= w.latest - 0.12, self.actual_col + 1]
                tail = tail[np.isfinite(tail)]
                self.base_y = float(np.mean(tail)) if len(tail) else float(a[-1, self.actual_col + 1])
            else:
                self.base_y = self.engine.base

            if not w.commands.submit([self._cmd("RST"), self._cmd("TGT", self.engine.target)], quiet=True):
                self.stop("下发阶跃目标失败", apply_best=True)
                return
            self.state = self.WAIT_STEP_ACK
            return

        if self.state == self.WAIT_STEP_ACK:
            if w.commands.busy():
                return
            self.step_t0 = w.latest
            self.wall_deadline = now + max(5.0, self.engine.step_seconds * 3.0)
            self.state = self.OBSERVE
            return

        if self.state == self.OBSERVE:
            if now > self.wall_deadline:
                self.stop("阶跃采样中断，已采用已测最优参数", apply_best=True)
                return
            elapsed = max(0., w.latest - self.step_t0)
            rnd = self.engine.round
            self._set_status(
                f"第 {rnd}/{self.engine.max_rounds} 轮 · {self.current_tag} · 阶跃评估 {min(elapsed, self.engine.step_seconds):.1f}/{self.engine.step_seconds:.1f} s",
                self._best_brief(),
            )
            if elapsed < self.engine.step_seconds:
                return

            a = w.buffer.view()
            seg = a[a[:, 0] >= self.step_t0]
            if len(seg) < 6:
                if now <= self.wall_deadline - 1.0:
                    return
                self.stop("采样率过低（单轮少于 6 帧），无法可靠整定", apply_best=True)
                return

            times = seg[:, 0]
            actual = seg[:, self.actual_col + 1]
            outputs = seg[:, self.output_col + 1] if self.output_col is not None else None
            prev_best = self.engine.best_eval.score if self.engine.best_eval else float("inf")
            ev = self.engine.record_result(times, actual, base_y=self.base_y, outputs=outputs)

            cand = self.current_cand
            settle_txt = f"{ev.settle_s:.2f} s" if ev.settle_s is not None else "未稳"
            note_txt = f" [{ev.note}]" if ev.note else ""
            w.log(
                "整定",
                f"R{rnd:02d} {self.current_tag}  KP={cand['KP']:.4g} KI={cand['KI']:.4g} KD={cand['KD']:.4g} → "
                f"J={ev.score:.3f}  超调 {ev.overshoot_pct:.1f}%  调节 {settle_txt}{note_txt}",
            )
            if ev.score < prev_best:
                w.mark(f"最优#{rnd} J={ev.score:.2f}")
            if ev.saturated and rnd <= 2:
                w.log("整定", "提示：输出通道接近满幅饱和，若始终达不到目标请适当降低目标值或检查输出限幅")
            self.state = self.PREPARE
