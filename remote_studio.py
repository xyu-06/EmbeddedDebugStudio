"""Car Differential Remote Control (J,x,y) & HC-05 AT Configuration Toolbox (WorkbenchModule).

100% protocol-compatible desktop companion to https://github.com/xyu-06/BlueSerialStudio:
  - Supports the three mutually exclusive car control modes (360° single joystick, dual axis
    joysticks, and 4-key / WASD progressive hold-to-accelerate buttons) outputting `J,{x},{y}`.
  - Displays real-time left/right wheel differential mixing (`BlueSerial_MixWheels`).
  - Integrates the HC-05 compatible AT command builder (including verbatim `5760` baud entry,
    `38400,0,0` ORGL note, CRLF enforcement) with both Clipboard Copy AND direct USB-TTL UART TX/RX.
"""
from collections import deque
from datetime import datetime
import json
import math
from time import monotonic

from serial.tools import list_ports
from PySide6 import QtCore as C, QtGui, QtWidgets as W

from transport import SerialLink


# Baud rate table from BlueSerialStudio AtCommandBuilder.java + standard 57600 (keep 5760 explicitly flagged)
HC05_BAUD_RATES = (
    2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600, 1382400, 5760
)

HC05_CAUTIONS = (
    "1. AT 配置须通过电脑 USB-TTL 转串口连接 HC-05 的 UART 引脚（GND 共地、TXD/RXD 交叉）；普通蓝牙无线透传不等于 AT 模式。\n"
    "2. 进入 AT 模式：模块上电后红灯先快闪约 3 秒，在此 3 秒内长按板载 KEY 按键，红灯变为慢闪即进入 AT 模式（默认 38400,8,N,1）。\n"
    "3. 电平注意：底板通常标供电 3.6–6V，但串口 RXD 逻辑电平为 3.3V，请勿将 5V 供电误认为 RXD 可直接承受 5V。\n"
    "4. 指令格式：8 数据位、1 停止位、无校验、无流控，所有 AT 指令末尾必须带 CRLF（\\r\\n），本页发送与复制均自动附加 CRLF。\n"
    "5. 已补充常用标准速率 57600；兼容说明书原文中的 5760 已明确标注“疑似笔误”并设二次确认。若误设成难连接速率，可用 KEY 上电进 38400 AT 模式发送 AT+ORGL 恢复。\n"
    "6. AT+RESET 与 AT+ORGL 均会退出 AT 模式；执行 AT+ORGL 后串口恢复为 38400,0,0（与部分模块出厂预设的 9600 不同）。"
)


class AtCommandBuilder:
    """Port of com.embedded.blueserial.AtCommandBuilder with identical validation."""

    @staticmethod
    def _clean(raw: str, err_msg: str) -> str:
        if raw is None:
            raise ValueError(err_msg)
        if "\r" in raw or "\n" in raw:
            raise ValueError("参数中不能包含换行符")
        val = raw.strip()
        if not val:
            raise ValueError(err_msg)
        return val

    @classmethod
    def set_name(cls, name: str) -> str:
        return "AT+NAME=" + cls._clean(name, "蓝牙名称不能为空")

    @classmethod
    def set_pin(cls, pin: str) -> str:
        val = cls._clean(pin, "配对码不能为空")
        if "," in val:
            raise ValueError("配对码不能包含逗号")
        return "AT+PSWD=" + val

    @classmethod
    def set_role(cls, role: int) -> str:
        if role not in (0, 1, 2):
            raise ValueError("角色参数只能是 0、1 或 2")
        return f"AT+ROLE={role}"

    @classmethod
    def set_uart(cls, baud: int, stop_bits: int, parity: int) -> str:
        if baud not in HC05_BAUD_RATES:
            raise ValueError("波特率不在说明书列出的范围内")
        if stop_bits not in (0, 1):
            raise ValueError("停止位参数只能是 0 或 1")
        if parity not in (0, 1, 2):
            raise ValueError("校验位参数只能是 0、1 或 2")
        return f"AT+UART={baud},{stop_bits},{parity}"

    @classmethod
    def with_crlf(cls, command: str) -> str:
        val = cls._clean(command, "AT 指令不能为空")
        return val + "\r\n"


def held_speed(held_ms: float) -> int:
    """Port of DriveControlMath.heldSpeed: reaches 100 after 1600 ms hold."""
    safe = max(0, int(held_ms))
    return min(100, 18 + (safe * 82) // 1600)


def button_axis(pos: bool, pos_since_ms: float, neg: bool, neg_since_ms: float, now_ms: float) -> int:
    """Opposite directions held together cancel out cleanly."""
    if pos == neg:
        return 0
    return held_speed(now_ms - pos_since_ms) if pos else -held_speed(now_ms - neg_since_ms)


def clamp100(n: int) -> int:
    return -100 if n < -100 else (100 if n > 100 else int(n))


def mix_wheels(x: int, y: int) -> tuple[int, int]:
    """Port of BlueSerial_MixWheels in stm32_sample/blue_serial_drive_mixer.h."""
    x = clamp100(x)
    y = clamp100(y)
    if y == 0:
        return (x, -x)
    mag = -x if x < 0 else x
    inner = ((-y if y < 0 else y) * (100 - mag) + 50) // 100
    if x > 0:
        return (y, inner) if y > 0 else (-inner, y)
    if x < 0:
        return (inner, y) if y > 0 else (y, -inner)
    return (y, y)


def format_drive_command(fmt: str, steering_x: int, throttle_y: int) -> str:
    tpl = fmt.strip() or "J,{x},{y}"
    return tpl.replace("{x}", str(clamp100(steering_x))).replace("{y}", str(clamp100(throttle_y)))


class SingleJoystickWidget(W.QWidget):
    """360-degree spring-centering virtual joystick (-100..100 X steering, -100..100 Y throttle)."""

    changed = C.Signal(int, int, bool)  # x, y, released

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(195, 195)
        self.nx = 0.0
        self.ny = 0.0
        self.dragging = False
        self.dark = False

    def reset_center(self, emit_signal=False):
        self.nx = self.ny = 0.0
        self.dragging = False
        self.update()
        if emit_signal:
            self.changed.emit(0, 0, True)

    def _update_from_pos(self, pos: C.QPointF, released: bool):
        cx, cy = self.width() * 0.5, self.height() * 0.5
        max_r = max(24.0, min(cx, cy) - 24.0)
        dx = pos.x() - cx
        dy = cy - pos.y()
        dist = math.hypot(dx, dy)
        if dist > max_r:
            dx = dx / dist * max_r
            dy = dy / dist * max_r
        self.nx = dx / max_r
        self.ny = dy / max_r
        self.update()
        self.changed.emit(round(self.nx * 100), round(self.ny * 100), released)

    def mousePressEvent(self, event):
        if event.button() == C.Qt.MouseButton.LeftButton:
            self.dragging = True
            self._update_from_pos(event.position(), False)

    def mouseMoveEvent(self, event):
        if self.dragging:
            self._update_from_pos(event.position(), False)

    def mouseReleaseEvent(self, event):
        if self.dragging and event.button() == C.Qt.MouseButton.LeftButton:
            self.reset_center(emit_signal=True)

    def focusOutEvent(self, event):
        if self.dragging:
            self.reset_center(emit_signal=True)
        super().focusOutEvent(event)

    def hideEvent(self, event):
        if self.dragging:
            self.reset_center(emit_signal=True)
        super().hideEvent(event)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        cx, cy = self.width() * 0.5, self.height() * 0.5
        r = max(24.0, min(cx, cy) - 20.0)
        bg = QtGui.QColor("#232e3a" if self.dark else "#f0f5fa")
        ring = QtGui.QColor("#4c6278" if self.dark else "#bfd0e0")
        cross = QtGui.QColor("#394b5e" if self.dark else "#d6e2ec")
        p.setBrush(bg)
        p.setPen(QtGui.QPen(ring, 2))
        p.drawEllipse(C.QPointF(cx, cy), r, r)
        p.setPen(QtGui.QPen(cross, 1, C.Qt.PenStyle.DashLine))
        p.drawEllipse(C.QPointF(cx, cy), r * 0.5, r * 0.5)
        p.drawLine(C.QPointF(cx - r, cy), C.QPointF(cx + r, cy))
        p.drawLine(C.QPointF(cx, cy - r), C.QPointF(cx, cy + r))

        kx = cx + self.nx * (r - 6.0)
        ky = cy - self.ny * (r - 6.0)
        p.setPen(QtGui.QPen(QtGui.QColor("#225fc1"), 2.5))
        p.drawLine(C.QPointF(cx, cy), C.QPointF(kx, ky))
        knob_col = QtGui.QColor("#197a63" if self.dragging else "#225fc1")
        p.setBrush(knob_col)
        p.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
        p.drawEllipse(C.QPointF(kx, ky), 16.0, 16.0)
        p.end()


class AxisBarJoystick(W.QWidget):
    """1D vertical (throttle Y) or horizontal (steering X) spring-return slider joystick."""

    changed = C.Signal(int, bool)  # value (-100..100), released

    def __init__(self, vertical=True, parent=None):
        super().__init__(parent)
        self.vertical = vertical
        self.setMinimumSize(90 if vertical else 150, 155 if vertical else 70)
        self.norm = 0.0
        self.dragging = False
        self.dark = False

    def reset_center(self, emit_signal=False):
        self.norm = 0.0
        self.dragging = False
        self.update()
        if emit_signal:
            self.changed.emit(0, True)

    def _update_pos(self, pos: C.QPointF, released: bool):
        if self.vertical:
            cy = self.height() * 0.5
            span = max(20.0, cy - 20.0)
            self.norm = max(-1.0, min(1.0, (cy - pos.y()) / span))
        else:
            cx = self.width() * 0.5
            span = max(20.0, cx - 20.0)
            self.norm = max(-1.0, min(1.0, (pos.x() - cx) / span))
        self.update()
        self.changed.emit(round(self.norm * 100), released)

    def mousePressEvent(self, event):
        if event.button() == C.Qt.MouseButton.LeftButton:
            self.dragging = True
            self._update_pos(event.position(), False)

    def mouseMoveEvent(self, event):
        if self.dragging:
            self._update_pos(event.position(), False)

    def mouseReleaseEvent(self, event):
        if self.dragging and event.button() == C.Qt.MouseButton.LeftButton:
            self.reset_center(emit_signal=True)

    def focusOutEvent(self, event):
        if self.dragging:
            self.reset_center(emit_signal=True)
        super().focusOutEvent(event)

    def hideEvent(self, event):
        if self.dragging:
            self.reset_center(emit_signal=True)
        super().hideEvent(event)

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        w, h = self.width(), self.height()
        bg = QtGui.QColor("#232e3a" if self.dark else "#f0f5fa")
        bd = QtGui.QColor("#4c6278" if self.dark else "#bfd0e0")
        p.setBrush(bg)
        p.setPen(QtGui.QPen(bd, 1.8))
        if self.vertical:
            tw = 36
            p.drawRoundedRect(C.QRectF((w - tw) / 2, 10, tw, h - 20), 16, 16)
            cy = h * 0.5
            ky = cy - self.norm * (cy - 22.0)
            p.setPen(QtGui.QPen(QtGui.QColor("#8aa0b6"), 1.2, C.Qt.PenStyle.DashLine))
            p.drawLine(C.QPointF((w - tw) / 2 + 4, cy), C.QPointF((w + tw) / 2 - 4, cy))
            p.setBrush(QtGui.QColor("#197a63" if self.dragging else "#225fc1"))
            p.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
            p.drawEllipse(C.QPointF(w * 0.5, ky), 14, 14)
        else:
            th = 36
            p.drawRoundedRect(C.QRectF(10, (h - th) / 2, w - 20, th), 16, 16)
            cx = w * 0.5
            kx = cx + self.norm * (cx - 22.0)
            p.setPen(QtGui.QPen(QtGui.QColor("#8aa0b6"), 1.2, C.Qt.PenStyle.DashLine))
            p.drawLine(C.QPointF(cx, (h - th) / 2 + 4), C.QPointF(cx, (h + th) / 2 - 4))
            p.setBrush(QtGui.QColor("#197a63" if self.dragging else "#225fc1"))
            p.setPen(QtGui.QPen(QtGui.QColor("white"), 2))
            p.drawEllipse(C.QPointF(kx, h * 0.5), 14, 14)
        p.end()


class RemoteBtPage(W.QWidget):
    """4th Workbench Page: Car Remote Control (J,x,y) + HC-05 AT Configuration Toolbox."""

    module_id = "remote_bt"
    module_title = "🎮 遥控与 HC-05"

    def __init__(self, host=None):
        super().__init__(host)
        self.host = host
        self.link = None
        self.connected = False
        self.closing = False
        self.reconnect_opts = None
        self.rx_bytes = self.tx_bytes = 0
        self.steering_x = 0
        self.throttle_y = 0
        self.last_sent_xy = (0, 0)
        self.last_drive_tx = 0.0
        self.held_keys = {}  # dir_name -> monotonic_ms
        self.confirm_at = None
        self._pending_uart_write = None  # (cmd, (baud, stop, parity), deadline_mono)
        self.logs = deque(maxlen=300)
        self.rx_tail = ""

        self._build_ui()
        self.refresh_ports()
        self._sync_conn_ui()
        self._refresh_drive_readout()
        self._refresh_at_previews()

        self.timer = C.QTimer(self)
        self.timer.setInterval(35)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    # ---------- WorkbenchModule standard interface ----------
    def active_port(self):
        if (self.connected or self.link is not None) and self.reconnect_opts:
            return self.reconnect_opts.get("url")
        return None

    def release_port(self):
        if self.connected or self.link:
            self.disconnect_port()
            if self.link:
                self.link.join(0.5)
                if not self.link.is_alive():
                    self.link = None
                    self.closing = False
                    self._sync_conn_ui()

    def apply_theme(self, dark: bool):
        self.single_joy.dark = dark
        self.dual_y.dark = dark
        self.dual_x.dark = dark
        self.single_joy.update()
        self.dual_y.update()
        self.dual_x.update()
        bg = "#19212a" if dark else "#ffffff"
        fg = "#e4eaf1" if dark else "#1f2f40"
        bd = "#4a5868" if dark else "#d5dce3"
        self.console.setStyleSheet(
            f"QPlainTextEdit {{background:{bg};color:{fg};border:1px solid {bd};"
            "font:9.5pt 'Consolas','Microsoft YaHei UI';}}"
        )

    def save_state(self, settings: C.QSettings):
        st = {
            "port": self.port_box.currentText(),
            "baud": self.baud_box.currentText(),
            "mode": self.mode_box.currentIndex(),
            "format": self.fmt_edit.text(),
            "suffix": self.suffix_box.currentIndex(),
            "bt_name": self.at_name_edit.text(),
            "bt_pin": self.at_pin_edit.text(),
            "bt_baud_idx": self.at_baud_box.currentIndex(),
            "bt_stop_idx": self.at_stop_box.currentIndex(),
            "bt_parity_idx": self.at_parity_box.currentIndex(),
            "bt_role_idx": self.at_role_box.currentIndex(),
        }
        settings.setValue("remote_bt/state", json.dumps(st, ensure_ascii=False))

    def restore_state(self, settings: C.QSettings):
        try:
            raw = settings.value("remote_bt/state", "")
            if raw:
                st = json.loads(raw)
                if st.get("port"):
                    self.port_box.setCurrentText(str(st["port"]))
                if st.get("baud"):
                    self.baud_box.setCurrentText(str(st["baud"]))
                if "mode" in st and 0 <= int(st["mode"]) < 3:
                    self.mode_box.setCurrentIndex(int(st["mode"]))
                if st.get("format"):
                    self.fmt_edit.setText(str(st["format"])[:48])
                if "suffix" in st and 0 <= int(st["suffix"]) < 4:
                    self.suffix_box.setCurrentIndex(int(st["suffix"]))
                if st.get("bt_name"):
                    self.at_name_edit.setText(str(st["bt_name"])[:32])
                if st.get("bt_pin"):
                    self.at_pin_edit.setText(str(st["bt_pin"])[:16])
                for key, box in (
                    ("bt_baud_idx", self.at_baud_box),
                    ("bt_stop_idx", self.at_stop_box),
                    ("bt_parity_idx", self.at_parity_box),
                    ("bt_role_idx", self.at_role_box),
                ):
                    idx = st.get(key, box.currentIndex())
                    if isinstance(idx, int) and 0 <= idx < box.count():
                        box.setCurrentIndex(idx)
        except (ValueError, TypeError):
            pass
        self._refresh_at_previews()

    def shutdown(self):
        self.timer.stop()
        self.stop_all_motion(send_stop=False)
        if self.link:
            self.link.closed.set()
            self.link.join(0.6)
            self.link = None

    # ---------- UI construction ----------
    def _btn(self, text, fn, role=""):
        b = W.QPushButton(text)
        b.setCursor(C.Qt.CursorShape.PointingHandCursor)
        if role:
            b.setObjectName(role)
        b.clicked.connect(lambda _=False: fn())
        return b

    def _build_ui(self):
        self.setFocusPolicy(C.Qt.FocusPolicy.StrongFocus)
        root = W.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        # Top connection & safety toolbar
        top = W.QFrame()
        top.setObjectName("top")
        tbar = W.QHBoxLayout(top)
        tbar.setContentsMargins(10, 7, 10, 7)
        self.port_box = W.QComboBox(); self.port_box.setEditable(True); self.port_box.setMinimumWidth(105)
        self.ref_btn = self._btn("刷新", self.refresh_ports)
        self.baud_box = W.QComboBox(); self.baud_box.setEditable(True)
        self.baud_box.addItems(["9600", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_box.setCurrentText("9600")
        self.conn_btn = self._btn("打开串口", self.toggle_port, "primary")
        self.at_baud_quick = self._btn("切 38400 (HC-05 AT默认)", lambda: self.baud_box.setCurrentText("38400"))
        self.estop_btn = self._btn("紧急停车 (J,0,0 / S)", self.emergency_stop, "danger")
        self.badge = W.QLabel("● 未连接")
        self.badge.setStyleSheet("color:#71808b;font-weight:600;padding-left:6px;")

        tbar.addWidget(W.QLabel("端口:"))
        tbar.addWidget(self.port_box)
        tbar.addWidget(self.ref_btn)
        tbar.addWidget(W.QLabel("波特率:"))
        tbar.addWidget(self.baud_box)
        tbar.addWidget(self.conn_btn)
        tbar.addWidget(self.at_baud_quick)
        tbar.addWidget(self._btn("⚠ AT 必读说明", self.show_cautions_dialog))
        rem_iface_btn = self._btn("🧩 遥控接口与说明", self.open_remote_interface_builder)
        rem_iface_btn.setToolTip("跳转至【🧩 接口构建】统一选择 STM32 HAL / 标准库 SPL / ESP32·Arduino，复制或导出差速混控代码及查看配置说明")
        tbar.addWidget(rem_iface_btn)
        tbar.addWidget(self.badge)
        tbar.addStretch()
        tbar.addWidget(self.estop_btn)
        root.addWidget(top)

        vsplit = W.QSplitter(C.Qt.Orientation.Vertical)
        root.addWidget(vsplit, 1)

        upper = W.QSplitter(C.Qt.Orientation.Horizontal)
        vsplit.addWidget(upper)

        # ================= LEFT: Car Remote Control (BlueSerialStudio J,x,y) =================
        drive_card = W.QFrame()
        drive_card.setObjectName("card")
        dv = W.QVBoxLayout(drive_card)
        dv.setContentsMargins(10, 8, 10, 8)
        dv.setSpacing(6)
        dv.addWidget(W.QLabel("<b>✥ 小车差速遥控台（兼容 BlueSerialStudio 协议）</b>"))

        cfg_row = W.QHBoxLayout()
        self.mode_box = W.QComboBox()
        self.mode_box.addItems(["360° 单摇杆（鼠标拖拽）", "双摇杆（左油门 / 右转向）", "四按键 / 键盘 WASD 长按加速"])
        self.mode_box.currentIndexChanged.connect(self._switch_drive_mode)
        cfg_row.addWidget(W.QLabel("控制模式:"))
        cfg_row.addWidget(self.mode_box, 1)
        dv.addLayout(cfg_row)

        fmt_row = W.QHBoxLayout()
        self.fmt_edit = W.QLineEdit("J,{x},{y}")
        self.fmt_edit.setToolTip("与 BlueSerialStudio 一致：{x} 为转向(-100..100)，{y} 为前后速度(-100..100)")
        self.suffix_box = W.QComboBox()
        self.suffix_box.addItems(["无结束符", "追加 \\n (默认)", "追加 \\r\\n", "追加 \\r"])
        self.suffix_box.setCurrentIndex(1)
        fmt_row.addWidget(W.QLabel("数据包模板:"))
        fmt_row.addWidget(self.fmt_edit, 1)
        fmt_row.addWidget(self.suffix_box)
        dv.addLayout(fmt_row)

        self.drive_stack = W.QStackedWidget()
        # Mode 0: 360 single joystick
        p0 = W.QWidget(); p0_l = W.QVBoxLayout(p0); p0_l.setContentsMargins(0, 2, 0, 2)
        h0 = W.QLabel("向上前进、向下后退、左右转向；松开鼠标自动回中归零。")
        h0.setStyleSheet("color:#5b7083;font-size:9pt;")
        p0_l.addWidget(h0)
        self.single_joy = SingleJoystickWidget()
        self.single_joy.changed.connect(self._on_single_joy)
        p0_l.addWidget(self.single_joy, 1)
        self.drive_stack.addWidget(p0)

        # Mode 1: Dual axis joysticks
        p1 = W.QWidget(); p1_l = W.QHBoxLayout(p1); p1_l.setContentsMargins(4, 6, 4, 6)
        c_left = W.QVBoxLayout(); c_right = W.QVBoxLayout()
        c_left.addWidget(W.QLabel("<b>左 · 前后速度 Y</b>"), alignment=C.Qt.AlignmentFlag.AlignCenter)
        self.dual_y = AxisBarJoystick(vertical=True)
        self.dual_y.changed.connect(self._on_dual_y)
        c_left.addWidget(self.dual_y, 1)
        c_right.addWidget(W.QLabel("<b>右 · 左右转向 X</b>"), alignment=C.Qt.AlignmentFlag.AlignCenter)
        self.dual_x = AxisBarJoystick(vertical=False)
        self.dual_x.changed.connect(self._on_dual_x)
        c_right.addWidget(self.dual_x, 1)
        p1_l.addLayout(c_left, 1); p1_l.addLayout(c_right, 1)
        self.drive_stack.addWidget(p1)

        # Mode 2: 4-button progressive hold + WASD keys
        p2 = W.QWidget(); p2_l = W.QVBoxLayout(p2); p2_l.setContentsMargins(6, 6, 6, 6)
        h2 = W.QLabel("长按按钮或按住键盘 W/A/S/D（方向键）：18% 起步，1.6 秒渐进加速至 100%，松开即归零。")
        h2.setWordWrap(True); h2.setStyleSheet("color:#5b7083;font-size:9pt;")
        p2_l.addWidget(h2)
        grid_b = W.QGridLayout()
        self.hold_btns = {}
        for d_key, title, r_i, c_i in (
            ("up", "▲ 前进 (W)", 0, 1),
            ("left", "◀ 左转 (A)", 1, 0),
            ("stop", "■ 停车 (Space)", 1, 1),
            ("right", "▶ 右转 (D)", 1, 2),
            ("down", "▼ 后退 (S)", 2, 1),
        ):
            b = W.QPushButton(title)
            b.setMinimumHeight(46)
            if d_key == "stop":
                b.setObjectName("danger")
                b.clicked.connect(self.emergency_stop)
            else:
                b.pressed.connect(lambda k=d_key: self._dir_pressed(k))
                b.released.connect(lambda k=d_key: self._dir_released(k))
            grid_b.addWidget(b, r_i, c_i)
            self.hold_btns[d_key] = b
        p2_l.addLayout(grid_b)
        p2_l.addStretch()
        self.drive_stack.addWidget(p2)

        dv.addWidget(self.drive_stack, 1)

        self.mix_label = W.QLabel()
        self.mix_label.setStyleSheet("font:10pt 'Consolas','Microsoft YaHei UI';padding:4px 6px;border-radius:4px;")
        dv.addWidget(self.mix_label)

        # Quick car auxiliary commands from blue_serial_protocol.c
        q_row = W.QHBoxLayout()
        for lbl, cmd in (("开灯 LED_ON", "LED_ON"), ("关灯 LED_OFF", "LED_OFF"), ("鸣笛 BEEP", "BEEP"), ("舵机归中", "SERVO,90")):
            q_row.addWidget(self._btn(lbl, lambda c=cmd: self.send_car_text(c)))
        dv.addLayout(q_row)
        upper.addWidget(drive_card)

        # ================= RIGHT: HC-05 AT Command Toolbox =================
        at_card = W.QFrame()
        at_card.setObjectName("card")
        av = W.QVBoxLayout(at_card)
        av.setContentsMargins(10, 8, 10, 8)
        av.setSpacing(6)
        av.addWidget(W.QLabel("<b>📡 HC-05 蓝牙模块 AT 配置工具箱（自动附加 CRLF · 支持 USB-TTL 直发与复制）</b>"))

        tip = W.QLabel(
            "提示：将 HC-05 插上 USB-TTL（按住 KEY 上电进慢闪 AT 模式，波特率选 38400），打开串口即可直接点击【发送】配置并查看下方 OK 应答；未连串口时点【复制】也会附加 \\r\\n。"
        )
        tip.setWordWrap(True)
        tip.setStyleSheet("color:#197a63;font-size:9pt;")
        av.addWidget(tip)

        # Parameterized AT rows
        p_grid = W.QGridLayout()
        p_grid.setHorizontalSpacing(6)
        p_grid.setVerticalSpacing(6)

        # 1. Name
        self.at_name_edit = W.QLineEdit("BlueCar")
        self.at_name_prev = W.QLabel()
        self.at_name_edit.textChanged.connect(self._refresh_at_previews)
        p_grid.addWidget(W.QLabel("蓝牙名称:"), 0, 0)
        p_grid.addWidget(self.at_name_edit, 0, 1)
        p_grid.addWidget(self.at_name_prev, 0, 2)
        p_grid.addWidget(self._btn("查名称", lambda: self.exec_at("AT+NAME?")), 0, 3)
        p_grid.addWidget(self._btn("发送修改", self._apply_at_name, "primary"), 0, 4)
        p_grid.addWidget(self._btn("复制", lambda: self._apply_at_name(copy_only=True)), 0, 5)

        # 2. UART
        uart_w = W.QWidget(); uw = W.QHBoxLayout(uart_w); uw.setContentsMargins(0, 0, 0, 0); uw.setSpacing(4)
        self.at_baud_box = W.QComboBox()
        for b in HC05_BAUD_RATES:
            label = f"{b}（说明书原文，疑似笔误）" if b == 5760 else str(b)
            self.at_baud_box.addItem(label, b)
        self.at_baud_box.setCurrentIndex(self.at_baud_box.findData(115200))
        self.at_stop_box = W.QComboBox(); self.at_stop_box.addItems(["0 (1停止位)", "1 (2停止位)"])
        self.at_parity_box = W.QComboBox(); self.at_parity_box.addItems(["0 (无校验)", "1 (奇校验)", "2 (偶校验)"])
        for box in (self.at_baud_box, self.at_stop_box, self.at_parity_box):
            box.currentIndexChanged.connect(self._refresh_at_previews)
            uw.addWidget(box)
        self.at_uart_prev = W.QLabel()
        p_grid.addWidget(W.QLabel("串口参数:"), 1, 0)
        p_grid.addWidget(uart_w, 1, 1)
        p_grid.addWidget(self.at_uart_prev, 1, 2)
        p_grid.addWidget(self._btn("查串口", lambda: self.exec_at("AT+UART?")), 1, 3)
        p_grid.addWidget(self._btn("发送修改", self._apply_at_uart, "primary"), 1, 4)
        p_grid.addWidget(self._btn("复制", lambda: self._apply_at_uart(copy_only=True)), 1, 5)

        # 3. PIN
        self.at_pin_edit = W.QLineEdit("1234")
        self.at_pin_prev = W.QLabel()
        self.at_pin_edit.textChanged.connect(self._refresh_at_previews)
        p_grid.addWidget(W.QLabel("配对密码:"), 2, 0)
        p_grid.addWidget(self.at_pin_edit, 2, 1)
        p_grid.addWidget(self.at_pin_prev, 2, 2)
        p_grid.addWidget(self._btn("查密码", lambda: self.exec_at("AT+PSWD?")), 2, 3)
        p_grid.addWidget(self._btn("发送修改", self._apply_at_pin, "primary"), 2, 4)
        p_grid.addWidget(self._btn("复制", lambda: self._apply_at_pin(copy_only=True)), 2, 5)

        # 4. Role
        self.at_role_box = W.QComboBox()
        self.at_role_box.addItems(["0 · 从机 Slave（默认）", "1 · 主机 Master", "2 · 回环 Slave-Loop"])
        self.at_role_box.currentIndexChanged.connect(self._refresh_at_previews)
        self.at_role_prev = W.QLabel()
        p_grid.addWidget(W.QLabel("模块角色:"), 3, 0)
        p_grid.addWidget(self.at_role_box, 3, 1)
        p_grid.addWidget(self.at_role_prev, 3, 2)
        p_grid.addWidget(self._btn("查角色", lambda: self.exec_at("AT+ROLE?")), 3, 3)
        p_grid.addWidget(self._btn("发送修改", self._apply_at_role, "primary"), 3, 4)
        p_grid.addWidget(self._btn("复制", lambda: self._apply_at_role(copy_only=True)), 3, 5)

        for prev in (self.at_name_prev, self.at_uart_prev, self.at_pin_prev, self.at_role_prev):
            prev.setStyleSheet("font:9.5pt 'Consolas';color:#1c5f4e;")
        av.addLayout(p_grid)

        av.addWidget(W.QLabel("<b>常用查询与管理 AT 指令（点击即发并复制 CRLF）：</b>"))
        q_at = W.QGridLayout()
        q_at.setSpacing(5)
        quick_at_items = [
            ("通信测试 AT", "AT"),
            ("查版本 AT+VERSION?", "AT+VERSION?"),
            ("查地址 AT+ADDR?", "AT+ADDR?"),
            ("查状态 AT+STATE?", "AT+STATE?"),
            ("查名称 AT+NAME?", "AT+NAME?"),
            ("查波特率 AT+UART?", "AT+UART?"),
            ("清空配对 AT+RMAAD", "AT+RMAAD"),
            ("重启模块 AT+RESET", "AT+RESET"),
            ("恢复默认 AT+ORGL (38400)", "AT+ORGL"),
        ]
        for idx, (label, cmd) in enumerate(quick_at_items):
            q_at.addWidget(self._btn(label, lambda c=cmd: self.exec_at(c)), idx // 3, idx % 3)
        av.addLayout(q_at)
        av.addStretch()
        upper.addWidget(at_card)
        upper.setStretchFactor(0, 5)
        upper.setStretchFactor(1, 7)
        upper.setSizes([500, 760])

        # ================= BOTTOM: Live AT & Remote Serial Console =================
        bot = W.QWidget()
        bv = W.QVBoxLayout(bot)
        bv.setContentsMargins(8, 4, 8, 4)
        bv.setSpacing(4)
        bh = W.QHBoxLayout()
        bh.addWidget(W.QLabel("<b>实时收发回显（AT 应答 OK / 遥控报文日志）</b>"))
        bh.addStretch()
        bh.addWidget(self._btn("清空日志", self.clear_console))
        bv.addLayout(bh)
        self.console = W.QPlainTextEdit()
        self.console.setReadOnly(True)
        self.console.setMaximumBlockCount(300)
        bv.addWidget(self.console, 1)
        vsplit.addWidget(bot)
        vsplit.setSizes([520, 190])

        self.status_bar = W.QLabel()
        self.status_bar.setObjectName("status")
        root.addWidget(self.status_bar)
        self._update_status_bar()

    # ---------- Port handling ----------
    def refresh_ports(self):
        old = self.port_box.currentText()
        self.port_box.clear()
        try:
            for p in sorted(list_ports.comports(), key=lambda x: x.device):
                self.port_box.addItem(p.device)
        except OSError:
            pass
        if old:
            self.port_box.setCurrentText(old)

    def toggle_port(self):
        if self.connected or self.link is not None:
            self.disconnect_port()
            return
        name = self.port_box.currentText().strip()
        if not name:
            self._log("系统", "请先选择或输入 USB-TTL / 蓝牙虚拟串口名称（如 COM5）")
            return
        if self.host is not None and hasattr(self.host, "request_port_access"):
            if not self.host.request_port_access(self, name):
                return
        try:
            baud = int(self.baud_box.currentText().strip())
            if not 300 <= baud <= 12_000_000:
                raise ValueError("波特率无效")
            opts = dict(url=name, baudrate=baud, bytesize=8, parity="N", stopbits=1.0)
        except ValueError as exc:
            self._log("系统", str(exc))
            return
        self.reconnect_opts = opts
        self.closing = False
        self.rx_tail = ""
        self.link = SerialLink(opts)
        self.link.start()
        self._sync_conn_ui()

    def disconnect_port(self):
        had_conn = self.connected and self.link is not None
        self.stop_all_motion(send_stop=False)
        self._pending_uart_write = None
        if self.link:
            self.closing = True
            if had_conn:
                suf = self._suffix_str()
                stop_bytes = (format_drive_command(self.fmt_edit.text(), 0, 0) + suf + "S" + suf).encode("utf-8")
                self.link.flush_and_close(stop_bytes, timeout=0.25)
            else:
                self.link.closed.set()
        self.connected = False
        self._sync_conn_ui()
        self._log("系统", "已安全停车并关闭串口连接" if had_conn else "已关闭串口连接")

    def _sync_conn_ui(self):
        busy = self.connected or self.link is not None
        for w in (self.port_box, self.ref_btn, self.baud_box, self.at_baud_quick):
            w.setEnabled(not busy)
        self.conn_btn.setText("关闭串口" if self.connected else "正在打开…" if busy else "打开串口")
        self.conn_btn.setObjectName("danger" if self.connected else "primary")
        self.conn_btn.style().unpolish(self.conn_btn)
        self.conn_btn.style().polish(self.conn_btn)
        if self.connected:
            url = self.reconnect_opts["url"] if self.reconnect_opts else self.port_box.currentText()
            self.badge.setText(f"● 已连接 {url} @ {self.baud_box.currentText()}")
            self.badge.setStyleSheet("color:#188467;font-weight:600;padding-left:6px;")
        else:
            self.badge.setText("● 未连接")
            self.badge.setStyleSheet("color:#71808b;font-weight:600;padding-left:6px;")
        if self.host is not None and hasattr(self.host, "update_module_badges"):
            self.host.update_module_badges()
        self._update_status_bar()

    # ---------- Drive modes & keyboard WASD ----------
    def _switch_drive_mode(self, idx: int):
        self.stop_all_motion(send_stop=self.connected)
        self.drive_stack.setCurrentIndex(idx)

    def _on_single_joy(self, x: int, y: int, released: bool):
        self.steering_x, self.throttle_y = x, y
        self._emit_drive(force=released)

    def _on_dual_y(self, y: int, released: bool):
        self.throttle_y = y
        self._emit_drive(force=released)

    def _on_dual_x(self, x: int, released: bool):
        self.steering_x = x
        self._emit_drive(force=released)

    def _dir_pressed(self, direction: str):
        if direction not in self.held_keys:
            self.held_keys[direction] = monotonic() * 1000.0
            self._update_held_axes()

    def _dir_released(self, direction: str):
        if direction in self.held_keys:
            del self.held_keys[direction]
            self._update_held_axes(force_zero=not self.held_keys)

    def _update_held_axes(self, force_zero=False):
        now_ms = monotonic() * 1000.0
        self.throttle_y = button_axis(
            "up" in self.held_keys, self.held_keys.get("up", now_ms),
            "down" in self.held_keys, self.held_keys.get("down", now_ms),
            now_ms,
        )
        self.steering_x = button_axis(
            "right" in self.held_keys, self.held_keys.get("right", now_ms),
            "left" in self.held_keys, self.held_keys.get("left", now_ms),
            now_ms,
        )
        self._emit_drive(force=force_zero or (self.steering_x == 0 and self.throttle_y == 0))

    def hideEvent(self, event):
        if self.held_keys or self.steering_x or self.throttle_y:
            self.stop_all_motion(send_stop=self.connected)
        super().hideEvent(event)

    def focusOutEvent(self, event):
        focus_now = W.QApplication.focusWidget()
        if (self.held_keys or self.steering_x or self.throttle_y) and (focus_now is None or not self.isAncestorOf(focus_now)):
            self.stop_all_motion(send_stop=self.connected)
        super().focusOutEvent(event)

    def changeEvent(self, event):
        if event.type() in (C.QEvent.Type.ActivationChange, C.QEvent.Type.WindowStateChange):
            win = self.window()
            if (self.held_keys or self.steering_x or self.throttle_y) and (win is None or not win.isActiveWindow()):
                self.stop_all_motion(send_stop=self.connected)
        super().changeEvent(event)

    def keyPressEvent(self, event):
        if event.isAutoRepeat() or self.mode_box.currentIndex() != 2:
            super().keyPressEvent(event)
            return
        mapping = {
            C.Qt.Key.Key_W: "up", C.Qt.Key.Key_Up: "up",
            C.Qt.Key.Key_S: "down", C.Qt.Key.Key_Down: "down",
            C.Qt.Key.Key_A: "left", C.Qt.Key.Key_Left: "left",
            C.Qt.Key.Key_D: "right", C.Qt.Key.Key_Right: "right",
        }
        if event.key() in mapping:
            self._dir_pressed(mapping[event.key()])
            event.accept()
        elif event.key() == C.Qt.Key.Key_Space:
            self.emergency_stop()
            event.accept()
        else:
            super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat() or self.mode_box.currentIndex() != 2:
            super().keyReleaseEvent(event)
            return
        mapping = {
            C.Qt.Key.Key_W: "up", C.Qt.Key.Key_Up: "up",
            C.Qt.Key.Key_S: "down", C.Qt.Key.Key_Down: "down",
            C.Qt.Key.Key_A: "left", C.Qt.Key.Key_Left: "left",
            C.Qt.Key.Key_D: "right", C.Qt.Key.Key_Right: "right",
        }
        if event.key() in mapping:
            self._dir_released(mapping[event.key()])
            event.accept()
        else:
            super().keyReleaseEvent(event)

    def _suffix_str(self) -> str:
        return ("", "\n", "\r\n", "\r")[self.suffix_box.currentIndex()]

    def _emit_drive(self, force=False):
        self._refresh_drive_readout()
        if not self.connected or not self.link:
            return
        now = monotonic()
        xy = (self.steering_x, self.throttle_y)
        if not force and xy == self.last_sent_xy and (now - self.last_drive_tx) < 0.12:
            return
        if not force and (now - self.last_drive_tx) < 0.045:
            return
        pkt = format_drive_command(self.fmt_edit.text(), xy[0], xy[1]) + self._suffix_str()
        if self.link.send(pkt.encode("utf-8")):
            self.last_sent_xy = xy
            self.last_drive_tx = now

    def stop_all_motion(self, send_stop=True):
        self.held_keys.clear()
        self.steering_x = self.throttle_y = 0
        self.single_joy.reset_center(False)
        self.dual_y.reset_center(False)
        self.dual_x.reset_center(False)
        self._refresh_drive_readout()
        if send_stop and self.connected and self.link:
            pkt = format_drive_command(self.fmt_edit.text(), 0, 0) + self._suffix_str()
            self.link.send(pkt.encode("utf-8"), urgent=True)
            self.last_sent_xy = (0, 0)

    def emergency_stop(self):
        self.stop_all_motion(send_stop=False)
        if self.connected and self.link:
            pkt = format_drive_command(self.fmt_edit.text(), 0, 0) + self._suffix_str()
            self.link.send(pkt.encode("utf-8"), urgent=True)
            self.link.send(("S" + self._suffix_str()).encode("utf-8"))
            self._log("急停", "已清空队列并下发 J,0,0 与停车指令 S")

    def send_car_text(self, cmd: str):
        if not self.connected or not self.link:
            self._log("提示", f"串口未打开（指令预览：{cmd}）")
            return
        self.link.send((cmd + self._suffix_str()).encode("utf-8"))

    def _refresh_drive_readout(self):
        left, right = mix_wheels(self.steering_x, self.throttle_y)
        pkt_preview = format_drive_command(self.fmt_edit.text(), self.steering_x, self.throttle_y)
        self.mix_label.setText(
            f"转向 X: {self.steering_x:+4d}  |  速度 Y: {self.throttle_y:+4d}  →  "
            f"差速混控 左轮:{left:+4d}%  右轮:{right:+4d}%  |  报文: {pkt_preview}"
        )

    # ---------- HC-05 AT Builder & Direct UART Execution ----------
    def _selected_at_baud(self) -> int:
        val = self.at_baud_box.currentData()
        if val is not None:
            return int(val)
        txt = self.at_baud_box.currentText().split("（", 1)[0].strip()
        return int(txt)

    def _ask_at(self, title: str, text: str) -> bool:
        if self.confirm_at is not None:
            return bool(self.confirm_at(title, text))
        ans = W.QMessageBox.question(
            self,
            title,
            text,
            W.QMessageBox.StandardButton.Yes | W.QMessageBox.StandardButton.No,
            W.QMessageBox.StandardButton.No,
        )
        return ans == W.QMessageBox.StandardButton.Yes

    def _refresh_at_previews(self):
        name = self.at_name_edit.text().strip() or "BlueCar"
        baud = self._selected_at_baud()
        stop = self.at_stop_box.currentIndex()
        par = self.at_parity_box.currentIndex()
        pin = self.at_pin_edit.text().strip() or "1234"
        role = self.at_role_box.currentIndex()
        self.at_name_prev.setText(f"AT+NAME={name}\\r\\n")
        self.at_uart_prev.setText(f"AT+UART={baud},{stop},{par}\\r\\n")
        self.at_pin_prev.setText(f"AT+PSWD={pin}\\r\\n")
        self.at_role_prev.setText(f"AT+ROLE={role}\\r\\n")

    def exec_at(self, cmd: str, copy_only=False):
        try:
            wire_str = AtCommandBuilder.with_crlf(cmd)
        except ValueError as exc:
            self._log("校验拦截", str(exc))
            return False
        W.QApplication.clipboard().setText(wire_str)
        if not copy_only and self.connected and self.link:
            self.link.send(wire_str.encode("utf-8"))
            self._log("AT直发", f"已发送并复制：{cmd} + CRLF")
        else:
            self._log("AT复制", f"已复制到剪贴板（含 CRLF）：{cmd}")
        return True

    def _apply_at_name(self, copy_only=False):
        try:
            self.exec_at(AtCommandBuilder.set_name(self.at_name_edit.text()), copy_only=copy_only)
        except ValueError as exc:
            self._log("校验拦截", str(exc))

    def _apply_at_uart(self, copy_only=False):
        try:
            baud = self._selected_at_baud()
            stop = self.at_stop_box.currentIndex()
            parity = self.at_parity_box.currentIndex()
            cmd = AtCommandBuilder.set_uart(baud, stop, parity)
        except ValueError as exc:
            self._log("校验拦截", str(exc))
            return False

        if baud == 5760:
            ok = self._ask_at(
                "HC-05 波特率安全确认",
                "你选择了 5760 bps（兼容版说明书原文，疑似 57600 的笔误）。\n"
                "若写入非标准速率，普通串口模式下将难以连接；如误写可按住 KEY 上电进入 38400 AT 模式后发送 AT+ORGL 恢复。\n\n"
                "仍确认使用 5760 吗？",
            )
            if not ok:
                self._log("校验拦截", "已取消非标准波特率 5760 操作")
                return False

        if copy_only or not (self.connected and self.link):
            return self.exec_at(cmd, copy_only=True)

        # Read back current module UART first before writing!
        self._pending_uart_write = (cmd, (baud, stop, parity), monotonic() + 0.65)
        self.link.send(AtCommandBuilder.with_crlf("AT+UART?").encode("utf-8"))
        self._log("AT校验", f"写入前正在发送 AT+UART? 读取模块当前参数以对比 {baud},{stop},{parity}…")
        return True

    def _finish_uart_write(self, cmd: str, target_tuple: tuple[int, int, int], current):
        tgt_str = f"{target_tuple[0]},{target_tuple[1]},{target_tuple[2]}"
        if current == target_tuple:
            self._log("AT跳过", f"模块当前 UART 已是 {tgt_str}，无需重复写入")
            return
        if current is not None:
            cur_str = f"{current[0]},{current[1]},{current[2]}"
            msg = f"模块当前 UART：{cur_str}\n即将写入 UART：{tgt_str}\n\n确认发送 {cmd}？（修改后需同步切换串口波特率）"
        else:
            msg = f"未能从串口读回当前 +UART 应答（模块可能未处于 38400 AT 模式）。\n\n是否仍确认写入 {cmd}？"
        if self._ask_at("写入前参数对比确认", msg):
            self.exec_at(cmd, copy_only=False)
        else:
            self._log("AT取消", f"已取消写入 {cmd}")

    def _apply_at_pin(self, copy_only=False):
        try:
            self.exec_at(AtCommandBuilder.set_pin(self.at_pin_edit.text()), copy_only=copy_only)
        except ValueError as exc:
            self._log("校验拦截", str(exc))

    def _apply_at_role(self, copy_only=False):
        try:
            self.exec_at(AtCommandBuilder.set_role(self.at_role_box.currentIndex()), copy_only=copy_only)
        except ValueError as exc:
            self._log("校验拦截", str(exc))

    def show_cautions_dialog(self):
        W.QMessageBox.information(self, "HC-05 AT 配置注意事项（源自 BlueSerialStudio）", HC05_CAUTIONS)

    def open_remote_interface_builder(self):
        if self.host is not None and hasattr(self.host, "open_firmware_builder"):
            self.host.open_firmware_builder(
                2,
                remote_cmd_fmt=self.fmt_edit.text().strip(),
                baudrate=self.baud_box.currentText().strip(),
            )
        else:
            self.copy_stm32_mixer_code()

    def copy_stm32_mixer_code(self):
        code = (
            "/* 兼容 BlueSerialStudio 与 PC 综合调试助手的差速混控解析 (J,x,y) */\n"
            "static int Clamp100(int n) { return n < -100 ? -100 : (n > 100 ? 100 : n); }\n"
            "static void MixWheels(int x, int y, int *left, int *right) {\n"
            "    x = Clamp100(x); y = Clamp100(y);\n"
            "    if (y == 0) { *left = x; *right = -x; return; }\n"
            "    int mag = x < 0 ? -x : x;\n"
            "    int inner = ((y < 0 ? -y : y) * (100 - mag) + 50) / 100;\n"
            "    if (x > 0) { if (y > 0) { *left = y; *right = inner; } else { *left = -inner; *right = y; } }\n"
            "    else if (x < 0) { if (y > 0) { *left = inner; *right = y; } else { *left = y; *right = -inner; } }\n"
            "    else { *left = y; *right = y; }\n"
            "}\n"
        )
        W.QApplication.clipboard().setText(code)
        self._log("复制成功", "已复制 STM32 差速混控函数 MixWheels 到剪贴板")

    # ---------- Console & periodic pump ----------
    def _log(self, tag: str, msg: str):
        stamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        line = f"[{stamp}] {tag:<4s}  {msg}"
        self.logs.append(line)
        self.console.appendPlainText(line)

    def clear_console(self):
        self.logs.clear()
        self.console.clear()

    def tick(self):
        if self.mode_box.currentIndex() == 2 and self.held_keys:
            self._update_held_axes()
        elif self.connected and self.link and (self.steering_x != 0 or self.throttle_y != 0):
            if monotonic() - self.last_drive_tx >= 0.10:
                self._emit_drive(force=True)

        if self.link:
            batch, _ = self.link.take()
            for kind, _, data in batch:
                if kind == "open" and not self.closing:
                    self.connected = True
                    self._sync_conn_ui()
                    self._log("系统", f"已打开 {data}")
                elif kind == "tx":
                    self.tx_bytes += len(data)
                    shown = data.decode("utf-8", errors="replace").replace("\r", "\\r").replace("\n", "\\n")
                    self._log("TX", shown)
                elif kind == "rx":
                    self.rx_bytes += len(data)
                    self.rx_tail += data.decode("utf-8", errors="replace")
                    while "\n" in self.rx_tail or len(self.rx_tail) > 240:
                        if "\n" in self.rx_tail:
                            line, self.rx_tail = self.rx_tail.split("\n", 1)
                        else:
                            line, self.rx_tail = self.rx_tail[:240], self.rx_tail[240:]
                        clean_line = line.rstrip("\r")
                        if clean_line:
                            self._log("RX", clean_line)
                            if self._pending_uart_write is not None and "+UART:" in clean_line.upper():
                                try:
                                    raw_vals = clean_line.split(":", 1)[1].split(",")[:3]
                                    cur_tuple = tuple(int(v.strip()) for v in raw_vals)
                                    if len(cur_tuple) == 3:
                                        pending_cmd, target_tuple, _ = self._pending_uart_write
                                        self._pending_uart_write = None
                                        self._finish_uart_write(pending_cmd, target_tuple, cur_tuple)
                                except ValueError:
                                    pass
                elif kind == "error":
                    self._log("异常", str(data))
                elif kind == "closed":
                    self.link = None
                    self.connected = self.closing = False
                    self._sync_conn_ui()
                    break
            self._update_status_bar()

        if self._pending_uart_write is not None and monotonic() >= self._pending_uart_write[2]:
            pending_cmd, target_tuple, _ = self._pending_uart_write
            self._pending_uart_write = None
            self._finish_uart_write(pending_cmd, target_tuple, None)

    def _update_status_bar(self):
        self.status_bar.setText(
            f"{self.badge.text()}     RX  {self.rx_bytes:,} B     TX  {self.tx_bytes:,} B     "
            "协议兼容：BlueSerialStudio (J,x,y 差速遥控 & HC-05 AT CRLF)"
        )
