"""3D Attitude, Artificial Horizon & 6-Axis IMU Debug Workbench Module (MPU6050 / Balance Car).

Supports:
  0. Balance-car 4-col format from Doc/main_balance_test.c: target_deg,pitch_deg,gyro_dps,pwm
     plus #BALANCE:CALIBRATING / READY / TILT_STOP / INIT_ERROR=1..8 status lines.
  1. Euler 3-col format: pitch_deg,roll_deg,yaw_deg.
  2. Raw/scaled 6-axis format: ax,ay,az,gx,gy,gz (auto-detects LSB vs g/dps and runs real-time
     complementary filter fusion with adjustable tau, matching User/balance.c).
  3. Built-in interactive IMU & Balance-Car motion simulator for hardware-free preview.
"""
from collections import deque
import json
import math
from time import monotonic

import numpy as np
import pyqtgraph as pg
from serial.tools import list_ports
from PySide6 import QtCore as C, QtGui, QtWidgets as W

from engine import Lines
from transport import SerialLink


IMU_PROTOCOLS = (
    "平衡车四列 (target, pitch, gyro, pwm)",
    "欧拉角三列 (pitch, roll, yaw [°])",
    "六轴物理量 (ax,ay,az [g], gx,gy,gz [°/s])",
    "MPU6050 原始计数 (16384 LSB/g, 131 LSB/°/s)",
)


class Attitude3DWidget(W.QWidget):
    """Pure-QPainter 3D Balance Car + Attitude Horizon gauge (zero OpenGL dependency)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(460, 320)
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.target_pitch = 0.0
        self.pwm = 0.0
        self.fall_limit = 30.0
        self.dark = False
        self.cam_yaw = -32.0
        self.cam_pitch = 20.0
        self._drag_pos = None
        self.wheel_phase = 0.0

    def set_attitude(self, pitch, roll, yaw, target_pitch=0.0, pwm=0.0):
        self.pitch = float(pitch)
        self.roll = float(roll)
        self.yaw = float(yaw)
        self.target_pitch = float(target_pitch)
        self.pwm = float(pwm)
        self.wheel_phase = (self.wheel_phase + pwm * 0.008) % 360.0
        self.update()

    def mousePressEvent(self, event):
        if event.button() == C.Qt.MouseButton.LeftButton:
            self._drag_pos = event.position()
            event.accept()

    def mouseMoveEvent(self, event):
        if self._drag_pos is not None:
            delta = event.position() - self._drag_pos
            self._drag_pos = event.position()
            self.cam_yaw += delta.x() * 0.45
            self.cam_pitch = max(-65.0, min(75.0, self.cam_pitch + delta.y() * 0.45))
            self.update()
            event.accept()

    def mouseReleaseEvent(self, event):
        self._drag_pos = None

    def mouseDoubleClickEvent(self, event):
        self.cam_yaw, self.cam_pitch = -32.0, 20.0
        self.update()

    def _rot_body(self, pt):
        """Rotate body point (x=forward, y=right, z=up) by intrinsics yaw -> pitch -> roll."""
        x, y, z = pt
        # Roll around X (forward)
        cr, sr = math.cos(math.radians(self.roll)), math.sin(math.radians(self.roll))
        y, z = y * cr - z * sr, y * sr + z * cr
        # Pitch around Y (right): positive pitch leans forward (+x down or +x forward tilt)
        cp, sp = math.cos(math.radians(self.pitch)), math.sin(math.radians(self.pitch))
        x, z = x * cp + z * sp, -x * sp + z * cp
        # Yaw around Z (up)
        cy, sy = math.cos(math.radians(self.yaw)), math.sin(math.radians(self.yaw))
        x, y = x * cy - y * sy, x * sy + y * cy
        return (x, y, z)

    def _project(self, pt3, cx, cy, scale):
        x, y, z = pt3
        # Camera orbit
        ca, sa = math.cos(math.radians(self.cam_yaw)), math.sin(math.radians(self.cam_yaw))
        x1, y1 = x * ca - y * sa, x * sa + y * ca
        ce, se = math.cos(math.radians(self.cam_pitch)), math.sin(math.radians(self.cam_pitch))
        x2 = x1 * ce + z * se
        z2 = -x1 * se + z * ce
        dist = 5.2
        persp = dist / max(1.2, dist - x2 * 0.45)
        sx = cx + y1 * scale * persp
        sy = cy - z2 * scale * persp
        return C.QPointF(sx, sy), x2

    def paintEvent(self, event):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.RenderHint.Antialiasing, True)
        w, h = self.width(), self.height()
        bg = QtGui.QColor("#19212a" if self.dark else "#ffffff")
        border = QtGui.QColor("#3c4b5c" if self.dark else "#d3dae1")
        p.fillRect(0, 0, w, h, bg)
        p.setPen(QtGui.QPen(border, 1))
        p.drawRect(0, 0, w - 1, h - 1)

        # Split canvas: left 62% = 3D Balance Car Model, right 38% = Artificial Horizon + Compass
        split_x = int(w * 0.62)
        p.drawLine(split_x, 10, split_x, h - 10)

        self._draw_3d_car(p, 0, 0, split_x, h)
        self._draw_horizon(p, split_x, 0, w - split_x, h)
        p.end()

    def _draw_3d_car(self, p: QtGui.QPainter, x0, y0, w, h):
        cx, cy = x0 + w * 0.50, y0 + h * 0.60
        scale = min(w, h) * 0.25

        # Ground reference grid (z = -0.55)
        grid_pen = QtGui.QPen(QtGui.QColor("#2c3a48" if self.dark else "#e3e9ef"), 1)
        p.setPen(grid_pen)
        gz = -0.55
        for g in (-1.2, -0.6, 0.0, 0.6, 1.2):
            a1, _ = self._project((-1.2, g, gz), cx, cy, scale)
            a2, _ = self._project((1.2, g, gz), cx, cy, scale)
            b1, _ = self._project((g, -1.2, gz), cx, cy, scale)
            b2, _ = self._project((g, 1.2, gz), cx, cy, scale)
            p.drawLine(a1, a2)
            p.drawLine(b1, b2)

        # Draw upright chassis box + top battery module (pivoting around axle at z=0)
        # Box helper: (xmin, xmax, ymin, ymax, zmin, zmax, fill_hex, edge_hex)
        fallen = abs(self.pitch - self.target_pitch) >= self.fall_limit
        body_fill = "#b83b46" if fallen else ("#2f6db5" if self.dark else "#3b78c2")
        pcb_fill = "#218366" if not fallen else "#8c2f39"
        boxes = [
            # Lower motor mount block
            (-0.16, 0.16, -0.58, 0.58, -0.10, 0.14, "#4b5a6a", "#263442"),
            # Main upright PCB & frame
            (-0.08, 0.08, -0.50, 0.50, 0.14, 1.05, body_fill, "#1d3f6e"),
            # Top 3S LiPo / controller deck
            (-0.20, 0.20, -0.38, 0.38, 0.92, 1.14, pcb_fill, "#145340"),
        ]

        faces_to_draw = []
        face_defs = [
            (0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
            (2, 3, 7, 6), (1, 2, 6, 5), (0, 3, 7, 4),
        ]
        for x1, x2, y1, y2, z1, z2, fcol, ecol in boxes:
            verts = [
                (x1, y1, z1), (x2, y1, z1), (x2, y2, z1), (x1, y2, z1),
                (x1, y1, z2), (x2, y1, z2), (x2, y2, z2), (x1, y2, z2),
            ]
            rverts = [self._rot_body(v) for v in verts]
            proj = [self._project(rv, cx, cy, scale) for rv in rverts]
            for idxs in face_defs:
                pts = [proj[i][0] for i in idxs]
                depth = sum(proj[i][1] for i in idxs) / 4.0
                faces_to_draw.append((depth, pts, QtGui.QColor(fcol), QtGui.QColor(ecol)))

        # Painter's algorithm back-to-front
        faces_to_draw.sort(key=lambda item: item[0])
        for _, pts, fcol, ecol in faces_to_draw:
            fc = QtGui.QColor(fcol)
            fc.setAlpha(225)
            p.setBrush(fc)
            p.setPen(QtGui.QPen(ecol, 1.3))
            p.drawPolygon(QtGui.QPolygonF(pts))

        # Left and right wheels (circles in X-Z plane at y = -0.66 and y = +0.66, radius = 0.52)
        wheel_pen = QtGui.QPen(QtGui.QColor("#26323e" if not self.dark else "#9fb1c1"), 3.0)
        spoke_pen = QtGui.QPen(QtGui.QColor("#7b8c9d"), 1.4)
        p.setBrush(QtGui.QColor(50, 62, 74, 90))
        for wy in (-0.66, 0.66):
            ring = []
            for deg in range(0, 360, 15):
                rad = math.radians(deg)
                wx, wz = 0.52 * math.cos(rad), 0.52 * math.sin(rad)
                pt, _ = self._project(self._rot_body((wx, wy, wz)), cx, cy, scale)
                ring.append(pt)
            p.setPen(wheel_pen)
            p.drawPolygon(QtGui.QPolygonF(ring))
            p.setPen(spoke_pen)
            center_pt, _ = self._project(self._rot_body((0, wy, 0)), cx, cy, scale)
            for s_deg in (0, 90, 180, 270):
                rad = math.radians(s_deg + self.wheel_phase)
                rim_pt, _ = self._project(self._rot_body((0.50 * math.cos(rad), wy, 0.50 * math.sin(rad))), cx, cy, scale)
                p.drawLine(center_pt, rim_pt)

        # Body XYZ coordinate axes from top center
        origin, _ = self._project(self._rot_body((0, 0, 0.55)), cx, cy, scale)
        for vec, col, name in (
            ((0.95, 0, 0.55), "#e03e49", "前 +X"),
            ((0, 0.95, 0.55), "#1ca67a", "右 +Y"),
            ((0, 0, 1.50), "#2c7be5", "上 +Z"),
        ):
            tip, _ = self._project(self._rot_body(vec), cx, cy, scale)
            p.setPen(QtGui.QPen(QtGui.QColor(col), 2.2))
            p.drawLine(origin, tip)
            p.drawText(tip + C.QPointF(4, 4), name)

        # Title & hint
        txt_col = QtGui.QColor("#cfdbe6" if self.dark else "#33485e")
        p.setPen(txt_col)
        font = p.font()
        font.setBold(True)
        p.setFont(font)
        p.drawText(x0 + 12, y0 + 22, "3D 平衡车实时姿态（左键拖拽旋转视角 · 双击复位）")
        font.setBold(False)
        p.setFont(font)
        status_str = f"俯仰 Pitch: {self.pitch:+.2f}°   横滚 Roll: {self.roll:+.2f}°   航向 Yaw: {self.yaw:+.1f}°"
        p.drawText(x0 + 12, y0 + h - 12, status_str)

    def _draw_horizon(self, p: QtGui.QPainter, x0, y0, w, h):
        cx, cy = x0 + w * 0.5, y0 + h * 0.46
        r = min(w * 0.39, h * 0.34)
        if r < 30:
            return

        p.save()
        clip = QtGui.QPainterPath()
        clip.addEllipse(C.QPointF(cx, cy), r, r)
        p.setClipPath(clip)

        # Translate & rotate horizon by -roll and pitch offset
        p.translate(cx, cy)
        p.rotate(-self.roll)
        px_per_deg = r / 40.0
        pitch_px = max(-r * 1.4, min(r * 1.4, self.pitch * px_per_deg))

        # Sky & Ground
        p.fillRect(C.QRectF(-r * 2, -r * 2, r * 4, r * 2 + pitch_px), QtGui.QColor("#3a7ec4"))
        p.fillRect(C.QRectF(-r * 2, pitch_px, r * 4, r * 2.5), QtGui.QColor("#8c6747"))
        p.setPen(QtGui.QPen(QtGui.QColor("white"), 2.0))
        p.drawLine(C.QPointF(-r * 1.5, pitch_px), C.QPointF(r * 1.5, pitch_px))

        # Pitch ladder every 10 deg; +/-30 fall protection in red!
        for deg in (-30, -20, -10, 10, 20, 30):
            yy = pitch_px - deg * px_per_deg
            half_w = r * (0.42 if abs(deg) == 30 else 0.28)
            col = QtGui.QColor("#ff6b6b") if abs(deg) == 30 else QtGui.QColor(255, 255, 255, 215)
            p.setPen(QtGui.QPen(col, 2.0 if abs(deg) == 30 else 1.2))
            p.drawLine(C.QPointF(-half_w, yy), C.QPointF(half_w, yy))
            p.drawText(C.QPointF(half_w + 4, yy + 4), f"{deg:+d}°")
        p.restore()

        # Outer bezel & fixed aircraft/car crosshair
        p.setPen(QtGui.QPen(QtGui.QColor("#4c6075" if not self.dark else "#8fa4b8"), 2.5))
        p.setBrush(C.Qt.BrushStyle.NoBrush)
        p.drawEllipse(C.QPointF(cx, cy), r, r)
        p.setPen(QtGui.QPen(QtGui.QColor("#f59f00"), 2.6))
        p.drawLine(C.QPointF(cx - r * 0.35, cy), C.QPointF(cx - r * 0.10, cy))
        p.drawLine(C.QPointF(cx + r * 0.10, cy), C.QPointF(cx + r * 0.35, cy))
        p.drawEllipse(C.QPointF(cx, cy), 3.5, 3.5)

        txt_col = QtGui.QColor("#cfdbe6" if self.dark else "#33485e")
        p.setPen(txt_col)
        f = p.font(); f.setBold(True); p.setFont(f)
        p.drawText(int(x0 + 12), int(y0 + 22), "人工地平仪（红线 ±30° 倾角保护）")
        f.setBold(False); p.setFont(f)
        fall_warn = abs(self.pitch - self.target_pitch) >= self.fall_limit
        p.setPen(QtGui.QColor("#d9383a" if fall_warn else ("#6ed4b4" if self.dark else "#197a63")))
        state_msg = "⚠ 超过倾角保护阈值！" if fall_warn else f"目标中值角 {self.target_pitch:+.2f}° · 偏差 {self.pitch - self.target_pitch:+.2f}°"
        p.drawText(int(x0 + 14), int(cy + r + 28), state_msg)
        p.setPen(txt_col)
        p.drawText(int(x0 + 14), int(cy + r + 50), f"直立输出 PWM: {self.pwm:+.0f}")


class ImuStudioPage(W.QWidget):
    """Standalone IMU Attitude & 6-Axis Sensor Calibration/Debug Workbench Page."""

    module_id = "imu"
    module_title = "🧭 IMU 姿态"

    def __init__(self, host=None):
        super().__init__(host)
        self.host = host
        self.link = None
        self.sim_active = False
        self.connected = False
        self.closing = False
        self.reconnect_opts = None
        self.parser = Lines()
        self.rx_bytes = self.frames = self.fps = self.rate_frames = 0
        self.epoch = self.rate_start = monotonic()
        self.sim_t = 0.0
        self.last_sample_mono = None

        # Attitude & raw sensor state
        self.pitch = self.roll = self.yaw = 0.0
        self.target_pitch = 0.0
        self.gx = self.gy = self.gz = 0.0
        self.ax = self.ay = 0.0
        self.az = 1.0
        self.pwm = 0.0
        self.acc_norm = 1.0
        self.fused_init = False
        self.zero_collecting = []  # samples for 1-second mechanical center calibration

        self.hist_t = deque(maxlen=800)
        self.hist_pitch = deque(maxlen=800)
        self.hist_roll = deque(maxlen=800)
        self.hist_gyro = deque(maxlen=800)

        self._build_ui()
        self.refresh_ports()
        self._sync_ui()

        self.timer = C.QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.tick)
        self.timer.start()

    # ---------- WorkbenchModule standard interface ----------
    def active_port(self):
        if (self.connected or self.link is not None) and not self.sim_active and self.reconnect_opts:
            return self.reconnect_opts.get("url")
        return None

    def release_port(self):
        if self.connected or self.link:
            self.disconnect_source()
            if self.link:
                self.link.join(0.5)
                if not self.link.is_alive():
                    self.link = None
                    self.closing = False
                    self._sync_ui()

    def apply_theme(self, dark: bool):
        self.canvas.dark = dark
        self.canvas.update()
        self.plot.setBackground("#19212a" if dark else "white")
        for side in ("bottom", "left"):
            self.plot.getAxis(side).setTextPen("#b4c7d5" if dark else "#53667a")

    def save_state(self, settings: C.QSettings):
        st = {
            "source": self.source_box.currentIndex(),
            "port": self.port_box.currentText(),
            "baud": self.baud_box.currentText(),
            "protocol": self.proto_box.currentIndex(),
            "tau": self.tau_spin.value(),
            "invert_pitch": self.inv_pitch.isChecked(),
        }
        settings.setValue("imu_studio/state", json.dumps(st, ensure_ascii=False))

    def restore_state(self, settings: C.QSettings):
        try:
            raw = settings.value("imu_studio/state", "")
            if raw:
                st = json.loads(raw)
                if "source" in st and 0 <= int(st["source"]) < 2:
                    self.source_box.setCurrentIndex(int(st["source"]))
                if st.get("port"):
                    self.port_box.setCurrentText(str(st["port"]))
                if st.get("baud"):
                    self.baud_box.setCurrentText(str(st["baud"]))
                if "protocol" in st and 0 <= int(st["protocol"]) < len(IMU_PROTOCOLS):
                    self.proto_box.setCurrentIndex(int(st["protocol"]))
                if "tau" in st:
                    self.tau_spin.setValue(float(st["tau"]))
                if "invert_pitch" in st:
                    self.inv_pitch.setChecked(bool(st["invert_pitch"]))
        except (ValueError, TypeError):
            pass
        self._sync_ui()

    def shutdown(self):
        self.timer.stop()
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
        root = W.QVBoxLayout(self)
        root.setContentsMargins(0, 0, 0, 0)
        root.setSpacing(0)

        top = W.QFrame()
        top.setObjectName("top")
        tbar = W.QHBoxLayout(top)
        tbar.setContentsMargins(10, 7, 10, 7)
        self.conn_btn = self._btn("连接 IMU", self.toggle_source, "primary")
        tbar.addWidget(self.conn_btn)
        tbar.addWidget(self._btn("航向归零 (Yaw=0)", self.zero_yaw))
        tbar.addWidget(self._btn("清空曲线", self.clear_history))
        tbar.addWidget(self._btn("🎯 标定机械中值角", self.start_zero_capture, "accent"))
        tbar.addWidget(self._btn("复制 MPU6050 上报 C 代码", self.copy_imu_c_snippet))
        self.state_badge = W.QLabel("● 未连接")
        self.state_badge.setStyleSheet("color:#71808b;font-weight:600;padding-left:8px;")
        tbar.addWidget(self.state_badge)
        tbar.addStretch()
        root.addWidget(top)

        self.banner = W.QLabel()
        self.banner.setStyleSheet("background:#8d691e;color:white;padding:5px 10px;font-weight:600;")
        self.banner.hide()
        root.addWidget(self.banner)

        hsplit = W.QSplitter(C.Qt.Orientation.Horizontal)
        root.addWidget(hsplit, 1)

        # Left config panel
        left = W.QWidget()
        left.setMinimumWidth(260)
        left.setMaximumWidth(350)
        lv = W.QVBoxLayout(left)
        lv.setContentsMargins(7, 7, 7, 7)
        lv.setSpacing(6)

        c1 = W.QFrame(); c1.setObjectName("card")
        f1 = W.QFormLayout(c1); f1.setContentsMargins(10, 8, 10, 10); f1.setSpacing(6)
        f1.addRow(W.QLabel("<b>IMU 数据源与协议</b>"))
        self.source_box = W.QComboBox(); self.source_box.addItems(["串口真机 (MPU6050)", "姿态动态仿真"])
        self.source_box.currentIndexChanged.connect(lambda _: self._sync_ui())
        f1.addRow("数据源", self.source_box)
        self.port_box = W.QComboBox(); self.port_box.setEditable(True)
        self.ref_btn = self._btn("刷新", self.refresh_ports)
        pr = W.QHBoxLayout(); pr.setContentsMargins(0, 0, 0, 0)
        pr.addWidget(self.port_box, 1); pr.addWidget(self.ref_btn)
        f1.addRow("串口", pr)
        self.baud_box = W.QComboBox(); self.baud_box.setEditable(True)
        self.baud_box.addItems(["9600", "38400", "57600", "115200", "230400", "460800", "921600"])
        self.baud_box.setCurrentText("115200")
        f1.addRow("波特率", self.baud_box)
        self.proto_box = W.QComboBox(); self.proto_box.addItems(list(IMU_PROTOCOLS))
        f1.addRow("数据格式", self.proto_box)
        self.sim_mode = W.QComboBox()
        self.sim_mode.addItems(["平衡车前后微摆回正", "三轴正弦摇晃演示", "前倾超 30° 倒地保护演示"])
        f1.addRow("仿真动作", self.sim_mode)
        lv.addWidget(c1)

        c2 = W.QFrame(); c2.setObjectName("card")
        f2 = W.QFormLayout(c2); f2.setContentsMargins(10, 8, 10, 10); f2.setSpacing(6)
        f2.addRow(W.QLabel("<b>互补滤波与安装方向</b>"))
        self.tau_spin = W.QDoubleSpinBox()
        self.tau_spin.setRange(0.05, 5.0); self.tau_spin.setSingleStep(0.05); self.tau_spin.setDecimals(2)
        self.tau_spin.setValue(0.35)
        self.tau_spin.setToolTip("与 User/balance.c 中 BALANCE_FILTER_TAU_S 一致，仅在六轴输入模式下参与上位机解算")
        f2.addRow("滤波时间常数 τ(s)", self.tau_spin)
        self.fall_spin = W.QDoubleSpinBox()
        self.fall_spin.setRange(10.0, 80.0); self.fall_spin.setValue(30.0); self.fall_spin.setSuffix(" °")
        self.fall_spin.valueChanged.connect(self._update_fall_limit)
        f2.addRow("倾角保护告警线", self.fall_spin)
        self.inv_pitch = W.QCheckBox("反转俯仰角极性 (适配模块反装)")
        f2.addRow(self.inv_pitch)
        lv.addWidget(c2)

        # Readout dashboard card
        c3 = W.QFrame(); c3.setObjectName("card")
        f3 = W.QVBoxLayout(c3); f3.setContentsMargins(10, 8, 10, 10); f3.setSpacing(5)
        f3.addWidget(W.QLabel("<b>实时六轴与姿态读数</b>"))
        self.readout_lbl = W.QLabel()
        self.readout_lbl.setStyleSheet("font:10pt 'Consolas','Microsoft YaHei UI';")
        self.readout_lbl.setTextInteractionFlags(C.Qt.TextInteractionFlag.TextSelectableByMouse)
        f3.addWidget(self.readout_lbl)
        self.zero_result = W.QLabel("扶正小车静止后点顶部【标定机械中值角】可自动测出重心零点并同步至 PID 页。")
        self.zero_result.setWordWrap(True)
        self.zero_result.setStyleSheet("color:#52687b;font-size:9pt;")
        f3.addWidget(self.zero_result)
        self.sync_pid_btn = self._btn("将中值角同步至 PID 页 ANG.TGT", self.sync_zero_to_pid)
        self.sync_pid_btn.setEnabled(False)
        f3.addWidget(self.sync_pid_btn)
        lv.addWidget(c3)
        lv.addStretch()
        hsplit.addWidget(left)

        # Right main area: top 3D + Horizon canvas, bottom rolling attitude curves
        vsplit = W.QSplitter(C.Qt.Orientation.Vertical)
        self.canvas = Attitude3DWidget()
        vsplit.addWidget(self.canvas)

        self.plot = pg.PlotWidget(background="white")
        self.plot.setMenuEnabled(False)
        self.plot.hideButtons()
        self.plot.showGrid(x=True, y=True, alpha=0.16)
        self.plot.setLabel("bottom", "时间", units="s")
        self.plot.getAxis("bottom").enableAutoSIPrefix(False)
        self.plot.getAxis("left").enableAutoSIPrefix(False)
        self.plot.addLegend(offset=(10, 6))
        self.curve_pitch = self.plot.plot(pen=pg.mkPen("#1b62c7", width=2), name="俯仰 Pitch (°)")
        self.curve_roll = self.plot.plot(pen=pg.mkPen("#008673", width=1.8), name="横滚 Roll (°)")
        self.curve_gyro = self.plot.plot(pen=pg.mkPen("#d47b12", width=1.4), name="角速度 Gyro (°/s ×0.1)")
        vsplit.addWidget(self.plot)
        vsplit.setSizes([430, 230])
        hsplit.addWidget(vsplit)
        hsplit.setStretchFactor(1, 1)
        hsplit.setSizes([300, 960])

        self.status_bar = W.QLabel()
        self.status_bar.setObjectName("status")
        root.addWidget(self.status_bar)
        self._refresh_readouts()

    def _update_fall_limit(self, val):
        self.canvas.fall_limit = float(val)
        self.canvas.update()

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

    def _sync_ui(self):
        sim = self.source_box.currentIndex() == 1
        busy = self.connected or self.link is not None
        self.source_box.setEnabled(not busy)
        for w in (self.port_box, self.ref_btn, self.baud_box, self.proto_box):
            w.setEnabled(not busy and not sim)
        self.sim_mode.setEnabled(sim)
        self.conn_btn.setText("关闭仿真" if self.sim_active else "断开连接" if busy else "启动姿态仿真" if sim else "连接 IMU 串口")
        self.conn_btn.setObjectName("danger" if busy else "primary")
        self.conn_btn.style().unpolish(self.conn_btn)
        self.conn_btn.style().polish(self.conn_btn)
        if self.sim_active:
            self.state_badge.setText("● 姿态仿真运行中")
            self.state_badge.setStyleSheet("color:#188467;font-weight:600;padding-left:8px;")
        elif self.connected:
            self.state_badge.setText(f"● 已连接 {self.port_box.currentText()}")
            self.state_badge.setStyleSheet("color:#188467;font-weight:600;padding-left:8px;")
        else:
            self.state_badge.setText("● 未连接")
            self.state_badge.setStyleSheet("color:#71808b;font-weight:600;padding-left:8px;")
        if self.host is not None and hasattr(self.host, "update_module_badges"):
            self.host.update_module_badges()

    def toggle_source(self):
        if self.connected or self.link is not None:
            self.disconnect_source()
            return
        self.clear_history()
        self.epoch = monotonic()
        self.sim_t = 0.0
        self.fused_init = False
        self.last_sample_mono = None
        if self.source_box.currentIndex() == 1:
            self.sim_active = self.connected = True
            self._sync_ui()
            return

        name = self.port_box.currentText().strip()
        if not name:
            self.banner.setText("请先选择串口，或将数据源切换为【姿态动态仿真】体验 3D 效果")
            self.banner.show()
            return
        if self.host is not None and hasattr(self.host, "request_port_access"):
            if not self.host.request_port_access(self, name):
                return
        try:
            baud = int(self.baud_box.currentText().strip())
            opts = dict(url=name, baudrate=baud, bytesize=8, parity="N", stopbits=1.0)
        except ValueError:
            return
        self.banner.hide()
        self.parser = Lines()
        self.reconnect_opts = opts
        self.link = SerialLink(opts)
        self.link.start()
        self._sync_ui()

    def disconnect_source(self):
        if self.link:
            self.link.closed.set()
            self.closing = True
        self.sim_active = self.connected = False
        self.banner.hide()
        self._sync_ui()

    def zero_yaw(self):
        self.yaw = 0.0
        self.canvas.set_attitude(self.pitch, self.roll, self.yaw, self.target_pitch, self.pwm)

    def clear_history(self):
        self.hist_t.clear()
        self.hist_pitch.clear()
        self.hist_roll.clear()
        self.hist_gyro.clear()
        self.curve_pitch.clear()
        self.curve_roll.clear()
        self.curve_gyro.clear()

    def start_zero_capture(self):
        if not self.connected:
            self.zero_result.setText("请先连接 IMU 串口或打开姿态仿真后再采样中值角。")
            return
        self.zero_collecting = []
        self._capturing_zero = True
        self.zero_result.setText("正在采集 50 帧静止俯仰角均值，请扶稳车身保持平衡不动…")

    def sync_zero_to_pid(self):
        if self.host is not None and hasattr(self.host, "drafts"):
            val = round(self.target_pitch, 4)
            self.host.drafts["ANG"]["TGT"] = val
            if self.host.selected() == "ANG":
                self.host.load_fields()
            W.QApplication.clipboard().setText(f"#define BALANCE_TARGET_PITCH_DEG  {val:.4f}f\n")
            self.zero_result.setText(f"✓ 已将机械中值角 {val:+.3f}° 同步至 PID 页 ANG.TGT 并复制 C 宏定义！")

    def copy_imu_c_snippet(self):
        snippet = (
            "// 在 MPU6050 读取循环中通过 USART2 发送六轴数据（100 Hz，适配【IMU 姿态】页“六轴物理量”模式）：\n"
            "MPU6050_Data_t imu;\n"
            "if (MPU6050_GetCalibratedData(&imu) == MPU6050_OK) {\n"
            "    char buf[80];\n"
            "    int n = npf_snprintf(buf, sizeof(buf), \"%.3f,%.3f,%.3f,%.2f,%.2f,%.2f\\n\",\n"
            "                         imu.AccX, imu.AccY, imu.AccZ, imu.GyroX, imu.GyroY, imu.GyroZ);\n"
            "    if (n > 0 && huart2.gState == HAL_UART_STATE_READY)\n"
            "        HAL_UART_Transmit_IT(&huart2, (uint8_t *)buf, (uint16_t)n);\n"
            "}\n"
        )
        W.QApplication.clipboard().setText(snippet)
        self.zero_result.setText("✓ 已复制 STM32 MPU6050 六轴串口上报 C 代码到剪贴板！")

    # ---------- Telemetry ingestion & Complementary Filter ----------
    def ingest_values(self, vals, t_sec: float, dt_s: float):
        sign = -1.0 if self.inv_pitch.isChecked() else 1.0
        proto = self.proto_box.currentIndex()

        if len(vals) >= 6 or proto in (2, 3):
            if len(vals) < 6:
                return
            if proto == 3 or max(abs(vals[0]), abs(vals[1]), abs(vals[2])) > 20.0:
                ax, ay, az = vals[0] / 16384.0, vals[1] / 16384.0, vals[2] / 16384.0
                gx, gy, gz = vals[3] / 131.0, vals[4] / 131.0, vals[5] / 131.0
            else:
                ax, ay, az, gx, gy, gz = vals[:6]
            self.ax, self.ay, self.az = ax, ay, az
            self.gx, self.gy, self.gz = gx, gy, gz
            self.acc_norm = math.sqrt(ax * ax + ay * ay + az * az)
            acc_pitch = sign * math.degrees(math.atan2(ay, az))
            acc_roll = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))
            gyro_pitch = sign * gx
            tau = self.tau_spin.value()
            alpha_acc = dt_s / (tau + dt_s) if 0.75 <= self.acc_norm <= 1.25 else 0.0
            if not self.fused_init:
                self.pitch, self.roll, self.fused_init = acc_pitch, acc_roll, True
            else:
                self.pitch = (1.0 - alpha_acc) * (self.pitch + gyro_pitch * dt_s) + alpha_acc * acc_pitch
                self.roll = (1.0 - alpha_acc) * (self.roll + gy * dt_s) + alpha_acc * acc_roll
                self.yaw = ((self.yaw + gz * dt_s + 180.0) % 360.0) - 180.0
        elif proto == 1 and len(vals) >= 3:
            self.pitch = sign * vals[0]
            self.roll = vals[1]
            self.yaw = vals[2]
        else:
            # Balance car 4-col: target_deg, pitch_deg, gyro_dps, pwm (or 1..4 cols)
            if len(vals) >= 4:
                self.target_pitch = vals[0]
                self.pitch = sign * vals[1]
                self.gx = sign * vals[2]
                self.pwm = vals[3]
                self.yaw = ((self.yaw + 0.0) % 360.0)
            elif len(vals) >= 2:
                self.target_pitch, self.pitch = vals[0], sign * vals[1]
            elif len(vals) == 1:
                self.pitch = sign * vals[0]

        self.frames += 1
        self.rate_frames += 1
        self.hist_t.append(t_sec)
        self.hist_pitch.append(self.pitch)
        self.hist_roll.append(self.roll)
        self.hist_gyro.append(self.gx * 0.1)

        if getattr(self, "_capturing_zero", False):
            self.zero_collecting.append(self.pitch)
            if len(self.zero_collecting) >= 50:
                self._capturing_zero = False
                mean_p = float(np.mean(self.zero_collecting))
                std_p = float(np.std(self.zero_collecting))
                self.target_pitch = mean_p
                self.sync_pid_btn.setEnabled(True)
                self.zero_result.setText(
                    f"标定完成：机械中值角 = {mean_p:+.3f}°（静止波动 σ={std_p:.3f}°）"
                )

    def _feed_bytes(self, data: bytes, stamp_mono: float):
        self.rx_bytes += len(data)
        # Support up to 6 comma-separated floats on IMU lines!
        for raw_line in data.decode("utf-8", errors="ignore").splitlines():
            s = raw_line.strip()
            if not s:
                continue
            if s.startswith("#BALANCE:"):
                tag = s[9:].strip()
                if tag.startswith("CALIBRATING"):
                    self.banner.setStyleSheet("background:#8d691e;color:white;padding:5px 10px;font-weight:600;")
                    self.banner.setText("IMU 正在预热与 500 帧静止校准，请保持车身静止…")
                    self.banner.show()
                elif tag in ("READY", "RUN"):
                    self.banner.hide()
                elif tag == "TILT_STOP":
                    self.banner.setStyleSheet("background:#a83139;color:white;padding:5px 10px;font-weight:600;")
                    self.banner.setText("倾角超过保护阈值 (TILT_STOP) · 电机驱动已切断")
                    self.banner.show()
                elif tag.startswith("INIT_ERROR"):
                    self.banner.setStyleSheet("background:#a83139;color:white;padding:5px 10px;font-weight:600;")
                    self.banner.setText("IMU 初始化或静止校准失败：" + tag)
                    self.banner.show()
                continue
            if s.startswith(("#", "@")):
                continue
            parts = s.split(",")
            if 1 <= len(parts) <= 6:
                try:
                    vals = [float(p.rsplit(":", 1)[-1]) for p in parts]
                    if all(math.isfinite(v) for v in vals):
                        dt = 0.01 if self.last_sample_mono is None else max(0.001, min(0.2, stamp_mono - self.last_sample_mono))
                        self.last_sample_mono = stamp_mono
                        self.ingest_values(vals, max(0.0, stamp_mono - self.epoch), dt)
                except ValueError:
                    pass

    def tick(self):
        now = monotonic()
        if self.link:
            batch, _ = self.link.take()
            for kind, stamp, data in batch:
                if kind == "open" and not self.closing:
                    self.connected = True
                    self._sync_ui()
                elif kind == "rx":
                    self._feed_bytes(data, stamp)
                elif kind == "closed":
                    self.link = None
                    self.connected = self.closing = False
                    self._sync_ui()
                    break

        if self.sim_active:
            due = int((now - self.epoch) / 0.02)
            done = int(round(self.sim_t / 0.02))
            for _ in range(min(10, max(0, due - done))):
                self.sim_t += 0.02
                t = self.sim_t
                mode = self.sim_mode.currentIndex()
                if mode == 0:
                    # Damped upright balance-car micro-sway with periodic nudge
                    cycle = t % 4.0
                    pitch = 11.0 * math.exp(-1.6 * cycle) * math.cos(5.5 * cycle)
                    gyro = -11.0 * math.exp(-1.6 * cycle) * (1.6 * math.cos(5.5 * cycle) + 5.5 * math.sin(5.5 * cycle))
                    roll = 1.5 * math.sin(1.2 * t)
                    yaw = 8.0 * math.sin(0.5 * t)
                    pwm = max(-450.0, min(450.0, 25.0 * pitch + 1.0 * gyro))
                elif mode == 1:
                    pitch = 18.0 * math.sin(1.6 * t)
                    roll = 22.0 * math.cos(1.1 * t)
                    yaw = ((t * 28.0 + 180.0) % 360.0) - 180.0
                    gyro = 18.0 * 1.6 * math.cos(1.6 * t)
                    pwm = 0.0
                else:
                    # Tips forward past 30 deg protection!
                    cycle = t % 5.0
                    pitch = min(44.0, 2.0 * math.exp(1.35 * cycle)) if cycle < 2.6 else max(0.0, 44.0 - (cycle - 2.6) * 25.0)
                    gyro = 20.0 if cycle < 2.4 else 0.0
                    roll = 0.0
                    yaw = 0.0
                    pwm = 0.0 if pitch >= self.fall_spin.value() else 25.0 * pitch
                self.roll, self.yaw = roll, yaw
                self.ax = -math.sin(math.radians(roll))
                self.ay = math.sin(math.radians(pitch)) * math.cos(math.radians(roll))
                self.az = math.cos(math.radians(pitch)) * math.cos(math.radians(roll))
                self.acc_norm = 1.0
                self.ingest_values([0.0, pitch, gyro, pwm], t, 0.02)
                self.roll, self.yaw = roll, yaw

        if self.connected:
            self.canvas.set_attitude(self.pitch, self.roll, self.yaw, self.target_pitch, self.pwm)
            if len(self.hist_t) >= 2:
                tx = np.fromiter(self.hist_t, dtype=float)
                newest = tx[-1]
                self.plot.setXRange(max(0.0, newest - 8.0), max(8.0, newest), padding=0)
                self.curve_pitch.setData(tx, np.fromiter(self.hist_pitch, dtype=float))
                self.curve_roll.setData(tx, np.fromiter(self.hist_roll, dtype=float))
                self.curve_gyro.setData(tx, np.fromiter(self.hist_gyro, dtype=float))
            self._refresh_readouts()

        if now - self.rate_start >= 1.0:
            self.fps = round(self.rate_frames / (now - self.rate_start))
            self.rate_start, self.rate_frames = now, 0
            self._refresh_readouts()

    def _refresh_readouts(self):
        self.readout_lbl.setText(
            f"俯仰 Pitch : {self.pitch:+7.2f} °\n"
            f"横滚 Roll  : {self.roll:+7.2f} °\n"
            f"航向 Yaw   : {self.yaw:+7.2f} °\n"
            f"目标中值角 : {self.target_pitch:+7.2f} °\n"
            f"角速度 GX  : {self.gx:+7.1f} °/s\n"
            f"重力模 |A| : {self.acc_norm:7.3f} g\n"
            f"直立 PWM   : {self.pwm:+7.0f}"
        )
        self.status_bar.setText(
            f"{self.state_badge.text()}     已收  {self.rx_bytes:,} B     "
            f"姿态帧率  {self.fps} Hz     累计帧  {self.frames:,}"
        )
