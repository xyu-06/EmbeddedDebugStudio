"""Standalone full-featured Serial Debug Assistant module for the multi-tool workbench.

Designed with the unified WorkbenchModule interface (active_port, release_port,
apply_theme, save_state, restore_state, shutdown) so future modules (Camera, IMU,
Bluetooth, etc.) plug into the top navigation bar identically.
"""
from collections import deque
from datetime import datetime
import json
from pathlib import Path
from time import monotonic

import serial
from serial.tools import list_ports
from PySide6 import QtCore as C, QtGui, QtWidgets as W

from bluetooth_link import (
    BluetoothRfcommLink,
    discover_bluetooth_devices,
    format_bt_port_token,
    parse_bt_port_token,
)
from bluetooth_ui import BluetoothPickerDialog
from transport import SerialLink


def crc16_modbus(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if (crc & 1) else (crc >> 1)
    return bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def append_checksum(payload: bytes, mode: int) -> bytes:
    """mode: 0=None, 1=Sum8, 2=XOR8, 3=Modbus CRC16 (little-endian)."""
    if mode == 1:
        return payload + bytes((sum(payload) & 0xFF,))
    if mode == 2:
        x = 0
        for b in payload:
            x ^= b
        return payload + bytes((x,))
    if mode == 3:
        return payload + crc16_modbus(payload)
    return payload


DEFAULT_QUICK_SLOTS = [
    ("AT 测试", "AT", False),
    ("查版本", "AT+VERSION?", False),
    ("读SPD参数", "@SPD.GET", False),
    ("读ANG参数", "@ANG.GET", False),
    ("启动闭环", "@RUN", False),
    ("紧急停机", "@STOP", False),
    ("清零积分", "@SPD.RST", False),
    ("校准陀螺仪", "@CAL", False),
    ("查询电压", "@BAT?", False),
    ("复位设备", "@RESET", False),
    ("HEX 帧头示例", "AA 55 01 00", True),
    ("查询状态", "@STATUS", False),
]


class SerialAssistantPage(W.QWidget):
    """Independent Serial Debug Assistant page with ASCII/HEX, UTF-8/GBK, RTS/DTR,
    keyword filter, frame timeout line-break, checksums, cyclic TX, file TX & 12 quick slots."""

    module_id = "serial"
    module_title = "🔌 串口调试"

    def __init__(self, host=None):
        super().__init__(host)
        self.host = host
        self.link = None
        self.connected = False
        self.closing = False
        self.reconnect_at = None
        self.reconnect_opts = None
        self.rx_bytes = self.tx_bytes = self.rx_frames = self.tx_frames = self.dropped = 0
        self.rate_bytes = self.bps = 0
        self.rate_start = monotonic()
        self.entries = deque(maxlen=1200)  # [stamp_str, direction, bytearray, complete_bool]
        self.rx_tail = None
        self.last_rx_mono = 0.0
        self.dirty = False
        self.last_render = 0.0
        self.tx_history = deque(maxlen=30)
        self.quick_slots = [list(item) for item in DEFAULT_QUICK_SLOTS]

        self._build_ui()
        self.refresh_ports()
        self._update_conn_ui()

        self.poll_timer = C.QTimer(self)
        self.poll_timer.setInterval(30)
        self.poll_timer.timeout.connect(self.tick)
        self.poll_timer.start()

        self.cycle_timer = C.QTimer(self)
        self.cycle_timer.timeout.connect(self._send_cyclic)

    # ---------- WorkbenchModule standard interface ----------
    def active_port(self):
        if (self.connected or self.link is not None or self.reconnect_at is not None) and self.reconnect_opts:
            return self.reconnect_opts.get("url")
        return None

    def release_port(self):
        if self.connected or self.link or self.reconnect_at is not None:
            self.disconnect_port()
            if self.link:
                self.link.join(0.5)
                if not self.link.is_alive():
                    self.link = None
                    self.closing = False
                    self._update_conn_ui()

    def apply_theme(self, dark: bool):
        bg = "#19212a" if dark else "#ffffff"
        fg = "#e4eaf1" if dark else "#1f2f40"
        border = "#4a5868" if dark else "#d5dce3"
        self.log_view.setStyleSheet(
            f"QPlainTextEdit {{background:{bg};color:{fg};border:1px solid {border};"
            "font:10pt 'Consolas','Microsoft YaHei UI';}}"
        )
        self.tx_edit.setStyleSheet(
            f"QPlainTextEdit {{background:{bg};color:{fg};border:1px solid {border};"
            "font:10pt 'Consolas','Microsoft YaHei UI';}}"
        )

    def save_state(self, settings: C.QSettings):
        state = {
            "port": self.port_box.currentText(),
            "baud": self.baud_box.currentText(),
            "bits": self.bits_box.currentIndex(),
            "parity": self.parity_box.currentIndex(),
            "stops": self.stops_box.currentIndex(),
            "flow": self.flow_box.currentIndex(),
            "encoding": self.encoding_box.currentIndex(),
            "rx_hex": self.rx_hex.isChecked(),
            "show_time": self.show_time.isChecked(),
            "show_dir": self.show_dir.isChecked(),
            "autoscroll": self.autoscroll.isChecked(),
            "timeout_break": self.timeout_break.isChecked(),
            "frame_ms": self.frame_ms.value(),
            "auto_reconnect": self.auto_reconnect.isChecked(),
            "tx_hex": self.tx_hex.isChecked(),
            "newline": self.newline_box.currentIndex(),
            "checksum": self.checksum_box.currentIndex(),
            "cycle_ms": self.cycle_ms.value(),
            "tx_text": self.tx_edit.toPlainText()[:2000],
            "slots": self.quick_slots,
        }
        settings.setValue("serial_studio/state", json.dumps(state, ensure_ascii=False))
        settings.setValue("serial_studio/hsplit", self.hsplit.saveState())
        settings.setValue("serial_studio/vsplit", self.vsplit.saveState())

    def restore_state(self, settings: C.QSettings):
        try:
            raw = settings.value("serial_studio/state", "")
            if raw:
                st = json.loads(raw)
                if st.get("port"):
                    self.port_box.setCurrentText(str(st["port"]))
                if st.get("baud"):
                    self.baud_box.setCurrentText(str(st["baud"]))
                for k, box in (
                    ("bits", self.bits_box),
                    ("parity", self.parity_box),
                    ("stops", self.stops_box),
                    ("flow", self.flow_box),
                    ("encoding", self.encoding_box),
                    ("newline", self.newline_box),
                    ("checksum", self.checksum_box),
                ):
                    idx = st.get(k, box.currentIndex())
                    if isinstance(idx, int) and 0 <= idx < box.count():
                        box.setCurrentIndex(idx)
                for k, chk in (
                    ("rx_hex", self.rx_hex),
                    ("show_time", self.show_time),
                    ("show_dir", self.show_dir),
                    ("autoscroll", self.autoscroll),
                    ("timeout_break", self.timeout_break),
                    ("auto_reconnect", self.auto_reconnect),
                    ("tx_hex", self.tx_hex),
                ):
                    if k in st:
                        chk.setChecked(bool(st[k]))
                if "frame_ms" in st:
                    self.frame_ms.setValue(int(st["frame_ms"]))
                if "cycle_ms" in st:
                    self.cycle_ms.setValue(int(st["cycle_ms"]))
                if "tx_text" in st:
                    self.tx_edit.setPlainText(str(st["tx_text"]))
                if isinstance(st.get("slots"), list):
                    for i, item in enumerate(st["slots"][: len(self.quick_slots)]):
                        if isinstance(item, (list, tuple)) and len(item) == 3:
                            self.quick_slots[i] = [str(item[0])[:20], str(item[1])[:256], bool(item[2])]
                            self._sync_slot_row(i)
            if settings.contains("serial_studio/hsplit"):
                self.hsplit.restoreState(settings.value("serial_studio/hsplit"))
            if settings.contains("serial_studio/vsplit"):
                self.vsplit.restoreState(settings.value("serial_studio/vsplit"))
        except (ValueError, TypeError, KeyError):
            pass

    def shutdown(self):
        self.poll_timer.stop()
        self.cycle_timer.stop()
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

        # Top action bar dedicated to Serial Assistant
        top = W.QFrame()
        top.setObjectName("top")
        tbar = W.QHBoxLayout(top)
        tbar.setContentsMargins(10, 7, 10, 7)
        self.conn_btn = self._btn("打开串口", self.toggle_port, "primary")
        self.pause_rx = W.QPushButton("暂停显示")
        self.pause_rx.setCheckable(True)
        self.pause_rx.toggled.connect(self._on_pause_toggled)
        tbar.addWidget(self.conn_btn)
        tbar.addWidget(self.pause_rx)
        tbar.addWidget(self._btn("清空接收", self.clear_logs))
        tbar.addWidget(self._btn("清零计数", self.reset_counters))
        tbar.addWidget(self._btn("保存日志", self.save_log_file))
        tbar.addWidget(self._btn("发送文件…", self.send_file_dialog))
        self.conn_badge = W.QLabel("● 未连接")
        self.conn_badge.setStyleSheet("color:#71808b;font-weight:600;padding-left:6px;")
        tbar.addWidget(self.conn_badge)
        tbar.addStretch()
        tbar.addWidget(W.QLabel("过滤:"))
        self.filter_edit = W.QLineEdit()
        self.filter_edit.setPlaceholderText("输入关键字筛选日志（留空显示全部）")
        self.filter_edit.setMaximumWidth(230)
        self.filter_edit.textChanged.connect(lambda _: self.render_logs(force=True))
        tbar.addWidget(self.filter_edit)
        root.addWidget(top)

        self.hsplit = W.QSplitter(C.Qt.Orientation.Horizontal)
        root.addWidget(self.hsplit, 1)

        # 1) Left configuration scroll panel
        left_scroll = W.QScrollArea()
        left_scroll.setFrameShape(W.QFrame.Shape.NoFrame)
        left_scroll.setWidgetResizable(True)
        left_scroll.setMinimumWidth(255)
        left_scroll.setMaximumWidth(360)
        left_wrap = W.QWidget()
        left_v = W.QVBoxLayout(left_wrap)
        left_v.setContentsMargins(7, 7, 7, 7)
        left_v.setSpacing(6)
        left_scroll.setWidget(left_wrap)
        self.hsplit.addWidget(left_scroll)

        # Card A: Port Settings
        port_card, pf = self._make_card("串口参数设置")
        self.port_box = W.QComboBox()
        self.port_box.setEditable(True)
        self.refresh_btn = self._btn("刷新", self.refresh_ports)
        self.bt_btn = self._btn("🔵 蓝牙", self.pick_bluetooth_device)
        self.bt_btn.setToolTip("搜索/配对并选择经典蓝牙 SPP 设备（如 HC-05）")
        pr = W.QHBoxLayout(); pr.setContentsMargins(0, 0, 0, 0); pr.setSpacing(4)
        pr.addWidget(self.port_box, 1); pr.addWidget(self.refresh_btn); pr.addWidget(self.bt_btn)
        pf.addRow("端口/蓝牙", pr)
        self.baud_box = W.QComboBox()
        self.baud_box.setEditable(True)
        self.baud_box.addItems(["4800", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600", "1000000", "2000000"])
        self.baud_box.setCurrentText("115200")
        pf.addRow("波特率", self.baud_box)
        self.bits_box = W.QComboBox(); self.bits_box.addItems(["5", "6", "7", "8"]); self.bits_box.setCurrentIndex(3)
        self.parity_box = W.QComboBox(); self.parity_box.addItems(["无 (None)", "偶 (Even)", "奇 (Odd)", "Mark", "Space"])
        self.stops_box = W.QComboBox(); self.stops_box.addItems(["1", "1.5", "2"])
        self.flow_box = W.QComboBox(); self.flow_box.addItems(["无", "RTS/CTS", "XON/XOFF"])
        pf.addRow("数据位", self.bits_box)
        pf.addRow("校验位", self.parity_box)
        pf.addRow("停止位", self.stops_box)
        pf.addRow("硬件流控", self.flow_box)
        self.dtr_chk = W.QCheckBox("DTR 置高")
        self.rts_chk = W.QCheckBox("RTS 置高")
        self.dtr_chk.toggled.connect(self._apply_modem_lines)
        self.rts_chk.toggled.connect(self._apply_modem_lines)
        mr = W.QHBoxLayout(); mr.setContentsMargins(0, 0, 0, 0)
        mr.addWidget(self.dtr_chk); mr.addWidget(self.rts_chk)
        pf.addRow("控制线", mr)
        self.auto_reconnect = W.QCheckBox("意外断线自动重连")
        self.auto_reconnect.setChecked(True)
        pf.addRow(self.auto_reconnect)
        left_v.addWidget(port_card)

        # Card B: Receive Settings
        rx_card, rf = self._make_card("接收显示设置")
        self.rx_hex = W.QCheckBox("HEX 十六进制显示")
        self.rx_hex.toggled.connect(lambda _: self.render_logs(force=True))
        rf.addRow(self.rx_hex)
        self.encoding_box = W.QComboBox()
        self.encoding_box.addItems(["UTF-8（现代工程默认）", "GBK（兼容中文老工程）", "Latin-1（单字节原样）"])
        self.encoding_box.currentIndexChanged.connect(lambda _: self.render_logs(force=True))
        rf.addRow("文本编码", self.encoding_box)
        self.show_time = W.QCheckBox("显示毫秒时间戳"); self.show_time.setChecked(True)
        self.show_dir = W.QCheckBox("显示收/发方向"); self.show_dir.setChecked(True)
        self.autoscroll = W.QCheckBox("自动滚动到底部"); self.autoscroll.setChecked(True)
        for chk in (self.show_time, self.show_dir):
            chk.toggled.connect(lambda _: self.render_logs(force=True))
        rf.addRow(self.show_time)
        rf.addRow(self.show_dir)
        rf.addRow(self.autoscroll)
        self.timeout_break = W.QCheckBox("按空闲超时断帧")
        self.timeout_break.setChecked(True)
        self.frame_ms = W.QSpinBox(); self.frame_ms.setRange(5, 2000); self.frame_ms.setValue(40); self.frame_ms.setSuffix(" ms")
        tr = W.QHBoxLayout(); tr.setContentsMargins(0, 0, 0, 0)
        tr.addWidget(self.timeout_break); tr.addWidget(self.frame_ms)
        rf.addRow(tr)
        left_v.addWidget(rx_card)

        # Card C: Transmit Settings
        tx_card, tf = self._make_card("发送格式设置")
        self.tx_hex = W.QCheckBox("HEX 十六进制发送")
        tf.addRow(self.tx_hex)
        self.newline_box = W.QComboBox()
        self.newline_box.addItems(["不追加换行", "追加 \\r\\n (CRLF)", "追加 \\n (LF)", "追加 \\r (CR)"])
        self.newline_box.setCurrentIndex(2)
        tf.addRow("行尾追加", self.newline_box)
        self.checksum_box = W.QComboBox()
        self.checksum_box.addItems(["无校验", "末尾追加 Sum8 累加和", "末尾追加 XOR 异或字节", "末尾追加 Modbus CRC16"])
        tf.addRow("附加校验", self.checksum_box)
        self.cycle_chk = W.QCheckBox("定时发送")
        self.cycle_ms = W.QSpinBox(); self.cycle_ms.setRange(10, 60000); self.cycle_ms.setValue(1000); self.cycle_ms.setSuffix(" ms")
        self.cycle_chk.toggled.connect(self._toggle_cyclic)
        self.cycle_ms.valueChanged.connect(self._update_cycle_interval)
        cr = W.QHBoxLayout(); cr.setContentsMargins(0, 0, 0, 0)
        cr.addWidget(self.cycle_chk); cr.addWidget(self.cycle_ms)
        tf.addRow(cr)
        left_v.addWidget(tx_card)
        left_v.addStretch()

        # 2) Center: Vertical splitter (Receive Log View + Multi-line Transmit Box)
        self.vsplit = W.QSplitter(C.Qt.Orientation.Vertical)
        self.hsplit.addWidget(self.vsplit)

        rx_panel = W.QWidget()
        rx_lay = W.QVBoxLayout(rx_panel)
        rx_lay.setContentsMargins(4, 6, 4, 2)
        rx_lay.setSpacing(4)
        self.log_view = W.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setUndoRedoEnabled(False)
        rx_lay.addWidget(self.log_view, 1)
        self.vsplit.addWidget(rx_panel)

        tx_panel = W.QWidget()
        tx_lay = W.QVBoxLayout(tx_panel)
        tx_lay.setContentsMargins(4, 2, 4, 6)
        tx_lay.setSpacing(5)
        hist_bar = W.QHBoxLayout()
        hist_bar.addWidget(W.QLabel("<b>发送区</b>（Ctrl+Enter 快捷发送）"))
        hist_bar.addStretch()
        hist_bar.addWidget(W.QLabel("历史发送:"))
        self.history_box = W.QComboBox()
        self.history_box.setMinimumWidth(200)
        self.history_box.addItem("选择历史记录…")
        self.history_box.activated.connect(self._pick_history)
        hist_bar.addWidget(self.history_box)
        tx_lay.addLayout(hist_bar)

        send_row = W.QHBoxLayout()
        self.tx_edit = W.QPlainTextEdit()
        self.tx_edit.setPlaceholderText("输入要发送的文本，或勾选 HEX 发送后输入如 AA 55 01 02 的十六进制字节…")
        self.tx_edit.setMaximumHeight(115)
        send_Shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Return"), self.tx_edit)
        send_Shortcut.activated.connect(self.send_from_editor)
        send_btn = self._btn("发 送", self.send_from_editor, "primary")
        send_btn.setMinimumSize(86, 64)
        clear_tx_btn = self._btn("清空输入", self.tx_edit.clear)
        btn_col = W.QVBoxLayout()
        btn_col.addWidget(send_btn)
        btn_col.addWidget(clear_tx_btn)
        send_row.addWidget(self.tx_edit, 1)
        send_row.addLayout(btn_col)
        tx_lay.addLayout(send_row)
        self.vsplit.addWidget(tx_panel)

        # 3) Right: 12-slot Multi-command Quick Send Panel
        right_card = W.QFrame()
        right_card.setObjectName("pidPanel")
        right_card.setMinimumWidth(250)
        right_card.setMaximumWidth(360)
        rv = W.QVBoxLayout(right_card)
        rv.setContentsMargins(8, 8, 8, 8)
        rv.setSpacing(5)
        hdr = W.QHBoxLayout()
        hdr.addWidget(W.QLabel("<b>多条快捷指令集</b>"))
        hdr.addStretch()
        hdr.addWidget(self._btn("导入", self.import_slots))
        hdr.addWidget(self._btn("导出", self.export_slots))
        rv.addLayout(hdr)
        sub = W.QLabel("勾选 HEX 按十六进制发，其余按文本追加当前行尾发送：")
        sub.setWordWrap(True)
        sub.setStyleSheet("color:#5b7083;font-size:8.5pt;")
        rv.addWidget(sub)

        slots_scroll = W.QScrollArea()
        slots_scroll.setFrameShape(W.QFrame.Shape.NoFrame)
        slots_scroll.setWidgetResizable(True)
        slots_wrap = W.QWidget()
        slots_v = W.QVBoxLayout(slots_wrap)
        slots_v.setContentsMargins(0, 2, 2, 2)
        slots_v.setSpacing(4)
        self.slot_widgets = []
        for i, (label, cmd, is_hex) in enumerate(self.quick_slots):
            row_w = W.QWidget()
            rl = W.QHBoxLayout(row_w)
            rl.setContentsMargins(0, 0, 0, 0)
            rl.setSpacing(3)
            hex_c = W.QCheckBox("H")
            hex_c.setToolTip("勾选后按 HEX 字节发送")
            hex_c.setChecked(is_hex)
            cmd_ed = W.QLineEdit(cmd)
            cmd_ed.setPlaceholderText(f"指令 {i+1}")
            fire_b = W.QPushButton(label)
            fire_b.setFixedWidth(78)
            fire_b.setToolTip("左键发送，右键修改按钮名称")
            fire_b.setContextMenuPolicy(C.Qt.ContextMenuPolicy.CustomContextMenu)
            hex_c.toggled.connect(lambda v, idx=i: self._slot_edited(idx))
            cmd_ed.textChanged.connect(lambda _, idx=i: self._slot_edited(idx))
            fire_b.clicked.connect(lambda _=False, idx=i: self.send_quick_slot(idx))
            fire_b.customContextMenuRequested.connect(lambda _, idx=i: self.rename_slot(idx))
            rl.addWidget(hex_c)
            rl.addWidget(cmd_ed, 1)
            rl.addWidget(fire_b)
            slots_v.addWidget(row_w)
            self.slot_widgets.append((hex_c, cmd_ed, fire_b))
        slots_v.addStretch()
        slots_scroll.setWidget(slots_wrap)
        rv.addWidget(slots_scroll, 1)
        self.hsplit.addWidget(right_card)

        self.hsplit.setCollapsible(1, False)
        self.hsplit.setStretchFactor(1, 1)
        self.hsplit.setSizes([270, 720, 290])
        self.vsplit.setSizes([520, 170])

        # Bottom dedicated status bar
        self.status_bar = W.QLabel()
        self.status_bar.setObjectName("status")
        root.addWidget(self.status_bar)
        self._refresh_status_bar()

    def _make_card(self, title):
        box = W.QFrame()
        box.setObjectName("card")
        v = W.QVBoxLayout(box)
        v.setContentsMargins(10, 7, 10, 9)
        v.setSpacing(5)
        lbl = W.QLabel(f"<b>{title}</b>")
        v.addWidget(lbl)
        form = W.QFormLayout()
        form.setContentsMargins(0, 2, 0, 0)
        form.setSpacing(5)
        v.addLayout(form)
        return box, form

    # ---------- Port management ----------
    def refresh_ports(self):
        prev = self.port_box.currentText()
        self.port_box.clear()
        try:
            for p in sorted(list_ports.comports(), key=lambda x: x.device):
                self.port_box.addItem(p.device)
                idx = self.port_box.count() - 1
                self.port_box.setItemData(idx, f"{p.device} · {p.description}", C.Qt.ItemDataRole.ToolTipRole)
        except OSError as exc:
            self._append_sys(f"枚举串口失败：{exc}")
        try:
            for dev in discover_bluetooth_devices(inquiry=False):
                if dev.authenticated:
                    token = format_bt_port_token(dev.address, dev.name)
                    self.port_box.addItem(token)
                    idx = self.port_box.count() - 1
                    self.port_box.setItemData(idx, f"蓝牙 SPP 直连 · {dev.name} ({dev.address})", C.Qt.ItemDataRole.ToolTipRole)
        except Exception:
            pass
        if prev:
            self.port_box.setCurrentText(prev)

    def pick_bluetooth_device(self):
        dlg = BluetoothPickerDialog(self)
        if dlg.exec() == W.QDialog.DialogCode.Accepted and dlg.selected_port_text:
            if self.port_box.findText(dlg.selected_port_text) < 0:
                self.port_box.addItem(dlg.selected_port_text)
            self.port_box.setCurrentText(dlg.selected_port_text)

    def _create_link(self, opts):
        bt = parse_bt_port_token(opts.get("url", ""))
        if bt is not None:
            return BluetoothRfcommLink(bt[0], channel=1, name=bt[1] or bt[0])
        return SerialLink(opts)

    def toggle_port(self):
        if self.connected or self.link or self.reconnect_at is not None:
            self.disconnect_port()
            return
        name = self.port_box.currentText().strip()
        if not name:
            self._append_sys("请先选择或输入串口名称（如 COM5），或点击【🔵 蓝牙】")
            return
        # Check cross-module port conflict via host workbench!
        if self.host is not None and hasattr(self.host, "request_port_access"):
            if not self.host.request_port_access(self, name):
                return
        bt = parse_bt_port_token(name)
        if bt is not None:
            opts = dict(url=name)
        else:
            try:
                baud = int(self.baud_box.currentText().strip())
                if not 50 <= baud <= 12_000_000:
                    raise ValueError("波特率超出有效范围")
                parity_code = ("N", "E", "O", "M", "S")[self.parity_box.currentIndex()]
                opts = dict(
                    url=name,
                    baudrate=baud,
                    bytesize=int(self.bits_box.currentText()),
                    parity=parity_code,
                    stopbits=float(self.stops_box.currentText()),
                    rtscts=self.flow_box.currentIndex() == 1,
                    xonxoff=self.flow_box.currentIndex() == 2,
                )
            except ValueError as exc:
                self._append_sys(f"参数无效：{exc}")
                return

        self.reconnect_opts = opts
        self.reconnect_at = None
        self.closing = False
        self.rx_tail = None
        self.link = self._create_link(opts)
        if bt is None and (self.dtr_chk.isChecked() or self.rts_chk.isChecked()):
            self.link.set_modem_lines(self.dtr_chk.isChecked(), self.rts_chk.isChecked())
        self.link.start()
        self._update_conn_ui()

    def disconnect_port(self):
        self.cycle_chk.setChecked(False)
        self.reconnect_at = None
        self.rx_tail = None
        if self.link:
            self.link.closed.set()
            self.closing = True
        self.connected = False
        self._update_conn_ui()
        self._append_sys("已关闭串口连接")

    def _apply_modem_lines(self):
        if self.link:
            self.link.set_modem_lines(self.dtr_chk.isChecked(), self.rts_chk.isChecked())

    def _update_conn_ui(self):
        waiting = self.reconnect_at is not None
        busy = self.connected or self.link is not None or waiting
        for w in (self.port_box, self.refresh_btn, self.bt_btn, self.baud_box, self.bits_box, self.parity_box, self.stops_box, self.flow_box):
            w.setEnabled(not busy)
        self.conn_btn.setEnabled(not self.closing)
        self.conn_btn.setText("关闭串口" if self.connected else "取消重连" if waiting else "断开" if busy else "打开串口")
        self.conn_btn.setObjectName("danger" if self.connected else "primary")
        self.conn_btn.style().unpolish(self.conn_btn)
        self.conn_btn.style().polish(self.conn_btn)
        if self.connected:
            port_txt = self.reconnect_opts["url"] if self.reconnect_opts else self.port_box.currentText()
            if isinstance(self.link, BluetoothRfcommLink):
                self.conn_badge.setText(f"● 蓝牙已连 {port_txt}")
            else:
                self.conn_badge.setText(f"● 已连接 {port_txt} ({self.baud_box.currentText()})")
            self.conn_badge.setStyleSheet("color:#188467;font-weight:600;padding-left:6px;")
        elif waiting:
            self.conn_badge.setText("● 等待自动重连…")
            self.conn_badge.setStyleSheet("color:#8d691e;font-weight:600;padding-left:6px;")
        elif busy:
            self.conn_badge.setText("● 正在打开…")
            self.conn_badge.setStyleSheet("color:#8d691e;font-weight:600;padding-left:6px;")
        else:
            self.conn_badge.setText("● 未连接")
            self.conn_badge.setStyleSheet("color:#71808b;font-weight:600;padding-left:6px;")
        if self.host is not None and hasattr(self.host, "update_module_badges"):
            self.host.update_module_badges()

    # ---------- RX / TX processing ----------
    def _stamp(self):
        return datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def _append_sys(self, text: str):
        self.entries.append([self._stamp(), "系统", text.encode("utf-8"), True])
        self.dirty = True
        self.render_logs()

    def _ingest_rx(self, data: bytes, stamp_mono: float):
        self.rx_bytes += len(data)
        self.rate_bytes += len(data)
        # Break frame on timeout gap or newline!
        if (
            self.rx_tail is not None
            and self.timeout_break.isChecked()
            and (stamp_mono - self.last_rx_mono) * 1000.0 >= self.frame_ms.value()
        ):
            self.rx_tail[3] = True
            self.rx_tail = None
        self.last_rx_mono = stamp_mono

        if self.rx_hex.isChecked() and self.timeout_break.isChecked():
            # In HEX + frame-timeout mode, group contiguous bursts up to 512B per frame
            offset = 0
            while offset < len(data):
                if self.rx_tail is None:
                    entry = [self._stamp(), "RX", bytearray(), False]
                    self.entries.append(entry)
                    self.rx_tail = entry
                    self.rx_frames += 1
                room = 512 - len(self.rx_tail[2])
                self.rx_tail[2].extend(data[offset : offset + room])
                offset += room
                if len(self.rx_tail[2]) >= 512:
                    self.rx_tail[3] = True
                    self.rx_tail = None
        else:
            for part in data.splitlines(keepends=True):
                while part:
                    if self.rx_tail is None:
                        entry = [self._stamp(), "RX", bytearray(), False]
                        self.entries.append(entry)
                        self.rx_tail = entry
                        self.rx_frames += 1
                    room = 2048 - len(self.rx_tail[2])
                    self.rx_tail[2].extend(part[:room])
                    part = part[room:]
                    if self.rx_tail[2].endswith(b"\n") or len(self.rx_tail[2]) >= 2048:
                        self.rx_tail[3] = True
                        self.rx_tail = None
        self.dirty = True

    def encode_payload(self, text: str, is_hex: bool) -> bytes:
        if is_hex:
            cleaned = text.replace(",", " ").replace("0x", " ").replace("0X", " ")
            raw = bytes.fromhex(cleaned)
        else:
            enc = ("utf-8", "gbk", "latin1")[self.encoding_box.currentIndex()]
            raw = text.encode(enc, errors="replace")
            suffix = (b"", b"\r\n", b"\n", b"\r")[self.newline_box.currentIndex()]
            raw += suffix
        return append_checksum(raw, self.checksum_box.currentIndex())

    def send_payload(self, raw: bytes, record_history: str = "") -> bool:
        if not self.connected or not self.link:
            self._append_sys("请先打开串口再发送数据")
            return False
        if not raw:
            return False
        if not self.link.send(raw):
            self._append_sys("发送队列已满")
            return False
        if record_history:
            s = record_history.strip()
            if s and (not self.tx_history or self.tx_history[-1] != s):
                self.tx_history.append(s)
                self.history_box.addItem(s[:60])
        return True

    def send_from_editor(self):
        text = self.tx_edit.toPlainText()
        if not text:
            return
        try:
            raw = self.encode_payload(text, self.tx_hex.isChecked())
        except ValueError:
            self._append_sys("HEX 格式错误：请输入两位一组的十六进制字节（如 AA 55 01 FF）")
            return
        self.send_payload(raw, record_history=text)

    def _send_cyclic(self):
        if not self.connected:
            self.cycle_chk.setChecked(False)
            return
        self.send_from_editor()

    def _toggle_cyclic(self, checked: bool):
        if checked:
            if not self.connected:
                self._append_sys("请先连接串口后再开启定时发送")
                self.cycle_chk.setChecked(False)
                return
            self.cycle_timer.start(self.cycle_ms.value())
        else:
            self.cycle_timer.stop()

    def _update_cycle_interval(self, val: int):
        if self.cycle_timer.isActive():
            self.cycle_timer.setInterval(val)

    def _pick_history(self, idx: int):
        if idx > 0 and idx - 1 < len(self.tx_history):
            self.tx_edit.setPlainText(self.tx_history[idx - 1])

    # ---------- Quick 12-slot commands ----------
    def _sync_slot_row(self, idx: int):
        label, cmd, is_hex = self.quick_slots[idx]
        hex_c, cmd_ed, fire_b = self.slot_widgets[idx]
        with C.QSignalBlocker(hex_c), C.QSignalBlocker(cmd_ed):
            hex_c.setChecked(is_hex)
            cmd_ed.setText(cmd)
        fire_b.setText(label)

    def _slot_edited(self, idx: int):
        hex_c, cmd_ed, _ = self.slot_widgets[idx]
        self.quick_slots[idx][1] = cmd_ed.text()
        self.quick_slots[idx][2] = hex_c.isChecked()

    def rename_slot(self, idx: int):
        old_name = self.quick_slots[idx][0]
        name, ok = W.QInputDialog.getText(self, "修改快捷按钮名称", "按钮显示文字：", text=old_name)
        if ok and name.strip():
            self.quick_slots[idx][0] = name.strip()[:16]
            self.slot_widgets[idx][2].setText(self.quick_slots[idx][0])

    def send_quick_slot(self, idx: int):
        _, cmd, is_hex = self.quick_slots[idx]
        if not cmd:
            return
        try:
            raw = self.encode_payload(cmd, is_hex)
        except ValueError:
            self._append_sys(f"快捷指令 #{idx+1} HEX 格式无效")
            return
        self.send_payload(raw)

    def export_slots(self):
        path, _ = W.QFileDialog.getSaveFileName(
            self, "导出快捷指令集", "serial_commands.json", "JSON (*.json)",
            options=W.QFileDialog.Option.DontUseNativeDialog,
        )
        if path:
            try:
                Path(path).write_text(json.dumps(self.quick_slots, ensure_ascii=False, indent=2), encoding="utf-8")
                self._append_sys("已导出快捷指令集：" + path)
            except OSError as exc:
                self._append_sys(str(exc))

    def import_slots(self):
        path, _ = W.QFileDialog.getOpenFileName(
            self, "导入快捷指令集", "", "JSON (*.json)",
            options=W.QFileDialog.Option.DontUseNativeDialog,
        )
        if path:
            try:
                items = json.loads(Path(path).read_text(encoding="utf-8"))
                for i, item in enumerate(items[: len(self.quick_slots)]):
                    if isinstance(item, list) and len(item) == 3:
                        self.quick_slots[i] = [str(item[0])[:16], str(item[1])[:256], bool(item[2])]
                        self._sync_slot_row(i)
                self._append_sys("已导入快捷指令集")
            except (OSError, ValueError) as exc:
                self._append_sys("导入失败：" + str(exc))

    # ---------- Log rendering, saving & file send ----------
    def _on_pause_toggled(self, paused: bool):
        self.pause_rx.setText("恢复显示" if paused else "暂停显示")
        if not paused:
            self.render_logs(force=True)

    def format_entry(self, entry) -> str:
        stamp, kind, data, _ = entry
        if kind == "系统":
            text = bytes(data).decode("utf-8", errors="replace")
        elif self.rx_hex.isChecked():
            text = bytes(data).hex(" ").upper()
        else:
            enc = ("utf-8", "gbk", "latin1")[self.encoding_box.currentIndex()]
            text = bytes(data).decode(enc, errors="replace").rstrip("\r\n")
        prefix = []
        if self.show_time.isChecked():
            prefix.append(f"[{stamp}]")
        if self.show_dir.isChecked():
            prefix.append(f"{kind:<2s}")
        return (" ".join(prefix) + ("  " if prefix else "") + text)

    def render_logs(self, force=False):
        if (not self.dirty and not force) or self.pause_rx.isChecked():
            return
        bar = self.log_view.verticalScrollBar()
        prev = bar.value()
        kw = self.filter_edit.text().strip().lower()
        lines = []
        total_chars = 0
        for entry in reversed(self.entries):
            line = self.format_entry(entry)
            if kw and kw not in line.lower():
                continue
            lines.append(line)
            total_chars += len(line)
            if total_chars >= 64000:
                break
        self.log_view.setPlainText("\n".join(reversed(lines)))
        bar.setValue(bar.maximum() if self.autoscroll.isChecked() else prev)
        self.dirty = False

    def clear_logs(self):
        self.rx_tail = None
        self.entries.clear()
        self.log_view.clear()

    def reset_counters(self):
        self.rx_bytes = self.tx_bytes = self.rx_frames = self.tx_frames = self.dropped = 0
        self._refresh_status_bar()

    def save_log_file(self):
        default_name = "SerialLog_" + datetime.now().strftime("%Y%m%d_%H%M%S") + ".txt"
        path, _ = W.QFileDialog.getSaveFileName(
            self, "保存串口接收日志", default_name, "文本日志 (*.txt);;所有文件 (*)",
            options=W.QFileDialog.Option.DontUseNativeDialog,
        )
        if path:
            try:
                content = "\n".join(self.format_entry(e) for e in self.entries)
                Path(path).write_text(content, encoding="utf-8")
                self._append_sys("已保存日志至 " + path)
            except OSError as exc:
                self._append_sys("保存失败：" + str(exc))

    def send_file_dialog(self):
        if not self.connected:
            self._append_sys("请先打开串口后再发送文件")
            return
        path, _ = W.QFileDialog.getOpenFileName(
            self, "选择要发送的文件", "", "所有文件 (*)",
            options=W.QFileDialog.Option.DontUseNativeDialog,
        )
        if not path:
            return
        try:
            raw = Path(path).read_bytes()
        except OSError as exc:
            self._append_sys("读取文件失败：" + str(exc))
            return
        if len(raw) > 256_000:
            self._append_sys("单次发送文件上限为 256 KB")
            return
        for off in range(0, len(raw), 4096):
            if not self.link.send(raw[off : off + 4096]):
                self._append_sys("发送队列已满，文件发送中断")
                return
        self._append_sys(f"已排队发送文件 {Path(path).name}（{len(raw):,} B）")

    # ---------- Periodic event pump ----------
    def tick(self):
        now = monotonic()
        if self.link:
            batch, lost = self.link.take()
            self.dropped += lost
            had_error = False
            for kind, stamp, data in batch:
                if kind == "open" and not self.closing:
                    self.connected = True
                    self._update_conn_ui()
                    self._append_sys(f"串口已打开：{data}")
                elif kind == "rx":
                    self._ingest_rx(data, stamp)
                elif kind == "tx":
                    self.tx_bytes += len(data)
                    self.tx_frames += 1
                    self.entries.append([self._stamp(), "TX", bytearray(data), True])
                    self.dirty = True
                elif kind == "error":
                    had_error = True
                    self._append_sys(f"串口异常：{data}")
                elif kind == "closed":
                    unexpected = not self.closing and had_error
                    self.link = None
                    self.connected = self.closing = False
                    self.cycle_chk.setChecked(False)
                    if unexpected and self.auto_reconnect.isChecked() and self.reconnect_opts:
                        self.reconnect_at = now + 1.5
                    self._update_conn_ui()
                    break
        elif self.reconnect_at is not None and now >= self.reconnect_at:
            self.reconnect_at = None
            self.link = self._create_link(self.reconnect_opts)
            self.link.start()
            self._update_conn_ui()

        # Close idle timeout-based RX frame
        if (
            self.rx_tail is not None
            and self.timeout_break.isChecked()
            and (now - self.last_rx_mono) * 1000.0 >= self.frame_ms.value()
        ):
            self.rx_tail[3] = True
            self.rx_tail = None

        if now - self.last_render >= 0.08 and self.dirty:
            self.last_render = now
            self.render_logs()

        if now - self.rate_start >= 1.0:
            self.bps = round(self.rate_bytes / (now - self.rate_start))
            self.rate_start, self.rate_bytes = now, 0
            self._refresh_status_bar()
        elif self.dirty:
            self._refresh_status_bar()

    def _refresh_status_bar(self):
        enc_name = ("UTF-8", "GBK", "Latin-1")[self.encoding_box.currentIndex()]
        mode_txt = "HEX" if self.rx_hex.isChecked() else enc_name
        self.status_bar.setText(
            f"{self.conn_badge.text()}     RX  {self.rx_bytes:,} B ({self.rx_frames} 帧)     "
            f"TX  {self.tx_bytes:,} B ({self.tx_frames} 帧)     速率  {self.bps:,} B/s     "
            f"显示模式  {mode_txt}     溢出丢弃  {self.dropped} B"
        )
