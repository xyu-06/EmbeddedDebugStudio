"""Reusable Classic Bluetooth SPP picker & PIN pairing dialog for all workbenches."""
from queue import Empty, Queue
from threading import Thread

from PySide6 import QtCore as C, QtGui, QtWidgets as W

from bluetooth_link import (
    BluetoothDevice,
    discover_bluetooth_devices,
    format_bt_port_token,
    open_windows_bluetooth_settings,
    pair_bluetooth_device,
)


class BluetoothPickerDialog(W.QDialog):
    """Scan, pair (PIN), and select a Classic Bluetooth SPP endpoint or mapped COM port."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("🔵 蓝牙设备扫描、配对与 SPP 选择")
        self.resize(650, 380)
        self._devices: list[BluetoothDevice] = []
        self._results: Queue = Queue()
        self._busy = False
        self.selected_port_text: str | None = None

        lay = W.QVBoxLayout(self)
        lay.setSpacing(8)

        bar = W.QHBoxLayout()
        self.scan_btn = W.QPushButton("🔍 搜索附近蓝牙")
        self.scan_btn.setObjectName("primary")
        self.scan_btn.clicked.connect(lambda _=False: self.scan(True))
        self.refresh_btn = W.QPushButton("刷新已配对")
        self.refresh_btn.clicked.connect(lambda _=False: self.scan(False))
        self.pin_edit = W.QLineEdit("1234")
        self.pin_edit.setMaximumWidth(75)
        self.pin_edit.setPlaceholderText("PIN")
        self.pair_btn = W.QPushButton("🔗 配对选中设备")
        self.pair_btn.clicked.connect(self.pair_selected)
        self.sys_btn = W.QPushButton("系统蓝牙设置")
        self.sys_btn.clicked.connect(open_windows_bluetooth_settings)
        for w in (self.scan_btn, self.refresh_btn, W.QLabel("PIN:"), self.pin_edit, self.pair_btn, self.sys_btn):
            bar.addWidget(w)
        bar.addStretch()
        lay.addLayout(bar)

        self.table = W.QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["设备名称", "蓝牙地址 (MAC)", "配对 / 连接状态", "传出虚拟 COM"])
        self.table.verticalHeader().setVisible(False)
        self.table.setSelectionBehavior(W.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table.setSelectionMode(W.QAbstractItemView.SelectionMode.SingleSelection)
        self.table.setEditTriggers(W.QAbstractItemView.EditTrigger.NoEditTriggers)
        hdr = self.table.horizontalHeader()
        hdr.setSectionResizeMode(0, W.QHeaderView.ResizeMode.Stretch)
        hdr.setSectionResizeMode(1, W.QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(2, W.QHeaderView.ResizeMode.ResizeToContents)
        hdr.setSectionResizeMode(3, W.QHeaderView.ResizeMode.Stretch)
        self.table.itemSelectionChanged.connect(self._sync_buttons)
        self.table.itemDoubleClicked.connect(lambda _: self._on_double_click())
        lay.addWidget(self.table, 1)

        self.status_lbl = W.QLabel("正在读取已配对蓝牙设备…（若找不到模块，请点击【🔍 搜索附近蓝牙】）")
        self.status_lbl.setStyleSheet("color:#5b7083;font-size:9pt;")
        self.status_lbl.setWordWrap(True)
        lay.addWidget(self.status_lbl)

        bottom = W.QHBoxLayout()
        self.use_spp_btn = W.QPushButton("⚡ 选择蓝牙直连 (SPP)")
        self.use_spp_btn.setObjectName("accent")
        self.use_spp_btn.clicked.connect(self.choose_spp)
        self.use_com_btn = W.QPushButton("使用映射 COM 口")
        self.use_com_btn.clicked.connect(self.choose_com)
        cancel_btn = W.QPushButton("取消")
        cancel_btn.clicked.connect(self.reject)
        bottom.addStretch()
        bottom.addWidget(self.use_spp_btn)
        bottom.addWidget(self.use_com_btn)
        bottom.addWidget(cancel_btn)
        lay.addLayout(bottom)

        self._poll = C.QTimer(self)
        self._poll.setInterval(80)
        self._poll.timeout.connect(self._drain_results)
        self._poll.start()

        self._sync_buttons()
        self.scan(False)

    def _selected(self) -> BluetoothDevice | None:
        row = self.table.currentRow()
        if 0 <= row < len(self._devices):
            return self._devices[row]
        return None

    def _sync_buttons(self):
        dev = self._selected()
        self.scan_btn.setEnabled(not self._busy)
        self.refresh_btn.setEnabled(not self._busy)
        self.pair_btn.setEnabled((not self._busy) and (dev is not None) and (not dev.authenticated))
        self.use_spp_btn.setEnabled((not self._busy) and (dev is not None) and dev.authenticated)
        self.use_com_btn.setEnabled((not self._busy) and (dev is not None) and bool(dev.com_port))

    def _populate(self, devices: list[BluetoothDevice], msg: str | None = None):
        prev = self._selected().address if self._selected() else None
        self._devices = list(devices)
        self.table.setRowCount(len(self._devices))
        sel = 0
        for row_i, dev in enumerate(self._devices):
            if prev and dev.address == prev:
                sel = row_i
            st = "● 系统已连" if dev.connected else ("✓ 已配对" if dev.authenticated else "○ 未配对")
            com_txt = f"{dev.com_port}（传出 SPP）" if dev.com_port else "可直连 SPP（未分配 COM）"
            for col_i, txt in enumerate((dev.name, dev.address, st, com_txt)):
                item = W.QTableWidgetItem(txt)
                if col_i == 2:
                    item.setForeground(QtGui.QColor("#188467" if dev.authenticated else "#8d691e"))
                self.table.setItem(row_i, col_i, item)
        if self._devices:
            self.table.selectRow(sel)
        if msg is not None:
            self.status_lbl.setText(msg)
        else:
            paired = sum(1 for d in self._devices if d.authenticated)
            self.status_lbl.setText(f"共列出 {len(self._devices)} 台蓝牙设备（{paired} 台已配对）；双击已配对设备可直接选定。")
        self._sync_buttons()

    def scan(self, inquiry: bool):
        if self._busy:
            return
        self._busy = True
        self._sync_buttons()
        self.status_lbl.setText(
            "正在搜索附近经典蓝牙设备（约 3~5 秒，请确认 HC-05 处于上电可发现状态）…"
            if inquiry
            else "正在读取 Windows 已配对蓝牙设备…"
        )

        def worker():
            try:
                devs = discover_bluetooth_devices(inquiry=inquiry, timeout_units=3)
                self._results.put(("scan", devs, None))
            except Exception as exc:
                self._results.put(("scan", [], str(exc)))

        Thread(target=worker, daemon=True).start()

    def pair_selected(self):
        dev = self._selected()
        if dev is None or self._busy:
            return
        pin = self.pin_edit.text().strip()
        hwnd = int(self.winId()) if self.isVisible() else 0
        self._busy = True
        self._sync_buttons()
        self.status_lbl.setText(f"正在与 {dev.name} ({dev.address}) 配对（PIN={pin or '系统弹窗'}）…")

        def worker():
            ok, msg = pair_bluetooth_device(dev.address, pin=pin, hwnd=hwnd)
            devs = discover_bluetooth_devices(inquiry=False) if ok else None
            self._results.put(("pair", (ok, msg, devs), dev.address))

        Thread(target=worker, daemon=True).start()

    def _drain_results(self):
        while True:
            try:
                kind, payload, extra = self._results.get_nowait()
            except Empty:
                break
            self._busy = False
            if kind == "scan":
                if extra:
                    self._populate([], f"扫描失败：{extra}")
                else:
                    self._populate(payload)
            elif kind == "pair":
                ok, msg, devs = payload
                if ok and devs is not None:
                    self._populate(devs, msg)
                else:
                    self.status_lbl.setText(msg)
                    self._sync_buttons()

    def _on_double_click(self):
        dev = self._selected()
        if dev is None:
            return
        if dev.authenticated:
            self.choose_spp()
        else:
            self.pair_selected()

    def choose_spp(self):
        dev = self._selected()
        if dev is None:
            return
        if not dev.authenticated:
            self.pair_selected()
            return
        self.selected_port_text = format_bt_port_token(dev.address, dev.name)
        self.accept()

    def choose_com(self):
        dev = self._selected()
        if dev is None or not dev.com_port:
            return
        self.selected_port_text = dev.com_port
        self.accept()
