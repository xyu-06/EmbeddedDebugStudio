from functools import partial

from PySide6 import QtCore as C, QtGui, QtWidgets as W

from engine import DEFAULTS, KEYS, LIMIT, LOOPS, MAX_CHANNELS, MODELS
from generator import SHAPES
from scope import COLORS, REF_COLORS, Scope
from version import APP_TITLE_ZH, APP_VERSION


def button(text, fn, role=""):
    w = W.QPushButton(text)
    w.setCursor(C.Qt.CursorShape.PointingHandCursor)
    if role:
        w.setObjectName(role)
    w.clicked.connect(lambda checked=False: fn())
    return w


def combo(values, idx=0):
    w = W.QComboBox()
    w.setFocusPolicy(C.Qt.FocusPolicy.StrongFocus)
    w.addItems(values)
    w.setCurrentIndex(idx)
    return w


class ParameterSpin(W.QDoubleSpinBox):
    def wheelEvent(self,event):
        le = self.lineEdit()
        if not (self.hasFocus() or (le is not None and le.hasFocus())):
            event.ignore()
            return
        steps = event.angleDelta().y()
        if steps:
            multiplier = 10 if event.modifiers() & C.Qt.KeyboardModifier.ShiftModifier else 1
            self.stepBy((1 if steps>0 else -1)*multiplier)
            event.accept()
        else:
            super().wheelEvent(event)


def number(value, low, high, step=.1, decimals=4, parameter=False):
    w = ParameterSpin() if parameter else W.QDoubleSpinBox()
    w.setFocusPolicy(C.Qt.FocusPolicy.StrongFocus)
    w.setDecimals(decimals)
    w.setRange(low, high)
    w.setSingleStep(step)
    w.setValue(value)
    w.setKeyboardTracking(False)
    w.setSizePolicy(W.QSizePolicy.Policy.Ignored,W.QSizePolicy.Policy.Fixed)
    w.setMinimumWidth(65)
    return w


def check(text, checked):
    w = W.QCheckBox(text)
    w.setChecked(checked)
    return w


def row(*widgets):
    r = W.QHBoxLayout()
    r.setContentsMargins(0, 0, 0, 0)
    r.setSpacing(5)
    for w in widgets:
        r.addWidget(w)
    return r


def color_button(window, slot, label):
    chip = button(label, partial(window.choose_color, slot))
    chip.setAccessibleName("设置" + label + "颜色" if label else f"设置 CH{slot+1} 颜色")
    chip.setFixedWidth(56 if label else 36)
    window.color_buttons.append(chip)
    return chip


def card(side, title, opened):
    box = W.QFrame()
    box.setObjectName("card")
    outer = W.QVBoxLayout(box)
    outer.setSpacing(0)
    outer.setContentsMargins(0, 0, 0, 0)
    toggle = W.QToolButton()
    toggle.setText(title)
    toggle.setCheckable(True)
    toggle.setToolButtonStyle(C.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
    toggle.setSizePolicy(W.QSizePolicy.Policy.Expanding, W.QSizePolicy.Policy.Fixed)
    toggle.setObjectName("section")
    contents = W.QWidget()
    form = W.QFormLayout(contents)
    form.setContentsMargins(10, 3, 10, 11)
    form.setSpacing(6)
    form.setFieldGrowthPolicy(W.QFormLayout.FieldGrowthPolicy.AllNonFixedFieldsGrow)
    def change(value):
        win = box.window()
        if box.isVisible() and hasattr(win,"workbench") and not win.reduce_motion.isChecked():
            animation = C.QPropertyAnimation(contents,b"maximumHeight",contents)
            previous = getattr(contents,"fold_animation",None)
            if previous:
                previous.stop()
                previous.deleteLater()
            contents.fold_animation = animation
            start = contents.height() if contents.isVisible() else 0
            contents.setVisible(True)
            animation.setDuration(160)
            animation.setStartValue(start)
            animation.setEndValue(contents.sizeHint().height() if value else 0)
            animation.setEasingCurve(C.QEasingCurve.Type.OutCubic)
            def finish():
                contents.setVisible(value)
                contents.setMaximumHeight(16777215)
            animation.finished.connect(finish)
            animation.start()
        else:
            contents.setMaximumHeight(16777215)
            contents.setVisible(value)
        toggle.setArrowType(C.Qt.ArrowType.DownArrow if value else C.Qt.ArrowType.RightArrow)
    toggle.toggled.connect(change)
    toggle.setChecked(opened)
    change(opened)
    outer.addWidget(toggle)
    outer.addWidget(contents)
    side.addWidget(box)
    return form, toggle


def build(w):
    w.setWindowTitle(f"{APP_TITLE_ZH} v{APP_VERSION} · PID / 串口 / IMU / 蓝牙遥控")
    w.resize(1280, 820)
    w.setMinimumSize(920, 590)

    # Top Menu Bar for module selection & view options
    menubar = w.menuBar()
    w.module_menu = menubar.addMenu("调试模块(&M)")
    view_menu = menubar.addMenu("视图与扩展(&V)")
    w.dark_action = view_menu.addAction("暗色护眼主题")
    w.dark_action.setCheckable(True)
    view_menu.addSeparator()
    view_menu.addAction("模块扩展接口说明…", w.show_extension_guide)

    root = W.QWidget()
    w.setCentralWidget(root)
    shell = W.QVBoxLayout(root)
    shell.setSpacing(0)
    shell.setContentsMargins(0, 0, 0, 0)

    # Unified top workbench module switcher bar
    w.nav_bar = W.QFrame()
    w.nav_bar.setObjectName("navBar")
    nav = W.QHBoxLayout(w.nav_bar)
    nav.setContentsMargins(12, 5, 12, 5)
    nav.setSpacing(6)
    brand = W.QLabel("⚡ 综合调试台")
    brand.setObjectName("navBrand")
    nav.addWidget(brand)
    nav.addSpacing(8)
    w.nav_buttons_layout = W.QHBoxLayout()
    w.nav_buttons_layout.setContentsMargins(0, 0, 0, 0)
    w.nav_buttons_layout.setSpacing(6)
    nav.addLayout(w.nav_buttons_layout)
    nav.addStretch()
    w.global_ports_label = W.QLabel("就绪 · 可按上方页签或 Ctrl+1/2/3 切换工作台")
    w.global_ports_label.setObjectName("navStatus")
    nav.addWidget(w.global_ports_label)
    shell.addWidget(w.nav_bar)

    w.module_stack = W.QStackedWidget()
    shell.addWidget(w.module_stack, 1)

    # Page 0: Existing PID Tuner Workbench (100% preserved)
    w.pid_page = W.QWidget()
    main = W.QVBoxLayout(w.pid_page)
    main.setSpacing(0)
    main.setContentsMargins(0, 0, 0, 0)
    w.module_stack.addWidget(w.pid_page)

    top = W.QFrame()
    top.setObjectName("top")
    bar = W.QHBoxLayout(top)
    bar.setContentsMargins(10, 7, 10, 7)
    w.connect_btn = button("连接串口", w.toggle_connection, "primary")
    w.pause_btn = button("暂停", w.toggle_freeze)
    w.record_btn = button("录制 CSV", w.record)
    w.stop_btn = button("急停", w.stop_device, "danger")
    w.badge = W.QLabel("未连接")
    iface_btn = button("🧩 接口代码与说明", lambda: w.open_firmware_builder(0))
    iface_btn.setToolTip("跳转至【🧩 接口构建】快速配置并复制 PID 串口接口代码（STM32 HAL / 标准库 SPL / ESP32·Arduino）及协议说明")
    for b in [w.connect_btn, w.pause_btn, button("清空波形", w.clear_plot), w.record_btn,
              button("快照 PNG", w.png), button("导出视图 CSV", w.export_view),
              button("打开 CSV", w.import_csv), button("🩺 信号体检", w.open_diagnostics),
              iface_btn]:
        bar.addWidget(b)
    bar.addWidget(w.badge)
    bar.addStretch()
    w.preset = combo(["调参","分析","终端"])
    bar.addWidget(w.preset)
    bar.addWidget(w.stop_btn)
    main.addWidget(top)

    w.horizontal = W.QSplitter()
    main.addWidget(w.horizontal, 1)
    scroll = W.QScrollArea()
    scroll.setFrameShape(W.QFrame.Shape.NoFrame)
    scroll.setWidgetResizable(True)
    scroll.setMinimumWidth(290)
    scroll.setMaximumWidth(520)
    sidebar = W.QWidget()
    side = W.QVBoxLayout(sidebar)
    side.setSpacing(6)
    side.setContentsMargins(7, 7, 7, 7)
    scroll.setWidget(sidebar)
    w.horizontal.addWidget(scroll)
    w.cards = []
    def section(title, opened):
        form, toggle = card(side, title, opened)
        w.cards.append(toggle)
        return form

    f = section("连接设置", True)
    w.source = combo(["串口设备", "仿真练习"])
    f.addRow("数据源", w.source)
    w.port = combo([])
    w.port.setEditable(True)
    w.refresh_btn = button("刷新", w.refresh_ports)
    f.addRow("串口", row(w.port, w.refresh_btn))
    w.baud = combo(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600"], 4)
    w.baud.setEditable(True)
    f.addRow("波特率", w.baud)
    w.model = combo(list(MODELS))
    w.model.setToolTip("倒立摆：含 25 ms 执行器滞后、传感器底噪，D 项作用于角速度，倾角超过 45° 自动停机")
    f.addRow("仿真对象", w.model)
    w.nudge_btn = button("👋 推一下（施加倾角扰动）", w.nudge_pendulum)
    w.nudge_btn.setToolTip("向倒立摆仿真模型施加 ±22°/s 外力冲击，测试抗扰与自恢复能力")
    f.addRow(w.nudge_btn)
    w.auto_reconnect = check("断线后自动重连（只读回参数，不自动启动）", True)
    f.addRow(w.auto_reconnect)
    w.serial_form = f

    f = section("串口与数据格式", False)
    w.bits, w.parity, w.stops = combo(["5","6","7","8"],3), combo(["无","偶","奇"]), combo(["1","1.5","2"])
    w.flow = combo(["无", "RTS/CTS", "XON/XOFF"])
    w.time_mode = combo(["电脑接收时间", "固定上报周期"])
    w.rate = number(100, 1, 5000, 10, 0)
    for label, widget in [("数据位",w.bits),("校验",w.parity),("停止位",w.stops),("流控",w.flow),
                          ("时间来源",w.time_mode),("上报频率 Hz",w.rate)]:
        f.addRow(label,widget)
    f.addRow(W.QLabel(f"文本 A / FireWater　·　每行 1–{MAX_CHANNELS} 个数值"))

    f = w.channel_form = section("通道", True)
    w.enabled_ch, w.name_edits = [], []
    w.color_buttons = []
    for i, color in enumerate(COLORS[:MAX_CHANNELS]):
        enabled = W.QPushButton()
        enabled.setCheckable(True)
        enabled.setChecked(True)
        enabled.setObjectName("channelChip")
        enabled.setToolTip("点击显示或隐藏曲线")
        controls = W.QWidget()
        controls.setLayout(row(W.QLabel(f"CH{i+1}"),color_button(w,i,"")))
        name = W.QLineEdit()
        name.setPlaceholderText(f"CH{i+1}")
        name.setMaxLength(32)
        f.addRow(controls,name)
        w.enabled_ch.append(enabled)
        w.name_edits.append(name)
    w.no_channels = W.QLabel("等待串口或仿真数据")
    f.addRow(w.no_channels)
    w.ref_rows = []
    for i, color in enumerate(REF_COLORS):
        name = W.QLineEdit()
        name.setMaxLength(32)
        name.setPlaceholderText("参考线名称")
        value = number(0, -LIMIT, LIMIT, 1, 3)
        value.setSizePolicy(W.QSizePolicy.Policy.Fixed, W.QSizePolicy.Policy.Fixed)
        value.setFixedWidth(88)
        remove = button("移除", partial(w.remove_reference, i))
        tag = color_button(w, MAX_CHANNELS + i, "固定")
        editors = W.QWidget()
        editors.setLayout(row(name,value,remove))
        f.addRow(tag,editors)
        w.ref_rows.append((tag,name,value,remove))
    w.add_ref_btn = button("＋ 添加固定参考线", w.add_reference)
    f.addRow(w.add_ref_btn)
    w.calibrate_btn = button("换算、单位与坐标轴",w.edit_channels)
    f.addRow(w.calibrate_btn)

    f = section("绘图设置", False)
    w.window = number(10,.001,86400,1,3)
    w.window.setSizePolicy(W.QSizePolicy.Policy.Expanding,W.QSizePolicy.Policy.Fixed)
    presets = W.QToolButton()
    presets.setText("常用")
    presets.setPopupMode(W.QToolButton.ToolButtonPopupMode.InstantPopup)
    menu = W.QMenu(presets)
    for seconds in (1,5,10,30,60):
        menu.addAction(f"{seconds} s").triggered.connect(lambda _, n=seconds: w.window.setValue(n))
    presets.setMenu(menu)
    w.yscale = combo(["自动（含零点）", "固定范围", "自动（放大波动）"])
    w.ymin = number(-100,-LIMIT,LIMIT,1,2)
    w.ymax = number(100,-LIMIT,LIMIT,1,2)
    w.grid, w.legend = check("网格",True), check("图例", True)
    w.line = number(1,.5,10,.25,2)
    w.x_div = number(0,0,86400,.1,3)
    w.y_div = number(0,0,LIMIT,.1,4)
    for axis_box in (w.x_div,w.y_div):
        axis_box.setSpecialValueText("自动")
        axis_box.setToolTip("输入每个主刻度的间隔；0 为自动。标签过密时按整数倍合并。")
    w.render_mode = combo(["像素均值（看趋势）","原始采样（看尖峰）"])
    f.addRow("时间窗口 s",row(w.window,presets))
    for label, widget in [("X 每格 s",w.x_div),("纵轴",w.yscale),("Y 最小",w.ymin),("Y 最大",w.ymax),
                          ("Y 每格",w.y_div),("密集波形",w.render_mode),("线宽",w.line)]:
        f.addRow(label,widget)
    f.addRow(row(w.grid, w.legend))
    w.show_stats = check("显示视窗统计",True)
    f.addRow(w.show_stats)
    w.dark = check("暗色主题",False)
    w.reduce_motion = check("减少动画",False)
    f.addRow(row(w.dark,w.reduce_motion))

    w.pid_panel = W.QFrame()
    w.pid_panel.setObjectName("pidPanel")
    w.pid_panel.setMinimumWidth(216)
    w.pid_panel.setMaximumWidth(350)
    panel = W.QVBoxLayout(w.pid_panel)
    panel.addWidget(W.QLabel("<b>PID 调参</b>"))
    f = W.QFormLayout()
    panel.addLayout(f)
    w.group = combo(["直立环  ANG","速度环  SPD","转向环  YAW","辅助环  AUX"])
    f.addRow("参数组",w.group)
    w.address = combo(["分组命令  @ANG.KP", "单环命令  @KP"])
    f.addRow("寻址方式",w.address)
    w.slider_toggle = check("滑块微调", False)
    w.step_gear = combo(["步长 0.001", "步长 0.01", "步长 0.1", "步长 1", "步长 10"], 2)
    f.addRow(row(w.slider_toggle, w.step_gear))
    w.fields = {}
    w.sliders = {}
    w.pid_form = f
    for key in KEYS:
        field = number(DEFAULTS[key], -LIMIT, LIMIT, .1, parameter=True)
        w.fields[key] = field
        tile = QtGui.QPixmap(20,18)
        tile.fill(C.Qt.GlobalColor.transparent)
        painter = QtGui.QPainter(tile)
        painter.setPen(QtGui.QColor("#5684aa"))
        painter.drawLine(16,4,16,10)
        painter.drawLine(16,10,4,10)
        painter.drawLine(4,10,8,6)
        painter.drawLine(4,10,8,14)
        painter.end()
        action = field.lineEdit().addAction(QtGui.QIcon(tile),W.QLineEdit.ActionPosition.TrailingPosition)
        action.setToolTip("下发此参数（回车）")
        action.triggered.connect(lambda _,k=key: w.send_field(k))
        f.addRow(key,field)
        slider = W.QSlider(C.Qt.Orientation.Horizontal)
        slider.setRange(0, 1000)
        slider.setValue(500)
        slider.setFixedHeight(15)
        slider.setToolTip(f"拖动连续调节 {key}（以当前值为中心微调）")
        w.sliders[key] = slider
        f.addRow("", slider)
        f.setRowVisible(slider, False)
    w.auto = check("编辑完成后自动发送", False)
    f.addRow(w.auto)
    f.addRow(row(button("读取", w.read_params),button("发送本组",w.send_group),button("启动",w.run_device)))
    f.addRow(row(button("清积分", w.reset_integral),button("保存预设",w.save_preset),button("载入预设",w.load_preset)))
    w.undo_btn = button("撤销",w.commands.undo_parameter)
    w.undo_btn.setToolTip("撤销上次下发的参数")
    f.addRow(row(button("参数保护",w.edit_limits),w.undo_btn,button("C 代码",w.export_c_code)))
    w.autotune_btn = button("⚡ 自动整定 PID…",w.open_autotune,"accent")
    w.autotune_btn.setToolTip("设定目标值后自动激励、评估波形并寻优 KP / KI / KD")
    f.addRow(w.autotune_btn)
    w.autotune_status = W.QLabel()
    w.autotune_status.setWordWrap(True)
    w.autotune_status.setStyleSheet("color:#1c5f4e;font-size:9pt;padding:2px 0;")
    w.autotune_status.hide()
    f.addRow(w.autotune_status)
    w.ack = W.QLabel("等待操作")
    w.ack.setWordWrap(True)
    f.addRow(w.ack)
    line = W.QFrame()
    line.setFrameShape(W.QFrame.Shape.HLine)
    line.setStyleSheet("color:#d3dae1")
    panel.addWidget(line)
    panel.addWidget(W.QLabel("<b>目标值发生器</b>"))
    g = W.QFormLayout()
    panel.addLayout(g)
    w.gen_shape = combo(list(SHAPES), 1)
    w.gen_amp = number(5, -LIMIT, LIMIT, 1, 3)
    w.gen_amp.setToolTip("相对启动时 TGT 输入框数值的偏移量，可为负")
    w.gen_period = number(2, .05, 3600, .5, 2)
    w.gen_rate = number(10, 1, 50, 1, 0)
    w.gen_rate.setToolTip("正弦、三角波每秒下发 TGT 的次数；阶跃、方波只在跳变时下发")
    for label, widget in [("波形", w.gen_shape), ("幅值", w.gen_amp), ("周期 s", w.gen_period), ("节拍 Hz", w.gen_rate)]:
        g.addRow(label, widget)
    w.gen_btn = button("启动发生器", w.toggle_generator)
    g.addRow(w.gen_btn)
    w.gen_status = W.QLabel("以 TGT 输入框为基准，停止后恢复")
    w.gen_status.setWordWrap(True)
    g.addRow(w.gen_status)
    panel.addStretch()

    f = section("阶跃与游标", True)
    w.target_ch, w.actual_ch = combo([]),combo([])
    f.addRow("目标 / 实际",row(w.target_ch,w.actual_ch))
    w.band = combo(["±2%","±5%"])
    f.addRow("调节误差带",w.band)
    w.metrics = W.QLabel("等待目标值发生阶跃")
    w.metrics.setWordWrap(True)
    f.addRow(w.metrics)
    w.overlays = check("在波形显示阶跃指标",True)
    f.addRow(w.overlays)
    w.pin_step_btn = button("📌 锁定对比底图", w.pin_step_ghost)
    w.clear_ghost_btn = button("清除底图", w.clear_step_ghost)
    w.clear_ghost_btn.setEnabled(False)
    f.addRow(row(w.pin_step_btn, w.clear_ghost_btn))
    f = section("单次触发捕获",False)
    w.trigger_ch = combo([])
    w.trigger_level = number(0,-LIMIT,LIMIT,.5,3)
    w.trigger_edge = combo(["上升沿","下降沿","任意边沿"])
    w.capture_seconds = number(2,.001,86400,.5,3)
    w.pre_percent = number(30,0,100,5,0)
    for label,widget in [("通道",w.trigger_ch),("电平",w.trigger_level),("边沿",w.trigger_edge),
                         ("捕获时间 s",w.capture_seconds),("预触发 %",w.pre_percent)]:
        f.addRow(label,widget)
    w.arm_btn = button("单次触发",w.capture.arm_capture)
    w.trigger_status = W.QLabel("等待设置")
    w.trigger_status.setWordWrap(True)
    f.addRow(w.arm_btn,w.trigger_status)
    side.addStretch()

    w.vertical = W.QSplitter(C.Qt.Orientation.Vertical)
    w.horizontal.addWidget(w.vertical)
    w.horizontal.addWidget(w.pid_panel)
    upper = w.plot_container = W.QWidget()
    up = W.QVBoxLayout(upper)
    up.setContentsMargins(0,0,0,0)
    up.setSpacing(0)
    w.chip_bar = W.QWidget()
    chips = row(*w.enabled_ch)
    chips.addStretch()
    w.live_btn = button("已暂停 · 返回实时（空格）",w.toggle_freeze)
    chips.addWidget(w.live_btn)
    w.chip_bar.setLayout(chips)
    up.addWidget(w.chip_bar)
    w.stop_banner = W.QLabel()
    w.stop_banner.setStyleSheet("background:#a83139;color:white;padding:5px")
    w.stop_banner.hide()
    up.addWidget(w.stop_banner)
    w.notice = W.QWidget()
    w.notice_text = W.QLabel()
    w.notice_text.setTextFormat(C.Qt.TextFormat.PlainText)
    w.retry_btn = button("重试",w.commands.retry_last)
    w.notice.setLayout(row(w.notice_text,w.retry_btn))
    w.notice.hide()
    up.addWidget(w.notice)
    w.scope = Scope()
    up.addWidget(w.scope,1)
    w.guide = W.QWidget()
    w.guide_text = W.QLabel("选择串口 → 连接设备 → 读取参数后启动")
    w.guide.setLayout(row(w.guide_text,button("试试仿真练习",w.try_simulation)))
    up.addWidget(w.guide)
    w.measure_text = W.QLabel("暂停后可缩放、拖动，单击波形放置 A、B 游标")
    w.measure_text.setObjectName("measure")
    w.measure_text.setTextFormat(C.Qt.TextFormat.PlainText)
    w.measure_text.setTextInteractionFlags(C.Qt.TextInteractionFlag.TextSelectableByMouse)
    w.measure_text.setWordWrap(True)
    up.addWidget(w.measure_text)
    w.terminal_toggle = W.QToolButton()
    w.terminal_toggle.setText("串口终端")
    w.terminal_toggle.setToolButtonStyle(C.Qt.ToolButtonStyle.ToolButtonTextBesideIcon)
    w.terminal_toggle.setCheckable(True)
    w.terminal_latest = W.QLabel()
    w.terminal_latest.setTextFormat(C.Qt.TextFormat.PlainText)
    w.terminal_latest.setSizePolicy(W.QSizePolicy.Policy.Ignored,W.QSizePolicy.Policy.Fixed)
    up.addLayout(row(w.terminal_toggle,w.terminal_latest))
    w.vertical.addWidget(upper)

    lower = w.terminal_panel = W.QWidget()
    lo = W.QVBoxLayout(lower)
    lo.setContentsMargins(6,3,6,5)
    lo.setSpacing(4)
    w.hex_mode, w.autoscroll = check("HEX 显示",False),check("自动滚动",True)
    tools = row(W.QLabel("串口终端"),w.hex_mode,w.autoscroll,button("清屏",w.clear_log))
    tools.addSpacing(8)
    tools.addWidget(W.QLabel("快捷指令:"))
    w.macro_btns = []
    for idx in range(5):
        mb = button("", partial(w.send_macro, idx))
        mb.setContextMenuPolicy(C.Qt.ContextMenuPolicy.CustomContextMenu)
        mb.customContextMenuRequested.connect(lambda _, i=idx: w.edit_macro(i))
        w.macro_btns.append(mb)
        tools.addWidget(mb)
    tools.addWidget(button("编辑快捷", lambda: w.edit_macro(0)))
    tools.addStretch()
    lo.addLayout(tools)
    w.terminal = W.QPlainTextEdit()
    w.terminal.setReadOnly(True)
    w.terminal.document().setMaximumBlockCount(400)
    w.terminal.setUndoRedoEnabled(False)
    lo.addWidget(w.terminal)
    w.raw = W.QLineEdit()
    w.raw.setPlaceholderText("发送文本命令，或输入空格分隔的十六进制字节")
    w.raw_hex = check("HEX 发送",False)
    lo.addLayout(row(w.raw,w.raw_hex,button("发送", w.send_raw)))
    w.vertical.addWidget(lower)
    w.vertical.setCollapsible(0,False)
    w.horizontal.setCollapsible(0,False)
    w.horizontal.setCollapsible(1,False)
    w.horizontal.setCollapsible(2,False)
    w.horizontal.setStretchFactor(1,1)
    w.horizontal.setSizes([300,740,240])
    w.vertical.setSizes([510,190])
    w.status = W.QLabel()
    w.status.setObjectName("status")
    main.addWidget(w.status)


STYLE = """
QMainWindow,QWidget { font-family:'Microsoft YaHei UI','Segoe UI'; font-size:10pt; color:#223449; }
QWidget { background:#f3f5f7; }
QMenuBar { background:#1c2b3a; color:#dce6f0; padding:2px 6px; border-bottom:1px solid #13202c; }
QMenuBar::item { background:transparent; padding:4px 10px; border-radius:3px; }
QMenuBar::item:selected { background:#2c435a; color:white; }
QMenu { background:white; color:#223449; border:1px solid #c6d0da; }
QMenu::item:selected { background:#eaf1fa; }
QFrame#navBar { background:#223449; border-bottom:1px solid #162433; }
QLabel#navBrand { background:transparent; color:#ffffff; font-weight:700; font-size:10.5pt; }
QLabel#navStatus { background:transparent; color:#b8cadb; font-size:9pt; }
QPushButton#navTab { background:#2d435c; color:#d7e3ef; border:1px solid #3d5672; border-radius:5px; padding:4px 14px; font-weight:600; }
QPushButton#navTab:hover { background:#395472; color:white; }
QPushButton#navTab:checked { background:#225fc1; color:white; border-color:#4b86e8; }
QFrame#top,QLabel#measure { background:white; border-bottom:1px solid #d3dae1; }
QFrame#top QLabel {background:white;}
QFrame#card {background:white;border:1px solid #cdd6df;border-radius:5px;}
QFrame#card QWidget {background:white;}
QToolButton#section {border:none;padding:7px 6px;text-align:left;font-weight:600;}
QPushButton,QComboBox,QLineEdit,QDoubleSpinBox {border:1px solid #c6d0da;border-radius:4px;background:white;padding:3px 5px;min-height:18px;}
QPushButton:hover {background:#eaf1fa;border-color:#8daaca;}
QPushButton:pressed {background:#d5e5f7;}
QPushButton#primary,QFrame#card QPushButton#primary {background:#225fc1;color:white;border:1px solid #225fc1;font-weight:600;padding:4px 12px;}
QPushButton#accent,QFrame#card QPushButton#accent {background:#197a63;color:white;border:1px solid #197a63;font-weight:600;padding:4px 10px;}
QPushButton#accent:hover,QFrame#card QPushButton#accent:hover {background:#146652;border-color:#146652;}
QPushButton#danger,QFrame#card QPushButton#danger {background:#ba333b;color:white;border:1px solid #ba333b;font-weight:600;padding:4px 18px;}
QPushButton:disabled,QWidget:disabled {color:#929ba5;}
QPlainTextEdit {background:white;border:1px solid #d5dce3;font:10pt 'Consolas','Microsoft YaHei UI';}
QLabel#measure,QLabel#status {padding:4px 10px;color:#52687b;font-size:9pt;}
QSplitter::handle {background:#d6dde4;width:2px;height:2px;}
"""

