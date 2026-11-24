import csv
from collections import deque
from autotune_ui import AutoTuneController
from device_session import DeviceSession
from capture import CaptureController
from configuration import DEFAULT_RULES, valid_rule, calibration_dialog, limits_dialog
from workbench import Workbench
from datetime import datetime
import json
import os
from pathlib import Path
import sys
from time import monotonic

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore as C, QtGui, QtWidgets as W
from serial.tools import list_ports

from engine import DEFAULTS, KEYS, LIMIT, LOOPS, Lines, Simulator, step_metrics
from generator import SHAPES, TargetGenerator
from layout_ui import STYLE, build, number
from scope import COLORS, REF_COLORS
from storage import History, PARAM_COLS, csv_header, csv_row
from transport import SerialLink


class Window(W.QMainWindow):
    def __init__(self, config=None):
        super().__init__()
        self.setAttribute(C.Qt.WidgetAttribute.WA_DeleteOnClose)
        base = Path(C.QStandardPaths.writableLocation(C.QStandardPaths.StandardLocation.AppConfigLocation))
        self.config = Path(config) if config else base / "settings.ini"
        self.config.parent.mkdir(parents=True, exist_ok=True)
        self.settings = C.QSettings(str(self.config), C.QSettings.Format.IniFormat)
        self.link = self.sim = None
        self.connected = self.closing = False
        self.commands = DeviceSession(self)
        self.autotuner = AutoTuneController(self)
        self.ping_capable = self.send_heartbeat = False
        self.next_ping = 0.
        self.generator = None
        self.reconnect_at = self.reconnect_opts = None
        self.reconnecting = False
        self.reconnect_attempts = 0
        self.field_errors = set()
        self.undo_history = deque(maxlen=25)
        self.rollback = None
        self.retry_command = None
        self.connection_error = False
        self.buffer = History(channels=8)
        self.ref_active = [False] * 4
        self.scales = [1.] * 4
        self.units = [""] * 4
        self.right_channel = -1
        self.follow = [""] * 4
        self.rules = {group:DEFAULT_RULES.copy() for group in LOOPS}
        self.frozen_count = 0
        self.channel_preferences = [None,None]
        self.colors = list(COLORS + REF_COLORS)
        self.capture = CaptureController(self)
        self.drafts = {g: dict(DEFAULTS) for g in LOOPS}
        self.confirmed = {}
        self.param_tuple = ("",) * 16
        self.parser = Lines()
        self.logs = deque(maxlen=400)
        self.rx_log_tail = None
        self.log_dirty = False
        self.deferred_marks = deque(maxlen=100)
        self.record_file = self.writer = None
        self.frozen_params = None
        self.rx = self.tx = self.bad = self.count = self.frames = self.dropped = 0
        self.rate_frames = self.fps = 0
        self.epoch = self.rate_start = monotonic()
        self.sim_steps = 0
        self.latest = 0.
        self.last_paint = -1
        self.last_metrics = 0.
        self.loading = True
        build(self)
        self.refresh_ports()
        self.restore()
        self.last_group = self.selected()
        self.load_fields()
        self.loading = False
        self.source.currentIndexChanged.connect(self.source_ui)
        self.group.currentIndexChanged.connect(self.change_group)
        self.address.currentIndexChanged.connect(self.address_ui)
        self.raw.returnPressed.connect(self.send_raw)
        self.hex_mode.toggled.connect(lambda: self.render_log(force=True))
        self.scope.measure = self.measure
        for name in self.name_edits:
            name.editingFinished.connect(self.channel_names)
        for ch in self.enabled_ch:
            ch.toggled.connect(self.plot_options)
        for widget in [self.legend, self.grid]:
            widget.toggled.connect(self.plot_options)
        self.yscale.currentIndexChanged.connect(self.plot_options)
        self.render_mode.currentIndexChanged.connect(self.plot_options)
        for widget in [self.window,self.ymin,self.ymax,self.line,self.x_div,self.y_div]:
            widget.valueChanged.connect(self.plot_options)
        self.show_stats.toggled.connect(self.measure)
        for idx,widget in enumerate((self.target_ch,self.actual_ch)):
            widget.currentIndexChanged.connect(lambda _, role=idx: self.select_channel(role))
        self.band.currentIndexChanged.connect(self.analyze)
        for idx,(_,name,value,_) in enumerate(self.ref_rows):
            name.editingFinished.connect(self.channel_names)
            value.valueChanged.connect(lambda _, slot=idx: self.reference_changed(slot))
        for key, field in self.fields.items():
            field.valueChanged.connect(lambda value, k=key: self.edited(k, value))
            field.lineEdit().returnPressed.connect(lambda k=key: self.enter_field(k))
        self.set_auto_names()
        self.refresh_channels()
        self.scope.reset()
        self.workbench = Workbench(self)
        self.preset.setCurrentIndex(self.settings.value("layoutPreset",0,type=int))
        if self.preset.currentIndex()!=0:
            self.workbench.layout(self.preset.currentIndex())
        self.plot_options()
        self.source_ui()
        self.address_ui()
        self.timer = C.QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.tick)
        self.timer.start()
        self.update_status()

    def try_simulation(self):
        if self.connected or self.link:
            return
        self.source.setCurrentIndex(1)
        self.toggle_connection()
        self.sample(0.,self.sim.step(self.selected()))
        self.run_device()

    def selected(self):
        return LOOPS[self.group.currentIndex()]

    def names(self):
        names = [n.text().strip() or n.placeholderText() for n in self.name_edits]
        return [f"{n} [{u}]" if u else n for n,u in zip(names,self.units)]

    def edit_channels(self):
        if not self.writer:
            calibration_dialog(self)

    def edit_limits(self):
        limits_dialog(self)

    def configure_channels(self,scales,units,right_channel,follow):
        if self.writer:
            return
        self.capture.cancel_capture("通道设置已更改")
        for i,(old,new) in enumerate(zip(self.scales,scales),1):
            if old != new:
                for part in self.buffer.slices():
                    self.buffer.data[part,i] *= new/old
                if self.scope.frozen:
                    self.scope.data[:,i] *= new/old
        self.scales,self.units,self.right_channel,self.follow = scales,units,right_channel,follow
        self.channel_names()
        self.analyze()

    def set_auto_names(self):
        defaults = ("目标值","实际值","输出","误差") if self.source.currentIndex() else ("CH1","CH2","CH3","CH4")
        for field,name in zip(self.name_edits,defaults):
            field.setPlaceholderText(name)

    def ref_name(self, slot):
        return self.ref_rows[slot][1].text().strip() or f"参考线 {slot+1}"

    def active_refs(self):
        return [i for i,active in enumerate(self.ref_active) if active]

    def persist(self):
        self.save_draft()
        s = self.settings
        opts = {name:getattr(self,name).currentIndex() for name in
                ("bits","parity","stops","flow","time_mode","yscale","model","address","group","band","source","trigger_edge","render_mode","gen_shape")}
        opts.update({name:getattr(self,name).value() for name in ("rate","ymin","ymax","line","trigger_level","capture_seconds","pre_percent","x_div","y_div","gen_amp","gen_period","gen_rate")})
        opts.update({name:getattr(self,name).isChecked() for name in ("grid","legend","show_stats","auto","autoscroll","hex_mode","reduce_motion","dark","overlays","terminal_toggle","auto_reconnect")})
        s.setValue("layoutPreset",self.preset.currentIndex())
        opts.update(window_seconds=self.window.value(),colors=self.colors)
        opts.update(scales=self.scales, units=self.units, right_channel=self.right_channel,
                    follow=self.follow,rules=self.rules)
        opts.update(port=self.port.currentText(),baud=self.baud.currentText(),aliases=[w.text() for w in self.name_edits],
                    references=[(self.ref_name(i),r[2].value()) if self.ref_active[i] else None for i,r in enumerate(self.ref_rows)],
                    channel_preferences=self.channel_preferences,
                    channels=[c.isChecked() for c in self.enabled_ch],cards=[t.isChecked() for t in self.cards],
                    drafts=self.drafts,autotune=self.autotuner.cfg)
        s.setValue("options",json.dumps(opts,ensure_ascii=False))
        for key, value in (("geometry",self.saveGeometry()),("horizontal",self.horizontal.saveState()),("vertical",self.vertical.saveState())):
            s.setValue(key,value)
        s.sync()

    def restore(self):
        try:
            opts = json.loads(self.settings.value("options","{}"))
            for i,v in enumerate(opts.get("scales",[])[:4]):
                if isinstance(v,(float,int)) and 1e-6 <= abs(v) <= LIMIT:
                    self.scales[i] = v
            for i,v in enumerate(opts.get("units",[])[:4]):
                self.units[i] = str(v)[:16]
            for i,v in enumerate(opts.get("follow",[])[:4]):
                self.follow[i] = v if v in LOOPS else ""
            axis = opts.get("right_channel",-1)
            self.right_channel = axis if isinstance(axis,int) and -1<=axis<4 else -1
            for g,rules in opts.get("rules",{}).items():
                if g in LOOPS and isinstance(rules,dict):
                    self.rules[g].update({k:tuple(v) for k,v in rules.items() if k in KEYS and valid_rule(v)})
            for key in ("bits","parity","stops","flow","time_mode","yscale","model","address","group","band","source","trigger_edge","render_mode","gen_shape"):
                w = getattr(self,key)
                value = opts.get(key,w.currentIndex())
                if isinstance(value,int) and 0 <= value < w.count():
                    w.setCurrentIndex(value)
            old_index = opts.get("window",1)
            old_windows = (5,10,30,60)
            default_window = old_windows[old_index] if isinstance(old_index,int) and 0<=old_index<4 else 10
            self.window.setValue(float(opts.get("window_seconds",default_window)))
            for i,color in enumerate(opts.get("colors",[])[:8]):
                if isinstance(color,str) and QtGui.QColor.isValidColorName(color):
                    self.colors[i] = QtGui.QColor(color).name()
            for key in ("rate","ymin","ymax","line","trigger_level","capture_seconds","pre_percent","x_div","y_div","gen_amp","gen_period","gen_rate"):
                if key in opts:
                    getattr(self,key).setValue(float(opts[key]))
            for key in ("grid","legend","show_stats","auto","autoscroll","hex_mode","reduce_motion","dark","overlays","terminal_toggle","auto_reconnect"):
                if key in opts:
                    getattr(self,key).setChecked(bool(opts[key]))
            for key in ("port","baud"):
                if opts.get(key):
                    getattr(self,key).setCurrentText(str(opts[key]))
            for group, params in opts.get("drafts",{}).items():
                if group in LOOPS:
                    self.drafts[group].update({k:float(v) for k,v in params.items() if k in KEYS and abs(float(v)) <= LIMIT})
            old_names = opts.get("names",[])
            if old_names == ["目标值","实际值","输出","误差"]:
                old_names = []
            for w,v in zip(self.name_edits,opts.get("aliases",old_names)):
                w.setText(str(v))
            for i,ref in enumerate(opts.get("references",[])[:4]):
                if ref is not None:
                    name,value = ref
                    self.ref_rows[i][1].setText(str(name))
                    self.ref_rows[i][2].setValue(float(value))
                    self.ref_active[i] = True
            legacy = [opts.get("target_ch"),opts.get("actual_ch")]
            preferences = opts.get("channel_preferences",[None,None] if legacy==[0,1] else legacy)
            if isinstance(preferences,list) and len(preferences) == 2:
                self.channel_preferences = preferences
            for widgets,key in [(self.enabled_ch,"channels"),(self.cards,"cards")]:
                for w,v in zip(widgets,opts.get(key,[])):
                    w.setChecked(bool(v))
            if isinstance(opts.get("autotune"),dict):
                for k,v in opts["autotune"].items():
                    if k in self.autotuner.cfg and isinstance(v,(int,float,bool)):
                        self.autotuner.cfg[k] = v
            for key,fn in (("geometry",self.restoreGeometry),("horizontal",self.horizontal.restoreState),("vertical",self.vertical.restoreState)):
                if self.settings.contains(key):
                    fn(self.settings.value(key))
        except (ValueError,TypeError,AttributeError):
            self.log("系统","设置中存在无效项目，已采用可用设置")

    def save_draft(self):
        self.drafts[self.last_group] = {k:w.value() for k,w in self.fields.items()}

    def load_fields(self):
        for k, w in self.fields.items():
            with C.QSignalBlocker(w):
                w.setValue(self.drafts[self.selected()][k])
        if hasattr(self,"workbench"):
            self.workbench.fields()

    def change_group(self):
        self.save_draft()
        self.last_group = self.selected()
        self.load_fields()
        if self.sim:
            self.clear_plot()
            self.log("系统","仿真波形已切换到 " + self.selected())

    def address_ui(self):
        single = self.address.currentIndex() == 1
        if single:
            self.group.setCurrentIndex(0)
        self.group.setEnabled(not single and not (self.sim and self.writer) and self.generator is None and not self.autotuner.active)

    def edited(self,key,value):
        self.drafts[self.selected()][key] = value
        self.field_errors.discard(self.selected()+"."+key)
        self.workbench.fields()
        if self.auto.isChecked() and self.connected:
            self.send_field(key)

    def enter_field(self,key):
        if not self.auto.isChecked():
            self.send_field(key)

    def refresh_ports(self):
        old = self.port.currentText()
        self.port.clear()
        try:
            for p in sorted(list_ports.comports(),key=lambda p:p.device):
                self.port.addItem(p.device)
                self.port.setItemData(self.port.count()-1,p.description,C.Qt.ItemDataRole.ToolTipRole)
        except OSError as exc:
            self.log("系统",str(exc))
        if old:
            self.port.setCurrentText(old)

    def source_ui(self):
        sim = self.source.currentIndex() == 1
        waiting = self.reconnect_at is not None
        busy = self.connected or self.link is not None or waiting
        self.source.setEnabled(not busy)
        self.model.setEnabled(not busy)
        self.serial_form.setRowVisible(self.model,sim)
        self.serial_form.setRowVisible(self.baud,not sim)
        self.serial_form.setRowVisible(1,not sim)
        self.serial_form.setRowVisible(self.auto_reconnect,not sim)
        for w in (self.port,self.refresh_btn,self.baud,self.bits,self.parity,self.stops,self.flow,self.time_mode,self.rate):
            w.setEnabled(not busy and not sim)
        self.connect_btn.setEnabled(not self.closing)
        self.connect_btn.setText("关闭仿真" if self.sim else "取消重连" if waiting or (self.reconnecting and not self.connected)
                                 else "断开" if busy else "打开仿真" if sim else "连接串口")
        self.stop_btn.setEnabled(self.connected)
        self.gen_btn.setEnabled((self.connected or self.generator is not None) and not self.autotuner.active)
        self.autotune_btn.setEnabled(self.connected or self.autotuner.active)
        self.badge.setText("仿真练习" if self.sim else "串口已连接" if self.connected else "正在关闭…" if self.closing
                           else "等待重连…" if waiting or self.reconnecting else "正在连接…" if busy else "未连接")
        if hasattr(self,"workbench"):
            self.workbench.connection()
            self.workbench.fields()

    def fresh_session(self):
        self.undo_history.clear()
        self.rollback = None
        self.field_errors.clear()
        self.retry_command = None
        self.connection_error = False
        self.stop_banner.hide()
        self.notice.hide()
        self.autotune_status.clear()
        self.autotune_status.hide()
        self.send_heartbeat = self.ping_capable = False
        self.end_record()
        self.rx_log_tail = None
        self.clear_plot()
        self.rx = self.tx = self.bad = self.frames = self.dropped = self.rate_frames = self.fps = 0
        self.count = 0
        self.set_auto_names()
        self.refresh_channels()
        self.epoch = self.rate_start = monotonic()
        self.sim_steps = 0
        self.latest = 0.
        self.parser = Lines()
        self.commands.clear()
        self.confirmed = {}
        self.refresh_channels()
        self.param_tuple = ("",)*16
        self.ack.setText("等待操作")

    def toggle_connection(self):
        if self.reconnect_at is not None or (self.reconnecting and not self.connected):
            self.cancel_reconnect()
            if self.link:
                self.disconnect()
            return
        if self.connected or self.link:
            self.disconnect()
            return
        if self.source.currentIndex() == 1:
            self.fresh_session()
            self.sim = Simulator(self.model.currentIndex())
            self.sim.params = {g:p.copy() for g,p in self.drafts.items()}
            self.connected = True
            self.log("系统","仿真已就绪。读取参数或发送本组，再点击启动。")
            self.read_params()
        else:
            try:
                name = self.port.currentText().strip()
                if not name:
                    raise ValueError("请选择串口；无设备时可选择仿真练习")
                baud = int(self.baud.currentText())
                if not 1 <= baud <= 12_000_000:
                    raise ValueError("波特率应在 1 至 12000000 之间")
                opts = dict(url=name, baudrate=baud, bytesize=int(self.bits.currentText()),
                            parity="NEO"[self.parity.currentIndex()],stopbits=float(self.stops.currentText()),
                            rtscts=self.flow.currentIndex()==1,xonxoff=self.flow.currentIndex()==2)
            except ValueError as exc:
                self.feedback(str(exc),True)
                return
            self.fresh_session()
            self.reconnect_opts = opts
            self.link = SerialLink(opts)
            self.link.start()
        self.source_ui()

    def cancel_reconnect(self):
        self.reconnect_at = None
        self.reconnecting = False
        self.reconnect_attempts = 0
        self.connection_error = False
        self.source_ui()

    def reconnect(self):
        """Reopen the same port after an unexpected loss. The plot and parameters stay;
        the device is only read back (@GET), never started automatically."""
        self.reconnect_at = None
        self.reconnecting = True
        self.reconnect_attempts += 1
        self.parser = Lines()
        self.parser.skipping = True
        self.rx_log_tail = None
        self.link = SerialLink(self.reconnect_opts)
        self.link.start()
        self.source_ui()

    def disconnect(self):
        self.autotuner.stop("连接已关闭",apply_best=False)
        self.stop_generator("连接已关闭",restore=False)
        self.reconnect_at = None
        self.reconnecting = False
        self.send_heartbeat = False
        self.capture.cancel_capture("连接已关闭")
        self.end_record()
        self.commands.clear()
        if self.link:
            self.link.closed.set()
            self.closing = True
        self.sim = None
        self.connected = False
        self.source_ui()
        self.address_ui()
        self.log("系统","已结束采集")

    def feedback(self,msg,error=False):
        self.ack.setText(msg)
        self.ack.setStyleSheet("color:#a22d38" if error else "color:#176a57")
        self.log("系统",msg)
        if error and hasattr(self,"workbench"):
            self.workbench.notice(msg,self.retry_command is not None)

    def log(self,kind,data):
        if isinstance(data,bytes):
            data = data[:4096]
        self.logs.append((datetime.now().strftime("%H:%M:%S.%f")[:-3],kind,data))
        self.log_dirty = True

    def log_received(self, data):
        # Keep one editable entry until the device terminates its line. A slow
        # or unterminated sender is still visible on the next terminal refresh.
        for part in data.splitlines(keepends=True):
            if self.rx_log_tail is not None and self.rx_log_tail.endswith(b"\r") and part != b"\n":
                self.rx_log_tail = None
            while part:
                if self.rx_log_tail is None:
                    self.rx_log_tail = bytearray()
                    self.log("接收", self.rx_log_tail)
                room = 4096 - len(self.rx_log_tail)
                self.rx_log_tail.extend(part[:room])
                part = part[room:]
                if self.rx_log_tail.endswith(b"\n") or len(self.rx_log_tail) == 4096:
                    self.rx_log_tail = None
        self.log_dirty = True

    def render_log(self,force=False,preview_only=False):
        if not self.log_dirty and not force:
            return
        bar = self.terminal.verticalScrollBar()
        prev = bar.value()
        lines = []
        chars = 0
        for stamp,kind,data in reversed(self.logs):
            if isinstance(data,(bytes,bytearray)):
                text = data.hex(" ").upper() if self.hex_mode.isChecked() else data.decode("utf-8","replace").rstrip("\r\n")
                if kind == "接收" and not text:
                    continue
            else:
                text = data
            lines.append(f"{stamp}  {kind}  {text}")
            if preview_only:
                break
            chars += len(lines[-1])
            if chars >= 48000:
                break
        self.terminal_latest.setText(lines[0].splitlines()[-1][:250] if lines else "")
        if preview_only:
            self.log_dirty = False
            return
        self.terminal.setPlainText("\n".join(reversed(lines)))
        bar.setValue(bar.maximum() if self.autoscroll.isChecked() else prev)
        self.log_dirty = False

    def clear_log(self):
        self.rx_log_tail = None
        self.logs.clear()
        self.terminal.clear()
        self.terminal_latest.clear()

    def key(self,field):
        return ("@" if self.address.currentIndex() else "@" + self.selected() + ".") + field

    def send_field(self,key):
        with C.QSignalBlocker(self.fields[key]):
            self.fields[key].interpretText()
        self.drafts[self.selected()][key] = self.fields[key].value()
        self.enqueue(self.key(key) + "=" + format(self.fields[key].value(),".4f"))

    def send_group(self):
        if not self.connected:
            self.feedback("请先连接串口或打开仿真",True)
            return
        self.commands.submit([self.key(k)+"="+format(self.fields[k].value(),".4f") for k in KEYS])

    def read_params(self):
        self.enqueue(self.key("GET"))

    def reset_integral(self):
        self.enqueue(self.key("RST"))

    def run_device(self):
        self.enqueue("@RUN")

    def open_autotune(self):
        self.autotuner.open_dialog()

    def stop_device(self):
        if not self.connected:
            return
        self.autotuner.on_device_stopped("急停")
        self.stop_generator("急停",restore=False)
        self.workbench.stop_feedback()
        self.send_heartbeat = False
        self.commands.clear()
        self.enqueue("@STOP",urgent=True)

    def toggle_generator(self):
        if self.generator:
            self.stop_generator("发生器已停止，TGT 已恢复")
            return
        if not self.connected:
            self.feedback("请先连接串口或打开仿真",True)
            return
        group = self.selected()
        with C.QSignalBlocker(self.fields["TGT"]):
            self.fields["TGT"].interpretText()
        try:
            gen = TargetGenerator(self.gen_shape.currentIndex(),self.fields["TGT"].value(),
                                  self.gen_amp.value(),self.gen_period.value(),self.gen_rate.value())
        except ValueError as exc:
            self.feedback(str(exc),True)
            return
        if gen.amplitude == 0:
            self.feedback("发生器幅值不能为零",True)
            return
        low,high,_ = self.rules[group]["TGT"]
        lo,hi = gen.bounds()
        if lo < low or hi > high:
            self.feedback(f"发生器范围 {lo:g} 至 {hi:g} 超出 {group}.TGT 允许范围 {low:g} 至 {high:g}",True)
            return
        self.generator = gen
        self.generator_group = group
        self.generator_ui()
        self.log("系统",f"目标值发生器启动  {SHAPES[gen.shape]}  基准 {gen.base:g}  幅值 {gen.amplitude:g}  周期 {gen.period:g} s")
        self.generator_tick(monotonic())

    def generator_ui(self):
        running = self.generator is not None
        self.gen_btn.setText("停止发生器" if running else "启动发生器")
        self.gen_btn.setObjectName("danger" if running else "")
        self.gen_btn.style().unpolish(self.gen_btn)
        self.gen_btn.style().polish(self.gen_btn)
        for widget in (self.gen_shape,self.gen_amp,self.gen_period,self.gen_rate,self.fields["TGT"]):
            widget.setEnabled(not running)
        self.group.setEnabled(not running and self.address.currentIndex()!=1 and not (self.sim and self.writer))
        if running:
            self.gen_status.setText(f"运行中 · {SHAPES[self.generator.shape]} · {self.generator_group}.TGT")
        self.gen_btn.setEnabled(self.connected or running)

    def stop_generator(self,message="",restore=True):
        gen = self.generator
        if gen is None:
            return
        self.generator = None
        self.generator_ui()
        self.gen_status.setText(message or "已停止")
        if restore and self.connected and gen.last_sent is not None and gen.last_sent != gen.base:
            self.commands.submit([self.key("TGT")+"="+format(gen.base,".4f")],quiet=True)
        if message:
            self.log("系统","目标值发生器  "+message)

    def generator_tick(self,now):
        gen = self.generator
        if gen is None or not self.connected or self.commands.busy():
            return
        value = gen.due(now)
        if value is StopIteration:
            self.stop_generator("单次阶跃完成",restore=False)
        elif value is not None:
            self.commands.submit([self.key("TGT")+"="+format(value,".4f")],quiet=True)

    def enqueue(self,command,urgent=False):
        self.commands.submit([command],urgent)

    def send_raw(self):
        if not self.connected:
            self.feedback("请先连接数据源",True)
            return
        text = self.raw.text()
        if not text.strip():
            return
        try:
            data = bytes.fromhex(text) if self.raw_hex.isChecked() else (text+"\n").encode()
        except ValueError:
            self.feedback("HEX 发送需要完整的十六进制字节",True)
            return
        if not self.raw_hex.isChecked() and text.startswith("@"):
            if text == "@STOP":
                self.stop_device()
            else:
                self.enqueue(text)
        elif self.sim:
            self.feedback("仿真终端使用 @ 开头的文本命令",True)
        elif not self.link.send(data):
            self.feedback("发送队列已满",True)
        else:
            self.raw.clear()

    def receive(self,data,stamp):
        self.rx += len(data)
        self.log_received(data)
        for kind,payload in self.parser.feed(data):
            if kind == "data":
                t = self.frames/self.rate.value() if self.time_mode.currentIndex() else max(0.,stamp-self.epoch)
                self.sample(t,payload)
            elif kind == "bad":
                self.bad += 1
                if self.bad <= 3 or self.bad % 100 == 0:
                    self.log("解析",f"错误帧 {self.bad}：{payload}")
            elif kind == "log" and payload == "#CAP:PING":
                self.ping_capable = True
            elif kind == "log" and payload.startswith("#ERROR:float printf"):
                self.feedback("设备浮点格式化自检失败，请启用 -u _printf_float 后重新链接",True)
            elif kind == "log" and payload.startswith("#STOP:"):
                reason = payload[6:].strip()
                text = {"heartbeat":("设备因心跳超时已停机","设备已报告心跳超时停机"),
                        "fall":("检测到倒地，设备已自动停机","检测到倒地 · 输出已清零，扶正后再点启动")}.get(
                        reason,("设备已停机  "+reason,"设备已停机 · "+reason))
                self.send_heartbeat = False
                self.autotuner.on_device_stopped(text[0])
                self.stop_generator(text[0],restore=False)
                self.commands.failed(text[0])
                self.stop_banner.setText(text[1])
                self.stop_banner.show()
            elif kind == "names":
                if not self.writer:
                    for w,n in zip(self.name_edits,payload):
                        w.setPlaceholderText(n[:32])
                    self.channel_names()
            elif kind in ("ACK","PARAM","ERR"):
                self.commands.reply(kind,payload)

    def update_params(self):
        self.param_tuple = tuple(self.confirmed.get(k,"") for k in PARAM_COLS)
        if any(self.follow):
            self.refresh_channels()

    def mark(self,text):
        event = (self.latest,text)
        if self.scope.frozen:
            self.deferred_marks.append(event)
        else:
            self.scope.mark(*event)

    def sample(self,t,values):
        values = [v*self.scales[i] for i,v in enumerate(values)]
        if self.count != len(values):
            self.count = len(values)
            if not self.scope.frozen:
                self.refresh_channels()
        self.frames += 1
        self.rate_frames += 1
        self.latest = t
        refs = [self.reference_value(i) if active else np.nan for i,active in enumerate(self.ref_active)]
        self.buffer.add(t,[*values,*([np.nan]*(4-self.count)),*refs],self.param_tuple)
        if self.capture.capture_channel is not None:
            self.capture.check_capture(t,values)
        if self.writer:
            try:
                self.writer.writerow(csv_row(t,values,self.param_tuple,[refs[i] for i in self.active_refs()]))
            except (OSError,ValueError) as exc:
                self.end_record()
                self.feedback("录制失败  " + str(exc),True)

    def tick(self):
        now = monotonic()
        if self.link:
            batch,lost = self.link.take()
            self.dropped += lost
            for kind,stamp,data in batch:
                if kind == "open" and not self.closing:
                    self.connected = True
                    self.connection_error = False
                    if self.reconnecting:
                        self.reconnecting = False
                        self.reconnect_attempts = 0
                        self.notice.hide()
                        self.log("系统","已重新连接 " + str(data) + "，正在读回参数（不会自动启动）")
                        self.source_ui()
                        self.read_params()
                    else:
                        self.source_ui()
                        self.log("系统","已连接 " + str(data))
                elif kind == "rx":
                    self.receive(data,stamp)
                elif kind == "gap":
                    self.capture.cancel_capture("数据流中断，已取消")
                    self.rx_log_tail = None
                    self.parser = Lines()
                    self.parser.skipping = True
                elif kind == "tx":
                    self.tx += len(data)
                    self.log("TX",data)
                    self.commands.written(data,stamp)
                elif kind == "error":
                    self.connection_error = True
                    self.badge.setToolTip(data)
                    if self.reconnecting:
                        if self.reconnect_attempts <= 1 or self.reconnect_attempts % 10 == 0:
                            self.log("系统",f"重连第 {self.reconnect_attempts} 次失败  {data}")
                    else:
                        self.stop_generator("串口异常",restore=False)
                        self.commands.failed("串口异常  " + data)
                elif kind == "closed":
                    unexpected = not self.closing and self.connection_error
                    self.send_heartbeat = False
                    self.capture.cancel_capture("连接已关闭")
                    self.rx_log_tail = None
                    self.link = None
                    self.closing = self.connected = False
                    self.commands.clear()
                    self.end_record()
                    if unexpected and self.auto_reconnect.isChecked() and self.reconnect_opts and not self.sim:
                        self.reconnect_at = now + min(3., 1. + .2*self.reconnect_attempts)
                        if not self.reconnecting:
                            self.feedback("串口断开，将自动重连（设备会因心跳超时自行停机）",True)
                    else:
                        self.reconnecting = False
                    self.source_ui()
                    break
        if self.link:
            self.link.keepalive(self.connected and self.send_heartbeat)
        elif self.reconnect_at is not None and now >= self.reconnect_at:
            self.reconnect()
        if self.sim:
            if self.send_heartbeat:
                self.sim.ping()
            due = int((now-self.epoch)/self.sim.dt)
            lag = due-self.sim_steps
            if lag > 30:
                # The UI stalled (window drag, modal dialog). Pause the simulated world
                # instead of fast-forwarding it unseen.
                self.epoch += (lag-30)*self.sim.dt
                lag = 30
            for _ in range(max(0,lag)):
                self.sim_steps += 1
                v = self.sim.step(self.selected())
                self.sample(self.sim_steps*self.sim.dt,v)
                wire = (",".join(f"{x:.3f}" for x in v)+"\n").encode()
                self.rx += len(wire)
                if self.sim_steps % 5 == 0:
                    self.log("仿真 RX",wire)
                for line in self.sim.drain():
                    self.receive((line+"\n").encode(),now)
                if not self.sim:
                    break
        if self.commands.timed_out(now):
            self.commands.failed("应答超时，已取消后续命令  " + self.commands.pending[0])
        self.autotuner.tick(now)
        self.generator_tick(now)
        self.commands.dispatch()
        if self.buffer.size and (self.scope.drawn_view != self.scope.viewport_key() or
                                 (not self.scope.frozen and self.last_paint != self.frames)):
            self.paint_scope()
            self.last_paint = self.frames
        if now-self.last_metrics >= .25:
            self.last_metrics = now
            self.analyze()
            self.render_log(preview_only=not self.terminal_toggle.isChecked())
            self.update_status()
        if now-self.rate_start >= 1:
            self.fps = round(self.rate_frames/(now-self.rate_start))
            self.rate_start,self.rate_frames = now,0
            if self.record_file:
                try:
                    self.record_file.flush()
                except OSError as exc:
                    self.end_record()
                    self.feedback("写入失败  "+str(exc),True)

    def update_status(self):
        self.status.setText(f"{self.badge.text()}     RX  {self.rx:,} B     TX  {self.tx:,} B     {self.fps} 帧/s     通道  {self.count}/4     错误帧  {self.bad}     溢出  {self.dropped} B")

    def channel_names(self):
        self.refresh_channels()
        self.paint_scope()

    def add_reference(self):
        if self.writer or all(self.ref_active):
            return
        slot = self.ref_active.index(False)
        dialog = W.QDialog(self)
        dialog.setWindowTitle("添加固定参考线")
        form = W.QFormLayout(dialog)
        name = W.QLineEdit("目标值" if slot == 0 else f"参考线 {slot+1}")
        name.setMaxLength(32)
        value = number(0,-LIMIT,LIMIT,1,3)
        form.addRow("名称",name)
        form.addRow("固定值",value)
        from layout_ui import combo
        bind = combo(["手动输入",*LOOPS])
        form.addRow("跟随已确认 TGT",bind)
        buttons = W.QDialogButtonBox(W.QDialogButtonBox.StandardButton.Ok | W.QDialogButtonBox.StandardButton.Cancel)
        buttons.button(W.QDialogButtonBox.StandardButton.Ok).setText("添加")
        buttons.button(W.QDialogButtonBox.StandardButton.Cancel).setText("取消")
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        form.addRow(buttons)
        if dialog.exec():
            self.follow[slot] = bind.currentText() if bind.currentIndex() else ""
            self.set_reference(slot,name.text(),value.value())

    def set_reference(self,slot,name,value):
        self.ref_rows[slot][1].setText(name)
        self.ref_rows[slot][2].setValue(value)
        self.ref_active[slot] = True
        self.channel_names()

    def remove_reference(self,slot):
        if self.writer:
            return
        self.ref_active[slot] = False
        self.follow[slot] = ""
        self.buffer.data[:,slot+5] = np.nan
        if self.scope.frozen:
            self.scope.data[:,slot+5] = np.nan
        self.channel_names()

    def reference_changed(self,slot):
        if self.ref_active[slot]:
            self.scope.set_reference(slot,self.ref_name(slot),self.ref_rows[slot][2].value(),True)
            self.paint_scope()

    def reference_value(self,slot):
        group = self.follow[slot]
        return self.confirmed.get(group+".TGT",np.nan) if group else self.ref_rows[slot][2].value()

    def select_channel(self,role):
        widget = (self.target_ch,self.actual_ch)[role]
        self.channel_preferences[role] = widget.currentData()
        self.analyze()

    def refresh_channels(self):
        count = self.frozen_count if self.scope.frozen else self.count
        names = self.names() + [self.ref_name(i) for i in range(4)]
        self.scope.set_names(names)
        self.scope.set_right_channel(self.right_channel if self.right_channel<count else -1)
        trigger_index = self.trigger_ch.currentData()
        with C.QSignalBlocker(self.trigger_ch):
            self.trigger_ch.clear()
            for i in range(count):
                self.trigger_ch.addItem(names[i],i)
            if trigger_index is not None and trigger_index<count:
                self.trigger_ch.setCurrentIndex(trigger_index)
        for i,w in enumerate(self.name_edits):
            self.channel_form.setRowVisible(w,i<count)
        self.channel_form.setRowVisible(self.no_channels,count == 0)
        refs = self.active_refs()
        for i,(tag,name,value,remove) in enumerate(self.ref_rows):
            self.channel_form.setRowVisible(tag,self.ref_active[i])
            tag.setText("跟随" if self.follow[i] else "固定")
            level = self.reference_value(i)
            if self.scope.frozen and self.follow[i] and len(self.scope.data):
                level = self.scope.data[-1,5+i]
            if self.follow[i] and np.isfinite(level):
                with C.QSignalBlocker(value):
                    value.setValue(level)
            value.setEnabled(not self.follow[i])
            value.setToolTip(f"跟随 {self.follow[i]}.TGT；等待读取或下发确认" if self.follow[i] else "")
            self.scope.set_reference(i,names[4+i],level,self.ref_active[i] and np.isfinite(level))
            name.setEnabled(not self.writer)
            remove.setEnabled(not self.writer)
        self.add_ref_btn.setEnabled(len(refs)<4 and not self.writer)
        self.calibrate_btn.setEnabled(not self.writer)
        ids = [*range(count),*(4+i for i in refs)]
        speed_pwm = count==2 and not self.sim and any(tag in names[1].lower() for tag in ("pwm","占空","duty"))
        defaults = [4+refs[0] if refs else None if speed_pwm else 0,
                    1 if count>=2 and (self.sim or not (refs or speed_pwm)) else 0]
        for role,widget in enumerate((self.target_ch,self.actual_ch)):
            with C.QSignalBlocker(widget):
                widget.clear()
                if role==0 and speed_pwm and not refs:
                    widget.addItem("选择目标通道",None)
                for i in ids:
                    widget.addItem(names[i] if i<4 else "固定 · "+names[i],i)
                preferred = self.channel_preferences[role]
                if preferred not in ids:
                    preferred = defaults[role]
                if preferred in ids:
                    widget.setCurrentIndex(widget.findData(preferred))

    def choose_color(self,slot):
        original = self.colors[slot]
        dialog = W.QColorDialog(QtGui.QColor(original),self)
        dialog.setWindowTitle("选择曲线颜色")
        dialog.setOption(W.QColorDialog.ColorDialogOption.DontUseNativeDialog,True)
        dialog.currentColorChanged.connect(lambda color: self.apply_color(slot,color.name()))
        if dialog.exec():
            self.apply_color(slot,dialog.selectedColor().name())
        else:
            self.apply_color(slot,original)

    def apply_color(self,slot,color):
        self.colors[slot] = color
        chip = self.color_buttons[slot]
        tile = QtGui.QPixmap(17,13)
        tile.fill(QtGui.QColor(color))
        chip.setIcon(QtGui.QIcon(tile))
        chip.setToolTip(f"选择颜色  {color.upper()}")
        if slot<4:
            self.scope.curves[slot].setPen(pg.mkPen(color,width=self.line.value()))
            self.enabled_ch[slot].setIcon(QtGui.QIcon(tile))
            for sample,_ in self.scope.legend.items:
                sample.update()
        else:
            guide = self.scope.references[slot-4]
            guide.setPen(pg.mkPen(color,width=self.line.value(),style=C.Qt.PenStyle.DashLine))
            guide.label.setColor(color)

    def plot_options(self):
        s = self.scope
        s.visible = [x.isChecked() for x in self.enabled_ch]
        s.set_window(self.window.value())
        s.average_pixels = self.render_mode.currentIndex()==0
        s.include_zero = self.yscale.currentIndex()==0
        for side,field in (("bottom",self.x_div),("left",self.y_div)):
            axis = s.getAxis(side)
            if axis.division != field.value():
                axis.division = field.value()
                axis.setTickSpacing()
        s.showGrid(x=self.grid.isChecked(),y=self.grid.isChecked(),alpha=.16)
        for slot,color in enumerate(self.colors):
            self.apply_color(slot,color)
        s.fixed_y = None
        self.ymin.setEnabled(self.yscale.currentIndex()==1)
        self.ymax.setEnabled(self.yscale.currentIndex()==1)
        if self.yscale.currentIndex()==1:
            if self.ymax.value() > self.ymin.value():
                s.fixed_y = (self.ymin.value(),self.ymax.value())
            else:
                self.feedback("固定纵轴要求 Y 最大值大于最小值",True)
        self.paint_scope()
        self.measure()

    def paint_scope(self):
        a = self.scope.data if self.scope.frozen else self.buffer.view()
        self.scope.draw(a,self.frozen_count if self.scope.frozen else self.count,self.legend.isChecked())
        self.update_chips()

    def update_chips(self):
        count = self.frozen_count if self.scope.frozen else self.count
        self.live_btn.setVisible(self.scope.frozen)
        self.chip_bar.setVisible((self.legend.isChecked() and count>0) or self.scope.frozen)
        self.guide.setVisible(not self.scope.data.size and not self.connected and not self.link)
        a = self.scope.data
        index = -1
        if len(a):
            left,right = self.scope.viewRange()[0]
            index = np.searchsorted(a[:,0],right,side="right")-1
            if index>=0 and a[index,0]<left:
                index = -1
        for i,chip in enumerate(self.enabled_ch):
            chip.setVisible(i<count and self.legend.isChecked())
            if i<count:
                if chip.font().strikeOut()==chip.isChecked():
                    font = chip.font()
                    font.setStrikeOut(not chip.isChecked())
                    chip.setFont(font)
                val = a[index,i+1] if index>=0 else np.nan
                name = self.scope.names[i]
                label = name+"  "+(f"{val:.5g}" if np.isfinite(val) else "—")
                if chip.text()!=label:
                    chip.setText(label)

    def toggle_freeze(self):
        self.capture.cancel_capture("已取消")
        new = not self.scope.frozen
        self.scope.set_frozen(new)
        if new:
            self.frozen_count = self.count
            self.scope.draw(self.buffer.view(copy=True),self.count,self.legend.isChecked())
            self.frozen_params = self.buffer.snapshots()
        else:
            self.frozen_params = None
            for ev in self.deferred_marks:
                self.scope.mark(*ev)
            self.deferred_marks.clear()
            self.refresh_channels()
            self.paint_scope()
        self.pause_btn.setText("继续" if new else "暂停")
        self.update_chips()
        if not new:
            self.trigger_status.setText("等待设置")
        self.measure()

    def clear_plot(self):
        self.capture.cancel_capture("等待设置")
        self.trigger_status.setText("等待设置")
        self.count = 0
        self.buffer.reset()
        self.scope.reset()
        self.scope.set_frozen(False)
        self.pause_btn.setText("暂停")
        self.deferred_marks.clear()
        self.frozen_params = None
        self.refresh_channels()
        self.paint_scope()
        self.measure()

    def measure(self):
        msg = self.scope.measurement(self.actual_ch.currentData())
        rows = self.scope.readouts(self.show_stats.isChecked())
        if msg:
            rows.insert(0,msg)
        hint = "单击曲线放置游标 A、B，滚轮缩放或拖动平移" if self.scope.frozen else "暂停后可缩放、拖动，单击波形放置 A、B 游标"
        self.measure_text.setText("\n".join(rows) or hint)

    def analyze(self):
        a = self.scope.data
        target,actual = self.target_ch.currentData(),self.actual_ch.currentData()
        metrics = None
        if target is not None and actual is not None and target != actual and len(a):
            # A new or temporarily missing channel begins a new measurement interval.
            missing = np.flatnonzero(~np.isfinite(a[:,target+1]) | ~np.isfinite(a[:,actual+1]))
            segment = a[missing[-1]+1:] if len(missing) else a
            metrics = step_metrics(segment[:,0],segment[:,target+1],segment[:,actual+1],.05 if self.band.currentIndex() else .02)
        if metrics is None:
            self.scope.clear_step()
            self.metrics.setText("等待所选目标通道发生阶跃")
        else:
            if self.overlays.isChecked():
                self.scope.step_overlay(segment,target+1,actual+1,.05 if self.band.currentIndex() else .02,metrics)
            else:
                self.scope.clear_step()
            vals = ["—" if x is None else f"{x:.3g}" for x in metrics]
            self.metrics.setText(f"超调  {vals[0]} %　 上升  {vals[1]} s\n调节  {vals[2]} s　 尾段误差  {vals[3]}\nIAE  {vals[4]}")
        self.measure()

    def choose_file(self,title,suffix):
        filename = "PID_"+datetime.now().strftime("%Y%m%d_%H%M%S")+"."+suffix
        path,_ = W.QFileDialog.getSaveFileName(self,title,filename,f"{suffix.upper()} (*.{suffix})",
                                               options=W.QFileDialog.Option.DontUseNativeDialog)
        return path

    def export_view(self):
        a = self.scope.data if self.scope.frozen else self.buffer.view(copy=True)
        if not len(a):
            self.feedback("当前视图没有数据",True)
            return
        left,right = self.scope.viewRange()[0]
        params = self.frozen_params if self.scope.frozen else self.buffer.snapshots()
        names = self.names()
        refs = self.active_refs()
        ref_names = [self.ref_name(i) for i in refs]
        path = self.choose_file("导出当前可见时间段","csv")
        if not path:
            return
        try:
            with open(path,"w",newline="",encoding="utf-8-sig") as f:
                out = csv.writer(f)
                out.writerow(csv_header(names,ref_names))
                for data,p in zip(a,params):
                    if left <= data[0] <= right:
                        out.writerow(csv_row(data[0], data[1:5], p,[data[5+i] for i in refs]))
            self.feedback("已导出视图 CSV")
        except OSError as exc:
            self.feedback(str(exc),True)

    def png(self):
        path = self.choose_file("保存波形快照","png")
        if path:
            bounds = C.QRect(C.QPoint(0,0),self.scope.geometry().bottomRight())
            ok = self.plot_container.grab(bounds).save(path,"PNG")
            self.feedback("已保存 PNG" if ok else "PNG 保存失败",not ok)

    def record(self):
        if self.writer:
            self.end_record()
            return
        if not self.connected:
            self.feedback("请先打开数据源",True)
            return
        path = self.choose_file("录制完整采集数据","csv")
        if not path:
            return
        try:
            self.record_file = open(path,"w",newline="",encoding="utf-8-sig")
            self.writer = csv.writer(self.record_file)
            self.writer.writerow(csv_header(self.names(),[self.ref_name(i) for i in self.active_refs()]))
        except OSError as exc:
            self.end_record()
            self.feedback(str(exc),True)
            return
        self.record_btn.setText("停止录制")
        self.refresh_channels()
        for field in self.name_edits:
            field.setEnabled(False)
        self.address.setEnabled(not self.sim)
        self.address_ui()
        self.log("系统","录制至 " + path)

    def end_record(self):
        if self.record_file:
            try:
                self.record_file.close()
            except OSError as exc:
                self.feedback("录制文件关闭失败  "+str(exc),True)
            self.record_file = self.writer = None
            self.record_btn.setText("录制 CSV")
            self.refresh_channels()
            for field in self.name_edits:
                field.setEnabled(True)
            self.address.setEnabled(True)
            self.address_ui()
            self.log("系统","录制结束")

    def presets(self):
        try:
            p = json.loads(self.settings.value("presets/"+self.selected(),"{}"))
            return p if isinstance(p,dict) else {}
        except ValueError:
            return {}

    def save_preset(self):
        name,ok = W.QInputDialog.getText(self,"保存本组预设","预设名称")
        if ok and name.strip():
            p = self.presets()
            self.save_draft()
            p[name.strip()] = self.drafts[self.selected()].copy()
            self.settings.setValue("presets/"+self.selected(),json.dumps(p,ensure_ascii=False))
            self.feedback("已保存预设  "+name.strip())

    def load_preset(self):
        p = self.presets()
        if not p:
            self.feedback("本组还没有预设")
            return
        name,ok = W.QInputDialog.getItem(self,"载入本组预设","选择预设",list(p),0,False)
        if ok:
            self.drafts[self.selected()] = p[name].copy()
            self.load_fields()
            self.feedback("预设已载入，点击“发送本组”可下发")

    def closeEvent(self,event):
        W.QApplication.instance().removeEventFilter(self.workbench)
        self.timer.stop()
        self.end_record()
        if self.link:
            self.link.closed.set()
            self.link.join(.6)
        self.persist()
        # The legend follows ViewBox geometry; stop their callbacks before Qt destroys the scene.
        self.scope.legend.blockSignals(True)
        self.scope.getViewBox().blockSignals(True)
        self.scope.secondary.blockSignals(True)
        self.scope.shutdown()
        super().closeEvent(event)


def main():
    verify = "--self-test" in sys.argv
    if verify:
        os.environ["QT_QPA_PLATFORM"] = "offscreen"
    if sys.platform == "win32":
        from ctypes import windll
        windll.shell32.SetCurrentProcessExplicitAppUserModelID("PIDLab.PIDTuner")
    app = W.QApplication(sys.argv)
    app.setOrganizationName("PIDLab")
    app.setApplicationName("PIDTuner")
    app.setWindowIcon(QtGui.QIcon(str(Path(__file__).resolve().parent/"assets"/"pidtuner.ico")))
    app.setStyle("Fusion")
    app.setStyleSheet(STYLE)
    if verify:
        from tempfile import TemporaryDirectory
        with TemporaryDirectory() as tmp:
            win = Window(Path(tmp)/"settings.ini")
            win.show()
            app.processEvents()
            for chunk in (b"1", b"5", b"6\r", b"\n"):
                win.receive(chunk, monotonic())
            win.render_log()
            received = win.terminal.toPlainText().splitlines()
            terminal_ok = len(received) == 1 and received[0].endswith("接收  156")
            win.set_reference(0,"目标转速",150)
            win.configure_channels([.01,1,1,1],["RPM","","",""],1,["","","",""])
            win.receive(b"15600,600\n",monotonic())
            win.paint_scope()
            calibrated_ok = win.actual_ch.currentData()==0 and win.scope.secondary is win.scope.curves[1].getViewBox()
            calibrated_ok = calibrated_ok and win.buffer.view()[-1,1]==156
            win.window.setValue(7.5)
            win.apply_color(0,"#e6194b")
            viewport_ok = win.scope.width_s == 7.5 and win.scope.curves[0].opts["pen"].color().name() == "#e6194b"
            channels_ok = win.count == 2 and win.actual_ch.currentData() == 0 and win.target_ch.currentData() == 4
            channels_ok = channels_ok and win.scope.references[0].value() == 150 and win.tx == 0
            win.source.setCurrentIndex(1)
            win.toggle_connection()
            win.run_device()
            win.epoch = monotonic() - .2
            win.tick()
            win.toggle_freeze()
            ok = terminal_ok and calibrated_ok and channels_ok and viewport_ok and win.count == 4 and win.buffer.size >= 19 and win.commands.pending is None
            win.stop_device()
            ok = ok and not win.sim.running and not win.scope.grab().isNull()
            ok = ok and not win.windowIcon().pixmap(32,32).isNull()
            win.close()
            win.deleteLater()
            C.QCoreApplication.sendPostedEvents(None,C.QEvent.Type.DeferredDelete)
            return 0 if ok else 1
    win = Window()
    win.setAttribute(C.Qt.WidgetAttribute.WA_DeleteOnClose)
    win.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())

