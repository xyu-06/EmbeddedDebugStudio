"""Device command validation, acknowledgements and rollback."""
from time import monotonic
from PySide6 import QtCore as C, QtWidgets as W
from command_queue import CommandQueue
from engine import LOOPS, KEYS, LIMIT, match_ack, parameter_pairs


def ask_confirmation(parent, changes):
    """Default confirmation for large parameter changes. Tests and tools may inject another."""
    return W.QMessageBox.question(parent,"确认参数变化","确认下发以下变化？\n"+"\n".join(changes),
            W.QMessageBox.StandardButton.Yes | W.QMessageBox.StandardButton.No,
            W.QMessageBox.StandardButton.No) == W.QMessageBox.StandardButton.Yes


class DeviceSession(CommandQueue):
    def __init__(self, window, confirm=None):
        super().__init__()
        self.ui = window
        self.confirm = confirm or (lambda changes: ask_confirmation(self.ui, changes))

    def retry_last(self):
        cmd = self.ui.retry_command
        if self.ui.connected and cmd:
            if cmd=="@STOP":
                self.ui.stop_device()
            else:
                self.ui.enqueue(cmd)


    def undo_parameter(self):
        if not self.ui.undo_history or self.pending or self.waiting:
            return
        batch,previous = self.ui.undo_history[-1]
        commands = [f"@{addr}={value:.4f}" for addr,value in previous.values()]
        self.submit(commands,undo=(batch,previous))


    def submit(self,commands,urgent=False,undo=None,quiet=False):
        """quiet: generator traffic. Range limits still apply, but no confirmation,
        undo history, change markers or log entries per acknowledgement."""
        if not self.ui.connected:
            self.ui.feedback("请先连接串口或打开仿真",True)
            return
        group = self.ui.selected()
        changes = []
        for command in commands:
            if "=" not in command:
                continue
            try:
                updates = parameter_pairs(command.lstrip("@"), group)
            except ValueError:
                updates = {}
            if not updates:
                continue
            for full_key, value in updates.items():
                g, k = full_key.split(".", 1)
                low, high, threshold = self.ui.rules[g][k]
                if not low <= value <= high:
                    self.ui.feedback(f"{g}.{k} 允许范围为 {low:g} 至 {high:g}", True)
                    return
                previous = self.ui.confirmed.get(full_key)
                if not quiet and abs(value - (previous if previous is not None else 0.)) >= threshold:
                    before = "未读回" if previous is None else f"{previous:g}"
                    changes.append(f"{g}.{k}　{before} → {value:g}")
        if changes and not self.confirm(changes):
            return
        if not self.ui.connected or not self.append(commands,group,urgent,quiet):
            self.ui.feedback("待发送命令已满",True)
            return
        self.ui.rollback = (self.batch,undo) if undo else self.ui.rollback
        self.dispatch()
        return True


    def dispatch(self):
        if not self.ui.connected:
            return
        next_cmd = self.next_command()
        if next_cmd is None:
            return
        cmd,urgent = next_cmd
        wire = (cmd+"\n").encode()
        if not self.active_is_quiet():
            self.ui.ack.setText("等待应答  " + cmd)
        if self.ui.sim:
            self.ui.tx += len(wire)
            self.ui.log("TX",wire)
            reply = self.ui.sim.command(cmd)
            if reply:
                self.ui.receive((reply+"\n").encode(),monotonic())
            else:
                self.pending = None   # e.g. @PING: the device sends no reply
        elif not self.ui.link.send(wire,urgent):
            self.failed("发送队列已满")


    def failed(self,msg):
        if getattr(self.ui,"generator",None):
            self.ui.stop_generator("命令失败，发生器已停止",restore=False)
        was_pending = self.pending
        at = getattr(self.ui,"autotuner",None)
        at_engaged = bool(at and at.state != at.IDLE)
        self.ui.rollback = None
        self.ui.retry_command = was_pending[0] if was_pending else None
        if self.ui.retry_command and "=" in self.ui.retry_command:
            addr = self.ui.retry_command[1:].split("=",1)[0]
            if "." not in addr:
                addr = "ANG."+addr
            if addr.split(".")[-1] in KEYS:
                self.ui.field_errors.add(addr)
                self.ui.workbench.field_event(addr,True)
        if self.ui.retry_command == "@STOP":
            self.ui.stop_banner.setText("急停未获确认 · 请检查设备输出")
        self.clear()
        if at_engaged:
            at.on_command_failed(msg)
        self.ui.feedback(msg,True)


    def reply(self,kind,payload):
        cmd,group,_ = self.pending or ("","ANG",None)
        if cmd and "." not in cmd.partition("=")[0]:
            group = "ANG"
        if kind == "ERR":
            if (".SET=" in cmd or cmd.startswith("@SET=")) and any(tag in payload.lower() for tag in ("unknown", "expected")):
                self.ui.set_capable = False
            if "only spd" in payload.lower() and self.ui.selected() != "SPD":
                was_get = cmd.endswith("GET")
                self.clear()
                self.ui.group.setCurrentIndex(LOOPS.index("SPD"))
                self.ui.feedback("下位机仅启用 SPD 速度环，已自动切换至 SPD")
                if was_get:
                    self.ui.read_params()
                return
            self.failed("设备报错  " + payload)
            return
        if kind == "PARAM":
            try:
                updates = parameter_pairs(payload,group)
            except (ValueError,OverflowError):
                self.ui.bad += 1
                return
            is_get = cmd.endswith("GET")
            expected = cmd[1:].split(".")[0] if "." in cmd else group
            full = all(f"{expected}.{k}" in updates for k in KEYS)
            for key,v in updates.items():
                g,k = key.split(".")
                self.ui.confirmed[key] = v
                if (not self.pending or is_get) and abs(v) <= LIMIT:
                    self.ui.drafts[g][k] = v
                    if g == self.ui.selected():
                        with C.QSignalBlocker(self.ui.fields[k]):
                            self.ui.fields[k].setValue(v)
            self.ui.update_params()
            self.ui.workbench.fields()
            if is_get and full:
                self.pending = None
                self.ui.feedback("已读回  " + expected)
            return
        if not cmd or not match_ack(cmd,payload):
            return
        if cmd in ("@RUN","@STOP"):
            self.ui.send_heartbeat = cmd == "@RUN" and self.ui.ping_capable
            self.ui.next_ping = 0.
            if cmd == "@STOP":
                self.ui.stop_banner.setText("已急停 · 设备已确认 STOP")
                self.ui.stop_banner.show()
            else:
                self.ui.stop_banner.hide()
        self.ui.retry_command = None
        self.ui.notice.hide()
        quiet = self.active_is_quiet()
        if "=" in cmd:
            try:
                updates = parameter_pairs(payload,group)
            except ValueError:
                updates = {}
            for addr,value in updates.items():
                old = self.ui.confirmed.get(addr)
                if self.ui.rollback and self.ui.rollback[0]==self.active_batch:
                    entry = self.ui.rollback[1]
                    entry[1].pop(addr,None)
                    if not entry[1]:
                        self.ui.undo_history.remove(entry)
                        self.ui.rollback = None
                    g,k = addr.split(".")
                    self.ui.drafts[g][k] = value
                    if g==self.ui.selected():
                        with C.QSignalBlocker(self.ui.fields[k]):
                            self.ui.fields[k].setValue(value)
                elif old is not None and old!=value and not quiet:
                    batch = self.active_batch
                    if not self.ui.undo_history or self.ui.undo_history[-1][0]!=batch:
                        self.ui.undo_history.append((batch,{}))
                    wire_key = addr if "." in cmd.partition("=")[0] else addr.split(".", 1)[1]
                    self.ui.undo_history[-1][1].setdefault(addr,(wire_key,old))
                self.ui.field_errors.discard(addr)
            self.ui.confirmed.update(updates)
            self.ui.update_params()
            if quiet:
                self.pending = None
                self.ui.ack.setText("发生器  " + payload)
                self.ui.workbench.fields()
                at = getattr(self.ui, "autotuner", None)
                if at and at.state != at.IDLE:
                    at.on_command_acked(cmd)
                return
            for addr in updates:
                self.ui.workbench.field_event(addr)
            self.ui.mark(payload)
        elif cmd.endswith("GET"):
            return
        elif not quiet:
            self.ui.mark(payload)
        self.pending = None
        at = getattr(self.ui, "autotuner", None)
        if at and at.state != at.IDLE:
            at.on_command_acked(cmd)
        if not quiet:
            self.ui.feedback("已确认  " + payload)

