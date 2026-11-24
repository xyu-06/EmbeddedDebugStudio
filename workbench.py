"""Desktop interaction, device feedback and layout presets."""
from PySide6 import QtCore as C, QtGui as G, QtWidgets as W
import pyqtgraph as pg
from engine import KEYS
from layout_ui import STYLE


DARK = """
QMainWindow,QWidget {background:#202730;color:#e4eaf1;}
QFrame#card,QFrame#top,QFrame#pidPanel,QFrame#card QWidget,QFrame#top QLabel {background:#29323e;color:#e4eaf1;}
QPushButton,QComboBox,QLineEdit,QDoubleSpinBox,QPlainTextEdit,QToolButton {background:#25303a;color:#ecf2f6;border-color:#526071;}
QLabel#measure,QLabel#status {background:#202730;color:#c7d6de;}
QPushButton:hover {background:#384759;}
QPushButton#primary {background:#285fa3;} QPushButton#danger {background:#a33440;}
"""


class Workbench(C.QObject):
    def __init__(self,w):
        super().__init__(w)
        self.w = w
        self.hide_notice = C.QTimer(self)
        self.hide_notice.setSingleShot(True)
        self.hide_notice.timeout.connect(w.notice.hide)
        self.breath = None
        self.previous_connection = False
        self.last_badge_mode = None
        self.last_sizes = [510,190]
        w.terminal_toggle.toggled.connect(self.terminal)
        w.preset.activated.connect(self.layout)
        w.dark.toggled.connect(self.theme)
        w.reduce_motion.toggled.connect(self.motion)
        w.scope.pause_for_input = self.freeze
        w.scope.default_view = self.default_view
        w.scope.hover_report = w.measure_text.setToolTip
        W.QApplication.instance().installEventFilter(self)
        self.terminal(w.terminal_toggle.isChecked())
        self.theme()
        self.motion()

    def eventFilter(self,obj,event):
        w = self.w
        if event.type()==C.QEvent.Type.KeyPress and not event.isAutoRepeat() and w.isVisible():
            key = event.key()
            if key==C.Qt.Key.Key_Escape and w.connected:
                modal = W.QApplication.activeModalWidget()
                if isinstance(modal,W.QDialog):
                    modal.reject()
                w.stop_device()
                return True
            if W.QApplication.activeModalWidget() or not w.isActiveWindow():
                return False
            if key==C.Qt.Key.Key_F5:
                w.refresh_ports()
                return True
            if key==C.Qt.Key.Key_S and event.modifiers()==C.Qt.KeyboardModifier.ControlModifier:
                w.png()
                return True
            focus = W.QApplication.focusWidget()
            editing = isinstance(focus,(W.QLineEdit,W.QAbstractSpinBox)) or isinstance(focus,W.QPlainTextEdit) and not focus.isReadOnly()
            if key==C.Qt.Key.Key_Space and not editing:
                w.toggle_freeze()
                return True
        return super().eventFilter(obj,event)

    def freeze(self):
        if not self.w.scope.frozen and self.w.buffer.size:
            self.w.toggle_freeze()

    def default_view(self):
        w = self.w
        if w.scope.frozen:
            w.toggle_freeze()
        w.paint_scope()

    def terminal(self,opened):
        w = self.w
        if not opened and w.terminal_panel.isVisible() and min(w.vertical.sizes())>0:
            self.last_sizes = w.vertical.sizes()
        w.terminal_panel.setVisible(opened)
        w.terminal_toggle.setArrowType(C.Qt.ArrowType.DownArrow if opened else C.Qt.ArrowType.RightArrow)
        if opened:
            w.vertical.setSizes(self.last_sizes)
            w.render_log(force=True)

    def layout(self,index):
        w = self.w
        w.pid_panel.setVisible(index==0)
        w.terminal_toggle.setChecked(index==2)
        if index==2:
            w.vertical.setSizes([180,550])
        elif index==1:
            self.freeze()
            w.cards[-2].setChecked(True)
        elif w.scope.frozen:
            self.default_view()

    def theme(self):
        dark = self.w.dark.isChecked()
        self.w.setStyleSheet(STYLE+(DARK if dark else ""))
        s = self.w.scope
        s.setBackground("#19212a" if dark else "white")
        s.dark_mode = dark
        for side in ("bottom","left","right"):
            s.getAxis(side).setTextPen("#b4c7d5" if dark else "#53667a")
        s.empty.setColor("#b4c7d5" if dark else "#64778a")
        for line in s.markers:
            line.label.fill = pg.mkBrush("#25303a" if dark else "#ffffff")
            line.label.setColor("#aec1d1" if dark else "#52677d")
        self.fields()

    def motion(self):
        w = self.w
        w.scope.reduce_motion = w.reduce_motion.isChecked()
        self.last_badge_mode = None
        self.connection()

    def connection(self):
        w = self.w
        if w.connected and not self.previous_connection:
            w.cards[0].setChecked(False)
            w.cards[1].setChecked(False)
        elif self.previous_connection and not w.connected:
            w.cards[0].setChecked(True)
        self.previous_connection = w.connected
        reconnecting = getattr(w,"reconnect_at",None) is not None or getattr(w,"reconnecting",False)
        mode = ("connected" if w.connected else "opening" if w.link or reconnecting else "error" if w.connection_error else "idle")
        if mode!=self.last_badge_mode:
            self.last_badge_mode = mode
            if self.breath:
                self.breath.stop()
                self.breath.deleteLater()
                self.breath = None
            effect = w.badge.graphicsEffect()
            if not effect:
                effect = W.QGraphicsOpacityEffect(w.badge)
                w.badge.setGraphicsEffect(effect)
            effect.setOpacity(1)
            if mode=="opening" and not w.reduce_motion.isChecked():
                self.breath = C.QPropertyAnimation(effect,b"opacity",self)
                self.breath.setDuration(500)
                self.breath.setStartValue(1.)
                self.breath.setKeyValueAt(.5,.35)
                self.breath.setEndValue(1.)
                self.breath.setLoopCount(-1)
                self.breath.start()
        color = {"connected":"#188467","opening":"#8d691e","error":"#ca3945","idle":"#71808b"}[mode]
        text = w.badge.text().removeprefix("● ")
        w.badge.setText("● "+("连接异常" if mode=="error" else text))
        w.badge.setStyleSheet("color:"+color)
        w.guide.setVisible(not w.connected and not w.link and not reconnecting and not w.buffer.size)

    def fields(self):
        w = self.w
        group = w.selected()
        for key in KEYS:
            field = w.fields[key]
            prior = getattr(field,"state_animation",None)
            if prior and prior.state()==C.QAbstractAnimation.State.Running:
                prior.stop()
                if hasattr(field,"shake_origin"):
                    field.move(field.shake_origin)
            addr = group+"."+key
            confirmed = w.confirmed.get(addr)
            status = ("错误" if addr in w.field_errors else "已确认" if confirmed is not None and
                      abs(field.value()-confirmed)<.00051 else "草稿")
            color = {"错误":"#bd3947","已确认":"#218366","草稿":"#ad7921"}[status]
            field.setStyleSheet(f"QDoubleSpinBox {{border-left:4px solid {color};}}")
            field.setToolTip(status+(" · 设备值 "+format(confirmed,"g") if confirmed is not None else " · 尚未读回"))
        w.undo_btn.setEnabled(bool(w.undo_history) and w.connected)

    def animate(self,widget,start,end,paint,duration):
        prior = getattr(widget,"state_animation",None)
        if prior:
            prior.stop()
            prior.deleteLater()
        animation = C.QVariantAnimation(widget)
        widget.state_animation = animation
        animation.setDuration(duration)
        animation.setStartValue(start)
        animation.setEndValue(end)
        animation.valueChanged.connect(paint)
        animation.start()

    def field_event(self,addr,error=False):
        self.fields()
        w = self.w
        if w.reduce_motion.isChecked() or not addr.startswith(w.selected()+"."):
            return
        field = w.fields[addr.split(".")[1]]
        if error:
            origin = field.pos()
            field.shake_origin = origin
            self.animate(field,0.,1.,lambda f:field.move(origin+C.QPoint(round(3*(1-f)*(1 if int(f*6)%2 else -1)),0)),180)
        else:
            base = G.QColor("#25303a" if w.dark.isChecked() else "white")
            self.animate(field,G.QColor("#5ebb99"),base,
                         lambda c:field.setStyleSheet(f"border-left:4px solid #218366;background:{c.name()}"),400)
            field.state_animation.finished.connect(self.fields)

    def notice(self,msg,retry=False):
        self.w.notice_text.setText(msg)
        self.w.retry_btn.setVisible(retry)
        self.w.notice.show()
        self.hide_notice.start(5500)

    def stop_feedback(self):
        w = self.w
        w.stop_banner.setText("急停命令已发出 · 等待设备确认")
        w.stop_banner.show()
        if not w.reduce_motion.isChecked():
            shell = w.centralWidget()
            shell.setObjectName("shell")
            self.animate(shell,3.,0.,lambda n:shell.setStyleSheet(f"QWidget#shell {{border:{round(n)}px solid #c83f4c;}}"),200)
