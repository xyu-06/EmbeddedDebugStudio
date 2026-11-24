"""Channel calibration, target bindings and per-loop command limits."""
import math

from PySide6 import QtWidgets as W

from engine import KEYS, LOOPS, LIMIT
from layout_ui import combo, number


DEFAULT_RULES = {
    "KP": (-1000.,1000.,20.), "KI": (-1000.,1000.,50.),
    "KD": (-100.,100.,1.), "TGT": (-1000.,1000.,100.)
}


def valid_rule(values):
    return (isinstance(values,(list,tuple)) and len(values)==3
            and all(isinstance(v,(int,float)) and math.isfinite(v) and abs(v)<=LIMIT for v in values)
            and values[0]<values[1] and values[2]>0)


def finish_dialog(dialog, layout, validate):
    box = W.QDialogButtonBox(W.QDialogButtonBox.StandardButton.Ok | W.QDialogButtonBox.StandardButton.Cancel)
    box.button(W.QDialogButtonBox.StandardButton.Ok).setText("应用")
    box.button(W.QDialogButtonBox.StandardButton.Cancel).setText("取消")
    def apply():
        problem = validate()
        if problem:
            W.QMessageBox.warning(dialog,"检查设置",problem)
        else:
            dialog.accept()
    box.accepted.connect(apply)
    box.rejected.connect(dialog.reject)
    layout.addWidget(box)
    return dialog.exec() == W.QDialog.DialogCode.Accepted


def calibration_dialog(w):
    dialog = W.QDialog(w)
    dialog.setWindowTitle("通道换算与坐标轴")
    dialog.setMinimumWidth(370)
    outer = W.QVBoxLayout(dialog)
    outer.addWidget(W.QLabel("显示值 = 收到的数值 × 系数"))
    grid = W.QGridLayout()
    outer.addLayout(grid)
    rows = []
    for col,text in enumerate(("通道","系数","单位")):
        grid.addWidget(W.QLabel(text),0,col)
    for i in range(4):
        factor = number(w.scales[i],-LIMIT,LIMIT,.01,6)
        factor.setSizePolicy(W.QSizePolicy.Policy.Preferred,W.QSizePolicy.Policy.Fixed)
        factor.setMinimumWidth(100)
        unit = W.QLineEdit(w.units[i])
        unit.setMaxLength(16)
        grid.addWidget(W.QLabel(f"CH{i+1}"),i+1,0)
        grid.addWidget(factor,i+1,1)
        grid.addWidget(unit,i+1,2)
        rows.append((factor,unit))
    form = W.QFormLayout()
    outer.addLayout(form)
    axis = combo(["共用左轴","CH1","CH2","CH3","CH4"],w.right_channel+1)
    form.addRow("独立右轴",axis)
    bindings = {}
    for i in w.active_refs():
        bind = combo(["手动输入",*LOOPS],LOOPS.index(w.follow[i])+1 if w.follow[i] in LOOPS else 0)
        form.addRow(w.ref_name(i)+" · 跟随 TGT",bind)
        bindings[i] = bind
    if finish_dialog(dialog,outer,lambda: "换算系数不能为零" if any(f.value()==0 for f,_ in rows) else ""):
        following = w.follow.copy()
        for i,bind in bindings.items():
            following[i] = bind.currentText() if bind.currentIndex() else ""
        w.configure_channels([f.value() for f,_ in rows],[u.text().strip() for _,u in rows],axis.currentIndex()-1,following)


def limits_dialog(w):
    group = w.selected()
    dialog = W.QDialog(w)
    dialog.setWindowTitle(group+" 参数保护")
    dialog.setMinimumWidth(460)
    outer = W.QVBoxLayout(dialog)
    grid = W.QGridLayout()
    outer.addLayout(grid)
    for col,text in enumerate(("参数","最小值","最大值","变化确认阈值")):
        grid.addWidget(W.QLabel(text),0,col)
    fields = {}
    for i,key in enumerate(KEYS,1):
        grid.addWidget(W.QLabel(key),i,0)
        fields[key] = []
        for j,v in enumerate(w.rules[group][key],1):
            widget = number(v,.0001 if j==3 else -LIMIT,LIMIT,.1,4)
            widget.setMinimumWidth(95)
            grid.addWidget(widget,i,j)
            fields[key].append(widget)
    def values():
        return {k:tuple(v.value() for v in fields[k]) for k in KEYS}
    if finish_dialog(dialog,outer,lambda: "" if all(map(valid_rule,values().values())) else "每个下限须小于上限，确认阈值须大于零"):
        w.rules[group] = values()
