"""One-shot capture controls and their lifecycle."""
import numpy as np


class CaptureController:
    def __init__(self, window):
        self.ui = window
        self.capture_channel = self.trigger_at = self.previous_trigger_value = None
        self.last_crossing = None
        self.last_range = None

    def arm_capture(self):
        if self.capture_channel is not None:
            self.cancel_capture("已取消")
            return
        channel = self.ui.trigger_ch.currentData()
        if not self.ui.connected or channel is None or channel>=self.ui.count or not self.ui.buffer.size:
            self.ui.trigger_status.setText("请先连接并接收该通道的数据")
            return
        if self.ui.scope.frozen:
            self.ui.toggle_freeze()
        self.capture_channel = channel
        self.previous_trigger_value = self.ui.buffer.data[(self.ui.buffer.end-1)%self.ui.buffer.capacity,channel+1]
        self.trigger_at = None
        self.ui.scope.trigger_guide.setValue(self.ui.trigger_level.value())
        self.ui.scope.trigger_axis(channel)
        self.ui.scope.trigger_guide.show()
        for widget in (self.ui.trigger_ch,self.ui.trigger_edge,self.ui.trigger_level,self.ui.capture_seconds,self.ui.pre_percent):
            widget.setEnabled(False)
        self.ui.arm_btn.setText("取消捕获")
        self.ui.trigger_status.setText("等待边沿…")

    def cancel_capture(self,message):
        if self.capture_channel is None:
            return
        self.capture_channel = self.previous_trigger_value = self.trigger_at = None
        self.ui.scope.trigger_guide.hide()
        for widget in (self.ui.trigger_ch,self.ui.trigger_edge,self.ui.trigger_level,self.ui.capture_seconds,self.ui.pre_percent):
            widget.setEnabled(True)
        self.ui.arm_btn.setText("单次触发")
        self.ui.trigger_status.setText(message)

    def check_capture(self,t,values):
        if self.capture_channel >= len(values):
            self.cancel_capture("该通道已停止上报")
            return
        if self.trigger_at is None:
            current = values[self.capture_channel]
            previous = self.previous_trigger_value
            self.previous_trigger_value = current
            level = self.ui.trigger_level.value()
            edge = self.ui.trigger_edge.currentIndex()
            rising = previous < level <= current
            falling = previous > level >= current
            if not ((edge!=1 and rising) or (edge!=0 and falling)):
                return
            self.trigger_at = t
            self.ui.trigger_status.setText("已触发，采集中…")
        start = self.trigger_at-self.ui.capture_seconds.value()*self.ui.pre_percent.value()/100.
        end = start+self.ui.capture_seconds.value()
        if t+1e-10 < end:
            return
        crossing = self.trigger_at
        self.cancel_capture("已捕获，点击“继续”恢复")
        self.ui.toggle_freeze()
        data = self.ui.scope.data
        if data[0,0] > start+1e-8:
            self.ui.trigger_status.setText("已捕获，窗口前段无可用样本")
        self.ui.scope.setXRange(start,end,padding=0)
        self.ui.scope.mark(crossing,"触发")
        if not self.ui.scope.fixed_y:
            captured = data[(data[:,0]>=start) & (data[:,0]<=end)]
            levels = [line.value() for line in self.ui.scope.references if line.isVisible()]
            for i,_ in self.ui.scope.legend_channels:
                if i == self.ui.scope.right_channel:
                    continue
                y = captured[:,i+1]
                valid = y[np.isfinite(y)]
                if len(valid):
                    levels.extend((valid.min(),valid.max()))
            if levels:
                if self.ui.scope.include_zero:
                    levels.append(0.)
                low,high = min(levels),max(levels)
                pad = max((high-low)*.09,.1)
                self.ui.scope.setYRange(low-pad,high+pad,padding=0)
        self.last_crossing = crossing
        self.last_range = (start, end)
        if hasattr(self.ui, "on_capture_complete"):
            self.ui.on_capture_complete(crossing, start, end)
        self.ui.measure()

