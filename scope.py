from collections import deque
from html import escape
from math import ceil

import numpy as np
import pyqtgraph as pg
from PySide6 import QtCore, QtWidgets


COLORS = ("#1b62c7", "#008673", "#d47b12", "#8651b8")
REF_COLORS = ("#b42e57", "#875a28", "#356a84", "#6e718a")

class DivisionAxis(pg.AxisItem):
    division = 0.

    def tickStrings(self,values,scale,spacing):
        if self.division>0:
            return [f"{value*scale:.10g}" for value in values]
        return super().tickStrings(values,scale,spacing)

    def tickSpacing(self,minVal,maxVal,size):
        if self.division <= 0:
            return super().tickSpacing(minVal,maxVal,size)
        base = self.division*self.scale
        # Merge adjacent divisions only when there is no room to label them.
        min_pixels = 44 if self.orientation == "bottom" else 22
        multiple = max(1,ceil((maxVal-minVal)/base*min_pixels/max(1,size)))
        return [(base*multiple,0)]


def pixel_mean(x,y,left,right,width):
    """Aggregate samples sharing a screen pixel, keeping gaps between finite runs."""
    if len(x)<2 or right<=left:
        return x,y
    bins = np.floor(np.clip((x-left)/(right-left),-1.,2.)*width).astype(np.int64)
    starts = np.r_[0,np.flatnonzero((bins[1:]!=bins[:-1]) | (np.isfinite(y[1:])!=np.isfinite(y[:-1])))+1]
    if len(starts)==len(x):
        return x,y
    counts = np.diff(np.r_[starts,len(x)])
    return np.add.reduceat(x,starts)/counts, np.add.reduceat(y,starts)/counts


class Scope(pg.PlotWidget):
    def __init__(self):
        axes = {side:DivisionAxis(orientation=side) for side in ("bottom","left")}
        super().__init__(background="white",axisItems=axes)
        self.setMenuEnabled(False)
        self.hideButtons()
        self.setLabel("bottom", "时间", units="s")
        for axis in ("left", "bottom"):
            self.getAxis(axis).enableAutoSIPrefix(False)
            self.getAxis(axis).setPen("#afbdca")
            self.getAxis(axis).setTextPen("#53667a")
        self.legend = self.addLegend(offset=(12, 8), labelTextColor="#374a60",
                                     brush=pg.mkBrush(255, 255, 255, 220))
        self.curves = [self.plot(pen=pg.mkPen(c, width=2), name="CH" + str(i + 1))
                       for i, c in enumerate(COLORS)]
        self.right_channel = -1
        self.secondary = pg.ViewBox(enableMenu=False)
        self.scene().addItem(self.secondary)
        self.secondary.setXLink(self.getViewBox())
        self.secondary.setMouseEnabled(x=False,y=False)
        self.secondary.setZValue(-90)
        self.getAxis("right").linkToView(self.secondary)
        self.getAxis("right").enableAutoSIPrefix(False)
        self.getViewBox().sigResized.connect(self.align_axes)
        self.align_axes()
        self.legend.clear()
        self.legend_channels = ()
        self.references = []
        for i, color in enumerate(REF_COLORS):
            guide = pg.InfiniteLine(angle=0, movable=False,
                                    pen=pg.mkPen(color, width=1.5, style=QtCore.Qt.PenStyle.DashLine),
                                    label="{value:g}", labelOpts={"position": .94 - i*.15, "color": color,
                                                                "fill": (255,255,255,220)})
            self.addItem(guide, ignoreBounds=True)
            guide.hide()
            self.references.append(guide)
        self.markers = deque()
        self.trigger_guide = pg.InfiniteLine(angle=0,movable=False,pen=pg.mkPen("#667684",style=QtCore.Qt.PenStyle.DotLine))
        self.addItem(self.trigger_guide,ignoreBounds=True)
        self.trigger_guide.hide()
        self.cursors = []
        self.frozen = False
        self.data = np.empty((0, 9))
        self.names = ["目标值", "实际值", "输出", "误差"]
        self.visible = [True] * 4
        self.width_s = 10
        self.fixed_y = None
        self.include_zero = True
        self.average_pixels = True
        self.drawn_view = None
        self.setMouseEnabled(x=False, y=False)
        self.showGrid(x=True, y=True, alpha=.16)
        self.getViewBox().setDefaultPadding(.03)
        self.scene().sigMouseClicked.connect(self.click_plot)
        self.empty = pg.TextItem("等待采样", color="#64778a", anchor=(.5, .5))
        self.addItem(self.empty, ignoreBounds=True)
        self.sigRangeChanged.connect(self.position_hint)
        self.setXRange(0, 10, padding=0)
        self.setYRange(-1, 1, padding=.03)
        self.measure = None
        self.pause_for_input = self.default_view = None
        self.reduce_motion = False
        self.dark_mode = False
        self.press = None
        self.selection = pg.LinearRegionItem(movable=False,brush=(70,130,200,40))
        self.addItem(self.selection,ignoreBounds=True)
        self.selection.hide()
        self.cross = [pg.InfiniteLine(angle=a,pen=pg.mkPen("#8395a5",style=QtCore.Qt.PenStyle.DotLine)) for a in (90,0)]
        for item in self.cross:
            self.addItem(item,ignoreBounds=True)
            item.hide()
        self.hover_label = pg.TextItem(anchor=(0,1),color="#263c53",fill=(248,250,253,225))
        self.addItem(self.hover_label,ignoreBounds=True)
        self.hover_label.setZValue(20)
        self.hover_label.hide()
        self.hover_proxy = pg.SignalProxy(self.scene().sigMouseMoved,rateLimit=30,slot=self.hover)
        self.step_items = []
        self.step_view = None
        self.step_signature = None

    def hover(self,args):
        pos = args[0]
        box = self.getViewBox()
        show = len(self.data)>0 and box.sceneBoundingRect().contains(pos) and self.underMouse()
        for item in (*self.cross,self.hover_label):
            item.setVisible(show)
        if not show:
            return
        point = box.mapSceneToView(pos)
        times = self.data[:,0]
        i = min(len(times)-1,np.searchsorted(times,point.x()))
        if i and abs(times[i-1]-point.x())<abs(times[i]-point.x()):
            i -= 1
        self.cross[0].setValue(times[i])
        self.cross[1].setValue(point.y())
        values = [f"{name} {self.data[i,c+1]:.5g}" for c,name in self.legend_channels if np.isfinite(self.data[i,c+1])]
        self.hover_label.setText(f"{times[i]:.3f} s  "+"   ".join(values))
        self.hover_label.setAnchor((1 if point.x()>sum(self.viewRange()[0])/2 else 0,1))
        self.hover_label.setPos(times[i],point.y())

    def leaveEvent(self,event):
        for item in (*self.cross,self.hover_label):
            item.hide()
        super().leaveEvent(event)

    def wheelEvent(self,event):
        if self.pause_for_input and len(self.data):
            self.pause_for_input()
        super().wheelEvent(event)

    def mouseDoubleClickEvent(self,event):
        if self.default_view:
            self.press = None
            self.default_view()
            event.accept()
            return
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self,event):
        scene = self.mapToScene(event.position().toPoint())
        if event.button()==QtCore.Qt.MouseButton.LeftButton and len(self.data) and self.getViewBox().sceneBoundingRect().contains(scene):
            self.pause_for_input()
            point = self.getViewBox().mapSceneToView(scene)
            self.press = (event.position(),point.x(),self.viewRange(),bool(event.modifiers() & QtCore.Qt.KeyboardModifier.ShiftModifier))
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseMoveEvent(self,event):
        if self.press:
            origin,start,ranges,selecting = self.press
            delta = event.position()-origin
            if delta.manhattanLength()>3:
                vb = self.getViewBox()
                if selecting:
                    end = vb.mapSceneToView(self.mapToScene(event.position().toPoint())).x()
                    self.selection.setRegion(sorted((start,end)))
                    self.selection.show()
                else:
                    dx = -delta.x()*(ranges[0][1]-ranges[0][0])/max(1,vb.width())
                    dy = delta.y()*(ranges[1][1]-ranges[1][0])/max(1,vb.height())
                    self.setRange(xRange=[x+dx for x in ranges[0]],yRange=[y+dy for y in ranges[1]],padding=0)
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self,event):
        if self.press and event.button()==QtCore.Qt.MouseButton.LeftButton:
            origin,start,_,selecting = self.press
            self.press = None
            if (event.position()-origin).manhattanLength()<=3:
                self.place_cursor(start)
            elif selecting and self.selection.isVisible():
                self.setXRange(*self.selection.getRegion(),padding=0)
                self.selection.hide()
                if self.measure:
                    self.measure()
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def clear_step(self):
        for item in self.step_items:
            self.step_view.removeItem(item)
        self.step_items.clear()
        self.step_signature = None

    def step_overlay(self,data,target,actual,band,metrics):
        targets = data[:,target]
        k = np.flatnonzero(~np.isclose(targets[1:],targets[:-1],rtol=1e-7,atol=1e-9))[-1]+1
        t,y = data[k:,0],data[k:,actual]
        goal,y0 = targets[k],data[k-1,actual]
        z = (y-y0)/(goal-y0)
        peak = int(np.argmax(z))
        view = self.secondary if actual-1==self.right_channel else self.getViewBox()
        signature = (t[0],goal,y0,band,t[peak],y[peak],metrics[1],metrics[2],view,actual)
        if self.step_signature!=signature:
            self.clear_step()
            self.step_signature,self.step_view = signature,view
            self.step_items = [QtWidgets.QGraphicsRectItem()]
            rect = self.step_items[0]
            rect.setBrush(pg.mkBrush(43,153,126,32))
            rect.setPen(pg.mkPen(None))
            rect.setZValue(-10)
            dot = pg.ScatterPlotItem([t[peak]],[y[peak]],size=7,brush="#bb7234",pen=None)
            label = pg.TextItem(f"峰值 {y[peak]:.4g}",color="#c98a47",anchor=(1,1))
            label.setPos(t[peak],y[peak])
            self.step_items.extend((dot,label))
            if metrics[1] is not None:
                i = np.flatnonzero(z>=.9)[0]
                t90 = t[0] if i==0 else t[i-1]+(.9-z[i-1])*(t[i]-t[i-1])/(z[i]-z[i-1])
                for stamp,name in ((t90-metrics[1],"10%"),(t90,f"90%　{metrics[1]:.3g} s")):
                    self.step_items.append(pg.InfiniteLine(stamp,angle=90,pen=pg.mkPen("#738fa4",style=QtCore.Qt.PenStyle.DotLine),
                        label=name,labelOpts={"position":.18,"color":"#738fa4"}))
            if metrics[2] is not None:
                self.step_items.append(pg.InfiniteLine(t[0]+metrics[2],angle=90,pen="#579785",
                    label=f"调节 {metrics[2]:.3g} s",labelOpts={"position":.32,"color":"#579785"}))
            for item in self.step_items:
                view.addItem(item,ignoreBounds=True)
        span = abs(goal-y0)*band
        self.step_items[0].setRect(QtCore.QRectF(t[0],goal-span,t[-1]-t[0],2*span))

    def align_axes(self):
        self.secondary.setGeometry(self.getViewBox().sceneBoundingRect())
        self.secondary.linkedViewChanged(self.getViewBox(),pg.ViewBox.XAxis)

    def shutdown(self):
        self.hover_proxy.disconnect()
        self.getViewBox().sigResized.disconnect(self.align_axes)
        self.getAxis("right").unlinkFromView()
        self.secondary.setXLink(None)
        self.secondary.close()
        self.scene().removeItem(self.secondary)
        self.secondary.deleteLater()
        self.getViewBox().unregister()

    def set_right_channel(self,channel):
        if self.right_channel != channel:
            if self.right_channel >= 0:
                curve = self.curves[self.right_channel]
                self.secondary.removeItem(curve)
                self.getViewBox().addItem(curve)
            self.right_channel = channel
            if channel >= 0:
                curve = self.curves[channel]
                self.getViewBox().removeItem(curve)
                self.secondary.addItem(curve)
            self.showAxis("right",channel>=0)
            self.align_axes()
        if channel>=0:
            self.getAxis("right").setLabel(escape(self.names[channel]))

    def trigger_axis(self,channel):
        view = self.secondary if channel==self.right_channel and channel>=0 else self.getViewBox()
        old = self.trigger_guide.getViewBox()
        if old is not view:
            old.removeItem(self.trigger_guide)
            view.addItem(self.trigger_guide,ignoreBounds=True)

    def position_hint(self, *_):
        x, y = self.viewRange()
        self.empty.setPos(sum(x)/2, sum(y)/2)

    def reset(self):
        self.clear_step()
        self.selection.hide()
        self.press = None
        self.data = np.empty((0, 9))
        for curve in self.curves:
            curve.clear()
        for item in (*self.markers, *self.cursors):
            self.removeItem(item)
        self.markers.clear()
        self.cursors.clear()
        self.empty.show()
        self.legend.hide()
        self.setXRange(0, self.width_s, padding=0)
        self.setYRange(-1, 1, padding=.03)

    def set_frozen(self, value):
        self.frozen = value
        self.setMouseEnabled(x=value, y=value)
        if not value:
            for item in self.cursors:
                self.removeItem(item)
            self.cursors.clear()

    def set_names(self, names):
        self.names = names[:]

    def viewport_key(self):
        left,right = self.viewRange()[0]
        return left,right,max(1,round(self.getViewBox().width()))

    def set_window(self, seconds):
        if self.width_s == seconds:
            return
        self.width_s = seconds
        if self.frozen:
            right = max(seconds,self.viewRange()[0][1])
            self.setXRange(right-seconds,right,padding=0)
        elif not len(self.data):
            self.setXRange(0,seconds,padding=0)

    def set_reference(self, slot, name, value, active):
        guide = self.references[slot]
        guide.setVisible(bool(active))
        if active:
            guide.setValue(value)
            guide.label.setFormat(name.replace("{","{{").replace("}","}}") + "  {value:g}")

    def draw(self, a, count, show_legend):
        self.data = a
        levels = [guide.value() for guide in self.references if guide.isVisible()]
        channels = tuple((i,self.names[i]) for i in range(count) if self.visible[i])
        if channels != self.legend_channels:
            self.legend.clear()
            for i,name in channels:
                self.legend.addItem(self.curves[i],escape(name))
            self.legend_channels = channels
        if not len(a):
            self.empty.setVisible(not levels)
            self.legend.hide()
            if levels and not self.fixed_y:
                low, high = min(0., *levels), max(0., *levels)
                self.setYRange(low, max(low+.1,high), padding=.1)
            elif self.fixed_y:
                self.setYRange(*self.fixed_y, padding=0)
            return
        self.empty.hide()
        self.legend.hide()  # Interactive channel chips live above the plot.
        newest = a[-1, 0]
        start = max(0., newest - self.width_s)
        if not self.frozen:
            self.setXRange(start, max(self.width_s, newest), padding=0)
        left,right,width = self.viewport_key()
        self.drawn_view = left,right,width
        first = max(0,np.searchsorted(a[:,0],left)-1)
        end = np.searchsorted(a[:,0],right,side="right")+1
        use = a[first:end]
        for i, curve in enumerate(self.curves):
            on = i < count and self.visible[i]
            curve.setVisible(on)
            if on:
                x,y = use[:,0],use[:,i+1]
                if self.average_pixels:
                    x,y = pixel_mean(x,y,left,right,width)
                curve.setData(x,y,autoDownsample=False,connect="finite")
        if self.right_channel>=0 and self.visible[self.right_channel]:
            y = use[:,self.right_channel+1]
            y = y[np.isfinite(y)]
            if len(y):
                low,high = y.min(),y.max()
                if self.include_zero:
                    low,high = min(0.,low),max(0.,high)
                pad = max(.1,(high-low)*.09)
                self.secondary.setYRange(low-pad,high+pad,padding=0)
        if self.fixed_y:
            self.setYRange(*self.fixed_y, padding=0)
        elif not self.frozen:
            shown = [use[:, i+1] for i in range(count) if self.visible[i] and i!=self.right_channel]
            for values in shown:
                valid = values[np.isfinite(values)]
                if len(valid):
                    levels.extend((valid.min(),valid.max()))
            if levels:
                if self.include_zero:
                    levels.append(0.)
                low, high = min(levels), max(levels)
                pad = max((high-low)*.09, .1)
                self.setYRange(low-pad, high+pad, padding=0)

    def readouts(self, statistics):
        """Update legend values and measure only samples inside the visible time range."""
        if not len(self.data):
            return []
        left,right = self.viewRange()[0]
        t = self.data[:,0]
        view = self.data[np.searchsorted(t,left):np.searchsorted(t,right,side="right")]
        rows = []
        for i,name in self.legend_channels:
            label = self.legend.getLabel(self.curves[i])
            y = view[:,i+1]
            value = f"{y[-1]:.5g}" if len(y) and np.isfinite(y[-1]) else "—"
            text = f"{escape(name)}　{value}"
            if label.text != text:
                label.setText(text,color="#374a60")
            if not statistics:
                continue
            valid = y[np.isfinite(y)]
            if not len(valid):
                continue
            low,high,mean = valid.min(),valid.max(),valid.mean()
            summary = f"{name}　最小 {low:.5g}　最大 {high:.5g}　均值 {mean:.5g}　峰峰值 {high-low:.5g}"
            # Require three upwards crossings of the mean within an uninterrupted trace.
            if len(valid) == len(y):
                hits = np.flatnonzero((y[:-1]<mean) & (y[1:]>=mean))
                if len(hits)>=3:
                    times = view[:,0]
                    cross = times[hits]+(mean-y[hits])*(times[hits+1]-times[hits])/(y[hits+1]-y[hits])
                    period = (cross[-1]-cross[0])/(len(cross)-1)
                    if period>0:
                        summary += f"　周期 ≈ {period:.4g} s（{1/period:.4g} Hz）"
            rows.append(summary)
        return rows

    def mark(self, t, label):
        if self.markers:
            previous_fade = getattr(self.markers[-1],"fade",None)
            if previous_fade:
                previous_fade.stop()
            self.markers[-1].setOpacity(.53)
            self.markers[-1].setPen(pg.mkPen("#8496aa",style=QtCore.Qt.PenStyle.DashLine))
        line = pg.InfiniteLine(t, angle=90, movable=False, pen=pg.mkPen("#8496aa", width=2, style=QtCore.Qt.PenStyle.DashLine),
                               label=label.replace("{","{{").replace("}","}}"), labelOpts={"position": .78 - .09*(len(self.markers)%3),
                                                       "color": "#aec1d1" if self.dark_mode else "#52677d",
                                                       "fill": "#25303a" if self.dark_mode else (255,255,255,220)})
        self.addItem(line, ignoreBounds=True)
        if not self.reduce_motion:
            fade = QtCore.QPropertyAnimation(line,b"opacity",line)
            line.fade = fade
            fade.setDuration(170)
            fade.setStartValue(.18)
            fade.setEndValue(1.)
            fade.start()
        self.markers.append(line)
        if len(self.markers) > 100:
            self.removeItem(self.markers.popleft())

    def click_plot(self, event):
        if not self.frozen or not len(self.data) or event.button() != QtCore.Qt.MouseButton.LeftButton:
            return
        pos = self.getViewBox().mapSceneToView(event.scenePos())
        if not self.getViewBox().sceneBoundingRect().contains(event.scenePos()):
            return
        self.place_cursor(pos.x())

    def place_cursor(self,x):
        x = self.data[np.argmin(abs(self.data[:, 0]-x)), 0]
        if len(self.cursors) == 2:
            for item in self.cursors:
                self.removeItem(item)
            self.cursors.clear()
        item = pg.InfiniteLine(x, angle=90, movable=False, pen="#ad513a",
                               label="AB"[len(self.cursors)], labelOpts={"position": .12})
        self.addItem(item, ignoreBounds=True)
        self.cursors.append(item)
        if self.measure:
            self.measure()

    def measurement(self, channel):
        if channel is None or not self.cursors or not len(self.data):
            return ""
        points = [self.data[np.argmin(abs(self.data[:, 0]-c.value()))] for c in self.cursors]
        a = points[0]
        s = f"A  {a[0]:.3f} s   {self.names[channel]} {a[channel+1]:.4g}"
        if len(points) == 2:
            d = points[1] - a
            s += f"     Δt  {d[0]:.3f} s     ΔY  {d[channel+1]:.4g}"
        return s

