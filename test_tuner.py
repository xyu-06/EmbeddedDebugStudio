"""Regression checks. Run with the dependencies in requirements.txt installed."""
import csv
import math
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM","offscreen")

import numpy as np
from PySide6 import QtCore as C, QtGui, QtWidgets as W
from autotune import AutoTuner, evaluate_step
from diagnostics import RuntimeMonitor, analyze_frame_timing, dominant_fft, format_c_code
from engine import Lines, MAX_CHANNELS, REF_BASE, Simulator, match_ack, step_metrics
from generator import TargetGenerator
from main import Window
from remote_studio import AtCommandBuilder, button_axis, format_drive_command, held_speed, mix_wheels
from serial_studio import append_checksum, crc16_modbus
from storage import History, load_csv
from transport import SerialLink
from scope import pixel_mean


class CoreChecks(unittest.TestCase):
    def test_pixel_averages_keep_raw_ripple_and_missing_data(self):
        x = np.arange(200)/1000
        y = np.tile([165.,165.8],100)
        _,trend = pixel_mean(x,y,0.,1.,1)
        np.testing.assert_allclose(trend,[165.4])
        self.assertAlmostEqual(np.ptp(y),.8)
        y[20:30] = np.nan
        _,trend = pixel_mean(x,y,0.,1.,1)
        np.testing.assert_allclose(trend,[165.4,np.nan,165.4],equal_nan=True)
        np.testing.assert_allclose(pixel_mean(x,y,0.,.2,1000)[1],y,equal_nan=True)

    def test_stream_recovery(self):
        p = Lines()
        self.assertEqual(list(p.feed(b"12,")),[])
        events = list(p.feed(b"3\r\n#NAMES:t,y\nwave:4,5\nangle:1,gyro:2\n1,2,3,4,5\n\xff\ninf\n1,2,3,4,5,6,7,8,9\n"))
        self.assertEqual(events[:5],[("data",[12.,3.]),("names",["t","y"]),("data",[4.,5.]),("data",[1.,2.]),("data",[1.,2.,3.,4.,5.])])
        self.assertEqual([x[0] for x in events[5:]],["bad"]*3)
        self.assertEqual([x[0] for x in p.feed(b"9"*2049)],["bad"])
        self.assertEqual(list(p.feed(b"12\n5\n")),[("data",[5.])])

    def test_ack(self):
        self.assertTrue(match_ack("@ANG.KP=1.2345","ANG.KP=1.235"))
        self.assertFalse(match_ack("@ANG.KP=1","SPD.KP=1"))
        self.assertFalse(match_ack("@ANG.KP=1","ANG.KP=2"))
        self.assertFalse(match_ack("@STOP","RUN"))

    def test_steps(self):
        t = np.arange(0,10,.01)
        for direction in [1,-1]:
            y = np.where(t<1,0, direction*(1-np.exp(-np.maximum(0,t-1))))
            sp = direction*(t>=1)
            overshoot,rise,settle,err,_ = step_metrics(t,sp,y)
            self.assertEqual(overshoot,0)
            self.assertAlmostEqual(rise,math.log(9),places=3)
            self.assertAlmostEqual(settle,3.92,places=2)
            self.assertLess(abs(err),.001)
        self.assertIsNone(step_metrics(t,np.ones_like(t),t))

    def test_four_channels_rolling(self):
        b = History()
        for i in range(30000):
            b.add(i*.01,[i,2,3,4],(i,))
        self.assertEqual(b.view().shape,(20000,5))
        self.assertEqual(b.view()[0,1],10000)
        self.assertEqual(b.snapshots()[-1],(29999,))

    def test_serial_loopback(self):
        link = SerialLink({"url":"loop://","baudrate":115200})
        link.start()
        try:
            link.send(b"10,20,30,40\n")
            received = bytearray()
            deadline = time.monotonic()+2
            while time.monotonic()<deadline and not received.endswith(b"\n"):
                events,_ = link.take()
                for kind,_,data in events:
                    if kind == "rx":
                        received.extend(data)
                time.sleep(.005)
            self.assertEqual(received,b"10,20,30,40\n")
        finally:
            link.closed.set()
            link.join(1)

    def test_all_models(self):
        for model in range(4):
            sim = Simulator(model)
            sim.command("@SPD.KP=3")
            self.assertEqual(sim.params["ANG"]["KP"],2)
            sim.command("@RUN")
            for k in range(600):
                if k % 20 == 0:
                    sim.ping()
                samples = sim.step("SPD")
            self.assertTrue(all(np.isfinite(samples)))
            sim.command("@STOP")
            self.assertEqual(sim.step("SPD")[2],0)

    def test_pendulum_balances_and_falls(self):
        sim = Simulator(3)
        self.assertEqual(sim.command("@RUN"),"#CAP:PING\n#CAP:SET\n@ACK:RUN")
        for k in range(500):
            if k % 20 == 0:
                sim.ping()
            target,angle,_,_ = sim.step("ANG")
        self.assertTrue(sim.running)
        self.assertAlmostEqual(angle,target,delta=.65)
        sim.nudge("ANG", 22.0)
        for k in range(400):
            if k % 20 == 0:
                sim.ping()
            target,angle,_,_ = sim.step("ANG")
        self.assertTrue(sim.running)
        self.assertAlmostEqual(angle,target,delta=.65)
        sim.command("@STOP")
        for key in ("KP","KI","KD"):
            sim.command(f"@ANG.{key}=0")
        sim.command("@RUN")
        for k in range(300):
            if k % 20 == 0:
                sim.ping()
            sim.step("ANG")
            if not sim.running:
                break
        self.assertFalse(sim.running)
        self.assertEqual(sim.drain(),["#STOP:fall"])
        self.assertEqual(sim.step("ANG")[2],0)

    def test_simulated_heartbeat(self):
        sim = Simulator(0)
        sim.command("@RUN")
        for _ in range(55):
            sim.step("ANG")
        self.assertFalse(sim.running)       # matches firmware: stops 0.5 s after @RUN if never pinged
        self.assertEqual(sim.drain(),["#STOP:heartbeat"])
        sim.command("@RUN")
        self.assertEqual(sim.command("@PING"),"")
        for _ in range(40):
            sim.step("ANG")
        self.assertTrue(sim.running)
        sim.ping()
        for _ in range(40):
            sim.step("ANG")
        self.assertTrue(sim.running)
        for _ in range(55):
            sim.step("ANG")
        self.assertFalse(sim.running)
        self.assertEqual(sim.drain(),["#STOP:heartbeat"])

    def test_metrics_ignore_missing_samples(self):
        t = np.arange(0,5,.01)
        y = np.where(t<1,0,1-np.exp(-np.maximum(0,t-1)))
        y[150:155] = np.nan
        metrics = step_metrics(t,(t>=1).astype(float),y)
        self.assertTrue(all(m is None or np.isfinite(m) for m in metrics))
        self.assertGreater(metrics[4],0)

    def test_generator_shapes(self):
        square = TargetGenerator(1,10,5,1.)
        self.assertEqual([square.value(x) for x in (0,.4,.6,1.1)],[15,15,10,15])
        sine = TargetGenerator(2,0,2,4.)
        self.assertAlmostEqual(sine.value(1.),2)
        self.assertEqual(sine.bounds(),(-2,2))
        tri = TargetGenerator(3,0,4,4.)
        self.assertEqual([tri.value(x) for x in (0,1,2,3)],[0,4,0,-4])
        step = TargetGenerator(0,1,2,.5)
        self.assertEqual(step.due(100.),3)
        self.assertIsNone(step.due(100.2))
        self.assertEqual(step.due(100.6),1)
        self.assertIs(step.due(100.7),StopIteration)
        sine = TargetGenerator(2,0,1,1.,rate=10)
        self.assertIsNotNone(sine.due(0.))
        self.assertIsNone(sine.due(.05))
        self.assertIsNotNone(sine.due(.11))
        with self.assertRaises(ValueError):
            TargetGenerator(1,0,1,0)

    def test_autotuner_identifies_and_optimizes_plants(self):
        rules = {"KP":(-1000.,1000.,20.),"KI":(-1000.,1000.,50.),"KD":(-100.,100.,1.)}
        def closed_loop_run(model,mode,strategy,init,target,obs_s=2.0,rounds=8):
            sim = Simulator(model)
            sim.params["ANG"].update(init,TGT=0.)
            sim.command("@RUN")
            tuner = AutoTuner(mode,1,strategy,init,rules,0.,target,obs_s,rounds)
            while True:
                nxt = tuner.next_candidate()
                if nxt is None:
                    break
                _,cand,_ = nxt
                sim.params["ANG"].update(cand,TGT=0.)
                sim.command("@ANG.RST")
                for step_i in range(int(1.0/sim.dt)):
                    if step_i % 20 == 0:
                        sim.ping()
                    sim.step("ANG")
                base_y = sim.states["ANG"][0]
                sim.command("@ANG.RST")
                sim.params["ANG"]["TGT"] = target
                ts,ys,us = [],[],[]
                for k in range(int(obs_s/sim.dt)):
                    if k % 20 == 0:
                        sim.ping()
                    _,y,u,_ = sim.step("ANG")
                    ts.append(k*sim.dt); ys.append(y); us.append(u)
                tuner.record_result(ts,ys,base_y=base_y,outputs=us)
            return tuner

        t0 = closed_loop_run(0,0,0,{"KP":.25,"KI":.05,"KD":0.},60.)
        self.assertLess(t0.best_eval.score, 0.35)
        self.assertLess(t0.best_eval.overshoot_pct, 5.0)
        self.assertLess(t0.best_eval.score, t0.baseline_eval.score * 0.15)

        # Actuator-ceiling target (100 == u_max): flags saturation and stays bounded
        t_sat = closed_loop_run(0,0,0,{"KP":.25,"KI":.05,"KD":0.},100.,rounds=6)
        self.assertLess(t_sat.best_eval.score, 0.75)
        self.assertLess(t_sat.best_params["KP"], 50.0)

        t1 = closed_loop_run(1,1,0,{"KP":4.,"KI":5.,"KD":0.},50.,obs_s=2.4)
        self.assertLess(t1.best_eval.score, 0.32)
        self.assertGreater(t1.best_params["KD"], 0.5)

        t2 = closed_loop_run(2,0,0,{"KP":0.,"KI":0.,"KD":0.},60.,obs_s=2.8)
        self.assertLess(t2.best_eval.score, 0.40)

        # Reverse feedback polarity stops immediately on probe
        bad = AutoTuner(0,1,0,{"KP":1.,"KI":0.,"KD":0.},rules,0.,50.,1.0,6)
        bad.next_candidate()
        ev = bad.record_result(np.linspace(0,1,50), -np.linspace(0,20,50), base_y=0.)
        self.assertTrue(ev.reversed_sign)
        self.assertTrue(bad.finished)

    def test_fft_timing_watchdog_and_c_codegen(self):
        t = np.arange(0, 4.0, 0.005)
        y = 12.0 + 3.5 * np.sin(2 * math.pi * 7.5 * t)
        freq, amp = dominant_fft(t, y)
        self.assertAlmostEqual(freq, 7.5, delta=0.35)
        self.assertGreater(amp, 2.5)

        # Coalesced USB-TTL timestamps (pairs arriving 0.1ms apart every 20ms)
        batched = np.ravel(np.column_stack((np.arange(0, 1.0, 0.02), np.arange(0, 1.0, 0.02) + 0.0001)))
        fps, _, _, ratio = analyze_frame_timing(batched)
        self.assertGreater(fps, 90)
        self.assertGreater(ratio, 0.4)

        # Stuck sensor & saturation monitor
        mon = RuntimeMonitor()
        mat = np.zeros((300, 5))
        mat[:, 0] = np.arange(300) * 0.01
        mat[:, 2] = 0.0       # frozen actual
        mat[:, 3] = 450.0     # active PWM output!
        alert = mon.check(10.0, mat, 1, 2, True)
        self.assertIn("毫无变化", alert)

        mon.reset()
        mat[:, 2] = np.linspace(0, 60, 300)
        mat[-30:, 2] = 60.0
        mat[:, 3] = 1000.0
        sat_alert = mon.check(20.0, mat, 1, 2, True)
        self.assertIn("满幅饱和", sat_alert)

        params = {g: {"KP": 3.5, "KI": 12.0, "KD": 0.08, "TGT": 100.0} for g in ("ANG", "SPD", "YAW", "AUX")}
        self.assertIn("PID_DEBUG_DEFAULT_KP", format_c_code("SPD", params, 0))
        self.assertIn("BALANCE_KP", format_c_code("ANG", params, 1))
        self.assertIn("pid_loops[PID_YAW]", format_c_code("YAW", params, 2))

    def test_v17_eight_channels_atomic_set_and_autotune_guards(self):
        from storage import csv_header, csv_row
        p = Lines()
        ev8 = list(p.feed(b"1,2,3,4,5,6,7,8\n"))
        self.assertEqual(ev8, [("data", [1., 2., 3., 4., 5., 6., 7., 8.])])
        self.assertEqual([x[0] for x in p.feed(b"1,2,3,4,5,6,7,8,9\n")], ["bad"])

        # CSV roundtrip with 6 channels & backward-compat 21 columns for 4 channels
        with TemporaryDirectory() as tmp:
            p4 = Path(tmp) / "four.csv"
            with open(p4, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(csv_header(["C1", "C2", "C3", "C4"]))
                w.writerow(csv_row(0.0, [1, 2, 3, 4], ("",) * 16, ch_cols=4))
            d4 = load_csv(p4)
            self.assertEqual(d4["count"], 4)

            p6 = Path(tmp) / "six.csv"
            with open(p6, "w", newline="", encoding="utf-8-sig") as f:
                w = csv.writer(f)
                w.writerow(csv_header(["A", "B", "C", "D", "E", "F"], ["Ref0"]))
                w.writerow(csv_row(0.05, [10, 20, 30, 40, 50, 60], ("",) * 16, [99.0], ch_cols=6))
            d6 = load_csv(p6)
            self.assertEqual(d6["count"], 6)
            self.assertEqual(d6["aliases"][:6], ["A", "B", "C", "D", "E", "F"])
            np.testing.assert_allclose(d6["data"][0, 1:7], [10, 20, 30, 40, 50, 60])
            self.assertAlmostEqual(float(d6["data"][0, REF_BASE + 1]), 99.0)

        # Atomic @SET command in Simulator and match_ack
        sim = Simulator(0)
        ack = sim.command("@ANG.SET=1.5,2.5,0.25,12")
        self.assertEqual(ack, "@ACK:ANG.SET=1.5,2.5,0.25,12")
        self.assertEqual(sim.params["ANG"], {"KP": 1.5, "KI": 2.5, "KD": 0.25, "TGT": 12.0})
        self.assertTrue(match_ack("@ANG.SET=1.5,2.5,0.25,12", "ANG.SET=1.5000,2.5000,0.2500,12.0000"))
        bad_ack = sim.command("@ANG.SET=1,2,3")
        self.assertTrue(bad_ack.startswith("@ERR:"))
        self.assertEqual(sim.params["ANG"]["TGT"], 12.0)

        # Pendulum actuator lag + noise + nudge & fall
        pend = Simulator(3, noise=True, seed=42)
        pend.command("@ANG.TGT=0")
        pend.command("@RUN")
        for k in range(250):
            if k % 20 == 0:
                pend.ping()
            _, ang, _, _ = pend.step("ANG")
        self.assertTrue(pend.running)
        self.assertLess(abs(ang), 0.7)
        pend.command("@NUDGE")
        for k in range(500):
            if k % 20 == 0:
                pend.ping()
            _, ang_after, _, _ = pend.step("ANG")
        self.assertTrue(pend.running)
        self.assertLess(abs(ang_after), 0.7)
        pend.nudge("ANG", 400.0)
        for k in range(200):
            if k % 20 == 0:
                pend.ping()
            pend.step("ANG")
            if not pend.running:
                break
        self.assertFalse(pend.running)
        self.assertEqual(pend.drain(), ["#STOP:fall"])

        # AutoTuner strategy=1 safety validation & per-round cap
        rules = {"KP": (0., 100., 5.), "KI": (0., 100., 5.), "KD": (0., 20., 1.)}
        with self.assertRaises(ValueError):
            AutoTuner(0, 1, 1, {"KP": 0., "KI": 0., "KD": 0.}, rules, 0., 10., 1.5, 6)
        capped = AutoTuner(0, 1, 1, {"KP": 4., "KI": 2., "KD": 0.2}, rules, 0., 10., 1.5, 6)
        safe = capped._cap_step_change({"KP": 100., "KI": 0.01, "KD": 10.}, capped.initial)
        self.assertLessEqual(safe["KP"], 4. * 1.35 + 1e-9)
        self.assertGreaterEqual(safe["KI"], 2. / 1.35 - 1e-9)

    def test_thread_heartbeat_needs_a_live_ui(self):
        link = SerialLink({"url":"loop://","baudrate":115200})
        link.start()
        try:
            link.keepalive(True)
            received = bytearray()
            deadline = time.monotonic()+1
            while time.monotonic()<deadline and received.count(b"@PING\n")<2:
                link.keepalive(True)
                for kind,_,data in link.take()[0]:
                    self.assertNotEqual(kind,"tx")
                    if kind == "rx":
                        received.extend(data)
                time.sleep(.01)
            self.assertGreaterEqual(received.count(b"@PING\n"),2)
            with link.lock:
                link.alive_at = time.monotonic()-2   # UI frozen for 2 s
            time.sleep(.1)
            link.take()
            sent = link.pings
            time.sleep(.5)
            self.assertEqual(link.pings,sent)
        finally:
            link.closed.set()
            link.join(1)


class GuiChecks(unittest.TestCase):
    def tearDown(self):
        C.QCoreApplication.sendPostedEvents(None,C.QEvent.Type.DeferredDelete)

    @classmethod
    def setUpClass(cls):
        cls.app = W.QApplication.instance() or W.QApplication([])

    def test_engineering_units_dual_axis_and_target_ack(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.set_reference(0,"目标转速",25)
            w.sample(0.,[12345,700])
            w.configure_channels([.01,1,1,1],["RPM","PWM","",""],1,["SPD","","",""])
            self.assertEqual(w.actual_ch.currentData(),0)
            self.assertEqual(w.target_ch.currentData(),REF_BASE)
            self.assertFalse(w.scope.references[0].isVisible())
            self.assertAlmostEqual(w.buffer.view()[0,1],123.45)
            w.commands.pending = ("@SPD.TGT=150","SPD",None)
            w.receive(b"@ACK:ANG.TGT=150\n",time.monotonic())
            self.assertTrue(np.isnan(w.reference_value(0)))
            w.receive(b"@ACK:SPD.TGT=150\n",time.monotonic())
            self.assertEqual(w.reference_value(0),150)
            w.sample(.1,[15000,850])
            w.paint_scope()
            self.assertEqual(w.scope.curves[1].getViewBox(),w.scope.secondary)
            self.assertLess(w.scope.viewRange()[1][1],200)
            self.assertGreater(w.scope.secondary.viewRange()[1][1],850)
            self.assertIn("RPM",w.names()[0])
            self.assertEqual(w.buffer.view()[-1,REF_BASE+1],150)
            w.select_channel(1)
            w.actual_ch.setCurrentIndex(w.actual_ch.findData(1))
            w.refresh_channels()
            self.assertEqual(w.actual_ch.currentData(),1)
            w.close()

    def test_parameter_confirmation_batch_and_undo(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.source.setCurrentIndex(1)
            w.toggle_connection()
            old = w.sim.params["ANG"].copy()
            w.rules["ANG"]["KP"] = (0,5,.5)
            w.fields["KP"].setValue(8)
            w.send_group()
            self.assertEqual(old,w.sim.params["ANG"])
            w.fields["KP"].setValue(4)
            w.fields["KI"].setValue(2.5)
            with patch.object(W.QMessageBox,"question",return_value=W.QMessageBox.StandardButton.No):
                w.send_group()
            self.assertEqual(old,w.sim.params["ANG"])
            with patch.object(W.QMessageBox,"question",return_value=W.QMessageBox.StandardButton.Yes):
                w.send_group()
                for _ in range(5): w.commands.dispatch()
                self.assertEqual(w.sim.params["ANG"]["KP"],4)
                self.assertEqual(len(w.undo_history),1)
                w.commands.undo_parameter()
                for _ in range(5): w.commands.dispatch()
            self.assertEqual(w.sim.params["ANG"],old)
            self.assertFalse(w.undo_history)
            self.assertEqual(w.fields["KP"].value(),old["KP"])
            w.close()

    def test_interaction_presets_chips_and_heartbeat(self):
        class Link:
            def __init__(self):
                self.heartbeat = None
                self.calls = 0
            def take(self):
                return [],0
            def keepalive(self,enabled):
                self.heartbeat = enabled
                self.calls += 1
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.show()
            self.app.processEvents()
            self.assertTrue(w.pid_panel.isVisible())
            self.assertFalse(w.terminal_panel.isVisible())
            link = w.link = Link()
            w.connected = True
            w.tick()
            self.assertFalse(link.heartbeat)
            w.receive(b"#CAP:PING\n",time.monotonic())
            w.commands.pending = ("@RUN","ANG",None)
            w.receive(b"@ACK:RUN\n100,600\n",time.monotonic())
            w.tick()
            self.assertTrue(link.heartbeat)
            w.tick()
            self.assertEqual(link.calls,3)
            w.workbench.layout(1)
            self.assertTrue(w.scope.frozen)
            w.enabled_ch[1].click()
            self.assertFalse(w.scope.curves[1].isVisible())
            w.workbench.layout(2)
            self.assertTrue(w.terminal_panel.isVisible())
            w.receive(b"#STOP:heartbeat\n",time.monotonic())
            self.assertFalse(w.send_heartbeat)
            self.assertIn("超时",w.stop_banner.text())
            w.tick()
            self.assertFalse(link.heartbeat)
            w.link = None
            w.close()

    def test_generator_in_simulation(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.commands.confirm = lambda changes: self.fail("generator must not ask: "+str(changes))
            w.source.setCurrentIndex(1)
            w.toggle_connection()
            w.fields["TGT"].setValue(10)
            w.gen_shape.setCurrentIndex(1)
            w.rules["ANG"]["TGT"] = (-100.,100.,1.)
            w.gen_amp.setValue(150)
            w.toggle_generator()
            self.assertIsNone(w.generator)
            self.assertIn("超出",w.ack.text())
            w.gen_amp.setValue(5)
            w.gen_period.setValue(1)
            w.toggle_generator()
            self.assertIsNotNone(w.generator)
            self.assertFalse(w.group.isEnabled())
            start = w.generator.start
            seen = []
            for t in (0,.2,.55,.7,1.05):
                w.generator_tick(start+t)
                w.commands.dispatch()
                seen.append(w.sim.params["ANG"]["TGT"])
            self.assertEqual(seen,[15,15,10,10,15])
            self.assertFalse(w.undo_history)
            self.assertEqual(len(w.scope.markers),0)
            w.toggle_generator()
            for _ in range(3): w.commands.dispatch()
            self.assertIsNone(w.generator)
            self.assertEqual(w.sim.params["ANG"]["TGT"],10)
            self.assertTrue(w.group.isEnabled())
            w.toggle_generator()
            w.stop_device()
            self.assertIsNone(w.generator)
            w.close()

    def test_simulated_fall_stops_and_reports(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.source.setCurrentIndex(1)
            w.model.setCurrentIndex(3)
            w.toggle_connection()
            for key in ("KP","KI","KD"):
                w.sim.params["ANG"][key] = 0
            w.run_device()
            self.assertTrue(w.send_heartbeat)
            for _ in range(20):
                w.epoch -= .3
                w.tick()
            self.assertFalse(w.sim.running)
            self.assertIn("倒地",w.stop_banner.text())
            self.assertFalse(w.send_heartbeat)
            w.close()

    def test_auto_reconnect_reads_back_only(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.port.setCurrentText("loop://")
            w.toggle_connection()
            def pump(cond,seconds=2):
                deadline = time.monotonic()+seconds
                while time.monotonic()<deadline and not cond():
                    w.tick()
                    time.sleep(.01)
                return cond()
            self.assertTrue(pump(lambda: w.connected))
            first = w.link
            first.event("error","device unplugged")
            first.closed.set()
            first.join(1)
            self.assertTrue(pump(lambda: w.reconnect_at is not None))
            self.assertFalse(w.connected)
            self.assertEqual(w.connect_btn.text(),"取消重连")
            self.assertIn("等待重连",w.badge.text())
            w.reconnect_at = time.monotonic()
            self.assertTrue(pump(lambda: w.connected))
            self.assertIsNot(w.link,first)
            def sent(text):
                return any(k=="TX" and isinstance(d,(bytes,bytearray)) and text in bytes(d) for _,k,d in w.logs)
            self.assertTrue(pump(lambda: sent(b"GET")))
            self.assertFalse(sent(b"RUN"))
            w.disconnect()
            self.assertTrue(pump(lambda: w.link is None))
            self.assertIsNone(w.reconnect_at)
            w.close()

    def test_autotune_gui_workflow_and_one_click_undo(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"cfg.ini")
            w.timer.stop()
            w.commands.confirm = lambda changes: True
            w.source.setCurrentIndex(1)
            w.model.setCurrentIndex(0)
            w.group.setCurrentIndex(1)  # SPD group supports auto-identification strategy=0
            w.toggle_connection()
            w.fields["KP"].setValue(0.3)
            w.fields["KI"].setValue(0.05)
            w.fields["KD"].setValue(0.0)
            w.fields["TGT"].setValue(10.0)
            w.send_group()
            for _ in range(6):
                w.commands.dispatch()
            self.assertAlmostEqual(w.confirmed["SPD.KP"], 0.3)

            ok = w.autotuner.start(mode=0,style=1,strategy=0,base=0.,target=100.,step_seconds=1.5,max_rounds=6,hold_target=True)
            self.assertTrue(ok)
            self.assertTrue(w.autotuner.active)
            for _ in range(260):
                if not w.autotuner.active and not w.commands.busy():
                    break
                w.epoch -= .25
                w.tick()
            self.assertFalse(w.autotuner.active)
            for _ in range(8):
                w.commands.dispatch()
            self.assertGreater(w.confirmed["SPD.KP"], 1.5)
            self.assertGreater(w.confirmed["SPD.KI"], 3.0)
            self.assertAlmostEqual(w.confirmed["SPD.TGT"], 100.0)
            self.assertIn("评分 J=", w.autotune_status.text())
            self.assertTrue(w.undo_btn.isEnabled())

            # One-click undo restores the exact pre-autotune parameters
            w.commands.undo_parameter()
            for _ in range(8):
                w.commands.dispatch()
            self.assertAlmostEqual(w.confirmed["SPD.KP"], 0.3)
            self.assertAlmostEqual(w.confirmed["SPD.KI"], 0.05)
            self.assertAlmostEqual(w.confirmed["SPD.TGT"], 10.0)

            # Emergency stop aborts an active auto-tune cleanly
            w.autotuner.start(mode=0,style=1,strategy=0,base=0.,target=80.,step_seconds=1.5,max_rounds=6)
            self.assertTrue(w.autotuner.active)
            self.stop_device_check = True
            w.stop_device()
            self.assertFalse(w.autotuner.active)
            w.close()

    def test_ghost_csv_replay_sliders_macros_and_balance_protocol(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "cfg.ini")
            w.timer.stop()
            w.commands.confirm = lambda changes: True
            w.source.setCurrentIndex(1)
            w.model.setCurrentIndex(1)
            w.toggle_connection()
            w.run_device()
            for k in range(30):
                if k % 20 == 0:
                    w.sim.ping()
                w.sample(w.frames * .01, w.sim.step("ANG"))
            w.fields["TGT"].setValue(25.0)
            w.send_field("TGT")
            w.commands.dispatch()
            for k in range(250):
                if k % 20 == 0:
                    w.sim.ping()
                w.sample(w.frames * .01, w.sim.step("ANG"))
            w.paint_scope()
            w.analyze()
            w.pin_step_ghost()
            self.assertIsNotNone(w.scope.ghost)
            self.assertTrue(w.scope.ghost_curve.isVisible())
            self.assertIn("对比底图", w.metrics.text())

            # Second step aligns Ghost to the new step timestamp
            w.fields["KP"].setValue(8.0)
            w.fields["KD"].setValue(1.2)
            w.fields["TGT"].setValue(40.0)
            w.send_group()
            for _ in range(6):
                w.commands.dispatch()
            step2_t = w.latest + .01
            for k in range(200):
                if k % 20 == 0:
                    w.sim.ping()
                w.sample(w.frames * .01, w.sim.step("ANG"))
            w.paint_scope()
            w.analyze()
            self.assertAlmostEqual(float(w.scope.ghost_curve.xData[0]), step2_t, delta=0.02)

            # Export view CSV, clear everything, and reload CSV offline!
            csv_path = Path(tmp) / "replay.csv"
            w.choose_file = lambda *_: str(csv_path)
            w.export_view()
            self.assertTrue(w.load_csv_file(csv_path))
            self.assertTrue(w.scope.frozen)
            self.assertEqual(w.count, 4)
            self.assertAlmostEqual(w.confirmed["ANG.KP"], 8.0)

            # Sliders, step gear, macro send, SPD auto-switch & #BALANCE protocol
            w.source.setCurrentIndex(1)
            w.toggle_connection()
            w.slider_toggle.setChecked(True)
            w.step_gear.setCurrentIndex(1)  # 0.01
            self.assertAlmostEqual(w.fields["KP"].singleStep(), 0.01)
            w.slider_moved("KP", 750)
            self.assertGreater(w.fields["KP"].value(), 8.0)

            w.send_macro(1)  # @RST
            w.commands.dispatch()

            # Simulate User/pid_debug.c replying "@ERR:only SPD supported" on ANG.GET
            w.group.setCurrentIndex(0)
            w.commands.pending = ("@ANG.GET", "ANG", None)
            w.receive(b"@ERR:only SPD supported\n", time.monotonic())
            self.assertEqual(w.selected(), "SPD")

            # Simulate Doc/main_balance_test.c status frames
            w.receive(b"#BALANCE:CALIBRATING,KEEP_STILL\n", time.monotonic())
            self.assertFalse(w.stop_banner.isHidden())
            self.assertIn("静止校准", w.stop_banner.text())
            w.receive(b"#BALANCE:READY\n", time.monotonic())
            self.assertTrue(w.stop_banner.isHidden())
            w.receive(b"#BALANCE:INIT_ERROR=5\n", time.monotonic())
            self.assertIn("晃动", w.stop_banner.text())
            w.receive(b"#BALANCE:TILT_STOP\n", time.monotonic())
            self.assertIn("30°", w.stop_banner.text())
            w.close()

    def test_scope_controls_and_saved_settings(self):
        with TemporaryDirectory() as tmp:
            config = Path(tmp)/"settings.ini"
            old = C.QSettings(str(config),C.QSettings.Format.IniFormat)
            old.setValue("options",'{"window": 2}')
            old.sync()
            w = Window(config)
            w.timer.stop()
            self.assertEqual(w.window.value(),30)
            w.set_reference(0,"目标",2)
            for i in range(10):
                w.sample(float(i),[float(i)])
            w.window.setValue(2.75)
            w.x_div.setValue(.25)
            w.y_div.setValue(20)
            self.assertEqual(w.scope.getAxis("bottom").tickSpacing(0,2.75,900)[0],(.25,0))
            self.assertEqual(w.scope.getAxis("bottom").tickStrings([.25,.75],1,.25),["0.25","0.75"])
            self.assertEqual(w.scope.getAxis("left").tickSpacing(0,200,600)[0],(20,0))
            self.assertLess(len(w.scope.getAxis("bottom").tickValues(0,86400,1000)[0][1]),40)
            w.measure()
            np.testing.assert_allclose(w.scope.viewRange()[0],[6.25,9])
            self.assertIn("峰峰值 2",w.measure_text.text())
            self.assertIn("9",w.scope.legend.getLabel(w.scope.curves[0]).text)
            w.toggle_freeze()
            w.window.setValue(1.5)
            np.testing.assert_allclose(w.scope.viewRange()[0],[7.5,9])
            def select(hex_color,accept):
                dialog = W.QApplication.activeModalWidget()
                dialog.setCurrentColor(QtGui.QColor(hex_color))
                dialog.accept() if accept else dialog.reject()
            C.QTimer.singleShot(0,lambda: select("#e6194b",True))
            w.color_buttons[0].click()
            C.QTimer.singleShot(0,lambda: select("#00ff00",False))
            w.color_buttons[0].click()
            C.QTimer.singleShot(0,lambda: select("#aa3377",True))
            w.color_buttons[MAX_CHANNELS].click()
            w.line.setValue(1.75)
            self.assertEqual(w.scope.curves[0].opts["pen"].color().name(),"#e6194b")
            self.assertEqual(w.scope.references[0].pen.color().name(),"#aa3377")
            self.assertEqual(w.scope.references[0].pen.widthF(),1.75)
            w.close()
            reopened = Window(config)
            self.assertEqual(reopened.window.value(),1.5)
            self.assertEqual(reopened.x_div.value(),.25)
            self.assertEqual(reopened.y_div.value(),20)
            self.assertEqual(reopened.colors[0],"#e6194b")
            self.assertEqual(reopened.colors[MAX_CHANNELS],"#aa3377")
            reopened.close()

    def test_edge_capture_and_cancel(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"settings.ini")
            w.timer.stop()
            w.connected = True
            w.trigger_level.setValue(5)
            w.pre_percent.setValue(50)
            for edge,before,after in ((0,4,6),(1,6,4),(2,6,4)):
                w.clear_plot()
                w.trigger_edge.setCurrentIndex(edge)
                w.sample(0.,[before])
                w.capture.arm_capture()
                w.sample(1.,[after])
                self.assertFalse(w.scope.frozen)
                w.sample(2.,[after])
                self.assertTrue(w.scope.frozen)
                np.testing.assert_allclose(w.scope.viewRange()[0],[0.,2.])
                w.sample(3.,[0.])
                self.assertEqual(w.scope.data[-1,0],2.)
                self.assertEqual(w.buffer.view()[-1,0],3.)
                w.toggle_freeze()
            w.capture.arm_capture()
            w.capture.arm_capture()
            self.assertIsNone(w.capture.capture_channel)
            self.assertEqual(w.tx,0)
            w.close()

    def test_terminal_groups_split_and_batched_readings(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"settings.ini")
            w.timer.stop()
            w.clear_log()
            stamp = time.monotonic()
            def displayed():
                w.render_log()
                return [line.split("  ",1)[1] for line in w.terminal.toPlainText().splitlines()]

            for chunk in (b"1", b"5", b"6"):
                w.receive(chunk,stamp)
            self.assertEqual(displayed(),["接收  156"])
            w.receive(b"\r",stamp)
            w.receive(b"\n122.8\r\n\r\n-3.1\n",stamp)
            self.assertEqual(displayed(),["接收  156","接收  122.8","接收  -3.1"])
            self.assertEqual(w.frames,3)
            w.hex_mode.setChecked(True)
            self.assertEqual(displayed()[0],"接收  31 35 36 0D 0A")
            w.hex_mode.setChecked(False)
            w.receive(b"old",stamp)
            w.clear_log()
            w.receive(b"7\r8\n",stamp)
            entries = displayed()
            self.assertEqual(entries[:2],["接收  7","接收  8"])
            self.assertIn("错误帧 1",entries[2])
            w.close()

    def test_session(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"settings.ini")
            w.timer.stop()
            w.source.setCurrentIndex(1)
            w.toggle_connection()
            self.assertIn("ANG.KP",w.confirmed)
            for _ in range(5):
                w.sample(w.frames*.01,w.sim.step("ANG"))
            w.run_device()
            for _ in range(500):
                w.sample(w.frames*.01,w.sim.step("ANG"))
            w.fields["KP"].setValue(2.75)
            w.send_field("KP")
            self.assertEqual(w.confirmed["ANG.KP"],2.75)
            w.toggle_freeze()
            w.analyze()
            self.assertIn("超调",w.metrics.text())
            old = w.scope.data.copy()
            w.sample(5.05,w.sim.step("ANG"))
            np.testing.assert_array_equal(old,w.scope.data)
            w.choose_file = lambda title,suffix: str(Path(tmp)/("data."+suffix))
            w.export_view()
            with open(Path(tmp)/"data.csv",encoding="utf-8-sig") as f:
                csvdata = list(csv.reader(f))
                self.assertEqual(len(csvdata[0]),21)
                self.assertEqual(len(csvdata)-1,len(old))
            w.record()
            w.sample(5.06,w.sim.step("ANG"))
            w.end_record()
            with open(Path(tmp)/"data.csv",encoding="utf-8-sig") as f:
                row = list(csv.reader(f))[1]
                self.assertEqual(float(row[5]),2.75)
            w.png()
            self.assertTrue((Path(tmp)/"data.png").read_bytes().startswith(b"\x89PNG"))
            w.stop_device()
            self.assertFalse(w.sim.running)
            w.disconnect()
            w.close()
            other = Window(Path(tmp)/"settings.ini")
            self.assertEqual(other.fields["KP"].value(),2.75)
            other.close()

    def test_detected_channels_and_local_reference(self):
        with TemporaryDirectory() as tmp:
            config = Path(tmp)/"settings.ini"
            w = Window(config)
            w.timer.stop()
            self.assertEqual(w.target_ch.count(),0)
            w.set_reference(0,"目标转速",150)
            w.sample(0.,[145])
            w.name_edits[0].setText("实际转速")
            w.channel_names()
            self.assertEqual(w.actual_ch.currentData(),0)
            self.assertEqual(w.target_ch.currentData(),REF_BASE)
            self.assertTrue(w.channel_form.isRowVisible(w.name_edits[0]))
            self.assertFalse(w.channel_form.isRowVisible(w.name_edits[1]))
            w.receive(b"#NAMES:rpm,extra\n",time.monotonic())
            self.assertEqual(w.names()[0],"实际转速")
            w.sample(.1,[146,8])
            self.assertTrue(w.channel_form.isRowVisible(w.name_edits[1]))
            w.toggle_freeze()
            w.sample(.2,[147])
            self.assertEqual(w.actual_ch.count(),3)
            w.toggle_freeze()
            self.assertEqual(w.actual_ch.count(),2)
            self.assertFalse(w.channel_form.isRowVisible(w.name_edits[1]))
            w.ref_rows[0][2].setValue(160)
            w.sample(.3,[148])
            w.sample(.4,[155])
            w.paint_scope()
            w.analyze()
            self.assertIn("超调",w.metrics.text())
            self.assertEqual(w.tx,0)
            self.assertEqual(w.bad,0)
            self.assertEqual(w.scope.references[0].value(),160)
            self.assertEqual(w.scope.legend_channels,((0,"实际转速"),))
            w.choose_file = lambda *_: str(Path(tmp)/"test.csv")
            w.export_view()
            with open(Path(tmp)/"test.csv",encoding="utf-8-sig") as f:
                rows = list(csv.DictReader(f))
            self.assertEqual([float(r["REF_目标转速"]) for r in rows],[150,150,150,160,160])
            self.assertEqual(rows[2]["CH2_extra"],"")
            w.close()
            w = Window(config)
            self.assertEqual(w.names()[0],"实际转速")
            self.assertTrue(w.ref_active[0])
            self.assertEqual(w.scope.references[0].value(),160)
            w.remove_reference(0)
            self.assertFalse(w.scope.references[0].isVisible())
            w.close()

    def test_wrong_ack_and_timeout(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp)/"settings.ini")
            w.timer.stop()
            w.commands.pending = ("@ANG.KP=1","ANG",time.monotonic()-1)
            w.commands.waiting.append(("@RUN","ANG",False))
            w.receive(b"@ACK:SPD.KP=1\n",time.monotonic())
            self.assertIsNotNone(w.commands.pending)
            w.tick()
            self.assertIsNone(w.commands.pending)
            self.assertFalse(w.commands.waiting)
            self.assertFalse(w.confirmed)
            self.assertIn("超时",w.ack.text())
            w.close()

    def test_serial_checksums_and_loopback(self):
        self.assertEqual(crc16_modbus(b"\x01\x03\x00\x00\x00\x0A"), b"\xC5\xCD")
        self.assertEqual(append_checksum(b"\x10\x20\x30", 1), b"\x10\x20\x30\x60")
        self.assertEqual(append_checksum(b"\xAA\x55\x0F", 2), b"\xAA\x55\x0F\xF0")
        self.assertEqual(append_checksum(b"\x01\x03\x00\x00\x00\x0A", 3), b"\x01\x03\x00\x00\x00\x0A\xC5\xCD")

        with TemporaryDirectory() as tmp:
            cfg = Path(tmp) / "settings.ini"
            w = Window(cfg)
            w.timer.stop()
            sp = w.serial_page
            sp.poll_timer.stop()
            sp.port_box.setCurrentText("loop://")
            sp.toggle_port()
            deadline = time.monotonic() + 1.5
            while not sp.connected and time.monotonic() < deadline:
                sp.tick()
                time.sleep(0.01)
            self.assertTrue(sp.connected)
            self.assertEqual(sp.active_port(), "loop://")

            # GBK Chinese loopback test
            sp.encoding_box.setCurrentIndex(1)  # GBK
            sp.tx_edit.setPlainText("平衡车OK")
            sp.send_from_editor()
            deadline = time.monotonic() + 1.0
            while sp.rx_bytes < len("平衡车OK\n".encode("gbk")) and time.monotonic() < deadline:
                sp.tick()
                time.sleep(0.01)
            sp.render_logs(force=True)
            self.assertIn("平衡车OK", sp.log_view.toPlainText())

            # Keyword filter test
            sp.filter_edit.setText("不存在关键字")
            self.assertEqual(sp.log_view.toPlainText(), "")
            sp.filter_edit.clear()
            self.assertIn("平衡车OK", sp.log_view.toPlainText())

            # HEX + CRC16 quick slot test
            sp.checksum_box.setCurrentIndex(3)
            sp.slot_widgets[10][1].setText("01 03 00 00 00 0A")
            sp.send_quick_slot(10)
            prev_rx = sp.rx_bytes
            deadline = time.monotonic() + 1.0
            while sp.rx_bytes < prev_rx + 8 and time.monotonic() < deadline:
                sp.tick()
                time.sleep(0.01)
            sp.rx_hex.setChecked(True)
            sp.render_logs(force=True)
            self.assertIn("C5 CD", sp.log_view.toPlainText())

            w.switch_module(1)
            sp.release_port()
            w.close()

            w2 = Window(cfg)
            w2.timer.stop()
            self.assertEqual(w2.module_stack.currentIndex(), 1)
            self.assertEqual(w2.serial_page.encoding_box.currentIndex(), 1)
            self.assertEqual(w2.serial_page.quick_slots[10][1], "01 03 00 00 00 0A")
            w2.close()

    def test_imu_studio_fusion_and_calibration(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "settings.ini")
            w.timer.stop()
            imu = w.imu_page
            imu.timer.stop()
            imu.connected = True

            # 1) Balance-car 4-col & #BALANCE status lines
            t0 = time.monotonic()
            imu._feed_bytes(b"#BALANCE:CALIBRATING,KEEP_STILL\n", t0)
            self.assertFalse(imu.banner.isHidden())
            imu._feed_bytes(b"#BALANCE:READY\n", t0 + 0.01)
            self.assertTrue(imu.banner.isHidden())

            imu.start_zero_capture()
            for i in range(50):
                imu._feed_bytes(b"0.00,1.85,-0.4,12\n", t0 + 0.02 + i * 0.01)
            self.assertAlmostEqual(imu.target_pitch, 1.85, places=3)
            self.assertTrue(imu.sync_pid_btn.isEnabled())
            imu.sync_zero_to_pid()
            self.assertAlmostEqual(w.drafts["ANG"]["TGT"], 1.85, places=3)
            self.assertAlmostEqual(w.fields["TGT"].value(), 1.85, places=3)

            # 2) Physical 6-axis split-chunk assembly, then raw MPU6050 LSB fusion (+30 deg pitch)
            imu.proto_box.setCurrentIndex(2)
            imu.fused_init = False
            imu._feed_bytes(b"#BALANCE:CALIBRATING,KEEP_STILL\n", t0 + 0.8)
            self.assertFalse(imu.banner.isHidden())
            imu._feed_bytes(b"#NAMES:ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps\n#BALANCE:READY\n", t0 + 0.82)
            self.assertTrue(imu.banner.isHidden())
            frames_before = imu.frames
            for chunk in (b"0.0100,-0.0", b"870,0.9960,0.2", b"0,-0.10,0.05\n"):
                imu._feed_bytes(chunk, t0 + 0.85)
            self.assertEqual(imu.frames, frames_before + 1)
            self.assertAlmostEqual(imu.acc_norm, 1.0, delta=0.02)

            imu.proto_box.setCurrentIndex(3)
            imu.fused_init = False
            imu.ingest_values([0.0, 8192.0, 14189.0, 0.0, 0.0, 1310.0], 1.0, 0.01)
            imu.ingest_values([0.0, 8192.0, 14189.0, 0.0, 0.0, 1310.0], 1.1, 0.1)
            self.assertAlmostEqual(imu.pitch, 30.0, delta=0.2)
            self.assertAlmostEqual(imu.yaw, 1.0, delta=0.05)

            # 3) Render 3D Balance Car & Horizon in both light and dark modes
            imu.canvas.resize(720, 420)
            imu.canvas.set_attitude(imu.pitch, 8.5, imu.yaw, 0.0, 220.0)
            self.assertFalse(imu.canvas.grab().isNull())
            w.dark.setChecked(True)
            self.assertTrue(imu.canvas.dark)
            self.assertFalse(imu.canvas.grab().isNull())
            w.close()

    def test_multi_module_navigation_and_port_handoff(self):
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "settings.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            w.remote_page.timer.stop()

            # Dynamic 6th module plug-in test (e.g. future Camera module)
            class DummyCameraPage(W.QWidget):
                module_id = "camera"
                module_title = "📷 摄像头调试"
                def active_port(self): return None
                def release_port(self): pass
                def apply_theme(self, dark): pass
                def save_state(self, s): pass
                def restore_state(self, s): pass
                def shutdown(self): pass

            cam = DummyCameraPage(w)
            w.register_module(cam.module_title, cam)
            self.assertEqual(w.module_stack.count(), 6)
            w.switch_module(5)
            self.assertIs(w.module_stack.currentWidget(), cam)

            # Open loop:// on SerialAssistantPage
            sp = w.serial_page
            sp.port_box.setCurrentText("loop://")
            sp.toggle_port()
            deadline = time.monotonic() + 1.5
            while not sp.connected and time.monotonic() < deadline:
                sp.tick()
                time.sleep(0.01)
            self.assertEqual(sp.active_port(), "loop://")
            self.assertIn("●", w.module_tabs[1][1].text())

            # Decline handoff -> SerialAssistantPage retains port
            imu = w.imu_page
            imu.port_box.setCurrentText("loop://")
            w.confirm_handoff = lambda *_: False
            imu.toggle_source()
            self.assertEqual(sp.active_port(), "loop://")
            self.assertIsNone(imu.active_port())

            # Accept handoff -> SerialAssistantPage releases port to ImuStudioPage
            w.confirm_handoff = lambda *_: True
            imu.toggle_source()
            deadline = time.monotonic() + 1.5
            while not imu.connected and time.monotonic() < deadline:
                imu.tick()
                time.sleep(0.01)
            self.assertIsNone(sp.active_port())
            self.assertEqual(imu.active_port(), "loop://")

            # Accept handoff -> ImuStudioPage releases port to PID Tuner Page
            w.port.setCurrentText("loop://")
            w.toggle_connection()
            deadline = time.monotonic() + 1.5
            while not w.connected and time.monotonic() < deadline:
                w.tick()
                time.sleep(0.01)
            self.assertIsNone(imu.active_port())
            self.assertEqual(w.active_port(), "loop://")
            w.release_port()
            w.close()

    def test_blueserial_remote_and_hc05_at_toolbox(self):
        # 1) BlueSerialStudio protocol & AT command parity checks
        self.assertEqual(AtCommandBuilder.with_crlf(AtCommandBuilder.set_name("BlueCar")), "AT+NAME=BlueCar\r\n")
        self.assertEqual(AtCommandBuilder.set_uart(5760, 0, 0), "AT+UART=5760,0,0")
        self.assertEqual(AtCommandBuilder.set_uart(115200, 1, 2), "AT+UART=115200,1,2")
        self.assertEqual(AtCommandBuilder.set_pin("1234"), "AT+PSWD=1234")
        self.assertEqual(AtCommandBuilder.set_role(0), "AT+ROLE=0")
        with self.assertRaises(ValueError):
            AtCommandBuilder.set_pin("12,34")
        self.assertEqual(AtCommandBuilder.set_uart(57600, 0, 0), "AT+UART=57600,0,0")
        with self.assertRaises(ValueError):
            AtCommandBuilder.set_uart(12345, 0, 0)

        self.assertEqual(held_speed(0), 18)
        self.assertEqual(held_speed(1600), 100)
        self.assertEqual(button_axis(True, 0, True, 100, 500), 0)
        self.assertEqual(mix_wheels(0, 80), (80, 80))
        self.assertEqual(mix_wheels(50, 80), (80, 40))
        self.assertEqual(mix_wheels(60, 0), (60, -60))
        self.assertEqual(format_drive_command("J,{x},{y}", -35, 90), "J,-35,90")

        # 2) Loopback TX/RX & auto-stop on tab switch
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "settings.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            rp = w.remote_page
            rp.timer.stop()
            w.switch_module(3)
            rp.port_box.setCurrentText("loop://")
            rp.toggle_port()
            deadline = time.monotonic() + 1.5
            while not rp.connected and time.monotonic() < deadline:
                rp.tick()
                time.sleep(0.01)
            self.assertTrue(rp.connected)

            rp.exec_at("AT+VERSION?")
            rp._on_single_joy(40, 80, False)
            deadline = time.monotonic() + 1.0
            while rp.rx_bytes < len(b"AT+VERSION?\r\nJ,40,80\n") and time.monotonic() < deadline:
                rp.tick()
                time.sleep(0.01)
            self.assertIn("AT+VERSION?", rp.console.toPlainText())
            self.assertIn("J,40,80", rp.console.toPlainText())

            # Switching away from remote tab automatically zeroes joystick & sends J,0,0
            w.switch_module(0)
            self.assertEqual((rp.steering_x, rp.throttle_y), (0, 0))
            deadline = time.monotonic() + 1.0
            while "J,0,0" not in rp.console.toPlainText() and time.monotonic() < deadline:
                rp.tick()
                time.sleep(0.01)
            self.assertIn("J,0,0", rp.console.toPlainText())
            rp.release_port()
            w.close()

    def test_v17_safety_rollback_remote_focus_keepalive_and_hc05_uart(self):
        # 1) ANG forces local fine-tuning, and abort without winner restores original params to device
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "cfg.ini")
            w.timer.stop()
            w.source.setCurrentIndex(1)
            w.model.setCurrentIndex(0)
            w.toggle_connection()

            # ANG + strategy=0 is rejected
            w.group.setCurrentIndex(0)
            self.assertFalse(w.autotuner.start(mode=1, style=1, strategy=0, base=0., target=5., step_seconds=1.5, max_rounds=4))

            # Switch to SPD with custom pre-autotune values and abort before winner
            w.group.setCurrentIndex(1)
            for k, v in (("KP", 0.8), ("KI", 1.2), ("KD", 0.05), ("TGT", 30.0)):
                w.fields[k].setValue(v)
            w.send_group()
            for _ in range(6):
                w.commands.dispatch()
            self.assertTrue(w.autotuner.start(mode=0, style=1, strategy=0, base=0., target=60., step_seconds=1.5, max_rounds=6))
            w.epoch -= 0.1
            w.tick()  # dispatches @SPD.TGT=0, @RUN and enters PREPARE probe with changed KP
            for _ in range(8):
                w.commands.dispatch()
            # Timeout / cancel without winner must roll back simulator params too!
            w.autotuner.stop("模拟采样超时", apply_best=False)
            for _ in range(8):
                w.commands.dispatch()
            self.assertAlmostEqual(w.sim.params["SPD"]["KP"], 0.8)
            self.assertAlmostEqual(w.sim.params["SPD"]["KI"], 1.2)
            self.assertAlmostEqual(w.sim.params["SPD"]["KD"], 0.05)
            self.assertAlmostEqual(w.sim.params["SPD"]["TGT"], 30.0)
            w.close()

        # 2) 8-channel GUI display & release_port @STOP flush over loop://
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "cfg8.ini")
            w.timer.stop()
            w.receive(b"1,2,3,4,5,6,7,8\n", time.monotonic())
            self.assertEqual(w.count, MAX_CHANNELS)
            self.assertTrue(w.channel_form.isRowVisible(w.name_edits[7]))
            w.port.setCurrentText("loop://")
            w.toggle_connection()
            deadline = time.monotonic() + 1.5
            while not w.connected and time.monotonic() < deadline:
                w.tick()
                time.sleep(0.01)
            self.assertTrue(w.connected)
            w.release_port()
            self.assertFalse(w.connected)
            self.assertTrue(any(k == "TX" and b"@STOP" in bytes(d) for _, k, d in w.logs if isinstance(d, (bytes, bytearray))))
            w.close()

        # 3) RemoteBtPage focus-loss anti-stuck stop, 100ms keepalive resend, and HC-05 AT+UART? safety
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "remote.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            rp = w.remote_page
            rp.timer.stop()
            w.switch_module(3)
            rp.port_box.setCurrentText("loop://")
            rp.toggle_port()
            deadline = time.monotonic() + 1.5
            while not rp.connected and time.monotonic() < deadline:
                rp.tick()
                time.sleep(0.01)
            self.assertTrue(rp.connected)

            # Held keyboard motion clears on WindowDeactivate and transmits J,0,0
            rp.mode_box.setCurrentIndex(2)
            rp._dir_pressed("up")
            self.assertGreater(rp.throttle_y, 0)
            W.QApplication.sendEvent(w, C.QEvent(C.QEvent.Type.WindowDeactivate))
            self.assertEqual(rp.held_keys, {})
            self.assertEqual((rp.steering_x, rp.throttle_y), (0, 0))

            # Non-zero joystick keeps alive every 100 ms in tick()
            rp.rx_bytes = 0
            rp._on_single_joy(35, 65, False)
            rp.last_drive_tx = time.monotonic() - 0.15
            rp.tick()
            deadline = time.monotonic() + 1.0
            while rp.rx_bytes < len(b"J,35,65\n") * 2 and time.monotonic() < deadline:
                rp.tick()
                time.sleep(0.01)
            self.assertGreaterEqual(rp.console.toPlainText().count("J,35,65"), 2)

            # HC-05 5760 warning + 57600 support + AT+UART? read-before-write compare
            self.assertIn("57600", [rp.at_baud_box.itemText(i) for i in range(rp.at_baud_box.count())])
            prompts = []
            rp.confirm_at = lambda title, text: prompts.append((title, text)) or True
            idx_5760 = rp.at_baud_box.findData(5760)
            self.assertGreaterEqual(idx_5760, 0)
            self.assertIn("疑似笔误", rp.at_baud_box.itemText(idx_5760))
            rp.at_baud_box.setCurrentIndex(idx_5760)
            rp._apply_at_uart(copy_only=False)
            self.assertEqual(len(prompts), 1)
            # Simulate HC-05 replying different current UART so write confirmation fires
            deadline = time.monotonic() + 1.0
            while "+UART?" not in rp.console.toPlainText() and time.monotonic() < deadline:
                rp.tick()
                time.sleep(0.01)
            rp.link.event("rx", b"+UART:9600,0,0\r\nOK\r\n")
            rp.tick()
            self.assertEqual(len(prompts), 2)
            self.assertIn("9600,0,0", prompts[1][1])
            rp.release_port()
            w.close()

    def test_v171_pid_debug_zero_start_channel_detection_and_target_guard(self):
        # 1) Reproduce User/pid_debug.c: count==0 before @RUN, then #NAMES:target,actual,output,error + 4-col data
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "cfg.ini")
            w.timer.stop()
            w.connected = True
            w.sim = None
            w.group.setCurrentIndex(1)  # SPD
            w.fields["KP"].setValue(5.0)
            w.fields["KI"].setValue(30.0)
            w.fields["KD"].setValue(0.05)
            w.fields["TGT"].setValue(100.0)
            w.confirmed.update({"SPD.KP": 5.0, "SPD.KI": 30.0, "SPD.KD": 0.05, "SPD.TGT": 100.0})

            w.commands.submit = lambda cmds, quiet=False: True
            w.commands.busy = lambda: False

            ok = w.autotuner.start(mode=0, style=1, strategy=0, base=0.0, target=100.0, step_seconds=1.5, max_rounds=6)
            self.assertTrue(ok)
            # Must NOT prematurely create reference line when count == 0
            self.assertFalse(w.ref_active[0])

            # Simulated first frame arriving after @RUN (with STM32 #NAMES:target,actual,output,error)
            t0 = time.monotonic()
            w.receive(b"#NAMES:target,actual,output,error\n", t0)
            w.receive(b"0.0,0.0,0.0,0.0\n", t0 + 0.01)
            w.autotuner.tick(t0 + 0.02)

            self.assertFalse(w.ref_active[0])
            self.assertEqual(w.actual_ch.currentData(), 1)
            self.assertEqual(w.autotuner.actual_col, 1)
            self.assertEqual(w.autotuner.output_col, 2)

            # 2) Even if a manual reference line exists, 4-ch named stream still picks CH2 (actual=1)
            w.set_reference(0, "参考", 50.0)
            w.channel_preferences[1] = None
            w.refresh_channels()
            self.assertEqual(w.actual_ch.currentData(), 1)
            w.close()

        # 3) Anti-target-channel guard aborts and restores original params if sampled actual tracks target at sample 0
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "guard.ini")
            w.timer.stop()
            w.connected = True
            w.sim = None
            w.group.setCurrentIndex(1)
            w.fields["KP"].setValue(5.0)
            w.fields["KI"].setValue(30.0)
            w.fields["KD"].setValue(0.05)
            w.fields["TGT"].setValue(100.0)
            w.confirmed.update({"SPD.KP": 5.0, "SPD.KI": 30.0, "SPD.KD": 0.05, "SPD.TGT": 100.0})
            sent = []
            def fake_submit(cmds, quiet=False):
                sent.extend(cmds)
                return True
            w.commands.submit = fake_submit
            w.commands.busy = lambda: False

            w.receive(b"100.0,15.0,69,85.0\n", time.monotonic())
            w.channel_preferences[1] = 0
            w.refresh_channels()
            self.assertEqual(w.actual_ch.currentData(), 0)

            self.assertTrue(w.autotuner.start(mode=0, style=1, strategy=0, base=0.0, target=100.0, step_seconds=0.5, max_rounds=6))
            w.autotuner.tick(time.monotonic())  # WAIT_START -> PREPARE
            w.autotuner.tick(time.monotonic())  # PREPARE -> WAIT_BASE_ACK
            w.autotuner.tick(time.monotonic())  # WAIT_BASE_ACK -> SETTLE_BASE
            for k in range(12):
                w.sample(w.latest + 0.05, [0.0, 0.0, 0.0, 0.0])
            w.autotuner.tick(time.monotonic())  # SETTLE_BASE -> WAIT_STEP_ACK
            w.autotuner.tick(time.monotonic())  # WAIT_STEP_ACK -> OBSERVE
            sent.clear()
            for k in range(12):
                # Mis-selected CH1 jumps instantaneously to target=100 from the very first sample
                w.sample(w.latest + 0.05, [100.0, 15.0, 69.0, 85.0])
            w.autotuner.tick(time.monotonic())

            self.assertFalse(w.autotuner.active)
            self.assertIn("误选目标通道", w.autotune_status.text())
            self.assertTrue(any("SPD.KP=5.0000" in c for c in sent))
            self.assertTrue(any("SPD.KI=30.0000" in c for c in sent))
            w.close()

    def test_v18_roles_order_free_and_firmware_interface_builder(self):
        from firmware_studio import FirmwareStudioPage, build_interface_guide_html, generate_interface_bundle

        # 1) Lines parses valid #ROLES and rejects invalid tokens or >8 items
        parser = Lines()
        ev = list(parser.feed(b"#ROLES:output,error,actual,target\n"))
        self.assertEqual(ev, [("roles", ["output", "error", "actual", "target"])])
        self.assertEqual([x[0] for x in parser.feed(b"#ROLES:bad_role,actual\n")], ["bad"])
        self.assertEqual([x[0] for x in parser.feed(b"#ROLES:" + b",".join([b"other"] * 9) + b"\n")], ["bad"])

        # 2) Reordered stream (PWM at CH1, Error at CH2, Actual at CH3, Target at CH4) binds automatically
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "roles.ini")
            w.timer.stop()
            w.connected = True
            w.sim = None
            w.receive(b"#NAMES:pwm_out,err,my_rpm,my_set\n", time.monotonic())
            w.receive(b"#ROLES:output,error,actual,target\n", time.monotonic())
            w.receive(b"650.0,84.5,15.5,100.0\n", time.monotonic())
            self.assertEqual(w.target_ch.currentData(), 3)
            self.assertEqual(w.actual_ch.currentData(), 2)
            self.assertEqual(w.autotuner.detect_channels(), (2, 0))
            w.close()

        # 3) All 3 interfaces x 3 platforms (HAL / SPL stm32f10x / ESP32·Arduino) code generation & guides
        pid_cfg = {
            "interface_type": 0,
            "platform": 0,
            "uart_handle": "huart1",
            "baudrate": 115200,
            "primary_loop": "SPD",
            "control_ms": 10,
            "report_ms": 20,
            "pwm_limit": 1000.0,
            "defaults": {"KP": 5.0, "KI": 30.0, "KD": 0.05, "TGT": 100.0},
            "emit_roles": True,
            "channels": [("pwm", "output"), ("err", "error"), ("vel", "actual"), ("sp", "target")],
        }
        hal_bundle = generate_interface_bundle(pid_cfg)
        self.assertEqual(set(hal_bundle), {"main_integration_example.c", "pid_port_hal.h", "pid_port_hal.c", "pid_link.h", "pid_link.c", "README_接入说明.md"})
        self.assertIn("#NAMES:pwm,err,vel,sp", hal_bundle["pid_port_hal.c"])
        self.assertIn("#ROLES:output,error,actual,target", hal_bundle["pid_port_hal.c"])
        self.assertIn("PID_Port_Init(&huart1)", hal_bundle["main_integration_example.c"])
        self.assertIn("PID_SetDefaultGroup(PRIMARY_PID_GROUP)", hal_bundle["pid_port_hal.c"])

        spl_pid = generate_interface_bundle({**pid_cfg, "platform": 1, "uart_handle": "USART1"})
        self.assertEqual(set(spl_pid), {"main_integration_spl.c", "pid_port_spl.h", "pid_port_spl.c", "pid_link.h", "pid_link.c", "README_接入说明.md"})
        self.assertIn('#include "stm32f10x.h"', spl_pid["pid_port_spl.h"])
        self.assertIn("USART_IT_RXNE", spl_pid["pid_port_spl.c"])
        self.assertIn("GPIO_Mode_AF_PP", spl_pid["main_integration_spl.c"])
        self.assertIn("PID_SetDefaultGroup(PRIMARY_PID_GROUP)", spl_pid["pid_port_spl.c"])
        self.assertIn("USART1_IRQHandler", spl_pid["main_integration_spl.c"])

        ino_pid = generate_interface_bundle({**pid_cfg, "platform": 2})
        self.assertIn("EmbeddedDebugStudio_PID.ino", ino_pid)
        self.assertIn("#ROLES:output,error,actual,target", ino_pid["EmbeddedDebugStudio_PID.ino"])
        self.assertIn("parseStrictFloat", ino_pid["EmbeddedDebugStudio_PID.ino"])
        self.assertNotIn("Serial.printf", ino_pid["EmbeddedDebugStudio_PID.ino"])

        # IMU across 4 formats & 3 platforms (including canonical 5-col balance-car format)
        for fmt_id, expected_hdr in (
            (0, "#NAMES:ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps"),
            (1, "#NAMES:ax_raw,ay_raw,az_raw,gx_raw,gy_raw,gz_raw"),
            (2, "#NAMES:pitch_target,pitch,gyro_pitch,pwm,wheel_speed"),
            (3, "#NAMES:pitch,roll,yaw"),
        ):
            imu_hal = generate_interface_bundle({"interface_type": 1, "platform": 0, "imu_format": fmt_id, "imu_report_ms": 20})
            self.assertIn(expected_hdr, imu_hal["imu_port_hal.c"])
            imu_hal_ts = generate_interface_bundle({"interface_type": 1, "platform": 0, "imu_format": fmt_id, "imu_report_ms": 20, "imu_include_ts": True})
            self.assertIn(expected_hdr + ",seq,mcu_ms", imu_hal_ts["imu_port_hal.c"])
            imu_spl = generate_interface_bundle({"interface_type": 1, "platform": 1, "imu_format": fmt_id, "uart_handle": "USART3"})
            self.assertIn("USART3_IRQHandler", imu_spl["main_imu_spl.c"])
            imu_ino = generate_interface_bundle({"interface_type": 1, "platform": 2, "imu_format": fmt_id})
            self.assertIn(expected_hdr, imu_ino["EmbeddedDebugStudio_IMU.ino"])

        # Remote J,x,y across 3 platforms with exact mix_wheels parity & failsafe
        rem_hal = generate_interface_bundle({"interface_type": 2, "platform": 0, "remote_failsafe_ms": 350, "remote_pwm_limit": 7200})
        self.assertIn("REMOTE_FAILSAFE_MS   350U", rem_hal["remote_port_hal.h"])
        self.assertIn("left_pct = -inner;", rem_hal["remote_port_hal.c"])
        rem_spl = generate_interface_bundle({"interface_type": 2, "platform": 1, "uart_handle": "USART2"})
        self.assertIn("Remote_Port_SPL_IRQHandler", rem_spl["remote_port_spl.c"])
        rem_ino = generate_interface_bundle({"interface_type": 2, "platform": 2})
        self.assertIn("mixWheels", rem_ino["EmbeddedDebugStudio_Remote.ino"])

        # Per-interface guides exist for all 3 x 3 combinations
        for iface in (0, 1, 2):
            for plat in (0, 1, 2):
                html = build_interface_guide_html(iface, plat)
                self.assertIn("全软件三大串口接口速查总览", html)
                self.assertTrue(("PID" in html) or ("IMU" in html) or ("遥控" in html))

        # 4) UI switching, platform auto UART handle, presets, and workbench shortcuts
        page = FirmwareStudioPage()
        self.assertIsInstance(page.splitter, W.QSplitter)
        self.assertEqual(page.splitter.count(), 2)
        page.platform_box.setCurrentIndex(1)
        self.assertEqual(page.uart_edit.text(), "USART2")
        page.platform_box.setCurrentIndex(2)
        self.assertEqual(page.uart_edit.text(), "Serial")
        page.platform_box.setCurrentIndex(0)
        self.assertEqual(page.uart_edit.text(), "huart2")

        page.apply_preset(2)
        self.assertEqual(page.channels_data()[0][1], "output")
        page.row_selector.setValue(3)
        page.move_channel(-1)
        self.assertEqual(page.channels_data()[1][1], "actual")
        page.add_channel()
        self.assertEqual(len(page.channels_data()), 5)
        page.remove_channel()
        self.assertEqual(len(page.channels_data()), 4)
        page.set_interface_type(1)
        self.assertIn("imu_port_hal.c", page.current_bundle())
        page.set_interface_type(2)
        self.assertIn("remote_port_hal.c", page.current_bundle())
        with TemporaryDirectory() as exp_dir:
            written = page.export_bundle(exp_dir)
            self.assertEqual(len(written), 4)
            self.assertTrue((Path(exp_dir) / "remote_port_hal.c").is_file())
        page.close()

        # 5) Cross-workbench centralized jump shortcuts from PID / IMU / Remote
        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "jump.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            w.remote_page.timer.stop()

            w.imu_page.proto_box.setCurrentIndex(2)  # 6-axis physical -> imu_format 0
            w.imu_page.baud_box.setCurrentText("256000")
            w.imu_page.open_imu_interface_builder()
            self.assertIs(w.module_stack.currentWidget(), w.firmware_page)
            self.assertEqual(w.firmware_page.interface_type(), 1)
            self.assertEqual(w.firmware_page.imu_format_box.currentIndex(), 0)
            self.assertEqual(w.firmware_page.baud_box.currentText(), "256000")

            w.remote_page.fmt_edit.setText("CMD,{x},{y}")
            w.remote_page.baud_box.setCurrentText("9600")
            w.remote_page.open_remote_interface_builder()
            self.assertIs(w.module_stack.currentWidget(), w.firmware_page)
            self.assertEqual(w.firmware_page.interface_type(), 2)
            self.assertEqual(w.firmware_page.remote_fmt_edit.text(), "CMD,{x},{y}")
            self.assertEqual(w.firmware_page.baud_box.currentText(), "9600")

            w.open_firmware_builder(0)
            self.assertEqual(w.firmware_page.interface_type(), 0)
            w.close()

    def test_v18_wheel_guard_requires_click_focus(self):
        def wheel(widget, dy=120):
            ev = QtGui.QWheelEvent(
                C.QPointF(10, 10),
                C.QPointF(widget.mapToGlobal(C.QPoint(10, 10))),
                C.QPoint(0, 0),
                C.QPoint(0, dy),
                C.Qt.MouseButton.NoButton,
                C.Qt.KeyboardModifier.NoModifier,
                C.Qt.ScrollPhase.NoScrollPhase,
                False,
            )
            W.QApplication.sendEvent(widget, ev)

        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "wheel.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            w.remote_page.timer.stop()
            w.show()
            W.QApplication.setActiveWindow(w)
            W.QApplication.processEvents()

            # 1) PID ParameterSpin ignores wheel until clicked/focused
            kp = w.fields["KP"]
            kp.setValue(5.0)
            kp.clearFocus()
            if kp.lineEdit() is not None:
                kp.lineEdit().clearFocus()
            W.QApplication.processEvents()
            wheel(kp, 120)
            if kp.lineEdit() is not None:
                wheel(kp.lineEdit(), 120)
            self.assertAlmostEqual(kp.value(), 5.0)

            kp.setFocus(C.Qt.FocusReason.MouseFocusReason)
            W.QApplication.processEvents()
            wheel(kp, 120)
            self.assertGreater(kp.value(), 5.0)

            # 2) FirmwareStudioPage spinbox ignores wheel until clicked/focused
            w.switch_module(4)
            W.QApplication.processEvents()
            fw_kp = w.firmware_page.def_kp
            v0 = fw_kp.value()
            fw_kp.clearFocus()
            if fw_kp.lineEdit() is not None:
                fw_kp.lineEdit().clearFocus()
            w.firmware_page.code_edit.setFocus()
            W.QApplication.processEvents()
            wheel(fw_kp, 120)
            if fw_kp.lineEdit() is not None:
                wheel(fw_kp.lineEdit(), 120)
            self.assertAlmostEqual(fw_kp.value(), v0)

            fw_kp.setFocus(C.Qt.FocusReason.MouseFocusReason)
            W.QApplication.processEvents()
            wheel(fw_kp, 120)
            self.assertGreater(fw_kp.value(), v0)

            # 3) Platform & IMU format QComboBoxes render at full readable width (>=140 px)
            self.assertGreaterEqual(w.firmware_page.platform_box.width(), 140)
            w.firmware_page.set_interface_type(1)
            W.QApplication.processEvents()
            self.assertGreaterEqual(w.firmware_page.imu_format_box.width(), 140)
            w.close()

    def test_v18_bluetooth_scanner_pairing_and_rfcomm_remote(self):
        from types import SimpleNamespace
        from bluetooth_link import BluetoothDevice, extract_bt_mac_from_hwid, map_bluetooth_com_ports

        # 1) HWID MAC extraction & COM mapping
        self.assertEqual(
            extract_bt_mac_from_hwid(r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&005D\7&982190C&2&00260102EAC0_C00000000"),
            "00:26:01:02:EA:C0",
        )
        self.assertIsNone(
            extract_bt_mac_from_hwid(r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0000\7&982190C&2&000000000000_00000009")
        )
        com_map = map_bluetooth_com_ports([
            SimpleNamespace(device="COM10", hwid=r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&005D\7&1&00260102EAC0_C00000000"),
            SimpleNamespace(device="COM9", hwid=r"USB\VID_0843&PID_5740"),
        ])
        self.assertEqual(com_map, {"00:26:01:02:EA:C0": "COM10"})

        # 2) UI Bluetooth drawer expand/collapse, scan, PIN pairing, RFCOMM connect & flush_and_close
        class FakeRfcommLink:
            instances = []

            def __init__(self, address, channel=1, name="", timeout=5.0):
                from threading import Event
                self.address = address
                self.channel = channel
                self.device_name = name
                self.closed = Event()
                self.events = [("open", 0.0, f"蓝牙 {name} ({address})")]
                self.sent = []
                self.flushed = []
                self.closed_flag = False
                FakeRfcommLink.instances.append(self)

            def start(self):
                return None

            def send(self, data: bytes):
                self.sent.append(data)
                self.events.append(("tx", 0.0, data))

            def take(self):
                out = list(self.events)
                self.events.clear()
                return out, 0

            def flush_and_close(self, final_data: bytes = b"", timeout: float = 0.25):
                self.flushed.append(final_data)
                self.closed_flag = True
                self.closed.set()
                self.events.append(("closed", 0.0, None))

            def is_alive(self):
                return False

            def join(self, timeout=None):
                return None

        devs_before = [BluetoothDevice("HC-05", "00:26:01:02:EA:C0", authenticated=False, remembered=False, connected=False, com_port=None)]
        devs_after = [BluetoothDevice("HC-05", "00:26:01:02:EA:C0", authenticated=True, remembered=True, connected=False, com_port="COM10")]

        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "bt.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            rp = w.remote_page
            rp.timer.stop()
            w.switch_module(3)

            self.assertFalse(rp.bt_panel.isHidden())
            self.assertTrue(rp.bt_drawer_btn.isChecked())
            rp.toggle_bt_drawer(False)
            self.assertTrue(rp.bt_panel.isHidden())
            self.assertFalse(rp.bt_drawer_btn.isChecked())
            rp.toggle_bt_drawer(True)
            self.assertFalse(rp.bt_panel.isHidden())
            self.assertTrue(rp.bt_drawer_btn.isChecked())

            rp.set_bt_devices(devs_before)
            self.assertEqual(rp.bt_table.rowCount(), 1)
            self.assertTrue(rp.bt_pair_btn.isEnabled())

            with patch("remote_studio.pair_bluetooth_device", return_value=(True, "配对成功")) as mock_pair, \
                 patch("remote_studio.discover_bluetooth_devices", return_value=devs_after):
                rp.bt_pin_edit.setText("1234")
                rp.pair_selected_bluetooth()
                deadline = time.monotonic() + 1.5
                while rp._bt_busy and time.monotonic() < deadline:
                    rp.tick()
                    time.sleep(0.01)
                mock_pair.assert_called_once_with("00:26:01:02:EA:C0", pin="1234", hwnd=0)
            self.assertFalse(rp.bt_pair_btn.isEnabled())
            self.assertIn("COM10", rp.bt_table.item(0, 3).text())

            with patch("remote_studio.BluetoothRfcommLink", FakeRfcommLink):
                rp.connect_selected_bluetooth()
                rp.tick()
                self.assertTrue(rp.connected)
                self.assertIsNone(rp.active_port())
                self.assertIn("蓝牙已连 HC-05", rp.badge.text())
                rp._on_single_joy(30, 70, False)
                self.assertIn(b"J,30,70\n", FakeRfcommLink.instances[-1].sent)
                rp.disconnect_port()
                rp.tick()
                self.assertIn(b"J,0,0\nS\n", FakeRfcommLink.instances[-1].flushed)
                self.assertFalse(rp.connected)

            # Selecting mapped COM port sets COM10 and attempts Serial connection
            with patch("remote_studio.SerialLink") as mock_serial_cls:
                rp.use_selected_bt_com()
                self.assertEqual(rp.port_box.currentText(), "COM10")
                mock_serial_cls.assert_called_once()
            rp.link = None
            rp.disconnect_port()
            w.close()

    def test_v19_safety_schema_timestamps_disturbance_and_streaming_csv(self):
        from bluetooth_link import BluetoothRfcommLink
        from engine import disturbance_metrics
        from autotune import evaluate_step
        from storage import inspect_csv, load_csv
        from firmware_studio import generate_interface_bundle

        # 1) Bluetooth RFCOMM send(urgent=True) must purge stale motion frames without TypeError
        rf = BluetoothRfcommLink("00:26:01:02:EA:C0")
        rf.send(b"J,30,30\n")
        rf.send(b"J,50,50\n")
        rf.send(b"J,70,70\n")
        self.assertEqual(len(rf.writes), 3)
        self.assertTrue(rf.send(b"J,0,0\nS\n", urgent=True))
        self.assertEqual(list(rf.writes), [b"J,0,0\nS\n"])

        with TemporaryDirectory() as tmp:
            w = Window(Path(tmp) / "v19.ini")
            w.timer.stop()
            w.serial_page.poll_timer.stop()
            w.imu_page.timer.stop()
            w.remote_page.timer.stop()

            # 2) Full address labels in PID form update with group & addressing mode
            w.group.setCurrentIndex(1)  # SPD
            self.assertEqual(w.field_labels["KP"].text(), "SPD.KP")
            w.address.setCurrentIndex(1)  # @KP
            self.assertEqual(w.field_labels["KP"].text(), "@KP")
            w.address.setCurrentIndex(0)

            # 3) Balance-car 5-col schema parity across PID page and IMU page + "未上报" explicit indicator
            bal_imu_code = generate_interface_bundle({"interface_type": 1, "platform": 0, "imu_format": 2})["imu_port_hal.c"]
            self.assertIn("#NAMES:pitch_target,pitch,gyro_pitch,pwm,wheel_speed", bal_imu_code)

            frame_chunk = (
                b"#NAMES:pitch_target,pitch,gyro_pitch,pwm,wheel_speed\n"
                b"#ROLES:target,actual,other,output,other\n"
                b"1.50,-2.25,18.50,-340.0,120.0\n"
            )
            w.connected = True
            w.receive(frame_chunk, 1.0)
            self.assertEqual(w.count, 5)
            self.assertEqual(w.target_ch.currentData(), 0)
            self.assertEqual(w.actual_ch.currentData(), 1)
            self.assertIn("°", w.units[1])
            self.assertIn("°/s", w.units[2])
            self.assertEqual(w.units[4], "cnt/s")

            imu = w.imu_page
            imu.connected = True
            imu.proto_box.setCurrentIndex(0)
            imu._feed_bytes(frame_chunk, 1.0)
            self.assertAlmostEqual(imu.target_pitch, 1.50, places=4)
            self.assertAlmostEqual(imu.pitch, -2.25, places=4)
            self.assertAlmostEqual(imu.gx, 18.50, places=4)
            self.assertAlmostEqual(imu.pwm, -340.0, places=4)
            self.assertAlmostEqual(imu.wheel_speed, 120.0, places=4)

            # Feed a 4-col stream lacking gyro_pitch -> IMU must display "未上报" for gyro_pitch, NOT PWM
            imu.stream_names = []
            imu.semantic_map = {}
            imu.frames = 0
            imu._feed_bytes(
                b"#NAMES:pitch_target,pitch,pwm,wheel_speed\n2.00,-1.00,-280.0,95.0\n",
                2.0,
            )
            imu._refresh_readouts()
            txt_partial = imu.readout_lbl.text()
            self.assertIn("角速度 GX  :  未上报", txt_partial)
            self.assertIn("-280", txt_partial)
            self.assertIn("95.00", txt_partial)

            # 4) IMU burst coalescing immunity (seq,mcu_ms + fixed rate) & stationary zero guard
            from imu_studio import ImuStudioPage
            imu_seq = ImuStudioPage(w)
            imu_seq.timer.stop()
            imu_seq.proto_box.setCurrentIndex(2)  # 6-axis physical
            imu_seq.time_source_box.setCurrentIndex(0)  # Auto (MCU timestamp / fixed rate)
            imu_seq.sample_rate_spin.setValue(50.0)
            lines = [b"#NAMES:ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,seq,mcu_ms\n"]
            for idx in range(30):
                seq = idx + (1 if idx >= 10 else 0)  # deliberate drop of 1 seq at idx==10
                ms = idx * 20
                lines.append(f"0.000,0.020,1.000,12.0,0.0,25.0,{seq},{ms}\n".encode())
            imu_seq.connected = True
            for ln in lines:
                imu_seq._feed_bytes(ln, 10.0)  # same host stamp_mono!
            yaw_single = imu_seq.yaw
            pitch_single = imu_seq.pitch
            self.assertEqual(imu_seq.seq_lost, 1)
            self.assertIn("MCU时间戳", imu_seq.active_time_source)

            # Now feed all 30 frames coalesced in ONE chunk at same stamp_mono
            imu_burst = ImuStudioPage(w)
            imu_burst.timer.stop()
            imu_burst.proto_box.setCurrentIndex(2)
            imu_burst.time_source_box.setCurrentIndex(0)
            imu_burst.sample_rate_spin.setValue(50.0)
            imu_burst.connected = True
            imu_burst._feed_bytes(b"".join(lines), 10.0)
            self.assertAlmostEqual(pitch_single, imu_burst.pitch, places=6)
            self.assertAlmostEqual(yaw_single, imu_burst.yaw, places=6)

            # Stationary calibration accepts still car, rejects shaking car
            imu_burst.start_zero_capture()
            for _ in range(50):
                imu_burst._feed_bytes(b"#NAMES:pitch_target,pitch,gyro_pitch,pwm,wheel_speed\n0,1.25,0.2,0,0\n", 11.0)
            self.assertTrue(imu_burst.sync_pid_btn.isEnabled())
            imu_burst.start_zero_capture()
            for k in range(50):
                ang = 1.25 + (3.0 if k % 2 else -3.0)
                imu_burst._feed_bytes(f"0,{ang:.2f},18.0,0,0\n".encode(), 12.0 + 0.02 * k)
            self.assertFalse(imu_burst.sync_pid_btn.isEnabled())
            self.assertIn("波动过大", imu_burst.zero_result.text())
            imu_seq.close(); imu_burst.close()

            # 5) Load-disturbance capture metrics + trigger-aligned ghost comparison + cursor override
            t_d = np.linspace(0.0, 4.0, 201)
            y_d = np.full_like(t_d, 200.0)
            u_d = np.full_like(t_d, 350.0)
            # Load applied at 1.0..1.6s: speed drops to 160, PWM hits 1000 limit; unload -> overshoot 218, settles by 2.4s
            load_m = (t_d >= 1.0) & (t_d <= 1.6)
            y_d[load_m] = 200.0 - 40.0 * np.sin(np.pi * (t_d[load_m] - 1.0) / 0.6)
            u_d[load_m] = 1000.0
            rec_m = (t_d > 1.6) & (t_d < 2.4)
            y_d[rec_m] = 200.0 + 18.0 * np.exp(-2.5 * (t_d[rec_m] - 1.6)) * np.sin(np.pi * (t_d[rec_m] - 1.6) / 0.8)
            dm = disturbance_metrics(t_d, 200.0, y_d, outputs=u_d, band=0.02, output_limit=1000.0, t_load=1.0, t_unload=1.6)
            self.assertGreaterEqual(dm["max_drop"], 39.0)
            self.assertGreater(dm["unload_overshoot"], 7.0)
            self.assertIsNotNone(dm["recovery_s"])
            self.assertGreater(dm["sat_time_s"], 0.55)
            self.assertAlmostEqual(dm["pre_pwm"], 350.0, places=1)

            # UI trigger capture in "负载扰动对比" mode + ghost align + cursor recalc
            w.clear_plot()
            w.channel_roles = ["target", "actual", "output", ""] + [""] * 4
            w.count = 3
            w.refresh_channels()
            w.capture_mode.setCurrentIndex(1)
            w.capture_pwm_limit.setValue(1000.0)
            for ti, yi, ui in zip(t_d, y_d, u_d):
                w.buffer.add(float(ti), [200.0, float(yi), float(ui)], ("",) * 16)
            w.frames = len(t_d)
            w.latest = float(t_d[-1])
            w.toggle_freeze()
            w.capture.last_crossing = 1.05
            w.capture.last_range = (0.0, 4.0)
            w.on_capture_complete(1.05, 0.0, 4.0)
            self.assertTrue(w.pin_disturb_btn.isEnabled())
            self.assertIn("最大掉速", w.disturb_summary.text())
            w.pin_disturb_ghost()
            self.assertIsNotNone(w.scope.ghost)
            self.assertEqual(w.scope.ghost["kind"], "capture")
            w.scope.place_cursor(1.0)
            w.scope.place_cursor(1.6)
            w.recalc_disturbance_with_cursors()
            self.assertIn("触发前 PWM=350", w.disturb_summary.text())

            # 6) AutoTune real output_limit saturation + role conflict guard
            ev_sat = evaluate_step(t_d[:50], np.linspace(0, 120, 50), 0.0, 200.0, outputs=np.full(50, 990.0), output_limit=1000.0)
            self.assertTrue(ev_sat.saturated)
            self.assertAlmostEqual(ev_sat.sat_ratio, 1.0, places=2)
            self.assertEqual(ev_sat.sample_count, 50)

            w.toggle_freeze()  # unfreeze
            w.channel_preferences[1] = 2  # deliberately mis-select CH3 (output) while #ROLES declares CH2 as actual
            w.refresh_channels()
            conflict, _ = w.autotuner.has_role_conflict()
            self.assertTrue(conflict)
            self.assertFalse(w.autotuner.start(0, 1, 1, 0.0, 200.0, confirm_role=False))

            # 7) Streaming long CSV (>20,000 rows) inspect + [t_start, t_end] window replay
            long_csv = Path(tmp) / "long_record.csv"
            with open(long_csv, "w", encoding="utf-8-sig", newline="") as f:
                f.write("time_s,CH1_target,CH2_actual,CH3_pwm,CH4_err\n")
                for row_i in range(25000):
                    ts = row_i * 0.01
                    f.write(f"{ts:.2f},100.0,{min(100.0, ts * 5):.2f},400.0,0.0\n")
            meta = inspect_csv(long_csv)
            self.assertEqual(meta["total_rows"], 25000)
            self.assertAlmostEqual(meta["t_start"], 0.0, places=2)
            self.assertAlmostEqual(meta["t_end"], 249.99, places=2)
            self.assertTrue(w.load_csv_file(str(long_csv), t_start=0.0, t_end=25.0))
            self.assertEqual(w.frames, 2501)
            self.assertAlmostEqual(w.scope.data[0, 0], 0.0, places=2)
            self.assertAlmostEqual(w.scope.data[-1, 0], 25.0, places=2)
            self.assertIn("已载入 CSV 分段", w.ack.text())

            w.close()

    def test_v19_audit_fixes_spl_uarts_units_disturbance_peak_and_arduino_hook(self):
        from firmware_studio import generate_interface_bundle
        from telemetry_schema import infer_channel_unit, resolve_channel_semantics
        from engine import disturbance_metrics

        # 1) SPL UART4 & UART5 generate real PC10/11 and PC12/PD2 pins + APB1/APB2 RCC; USART6 raises ValueError
        u4 = generate_interface_bundle({"interface_type": 0, "platform": 1, "uart_handle": "UART4"})
        self.assertIn("RCC_APB1Periph_UART4", u4["main_integration_spl.c"])
        self.assertIn("GPIOC", u4["main_integration_spl.c"])
        self.assertIn("UART4_IRQn", u4["main_integration_spl.c"])
        self.assertIn("STM32F103C8T6", u4["README_接入说明.md"])
        self.assertIn("xE/xG", u4["README_接入说明.md"])
        u5 = generate_interface_bundle({"interface_type": 1, "platform": 1, "uart_handle": "UART5"})
        self.assertIn("RCC_APB1Periph_UART5", u5["main_imu_spl.c"])
        self.assertIn("GPIO_Pin_12", u5["main_imu_spl.c"])
        self.assertIn("GPIOD", u5["main_imu_spl.c"])
        self.assertIn("UART5_IRQn", u5["main_imu_spl.c"])
        self.assertIn("STM32F103C8T6", u5["README_接入说明.md"])
        u2 = generate_interface_bundle({"interface_type": 0, "platform": 1, "uart_handle": "USART2"})
        self.assertNotIn("STM32F103C8T6", u2["README_接入说明.md"])

        # Explicit UART auto-mapping notice when handle format is adapted across platforms
        mapped_spl = generate_interface_bundle({"interface_type": 0, "platform": 1, "uart_handle": "huart2"})
        self.assertIn("USART2_IRQHandler", mapped_spl["main_integration_spl.c"])
        self.assertIn("已自动映射：huart2 → USART2", mapped_spl["README_接入说明.md"])
        mapped_hal = generate_interface_bundle({"interface_type": 0, "platform": 0, "uart_handle": "USART2"})
        self.assertIn("huart2", mapped_hal["main_integration_example.c"])
        self.assertIn("已自动映射：USART2 → huart2", mapped_hal["README_接入说明.md"])

        with self.assertRaises(ValueError):
            generate_interface_bundle({"interface_type": 0, "platform": 1, "uart_handle": "USART6"})

        # 2) Generic PID channels ("target", "actual", "output") stay unitless while pitch_* keeps deg
        self.assertEqual(infer_channel_unit("target"), "")
        self.assertEqual(infer_channel_unit("actual"), "")
        self.assertEqual(infer_channel_unit("output"), "")
        self.assertEqual(infer_channel_unit("pitch_target"), "°")
        self.assertEqual(infer_channel_unit("pitch"), "°")
        self.assertEqual(infer_channel_unit("angle_deg"), "°")
        smap = resolve_channel_semantics(["target", "actual", "output", "error"])
        self.assertNotIn("pitch_target", smap)
        self.assertNotIn("pwm", smap)

        # 3) Overdamped disturbance with zero unload overshoot must report t_peak=None (not tail timestamp)
        t_over = np.linspace(0.0, 3.0, 151)
        y_over = np.full_like(t_over, 200.0)
        y_over[(t_over >= 0.8) & (t_over <= 1.4)] = 175.0
        dm_over = disturbance_metrics(t_over, 200.0, y_over, t_load=0.8, t_unload=1.4)
        self.assertEqual(dm_over["unload_overshoot"], 0.0)
        self.assertIsNone(dm_over["t_peak"])

        # 4) Arduino PID sketch enforces PID_SENSOR_HOOK_READY guard so unmodified 0.0f stub cannot @RUN
        ino = generate_interface_bundle({"interface_type": 0, "platform": 2})["EmbeddedDebugStudio_PID.ino"]
        self.assertIn("#include <stdlib.h>", ino)
        self.assertIn("#define PID_SENSOR_HOOK_READY 0", ino)
        self.assertIn("readSensorFeedback()", ino)
        self.assertIn("@ERR:implement readSensorFeedback()", ino)

    def test_v19_autotune_safety_and_identification_fixes(self):
        from autotune import AutoTuner, evaluate_step, identify_from_p_step
        from configuration import DEFAULT_RULES

        rules = dict(DEFAULT_RULES)
        t_v = np.linspace(0.0, 2.0, 50)

        # 1a) Saturated stall (actual==0, output==1000): must stop immediately without escalating KP
        tuner_stall = AutoTuner(0, 1, 0, {"KP": 0.0, "KI": 0.0, "KD": 0.0}, rules, base=0.0, target=100.0, output_limit=1000.0)
        rnd, cand, _ = tuner_stall.next_candidate()
        self.assertEqual(rnd, 1)
        ev_stall = tuner_stall.record_result(t_v, np.zeros_like(t_v), base_y=0.0, outputs=np.full_like(t_v, 1000.0))
        self.assertTrue(ev_stall.stalled)
        self.assertFalse(ev_stall.valid)
        self.assertTrue(tuner_stall.finished)
        self.assertIsNone(tuner_stall.next_candidate())
        self.assertIsNone(tuner_stall.best_params)

        # 1b) Without output channel, zero response may boost probe AT MOST ONCE then stop cleanly
        tuner_nfb = AutoTuner(0, 1, 0, {"KP": 0.0, "KI": 0.0, "KD": 0.0}, rules, base=0.0, target=100.0)
        kps_seen = []
        for _ in range(5):
            nxt = tuner_nfb.next_candidate()
            if nxt is None:
                break
            _, c, _ = nxt
            kps_seen.append(c["KP"])
            tuner_nfb.record_result(t_v, np.zeros_like(t_v), base_y=0.0, outputs=None)
        self.assertEqual(len(kps_seen), 2)
        self.assertAlmostEqual(kps_seen[0], 1.0, places=3)
        self.assertAlmostEqual(kps_seen[1], 3.0, places=3)
        self.assertTrue(tuner_nfb.finished)
        self.assertIsNone(tuner_nfb.best_params)

        # 4) ANG local tuning preserves Round 1 baseline verbatim (e.g. KP=25, KI=0, KD=1.0 even in PI mode)
        tuner_ang = AutoTuner(0, 1, 1, {"KP": 25.0, "KI": 0.0, "KD": 1.0}, rules, base=0.0, target=3.0)
        _, b_ang, b_tag = tuner_ang.next_candidate()
        self.assertEqual(b_tag, "当前基准")
        self.assertEqual(b_ang, {"KP": 25.0, "KI": 0.0, "KD": 1.0})

        # 6) All-NaN feedback round produces sample_count=0, valid=False, and NEVER enters best_params
        ev_nan = evaluate_step(t_v, np.full_like(t_v, np.nan), 0.0, 100.0)
        self.assertFalse(ev_nan.valid)
        self.assertEqual(ev_nan.sample_count, 0)
        tuner_nan = AutoTuner(0, 1, 0, {"KP": 1.0, "KI": 0.0, "KD": 0.0}, rules, base=0.0, target=100.0)
        tuner_nan.next_candidate()
        ev_r = tuner_nan.record_result(t_v, np.full_like(t_v, np.nan), base_y=0.0)
        self.assertFalse(ev_r.valid)
        self.assertIsNone(tuner_nan.best_params)

        # 7a) Non-zero baseline identification uses delta_y / delta_r (r:100->150, y:80->120 => alpha=0.8)
        y_nz = 80.0 + 40.0 * (1.0 - np.exp(-t_v / 0.25))
        cands_nz = identify_from_p_step(t_v, y_nz, base_y=80.0, goal_y=150.0, kp_probe=2.0, mode=0, style=1, base_cmd=100.0)
        cands_old = identify_from_p_step(t_v, y_nz, base_y=80.0, goal_y=150.0, kp_probe=2.0, mode=0, style=1)
        self.assertTrue(cands_nz and cands_old)
        # True K_plant is (0.8/(0.2*2))=2.0 (smaller KP needed than biased alpha=0.57 => K=0.66)
        self.assertLess(abs(cands_nz[0]["KP"]), abs(cands_old[0]["KP"]))

        with TemporaryDirectory() as tmp:
            # 2 & 3) Command failure clears old queue BEFORE enqueuing restore; partial @STOP ACK reported clearly
            w = Window(Path(tmp) / "fail_order.ini")
            w.timer.stop()
            w.connected = True
            w.sim = None
            w.group.setCurrentIndex(1)  # SPD
            for k, v in (("KP", 5.0), ("KI", 30.0), ("KD", 0.05), ("TGT", 100.0)):
                w.fields[k].setValue(v)
                w.confirmed[f"SPD.{k}"] = v
            sent_bytes = []
            class FakeAsyncLink:
                def send(self, data, urgent=False):
                    sent_bytes.append((bytes(data), urgent))
                    return True
            w.link = FakeAsyncLink()
            self.assertTrue(w.autotuner.start(0, 1, 0, base=0.0, target=100.0, step_seconds=2.0, max_rounds=4))
            # Enqueue extra command behind pending, then fail mid-run
            w.commands.waiting.append(("@SPD.KP=99.0000", "SPD", False, 999))
            w.commands.failed("应答超时")
            # Stale @SPD.KP=99 must be gone; FINALIZING must still be active with @STOP + restore queued!
            queued_cmds = [cmd for cmd, _, _, _ in w.commands.waiting]
            pending_cmd = w.commands.pending[0] if w.commands.pending else ""
            all_active_cmds = [pending_cmd, *queued_cmds]
            self.assertNotIn("@SPD.KP=99.0000", all_active_cmds)
            self.assertEqual(pending_cmd, "@STOP")
            self.assertTrue(any("5.0000" in c for c in queued_cmds))
            self.assertEqual(w.autotuner.state, w.autotuner.FINALIZING)

            # @STOP gets ACKed, then link fails while restoring KI -> clear partial status!
            w.commands.reply("ACK", "STOP")
            self.assertTrue(w.autotuner.finalize_stop_acked)
            w.commands.failed("串口断开")
            self.assertEqual(w.autotuner.state, w.autotuner.IDLE)
            self.assertIn("设备已停机，参数恢复未确认", w.autotune_status.text())
            w.link = None
            w.close()

            # 5) Reversed-sign Round 1 exits GUI cleanly with ZERO TypeError and sends @STOP + restore
            w2 = Window(Path(tmp) / "rev.ini")
            w2.timer.stop()
            w2.connected = True
            w2.sim = None
            w2.group.setCurrentIndex(1)
            for k, v in (("KP", 5.0), ("KI", 30.0), ("KD", 0.05), ("TGT", 100.0)):
                w2.fields[k].setValue(v)
                w2.confirmed[f"SPD.{k}"] = v
            sent_rev = []
            w2.commands.submit = lambda cmds, quiet=False: (sent_rev.extend(cmds) or True)
            w2.commands.busy = lambda: False
            w2.receive(b"#NAMES:target,actual,pwm,err\n#ROLES:target,actual,output,error\n0,0,0,0\n", 1.0)
            self.assertTrue(w2.autotuner.start(0, 1, 0, base=0.0, target=100.0, step_seconds=0.6, max_rounds=5))
            w2.autotuner.tick(1.02)  # WAIT_START -> PREPARE
            w2.autotuner.tick(1.03)  # PREPARE -> WAIT_BASE_ACK
            w2.autotuner.tick(1.04)  # WAIT_BASE_ACK -> SETTLE_BASE
            for i in range(10):
                w2.sample(1.05 + i * 0.04, [0.0, 0.0, 0.0, 0.0])
            w2.autotuner.tick(1.45)  # SETTLE_BASE -> WAIT_STEP_ACK
            w2.autotuner.tick(1.46)  # WAIT_STEP_ACK -> OBSERVE
            sent_rev.clear()
            for i in range(16):
                w2.sample(1.50 + i * 0.05, [100.0, -35.0, -500.0, 135.0])
            w2.autotuner.tick(2.30)
            self.assertFalse(w2.autotuner.active)
            self.assertIn("反向响应", w2.autotune_status.text())
            self.assertIn("@STOP", sent_rev)
            self.assertTrue(any("SPD.KP=5.0000" in c for c in sent_rev))
            w2.close()

            # 7b) Mid-round online stall abort (< step_seconds) + target-jump vs 2-col ACK fallback
            w3 = Window(Path(tmp) / "stall_and_t0.ini")
            w3.timer.stop()
            w3.connected = True
            w3.sim = None
            w3.group.setCurrentIndex(1)
            for k, v in (("KP", 2.0), ("KI", 5.0), ("KD", 0.0), ("TGT", 100.0)):
                w3.fields[k].setValue(v)
                w3.confirmed[f"SPD.{k}"] = v
            sent_s = []
            w3.commands.submit = lambda cmds, quiet=False: (sent_s.extend(cmds) or True)
            w3.commands.busy = lambda: False
            w3.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n0,0,0\n", 1.0)
            self.assertTrue(w3.autotuner.start(0, 1, 0, base=0.0, target=100.0, step_seconds=5.0, max_rounds=4, output_limit=1000.0))
            w3.autotuner.tick(1.01); w3.autotuner.tick(1.02); w3.autotuner.tick(1.03)
            for i in range(8):
                w3.sample(1.05 + i * 0.04, [0.0, 0.0, 0.0])
            w3.autotuner.tick(1.40)  # WAIT_STEP_ACK
            w3.autotuner.tick(1.41)  # OBSERVE starts
            # Feed target jump at exact frame t=1.60 while feedback stays 0 and PWM=1000 for 1.8s (<5.0s!)
            w3.sample(1.50, [0.0, 0.0, 0.0])
            for i in range(36):
                w3.sample(1.60 + i * 0.05, [100.0, 0.0, 1000.0])
            w3.autotuner.tick(3.40)
            self.assertFalse(w3.autotuner.active)
            self.assertEqual(w3.autotuner.step_t0_source, "目标列跳变起点")
            self.assertAlmostEqual(w3.autotuner.step_t0, 1.60, places=3)
            self.assertIn("输出触顶且反馈无响应", w3.autotune_status.text())
            self.assertIn("@STOP", sent_s)

            # Two-column mode (actual, pwm) without target channel falls back cleanly to ACK timing
            w3.clear_plot()
            w3.receive(b"#NAMES:actual,pwm\n#ROLES:actual,output\n0,0\n", 4.0)
            self.assertTrue(w3.autotuner.start(0, 1, 0, base=0.0, target=100.0, step_seconds=2.0, max_rounds=4))
            w3.autotuner.tick(4.01); w3.autotuner.tick(4.02); w3.autotuner.tick(4.03)
            for i in range(8):
                w3.sample(4.05 + i * 0.04, [0.0, 0.0])
            w3.autotuner.tick(4.40); w3.autotuner.tick(4.41)
            self.assertEqual(w3.autotuner.step_t0_source, "估计起点(ACK)")
            w3.close()

    def test_v19_autotune_feedback_loss_device_stop_and_round1_settle(self):
        with TemporaryDirectory() as tmp:
            # A) Feedback loss (OBSERVE wall_deadline timeout) after Round 1 succeeded and hold_target=True
            # MUST @STOP and restore saved_before (NOT apply winner target=200 without @STOP), while keeping best ref
            w_to = Window(Path(tmp) / "timeout_stop.ini")
            w_to.timer.stop()
            w_to.connected = True
            w_to.sim = None
            w_to.group.setCurrentIndex(1)  # SPD
            for k, v in (("KP", 2.0), ("KI", 5.0), ("KD", 0.0), ("TGT", 10.0)):
                w_to.fields[k].setValue(v)
                w_to.confirmed[f"SPD.{k}"] = v
            sent_to = []
            w_to.commands.submit = lambda cmds, quiet=False: (sent_to.extend(cmds) or True)
            w_to.commands.busy = lambda: False
            w_to.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n0,0,0\n", 1.0)
            self.assertTrue(
                w_to.autotuner.start(0, 1, 0, base=0.0, target=200.0, step_seconds=0.8, max_rounds=4, hold_target=True)
            )
            w_to.autotuner.tick(1.01)  # WAIT_START -> PREPARE
            w_to.autotuner.tick(1.02)  # PREPARE (R1) -> WAIT_BASE_ACK
            w_to.autotuner.tick(1.03)  # WAIT_BASE_ACK -> SETTLE_BASE
            for i in range(10):
                w_to.sample(1.05 + i * 0.04, [0.0, 0.0, 0.0])
            w_to.autotuner.tick(1.45)  # SETTLE_BASE -> WAIT_STEP_ACK
            w_to.autotuner.tick(1.46)  # WAIT_STEP_ACK -> OBSERVE (R1)
            for i in range(25):
                t_s = 1.50 + i * 0.04
                y_s = 200.0 * (1.0 - np.exp(-max(0.0, t_s - 1.50) / 0.18))
                w_to.sample(t_s, [200.0, float(y_s), 450.0])
            w_to.autotuner.tick(2.60)  # finishes R1 cleanly, advances to PREPARE(R2)
            self.assertIsNotNone(w_to.autotuner.engine.best_params)
            self.assertEqual(w_to.autotuner.state, w_to.autotuner.PREPARE)

            w_to.autotuner.tick(2.61)  # PREPARE (R2) -> WAIT_BASE_ACK
            w_to.autotuner.tick(2.62)  # WAIT_BASE_ACK -> SETTLE_BASE
            for i in range(10):
                w_to.sample(2.65 + i * 0.04, [0.0, 0.0, 0.0])
            w_to.autotuner.tick(3.05)  # SETTLE_BASE -> WAIT_STEP_ACK
            w_to.autotuner.tick(3.06)  # WAIT_STEP_ACK -> OBSERVE (R2)
            sent_to.clear()
            w_to.autotuner.tick(w_to.autotuner.wall_deadline + 1.0)  # telemetry dropped!
            self.assertFalse(w_to.autotuner.active)
            self.assertEqual(sent_to[0], "@STOP")
            self.assertTrue(any("SPD.KP=2.0000" in c for c in sent_to))
            self.assertTrue(any("SPD.TGT=10.0000" in c for c in sent_to))
            self.assertFalse(any("SPD.TGT=200.0000" in c for c in sent_to))
            st_to = w_to.autotune_status.text()
            self.assertIn("阶跃采样中断", st_to)
            self.assertIn("此前已测最优参考", st_to)
            w_to.close()

            # B) Device #STOP:fall during active tuning clears stale commands FIRST and keeps restore commands queued
            w_fall = Window(Path(tmp) / "fall_order.ini")
            w_fall.timer.stop()
            w_fall.connected = True
            w_fall.sim = None
            w_fall.group.setCurrentIndex(1)
            for k, v in (("KP", 4.0), ("KI", 20.0), ("KD", 0.0), ("TGT", 50.0)):
                w_fall.fields[k].setValue(v)
                w_fall.confirmed[f"SPD.{k}"] = v
            class FakeAsyncLink:
                def send(self, data, urgent=False):
                    return True
            w_fall.link = FakeAsyncLink()
            self.assertTrue(w_fall.autotuner.start(0, 1, 0, base=0.0, target=120.0, step_seconds=2.0, max_rounds=4))
            w_fall.commands.waiting.append(("@SPD.KP=88.0000", "SPD", False, 999))
            w_fall.receive(b"#STOP:fall\n", 2.0)
            queued_f = [cmd for cmd, _, _, _ in w_fall.commands.waiting]
            pending_f = w_fall.commands.pending[0] if w_fall.commands.pending else ""
            active_f = [pending_f, *queued_f]
            self.assertNotIn("@SPD.KP=88.0000", active_f)
            self.assertEqual(w_fall.autotuner.state, w_fall.autotuner.FINALIZING)
            self.assertTrue(any("4.0000" in c for c in active_f))
            self.assertFalse(w_fall.stop_banner.isHidden())
            self.assertIn("倒地", w_fall.stop_banner.text())
            # Drain all queued restore ACKs and verify clean IDLE completion
            for _ in range(12):
                if not w_fall.commands.pending:
                    w_fall.commands.dispatch()
                if not w_fall.commands.pending:
                    break
                cur_cmd = w_fall.commands.pending[0]
                w_fall.commands.reply("ACK", cur_cmd.lstrip("@"))
            self.assertFalse(w_fall.commands.busy())
            self.assertEqual(w_fall.autotuner.state, w_fall.autotuner.IDLE)
            self.assertIn("已恢复", w_fall.autotune_status.text())
            w_fall.link = None
            w_fall.close()

            # C) Round 1 SETTLE_BASE rejects stale pre-start frames & oscillating motion, accepts flat droop
            # C1: stale pre-start data only (no fresh frames after phase_t0) -> aborts cleanly
            w_r1 = Window(Path(tmp) / "r1_settle.ini")
            w_r1.timer.stop()
            w_r1.connected = True
            w_r1.sim = None
            w_r1.group.setCurrentIndex(1)
            for k, v in (("KP", 2.0), ("KI", 0.0), ("KD", 0.0), ("TGT", 0.0)):
                w_r1.fields[k].setValue(v)
                w_r1.confirmed[f"SPD.{k}"] = v
            sent_r1 = []
            w_r1.commands.submit = lambda cmds, quiet=False: (sent_r1.extend(cmds) or True)
            w_r1.commands.busy = lambda: False
            w_r1.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n0,0,0\n", 1.0)
            self.assertTrue(w_r1.autotuner.start(0, 1, 0, base=100.0, target=150.0, step_seconds=0.8, max_rounds=4))
            w_r1.autotuner.tick(1.01)  # WAIT_START -> PREPARE
            w_r1.autotuner.tick(1.02)  # PREPARE -> WAIT_BASE_ACK
            w_r1.autotuner.tick(1.03)  # WAIT_BASE_ACK -> SETTLE_BASE (phase_t0 = 1.0)
            sent_r1.clear()
            w_r1.autotuner.tick(w_r1.autotuner.wall_deadline + 0.5)
            self.assertFalse(w_r1.autotuner.active)
            self.assertIn("基准段未收到有效反馈", w_r1.autotune_status.text())
            self.assertIn("@STOP", sent_r1)
            self.assertFalse(any("SPD.TGT=150.0000" in c for c in sent_r1))

            # C2: fresh samples actively decelerating (not flat) -> rejects start with @STOP
            w_r1.clear_plot()
            w_r1.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n100,90,0\n", 2.0)
            self.assertTrue(w_r1.autotuner.start(0, 1, 0, base=100.0, target=150.0, step_seconds=0.8, max_rounds=4))
            w_r1.autotuner.tick(2.01); w_r1.autotuner.tick(2.02); w_r1.autotuner.tick(2.03)
            for i in range(12):
                w_r1.sample(2.05 + i * 0.10, [100.0, 90.0 - i * 5.0, 0.0])
            sent_r1.clear()
            w_r1.autotuner.tick(4.50)
            self.assertFalse(w_r1.autotuner.active)
            self.assertIn("起始基准未平稳", w_r1.autotune_status.text())
            self.assertIn("@STOP", sent_r1)
            self.assertFalse(any("SPD.TGT=150.0000" in c for c in sent_r1))

            # C3: fresh flat steady P-only droop at y=80 when base=100 -> allowed after settle_cap
            w_r1.clear_plot()
            w_r1.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n100,80,200\n", 5.0)
            self.assertTrue(w_r1.autotuner.start(0, 1, 0, base=100.0, target=150.0, step_seconds=0.8, max_rounds=4))
            w_r1.autotuner.tick(5.01); w_r1.autotuner.tick(5.02); w_r1.autotuner.tick(5.03)
            for i in range(16):
                w_r1.sample(5.05 + i * 0.06, [100.0, 80.0, 200.0])
            sent_r1.clear()
            w_r1.autotuner.tick(6.00)
            self.assertEqual(w_r1.autotuner.state, w_r1.autotuner.WAIT_STEP_ACK)
            self.assertAlmostEqual(w_r1.autotuner.base_y, 80.0, places=3)
            self.assertTrue(any("SPD.TGT=150.0000" in c for c in sent_r1))
            w_r1.close()

            # D) Stream keeps sending target-only frames (speed feedback column disappears) -> MUST @STOP & NOT advance
            w_drop = Window(Path(tmp) / "missing_speed_col.ini")
            w_drop.timer.stop()
            w_drop.connected = True
            w_drop.sim = None
            w_drop.group.setCurrentIndex(1)
            for k, v in (("KP", 2.0), ("KI", 5.0), ("KD", 0.0), ("TGT", 10.0)):
                w_drop.fields[k].setValue(v)
                w_drop.confirmed[f"SPD.{k}"] = v
            sent_d = []
            w_drop.commands.submit = lambda cmds, quiet=False: (sent_d.extend(cmds) or True)
            w_drop.commands.busy = lambda: False
            w_drop.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n0,0,0\n", 1.0)
            self.assertTrue(
                w_drop.autotuner.start(0, 1, 0, base=0.0, target=200.0, step_seconds=0.8, max_rounds=4, hold_target=True)
            )
            w_drop.autotuner.tick(1.01)  # WAIT_START -> PREPARE
            w_drop.autotuner.tick(1.02)  # PREPARE (R1) -> WAIT_BASE_ACK
            w_drop.autotuner.tick(1.03)  # WAIT_BASE_ACK -> SETTLE_BASE
            for i in range(10):
                w_drop.sample(1.05 + i * 0.04, [0.0, 0.0, 0.0])
            w_drop.autotuner.tick(1.45)  # SETTLE_BASE -> WAIT_STEP_ACK
            w_drop.autotuner.tick(1.46)  # WAIT_STEP_ACK -> OBSERVE (R1)
            for i in range(26):
                t_s = 1.50 + i * 0.04
                if i == 10:
                    w_drop.sample(t_s, [200.0])  # E) single dropped speed frame must be tolerated
                else:
                    y_s = 200.0 * (1.0 - np.exp(-max(0.0, t_s - 1.50) / 0.18))
                    w_drop.sample(t_s, [200.0, float(y_s), 450.0])
            w_drop.autotuner.tick(2.60)
            self.assertIsNotNone(w_drop.autotuner.engine.best_params)
            self.assertEqual(w_drop.autotuner.state, w_drop.autotuner.PREPARE)

            # Start Round 2 normally, then stream ONLY target=200 for 30 frames (actual col becomes NaN)
            w_drop.autotuner.tick(2.61)  # PREPARE (R2) -> WAIT_BASE_ACK
            w_drop.autotuner.tick(2.62)  # WAIT_BASE_ACK -> SETTLE_BASE
            for i in range(10):
                w_drop.sample(2.65 + i * 0.04, [0.0, 0.0, 0.0])
            w_drop.autotuner.tick(3.05)  # SETTLE_BASE -> WAIT_STEP_ACK
            w_drop.autotuner.tick(3.06)  # WAIT_STEP_ACK -> OBSERVE (R2)
            sent_d.clear()
            for i in range(30):
                w_drop.sample(3.10 + i * 0.04, [200.0])
            w_drop.autotuner.tick(4.40)
            self.assertFalse(w_drop.autotuner.active)
            self.assertEqual(w_drop.autotuner.state, w_drop.autotuner.IDLE)
            self.assertEqual(w_drop.autotuner.engine.round, 2)
            self.assertEqual(sent_d[0], "@STOP")
            self.assertTrue(any("SPD.KP=2.0000" in c for c in sent_d))
            self.assertTrue(any("SPD.TGT=10.0000" in c for c in sent_d))
            self.assertFalse(any("SPD.KP=" in c and "SPD.KP=2.0000" not in c for c in sent_d))
            self.assertFalse(any("SPD.TGT=200.0000" in c for c in sent_d))
            self.assertIn("反馈通道", w_drop.autotune_status.text())
            self.assertIn("此前已测最优参考", w_drop.autotune_status.text())
            w_drop.close()

            # F) Feedback column disappears BEFORE step is issued (3 frames at 80, then target-only) -> MUST @STOP first and NEVER send TGT=200
            w_pre = Window(Path(tmp) / "pre_step_missing_speed.ini")
            w_pre.timer.stop()
            w_pre.connected = True
            w_pre.sim = None
            w_pre.group.setCurrentIndex(1)
            for k, v in (("KP", 2.0), ("KI", 5.0), ("KD", 0.0), ("TGT", 10.0)):
                w_pre.fields[k].setValue(v)
                w_pre.confirmed[f"SPD.{k}"] = v
            sent_pre = []
            w_pre.commands.submit = lambda cmds, quiet=False: (sent_pre.extend(cmds) or True)
            w_pre.commands.busy = lambda: False
            w_pre.receive(b"#NAMES:target,actual,pwm\n#ROLES:target,actual,output\n100,80,200\n", 5.0)
            self.assertTrue(
                w_pre.autotuner.start(0, 1, 0, base=100.0, target=200.0, step_seconds=0.8, max_rounds=4, hold_target=True)
            )
            w_pre.autotuner.tick(5.01)  # WAIT_START -> PREPARE
            w_pre.autotuner.tick(5.02)  # PREPARE -> WAIT_BASE_ACK
            w_pre.autotuner.tick(5.03)  # WAIT_BASE_ACK -> SETTLE_BASE
            sent_pre.clear()
            # 3 valid frames at 80, then 3 target-only frames -> must NOT jump to TGT=200 using stale 3 frames
            base_t = w_pre.latest
            for i in range(3):
                w_pre.sample(base_t + 0.05 + i * 0.04, [100.0, 80.0, 200.0])
            for i in range(3):
                w_pre.sample(base_t + 0.17 + i * 0.04, [100.0])
            w_pre.autotuner.tick(5.30)
            self.assertEqual(w_pre.autotuner.state, w_pre.autotuner.SETTLE_BASE)
            self.assertEqual(sent_pre, [])
            # Continue target-only frames past timeout -> MUST abort with @STOP first, never emit TGT=200
            for i in range(20):
                w_pre.sample(base_t + 0.29 + i * 0.04, [100.0])
            w_pre.autotuner.tick(6.10)
            self.assertFalse(w_pre.autotuner.active)
            self.assertEqual(w_pre.autotuner.state, w_pre.autotuner.IDLE)
            self.assertTrue(sent_pre and sent_pre[0] == "@STOP")
            self.assertFalse(any("SPD.TGT=200.0000" in c for c in sent_pre))
            self.assertTrue(any("SPD.KP=2.0000" in c for c in sent_pre))
            self.assertTrue(any("SPD.TGT=10.0000" in c for c in sent_pre))
            self.assertIn("基准段", w_pre.autotune_status.text())
            self.assertIn("反馈通道", w_pre.autotune_status.text())
            w_pre.close()


if __name__ == "__main__":
    unittest.main()

