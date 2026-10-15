"""Exercise the real Tk key/capture/reset event path on the VNC display.

Run through mujoco-gpu --validation; generates a reviewable console image.
"""
from pathlib import Path
import os
import sys
import time
import threading
import unittest
import numpy as np

os.environ.setdefault("MUJOCO_GL", "egl")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from PIL import ImageGrab
from balance_lab.firmware.operator_console import OperatorInput, OperatorRenderer, OperatorWindow, telemetry
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.operator_worker import OperatorWorker


class OperatorWindowTests(unittest.TestCase):
    def test_default_help_hidden_text_reticle_resize_and_fullscreen(self):
        sim = VehicleSimulation(interface_mapping="coherent_candidate", controller_profile="jump46")
        renderer = OperatorRenderer(sim, width=800, height=450)
        window = OperatorWindow(sim, renderer)
        def wait_for(check):
            end = time.monotonic()+5
            while not check():
                self.assertLess(time.monotonic(), end, "window transition timed out")
                window.root.update()
                time.sleep(.005)
        try:
            window.root.update()
            window.root.focus_force()
            wait_for(lambda: window.frame_image is not None)
            self.assertTrue(window.state.show_help)
            self.assertEqual(window.root.winfo_children(), [window.canvas])
            native_size = (renderer.renderer.width, renderer.renderer.height)
            raw = np.asarray(window.scene_image).copy()
            visible = np.asarray(window.frame_image).copy()
            self.assertGreater(np.count_nonzero(visible[:160]-raw[:160]), 100)
            # Real key edge: H hides labels and help, preserving the image
            # everywhere except the small center reticle, even with an error.
            window.root.event_generate("<KeyPress>", keysym="h")
            window.root.update()
            window.root.event_generate("<KeyPress>", keysym="h")
            window.root.update()
            self.assertFalse(window.state.show_help)
            wait_for(lambda: not window.dirty)
            window.set_status("test error that must also be hidden", error=True)
            raw = np.asarray(window.scene_image)
            hidden = np.asarray(window.frame_image)
            changed = np.any(hidden != raw, axis=2)
            reticle = np.zeros(changed.shape, dtype=bool)
            cx,cy = hidden.shape[1]//2,hidden.shape[0]//2
            reticle[cy-20:cy+21,cx-20:cx+21] = True
            self.assertFalse(np.any(changed & ~reticle))
            self.assertTrue(np.any(changed & reticle))
            window.root.event_generate("<KeyRelease>", keysym="h")
            wait_for(lambda: "h" not in window.state.keys)
            window.root.event_generate("<KeyPress>", keysym="h")
            window.root.update()
            self.assertTrue(window.state.show_help)
            window.error_text = None
            window.state.release()
            window.action("f11")
            wait_for(lambda: bool(window.root.attributes("-fullscreen")) and
                     window.canvas.winfo_height() > 450)
            # Fullscreen covers the current monitor, not the combined X11
            # desktop when this workstation has multiple attached screens.
            self.assertEqual(window.canvas.winfo_width(), window.root.winfo_width())
            self.assertEqual(window.canvas.winfo_height(), window.root.winfo_height())
            self.assertEqual((renderer.renderer.width,renderer.renderer.height), native_size)
            window.root.event_generate("<KeyPress>", keysym="Escape")
            window.root.update()
            wait_for(lambda: not window.root.attributes("-fullscreen"))
            self.assertFalse(window.state.fullscreen)
            self.assertTrue(window.state.paused)
            # A non-16:9 window letterboxes and keeps the reticle centered.
            window.root.geometry("900x600")
            wait_for(lambda: window.canvas.winfo_width()==900 and window.canvas.winfo_height()==600)
            wait_for(lambda: not window.dirty)
            self.assertEqual(window.frame_image.size, (900,506))
            self.assertEqual(window.canvas.coords(window.image_item), [450,300])
            self.assertEqual((renderer.renderer.width,renderer.renderer.height), native_size)
        finally:
            window.close()
            renderer.close()
            sim.close()

    def test_third_person_orbit_is_independent_of_real_gimbal_motion(self):
        sim = VehicleSimulation(interface_mapping="coherent_candidate", controller_profile="jump46")
        renderer = OperatorRenderer(sim)
        window = OperatorWindow(sim, renderer, fps=25)
        control = OperatorInput(paused=False)
        try:
            for _ in range(5000):
                sim.step(control.command(sim.ticks*sim.tick_s))
            window.root.update()
            renderer.render(window.state, telemetry(sim))
            before = np.array(renderer.last_view["forward"])
            gun_before = sim.data.xmat[sim.gimbal].reshape(3, 3)[:, 0].copy()
            control.keys = {"d"}
            control.begin_frame(.04)
            for _ in range(700):
                sim.step(control.command(sim.ticks*sim.tick_s))
            renderer.render(window.state, telemetry(sim))
            np.testing.assert_allclose(renderer.last_view["forward"], before, atol=1e-6)
            np.testing.assert_allclose(renderer.last_view["lookat"],
                                       sim.data.xpos[sim.base]+[0, 0, .12], atol=1e-6)
            gun_after = sim.data.xmat[sim.gimbal].reshape(3, 3)[:, 0]
            self.assertLess(np.dot(gun_before, gun_after), np.cos(np.radians(10)))

            # A left drag orbits while paused and sends no gun increment.
            # Match the real X11 pointer to the press, otherwise grab_set()
            # can deliver a queued real-pointer move after synthetic events.
            window.canvas.event_generate("<Motion>", x=480, y=270, warp=True)
            window.root.update()
            window.state.yaw_raw = 123  # An independently held gun request.
            window.canvas.event_generate("<Button-1>", x=480, y=270)
            window.canvas.event_generate("<Motion>", x=520, y=250, warp=True)
            window.root.update()
            self.assertTrue(window.captured)
            self.assertEqual(window.drag_button, 1)
            self.assertAlmostEqual(window.state.camera_azimuth_deg, 350)
            self.assertAlmostEqual(window.state.camera_elevation_deg, -15)
            self.assertEqual(window.state.mouse_dx, 0)
            self.assertEqual(window.state.yaw_raw, 123)
            window.root.event_generate("<ButtonRelease-1>", x=5, y=5)
            window.root.update()
            self.assertFalse(window.captured)
            self.assertEqual(window.state.yaw_raw, 123)
            window.state.yaw_raw = 0
            renderer.render(window.state, telemetry(sim))
            self.assertLess(np.dot(before, renderer.last_view["forward"]), .999)
            orbit = (window.state.camera_azimuth_deg, window.state.camera_elevation_deg)

            # An unrelated left release cannot end an active right drag.
            window.canvas.event_generate("<Motion>", x=480, y=270, warp=True)
            window.root.update()
            window.canvas.event_generate("<Button-3>", x=480, y=270)
            window.root.event_generate("<ButtonRelease-1>", x=5, y=5)
            window.root.update()
            self.assertTrue(window.captured)
            self.assertEqual(window.drag_button, 3)
            window.root.event_generate("<ButtonRelease-3>", x=5, y=5)
            window.root.update()
            self.assertFalse(window.captured)
            self.assertEqual(orbit, (window.state.camera_azimuth_deg, window.state.camera_elevation_deg))

            window.state.camera = "first"
            renderer.render(window.state, telemetry(sim))
            self.assertGreater(np.dot(renderer.last_view["forward"], gun_after), .9)
            window.canvas.event_generate("<Button-1>", x=480, y=270)
            window.canvas.event_generate("<Motion>", x=520, y=250)
            window.root.update()
            self.assertFalse(window.captured)
            window.state.camera = "third"
            renderer.render(window.state, telemetry(sim))
            self.assertEqual(orbit, (window.state.camera_azimuth_deg, window.state.camera_elevation_deg))
        finally:
            window.close()
            renderer.close()
            sim.close()

    def test_busy_physics_keeps_buttons_and_mouse_input_responsive(self):
        sim = VehicleSimulation(interface_mapping="coherent_candidate", controller_profile="jump46")
        renderer = OperatorRenderer(sim)
        window = OperatorWindow(sim, renderer, fps=25)
        window.worker = OperatorWorker(sim, window.state)
        window.root.update()
        released = threading.Event()
        release_guard = threading.Lock()
        pressed = []
        try:
            # Hold the same lock a slow physical batch owns. A bounded backup
            # release also lets a regression fail without hanging the suite.
            window.worker.lock.acquire()
            def release_lock():
                with release_guard:
                    if not released.is_set():
                        released.set()
                        window.worker.lock.release()
            def backup_release():
                if not released.wait(.5):
                    release_lock()
            backup = threading.Thread(target=backup_release)
            backup.start()
            try:
                window.state.paused = False
                window.state.mouse_dx = 25
                began = time.monotonic()
                window.frame()
                self.assertLess(time.monotonic()-began, .1, "Tk waited for physical stepping")
                self.assertEqual(window.state.mouse_dx, 25, "busy frames must retain mouse input")
                self.assertTrue(window.worker.snapshot_requested.is_set())
                window.root.after(5, lambda: (pressed.append(True), window.action("p")))
                deadline = time.monotonic()+.15
                while not pressed and time.monotonic() < deadline:
                    window.root.update()
                    time.sleep(.002)
                self.assertTrue(pressed, "pause button was starved by a busy physical batch")
                self.assertTrue(window.state.paused)
            finally:
                release_lock()
                backup.join()
        finally:
            window.close()
            renderer.close()
            sim.close()

    def test_real_events_capture_focus_reset_and_camera(self):
        sim=VehicleSimulation(interface_mapping="coherent_candidate",yaw_damping_nms_rad=0,
                              controller_profile="jump46")
        renderer=OperatorRenderer(sim)
        window=OperatorWindow(sim,renderer,fps=25)
        def pump_until(check, timeout=15):
            end=time.monotonic()+timeout
            while not check():
                self.assertLess(time.monotonic(),end,"UI condition timed out")
                window.root.update()
                time.sleep(.003)
        def key(keysym, down):
            window.root.event_generate("<KeyPress>" if down else "<KeyRelease>",keysym=keysym)
            window.root.update()
        try:
            window.root.geometry("+20+20")
            window.root.update()
            window.root.focus_force()
            pump_until(lambda:window.frame_image is not None)
            self.assertTrue(window.state.paused)
            key("p",True)
            key("p",True)
            self.assertFalse(window.state.paused,"held P must toggle once")
            key("p",False)
            pump_until(lambda:sim.ticks>=2000)
            self.assertEqual(sim.output.stand_state,2)
            self.assertEqual(sim.output.chassis_mode,5)
            def assert_view_faces_muzzle():
                muzzle=sim.data.xmat[sim.gimbal].reshape(3,3)[:,0]
                view=renderer.renderer.scene.camera[0].forward
                self.assertGreater(np.dot(view,muzzle),.9,"both operator views must face the CAD gun muzzle")
            assert_view_faces_muzzle()
            key("Control_L",True)
            key("w",True)
            self.assertEqual(window.state.command(2).rc_channels[1],132)
            pump_until(lambda:sim.ticks>=2200)
            key("w",False)
            key("Control_L",False)
            pump_until(lambda:"w" not in window.state.keys)
            key("Tab",True)
            key("Tab",False)
            self.assertEqual(window.state.camera,"first")
            pump_until(lambda:renderer.last_view is not None and "eye" in renderer.last_view)
            assert_view_faces_muzzle()
            # Ordinary clicks/motion leave the gun alone. Right drag uses
            # actual pointer deltas, with no VNC cursor warping or recapture.
            window.canvas.event_generate("<Button-1>",x=480,y=270)
            window.canvas.event_generate("<Motion>",x=500,y=270)
            window.root.update()
            self.assertFalse(window.captured)
            self.assertEqual(window.state.mouse_dx,0)
            window.canvas.event_generate("<Motion>",x=480,y=270,warp=True)
            window.root.update()
            window.canvas.event_generate("<Button-3>",x=480,y=270)
            window.root.update()
            self.assertTrue(window.captured)
            # Inspect queued mouse increments before a scheduled frame
            # legitimately consumes them; keep the X11 pointer aligned.
            window.canvas.event_generate("<Motion>",x=500,y=270,warp=True)
            self.assertEqual(window.state.mouse_dx,20)
            window.canvas.event_generate("<Motion>",x=505,y=275,warp=True)
            self.assertEqual(window.state.mouse_dx,25)
            self.assertAlmostEqual(window.state.camera_pitch_deg,-12.2)
            for _ in range(5):
                window.action("bracketleft")
            window.root.update()
            self.assertAlmostEqual(window.state.mouse_sensitivity_rad_px,.000375)
            pump_until(lambda:sim.ticks>=2400)
            self.assertTrue(all(abs(v)<660 for v in window.state.command(2.4).rc_channels[:4]))
            window.root.event_generate("<ButtonRelease-3>",x=5,y=5)
            window.root.update()
            self.assertFalse(window.captured)
            self.assertIsNone(window.pointer_last)
            self.assertFalse(window.state.paused,"ending a drag must not pause driving")
            self.assertEqual(window.state.mouse_dx,0)
            self.assertEqual(window.state.yaw_raw,0)
            window.canvas.event_generate("<Motion>",x=600,y=300)
            window.root.update()
            self.assertEqual(window.state.mouse_dx,0)
            window.canvas.event_generate("<Button-3>",x=600,y=300)
            window.root.update()
            key("Escape",True)
            key("Escape",False)
            self.assertTrue(window.state.paused)
            self.assertFalse(window.captured)
            self.assertFalse(window.state.keys)
            key("r",True)
            key("r",False)
            self.assertEqual(sim.ticks,0)
            self.assertEqual(sim.output.chassis_mode,0)
            self.assertTrue(window.state.paused)
            pump_until(lambda:window.dirty is False)
            # Focus loss pauses and drops every held input and mouse increment.
            window.state.paused=False
            window.state.keys={"w"}
            window.state.mouse_dx=20
            other=window.tk.Toplevel(window.root)
            other.geometry("1x1+0+0")
            other.update()
            other.focus_force()
            window.focus_out(None)
            window.root.update()
            self.assertTrue(window.state.paused)
            self.assertFalse(window.state.keys)
            self.assertEqual(window.state.mouse_dx,0)
            other.destroy()
            window.state.camera="third"
            window.dirty=True
            pump_until(lambda:not window.dirty)
            output=Path(__file__).resolve().parents[1]/"out/native/operator_ui_20261014"
            output.mkdir(parents=True,exist_ok=True)
            window.frame_image.save(output/"reset_frame.png")
            x,y=window.root.winfo_rootx(),window.root.winfo_rooty()
            ImageGrab.grab(bbox=(x,y,x+window.root.winfo_width(),y+window.root.winfo_height())).save(output/"console.png")
        finally:
            window.close()
            renderer.close()
            sim.close()


if __name__=="__main__":
    unittest.main(verbosity=2)
