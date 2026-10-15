"""RM key/mouse front end; the original C firmware owns every control loop.

The NVIDIA renderer produces one bounded frame at a time. Tk displays pixels, so VNC does
not need an OpenGL context. Inputs emulate DR16 switches/sticks rather than
writing poses, velocities or actuator commands.
"""
from __future__ import annotations

import argparse
import copy
from contextlib import nullcontext
from dataclasses import dataclass, field
from functools import lru_cache
import json
from pathlib import Path
import re
import resource
import time

import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from ..plant import ARENA_VARIANTS, VARIANTS
from .build import VEHICLE
from .runtime import Command
from .simulation import VehicleSimulation

MODE_NAMES = {0: "松控", 1: "初始化", 2: "开环", 3: "自起身", 4: "跳跃", 5: "跟随云台", 6: "小陀螺", 7: "自动"}
LEG_NAMES = {2: "低腿", 3: "中腿", 1: "磕台阶"}
FONT_PATH = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"


def firmware_input_contract():
    """Read display/input scaling from the same snapshot compiled by VehicleRuntime."""
    config = (VEHICLE / "src/rm_config.h").read_text()
    chassis = (VEHICLE / "src/task/chassis/control/chassis_control.c").read_text()
    gimbal = (VEHICLE / "src/task/gimbal/gimbal_task.c").read_text()

    def number(source, name):
        match = re.search(r"^\s*#define\s+" + name + r"\s+([\d.]+)[fF]?\b", source, re.M)
        if not match:
            raise ValueError(f"missing numeric firmware input contract: {name}")
        return float(match[1])

    return {"rc_max": int(number(config, "RC_DBUS_MAX_VALUE")),
            "drive_limits_mps": [number(chassis, "CHASSIS_V_MAX_" + level) for level in ("LOW", "MID", "HIGH")],
            "gimbal_rate_rad_s": number(gimbal, "GIMBAL_YAW_MANUAL_RATE_MAX_RADPS")}


@dataclass
class OperatorInput:
    rc_max: int = 660
    gimbal_rate_rad_s: float = 5.3
    # Positive C vx moves toward the CAD gun muzzle. The raw chassis IMU
    # forward axis points the other way; it is not the operator view axis.
    drive_sign: int = 1
    keys: set[str] = field(default_factory=set)
    paused: bool = True
    enabled: bool = True
    spin: bool = False
    leg_switch: int = 2
    camera: str = "third"
    camera_pitch_deg: float = -12.0
    camera_distance: float = 2.0
    camera_azimuth_deg: float = 0.0
    camera_elevation_deg: float = -20.0
    show_help: bool = True
    fullscreen: bool = False
    jump_until: float = -1.0
    mouse_dx: float = 0.0
    mouse_sensitivity_rad_px: float = 0.00075
    yaw_raw: int = 0

    def release(self):
        self.keys.clear()
        self.mouse_dx = 0.0
        self.yaw_raw = 0
        self.jump_until = -1.0

    def press(self, key, elapsed):
        key = key.lower()
        if key in self.keys:
            return None
        self.keys.add(key)
        if key == "p":
            self.paused = not self.paused
            self.release()
            self.keys.add(key)
        elif key == "r":
            self.release()
            self.paused = True
            self.enabled, self.spin, self.leg_switch = True, False, 2
            self.keys.add(key)
            return "reset"
        elif key == "m":
            self.camera = "third" if self.camera == "overview" else "overview"
        elif key == "tab":
            self.camera = "first" if self.camera == "third" else "third"
        elif key == "h":
            self.show_help = not self.show_help
        elif key == "f11":
            self.fullscreen = not self.fullscreen
            return "fullscreen"
        elif key in ("bracketleft", "bracketright"):
            multiplier = self.mouse_sensitivity_rad_px / .00075
            multiplier = round(multiplier + (-.1 if key == "bracketleft" else .1), 2)
            self.mouse_sensitivity_rad_px = .00075 * float(np.clip(multiplier, .25, 2))
        elif key == "e":
            self.spin = True
        elif key == "q":
            self.spin = False
        elif key in ("z", "x", "c", "b"):
            self.leg_switch = {"z": 2, "x": 3, "c": 1, "b": 1}[key]
        elif key == "space" and not self.paused and self.enabled:
            # A real wheel edge. DR16 rejects recovery/spin/bump requests and
            # owns acknowledgement, centering, cooldown and jump readiness.
            self.jump_until = elapsed + 0.050
        elif key == "backspace":
            self.enabled = False
            self.release()
            self.keys.add(key)
        elif key == "return":
            self.enabled = True
        return None

    def begin_frame(self, seconds):
        # Pointer motion is an angular increment spread across this frame's
        # explicit firmware ticks. Discard excess motion beyond the C rate
        # envelope instead of buffering a long delayed turn.
        mouse_rate = self.mouse_dx * self.mouse_sensitivity_rad_px / seconds
        keyboard = (int("d" in self.keys) - int("a" in self.keys)) * 0.24
        self.yaw_raw = int(np.clip(mouse_rate / self.gimbal_rate_rad_s + keyboard, -1, 1) * self.rc_max)
        self.mouse_dx = 0.0

    def command(self, elapsed):
        if self.paused:
            return Command.remote(right_switch=1, left_switch=self.leg_switch)
        gain = 1.0 if "shift" in self.keys else 0.20 if "control" in self.keys else 0.55
        forward = int("w" in self.keys) - int("s" in self.keys)
        vx = int(self.drive_sign * forward * gain * self.rc_max)
        return Command.remote(channels=(0, vx, self.yaw_raw, 0, 660 if elapsed < self.jump_until else 0),
                              right_switch=1 if not self.enabled else 2 if self.spin else 3,
                              left_switch=self.leg_switch)


def telemetry(sim):
    out = sim.output
    frame = sim.data.xmat[sim.base].reshape(3, 3) @ sim.sensor_body
    world_velocity = sim._velocity(sim.base)[3:]
    muzzle = sim.data.xmat[sim.gimbal].reshape(3, 3)[:, 0]
    # Page39 is the original C's live limited velocity reference and limit.
    # Its recovery branch has a different ABI; do not read that as speed.
    page = sim.runtime.debug_page(39) if out.chassis_mode in (2, 4, 5, 6) else None
    body_contact_force = 0.0
    if sim.variant in ARENA_VARIANTS:
        wheels = {mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, name)
                  for name in ("l_wheel", "r_wheel")}
        force = np.empty(6)
        for index, contact in enumerate(sim.data.contact):
            bodies = {int(sim.model.geom_bodyid[g]) for g in contact.geom}
            if 0 in bodies and not bodies.intersection(wheels):
                mujoco.mj_contactForce(sim.model, sim.data, index, force)
                body_contact_force += max(0.0, float(force[0]))
    return {"time_s": sim.ticks * sim.tick_s,
            "speed_mps": -float(np.dot(world_velocity, frame[:, 1])),
            "speed_ref_mps": float(page[3]) if page else None,
            "speed_limit_mps": float(page[4]) if page else None,
            "pitch_deg": float(out.imu[1]), "roll_deg": float(out.imu[0]),
            "yaw_rate_rad_s": float(out.imu[5]), "relative_deg": float(np.degrees(out.yaw_relative)),
            "gimbal_heading_deg": float(np.degrees(np.arctan2(muzzle[1], muzzle[0]))),
            "length_m": list(out.length), "mode": int(out.chassis_mode),
            "stand": int(out.stand_state), "recovery": int(out.recovery_state),
            "recovery_fault": int(out.recovery_fault), "jump": int(out.jump_state),
            "jump_fault": int(out.jump_fault), "bump": int(out.bump_state),
            "body_contact_force_n": body_contact_force,
            "gimbal_ready": int(out.gimbal_ready)}


@lru_cache(maxsize=2)
def overlay_font(size):
    return ImageFont.truetype(FONT_PATH, size)


@lru_cache(maxsize=32)
def help_overlay(size, sensitivity):
    font = overlay_font(size)
    text = (
        "H  显示 / 隐藏文字    F11  全屏 / 窗口\n"
        "W / S  前后    Shift / Ctrl  满速 / 慢速\n"
        "Space  跳跃    Z / X  低 / 中腿    C / B  磕台阶\n"
        f"右键拖动 / A、D  转云台    [ / ]  灵敏度 {sensitivity:.2f}×\n"
        "左键拖动  第三人称视角    滚轮  缩放\n"
        "Tab  第一 / 第三人称    M  全场俯视\n"
        "E / Q  小陀螺 / 跟随    Backspace / Enter  松控 / 使能\n"
        "P  运行 / 暂停    R  重置    Esc  暂停并退出全屏"
    )
    bounds = ImageDraw.Draw(Image.new("RGBA", (1, 1))).multiline_textbbox((2, 2), text, font=font, spacing=4, stroke_width=2)
    layer = Image.new("RGBA", (bounds[2]+3, bounds[3]+3))
    ImageDraw.Draw(layer).multiline_text((2, 2), text, font=font, spacing=4,
        fill="#edf4ff", stroke_width=2, stroke_fill=(8, 14, 20, 220))
    return layer


def draw_operator_overlay(frame, state, info, realtime=1.0, error=None):
    """Plain text on the viewport; H hides everything except the reticle."""
    draw = ImageDraw.Draw(frame, "RGBA")
    w, h = frame.size
    if state.show_help:
        size = 13 if h < 450 else 16
        font = overlay_font(size)
        text_options = dict(font=font, fill="#edf4ff", stroke_width=2, stroke_fill=(8, 14, 20, 220))
        layer = help_overlay(size, round(state.mouse_sensitivity_rad_px / .00075, 2))
        frame.paste(layer, (14, 10), layer)
        camera = {"first": "第一人称", "third": "第三人称", "overview": "全场俯视"}[state.camera]
        mode = MODE_NAMES.get(info["mode"], str(info["mode"]))
        status = "已暂停 · P 开始" if state.paused else f"运行中 · {realtime:.2f}×"
        line = f"{status}    {camera} · {mode}    {info['speed_mps']:+.2f} m/s    {LEG_NAMES[state.leg_switch]}"
        draw.text((16, h-32), line, **text_options)
        if error:
            draw.text((16, h-58), error, **{**text_options, "fill": "#ff9393"})
    cx, cy = w // 2, h // 2
    # Keep this in display pixels so fullscreen retains a small, sharp reticle.
    for a, b in (((cx-17,cy),(cx-6,cy)), ((cx+6,cy),(cx+17,cy)),
                 ((cx,cy-17),(cx,cy-6)), ((cx,cy+6),(cx,cy+17))):
        draw.line((a,b), fill=(8, 14, 20, 230), width=4)
        draw.line((a,b), fill="#70dfd1", width=2)
    draw.ellipse((cx-2,cy-2,cx+2,cy+2), fill="#70dfd1")
    return frame


class OperatorRenderer:
    def __init__(self, sim, width=960, height=540):
        self.source_sim = sim
        self.external_snapshot = False
        self.cull_tiles = True
        self.tiles = None
        if sim.variant in ARENA_VARIANTS:
            from terrain_rmuc2025.operator_view import load_operator_view
            view = copy.copy(sim)
            view.model = load_operator_view(sim.variant)
            if (view.model.nq, view.model.nv, view.model.nbody) != (sim.model.nq, sim.model.nv, sim.model.nbody):
                raise ValueError("display robot topology differs from the physical robot")
            view.data = mujoco.MjData(view.model)
            sim = view
            self.tiles = np.array([i for i in range(sim.model.ngeom)
                if (mujoco.mj_id2name(sim.model, mujoco.mjtObj.mjOBJ_GEOM, i) or "").startswith(sim.variant + "_visual_")])
        self.sim, self.width, self.height = sim, width, height
        sim.model.vis.global_.offwidth = width
        sim.model.vis.global_.offheight = height
        if sim.variant in ARENA_VARIANTS:
            # MuJoCo clip distances scale with the whole arena's extent.
            # Keep the nearby gun/robot visible in both operator views.
            sim.model.vis.map.znear = .005 / sim.model.stat.extent
            sim.model.vis.headlight.ambient[:] = (.35, .35, .35)
            sim.model.vis.headlight.diffuse[:] = (.6, .6, .6)
            sim.model.vis.headlight.specular[:] = (.1, .1, .1)
        self.renderer = mujoco.Renderer(sim.model, width=width, height=height,
                                        max_geom=max(10000, sim.model.ngeom + 100))
        from OpenGL import GL
        self.gpu = GL.glGetString(GL.GL_RENDERER).decode()
        vendor = GL.glGetString(GL.GL_VENDOR).decode()
        if "NVIDIA" not in vendor.upper():
            self.renderer.close()
            raise RuntimeError(f"Expected NVIDIA renderer; got {vendor} / {self.gpu}")
        self.option = mujoco.MjvOption()
        self.option.geomgroup[3] = 0  # CAD visuals; collision proxies stay in physics.
        if sim.variant in ("rmuc2026", "rmuc2027"):
            self.option.geomgroup[4] = 1  # User-visible outer walls and top rails.
        if self.tiles is not None:
            self.option.geomgroup[2] = 0  # out-of-view tiles in the display model only
        self.last_view = None

    def render(self, state, info, realtime=1.0, *, overlay=True):
        began = time.perf_counter()
        sim = self.sim
        if sim.model is not self.source_sim.model:
            if not self.external_snapshot:
                sim.data.qpos[:] = self.source_sim.data.qpos
                sim.data.qvel[:] = self.source_sim.data.qvel
                sim.data.time = self.source_sim.data.time
                sim.ticks = self.source_sim.ticks
            mujoco.mj_forward(sim.model, sim.data)
        rotation = sim.data.xmat[sim.gimbal].reshape(3, 3)
        # The sensor reference was constructed from the yaw mount offset.
        # CAD gun muzzle points opposite that reference (local +X). First
        # person uses this axis without changing original IMU feedback.
        forward = -(rotation @ sim.gimbal_forward)
        heading = np.degrees(np.arctan2(forward[1], forward[0]))
        cam = mujoco.MjvCamera()
        cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.lookat[:] = sim.data.xpos[sim.base]
        cam.lookat[2] += 0.12
        # Third-person follows translation while its orientation stays in
        # world coordinates. MuJoCo azimuth is its world look direction;
        # only an explicit left drag changes that view.
        cam.distance = state.camera_distance
        cam.azimuth, cam.elevation = state.camera_azimuth_deg, state.camera_elevation_deg
        if state.camera == "overview":
            cam.lookat[:] = (0, 0, .2)
            cam.distance, cam.azimuth, cam.elevation = 33, 90, -65
        if self.tiles is not None:
            if state.camera == "first":
                horizontal = np.array([forward[0], forward[1], 0.0])
                horizontal /= max(1e-9, np.linalg.norm(horizontal))
                elevation = np.radians(state.camera_pitch_deg)
                direction = horizontal*np.cos(elevation)+np.array([0, 0, np.sin(elevation)])
                eye = sim.data.xpos[sim.gimbal]+.24*forward+np.array([0, 0, .10])
            else:
                az, el = np.radians([cam.azimuth, cam.elevation])
                direction = np.array([np.cos(az)*np.cos(el), np.sin(az)*np.cos(el), np.sin(el)])
                eye = cam.lookat-cam.distance*direction
            right = np.cross(direction, [0, 0, 1])
            right /= max(1e-9, np.linalg.norm(right))
            up = np.cross(right, direction)
            delta = sim.data.geom_xpos[self.tiles]-eye
            depth = delta@direction
            radius = sim.model.geom_rbound[self.tiles]+.05
            vertical = np.tan(np.radians(sim.model.vis.global_.fovy)/2)
            horizontal = vertical*self.width/self.height
            visible = ((depth+radius > .005) &
                (np.abs(delta@right) < depth*horizontal+radius*np.sqrt(1+horizontal**2)) &
                (np.abs(delta@up) < depth*vertical+radius*np.sqrt(1+vertical**2)))
            if state.camera == "overview" or not self.cull_tiles:
                visible[:] = True
            sim.model.geom_group[self.tiles] = np.where(visible, 1, 2)
        self.renderer.update_scene(sim.data, camera=cam, scene_option=self.option)
        if sim.variant in ARENA_VARIANTS:
            # Match the arena preview's overhead light without recompiling or
            # changing physical geometry. The robot XML has no world lights.
            scene = self.renderer.scene
            light = scene.lights[scene.nlight]
            light.pos[:] = (0, 0, 12)
            light.dir[:] = (0, 0, -1)
            light.type = mujoco.mjtLightType.mjLIGHT_DIRECTIONAL
            light.id = -2
            light.texid = -1
            light.headlight = False
            light.castshadow = False
            light.attenuation[:] = (1, 0, 0)
            light.ambient[:] = (0, 0, 0)
            light.diffuse[:] = (.5, .5, .5)
            light.specular[:] = (0, 0, 0)
            scene.nlight += 1
        if state.camera == "first":
            # World-up, gunner view from the yaw head, beyond its front skin.
            # Vertical mouse motion only changes the view: pitch is not modeled.
            horizontal = np.array([forward[0], forward[1], 0.0])
            if np.linalg.norm(horizontal) < 1e-6:
                horizontal[:] = (-np.cos(sim.reference_heading), -np.sin(sim.reference_heading), 0)
            else:
                horizontal /= np.linalg.norm(horizontal)
            elevation = np.radians(state.camera_pitch_deg)
            direction = horizontal * np.cos(elevation) + np.array([0, 0, np.sin(elevation)])
            eye = sim.data.xpos[sim.gimbal] + 0.24 * forward + np.array([0, 0, 0.10])
            right = np.cross(direction, [0, 0, 1])
            up = np.cross(right, direction)
            up /= np.linalg.norm(up)
            for camera in self.renderer.scene.camera:
                camera.pos[:] = eye
                camera.forward[:] = direction
                camera.up[:] = up
            self.last_view = {"eye": eye.tolist(), "forward": direction.tolist(), "up": up.tolist()}
        else:
            self.last_view = {"heading_deg": float(cam.azimuth), "lookat": cam.lookat.tolist(),
                              "forward": self.renderer.scene.camera[0].forward.tolist()}
        prepared = time.perf_counter()
        frame = Image.fromarray(self.renderer.render())
        rasterized = time.perf_counter()
        if overlay:
            draw_operator_overlay(frame, state, info, realtime)
        self.last_timings = {"scene_ms": (prepared-began)*1000,
                             "gpu_ms": (rasterized-prepared)*1000,
                             "hud_ms": (time.perf_counter()-rasterized)*1000}
        return frame

    def close(self):
        self.renderer.close()


class OperatorWindow:
    def __init__(self, sim, renderer, fps=25, *, fullscreen=False):
        import tkinter as tk
        from PIL import ImageTk
        self.tk, self.ImageTk = tk, ImageTk
        self.sim, self.renderer, self.fps = sim, renderer, fps
        self.contract = firmware_input_contract()
        self.state = OperatorInput(rc_max=self.contract["rc_max"], gimbal_rate_rad_s=self.contract["gimbal_rate_rad_s"])
        self.root = tk.Tk()
        self.root.title("RM 操控")
        if sim.variant in ARENA_VARIANTS:
            self.root.title(self.root.title() + f" · RMUC {sim.variant[4:]}")
        self.root.configure(bg="#0c1520")
        self.root.resizable(True, True)
        self.root.minsize(640, 360)
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.closed, self.captured, self.dirty = False, False, True
        self.pointer_last = None
        self.drag_button = None
        self.last_guard, self.last_wall = time.monotonic(), time.monotonic()
        self.realtime = 1.0
        self._tick_budget = 0.0
        self.released_jobs = {}
        self.image = None
        self.frame_image = None
        self.scene_image = None
        self.last_info = None
        self.error_text = None
        self.windowed_geometry = f"{renderer.width}x{renderer.height}"
        self.root.geometry(self.windowed_geometry)
        self.canvas = tk.Canvas(self.root, width=renderer.width, height=renderer.height, highlightthickness=0, bg="#111d29")
        self.canvas.pack(fill="both", expand=True)
        self.image_item = self.canvas.create_image(0, 0, anchor="center")
        self.canvas.bind("<Configure>", lambda _event: setattr(self, "dirty", True))
        self.root.bind("<KeyPress>", self.key_press)
        self.root.bind("<KeyRelease>", self.key_release)
        self.root.bind("<FocusOut>", self.focus_out)
        self.canvas.bind("<Button-1>", lambda e: self.capture(e, button=1))
        self.canvas.bind("<Button-3>", self.capture)
        self.root.bind("<ButtonRelease-1>", lambda e: self.release_mouse(button=1))
        self.root.bind("<ButtonRelease-3>", lambda e: self.release_mouse(button=3))
        self.canvas.bind("<Motion>", self.motion)
        self.canvas.bind("<Button-4>", lambda e: self.zoom(-0.1))
        self.canvas.bind("<Button-5>", lambda e: self.zoom(0.1))
        self.worker = None
        if sim.variant in ARENA_VARIANTS:
            from .operator_worker import OperatorWorker
            self.worker = OperatorWorker(sim, self.state, view=renderer.sim)
            renderer.external_snapshot = True
            renderer.sim = self.worker.view
        self.root.after(20, self.frame)
        if fullscreen:
            self.state.fullscreen = True
            self.root.after_idle(self.apply_fullscreen)

    def present_frame(self):
        if self.scene_image is None:
            return
        width, height = self.canvas.winfo_width(), self.canvas.winfo_height()
        scale = min(width/self.renderer.width, height/self.renderer.height)
        size = (max(1, round(self.renderer.width*scale)), max(1, round(self.renderer.height*scale)))
        # Resize only the displayed pixels. The GPU target and 1 ms physics
        # remain bounded; aspect ratio, camera and mouse deltas stay intact.
        frame = self.scene_image.resize(size, Image.Resampling.BILINEAR)
        self.frame_image = draw_operator_overlay(frame, self.state, self.last_info, self.realtime, self.error_text)
        self.image = self.ImageTk.PhotoImage(self.frame_image)
        self.canvas.itemconfigure(self.image_item, image=self.image)
        self.canvas.coords(self.image_item, width/2, height/2)

    def apply_fullscreen(self):
        if self.state.fullscreen:
            if self.root.winfo_width() > 1:
                self.windowed_geometry = self.root.geometry()
            self.root.attributes("-fullscreen", True)
        else:
            self.root.attributes("-fullscreen", False)
            if self.windowed_geometry:
                self.root.geometry(self.windowed_geometry)
        self.dirty = True

    def set_status(self, text, error=False):
        self.status_text = (text, error)
        self.error_text = text if error else None
        self.present_frame()

    @staticmethod
    def canonical(key):
        return "shift" if key.startswith("Shift") else "control" if key.startswith("Control") else key.lower()

    def action(self, key):
        event = self.state.press(key, self.sim.ticks*self.sim.tick_s)
        self.state.keys.discard(key)
        self.after_action(event)

    def after_action(self, event):
        if event == "fullscreen":
            self.apply_fullscreen()
        if event == "reset":
            with self.worker.lock if self.worker else nullcontext():
                self.sim.reset("encoder")
            self.contract = firmware_input_contract()
            self.state.rc_max = self.contract["rc_max"]
            self.state.gimbal_rate_rad_s = self.contract["gimbal_rate_rad_s"]
            self.last_wall = time.monotonic()
            self._tick_budget = 0.0
            self.error_text = None
        self.dirty = True
        if self.state.paused or (self.drag_button == 1 and self.state.camera != "third"):
            self.release_mouse()

    def key_press(self, event):
        key = self.canonical(event.keysym)
        if key == "escape":
            if self.state.fullscreen:
                self.state.fullscreen = False
                self.apply_fullscreen()
            self.state.release()
            self.state.paused = True
            self.release_mouse()
            self.dirty = True
        else:
            # X11 autorepeat emits release/press pairs. Delayed releases keep
            # held inputs steady and prevent toggles from repeating.
            job = self.released_jobs.pop(key, None)
            if job:
                self.root.after_cancel(job)
            self.after_action(self.state.press(key, self.sim.ticks*self.sim.tick_s))
        return "break"

    def key_release(self, event):
        key = self.canonical(event.keysym)
        old = self.released_jobs.pop(key, None)
        if old:
            self.root.after_cancel(old)
        def release():
            self.state.keys.discard(key)
            self.released_jobs.pop(key, None)
        self.released_jobs[key] = self.root.after(15, release)
        return "break"

    def focus_out(self, event):
        def check():
            focused = self.root.focus_displayof()
            if focused is None or focused.winfo_toplevel() != self.root:
                self.state.release()
                self.state.paused = True
                self.release_mouse()
                self.dirty = True
        self.root.after_idle(check)

    def capture(self, event, button=3):
        self.canvas.focus_set()
        if button == 1 and self.state.camera != "third":
            return
        if self.captured:
            self.release_mouse()
        self.captured = True
        self.drag_button = button
        self.pointer_last = (event.x_root, event.y_root)
        self.canvas.configure(cursor="fleur")
        self.canvas.grab_set()

    def release_mouse(self, button=None):
        if button is not None and button != self.drag_button:
            return
        gimbal_drag = self.drag_button == 3
        if self.captured:
            self.canvas.grab_release()
            self.canvas.configure(cursor="")
            self.captured = False
        self.pointer_last = None
        self.drag_button = None
        if gimbal_drag:
            self.state.mouse_dx = 0.0
            self.state.yaw_raw = 0

    def motion(self, event):
        if not self.captured:
            return
        current = (event.x_root, event.y_root)
        if self.pointer_last is None:
            self.pointer_last = current
            return
        dx, dy = current[0]-self.pointer_last[0], current[1]-self.pointer_last[1]
        self.pointer_last = current
        if dx == 0 and dy == 0:
            return
        if self.drag_button == 1:
            self.state.camera_azimuth_deg = (self.state.camera_azimuth_deg - dx*.25) % 360
            self.state.camera_elevation_deg = float(np.clip(
                self.state.camera_elevation_deg - dy*.25, -85, -5))
            self.dirty = True
            return
        if not self.state.paused:
            self.state.mouse_dx += dx
        view_sensitivity = 0.04 * self.state.mouse_sensitivity_rad_px / 0.00075
        self.state.camera_pitch_deg = float(np.clip(self.state.camera_pitch_deg - dy*view_sensitivity, -55, 35))
        self.dirty = True

    def zoom(self, delta):
        self.state.camera_distance = float(np.clip(self.state.camera_distance+delta, 0.8, 5))
        self.dirty = True

    def frame(self):
        if self.closed:
            return
        started = time.monotonic()
        snapshot_locked = False
        try:
            if started-self.last_guard > 2:
                available = int(next(x.split()[1] for x in Path("/proc/meminfo").read_text().splitlines() if x.startswith("MemAvailable:"))) / 1024
                if available < 2048:
                    raise RuntimeError("可用内存低于 2 GiB，已停止画面与仿真")
                self.last_guard = started
            if self.worker and (not self.state.paused or self.dirty):
                self.worker.snapshot_requested.set()
                if not self.worker.lock.acquire(blocking=False):
                    # Keep Tk's event loop alive while a physical batch is
                    # busy. Mouse deltas stay pending until this frame runs.
                    self.root.after(2, self.frame)
                    return
                snapshot_locked = True
                self.worker.snapshot_requested.clear()
            if not self.state.paused:
                # Distribute fractional ticks (30 FPS -> 33/33/34 ticks)
                # without changing the plant or firmware's 1 ms period.
                self._tick_budget += 1/self.fps/self.sim.tick_s
                ticks = int(self._tick_budget + 1e-9)
                self._tick_budget -= ticks
                input_seconds = max(self.sim.tick_s, started-self.last_wall) if self.worker else ticks*self.sim.tick_s
                self.state.begin_frame(input_seconds)
                if self.worker:
                    self.realtime = self.worker.realtime
                else:
                    for _ in range(ticks):
                        self.sim.step(self.state.command(self.sim.ticks*self.sim.tick_s))
                    self.realtime = ticks*self.sim.tick_s / max(1e-6, started-self.last_wall)
                self.dirty = True
            if self.worker and self.worker.error:
                raise RuntimeError(f"物理计算已停止：{self.worker.error}")
            if self.dirty:
                info = telemetry(self.sim)
                if self.worker:
                    self.worker.snapshot()
                    self.worker.lock.release()
                    snapshot_locked = False
                self.scene_image = self.renderer.render(self.state, info, self.realtime, overlay=False)
                self.last_info = info
                self.present_frame()
                self.dirty = False
        except Exception as exc:
            if snapshot_locked:
                self.worker.lock.release()
                snapshot_locked = False
            self.state.paused = True
            self.state.release()
            self.release_mouse()
            self.set_status(f"已停止：{exc}", error=True)
            print(f"Operator stopped: {exc}", flush=True)
            self.dirty = False
            self.root.after(200, self.frame)
            return
        finally:
            if snapshot_locked:
                self.worker.lock.release()
        self.last_wall = started
        self.root.after(max(1, round((1/self.fps-(time.monotonic()-started))*1000)), self.frame)

    def close(self):
        self.closed = True
        self.state.release()
        self.release_mouse()
        if self.worker:
            self.state.paused = True
            self.worker.close()
        self.root.destroy()

    def run(self):
        self.root.mainloop()


def demo(sim, renderer, directory, contract, fps):
    """Bounded scripted smoke/demo with the exact input and rendering path."""
    import imageio.v2 as imageio
    from .recording import Recorder, replay
    directory.mkdir(parents=True, exist_ok=False)
    state = OperatorInput(rc_max=contract["rc_max"], gimbal_rate_rad_s=contract["gimbal_rate_rad_s"], paused=False)
    recorder = Recorder(directory/"commands.jsonl", sim.metadata(), pages=(39,50,54))
    trace = []
    began = time.monotonic()
    try:
        with imageio.get_writer(directory/"operator.mp4", fps=fps, codec="libx264", pixelformat="yuv420p",
                                macro_block_size=2, ffmpeg_params=["-crf","22","-preset","fast","-threads","2","-movflags","+faststart"]) as writer:
            for index in range(10*fps):
                t = sim.ticks*sim.tick_s
                state.camera = "third" if t < 5.0 else "first"
                state.keys = {"w", "control"} if 3.0 <= t < 4.0 else {"s", "control"} if 4.3 <= t < 4.8 else {"d"} if 5.5 <= t < 6.0 else set()
                state.begin_frame(1/fps)
                for _ in range(round((index+1)/fps/sim.tick_s)-sim.ticks):
                    cmd = state.command(sim.ticks*sim.tick_s)
                    sim.step(cmd)
                    recorder.tick(sim,cmd)
                info = telemetry(sim)
                frame = renderer.render(state,info)
                writer.append_data(np.asarray(frame))
                if index in (round(2.5*fps), 7*fps):
                    frame.save(directory/(state.camera+"_person.png"))
                if index % fps == 0:
                    trace.append(info)
                    print(f"demo {index//fps+1}/10 s | {MODE_NAMES.get(info['mode'])} | pitch {info['pitch_deg']:+.2f}", flush=True)
        recorder.close()
        replay_result = replay(directory/"commands.jsonl")
        report = {"metadata": sim.metadata(), "input_contract": contract, "gpu": renderer.gpu,
                  "trace_1hz": trace, "final": telemetry(sim), "replay": replay_result,
                  "wall_seconds": time.monotonic()-began,
                  "peak_rss_mb": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024}
        (directory/"report.json").write_text(json.dumps(report, ensure_ascii=False, indent=2)+"\n")
        print(json.dumps(report["final"],ensure_ascii=False), flush=True)
    finally:
        if not recorder.file.closed:
            recorder.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--variant", choices=tuple(VARIANTS), default="fast")
    parser.add_argument("--plant", default="identified_2026-09-21")
    parser.add_argument("--interface-mapping", choices=("original","coherent_candidate"), default="coherent_candidate")
    parser.add_argument("--yaw-damping", type=float, default=0.0,
                        help="uncalibrated yaw bearing viscous damping in Nm s/rad; generic MJCF used 0.35")
    parser.add_argument("--fps", type=int, choices=(20,25,30), default=25)
    parser.add_argument("--demo-dir", type=Path, help="stream a 10-second GPU demo and replay every recorded C tick")
    parser.add_argument("--width", type=int, choices=(800,960), default=960)
    parser.add_argument("--fullscreen", action="store_true", help="open in fullscreen; F11 returns to a window")
    parser.add_argument("--controller-profile", choices=("jump46", "source"), default="jump46",
                        help="simulation-only jump profile, or the unchanged vehicle source")
    args = parser.parse_args()
    if args.variant in ARENA_VARIANTS:
        # A cold display compile needs temporary mesh/hull storage. Finish it
        # before allocating the full physical arena, then load cached models.
        import subprocess, sys
        subprocess.run([sys.executable, "-c",
                        "import sys; from terrain_rmuc2025.operator_view import load_operator_view; "
                        "load_operator_view(sys.argv[1], prepare_only=True)", args.variant], check=True)
    sim = VehicleSimulation(args.variant, plant=args.plant, interface_mapping=args.interface_mapping,
                            yaw_damping_nms_rad=args.yaw_damping,
                            controller_profile=None if args.controller_profile == "source" else args.controller_profile)
    renderer = None
    try:
        contract = firmware_input_contract()
        renderer = OperatorRenderer(sim, args.width, args.width*9//16)
        print(f"RM operator | GPU {renderer.gpu} | mapping {sim.interface_mapping} | C limits {contract['drive_limits_mps']}", flush=True)
        if args.demo_dir:
            demo(sim, renderer, args.demo_dir, contract, args.fps)
        else:
            OperatorWindow(sim, renderer, args.fps, fullscreen=args.fullscreen).run()
    finally:
        if renderer:
            renderer.close()
        sim.close()


if __name__ == "__main__":
    main()
