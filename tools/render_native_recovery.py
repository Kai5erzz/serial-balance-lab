"""Stream a GPU-rendered diagnostic video of the shared vehicle C recovery.

Run through rl-serial-balance-jointpd/scripts/heavy_run.sh. Frames are encoded
as they are rendered; no rollout-sized image buffer is retained in memory.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import subprocess
import sys
import time

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import imageio.v2 as imageio
import mujoco
import numpy as np
from OpenGL import GL
from PIL import Image, ImageDraw, ImageFont

from balance_lab.firmware import Command
from balance_lab.firmware.recording import Recorder
from balance_lab.firmware.simulation import VehicleSimulation
from balance_lab.firmware.viewer import summary
from balance_lab.plant import VARIANTS
from tools.probes.native_friction_audit import contacts
from tools.probes.native_wheel_handoff_audit import CASES, WheelHandoffSimulation
from tools.probes.native_yaw_follow_audit import FollowDiagnosticSimulation


def available_mb():
    for line in Path("/proc/meminfo").read_text().splitlines():
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 1024
    raise RuntimeError("MemAvailable is unavailable")


def gpu_free_mb():
    result = subprocess.run(["nvidia-smi", "--query-gpu=memory.free", "--format=csv,noheader,nounits",
                             "--id=0"], capture_output=True, text=True, check=True)
    return int(result.stdout.strip())


def camera(sim, kind, fixed=False):
    frame = sim.data.xmat[sim.base].reshape(3, 3) @ sim.sensor_body
    heading = np.degrees(np.arctan2(frame[1, 1], frame[0, 1]))
    if fixed:
        heading = np.degrees(sim.reference_heading)
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    # Fixed yaw exposes body spin; both modes follow position and world up.
    cam.lookat[:] = sim.data.xpos[sim.base]
    cam.lookat[2] = .25
    cam.distance = 1.25
    cam.azimuth = heading + (90 if kind == "side" else 0)
    cam.elevation = -10
    return cam


def compose(renderer, sim, font, small_font, contact, initial=False, fixed=False):
    canvas = Image.new("RGB", (1280, 720), (17, 23, 34))
    draw = ImageDraw.Draw(canvas)
    draw.text((22, 12), "共用实车 C 控制器 · 自起身诊断", font=font, fill="#F0F4FB")
    draw.text((22, 47), f"{sim.variant} | 精确零位 | 0.5 倍速 | 物理时间",
              font=small_font, fill="#9EB6D0")
    if getattr(sim, "wheel_case", "baseline") != "baseline":
        label = "两轮力矩与反馈同时反号" if sim.wheel_case == "both_neg_from_start" else sim.wheel_case
        if getattr(sim, "follow_case", "unchanged") == "angle_neg":
            label = "两轮反号 + 跟随角反号"
        elif getattr(sim, "follow_case", "unchanged") == "coherent_gimbal":
            label = "两轮反号 + yaw 齿轮完整映射"
        draw.text((750, 15), "轮接口实验："+label, font=small_font, fill="#FFD479")
    if not initial:
        draw.text((640, 47), f"yaw 速度 {sim.output.imu[5]:+.2f} rad/s | 跟随角 {np.degrees(sim.output.yaw_relative):+.1f}°",
                  font=small_font, fill="#FFD479")
    for index, kind in enumerate(("side", "front")):
        renderer.update_scene(sim.data, camera=camera(sim, kind, fixed))
        canvas.paste(Image.fromarray(renderer.render()), (640*index, 90))
        draw.rectangle((640*index+12, 100, 640*index+135, 136), fill="#111722")
        draw.text((640*index+22, 104), (("固定侧视", "固定前视") if fixed else ("侧面", "正面"))[index], font=font, fill="#F0F4FB")
    s = summary(sim)
    elapsed = sim.ticks * sim.tick_s
    stage = "初始姿态" if initial else f"恢复状态 {s['recovery_state']}"
    if not initial and s["recovery_state"] == 7:
        stage += " · LQR 接管"
    draw.text((22, 526), f"t={elapsed:.3f}s   {stage}   故障 {s['recovery_fault']}   站立确认 {int(s['stand_state']==2)}",
              font=font, fill="#F0F4FB")
    attitude = "INS 尚未启动" if initial else f"pitch {s['pitch_deg']:+.2f}°   roll {s['roll_deg']:+.2f}°"
    draw.text((22, 565), attitude, font=font, fill="#6BCBFF")
    draw.text((640, 565), f"轮电流指令 L/R {s['wheel_current'][0]:+.0f} / {s['wheel_current'][1]:+.0f}",
              font=font, fill="#FFD479")
    if not initial:
        draw.text((22, 604), f"腿长 L/R {s['leg_length_m'][0]:.3f} / {s['leg_length_m'][1]:.3f} m   phi0 {s['leg_phi0_deg'][0]:.1f} / {s['leg_phi0_deg'][1]:.1f}°",
                  font=small_font, fill="#C8D8E9")
        draw.text((640, 604), f"轮速 {sim.feedback.wheel_speed[0]:+.1f} / {sim.feedback.wheel_speed[1]:+.1f} rad/s",
                  font=small_font, fill="#C8D8E9")
    color = "#FF8D8D" if contact["other_n"] > 1 else "#A6DBAE"
    draw.text((22, 642), f"接地法向力：左轮 {contact['left_n']:.1f} N   右轮 {contact['right_n']:.1f} N   非轮部件 {contact['other_n']:.1f} N",
              font=small_font, fill=color)
    footer = "0–0.05 s 松控，随后原始遥控请求自起身；镜头跟随位置与航向，保持世界竖直方向。"
    if getattr(sim, "wheel_case", "baseline") != "baseline":
        footer = "极性诊断对照，默认配置未改；" + ("镜头航向固定，可直接观察机体旋转。" if fixed else "镜头跟随航向。")
        if getattr(sim, "follow_case", "unchanged") == "angle_neg":
            footer = "诊断中仅在回中完成后反号跟随角输入，保留原始 IMU 与 C 控制器；尚未作为默认移植修正。"
        elif getattr(sim, "follow_case", "unchanged") == "coherent_gimbal":
            footer = "候选接口：1:1 反向 yaw 齿轮与云台 IMU 方向；保留实车 C 及参数，默认配置未改。"
    draw.text((22, 678), footer,
              font=small_font, fill="#9EB6D0")
    return np.asarray(canvas)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("out/native/recovery_video_20261014"))
    parser.add_argument("--variant", choices=("fast", "full"), default="fast")
    parser.add_argument("--seconds", type=float, default=5)
    parser.add_argument("--wheel-case", choices=CASES, default="baseline")
    parser.add_argument("--follow-case", choices=("unchanged", "angle_neg", "angle_zero", "coherent_gimbal"), default="unchanged")
    parser.add_argument("--camera-yaw", choices=("follow", "fixed"), default="follow")
    args = parser.parse_args()
    if not 0 < args.seconds <= 10:
        parser.error("bounded diagnostic duration must be between 0 and 10 seconds")
    if args.follow_case != "unchanged" and args.wheel_case != "both_neg_from_start":
        parser.error("Follow counterfactual requires --wheel-case both_neg_from_start")
    args.output.mkdir(parents=True, exist_ok=True)
    video = args.output / f"recovery_{args.variant}.mp4"
    report_path = args.output / f"recovery_{args.variant}.json"
    record_path = args.output / f"recovery_{args.variant}.jsonl"
    if any(p.exists() for p in (video, report_path, record_path)):
        parser.error("output already exists; choose a new output directory")
    if gpu_free_mb() < 1024:
        raise RuntimeError("less than 1024 MiB GPU memory available")
    font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font = ImageFont.truetype(font_path, 25)
    small_font = ImageFont.truetype(font_path, 20)
    started = time.monotonic()
    sim = (FollowDiagnosticSimulation(args.variant, pose="encoder", follow_case=args.follow_case) if args.follow_case != "unchanged" else
           VehicleSimulation(args.variant, pose="encoder") if args.wheel_case == "baseline" else
           WheelHandoffSimulation(args.variant, pose="encoder", wheel_case=args.wheel_case))
    renderer = writer = recorder = None
    report = {"completed": False, "metadata": sim.metadata(), "seconds": args.seconds,
              "video": str(video.resolve()), "playback_speed": .5, "fps": 30,
              "resolution": [1280, 720], "render_backend": os.environ.get("MUJOCO_GL"),
              "renderer_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
              "model_sha256": hashlib.sha256(Path(VARIANTS[args.variant]).read_bytes()).hexdigest(),
              "adapter_sha256": hashlib.sha256((ROOT/'balance_lab/firmware/simulation.py').read_bytes()).hexdigest(),
              "wheel_case": args.wheel_case,
              "follow_case": args.follow_case, "camera_yaw": args.camera_yaw,
              "wheel_probe_sha256": hashlib.sha256((ROOT/'tools/probes/native_wheel_handoff_audit.py').read_bytes()).hexdigest(),
              "yaw_probe_sha256": hashlib.sha256((ROOT/'tools/probes/native_yaw_follow_audit.py').read_bytes()).hexdigest(),
              "resource_guard": {"min_ram_available_mb": 2048, "min_gpu_available_mb": 1024,
                                 "max_wall_seconds": 150, "stream_frames": True}}
    frames = 0
    try:
        sim.model.vis.global_.offwidth = 640
        sim.model.vis.global_.offheight = 432
        renderer = mujoco.Renderer(sim.model, height=432, width=640)
        gl = {k: GL.glGetString(v).decode() for k, v in
              (("vendor", GL.GL_VENDOR), ("renderer", GL.GL_RENDERER), ("version", GL.GL_VERSION))}
        report["opengl"] = gl
        if "NVIDIA" not in (gl["vendor"] + gl["renderer"]).upper():
            raise RuntimeError(f"NVIDIA GPU renderer required, obtained {gl}")
        print("GPU renderer:", gl["renderer"], flush=True)
        writer = imageio.get_writer(video, fps=30, codec="libx264", pixelformat="yuv420p",
                                    macro_block_size=8, ffmpeg_params=["-crf", "20", "-preset", "fast", "-threads", "2", "-movflags", "+faststart"])
        recorder = Recorder(record_path, sim.metadata(), (43, 50, 54, 64))
        frame = compose(renderer, sim, font, small_font, contacts(sim), initial=True, fixed=args.camera_yaw == "fixed")
        for _ in range(30):
            writer.append_data(frame); frames += 1
        last = None
        with (args.output/f"telemetry_{args.variant}.csv").open("w", newline="") as stream:
            csv_writer = csv.writer(stream)
            csv_writer.writerow(("time_s", "pitch_deg", "roll_deg", "recovery_state", "fault", "stand_state",
                                 "wheel_current_l", "wheel_current_r", "contact_l_n", "contact_r_n", "nonwheel_n",
                                 "yaw_rate_rad_s", "follow_angle_deg", "yaw_rate_ref_rad_s"))
            next_frame = 1/30
            for _ in range(round(args.seconds / sim.tick_s)):
                t = sim.ticks*sim.tick_s
                command = Command.remote(right_switch=1 if t < .05 else 3, left_switch=2)
                out = sim.step(command)
                recorder.tick(sim, command)
                elapsed = sim.ticks*sim.tick_s
                if elapsed + 1e-9 >= next_frame:
                    contact = contacts(sim)
                    frame = compose(renderer, sim, font, small_font, contact, fixed=args.camera_yaw == "fixed")
                    # Duplicate one current frame for half-speed viewing; do
                    # not retain earlier rendered frames.
                    for _ in range(2):
                        writer.append_data(frame); frames += 1
                    csv_writer.writerow((elapsed, out.imu[1], out.imu[0], out.recovery_state, out.recovery_fault,
                                         out.stand_state, *out.wheel_current, contact["left_n"], contact["right_n"], contact["other_n"],
                                         out.imu[5], np.degrees(out.yaw_relative), sim.runtime.debug_page(50)[8]))
                    next_frame += 1/30
                    last = frame
                if sim.ticks % 500 == 0:
                    ram, gpu = available_mb(), gpu_free_mb()
                    print(f"t={elapsed:.2f}s RAM available={ram:.0f} MiB GPU free={gpu} MiB", flush=True)
                    if ram < 2048 or gpu < 1024 or time.monotonic()-started > 150:
                        raise RuntimeError("render resource guard reached; stopping")
            if last is None:
                last = frame
            for _ in range(30):
                writer.append_data(last); frames += 1
        report["final"] = summary(sim)
        report["completed"] = True
    except Exception as exc:
        report["failure"] = f"{type(exc).__name__}: {exc}"
        raise
    finally:
        if recorder is not None: recorder.close()
        if writer is not None: writer.close()
        if renderer is not None: renderer.close()
        sim.close()
        report.update(frames=frames, duration_video_s=frames/30,
                      wall_seconds=time.monotonic()-started,
                      peak_process_ram_mb=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024)
        if video.is_file():
            report["video_sha256"] = hashlib.sha256(video.read_bytes()).hexdigest()
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)+"\n")
    print(video.resolve(), flush=True)


if __name__ == "__main__":
    main()
