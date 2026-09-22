"""Headless video renderer for LQR standing and in-place jump demonstrations.

Designed for minimal memory and VRAM usage:
- Streams frames directly to libx264 MP4 writer (O(1) memory per frame, no large array in RAM).
- Uses MuJoCo offscreen rendering without CUDA/PyTorch dependencies.
- Embeds telemetry HUD (mode, time, pitch, leg length, base height, jump state).
"""
from __future__ import annotations

import argparse
import gc
import os
from pathlib import Path
import resource
import sys
import time

import cv2
import imageio
import mujoco
import numpy as np

REPO_ROOT = Path("/home/kaiser/robomaster/rl-lab")
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from balance_lab.plant import load_model
from balance_lab.sysid import load_profile, MechanicsFit
from balance_lab.paths import CALIBRATED_STAND_POSE_PATH
from balance_lab.plant.poses import apply_saved_pose
from balance_lab.control.controller import TraditionalLQR
import balance_lab.control.jump as _jm


def _setup_simulation(use_sysid: bool = True):
    """Load model, controller, and apply calibrated standing pose."""
    if use_sysid:
        profile = load_profile("identified_2026-09-07_v2")
        mechanics = MechanicsFit(
            hip_damping_scale=0.4604 / 0.35,
            hip_frictionloss_nm=0.2202,
            hip_armature_kg_m2=0.0035,
            wheel_damping_scale=1.0,
            wheel_frictionloss_nm=0.0,
            source="real2sim_sysid_v2_temp4",
        )
        model, applied = load_model("fast", plant=(profile, mechanics))
        ctrl = TraditionalLQR(model)
        ctrl.apply_identified(profile)
        plant_tag = "Identified V2 (temp4)"
    else:
        model, _ = load_model("fast")
        ctrl = TraditionalLQR(model)
        plant_tag = "Nominal CAD"

    data = mujoco.MjData(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    mujoco.mj_forward(model, data)
    ctrl.reset(data, calibrate_frame=True)
    ctrl.enable_calibrated_lqr(data)

    return model, data, ctrl, plant_tag


def _setup_camera(model: mujoco.MjModel):
    """Setup tracking isometric camera."""
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
    cam.trackbodyid = base_id
    cam.azimuth = 205.0
    cam.elevation = -12.0
    cam.distance = 1.60
    cam.lookat[:] = [0.0, 0.0, 0.15]
    return cam, base_id


def _add_hud_overlay(
    img: np.ndarray,
    title: str,
    subtitle: str,
    sim_time: float,
    pitch_deg: float,
    l0: float,
    z: float,
    jump_state: str | None = None,
    off_ground: bool = False,
) -> np.ndarray:
    """Add HUD telemetry overlay to the frame."""
    frame = img.copy()
    h, w, _ = frame.shape

    # Draw semi-transparent header bar
    overlay = frame.copy()
    cv2.rectangle(overlay, (10, 10), (w - 10, 85), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
    cv2.rectangle(frame, (10, 10), (w - 10, 85), (70, 70, 70), 1)

    # Title & Subtitle
    cv2.putText(
        frame,
        title,
        (20, 32),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (240, 240, 240),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        frame,
        subtitle,
        (20, 52),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 220, 255),
        1,
        cv2.LINE_AA,
    )

    # Telemetry Line
    telemetry = (
        f"Time: {sim_time:4.2f}s | Pitch: {pitch_deg:+5.2f} deg | "
        f"Leg L0: {l0:.3f}m | Base Z: {z:.3f}m"
    )
    cv2.putText(
        frame,
        telemetry,
        (20, 74),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.42,
        (200, 255, 200),
        1,
        cv2.LINE_AA,
    )

    # If jump info available, draw state badge
    if jump_state is not None:
        state_text = f"STATE: {jump_state.upper()}"
        color = (0, 255, 0) if jump_state == "idle" else (0, 165, 255)
        if jump_state in ("thrust", "flight"):
            color = (0, 0, 255)
        cv2.putText(
            frame,
            state_text,
            (w - 190, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )
        if off_ground:
            cv2.putText(
                frame,
                "[ AIRBORNE ]",
                (w - 190, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.48,
                (0, 255, 255),
                1,
                cv2.LINE_AA,
            )

    return frame


def render_stand_5s(output_file: Path, use_sysid: bool = True, fps: int = 30) -> None:
    """Render 5 seconds of stable LQR standing."""
    print(f"\n[1/2] Rendering 5s LQR Standing Demo -> {output_file}")
    model, data, ctrl, plant_tag = _setup_simulation(use_sysid=use_sysid)
    cam, base_id = _setup_camera(model)
    renderer = mujoco.Renderer(model, height=480, width=640)

    output_file.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        str(output_file),
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
        format="FFMPEG",
        ffmpeg_params=["-crf", "20", "-preset", "fast"],
    )

    duration = 5.0
    dt = model.opt.timestep
    frame_interval = 1.0 / fps
    next_frame_time = 0.0
    frame_count = 0

    t0 = time.time()
    while data.time < duration:
        ctrl.step(data, dt)
        mujoco.mj_step(model, data)

        if data.time >= next_frame_time:
            renderer.update_scene(data, camera=cam)
            raw_rgb = renderer.render()

            l0 = 0.5 * sum(ctrl.vmc_kinematics(data, s).l0 for s in ("l", "r"))
            pitch_deg = float(np.degrees(ctrl.body_pitch(data)))
            z = float(data.xpos[base_id, 2])

            annotated = _add_hud_overlay(
                raw_rgb,
                title=f"BALANCE-LAB | Six-State LQR Balance [{plant_tag}]",
                subtitle="Mode: Calibrated Equilibrium Standing (5.0s)",
                sim_time=data.time,
                pitch_deg=pitch_deg,
                l0=l0,
                z=z,
            )
            writer.append_data(annotated)
            frame_count += 1
            next_frame_time += frame_interval

    writer.close()
    elapsed = time.time() - t0
    mem_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    print(
        f"  Done: {frame_count} frames rendered in {elapsed:.2f}s "
        f"({frame_count / elapsed:.1f} fps). Max RSS: {mem_mb:.1f} MB"
    )
    del renderer
    gc.collect()


def render_jump_inplace(
    output_file: Path, use_sysid: bool = True, fps: int = 30
) -> None:
    """Render in-place jump demonstration: stand -> preload -> thrust -> flight -> land -> recover."""
    print(f"\n[2/2] Rendering In-Place Jump Demo -> {output_file}")
    model, data, ctrl, plant_tag = _setup_simulation(use_sysid=use_sysid)
    cam, base_id = _setup_camera(model)
    renderer = mujoco.Renderer(model, height=480, width=640)

    # Set preload readiness threshold suitable for identified mechanics
    _jm.JUMP_PRELOAD_READY_LENGTH = 0.158

    output_file.parent.mkdir(parents=True, exist_ok=True)
    writer = imageio.get_writer(
        str(output_file),
        fps=fps,
        codec="libx264",
        pixelformat="yuv420p",
        format="FFMPEG",
        ffmpeg_params=["-crf", "20", "-preset", "fast"],
    )

    duration = 4.5
    dt = model.opt.timestep
    frame_interval = 1.0 / fps
    next_frame_time = 0.0
    frame_count = 0
    jump_triggered = False

    t0 = time.time()
    while data.time < duration:
        # Trigger jump at 1.0s after settling
        if data.time >= 1.0 and not jump_triggered:
            ok, reason = ctrl.begin_jump(data)
            jump_triggered = True
            print(f"  Triggered begin_jump at t={data.time:.2f}s: ok={ok}, reason={reason}")

        ctrl.step(data, dt)
        mujoco.mj_step(model, data)

        if data.time >= next_frame_time:
            renderer.update_scene(data, camera=cam)
            raw_rgb = renderer.render()

            l0 = 0.5 * sum(ctrl.vmc_kinematics(data, s).l0 for s in ("l", "r"))
            pitch_deg = float(np.degrees(ctrl.body_pitch(data)))
            z = float(data.xpos[base_id, 2])

            annotated = _add_hud_overlay(
                raw_rgb,
                title=f"BALANCE-LAB | In-Place Jump [{plant_tag}]",
                subtitle="Sequence: Stand -> Preload -> Thrust -> Flight -> Land -> Settle",
                sim_time=data.time,
                pitch_deg=pitch_deg,
                l0=l0,
                z=z,
                jump_state=ctrl.jump_state,
                off_ground=ctrl.jump_off_ground,
            )
            writer.append_data(annotated)
            frame_count += 1
            next_frame_time += frame_interval

    writer.close()
    elapsed = time.time() - t0
    mem_mb = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024
    print(
        f"  Done: {frame_count} frames rendered in {elapsed:.2f}s "
        f"({frame_count / elapsed:.1f} fps). Max RSS: {mem_mb:.1f} MB"
    )
    del renderer
    gc.collect()


def main():
    parser = argparse.ArgumentParser(description="Render LQR standing and jump videos")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("/home/kaiser/robomaster/rl-lab/out"),
        help="Directory to save videos",
    )
    parser.add_argument(
        "--nominal",
        action="store_true",
        help="Use nominal model instead of identified profile",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Video frame rate (default: 30)",
    )
    args = parser.parse_args()

    use_sysid = not args.nominal
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    stand_mp4 = out_dir / "lqr_stand_5s.mp4"
    jump_mp4 = out_dir / "lqr_jump_inplace.mp4"

    render_stand_5s(stand_mp4, use_sysid=use_sysid, fps=args.fps)
    render_jump_inplace(jump_mp4, use_sysid=use_sysid, fps=args.fps)

    print("\nAll videos rendered successfully!")
    print(f"1. Standing: {stand_mp4}")
    print(f"2. Jump:     {jump_mp4}")


if __name__ == "__main__":
    main()
