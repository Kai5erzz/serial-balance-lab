"""Run the first floating-base traditional-LQR experiment."""

from __future__ import annotations

import faulthandler
import os
from pathlib import Path
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np
from mujoco.glfw import glfw

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    DRIVE_SPEED_MAX,
    DRIVE_YAW_RATE_MAX,
    LEG_LENGTH_DEFAULT,
    LEG_LENGTH_PRESETS,
    TraditionalLQR,
    VMC_COMPRESSION_MAX,
    VMC_COMPRESSION_MIN,
    VMC_ENCODER_ZERO_COMPRESSION,
    VMC_ENCODER_ZERO_SWING,
    VMC_LENGTH_MAX,
    VMC_LENGTH_MIN,
    VMC_NORMAL_STAND_COMPRESSION,
    VMC_NORMAL_STAND_LENGTH,
    VMC_VERTICAL_SWING,
    apply_code_zero_pose,
    apply_encoder_zero_pose,
    apply_saved_pose,
    encoder_joint_reading,
)


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml"
PLATFORM_MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast_platform.xml"
# The saved standing pose has a 2.4 mm left/right wheel-height mismatch.
# Place both wheel contact patches on the 600 mm upper deck rather than
# leaving the higher wheel just outside MuJoCo's contact tolerance.
PLATFORM_SPAWN_Z_OFFSET = 0.597
HOVER_CLEARANCE_M = 0.18
CALIBRATION_STEP_RAD = np.radians(1.0)
CALIBRATION_HIP_KP = 55.0
CALIBRATION_HIP_KD = 3.0
CALIBRATION_HIP_TORQUE_LIMIT = 24.0
# Real-RC semantics: holding an arrow key sets the LQR speed reference,
# releasing it returns the reference to zero (chassis brakes).  The MuJoCo
# passive-viewer callback exposes only the key code, not the GLFW action, so
# the native key state is sampled every render cycle on both operating
# systems.  This makes release handling symmetric with key-press handling.
RC_TARGET_SPEED_MPS = 1.00
RC_TARGET_YAW_RAD_S = 2.00
RC_HOLD_TIMEOUT_S = 0.35
if os.name == "nt":
    import ctypes as _ctypes

    _user32 = _ctypes.windll.user32
    _VK = {
        "up": 0x26,
        "down": 0x28,
        "left": 0x25,
        "right": 0x27,
    }

    def _rc_key_held(name: str) -> bool:
        return bool(_user32.GetAsyncKeyState(_VK[name]) & 0x8000)

else:
    _rc_key_held = None


def run_session(config: dict) -> dict | None:
    # Native GLFW/OpenGL faults otherwise terminate the process without a
    # Python traceback on Windows.  Keep this enabled so an unexpected viewer
    # exit can be distinguished from a controller exception.
    faulthandler.enable(all_threads=True)
    model = mujoco.MjModel.from_xml_path(str(config["model"]))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    controller = TraditionalLQR(model)
    controller.set_leg_length(LEG_LENGTH_DEFAULT)
    controller.reset(data)
    paused = {"value": True}
    mode = {"name": "hover"}
    timing = {"rtf": 0.0}
    restart_request: dict | None = None
    pending_keys: list[int] = []
    pending_keys_lock = threading.Lock()
    native_window: dict[str, object | None] = {"handle": None}
    pitch_test_until = {"time": -1.0}
    camera_follow = {"enabled": True}
    base_free = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    base_qadr = model.jnt_qposadr[base_free]
    base_dadr = model.jnt_dofadr[base_free]
    base_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    ground_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    # the platform scene replaces the flat ground with pyramid layers
    ground_geoms = {
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
        for name in (
            "ground", "ground_outer",
            "platform_top", "platform_step1", "platform_step2", "platform_step3",
        )
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, name) >= 0
    }
    wheel_body = {side: ids.wheel_body for side, ids in controller.sides.items()}
    joint_qadr = {
        name: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in ("r_j1", "r_j2", "l_j1", "l_j2")
    }
    joint_dadr = {
        name: model.jnt_dofadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
        for name in joint_qadr
    }
    joint_motor = {
        name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_motor")
        for name in joint_qadr
    }
    leg_joint_ids = []
    for joint_id in range(model.njnt):
        joint_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id) or ""
        if joint_name.startswith(("l_", "r_")) and not joint_name.endswith("_wheel"):
            leg_joint_ids.append(joint_id)
    original_joint_limited = model.jnt_limited.copy()
    hover_pose = np.zeros(7)
    calibration = {
        "swing": 0.0,
        "compression": 0.0,
        "target": {name: 0.0 for name in joint_qadr},
    }

    def set_calibration_limits_removed(removed: bool) -> None:
        for joint_id in leg_joint_ids:
            model.jnt_limited[joint_id] = 0 if removed else original_joint_limited[joint_id]

    def update_calibration_target() -> None:
        """Map the user's motor-space directions into mirrored CAD axes.

        Within each physical leg, equal J1/J2 motion changes phi0 while
        opposite motion changes l0.  Left CAD hinge axes are mirrored, hence
        the leading minus signs on that side.
        """
        swing = calibration["swing"]
        compression = calibration["compression"]
        calibration["target"].update({
            "r_j1": swing + compression,
            "r_j2": swing - compression,
            "l_j1": -swing - compression,
            "l_j2": -swing + compression,
        })

    def set_mode(name: str) -> None:
        """Reset into one of the explicit calibration/physics modes."""
        set_calibration_limits_removed(False)
        mode["name"] = name
        paused["value"] = name != "leg_calibration"
        mujoco.mj_resetData(model, data)
        if name in {"lqr", "platform_lqr"}:
            apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
        elif name == "recovery":
            apply_encoder_zero_pose(model, data)
        else:
            apply_code_zero_pose(model, data)
        controller.reset(data, calibrate_frame=True)
        controller.enabled = False
        controller.stance_enabled = False
        controller.gas_enabled = False
        if name == "lqr":
            controller.enable_calibrated_lqr(data)
        elif name == "platform_lqr":
            # Mode 6 is a persistent platform mode.  Apply the deck lift as
            # part of every reset, including the R-key reset, so it cannot
            # fall back to the mode-5 ground birth pose.
            data.qpos[base_qadr + 2] += float(
                config.get("z_offset", PLATFORM_SPAWN_Z_OFFSET)
            )
            mujoco.mj_forward(model, data)
            controller.enable_calibrated_lqr(data)
        elif name == "recovery":
            controller.begin_encoder_zero_recovery(data)
        if name in {"hover", "leg_calibration"}:
            hover_pose[:] = data.qpos[base_qadr:base_qadr + 7]
            hover_pose[2] += HOVER_CLEARANCE_M
            data.qpos[base_qadr:base_qadr + 7] = hover_pose
            data.qvel[base_dadr:base_dadr + 6] = 0.0
            mujoco.mj_forward(model, data)
            controller.reset(data)
        if name == "leg_calibration":
            set_calibration_limits_removed(True)
            # Start at the exact firmware-frame normal stance rather than an
            # arbitrary CAD q=0 pose.
            calibration["swing"] = VMC_VERTICAL_SWING
            calibration["compression"] = VMC_NORMAL_STAND_COMPRESSION
            update_calibration_target()
            mujoco.mj_forward(model, data)

    def pin_hover_base() -> None:
        data.qpos[base_qadr:base_qadr + 7] = hover_pose
        data.qvel[base_dadr:base_dadr + 6] = 0.0

    def step_leg_calibration() -> None:
        """Torque-position the four drive joints while the base is pinned."""
        pin_hover_base()
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        controller._apply_gimbal_lock(data)
        for name, target in calibration["target"].items():
            error = target - float(data.qpos[joint_qadr[name]])
            torque = (
                CALIBRATION_HIP_KP * error
                - CALIBRATION_HIP_KD * float(data.qvel[joint_dadr[name]])
            )
            data.ctrl[joint_motor[name]] = float(np.clip(
                torque,
                -CALIBRATION_HIP_TORQUE_LIMIT,
                CALIBRATION_HIP_TORQUE_LIMIT,
            ))

    def update_info(viewer: mujoco.viewer.Handle) -> None:
        pitch_deg = float(np.degrees(controller.body_pitch(data)))
        encoder_text = "  ".join(
            f"{name}={encoder_joint_reading(model, data, name):+.3f}"
            for name in joint_qadr
        )
        cad_text = "  ".join(f"{name}={data.qpos[address]:+.3f}" for name, address in joint_qadr.items())
        wheel_motor = {
            side: float(data.ctrl[ids.wheel_motor])
            for side, ids in controller.sides.items()
        }
        leg_geometry = {}
        for side in ("l", "r"):
            kin = controller.vmc_kinematics(data, side)
            leg_geometry[side] = (float(np.degrees(kin.phi0)), kin.l0)
        average_phi0 = 0.5 * (leg_geometry["l"][0] + leg_geometry["r"][0])
        average_l0 = 0.5 * (leg_geometry["l"][1] + leg_geometry["r"][1])
        wheel_ground = {"l": False, "r": False}
        other_ground: set[str] = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if contact.geom1 not in ground_geoms and contact.geom2 not in ground_geoms:
                continue
            other_geom = contact.geom2 if contact.geom1 in ground_geoms else contact.geom1
            body = int(model.geom_bodyid[other_geom])
            matching_side = next((side for side, value in wheel_body.items() if value == body), None)
            if matching_side is not None:
                wheel_ground[matching_side] = True
            else:
                body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
                other_ground.add(body_name or f"body#{body}")
        other_ground_text = ",".join(sorted(other_ground)) if other_ground else "none"
        calibration_text = ""
        if mode["name"] == "leg_calibration":
            calibration_text = (
                f"\ncal target: swing={np.degrees(calibration['swing']):+.1f} deg  "
                f"compression={np.degrees(calibration['compression']):+.3f} deg"
                f"\ncalibrated range={VMC_LENGTH_MIN:.6f}..{VMC_LENGTH_MAX:.6f} m  "
                f"stand target={controller.normal_leg_length:.3f} m"
            )
        state_text = (
            f"MODE: {mode['name'].upper()}  |  {'PAUSED' if paused['value'] else 'RUNNING'}\n"
            f"base z={data.xpos[base_id, 2]:.3f} m   pitch={pitch_deg:+.2f} deg  "
            f"real-time={timing['rtf']:.2f}x\n"
            f"encoder: {encoder_text}\n"
            f"CAD q:   {cad_text}\n"
            f"leg phi0 L/R={leg_geometry['l'][0]:+.3f}/{leg_geometry['r'][0]:+.3f} deg  "
            f"avg={average_phi0:+.3f}\n"
            f"leg l0 L/R={leg_geometry['l'][1]:.6f}/{leg_geometry['r'][1]:.6f} m  "
            f"avg={average_l0:.6f}  target={controller.normal_leg_length:.3f} m"
            f"{calibration_text}\n"
            f"ground wheel L/R={'Y' if wheel_ground['l'] else 'N'}/"
            f"{'Y' if wheel_ground['r'] else 'N'}  other={other_ground_text}\n"
            f"LQR={'ON' if controller.enabled else 'OFF'} gain={controller.lqr_gain:.2f}  "
            f"gas={'ON' if controller.gas_enabled else 'OFF'}  VMC={'ON' if controller.stance_enabled else 'OFF'}\n"
            f"drive forward={controller.drive_forward_target:+.2f} m/s  "
            f"yaw target/actual={controller.drive_yaw_rate_target:+.2f}/"
            f"{controller.yaw_rate:+.2f} rad/s  "
            f"camera={'FOLLOW' if camera_follow['enabled'] else 'FREE'}\n"
            f"jump={controller.jump_state.upper()} fault={controller.jump_fault.upper()}  "
            f"airborne={'Y' if controller.jump_off_ground else 'N'}  "
            f"jump ref={controller.jump_length_target:.3f} m  "
            f"air ref={controller.jump_air_reference['l']:.3f}/"
            f"{controller.jump_air_reference['r']:.3f} m  "
            f"landing buffer={controller.landing_buffer_phase.upper()}\n"
            f"wheel motor L/R={wheel_motor['l']:+.3f}/{wheel_motor['r']:+.3f} Nm  "
            f"(zero error => zero torque)\n"
            f"recovery={controller.recovery_state.upper()}  "
            f"F0 L/R={controller.recovery_output['l'][0]:+.1f}/{controller.recovery_output['r'][0]:+.1f} N  "
            f"Tp L/R={controller.recovery_output['l'][1]:+.2f}/{controller.recovery_output['r'][1]:+.2f} Nm\n"
            f"hip motors L={controller.last_motor_torque['l'][0]:+.2f}/{controller.last_motor_torque['l'][1]:+.2f}  "
            f"R={controller.last_motor_torque['r'][0]:+.2f}/{controller.last_motor_torque['r'][1]:+.2f} Nm\n"
            f"gimbal yaw={data.qpos[controller.gimbal_qpos]:+.3f} rad  "
            f"{'LOCKED at 0' if controller.gimbal_locked else 'FREE'}"
        )
        help_text = (
            "1: HOVER (base fixed, legs passive)\n"
            "2: FREE  (gravity only, all outputs off)\n"
            "3: CALIBRATED N POSE -> firmware LOW LQR (starts paused)\n"
            "4: HOVER LEG CALIBRATION (CAD stops off; calibrated length limit on)\n"
            "5: CALIBRATED ENCODER ZERO -> firmware RECOVERY -> LOW LQR\n"
            "Arrow Up/Down: forward speed +/-0.60 m/s\n"
            "Arrow Left/Right: yaw +/-2.00 rad/s\n"
            "7/8/9: leg length 0.130/0.240/0.300 m (default 0.130)\n"
            "X: stop movement/turn   J: trigger firmware jump\n"
            "F: toggle base-follow/free camera\n"
            "In LEG CALIBRATION W/S=swing, D/A=compress/extend\n"
            "Z: firmware encoder zero   M/C: vertical max/min   N: 0.150 m stand\n"
            "P: apply a 2.0 N·m pitch pulse to verify recovery\n"
            "Space: run/pause   R: reset current mode"
        )
        viewer.set_texts((None, mujoco.mjtGridPos.mjGRID_TOPLEFT, state_text, help_text))

    def key_callback(key: int) -> None:
        # GLFW invokes this on the viewer thread.  Do not touch MuJoCo data or
        # acquire viewer.lock here: doing either can deadlock against sync().
        # The simulation thread drains this queue at the start of each batch.
        if os.name != "nt" and native_window["handle"] is None:
            # launch_passive intentionally hides the GLFW window handle, but
            # the callback runs on the render thread with its context current.
            # Keep that opaque handle so the simulation thread can query the
            # physical key state and observe key release as well as key press.
            native_window["handle"] = glfw.get_current_context()
        if key in (glfw.KEY_6, glfw.KEY_F6):
            print(f"key event: {key} (6/F6) received", flush=True)
        with pending_keys_lock:
            pending_keys.append(key)

    def process_pending_keys(viewer: mujoco.viewer.Handle) -> None:
        nonlocal restart_request
        with pending_keys_lock:
            keys = pending_keys[:]
            pending_keys.clear()
        if not keys:
            return

        # Resetting a mode mutates model/data, so serialize these short state
        # changes with the renderer.  Physics stepping itself remains outside
        # this lock below.
        with viewer.lock():
            for key in keys:
                if key == glfw.KEY_SPACE:
                    paused["value"] = not paused["value"]
                elif key == glfw.KEY_1:
                    set_mode("hover")
                    print("mode: HOVER")
                elif key == glfw.KEY_2:
                    set_mode("free")
                    print("mode: FREE")
                elif key == glfw.KEY_3:
                    set_mode("lqr")
                    print("mode: calibrated N equilibrium; firmware six-state LOW LQR armed; press Space to run")
                elif key == glfw.KEY_4:
                    set_mode("leg_calibration")
                    print("mode: HOVER LEG CALIBRATION; W/S swing, D/A length; Z/M/C/N calibrated presets")
                elif key == glfw.KEY_5:
                    set_mode("recovery")
                    print("mode: calibrated encoder zero; firmware HOLD/SWING/RETRACT/HANDOFF armed; press Space")
                elif key in (glfw.KEY_6, glfw.KEY_F6):
                    restart_request = {
                        "model": str(PLATFORM_MODEL),
                        "z_offset": PLATFORM_SPAWN_Z_OFFSET,
                        "platform_spawn": True,
                        "start_mode": "platform_lqr",
                    }
                    print(
                        "6: switching to 600 mm PLATFORM scene (LQR armed on the "
                        "platform top; viewer restarts). Drive off the edge to "
                        "test the off-ground / landing logic."
                    )
                elif key in (glfw.KEY_7, glfw.KEY_8, glfw.KEY_9):
                    leg_length_by_key = {
                        glfw.KEY_7: LEG_LENGTH_PRESETS[0],
                        glfw.KEY_8: LEG_LENGTH_PRESETS[1],
                        glfw.KEY_9: LEG_LENGTH_PRESETS[2],
                    }
                    selected = controller.set_leg_length(
                        leg_length_by_key[key]
                    )
                    print(f"leg length target: {selected:.3f} m")
                elif mode["name"] == "leg_calibration" and key in {glfw.KEY_W, glfw.KEY_S, glfw.KEY_A, glfw.KEY_D}:
                    if key == glfw.KEY_W:
                        calibration["swing"] += CALIBRATION_STEP_RAD
                    elif key == glfw.KEY_S:
                        calibration["swing"] -= CALIBRATION_STEP_RAD
                    elif key == glfw.KEY_D:
                        calibration["compression"] += CALIBRATION_STEP_RAD
                    elif key == glfw.KEY_A:
                        calibration["compression"] -= CALIBRATION_STEP_RAD
                    calibration["compression"] = float(np.clip(
                        calibration["compression"],
                        VMC_COMPRESSION_MIN,
                        VMC_COMPRESSION_MAX,
                    ))
                    update_calibration_target()
                    print(
                        f"cal target: swing={np.degrees(calibration['swing']):+.1f} deg, "
                        f"compression={np.degrees(calibration['compression']):+.1f} deg"
                    )
                elif mode["name"] == "leg_calibration" and key in {
                    glfw.KEY_Z, glfw.KEY_M, glfw.KEY_C, glfw.KEY_N
                }:
                    if key == glfw.KEY_Z:
                        calibration["swing"] = VMC_ENCODER_ZERO_SWING
                        calibration["compression"] = VMC_ENCODER_ZERO_COMPRESSION
                        anchor_name = "firmware encoder zero"
                    elif key == glfw.KEY_M:
                        calibration["swing"] = VMC_VERTICAL_SWING
                        calibration["compression"] = VMC_COMPRESSION_MAX
                        anchor_name = "vertical maximum"
                    elif key == glfw.KEY_C:
                        calibration["swing"] = VMC_VERTICAL_SWING
                        calibration["compression"] = VMC_COMPRESSION_MIN
                        anchor_name = "vertical minimum"
                    else:
                        calibration["swing"] = VMC_VERTICAL_SWING
                        calibration["compression"] = VMC_NORMAL_STAND_COMPRESSION
                        anchor_name = "normal 0.150 m stand"
                    update_calibration_target()
                    print(
                        f"cal anchor: {anchor_name}; "
                        f"swing={np.degrees(calibration['swing']):+.3f} deg, "
                        f"compression={np.degrees(calibration['compression']):+.3f} deg"
                    )
                elif key == glfw.KEY_X and mode["name"] in {"lqr", "platform_lqr", "recovery"}:
                    controller.stop_drive()
                    print("drive command: STOP")
                elif key == glfw.KEY_J and mode["name"] in {"lqr", "platform_lqr", "recovery"}:
                    accepted, reason = controller.begin_jump(data)
                    print(
                        "jump accepted: firmware PRELOAD"
                        if accepted else f"jump rejected: {reason}"
                    )
                elif key == glfw.KEY_F:
                    camera_follow["enabled"] = not camera_follow["enabled"]
                    if camera_follow["enabled"]:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = base_id
                    else:
                        viewer.cam.lookat[:] = data.xpos[base_id]
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.trackbodyid = -1
                    print(
                        "camera: BASE FOLLOW"
                        if camera_follow["enabled"] else "camera: FREE"
                    )
                elif key == glfw.KEY_P and mode["name"] in {"lqr", "platform_lqr"}:
                    pitch_test_until["time"] = data.time + 0.10
                    print("pitch-test pulse: 2.0 N m for 0.10 s")
                elif key == glfw.KEY_R:
                    set_mode(mode["name"])

    # Current experiment starts from the user's calibrated real encoder zero.
    # Recovery is torque/contact driven and hands off to the accepted
    # l0=0.150 m LOW-LQR equilibrium without rewriting qpos after reset.
    start_mode = config.get("start_mode", "recovery")
    set_mode(start_mode)
    if config.get("z_offset", 0.0) and start_mode != "platform_lqr":
        # lift the whole chassis onto the platform after the mode pose was
        # applied (set_mode resets qpos)
        data.qpos[base_qadr + 2] += config["z_offset"]
        mujoco.mj_forward(model, data)
        if config.get("platform_spawn", False):
            # Re-arm only after the lift.  This makes mode 6 a true calibrated
            # LQR start on the 600 mm upper deck instead of inheriting the
            # mode-5 ground/recovery birth state.
            controller.enable_calibrated_lqr(data)
        else:
            controller.reset(data, calibrate_frame=True)
    if start_mode == "recovery":
        print("CALIBRATED ENCODER-ZERO RECOVERY armed and paused. Press Space to run; 3 selects direct LQR.")
    else:
        print(
            f"armed and paused ({start_mode}). Press Space to run; "
            "6 switches to the 600 mm platform scene."
        )
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.azimuth, viewer.cam.elevation, viewer.cam.distance = 135, -18, 1.15
        viewer.cam.lookat[:] = (0.0, 0.0, 0.15)
        # Recovery and balance may translate the chassis appreciably.  Track
        # the base so a healthy controller cannot simply drive out of frame
        # and look like a frozen/empty viewer.
        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
        viewer.cam.trackbodyid = base_id
        previous_print = 0.0
        previous_info_update = -float("inf")
        physics_steps_per_batch = 5
        physics_time_per_batch = physics_steps_per_batch * model.opt.timestep
        paused_render_period = 1.0 / 60.0
        timing_wall = time.perf_counter()
        timing_sim = data.time
        while viewer.is_running():
            frame_start = time.perf_counter()
            process_pending_keys(viewer)
            # Poll the physical key state every frame: arrow keys held -> LQR
            # speed reference applied; released -> reference 0.  A physics
            # batch is only five 2 ms steps, so release latency is bounded to
            # roughly one render loop plus 10 ms rather than one stale command
            # timeout.  On Linux the window handle is captured by the GLFW
            # callback above; before the first key event there is no state to
            # sample and the controller remains at its zero command.
            if mode["name"] in {"lqr", "platform_lqr", "recovery"}:
                window = native_window["handle"]
                if os.name == "nt":
                    key_held = _rc_key_held
                elif window is not None:
                    key_held = lambda key: glfw.get_key(window, key) == glfw.PRESS
                else:
                    key_held = None
                if key_held is None:
                    keyboard_command = None
                else:
                    keyboard_command = (0.0, 0.0)
                    forward = 0.0
                    yaw = 0.0
                    if key_held(glfw.KEY_UP):
                        forward -= RC_TARGET_SPEED_MPS
                    if key_held(glfw.KEY_DOWN):
                        forward += RC_TARGET_SPEED_MPS
                    if key_held(glfw.KEY_LEFT):
                        yaw += RC_TARGET_YAW_RAD_S
                    if key_held(glfw.KEY_RIGHT):
                        yaw -= RC_TARGET_YAW_RAD_S
                    keyboard_command = (forward, yaw)
                if keyboard_command is not None:
                    forward, yaw = keyboard_command
                    if (forward, yaw) != (
                        controller.drive_forward_target,
                        controller.drive_yaw_rate_target,
                    ):
                        controller.set_drive_command(forward, yaw)
                        print(
                            f"RC drive: forward={forward:+.2f} m/s "
                            f"yaw={yaw:+.2f} rad/s",
                            flush=True,
                        )
            running = not paused["value"]
            if not running:
                timing["rtf"] = 0.0
                timing_wall = frame_start
                timing_sim = data.time
            else:
                # mj_step must not run under viewer.lock.  The passive viewer
                # receives the finished state at sync(), so rendering remains
                # responsive while a physics batch is being computed.
                for _ in range(physics_steps_per_batch):
                    if mode["name"] in {"lqr", "platform_lqr", "recovery"}:
                        controller.step(data, model.opt.timestep)
                        if data.time < pitch_test_until["time"]:
                            _, _, lateral = controller._frame(data)
                            mujoco.mj_applyFT(
                                model,
                                data,
                                np.zeros(3),
                                2.0 * lateral,
                                data.xpos[base_id],
                                base_id,
                                data.qfrc_applied,
                            )
                    elif mode["name"] == "leg_calibration":
                        step_leg_calibration()
                    else:
                        controller.passive_step(data)
                        if mode["name"] == "hover":
                            pin_hover_base()
                    mujoco.mj_step(model, data)
                if mode["name"] in {"hover", "leg_calibration"}:
                    pin_hover_base()
                    mujoco.mj_forward(model, data)

            timing_now = time.perf_counter()
            timing_elapsed = timing_now - timing_wall
            if running and timing_elapsed >= 0.5:
                timing["rtf"] = (data.time - timing_sim) / timing_elapsed
                timing_wall = timing_now
                timing_sim = data.time

            # set_texts() and sync() both acquire the viewer's native mutex
            # internally.  Wrapping set_texts() in viewer.lock() can deadlock
            # against the GLFW render thread because that mutex is not
            # re-entrant.  Ten UI updates per second are ample for telemetry
            # and avoid needless renderer contention.
            if frame_start - previous_info_update >= 0.10:
                update_info(viewer)
                previous_info_update = frame_start
            if data.time - previous_print >= 0.5:
                previous_print = data.time
                print(
                    f"t={data.time:5.2f}  L T/Tp={controller.last_output['l']}  "
                    f"R T/Tp={controller.last_output['r']}  "
                    f"F={controller.last_support['l']:.1f}/{controller.last_support['r']:.1f} N  "
                    f"recovery={controller.recovery_state}  "
                    f"jump={controller.jump_state}/{controller.jump_fault}  "
                    f"LQR gain={controller.lqr_gain:.2f}"
                )
            viewer.sync()
            if restart_request is not None:
                print("restart requested; closing viewer...", flush=True)
                break
            frame_elapsed = time.perf_counter() - frame_start
            target_period = physics_time_per_batch if running else paused_render_period
            time.sleep(max(0.0, target_period - frame_elapsed))
        print("viewer.is_running() became false; viewer window requested shutdown", flush=True)
    print("viewer context closed normally", flush=True)
    return restart_request


def main() -> None:
    """Launch viewer sessions; key 6 restarts on the platform terrain."""
    config: dict = {
        "model": str(MODEL),
        "z_offset": 0.0,
        "platform_spawn": False,
        "start_mode": "recovery",
    }
    while True:
        restart = run_session(config)
        if restart is None:
            break
        config = restart


if __name__ == "__main__":
    main()
