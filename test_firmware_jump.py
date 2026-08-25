"""Headless regression for the physically actuated jump controller.

The test covers takeoff, compact flight, wheel-only touchdown, buffered
landing, and the final LQR handoff.  No body force or qpos reset is used.
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    JUMP_HIP_MOTOR_TORQUE_LIMIT,
    TraditionalLQR,
    WHEEL_RADIUS_M,
    apply_saved_pose,
)


MODEL = Path(os.environ.get(
    "MUJOCO_MODEL",
    Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml",
))
TRACE = os.environ.get("JUMP_TRACE") == "1"
TRACE_PERIOD = float(os.environ.get("JUMP_TRACE_PERIOD", "0.025"))
FORWARD_TARGET = float(os.environ.get("JUMP_FORWARD_TARGET", "0.0"))


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller.set_drive_command(FORWARD_TARGET, 0.0)

    # Let the accepted LOW equilibrium finish its small contact transient.
    settle_end = data.time + 3.0
    while data.time < settle_end:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)

    start_z = float(data.xpos[controller.base, 2])
    accepted, reason = controller.begin_jump(data)
    if not accepted:
        raise SystemExit(f"jump precheck rejected: {reason}")

    transitions = [(data.time, controller.jump_state)]
    previous_state = controller.jump_state
    max_z = start_z
    min_z = start_z
    max_pitch = 0.0
    max_roll = 0.0
    max_air_pitch = 0.0
    apex_pitch = 0.0
    apex_wheel_clearance = 0.0
    contact_pitch = np.nan
    first_contact_time = np.nan
    max_touchdown_pitch = 0.0
    max_touchdown_wheel_step = 0.0
    previous_wheel_command = {
        side: float(data.ctrl[ids.wheel_motor])
        for side, ids in controller.sides.items()
    }
    landing_buffer_phases: set[str] = set()
    airborne_time = 0.0
    max_motor_torque = 0.0
    peak_motor_time = 0.0
    was_airborne = False
    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    wheel_bodies = {ids.wheel_body for ids in controller.sides.values()}
    non_wheel_ground_bodies: set[str] = set()
    non_wheel_ground_times: dict[str, list[float]] = {}
    tail_pitch: list[float] = []
    tail_roll: list[float] = []
    tail_z: list[float] = []
    tail_both_wheels: list[bool] = []
    finite = True
    next_trace = data.time
    end_time = data.time + 4.0
    while data.time < end_time:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if controller.jump_state != previous_state:
            previous_state = controller.jump_state
            transitions.append((data.time, previous_state))
        z = float(data.xpos[controller.base, 2])
        if z > max_z:
            max_z = z
            apex_pitch = float(np.degrees(controller.body_pitch(data)))
            apex_wheel_clearance = min(
                float(data.xpos[ids.wheel_body, 2]) - WHEEL_RADIUS_M
                for ids in controller.sides.values()
            )
        min_z = min(min_z, z)
        max_pitch = max(
            max_pitch, abs(float(np.degrees(controller.body_pitch(data))))
        )
        max_roll = max(
            max_roll, abs(float(np.degrees(controller.body_roll(data))))
        )
        if controller.jump_off_ground:
            was_airborne = True
            max_air_pitch = max(
                max_air_pitch,
                abs(float(np.degrees(controller.body_pitch(data)))),
            )
            airborne_time += model.opt.timestep
        motor_torque = max(
            abs(float(value))
            for values in controller.last_motor_torque.values()
            for value in values
        )
        max_motor_torque = max(max_motor_torque, motor_torque)
        peak_motor_time += model.opt.timestep * int(motor_torque > 20.0)

        wheels_touching: set[int] = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if floor not in (contact.geom1, contact.geom2):
                continue
            other_geom = (
                contact.geom2 if contact.geom1 == floor else contact.geom1
            )
            other_body = int(model.geom_bodyid[other_geom])
            if other_body in wheel_bodies:
                wheels_touching.add(other_body)
            else:
                body_name = mujoco.mj_id2name(
                    model, mujoco.mjtObj.mjOBJ_BODY, other_body
                )
                if was_airborne:
                    label = body_name or f"body#{other_body}"
                    non_wheel_ground_bodies.add(label)
                    window = non_wheel_ground_times.setdefault(
                        label, [float(data.time), float(data.time)]
                    )
                    window[1] = float(data.time)
        if was_airborne and np.isnan(contact_pitch) and wheels_touching:
            contact_pitch = float(np.degrees(controller.body_pitch(data)))
            first_contact_time = float(data.time)
        if np.isfinite(first_contact_time) and (
            data.time - first_contact_time <= 0.15
        ):
            max_touchdown_pitch = max(
                max_touchdown_pitch,
                abs(float(np.degrees(controller.body_pitch(data)))),
            )
            for side, ids in controller.sides.items():
                command = float(data.ctrl[ids.wheel_motor])
                max_touchdown_wheel_step = max(
                    max_touchdown_wheel_step,
                    abs(command - previous_wheel_command[side]),
                )
        for side, ids in controller.sides.items():
            previous_wheel_command[side] = float(data.ctrl[ids.wheel_motor])
        landing_buffer_phases.add(controller.landing_buffer_phase)
        if data.time >= end_time - 0.5:
            tail_pitch.append(float(np.degrees(controller.body_pitch(data))))
            tail_roll.append(float(np.degrees(controller.body_roll(data))))
            tail_z.append(z)
            tail_both_wheels.append(wheels_touching == wheel_bodies)
        finite &= bool(
            np.isfinite(data.qpos).all()
            and np.isfinite(data.qvel).all()
            and np.isfinite(data.ctrl).all()
        )
        if TRACE and data.time + 1e-12 >= next_trace:
            lengths = [
                controller.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            ]
            pitch = controller.body_pitch(data)
            theta = [
                np.pi / 2.0
                - controller.vmc_kinematics(data, side).phi0
                + pitch
                for side in ("l", "r")
            ]
            print(
                f"t={data.time:.3f} state={controller.jump_state:12s} "
                f"off={int(controller.jump_off_ground)} "
                f"buffer={controller.landing_buffer_phase:8s} "
                f"contact={int(controller.jump_raw_wheel_contact['l'])}/"
                f"{int(controller.jump_raw_wheel_contact['r'])} "
                f"z={z:.3f} l0={lengths[0]:.3f}/{lengths[1]:.3f} "
                f"ref={controller.length_reference['l']:.3f} "
                f"F={controller.last_support['l']:+.1f}/"
                f"{controller.last_support['r']:+.1f} "
                f"Tp={controller.last_output['l'][1]:+.1f}/"
                f"{controller.last_output['r'][1]:+.1f} "
                f"Tw={data.ctrl[controller.sides['l'].wheel_motor]:+.2f}/"
                f"{data.ctrl[controller.sides['r'].wheel_motor]:+.2f} "
                f"theta={theta[0]:+.2f}/{theta[1]:+.2f} "
                f"pitch={np.degrees(pitch):+.1f}"
            )
            next_trace += TRACE_PERIOD
        if not finite:
            break

    print(
        "transitions: "
        + " -> ".join(f"{state}@{timestamp:.3f}" for timestamp, state in transitions)
    )
    print(
        f"finite={finite} fault={controller.jump_fault} "
        f"forward_target={FORWARD_TARGET:+.2f}m/s "
        f"z_start/min/max={start_z:.3f}/{min_z:.3f}/{max_z:.3f} "
        f"airborne={airborne_time:.3f}s "
        f"apex_pitch={apex_pitch:+.2f}deg "
        f"apex_wheel_clearance={apex_wheel_clearance:.3f}m "
        f"contact_pitch={contact_pitch:+.2f}deg "
        f"touchdown_pitch={max_touchdown_pitch:.2f}deg "
        f"touchdown_wheel_step={max_touchdown_wheel_step:.2f}Nm "
        f"max_air_pitch={max_air_pitch:.2f}deg max_roll={max_roll:.2f}deg "
        f"hip_peak={max_motor_torque:.1f}Nm@{peak_motor_time:.3f}s "
        f"tail_pitch/roll={np.mean(tail_pitch):+.2f}/"
        f"{np.mean(tail_roll):+.2f}deg "
        f"tail_wheels={np.mean(tail_both_wheels):.3f} "
        f"other_ground={sorted(non_wheel_ground_bodies)} "
        f"other_times={non_wheel_ground_times} "
        f"landing_buffer={controller.landing_buffer_phase}"
    )
    if not finite:
        raise SystemExit("jump produced a non-finite physics state")
    states = {state for _, state in transitions}
    missing = {"preload", "thrust", "flight", "landing", "idle"} - states
    if missing:
        raise SystemExit(f"jump did not traverse required states: {sorted(missing)}")
    if airborne_time < 0.02:
        raise SystemExit("wheel contact never produced a confirmed airborne state")
    if max_z - start_z < 0.30:
        raise SystemExit("base jump height fell below 0.30 m")
    if apex_wheel_clearance < 0.30:
        raise SystemExit("legs did not compact enough at the jump apex")
    if abs(apex_pitch) > 2.5 or max_air_pitch > 10.0 or max_roll > 4.0:
        raise SystemExit("airborne pitch did not stay near zero")
    if not np.isfinite(contact_pitch) or abs(contact_pitch) > 8.0:
        raise SystemExit("wheel touchdown pitch exceeded 8 degrees")
    if "capture" not in landing_buffer_phases:
        raise SystemExit("first wheel contact did not enter landing capture")
    touchdown_pitch_limit = 7.0 + 15.0 * abs(FORWARD_TARGET)
    if max_touchdown_pitch > touchdown_pitch_limit:
        raise SystemExit("pitch oscillated during the touchdown capture window")
    if max_touchdown_wheel_step > 5.55:
        raise SystemExit("wheel torque changed too abruptly at touchdown")
    if non_wheel_ground_bodies:
        raise SystemExit(
            "non-wheel body touched ground: "
            + ", ".join(sorted(non_wheel_ground_bodies))
        )
    if np.mean(tail_both_wheels) < 0.95:
        raise SystemExit("both wheels were not stably grounded after landing")
    if abs(np.mean(tail_pitch)) > 1.0 or abs(np.mean(tail_roll)) > 1.0:
        raise SystemExit("post-landing LQR did not return upright")
    if np.ptp(tail_z) > 0.01:
        raise SystemExit("post-landing chassis height did not settle")
    if max_motor_torque > JUMP_HIP_MOTOR_TORQUE_LIMIT + 1e-6:
        raise SystemExit("hip motor torque exceeded the 40 Nm peak envelope")
    if peak_motor_time > 0.65:
        raise SystemExit("hip motor stayed above its 20 Nm rating too long")
    if controller.jump_fault != "none":
        raise SystemExit(f"jump finished with fault={controller.jump_fault}")


if __name__ == "__main__":
    main()
