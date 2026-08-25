"""Batch-search physically actuated jump air-attitude allocations."""

from __future__ import annotations

from dataclasses import dataclass
import os
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    WHEEL_RADIUS_M,
    apply_saved_pose,
)


MODEL = Path(os.environ.get(
    "MUJOCO_MODEL",
    Path(__file__).resolve().parent / "model" / "whole_body_lqr_fast.xml",
))
END_TIME = float(os.environ.get("TUNE_JUMP_END_TIME", "4.0"))


@dataclass(frozen=True)
class Config:
    name: str
    hip_kp: float
    hip_kd: float
    theta_tuck: float
    theta_deploy: float
    wheel_kp: float
    wheel_kd: float
    wheel_sign: float
    thrust_kp: float = 8.0
    thrust_kd: float = 12.0
    thrust_limit: float = 30.0
    thrust_rate: float = 2.00
    thrust_rate_max: float = 0.90
    thrust_rate_start_l0: float = 0.22
    thrust_rate_full_l0: float = 0.30
    thrust_pitch_slew: float = 50.0
    thrust_theta_kd: float = 0.0
    thrust_theta_kp: float = 18.0
    thrust_theta_limit: float = 22.0
    theta_kp: float = 18.0
    theta_kd: float = 8.0
    theta_limit: float = 22.0
    common_limit: float = 34.0
    hip_limit: float = 30.0
    tuck_length: float = 0.20
    tuck_time: float = 0.11
    tuck_hold: float = 0.12
    tuck_rate: float = 1.50
    landing_length: float = 0.27
    deploy_rate: float = 1.20
    deploy_kp: float = 700.0
    deploy_kd: float = 20.0
    retract_limit: float = 135.0
    retract_peak: float = 180.0
    tuck_pitch_bias_deg: float = 0.0
    deploy_pitch_bias_deg: float = 0.0
    pitch_reference_limit_deg: float = 15.0
    wheel_theta_ff: float = 0.0


CONFIGS = [
    Config(
        f"deploy_bias{deploy_bias:.0f}",
        195,
        13,
        2.0,
        8.0,
        20,
        2,
        1,
        thrust_kd=20,
        thrust_limit=40,
        thrust_rate=1.48,
        thrust_rate_max=1.48,
        thrust_rate_start_l0=.16,
        thrust_rate_full_l0=.26,
        thrust_pitch_slew=100,
        thrust_theta_kp=0,
        tuck_length=.14,
        tuck_rate=3.5,
        deploy_kp=1000,
        deploy_kd=26,
        retract_limit=200,
        retract_peak=260,
        tuck_pitch_bias_deg=12,
        deploy_pitch_bias_deg=deploy_bias,
        pitch_reference_limit_deg=30,
        hip_limit=40,
        wheel_theta_ff=.50,
        theta_kp=80,
        theta_kd=16,
        theta_limit=40,
        common_limit=40,
    )
    for deploy_bias in (
        6,
        8,
        10,
        12,
        14,
        15,
        16,
        18,
    )
]


def simulate(model: mujoco.MjModel, config: Config) -> dict[str, float | str | bool]:
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)
    controller.jump_air_pitch_kp = config.hip_kp
    controller.jump_air_pitch_kd = config.hip_kd
    controller.jump_air_pitch_limit = config.hip_limit
    controller.jump_air_tuck_pitch_bias = np.radians(
        config.tuck_pitch_bias_deg
    )
    controller.jump_air_deploy_pitch_bias = np.radians(
        config.deploy_pitch_bias_deg
    )
    controller.jump_air_pitch_reference_limit = np.radians(
        config.pitch_reference_limit_deg
    )
    controller.jump_air_theta_weight_tuck = config.theta_tuck
    controller.jump_air_theta_weight_deploy = config.theta_deploy
    controller.jump_air_wheel_pitch_kp = config.wheel_kp
    controller.jump_air_wheel_pitch_kd = config.wheel_kd
    controller.jump_air_wheel_sign = config.wheel_sign
    controller.jump_air_wheel_theta_ff = config.wheel_theta_ff
    controller.jump_thrust_pitch_kp = config.thrust_kp
    controller.jump_thrust_pitch_kd = config.thrust_kd
    controller.jump_thrust_pitch_limit = config.thrust_limit
    controller.jump_thrust_pitch_rate_target = config.thrust_rate
    controller.jump_thrust_pitch_rate_max = config.thrust_rate_max
    controller.jump_thrust_pitch_rate_start_l0 = config.thrust_rate_start_l0
    controller.jump_thrust_pitch_rate_full_l0 = config.thrust_rate_full_l0
    controller.jump_thrust_pitch_tp_slew_rate = config.thrust_pitch_slew
    controller.jump_thrust_theta_kd = config.thrust_theta_kd
    controller.jump_thrust_theta_kp = config.thrust_theta_kp
    controller.jump_thrust_theta_limit = config.thrust_theta_limit
    controller.jump_air_theta_kp = config.theta_kp
    controller.jump_air_theta_kd = config.theta_kd
    controller.jump_air_theta_limit = config.theta_limit
    controller.jump_air_common_tp_limit = config.common_limit
    controller.jump_air_tuck_length = config.tuck_length
    controller.jump_air_tuck_time = config.tuck_time
    controller.jump_air_tuck_hold_time = config.tuck_hold
    controller.jump_air_tuck_ref_rate = config.tuck_rate
    controller.jump_air_landing_length = config.landing_length
    controller.jump_air_deploy_ref_rate = config.deploy_rate
    controller.jump_air_deploy_kp = config.deploy_kp
    controller.jump_air_deploy_kd = config.deploy_kd
    controller.jump_air_retract_force_limit = config.retract_limit
    controller.jump_air_retract_force_peak = config.retract_peak
    controller.jump_abort_pitch_limit = np.pi
    controller.jump_abort_roll_limit = np.pi

    while data.time < 1.0:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
    accepted, reason = controller.begin_jump(data)
    if not accepted:
        return {"name": config.name, "error": reason}

    start_z = float(data.xpos[controller.base, 2])
    apex_z = start_z
    apex_pitch = 0.0
    apex_wheel_clearance = 0.0
    apex_l0 = 0.0
    apex_theta = 0.0
    max_air_pitch = 0.0
    min_air_pitch = np.inf
    max_signed_air_pitch = -np.inf
    first_contact_pitch = np.nan
    first_contact_theta = np.nan
    first_contact_time = np.nan
    first_ground = "none"
    first_ground_time = np.nan
    any_other_after_takeoff = False
    other_ground_labels: set[str] = set()
    takeoff_pitch_rate = np.nan
    max_motor = 0.0
    peak_motor_time = 0.0
    tail_pitch: list[float] = []
    tail_roll: list[float] = []
    tail_z: list[float] = []
    tail_both_contact: list[bool] = []
    tail_other_contact: list[bool] = []
    ground_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    wheel_bodies = {ids.wheel_body for ids in controller.sides.values()}

    def other_ground_contact() -> bool:
        for index in range(data.ncon):
            contact = data.contact[index]
            if contact.geom1 == ground_geom:
                other_geom = int(contact.geom2)
            elif contact.geom2 == ground_geom:
                other_geom = int(contact.geom1)
            else:
                continue
            if int(model.geom_bodyid[other_geom]) not in wheel_bodies:
                return True
        return False
    was_airborne = False
    end_time = END_TIME
    while data.time < end_time:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        z = float(data.xpos[controller.base, 2])
        pitch = float(np.degrees(controller.body_pitch(data)))
        roll = float(np.degrees(controller.body_roll(data)))
        if z > apex_z:
            apex_z = z
            apex_pitch = pitch
            apex_wheel_clearance = min(
                float(data.xpos[ids.wheel_body, 2]) - WHEEL_RADIUS_M
                for ids in controller.sides.values()
            )
            apex_l0 = 0.5 * sum(
                controller.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            )
            pitch_rad = controller.body_pitch(data)
            apex_theta = float(np.degrees(0.5 * sum(
                np.pi / 2.0
                - controller.vmc_kinematics(data, side).phi0
                + pitch_rad
                for side in ("l", "r")
            )))
        if controller.jump_off_ground:
            if not was_airborne:
                takeoff_pitch_rate = float(np.degrees(
                    -controller.last_state["l"][5]
                ))
            was_airborne = True
            max_air_pitch = max(max_air_pitch, abs(pitch))
            min_air_pitch = min(min_air_pitch, pitch)
            max_signed_air_pitch = max(max_signed_air_pitch, pitch)
        if was_airborne:
            ground_labels: list[str] = []
            for contact_index in range(data.ncon):
                contact = data.contact[contact_index]
                if ground_geom not in (contact.geom1, contact.geom2):
                    continue
                other_geom = (
                    contact.geom2
                    if contact.geom1 == ground_geom
                    else contact.geom1
                )
                other_body = int(model.geom_bodyid[other_geom])
                if other_body in wheel_bodies:
                    ground_labels.append("wheel")
                else:
                    any_other_after_takeoff = True
                    label = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_BODY, other_body
                    ) or f"body#{other_body}"
                    ground_labels.append(label)
                    other_ground_labels.add(label)
            if ground_labels and np.isnan(first_ground_time):
                first_ground_time = float(data.time)
                first_ground = ",".join(sorted(set(ground_labels)))
        if (
            was_airborne
            and np.isnan(first_contact_time)
            and any(controller.jump_raw_wheel_contact.values())
        ):
            first_contact_time = float(data.time)
            first_contact_pitch = pitch
            pitch_rad = controller.body_pitch(data)
            first_contact_theta = float(np.degrees(0.5 * sum(
                np.pi / 2.0
                - controller.vmc_kinematics(data, side).phi0
                + pitch_rad
                for side in ("l", "r")
            )))
        motor = max(
            abs(float(value))
            for values in controller.last_motor_torque.values()
            for value in values
        )
        max_motor = max(max_motor, motor)
        peak_motor_time += model.opt.timestep * int(motor > 20.0)
        if data.time >= end_time - 0.5:
            tail_pitch.append(pitch)
            tail_roll.append(roll)
            tail_z.append(z)
            tail_both_contact.append(all(controller._wheel_contacts(data).values()))
            tail_other_contact.append(other_ground_contact())

    return {
        "name": config.name,
        "apex_dz": apex_z - start_z,
        "apex_pitch": apex_pitch,
        "apex_wheel": apex_wheel_clearance,
        "apex_l0": apex_l0,
        "apex_theta": apex_theta,
        "max_air_pitch": max_air_pitch,
        "min_air_pitch": min_air_pitch,
        "max_signed_air_pitch": max_signed_air_pitch,
        "contact_t": first_contact_time,
        "takeoff_rate": takeoff_pitch_rate,
        "contact_pitch": first_contact_pitch,
        "contact_theta": first_contact_theta,
        "first_ground": first_ground,
        "first_ground_t": first_ground_time,
        "other_after": any_other_after_takeoff,
        "other_labels": ",".join(sorted(other_ground_labels)) or "none",
        "tail_pitch": float(np.mean(tail_pitch)),
        "tail_roll": float(np.mean(tail_roll)),
        "tail_z_span": float(np.ptp(tail_z)),
        "tail_contact": float(np.mean(tail_both_contact)),
        "tail_other": float(np.mean(tail_other_contact)),
        "off_end": controller.jump_off_ground,
        "fault": controller.jump_fault,
        "max_motor": max_motor,
        "peak_time": peak_motor_time,
    }


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    for config in CONFIGS:
        result = simulate(model, config)
        print(
            "{name:27s} apex dz={apex_dz:.3f} wheel={apex_wheel:.3f} "
            "P={apex_pitch:+5.1f}[{min_air_pitch:+4.0f},"
            "{max_signed_air_pitch:+4.0f}] "
            "L/th={apex_l0:.3f}/{apex_theta:+4.0f} "
            "takeoff_dP={takeoff_rate:+5.1f} "
            "first={first_ground}@{first_ground_t:.3f} "
            "other={other_after}:{other_labels} "
            "contact P/theta={contact_pitch:+5.1f}/{contact_theta:+5.1f} "
            "tail P/R={tail_pitch:+5.1f}/{tail_roll:+4.1f} "
            "zspan={tail_z_span:.3f} wheels={tail_contact:.2f} other={tail_other:.2f} "
            "off={off_end} fault={fault} motor={max_motor:.1f}@{peak_time:.2f}s".format(
                **result
            )
        )


if __name__ == "__main__":
    main()
