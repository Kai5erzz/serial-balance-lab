"""Faithful port of the firmware's SWING -> RETRACT recovery path.

This stand-alone acceptance test begins at the calibrated encoder-zero
rear-leg pose.  It applies VMC motor torque only; it never teleports the
chassis to a crouched or standing pose.
"""

from __future__ import annotations

import os
from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import CONTROL_PERIOD_S, TraditionalLQR, apply_code_zero_pose


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml"
L0_TARGET = 0.130
PHI_PREPOSE = np.radians(30.0)
# The recovery state machine itself deliberately targets 80 degrees.  It
# hands the last 10 degrees to the low-leg LQR, which is how the firmware
# reaches the vertical 90-degree standing geometry without a hip-torque step.
PHI_TARGET = np.radians(80.0)

# Firmware LEG_LOW LQR gain polynomials, rows [wheel torque, hip torque] and
# columns [theta, dtheta, x, dx, pitch, dpitch].
LOW_COEFFICIENTS = np.array([
    [[44.916491, -35.029826, -0.513056, -19.580009], [9.437026, -8.422520, -0.013035, -4.818016], [0.595311, -0.423460, -0.023375, -0.528522], [1.071533, -0.777102, -0.051666, -1.640241], [51.616394, -39.448117, -0.298388, 8.137242], [8.735194, -6.686198, -0.045734, 1.468985]],
    [[10.686609, -9.829563, 0.846726, 7.607636], [8.038049, -6.525741, 0.168430, 1.483438], [0.657245, -0.556778, 0.028162, 0.101608], [2.198227, -1.799988, 0.064107, 0.325382], [-16.337782, 10.735298, 0.878556, 39.530768], [-2.982729, 1.999001, 0.141819, 6.638457]],
], dtype=float)


def ramp(value: float, target: float, delta: float) -> float:
    return float(np.clip(target, value - delta, value + delta))


def reserve_force(kin, force: float, tp: float, torque_reserve: float) -> float:
    """Port of Recovery_Simple_ForceWithTorqueReserve()."""
    tp_torque = kin.jacobian @ np.array((0.0, tp))
    force_torque = kin.jacobian @ np.array((force, 0.0))
    scale = 1.0
    for force_term, tp_term in zip(force_torque, tp_torque):
        if abs(tp_term) >= torque_reserve:
            return 0.0
        if abs(force_term) > 1e-4:
            candidate = ((torque_reserve - tp_term) / force_term if force_term > 0.0
                         else (-torque_reserve - tp_term) / force_term)
            scale = min(scale, candidate)
    return force * float(np.clip(scale, 0.0, 1.0))


def low_gain(l0: float) -> np.ndarray:
    return LOW_COEFFICIENTS @ np.array((l0**3, l0**2, l0, 1.0))


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    if os.environ.get("RECOVERY_DISABLE_LEG_LIMIT", "0") == "1":
        for name in ("l_leg", "r_leg"):
            model.jnt_limited[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)] = 0
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    ctrl = TraditionalLQR(model)
    ctrl.reset(data)
    ctrl.gas_enabled = os.environ.get("RECOVERY_GAS", "1") != "0"
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")

    last_l0 = {s: ctrl.vmc_kinematics(data, s).l0 for s in "lr"}
    last_phi = {s: ctrl.vmc_kinematics(data, s).phi0 for s in "lr"}
    l0_ref = dict(last_l0)
    phi_ref = dict(last_phi)
    phi_rate = {s: 0.0 for s in "lr"}
    # Firmware starts with REC_SIMPLE_HOLD for .22 s so the fallen/zero-pose
    # chassis can settle onto the wheels before any hip torque is applied.
    phase = "hold"
    phase_time = 0.0
    elapsed = 0.0
    angle_gain = 0.0
    force_gain = 0.0
    max_pitch = 0.0
    max_motor = 0.0
    last_body_pitch = ctrl.body_pitch(data)
    wheel_output = 0.0
    trace: list[tuple[float, str, float, float, float, float]] = []

    while data.time < 12.0:
        ctrl._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        elapsed += model.opt.timestep
        if phase == "hold":
            if elapsed + 1e-12 >= CONTROL_PERIOD_S:
                phase_time += elapsed
                elapsed = 0.0
                for side in "lr":
                    kin = ctrl.vmc_kinematics(data, side)
                    last_l0[side], last_phi[side] = kin.l0, kin.phi0
                pitch = ctrl.body_pitch(data)
                pitch_rate = ctrl._wrap_pi(pitch - last_body_pitch) / CONTROL_PERIOD_S
                wheel_output = float(np.clip(
                    float(os.environ.get("RECOVERY_WHEEL_SIGN", "1")) *
                    (float(os.environ.get("RECOVERY_WHEEL_KP", "0")) * pitch +
                     float(os.environ.get("RECOVERY_WHEEL_KD", ".35")) * pitch_rate),
                    -3.69, 3.69))
                last_body_pitch = pitch
                if phase_time >= .22:
                    phase = "swing"
                    phase_time = 0.0
            mujoco.mj_step(model, data)
            max_pitch = max(max_pitch, abs(float(np.degrees(ctrl.body_pitch(data)))))
            continue
        if phase == "handoff":
            if elapsed + 1e-12 >= CONTROL_PERIOD_S:
                dt = elapsed
                elapsed = 0.0
                # Firmware starts handoff at .35 and ramps at 1.5 / s.
                angle_gain = min(1.0, angle_gain + 1.5 * dt)
                for side in "lr":
                    ids = ctrl.sides[side]
                    kin = ctrl.vmc_kinematics(data, side)
                    d_l0 = (kin.l0 - last_l0[side]) / dt
                    state, _ = ctrl._state(data, side, dt)
                    output = angle_gain * (low_gain(kin.l0) @ (-state))
                    wheel = float(np.clip(output[0], -3.69, 3.69))
                    # Normal leg controller: gravity minus the actual spring
                    # equivalent plus the firmware low-leg length P/D term.
                    support = float(np.clip(
                        .5 * 12.801 * 9.81 - ctrl._gas_virtual_support(kin.l0) +
                        450.0 * (L0_TARGET - kin.l0) - 22.0 * d_l0,
                        -200.0, 200.0))
                    tp = float(np.clip(ctrl.vmc_tp_sign * output[1], -10.0, 10.0))
                    ctrl.set_vmc_motor_torques(data, side, support, tp)
                    data.ctrl[ids.wheel_motor] = ctrl.wheel_sign[side] * wheel
                    ctrl.last_output[side] = np.array((wheel, tp))
                    last_l0[side], last_phi[side] = kin.l0, kin.phi0
                    ctrl.previous[f"theta_{side}"] = ctrl._leg_measurement(data, ids)[0]
                    ctrl.previous[f"length_{side}"] = ctrl._leg_measurement(data, ids)[1]
                ctrl.previous["pitch"] = ctrl.body_pitch(data)
                ctrl.previous["x"] = float(np.dot(data.xpos[ctrl.base] - ctrl.base_position0, ctrl._frame(data)[1]))
            else:
                for side in "lr":
                    ids = ctrl.sides[side]
                    data.ctrl[ids.hip_motors[0]] = ctrl.last_motor_torque[side][0]
                    data.ctrl[ids.hip_motors[1]] = ctrl.last_motor_torque[side][1]
                    data.ctrl[ids.wheel_motor] = ctrl.wheel_sign[side] * ctrl.last_output[side][0]
            mujoco.mj_step(model, data)
            max_pitch = max(max_pitch, abs(float(np.degrees(ctrl.body_pitch(data)))))
            max_motor = max(max_motor, *(abs(value) for value in ctrl.last_motor_torque["l"]), *(abs(value) for value in ctrl.last_motor_torque["r"]))
            continue
        if elapsed + 1e-12 >= CONTROL_PERIOD_S:
            dt = elapsed
            elapsed = 0.0
            phase_time += dt
            all_prepose = True
            all_stand = True
            for side in "lr":
                kin = ctrl.vmc_kinematics(data, side)
                d_l0 = (kin.l0 - last_l0[side]) / dt
                d_phi = ctrl._wrap_pi(kin.phi0 - last_phi[side]) / dt

                # Recovery_SwingAxisRun: 135 deg/s, 360 deg/s^2, braking
                # for remaining travel. Target is final standing angle even
                # while the 30-degree prepose gate is active.
                remaining = max(0.0, PHI_TARGET - phi_ref[side])
                omega_target = min(np.radians(135.0), np.sqrt(2.0 * np.radians(360.0) * remaining))
                phi_rate[side] = ramp(phi_rate[side], omega_target, np.radians(360.0) * dt)
                phi_ref[side] = min(PHI_TARGET, phi_ref[side] + phi_rate[side] * dt)

                spring_ff = min(140.0, max(0.0, ctrl._gas_virtual_support(kin.l0)))
                if phase == "swing":
                    angle_gain = min(1.0, angle_gain + 1.5 * dt)
                    force = 0.0
                    tp = angle_gain * (70.0 * (phi_ref[side] - kin.phi0) + 1.6 * (phi_rate[side] - d_phi))
                    tp += angle_gain * (phi_rate[side] / np.radians(135.0)) * 3.0
                    tp = float(np.clip(tp, -15.0, 15.0))
                    motor_limit = 20.0
                    all_prepose &= (PHI_PREPOSE - kin.phi0) <= np.radians(4.0)
                else:
                    # Firmware retraction references: .25 m/s one-way, 0.9/s
                    # force gain, 2.0/s angle gain, spring feedforward.
                    angle_gain = min(1.0, angle_gain + 2.0 * dt)
                    force_gain = min(1.0, force_gain + 0.9 * dt)
                    l0_ref[side] = max(L0_TARGET, min(kin.l0, l0_ref[side] - 0.25 * dt))
                    bias_scale = float(np.clip((kin.l0 - L0_TARGET) / 0.08, 0.0, 1.0))
                    force = force_gain * (1200.0 * (l0_ref[side] - kin.l0) - 30.0 * d_l0 - spring_ff - 40.0 * bias_scale)
                    # 180 N breakaway pulse (0.18 s), then 100 N sustain.
                    breakaway = 180.0 if phase_time < 0.18 else 100.0
                    if phase_time < 0.18:
                        force = min(force, -breakaway)
                    force = float(np.clip(force, -breakaway, 20.0))

                    settle = float(np.clip((0.20 - kin.l0) / 0.02, 0.0, 1.0))
                    kp = 70.0 + (32.0 - 70.0) * settle
                    kd = 1.6 + (5.5 - 1.6) * settle
                    limit = 15.0 + (10.0 - 15.0) * settle
                    tp = angle_gain * (kp * (phi_ref[side] - kin.phi0) + kd * (phi_rate[side] - d_phi))
                    tp = float(np.clip(tp, -limit, limit))
                    reserve = 24.0 if phase_time < 0.18 else 19.0
                    force = reserve_force(kin, force, tp, reserve)
                    motor_limit = reserve
                    # Firmware RETRACT gate: low enough and within 10 deg;
                    # the final vertical alignment is owned by low-leg LQR.
                    all_stand &= kin.l0 < .170 and abs(PHI_TARGET - kin.phi0) < np.radians(10.0)

                ctrl.set_vmc_motor_torques(data, side, force, tp, torque_limit=motor_limit)
                last_l0[side], last_phi[side] = kin.l0, kin.phi0
            if phase == "swing" and all_prepose:
                phase = "retract"
                phase_time = 0.0
                angle_gain = max(angle_gain, .30)
                force_gain = 0.0
            elif phase == "retract" and all_stand:
                phase = "handoff"
                angle_gain = .35
                ctrl.reset(data)
            trace.append((data.time, phase, ctrl.vmc_kinematics(data, "l").l0,
                          np.degrees(ctrl.vmc_kinematics(data, "l").phi0),
                          ctrl.last_motor_torque["l"][0], ctrl.last_motor_torque["l"][1]))
            pitch = ctrl.body_pitch(data)
            pitch_rate = ctrl._wrap_pi(pitch - last_body_pitch) / dt
            wheel_output = float(np.clip(
                float(os.environ.get("RECOVERY_WHEEL_SIGN", "1")) *
                (float(os.environ.get("RECOVERY_WHEEL_KP", "0")) * pitch +
                 float(os.environ.get("RECOVERY_WHEEL_KD", ".35")) * pitch_rate),
                -3.69, 3.69))
            last_body_pitch = pitch
            for side in "lr":
                data.ctrl[ctrl.sides[side].wheel_motor] = ctrl.wheel_sign[side] * wheel_output
        else:
            for side in "lr":
                ids = ctrl.sides[side]
                data.ctrl[ids.hip_motors[0]] = ctrl.last_motor_torque[side][0]
                data.ctrl[ids.hip_motors[1]] = ctrl.last_motor_torque[side][1]
                data.ctrl[ids.wheel_motor] = ctrl.wheel_sign[side] * wheel_output
        mujoco.mj_step(model, data)
        max_pitch = max(max_pitch, abs(float(np.degrees(ctrl.body_pitch(data)))))
        max_motor = max(max_motor, *(abs(value) for value in ctrl.last_motor_torque["l"]), *(abs(value) for value in ctrl.last_motor_torque["r"]))

    kin = {s: ctrl.vmc_kinematics(data, s) for s in "lr"}
    print(f"phase={phase}; finite={np.isfinite(data.qpos).all()}; t={data.time:.2f}; contacts={data.ncon}")
    print(f"base_z={data.xpos[base, 2]:.4f}; pitch={np.degrees(ctrl.body_pitch(data)):+.2f}; max_pitch={max_pitch:.2f}")
    print(f"l0={kin['l'].l0:.4f}/{kin['r'].l0:.4f}; phi_deg={np.degrees(kin['l'].phi0):.2f}/{np.degrees(kin['r'].phi0):.2f}")
    print(f"max_motor_Nm={max_motor:.3f}; last_tau_l={ctrl.last_motor_torque['l']}; last_tau_r={ctrl.last_motor_torque['r']}")
    for row in trace[::max(1, len(trace) // 10)]:
        print("trace t=%.2f phase=%s l0=%.3f phi=%.1f tau=(%.1f,%.1f)" % row)


if __name__ == "__main__":
    main()
