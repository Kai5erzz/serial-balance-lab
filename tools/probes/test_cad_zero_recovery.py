"""Headless regression test for direct LQR enablement at CAD zero."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import TraditionalLQR, apply_code_zero_pose


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"
PULSE_START_S = 1.75
PULSE_DURATION_S = 0.10
END_SECONDS = 8.0


def body_has_ground_contact(model: mujoco.MjModel, data: mujoco.MjData, body_name: str) -> bool:
    """Return whether any active contact joins the named body to world."""
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    for index in range(data.ncon):
        contact = data.contact[index]
        bodies = {
            int(model.geom_bodyid[contact.geom1]),
            int(model.geom_bodyid[contact.geom2]),
        }
        if body_id in bodies and 0 in bodies:
            return True
    return False


def run_trial(model: mujoco.MjModel, pulse_torque: float) -> None:
    """Run the same zero-clearance start used by the interactive viewer."""
    data = mujoco.MjData(model)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data, calibrate_frame=True)
    controller.enabled = False
    controller.stance_enabled = False
    controller.gas_enabled = False

    drive_joint_values = {
        name: float(data.qpos[model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name,
        )]])
        for name in ("l_j1", "l_j2", "r_j1", "r_j2")
    }
    if any(value != 0.0 for value in drive_joint_values.values()):
        raise SystemExit(f"not at CAD joint zero: {drive_joint_values}")

    controller.enable_cad_zero_lqr(data)
    if not controller.enabled or controller.cad_stand_wheel_origin is None:
        raise SystemExit("LQR was not armed synchronously at CAD zero")

    startup_peak = 0.0
    pulse_peak = 0.0
    wheel_peak = 0.0
    chassis_touched = False
    while data.time < END_SECONDS:
        controller.step(data, model.opt.timestep)
        if PULSE_START_S <= data.time < PULSE_START_S + PULSE_DURATION_S:
            _, _, lateral = controller._frame(data)
            mujoco.mj_applyFT(
                model,
                data,
                np.zeros(3),
                pulse_torque * lateral,
                data.xpos[controller.base],
                controller.base,
                data.qfrc_applied,
            )
        mujoco.mj_step(model, data)
        pitch = abs(float(np.degrees(controller.body_pitch(data))))
        wheel_peak = max(
            wheel_peak,
            abs(float(controller.last_output["l"][0])),
            abs(float(controller.last_output["r"][0])),
        )
        if data.time < PULSE_START_S:
            startup_peak = max(startup_peak, pitch)
        else:
            pulse_peak = max(pulse_peak, pitch)
        chassis_touched |= body_has_ground_contact(model, data, "base_link")

    pitch = float(np.degrees(controller.body_pitch(data)))
    print(
        f"pulse={pulse_torque:+.1f} Nm finite={np.isfinite(data.qpos).all()} "
        f"state={controller.recovery_state} startup_peak={startup_peak:.3f} deg "
        f"pulse_peak={pulse_peak:.3f} deg final={pitch:+.3f} deg "
        f"wheel_peak={wheel_peak:.3f} Nm chassis_touch={chassis_touched} "
        f"contacts={data.ncon}"
    )
    if not np.isfinite(data.qpos).all():
        raise SystemExit("non-finite physics state")
    if controller.recovery_state != "stand" or not controller.enabled:
        raise SystemExit("did not reach the LQR standing state")
    if startup_peak > 6.0:
        raise SystemExit(f"CAD-zero startup tipped too far: {startup_peak:.3f} deg")
    if chassis_touched:
        raise SystemExit("chassis touched the floor during LQR startup")
    if wheel_peak < 0.05:
        raise SystemExit("wheel LQR never produced a meaningful command")
    if pulse_peak < 1.0:
        raise SystemExit("pitch pulse did not exercise the controller")
    if abs(pitch) > 0.1 or data.ncon < 2:
        raise SystemExit("standing trim did not recover after the pulse")


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    rejected_data = mujoco.MjData(model)
    apply_code_zero_pose(model, rejected_data)
    rejected_controller = TraditionalLQR(model)
    rejected_controller.reset(rejected_data, calibrate_frame=True)
    rejected_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "l_j1")
    rejected_data.qpos[model.jnt_qposadr[rejected_joint]] = 1e-3
    try:
        rejected_controller.enable_cad_zero_lqr(rejected_data)
    except ValueError:
        pass
    else:
        raise SystemExit("nonzero CAD joint was incorrectly accepted")

    for pulse_torque in (-2.0, 2.0):
        run_trial(model, pulse_torque)


if __name__ == "__main__":
    main()
