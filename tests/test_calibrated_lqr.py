"""Regression test for the user-calibrated 0.15 m LQR equilibrium."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from dataclasses import dataclass
from pathlib import Path

import mujoco
import numpy as np

from balance_lab import (
    CALIBRATED_STAND_POSE_PATH,
    TraditionalLQR,
    VMC_NORMAL_STAND_LENGTH,
    WHEEL_TORQUE_LIMIT,
    apply_saved_pose,
)


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"


@dataclass
class Result:
    max_pitch_deg: float
    max_roll_deg: float
    max_phi_error_deg: float
    min_length: float
    max_length: float
    final_lengths: tuple[float, float]
    final_phi_deg: tuple[float, float]
    final_pitch_deg: float
    final_roll_deg: float
    wheel_contact_ratio: float
    wheel_saturation_ratio: float
    non_wheel_ground_bodies: set[str]


def simulate(
    duration: float = 8.0,
    pulse: bool = False,
    pulse_torque_nm: float = 2.0,
) -> Result:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    apply_saved_pose(model, data, CALIBRATED_STAND_POSE_PATH)
    controller.reset(data, calibrate_frame=True)
    controller.enable_calibrated_lqr(data)

    floor = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    wheel_bodies = {ids.wheel_body for ids in controller.sides.values()}
    base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
    _, _, lateral = controller._frame(data)
    steps = int(round(duration / model.opt.timestep))
    max_pitch = max_roll = max_phi_error = 0.0
    min_length = np.inf
    max_length = -np.inf
    wheel_contact_samples = saturated_samples = 0
    non_wheel_ground_bodies: set[str] = set()

    for _ in range(steps):
        controller.step(data, model.opt.timestep)
        if pulse and 1.0 <= data.time < 1.1:
            mujoco.mj_applyFT(
                model,
                data,
                np.zeros(3),
                pulse_torque_nm * lateral,
                data.xpos[base],
                base,
                data.qfrc_applied,
            )
        mujoco.mj_step(model, data)

        max_pitch = max(max_pitch, abs(float(np.degrees(controller.body_pitch(data)))))
        max_roll = max(max_roll, abs(float(np.degrees(controller.body_roll(data)))))
        lengths = []
        for side in ("l", "r"):
            kin = controller.vmc_kinematics(data, side)
            lengths.append(kin.l0)
            phi_error = (kin.phi0 - np.pi / 2.0 + np.pi) % (2.0 * np.pi) - np.pi
            max_phi_error = max(max_phi_error, abs(float(np.degrees(phi_error))))
        min_length = min(min_length, *lengths)
        max_length = max(max_length, *lengths)
        saturated_samples += int(any(
            abs(float(controller.last_output[side][0])) >= WHEEL_TORQUE_LIMIT - 1e-6
            for side in ("l", "r")
        ))

        wheels_touching: set[int] = set()
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            if floor not in (contact.geom1, contact.geom2):
                continue
            other_geom = contact.geom2 if contact.geom1 == floor else contact.geom1
            other_body = int(model.geom_bodyid[other_geom])
            if other_body in wheel_bodies:
                wheels_touching.add(other_body)
            else:
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, other_body)
                non_wheel_ground_bodies.add(name or f"body#{other_body}")
        wheel_contact_samples += int(wheels_touching == wheel_bodies)

    final_lengths = tuple(controller.vmc_kinematics(data, side).l0 for side in ("l", "r"))
    final_phi = tuple(
        float(np.degrees(controller.vmc_kinematics(data, side).phi0))
        for side in ("l", "r")
    )
    return Result(
        max_pitch,
        max_roll,
        max_phi_error,
        float(min_length),
        float(max_length),
        final_lengths,
        final_phi,
        float(np.degrees(controller.body_pitch(data))),
        float(np.degrees(controller.body_roll(data))),
        wheel_contact_samples / steps,
        saturated_samples / steps,
        non_wheel_ground_bodies,
    )


def main() -> None:
    for pulse in (False, True):
        result = simulate(pulse=pulse)
        print(f"pulse={pulse}: {result}")
        assert not result.non_wheel_ground_bodies
        assert result.max_pitch_deg < 1.0
        assert result.max_roll_deg < 1.0
        assert result.max_phi_error_deg < 10.0
        assert result.min_length > 0.145
        # Start transient with the MATLAB wheels (mw=0.577 kg): the initial
        # leg overshoot reaches 0.181 m; the 0.180 line was tuned for the old
        # light-wheel plant.  The leg still returns to the 0.150 stand.
        assert result.max_length < 0.185
        assert abs(np.mean(result.final_lengths) - VMC_NORMAL_STAND_LENGTH) < 0.010
        assert result.wheel_contact_ratio > 0.99
        assert result.wheel_saturation_ratio < 0.01


if __name__ == "__main__":
    main()
