"""Headless regression for calibrated encoder zero -> firmware recovery -> LQR."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import os
from pathlib import Path

import mujoco
import numpy as np

from balance_lab import (
    TraditionalLQR,
    VMC_ENCODER_ZERO_L0,
    VMC_ENCODER_ZERO_PHI0,
    apply_encoder_zero_pose,
)


MODEL = Path(os.environ.get(
    "MUJOCO_MODEL",
    REPO_ROOT / "model" / "whole_body_lqr_fast.xml",
))
RUN_SECONDS = float(os.environ.get("RECOVERY_RUN_SECONDS", "12.0"))
TRACE = os.environ.get("RECOVERY_TRACE") == "1"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    apply_encoder_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data, calibrate_frame=True)
    controller.begin_encoder_zero_recovery(data)

    initial_contacts = []
    for contact_id in range(data.ncon):
        contact = data.contact[contact_id]
        bodies = []
        for geom_id in (contact.geom1, contact.geom2):
            body_id = int(model.geom_bodyid[geom_id])
            bodies.append(
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                or "world"
            )
        initial_contacts.append(tuple(bodies))
    print(
        f"initial base_z={data.xpos[controller.base, 2]:.6f} "
        f"contacts={initial_contacts}"
    )

    initial = {
        side: controller.vmc_kinematics(data, side) for side in ("l", "r")
    }
    for side, kin in initial.items():
        np.testing.assert_allclose(kin.l0, VMC_ENCODER_ZERO_L0, atol=2e-8)
        np.testing.assert_allclose(
            kin.phi0, VMC_ENCODER_ZERO_PHI0[side], atol=2e-8
        )  # per-side zero anchor (2026-09-13 Z_new: asymmetric phi0 at E=0)

    phase = controller.recovery_state
    transitions = [(
        data.time,
        phase,
        controller.vmc_kinematics(data, "l").l0,
        np.degrees(controller.vmc_kinematics(data, "l").phi0),
    )]
    max_pitch = 0.0
    max_motor_torque = 0.0
    min_retract_l0 = {"l": float("inf"), "r": float("inf")}
    next_trace = 0.0
    while data.time < RUN_SECONDS:
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        if controller.recovery_state == "fw_retract":
            for side in ("l", "r"):
                min_retract_l0[side] = min(
                    min_retract_l0[side],
                    controller.vmc_kinematics(data, side).l0,
                )
        if controller.recovery_state != phase:
            phase = controller.recovery_state
            transitions.append((
                data.time,
                phase,
                controller.vmc_kinematics(data, "l").l0,
                np.degrees(controller.vmc_kinematics(data, "l").phi0),
            ))
        max_pitch = max(
            max_pitch,
            abs(float(np.degrees(controller.body_pitch(data)))),
        )
        max_motor_torque = max(
            max_motor_torque,
            *(abs(float(value)) for value in controller.last_motor_torque["l"]),
            *(abs(float(value)) for value in controller.last_motor_torque["r"]),
        )
        if TRACE and data.time + 1e-9 >= next_trace:
            contacts = []
            contact_forward = []
            for contact_id in range(data.ncon):
                contact = data.contact[contact_id]
                for geom_id in (contact.geom1, contact.geom2):
                    if int(model.geom_bodyid[geom_id]) == 0:
                        continue
                    name = mujoco.mj_id2name(
                        model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
                    )
                    body = mujoco.mj_id2name(
                        model,
                        mujoco.mjtObj.mjOBJ_BODY,
                        int(model.geom_bodyid[geom_id]),
                    )
                    contacts.append(name or body)
                    contact_forward.append(float(
                        np.dot(
                            contact.pos - data.xpos[controller.base],
                            controller.forward_world,
                        )
                    ))
            kin = controller.vmc_kinematics(data, "l")
            print(
                f"t={data.time:.3f} {controller.recovery_state:10s} "
                f"pitch={np.degrees(controller.body_pitch(data)):+6.2f} "
                f"l0={kin.l0:.3f} phi={np.degrees(kin.phi0):5.1f} "
                f"F={controller.recovery_output['l'][0]:+6.1f} "
                f"Tp={controller.recovery_output['l'][1]:+5.1f} "
                f"contacts={','.join(dict.fromkeys(contacts))} "
                f"contact_x={min(contact_forward, default=0):+.3f}/"
                f"{max(contact_forward, default=0):+.3f}"
            )
            next_trace += .05

    final = {
        side: controller.vmc_kinematics(data, side) for side in ("l", "r")
    }
    pitch = float(np.degrees(controller.body_pitch(data)))
    finite = bool(np.isfinite(data.qpos).all() and np.isfinite(data.qvel).all())
    print("transitions:", " -> ".join(
        f"{state}@{timestamp:.3f}s(l0={length:.3f},phi={phi:.1f})"
        for timestamp, state, length, phi in transitions
    ))
    print(
        f"finite={finite} state={controller.recovery_state} "
        f"gain={controller.lqr_gain:.3f} pitch={pitch:+.3f}deg "
        f"max_pitch={max_pitch:.3f}deg max_motor={max_motor_torque:.3f}Nm"
    )
    print(
        "l0=" + "/".join(f"{final[side].l0:.6f}" for side in ("l", "r"))
        + " phi0=" + "/".join(
            f"{np.degrees(final[side].phi0):.3f}" for side in ("l", "r")
        )
    )
    print(
        "length_ref=" + "/".join(
            f"{controller.length_reference[side]:.6f}" for side in ("l", "r")
        )
    )
    print(
        "min_retract_l0=" + "/".join(
            f"{min_retract_l0[side]:.6f}" for side in ("l", "r")
        )
    )
    if controller.fw_recovery_axes:
        print(
            "phi_rate=" + "/".join(
                f"{np.degrees(controller.fw_recovery_axes[side].omega_actual):.1f}deg/s"
                for side in ("l", "r")
            )
        )
    if not finite:
        raise SystemExit("recovery produced a non-finite physics state")
    if controller.recovery_state != "fw_stand":
        raise SystemExit(f"recovery did not reach fw_stand: {controller.recovery_state}")
    if abs(pitch) > 2.0:
        raise SystemExit(f"final pitch is outside LQR basin: {pitch:+.3f} deg")
    if max_pitch >= 25.0:
        raise SystemExit(
            f"recovery crossed the firmware pitch fault gate: {max_pitch:.3f} deg"
        )
    for side, kin in final.items():
        if abs(kin.l0 - 0.150) > 0.020:
            raise SystemExit(f"{side} final l0 did not converge to stand: {kin.l0:.6f}")
        phi_deg = float(np.degrees(kin.phi0))
        if not 80.0 <= phi_deg <= 95.0:
            raise SystemExit(f"{side} final phi0 is not upright: {phi_deg:.3f} deg")
    if max_motor_torque < 1.0:
        raise SystemExit("recovery never generated a meaningful hip torque")
    for side, length in min_retract_l0.items():
        if not np.isfinite(length) or length >= 0.170:
            raise SystemExit(
                f"{side} recovery handed off before retracting: l0={length:.6f}"
            )


if __name__ == "__main__":
    main()
