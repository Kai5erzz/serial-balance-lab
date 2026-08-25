"""Acceptance check for the screenshot-calibrated zero -> LQR path."""

from pathlib import Path

import mujoco
import numpy as np

from traditional_lqr import (
    CODE_ZERO_ENCODER_RAD,
    TraditionalLQR,
    apply_code_zero_pose,
    encoder_joint_reading,
)


MODEL = Path(__file__).resolve().parent / "model" / "whole_body_lqr.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    apply_code_zero_pose(model, data)
    controller = TraditionalLQR(model)
    controller.reset(data, calibrate_frame=True)

    for name, expected in CODE_ZERO_ENCODER_RAD.items():
        actual = encoder_joint_reading(model, data, name)
        if not np.isclose(actual, expected, atol=1e-9):
            raise AssertionError(f"{name}: expected {expected:+.3f}, got {actual:+.9f}")

    controller.enable_cad_zero_lqr(data)
    pitch_deg: list[float] = []
    phi0_error_deg: list[float] = []
    unexpected_contacts: set[tuple[str, str]] = set()
    wheel_contact_steps = 0
    for _ in range(round(3.0 / model.opt.timestep)):
        controller.step(data, model.opt.timestep)
        mujoco.mj_step(model, data)
        pitch_deg.append(float(np.degrees(controller.body_pitch(data))))
        for side in ("l", "r"):
            phi0_error_deg.append(abs(
                float(np.degrees(controller.vmc_kinematics(data, side).phi0)) - 90.0
            ))
        wheel_contact_this_step = False
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            names = []
            for geom_id in (contact.geom1, contact.geom2):
                body_id = int(model.geom_bodyid[geom_id])
                names.append(
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                    or "world"
                )
            pair = tuple(names)
            if "l_wheel" in pair or "r_wheel" in pair:
                wheel_contact_this_step = True
            else:
                unexpected_contacts.add(pair)
        wheel_contact_steps += int(wheel_contact_this_step)

    peak = max(abs(value) for value in pitch_deg)
    final = pitch_deg[-1]
    peak_phi0_error = max(phi0_error_deg)
    wheel_contact_ratio = wheel_contact_steps / len(pitch_deg)
    if (
        peak >= 5.0
        or abs(final) >= 0.25
        or peak_phi0_error >= 2.0
        or unexpected_contacts
        or wheel_contact_ratio < 0.95
    ):
        raise AssertionError(
            f"zero-start failed: peak={peak:.3f} deg final={final:.3f} deg "
            f"peak_phi0_error={peak_phi0_error:.3f} deg "
            f"wheel_contact_ratio={wheel_contact_ratio:.3f} "
            f"unexpected_contacts={sorted(unexpected_contacts)}"
        )
    print(
        f"PASS encoder zero -> LQR: peak={peak:.3f} deg, "
        f"final={final:.3f} deg, peak_phi0_error={peak_phi0_error:.3f} deg, "
        f"wheel_contact_ratio={wheel_contact_ratio:.3f}"
    )


if __name__ == "__main__":
    main()
