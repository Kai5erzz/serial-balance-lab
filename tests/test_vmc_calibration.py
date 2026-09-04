"""Deterministic checks for the user-measured firmware/CAD VMC anchors."""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from pathlib import Path

import mujoco
import numpy as np

from balance_lab import (
    TraditionalLQR,
    VMC_COMPRESSION_MAX,
    VMC_COMPRESSION_MIN,
    VMC_ENCODER_ZERO_CAD_Q,
    VMC_ENCODER_ZERO_COMPRESSION,
    VMC_ENCODER_ZERO_L0,
    VMC_ENCODER_ZERO_PHI0,
    VMC_ENCODER_ZERO_SWING,
    VMC_LENGTH_MAX,
    VMC_LENGTH_MIN,
    VMC_NORMAL_STAND_COMPRESSION,
    VMC_NORMAL_STAND_LENGTH,
    VMC_VERTICAL_SWING,
)


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL))
    data = mujoco.MjData(model)
    controller = TraditionalLQR(model)
    controller.reset(data, calibrate_frame=True)
    qadr = {
        name: int(model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )])
        for name in ("r_j1", "r_j2", "l_j1", "l_j2")
    }

    def set_pose(swing: float, compression: float) -> None:
        target = {
            "r_j1": swing + compression,
            "r_j2": swing - compression,
            "l_j1": -swing - compression,
            "l_j2": -swing + compression,
        }
        for name, value in target.items():
            data.qpos[qadr[name]] = value
        mujoco.mj_forward(model, data)

    def check_pose(name: str, expected_l0: float, expected_phi0: float) -> None:
        values = []
        for side in ("l", "r"):
            kin = controller.vmc_kinematics(data, side)
            np.testing.assert_allclose(kin.l0, expected_l0, atol=2e-12)
            np.testing.assert_allclose(kin.phi0, expected_phi0, atol=2e-12)
            values.append((side, kin.l0, np.degrees(kin.phi0)))
        print(name, values)

    for joint, value in VMC_ENCODER_ZERO_CAD_Q.items():
        data.qpos[qadr[joint]] = value
    mujoco.mj_forward(model, data)
    check_pose("encoder_zero", VMC_ENCODER_ZERO_L0, VMC_ENCODER_ZERO_PHI0)

    set_pose(VMC_VERTICAL_SWING, VMC_COMPRESSION_MAX)
    check_pose("vertical_max", VMC_LENGTH_MAX, np.pi / 2.0)

    set_pose(VMC_VERTICAL_SWING, VMC_COMPRESSION_MIN)
    check_pose("vertical_min", VMC_LENGTH_MIN, np.pi / 2.0)

    set_pose(VMC_VERTICAL_SWING, VMC_NORMAL_STAND_COMPRESSION)
    check_pose("normal_stand", VMC_NORMAL_STAND_LENGTH, np.pi / 2.0)

    # The returned 2x2 matrix must be d[l0,phi0]/d[J1,J2] transposed, so the
    # same calibrated kinematics are used for both feedback and virtual work.
    epsilon = 1e-7
    for side in ("l", "r"):
        kin = controller.vmc_kinematics(data, side)
        numerical = np.zeros((2, 2))
        for column, joint in enumerate((f"{side}_j1", f"{side}_j2")):
            address = qadr[joint]
            original = float(data.qpos[address])
            samples = []
            for delta in (-epsilon, epsilon):
                data.qpos[address] = original + delta
                mujoco.mj_forward(model, data)
                perturbed = controller.vmc_kinematics(data, side)
                samples.append(np.array((perturbed.l0, perturbed.phi0)))
            data.qpos[address] = original
            numerical[:, column] = (samples[1] - samples[0]) / (2.0 * epsilon)
        mujoco.mj_forward(model, data)
        np.testing.assert_allclose(kin.jacobian, numerical.T, rtol=2e-6, atol=2e-7)
        print(side, "jacobian", kin.jacobian)

    print("VMC calibration PASS")


if __name__ == "__main__":
    main()
