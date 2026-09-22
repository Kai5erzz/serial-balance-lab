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
from balance_lab.control.tuning import (
    _LEGACY_ZERO_Q,
    VMC_DM_ZERO_PHI1,
    VMC_DM_ZERO_PHI4,
    VMC_L1,
    VMC_L2,
    VMC_ZERO_PHI1,
    VMC_ZERO_PHI4,
)


MODEL = REPO_ROOT / "model" / "whole_body_lqr.xml"


def _legacy_fk(q_j1: float, q_j2: float, side: str) -> tuple[float, float]:
    """The pre-migration mapping: DM_ZERO_OFFSET + s*(Q - Z_old)."""
    raw = -1.0 if side == "l" else 1.0
    phi1 = (VMC_DM_ZERO_PHI1 + raw * (q_j2 - _LEGACY_ZERO_Q[f"{side}_j2"])) % (2.0 * np.pi)
    phi4 = (VMC_DM_ZERO_PHI4 + raw * (q_j1 - _LEGACY_ZERO_Q[f"{side}_j1"])) % (2.0 * np.pi)
    xb, yb = VMC_L1 * np.cos(phi1), VMC_L1 * np.sin(phi1)
    xd, yd = VMC_L1 * np.cos(phi4), VMC_L1 * np.sin(phi4)
    dx, dy = xd - xb, yd - yb
    a0, b0 = 2.0 * VMC_L2 * dx, 2.0 * VMC_L2 * dy
    c0 = dx * dx + dy * dy
    discriminant = max(0.0, a0 * a0 + b0 * b0 - c0 * c0)
    phi2 = 2.0 * np.arctan2(b0 + np.sqrt(discriminant), a0 + c0)
    xc, yc = xb + VMC_L2 * np.cos(phi2), yb + VMC_L2 * np.sin(phi2)
    l0 = max(float(np.hypot(xc, yc)), 1e-5)
    return l0, float(np.arctan2(yc, xc))


def legacy_fk_identity(controller, model, data, qadr) -> None:
    """Z_new migration is a pure re-anchor: FK on every pose Q is unchanged.

    RL_ZERO_UNIFICATION_PLAN section 4 requires this proof: the new
    ``VMC_ZERO_PHI*`` (per side, anchored at Z_new) must reproduce the legacy
    ``DM_ZERO_OFFSET + s*(Q - Z_old)`` mapping bit-for-bit within float
    assembly noise, so rod angles, leg length, Jacobian and every downstream
    calibration keep their meaning.
    """
    rng = np.random.default_rng(20260913)
    zeros = {name: float(data.qpos[qadr[name]]) for name in qadr}  # at Z_new now
    worst_l0 = 0.0
    worst_phi = 0.0
    for _ in range(256):
        # stay inside the real workspace around the calibrated poses so the
        # comparison exercises normal configurations, not fold singularities
        values = {
            "l_j1": rng.uniform(0.2, 1.5), "l_j2": rng.uniform(1.1, 2.4),
            "r_j1": rng.uniform(-1.5, -0.2), "r_j2": rng.uniform(-2.4, -1.1),
        }
        for name, value in values.items():
            data.qpos[qadr[name]] = value
        mujoco.mj_forward(model, data)
        for side in ("l", "r"):
            kin = controller.vmc_kinematics(data, side)
            old_l0, old_phi0 = _legacy_fk(
                float(data.qpos[qadr[f"{side}_j1"]]),
                float(data.qpos[qadr[f"{side}_j2"]]),
                side,
            )
            worst_l0 = max(worst_l0, abs(kin.l0 - old_l0))
            worst_phi = max(worst_phi, abs(kin.phi0 - old_phi0))
    np.testing.assert_allclose(worst_l0, 0.0, atol=1e-12)
    np.testing.assert_allclose(worst_phi, 0.0, atol=1e-12)
    # explicit anchor spot-check: controller at Z_new == legacy at Z_new
    for name, value in zeros.items():
        data.qpos[qadr[name]] = value
    mujoco.mj_forward(model, data)
    for side in ("l", "r"):
        kin = controller.vmc_kinematics(data, side)
        old_l0, old_phi0 = _legacy_fk(zeros[f"{side}_j1"], zeros[f"{side}_j2"], side)
        np.testing.assert_allclose(kin.l0, old_l0, atol=1e-12)
        np.testing.assert_allclose(kin.phi0, old_phi0, atol=1e-12)
    print(f"legacy FK identity PASS (worst dl0={worst_l0:.3e} m, dphi={worst_phi:.3e} rad)")


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

    # Z_new (E=0): l0 is shared, phi0 is PER SIDE -- the confirmed zero
    # posture lets the two legs swing independently (1.424531 deg left,
    # 0.221243 deg right vs the legacy zero), so 18.495 deg symmetric no
    # longer holds there.
    for joint, value in VMC_ENCODER_ZERO_CAD_Q.items():
        data.qpos[qadr[joint]] = value
    mujoco.mj_forward(model, data)
    zero_values = []
    for side in ("l", "r"):
        kin = controller.vmc_kinematics(data, side)
        np.testing.assert_allclose(kin.l0, VMC_ENCODER_ZERO_L0, atol=2e-12)
        np.testing.assert_allclose(kin.phi0, VMC_ENCODER_ZERO_PHI0[side], atol=2e-12)
        zero_values.append((side, kin.l0, np.degrees(kin.phi0)))
    print("encoder_zero", zero_values)

    legacy_fk_identity(controller, model, data, qadr)

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
