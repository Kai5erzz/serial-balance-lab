"""Scan closed-chain CAD workspace while preserving the two driven motors."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import json
from pathlib import Path

import mujoco
import numpy as np

from explore_recovery_poses import solve_pose
from traditional_lqr import VMC_BACK_ZERO_OFFSET, VMC_FRONT_ZERO_OFFSET


ROOT = REPO_ROOT


def two_link_angle(x: float, y: float, branch: int) -> float:
    radius = float(np.hypot(x, y))
    alpha = float(np.arctan2(y, x))
    cosine = np.clip((.210**2 + radius**2 - .250**2) / (2.0 * .210 * radius), -1.0, 1.0)
    beta = float(np.arccos(cosine))
    return alpha + beta if branch == 0 else alpha - beta


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(ROOT / "model" / "whole_body_lqr.xml"))
    data = mujoco.MjData(model)
    zero = json.loads((ROOT / "code_zero_pose.json").read_text(encoding="utf-8"))
    baseline = data.qpos.copy()
    baseline[:7] = zero["base_free"]
    for joint in range(1, model.njnt):
        name = model.joint(joint).name
        if name in zero:
            baseline[model.jnt_qposadr[joint]] = zero[name]
    adr = {name: model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]
           for name in ("r_j1", "r_j2", "r_leg", "l_j1", "l_j2", "l_leg")}
    for phi_deg in (30, 45, 60, 75, 90):
        for l0 in (.13, .17, .21, .25, .29, .33):
            x, y = l0 * np.cos(np.radians(phi_deg)), l0 * np.sin(np.radians(phi_deg))
            phi1, phi4 = two_link_angle(x, y, 0), two_link_angle(x, y, 1)
            drive = {
                "r_j1": zero["r_j1"] + phi1 - VMC_FRONT_ZERO_OFFSET,
                "r_j2": zero["r_j2"] + phi4 + VMC_BACK_ZERO_OFFSET,
                "l_j1": zero["l_j1"] + VMC_FRONT_ZERO_OFFSET - phi1,
                "l_j2": zero["l_j2"] - VMC_BACK_ZERO_OFFSET - phi4,
            }
            result = solve_pose(model, data, baseline, drive)
            if result is None:
                print(f"phi={phi_deg:2d} l0={l0:.2f} no closure")
                continue
            print(f"phi={phi_deg:2d} l0={l0:.2f} rleg={result[adr['r_leg']]:+.3f} lleg={result[adr['l_leg']]:+.3f}")


if __name__ == "__main__":
    main()
