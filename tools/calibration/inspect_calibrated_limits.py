"""Solve the passive four-bar coordinates at the calibrated length stops."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import json
from pathlib import Path

import mujoco

from explore_recovery_poses import solve_pose
from traditional_lqr import (
    CALIBRATED_STAND_POSE_PATH,
    VMC_COMPRESSION_MAX,
    VMC_COMPRESSION_MIN,
    VMC_VERTICAL_SWING,
)


ROOT = REPO_ROOT
MODEL_PATH = ROOT / "model" / "whole_body_lqr.xml"


def drive_pose(swing: float, compression: float) -> dict[str, float]:
    return {
        "r_j1": swing + compression,
        "r_j2": swing - compression,
        "l_j1": -swing - compression,
        "l_j2": -swing + compression,
    }


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    source = json.loads(CALIBRATED_STAND_POSE_PATH.read_text(encoding="utf-8"))
    baseline = model.qpos0.copy()
    baseline[:7] = source["base_free"]
    for joint_id in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
        if name in source:
            baseline[model.jnt_qposadr[joint_id]] = source[name]

    drive_addresses = {
        name: int(model.jnt_qposadr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_JOINT, name
        )])
        for name in drive_pose(0.0, 0.0)
    }
    for label, target in (
        ("vertical_max", drive_pose(VMC_VERTICAL_SWING, VMC_COMPRESSION_MAX)),
        ("vertical_min", drive_pose(VMC_VERTICAL_SWING, VMC_COMPRESSION_MIN)),
    ):
        result = baseline.copy()
        start = {name: float(result[address]) for name, address in drive_addresses.items()}
        for step in range(1, 81):
            alpha = step / 80.0
            drive = {
                name: start[name] + alpha * (target[name] - start[name])
                for name in target
            }
            result = solve_pose(model, data, result, drive)
            if result is None:
                raise RuntimeError(f"{label} failed at alpha={alpha:.3f}")
        data.qpos[:] = result
        mujoco.mj_forward(model, data)
        values = {}
        for name in ("l_leg", "r_leg"):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            values[name] = float(result[model.jnt_qposadr[joint_id]])
        print(label, values)


if __name__ == "__main__":
    main()
