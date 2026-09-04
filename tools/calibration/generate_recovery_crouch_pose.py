"""Generate the loop-closed compact-leg starting pose for recovery tests."""

from __future__ import annotations

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import json
from pathlib import Path

import mujoco

from explore_recovery_poses import DRIVE, FORWARD_SIGN, solve_pose


ROOT = REPO_ROOT
MODEL_PATH = ROOT / "model" / "whole_body_lqr.xml"
ZERO_PATH = ROOT / "code_zero_pose.json"
OUTPUT = ROOT / "recovery_crouch_pose.json"

# Compared with code zero this folds both virtual legs from about 0.315 m to
# about 0.135 m while retaining a closed four-bar mechanism on each side.
SWING = 0.0
FOLD = -0.50


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    zero = json.loads(ZERO_PATH.read_text(encoding="utf-8"))
    baseline = data.qpos.copy()
    baseline[:7] = zero["base_free"]
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if name in zero:
            baseline[model.jnt_qposadr[joint]] = zero[name]
    drive = {
        name: zero[name] + FORWARD_SIGN[name] * (SWING + (FOLD if name.endswith("j1") else -FOLD))
        for name in DRIVE
    }
    result = solve_pose(model, data, baseline, drive)
    if result is None:
        raise RuntimeError("Could not close the compact recovery pose")
    data.qpos[:] = result
    mujoco.mj_forward(model, data)
    pose = {"base_free": result[:7].tolist()}
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        pose[name] = float(result[model.jnt_qposadr[joint]])
    OUTPUT.write_text(json.dumps(pose, indent=2), encoding="utf-8")
    print(f"Wrote {OUTPUT}")
    for name in DRIVE:
        print(f"{name}={pose[name]:+.6f}")


if __name__ == "__main__":
    main()
