"""Saved-pose loading and the DM-encoder frame translation.

These helpers place the floating robot at one of the calibrated poses and
translate MJCF hinge coordinates into the calibrated DM multi-turn encoder
frame used by the firmware and the viewer overlay.
"""

from __future__ import annotations

import json
from pathlib import Path

import mujoco
import numpy as np

from ..control.tuning import (
    CODE_ZERO_ENCODER_RAD,
    ENCODER_ZERO_POSE_PATH,
    JOINT_FORWARD_SIGN,
    WHEEL_RADIUS_M,
)
from ..control.kinematics import unit


def encoder_joint_reading(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    """Translate one MJCF hinge coordinate into the calibrated DM frame."""
    if joint_name not in CODE_ZERO_ENCODER_RAD:
        raise KeyError(f"No calibrated encoder zero for {joint_name}")
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    q = float(data.qpos[model.jnt_qposadr[joint]])
    return CODE_ZERO_ENCODER_RAD[joint_name] + JOINT_FORWARD_SIGN[joint_name] * q

def apply_saved_pose(model: mujoco.MjModel, data: mujoco.MjData, pose_path: Path) -> None:
    """Load a complete loop-closed pose and place its lowest wheel on ground."""
    if not pose_path.is_file():
        raise FileNotFoundError(f"Missing saved pose: {pose_path}")
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    base_address = int(model.jnt_qposadr[base_joint])
    data.qpos[base_address:base_address + 7] = pose["base_free"]
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if name in pose:
            data.qpos[model.jnt_qposadr[joint]] = pose[name]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    wheel_heights = []
    for side in ("l", "r"):
        wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel")
        wheel_heights.append(float(data.xpos[wheel, 2] - WHEEL_RADIUS_M))
    # The calibrated joint pose is independent of global translation. Put its
    # lowest wheel exactly on the z=0 ground plane before starting a mode.
    data.qpos[base_address + 2] -= min(wheel_heights)
    mujoco.mj_forward(model, data)

def apply_code_zero_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Load the CAD pose corresponding to the calibrated encoder screenshot.

    At this pose the MJCF drive hinges are zero while ``encoder_joint_reading``
    reports r_j1/r_j2=+5.349/+4.615 and l_j1/l_j2=-5.310/-4.593 rad.
    """
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    wheel_heights = []
    for side in ("l", "r"):
        wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel")
        wheel_heights.append(float(data.xpos[wheel, 2] - WHEEL_RADIUS_M))
    base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    data.qpos[int(model.jnt_qposadr[base_joint]) + 2] -= min(wheel_heights)
    mujoco.mj_forward(model, data)

def apply_encoder_zero_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Load the unified physical encoder zero Z_new with both four-bars closed.

    The 2026-09-13 user-confirmed posture: chassis belly flat, both legs
    against the rear stops, both 58 mm tires on the same ground plane (hip
    encoders read zero there).  The VMC reads ``l0=0.332557082 m`` on both
    legs and a per-side ``phi0`` (17.070469 / 18.716243 deg) because the
    confirmed zero lets the two legs swing independently.  It is intentionally
    separate from the legacy CAD-q-zero experiment above, which predates the
    user's three-pose VMC calibration.
    """
    apply_saved_pose(model, data, ENCODER_ZERO_POSE_PATH)

def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the active 3-D rotation about a unit-independent axis."""
    axis = unit(axis)
    skew = np.array(((0.0, -axis[2], axis[1]),
                     (axis[2], 0.0, -axis[0]),
                     (-axis[1], axis[0], 0.0)))
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)

def apply_prone_zero_pose(model: mujoco.MjModel, data: mujoco.MjData, controller: "TraditionalLQR", direction: float = -1.0) -> None:
    """Place CAD q=0 on its chassis face for a physical self-righting test.

    Only the initial free-base pose is set.  Every following recovery motion
    is produced by torque actuators, the gas springs, and mesh contacts.
    ``direction`` accepts either front/back prone orientation; both were
    validated against the same pulse and contact-gated LQR handoff.
    """
    apply_code_zero_pose(model, data)
    controller.reset(data, calibrate_frame=True)
    _, _, lateral = controller._frame(data)
    base_rotation = data.xmat[controller.base].reshape(3, 3)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(
        quaternion,
        (_axis_angle_rotation(lateral, float(np.sign(direction) or -1.0) * np.pi / 2.0) @ base_rotation).ravel(),
    )
    base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    base_address = int(model.jnt_qposadr[base_joint])
    data.qpos[base_address + 3:base_address + 7] = quaternion
    # The exterior is asymmetric.  These values place the two face-down
    # orientations at a 2 mm mesh-contact preload, not embedded in the floor.
    data.qpos[base_address + 2] = 0.2365 if direction < 0.0 else 0.3605
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    controller.reset(data)  # retain the calibrated fixed world-forward axis
