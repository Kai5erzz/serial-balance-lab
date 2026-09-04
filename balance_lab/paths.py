"""Filesystem locations for models, calibration poses, and profiles.

The whole repository resolves its data files through this module so scripts
can run from any working directory.  Calibration poses stay at the repository
root: they are user-editable artifacts referenced by the notes.
"""

from __future__ import annotations

from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
MODEL_DIR = REPO_ROOT / "model"

MODEL_FULL = MODEL_DIR / "whole_body_lqr.xml"
MODEL_FAST = MODEL_DIR / "whole_body_lqr_fast.xml"
MODEL_FAST_PLATFORM = MODEL_DIR / "whole_body_lqr_fast_platform.xml"

CODE_ZERO_POSE_PATH = REPO_ROOT / "code_zero_pose.json"
ENCODER_ZERO_POSE_PATH = REPO_ROOT / "encoder_zero_pose.json"
RECOVERY_CROUCH_POSE_PATH = REPO_ROOT / "recovery_crouch_pose.json"
CALIBRATED_STAND_POSE_PATH = REPO_ROOT / "calibrated_stand_pose.json"
