# tools/ — archived scripts

These are development-time scripts kept for reproducibility and
archaeology. They are **not** part of the platform package and not covered
by the regression battery. Each file carries a small bootstrap header so
it runs from its new location exactly as it did from the repo root:

```bash
python tools/model/build_fast_model.py        # rebuild model/whole_body_lqr_fast.xml
python tools/probes/probe_plant_force.py      # one-off plant probes
python tools/calibration/generate_encoder_zero_pose.py
```

All of them still import the controller through the
`traditional_lqr.py` compatibility shim at the repo root; new code should
import `balance_lab` directly.

## model/

Build and repair the MuJoCo assets in `model/`.

- `build_model.py` / `build_fast_model.py` / `build_platform_model.py` —
  generate the full-collision, fast-viewer, and platform-scene XMLs.
- `make_collision_meshes.py`, `make_exterior_collision_mesh.py`,
  `decompose_base_collision.py`, `split_base_collision.py` — collision
  mesh generation (COACD/VHACD decomposition of the CAD body).
- `compare_revised_urdf.py`, `rebuild_urdf_from_revised.py`,
  `restore_original_structure.py` — URDF-export comparison and repair.
- `repair_inertias.py`, `update_wheel_inertia.py`,
  `estimate_gimbal_inertia.py`, `extract_log_inertia.py`,
  `parse_log_links.py` — inertia extraction/verification from embedded
  logs (imported by `tests/test_inertia_integrity.py`).

## calibration/

Pose and anchor tooling behind the calibrated constants in
`balance_lab/control/tuning.py`.

- `explore_recovery_poses.py` — pose solver (`solve_pose`) used by the
  generators below.
- `generate_encoder_zero_pose.py` → `encoder_zero_pose.json` (the active
  real-zero pose), `generate_recovery_crouch_pose.py`,
  `generate_vmc_stand_target.py`, `solve_code_zero_pose.py`.
- `find_target_equilibrium.py`, `scan_vmc_workspace.py`,
  `inspect_calibrated_limits.py`, `inspect_pose_clearance.py` —
  workspace/equilibrium scans and limit checks.

## probes/

One-off diagnostics, polarity hunts, and behaviour experiments. The
`probe_*.py` scripts and the older `test_*.py` files here predate the
regression battery or were falsification experiments; the conclusions
that survived live in `docs/DESCENT_NOTES.md` and
`docs/RECOVERY_PORT_NOTES.md`. Notables:

- `test_platform_descent*.py`, `climb_chain.py`, `mode7_*`, `stepup_*`,
  `viz_*` — the stair descent/climb experiments (2026-08-27 sessions).
- `tune_jump_air.py` — airborne gain sweep behind the accepted jump.
- `test_upright_recovery.py`, `test_prone_recovery.py`,
  `test_recovery_handoff.py` — earlier recovery-route acceptances,
  superseded by `tests/test_firmware_recovery.py`.
- polarity/sign hunts: `test_control_polarity.py`,
  `test_wheel_polarity.py`, `test_vmc_motor_polarity.py`,
  `probe_*polarity*`, `probe_*sign*`, `probe_drive_*`.
