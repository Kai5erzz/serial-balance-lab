# Architecture

This document explains how the platform is organised, how data flows, and
how to plug new work in. It targets the next developer (including future
you) and assumes only MuJoCo basics.

## Design goals

1. **Minimal sim-to-real gap, honestly tracked.** Every deviation between
   plant and robot is either identified (`balance_lab/sysid`) or written
   down (`docs/RECOVERY_PORT_NOTES.md` § system identification). The
   nominal plant never drifts silently: `tests/test_sysid.py` pins the
   shipped XML damping values and the profile fingerprint.
2. **Firmware is the source of truth.** Controller constants keep their
   firmware names and live in one place (`control/tuning.py`), so the port
   stays diffable against the C source. Deviations carry a comment and are
   listed in the notes.
3. **Low integration cost.** A new traditional controller needs only the
   model, the pose helpers, and the VMC/LQR primitives — see the recipe
   below. The regression battery is the safety net that makes changes
   cheap: run it, and behaviour drift is caught immediately.

## Package map

```
balance_lab/
├── paths.py       repo/model/pose file locations (single source)
├── control/       everything that computes torques
│   ├── tuning.py      ~400 firmware constants, sectioned, star-imported
│   ├── kinematics.py  unit(), SideIds, VmcKinematics, RecoverySwingAxis
│   ├── controller.py  TraditionalLQR core: state estimation, LQR, VMC,
│   │                  gains, gas springs, theta compensation, step()
│   ├── jump.py        JumpControlMixin: preload/thrust/flight/landing
│   ├── guards.py      TerrainSafetyMixin: off-ground, descent guard,
│   │                  danger check (all mixins share one `self` state)
│   └── recovery.py    RecoveryMixin: encoder-zero, REC_SIMPLE fall
│                      recovery, cad-stand, prone swing
├── plant/
│   ├── poses.py       apply_saved_pose / apply_encoder_zero_pose /
│   │                  apply_code_zero_pose / apply_prone_zero_pose,
│   │                  encoder_joint_reading (MJCF ↔ DM encoder frame)
│   └── __init__.py    load_model(variant, plant=...) → (MjModel, applied)
├── sysid/
│   ├── parameters.py  schema-v1 loader + fingerprint (rl-serial-balance
│   │                  compatible)
│   ├── apply.py       parameter → MjModel / controller application
│   └── profiles/      identified_*.yaml (committed measurement sets)
└── sim/
    ├── session.py     interactive viewer session (modes, keys, loop)
    └── viewer.py      main() entry, session restart on scene switch
```

`TraditionalLQR` is one class composed of the core plus three mixins; the
split is purely by responsibility, all methods read/write the same
controller state through `self`. Cross-concern calls (danger check →
recovery entry, landing → jump state) are ordinary `self.method()` calls.

### Why `from .tuning import *`

The controller modules deliberately star-import `tuning.py`: the tuning
surface is one flat namespace of firmware names (`JUMP_THRUST_LENGTH`,
`FW_RECOVERY_SWING_KP`, …), and keeping that namespace shared across the
four behaviour modules is what makes the C-port reviewable. Do not add
logic to `tuning.py`; constants only.

## Data flow

```
identified_*.yaml ──load_profile()──▶ ModelParameters
                                        │
              ┌─────────────────────────┤ apply_model_parameters()
              ▼                         ▼
        MjModel (plant)          TraditionalLQR.apply_identified()
              │                         │
              └────────┬────────────────┘
                       ▼
        controller.step(data, dt)  at CONTROL_PERIOD_S = 5 ms
                       ▼
        data.ctrl (wheel/hip motors) + qfrc_applied (gas springs)
                       ▼
        mujoco.mj_step(model, data)  at 1 ms physics
```

Model variants (`load_model`): `"fast"` (primitive collisions, default),
`"full"` (mesh collisions, regression-grade), `"platform"` (fast + 600 mm
stair terrain). The environment variable `MUJOCO_MODEL` overrides the
model file inside the regression tests, exactly as before.

## Compatibility shim

`traditional_lqr.py` at the repo root re-exports everything the old
monolith exported (415 names verified). It exists so the archived probe
scripts in `tools/probes/` and any old notebooks keep working. New code
imports from `balance_lab` directly.

## Adding a new controller (recipe)

The typical case: try a new traditional controller (new gain schedule,
MPC on the LQR model, a different attitude loop, …) against the
calibrated plant.

1. **New module next to the concern.** For a variant of existing
   behaviour, subclass or add a mixin in `balance_lab/control/`. For a
   standalone experiment, create `tools/probes/my_experiment.py` —
   probes are for pre-integration experiments; promote into
   `balance_lab/` only once they earn a regression test.
2. **Build the plant.**
   ```python
   from balance_lab.plant import load_model
   model, applied = load_model("fast")            # nominal
   model, applied = load_model("fast", plant="identified_2026-09-04")
   ```
3. **Place the robot.** Use `balance_lab.plant.poses`:
   `apply_encoder_zero_pose` (prone start), `apply_saved_pose` +
   `CALIBRATED_STAND_POSE_PATH` (standing start), `controller.reset`.
4. **Reuse the primitives.** VMC kinematics
   (`controller.vmc_kinematics(data, side)`), torque mapping
   (`controller.set_vmc_motor_torques`), state
   (`controller._state(data, side, dt)`), gas springs, and the gain
   schedule (`controller.gain(l0)`) are all methods on the controller —
   call them from your loop instead of reimplementing the Jacobian.
5. **Step the same way the platform does:**
   `controller.step(data, model.opt.timestep)` then `mujoco.mj_step`.
   Feedback is sampled/held at 5 ms; physics runs at 1 ms.
6. **Gate the acceptance.** `controller.is_properly_standing(data)` is the
   honest "is it up" oracle (wheels bearing, phi≈90°, l0≤0.22, z≥0.19,
   level). Pitch alone lies — a belly-flat chassis reads zero.
7. **Add a regression** in `tests/` mirroring an existing one
   (`test_calibrated_lqr.py` is the smallest template) and wire it into
   the battery list in `README.md`.

Rules of thumb that keep the gap small (extracted from the notes):

- Never compensate missing plant dynamics with controller-side
  multipliers; add the missing dynamics to the plant instead.
- Never teleport qpos as "control"; forces through the actuators only.
- A gait must never pass through the belly-rest attractor (see
  [docs/DESCENT_NOTES.md](docs/DESCENT_NOTES.md), night III).
- The wheel output path contains a per-side mount mirror
  (`wheel_sign = {l:+1, r:-1}`); any externally injected wheel term must
  be checked against the sign chain documented in
  [docs/DESCENT_NOTES.md](docs/DESCENT_NOTES.md) § R3.

## Testing strategy

- `tests/` is the battery: run it before and after any change. Recovery
  and jump tests are behavioural (12 s headless sims with hard acceptance
  lines), which is why the mixin split could be verified bit-for-bit.
- `tests/test_sysid.py` additionally pins: the bundled profile
  fingerprint (cross-repo provenance), the shipped XML nominal damping
  (scale semantics of the real2sim fitter), application idempotence, and
  an identified-plant balance smoke test.
- Platform-scene coverage: `export MUJOCO_MODEL=$PWD/model/whole_body_lqr.xml`
  reruns recovery/jump on the full collision model (see README).
