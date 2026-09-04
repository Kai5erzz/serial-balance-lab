# rl-lab — minimal-gap simulation validation platform for the wheeled-leg balance robot

MuJoCo whole-body simulator of the five-bar (wheeled-leg balance) infantry
robot, positioned as the **low-cost pre-flight bench for traditional
control algorithms**: firmware-exact LQR/VMC, recovery, jump, and terrain
safety logic run here against a calibrated plant so ideas are validated in
simulation before touching the real vehicle.

The controller is a faithful port of the embedded source
(`chassis_control.c`, `chassis_recovery.c`, `chassis_safety.c`,
`rm_config.h`); all known sim/real divergences are documented, and the
system-identification pipeline (see [docs/SYSID.md](docs/SYSID.md))
 progressively replaces nominal plant parameters with measured ones —
that is what "minimal gap" means here: **the model differs from the robot
only where a parameter has not been identified yet, and every such place
is written down.**

## Layout

```
balance_lab/            the simulation platform (Python package)
├── control/            firmware controller, split by concern
│   ├── tuning.py       every firmware constant, source-named, one place
│   ├── kinematics.py   five-bar VMC types + helpers
│   ├── controller.py   TraditionalLQR core (LQR + VMC + state)
│   ├── jump.py         jump state machine (preload→thrust→flight→landing)
│   ├── guards.py       off-ground detection, ledge guard, danger check
│   └── recovery.py     encoder-zero + fall recovery state machines
├── plant/              pose loading + load_model(variant, plant=...)
├── sysid/              identified-parameter profiles and application
└── sim/                interactive viewer session (run_session)
model/                  MuJoCo XMLs + meshes (full / fast / platform)
tests/                  regression battery — must stay green
docs/                   architecture, sysid pipeline, development notes
tools/                  model builders, calibration, probe archaeology
run_lqr.py              interactive viewer entry point
traditional_lqr.py      compatibility shim → balance_lab (legacy imports)
```

Documentation index:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) — module responsibilities,
  data flow, and a recipe for adding a new controller.
- [docs/SYSID.md](docs/SYSID.md) — identified-parameter pipeline, what is
  applied where, and what is still nominal.
- [docs/RECOVERY_PORT_NOTES.md](docs/RECOVERY_PORT_NOTES.md) — calibration
  anchors, the recovery port, and the system-identification checklist.
- [docs/JUMP_CONTROL_NOTES.md](docs/JUMP_CONTROL_NOTES.md) — jump phase
  allocation, torque envelope, acceptance results.
- [docs/DESCENT_NOTES.md](docs/DESCENT_NOTES.md) — stair descent/climb
  development record (root causes R1–R3, mode-7 feasibility).
- [tools/README.md](tools/README.md) — what the archived scripts are.

## Quick start (Linux)

```bash
cd /home/kaiser/robomaster/rl-lab
source /home/kaiser/miniconda3/etc/profile.d/conda.sh
conda activate rl-lab
python -m pip install -r requirements.txt
python run_lqr.py          # interactive viewer
```

Headless machines: run the regression battery instead (below).

The viewer starts paused at the calibrated encoder-zero recovery pose.
Press `Space` to execute recovery and hand off to LQR.

## Viewer controls

- `Space`: run/pause. `R`: reset current mode. `F`: follow/free camera.
- `1` hover inspection (fixed base) · `2` free physics · `3` direct
  calibrated LOW-LQR start · `4` suspended leg calibration (`W/S` swing,
  `D/A` compression; mechanical stops disabled in this mode only) ·
  `5` encoder zero → firmware recovery → LQR · `6` 600 mm platform scene.
- `7/8/9` leg length presets `0.130/0.240/0.300 m` (slew-limited).
- `Up/Down` forward speed, `Left/Right` yaw rate, `X` stop commands.
- `J` firmware jump sequence (interlocked to a near-stationary launch;
  drive/yaw cleared through touchdown).
- `P` 2 N·m pitch pulse in LQR modes.
- `Z/M/C/N` calibration presets (encoder zero / vertical max / min / stand).

The overlay reports real encoder-frame values, l0/phi0, contacts, recovery
phase, VMC F0/Tp, hip torques, LQR gain, and real-time factor. A small
pitch alone is never treated as balance — acceptance also requires upright
phi0, correct leg length, wheel contact, and no chassis/leg support
contact.

## Regression battery

```bash
python tests/test_vmc_calibration.py       # VMC anchors and Jacobians
python tests/test_firmware_recovery.py     # encoder zero → recovery → LQR
python tests/test_calibrated_lqr.py        # 0.15 m stand + pitch pulse
python tests/test_drive_controls.py        # drive/yaw commands
python tests/test_firmware_jump.py         # physically actuated jump
python tests/test_inertia_integrity.py     # CAD inertia guards
python tests/test_sysid.py                 # identified-parameter pipeline
python tests/smoke_test.py                 # finite-value sanity
```

Every test must pass after any controller or plant change; the recovery
and jump tests additionally rerun against the full collision model:

```bash
export MUJOCO_MODEL="$PWD/model/whole_body_lqr.xml"
python tests/test_firmware_recovery.py
python tests/test_firmware_jump.py
unset MUJOCO_MODEL
```

## Plant profiles (nominal vs identified)

The regression battery runs on the **nominal** plant — the XML as
committed — which is what the firmware tuning was validated against.
Running on the identified plant is one explicit argument:

```python
from balance_lab.plant import load_model

model, applied = load_model("fast", plant="identified_2026-09-04")
# applied: record of every parameter written; empty for plant="nominal"
```

See [docs/SYSID.md](docs/SYSID.md) for the parameter→consumer mapping and
the current identification status (PRBS actuator fit done; coupled
mechanics reserved for the real2sim pass).

## Calibration summary

- Encoder zero: `l0=0.332557082 m`, `phi0=18.495 deg`
  (`encoder_zero_pose.json`, the active real-zero pose).
- Vertical calibrated range: `0.116399057..0.332557082 m` at `phi0=90 deg`.
- Normal stance: `l0=0.150 m`, `phi0=90 deg`, level chassis, wheels down.
- `code_zero_pose.json` is a retained legacy diagnostic; it is not used by
  recovery or the accepted LQR path.

## Historical scope (Windows origin)

This project was developed on Windows (`D:\User\balance_infantry\...`);
the notes keep their original reproduce commands where relevant. All
current development happens on Linux/conda.
