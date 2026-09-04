# Whole-body MuJoCo LQR and recovery

This project runs the calibrated five-bar VMC, the embedded LOW LQR, and the
firmware encoder-zero recovery route on the complete floating robot. The fast
viewer model retains the robot's visual meshes while using lightweight
primitive collision; the full collision model remains available for
regression.

The detailed calibration, temporary recovery factors, and future
system-identification checklist are in
[RECOVERY_PORT_NOTES.md](RECOVERY_PORT_NOTES.md).
The jump phase allocation, torque envelope, and measured acceptance results
are in [JUMP_CONTROL_NOTES.md](JUMP_CONTROL_NOTES.md).
Stair-descent development status, the approach-brake design, the mode-7
climb feasibility memo, and remaining tuning are in
[DESCENT_NOTES.md](DESCENT_NOTES.md).

## Linux start

```bash
cd /home/kaiser/robomaster/rl-lab
source /home/kaiser/miniconda3/etc/profile.d/conda.sh
conda activate rl-lab
python -m pip install -r requirements.txt
python run_lqr.py
```

The viewer requires a graphical Linux session. For a headless machine, use
`python smoke_test.py` and the regression commands below.

## Windows start

```powershell
cd D:\User\balance_infantry\mujoco_lqr_whole_body
python run_lqr.py
```

The viewer starts paused at the calibrated encoder-zero recovery pose. Press
`Space` to execute recovery and automatically hand off to LQR.

## Viewer controls

- `Space`: run or pause.
- `R`: reset the current mode.
- `1`: hover inspection; fixed base, motors and gas springs off.
- `2`: free physics; gravity only.
- `3`: direct calibrated `l0=0.150 m` LOW-LQR start, paused.
- `4`: suspended leg calibration. `W/S` changes swing and `D/A` changes
  compression; mechanical limits are disabled only in this mode.
- `5`: calibrated real encoder zero, then firmware recovery and LOW LQR.
- Calibration presets: `Z` encoder zero, `M` vertical maximum, `C` vertical
  minimum, `N` normal 0.150 m stance.
- `P`: 2 N m pitch pulse in direct-LQR mode.
- `Up/Down`: set persistent forward/reverse speed in LQR modes
  (`+/-0.60 m/s` per press).
- `Left/Right`: set persistent left/right yaw rate in LQR modes
  (`+/-2.00 rad/s` per press).
- `7/8/9`: select the normal leg length (`0.130/0.240/0.300 m`); the
  interactive default is `0.130 m` and changes are slewed to avoid an axial
  force step.
- `X`: clear the forward and yaw commands.
- `J`: trigger the physical preload/thrust/flight/landing jump sequence. The
  jump is interlocked to a near-stationary launch, and drive/yaw commands are
  cleared through touchdown; press an arrow again after the landing settles.
- `F`: toggle camera tracking/free-view mode.

Jump touchdown uses a first-contact capture stage before the debounced landing
state. Airborne wheel torque, hip torque, and axial support are handed over on
bounded paths so a real wheel contact cannot remain under the air-only control
allocation.

The overlay reports real encoder-frame values, CAD hinges, l0/phi0, ground
contacts, recovery phase, VMC F0/Tp, all hip torques, LQR gain, and measured
real-time factor. A small pitch value alone is never considered balance; the
acceptance tests also require upright phi0, correct leg length, wheel contact,
and no chassis/leg support contact after handoff.

## Calibration summary

- Encoder zero: `l0=0.332557082 m`, `phi0=18.495 deg`.
- Vertical calibrated range: `0.116399057..0.332557082 m` at `phi0=90 deg`.
- Normal stance: `l0=0.150 m`, `phi0=90 deg`, body pitch approximately zero,
  wheels on the ground.
- `encoder_zero_pose.json` is the active real-zero pose.
- `code_zero_pose.json` is a retained legacy diagnostic and is not used by
  recovery or the accepted LQR path.

## Regression

```bash
python test_vmc_calibration.py
python test_firmware_recovery.py
python test_calibrated_lqr.py
python test_firmware_jump.py
python test_drive_controls.py
```

The jump regression requires a real loss of wheel contact, more than 0.30 m
wheel clearance at the apex, bounded airborne attitude, wheel-only touchdown,
settled two-wheel support, and the 40 Nm hip peak limit. Moving-jump coverage:

```bash
JUMP_FORWARD_TARGET='0.20' \
python test_firmware_jump.py
```

`test_firmware_recovery.py` checks the complete encoder-zero to stand route,
including the firmware 25 degree pitch gate and final l0/phi0. To use the full
collision model:

```bash
export MUJOCO_MODEL="$PWD/model/whole_body_lqr.xml"
python test_firmware_recovery.py
python test_firmware_jump.py
unset MUJOCO_MODEL
```
