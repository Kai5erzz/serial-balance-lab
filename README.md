# rl-lab — 实车共用 C 控制器的 MuJoCo 试炼场

默认运行 code_914 当前工作区的 C 控制代码和参数，包括完整底盘控制链与云台 yaw。Python 负责物理模型和硬件接口。实车源码改动后，下一次运行自动同步、校验并重新编译，镜像的本地改动会受到保护。

控制代码已经共用；物理上的 1:1 尚未完成。默认操控窗口使用仿真专用 jump46 配置；参数、硬件方向候选和已知通行限制需要结合验证记录判断。详见 [共用控制器与验证记录](docs/SHARED_VEHICLE_CONTROLLER.md)。历史 Python 控制器保留为 `--controller legacy`，其独立调参和回归结果属于历史路径。

## Layout

```
balance_lab/            the simulation platform (Python package)
├── firmware/           shared C runtime, hardware adapter, recording and viewer
├── control/            historical Python controller, split by concern
│   ├── tuning.py       every firmware constant, source-named, one place
│   ├── kinematics.py   five-bar VMC types + helpers
│   ├── controller.py   TraditionalLQR core (LQR + VMC + state)
│   ├── jump.py         jump state machine (preload→thrust→flight→landing)
│   ├── guards.py       off-ground detection, ledge guard, danger check
│   └── recovery.py     encoder-zero + fall recovery state machines
├── plant/              pose loading + load_model(variant, plant=...)
├── sysid/              identified-parameter profiles and application
└── sim/                interactive viewer session (run_session)
firmware/vehicle/       byte-exact vehicle C tree and scheduling files
firmware/host/          desktop hardware boundary and scalar CMSIS DSP
firmware/calibration/   validated jointpd encoder-zero record and evidence
model/                  MuJoCo XMLs + meshes (full / fast / platform)
tests/                  regression battery — must stay green
docs/                   architecture, sysid pipeline, development notes
tools/                  model builders, calibration, probe archaeology
run_lqr.py              interactive viewer entry point
traditional_lqr.py      compatibility shim → balance_lab (legacy imports)
```

Documentation index:

- [docs/SHARED_VEHICLE_CONTROLLER.md](docs/SHARED_VEHICLE_CONTROLLER.md) — current C integration, sync, transfer and physical validation status.
- [docs/NATIVE_FRICTION_AUDIT_20261014.md](docs/NATIVE_FRICTION_AUDIT_20261014.md) — contact friction and joint resistance comparisons on the shared C recovery chain.
- [docs/NATIVE_POLARITY_AUDIT_20261014.md](docs/NATIVE_POLARITY_AUDIT_20261014.md) — sensor and motor direction checks, and the shared C jump roll axis correction.
- [docs/NATIVE_WHEEL_HANDOFF_AUDIT_20261014.md](docs/NATIVE_WHEEL_HANDOFF_AUDIT_20261014.md) — wheel handoff comparisons and the mechanical Follow yaw feedback mismatch.
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
- [docs/TURN_ROLL_NOTES.md](docs/TURN_ROLL_NOTES.md) — turn-roll
  compensation port, cruise-governor arming, and the T1-A sign audit.
- [tools/README.md](tools/README.md) — what the archived scripts are.

## Quick start (Linux)

```bash
git clone https://github.com/Kai5erzz/serial-balance-lab.git
cd serial-balance-lab
python -m pip install -r requirements.txt
python run_operator.py --variant rmuc2027 --fullscreen --fps 30
```

需要 Python 3.11+、Linux 桌面、Tk（系统包通常为 `python3-tk`）、GCC、`patch` 和可用的 NVIDIA OpenGL/EGL 驱动。无显示器的机器不能打开操控窗口。中文说明使用系统 CJK 字体（例如 Noto Sans CJK）。

**RMUC2027 运行地图随 Git 仓库完整提交**：`terrain_rmuc2027/assets/runtime.zip` 约 39 MB，另含约 13 MB 的操控显示资源，普通 `git clone` 即可获得，无需 Git LFS、外部 CAD 下载或本机 D 盘。首次启动自动核对 SHA-256，解压到 `~/.cache/rl-lab/rmuc2027/` 并编译物理和显示缓存，后续复用。首次编译可能需要数分钟，请留意终端输出。显示资源先在独立子进程编译，结束后再加载物理模型，避免临时占用叠加；首次编译建议预留约 5 GiB 可用内存，运行时仍监测 2500 MiB 底线。若需放到另一块盘，在启动前设置 `RL_LAB_RMUC2027_STORAGE=/path/to/cache`。现有本机地图目录继续复用。

机器人从检查过的地板位置 `(-6.25, -4.75)` 出生，启动暂停、默认显示纯文本键位说明；按 **P** 起身和运行，**H** 显示/隐藏所有文字，**F11** 全屏/窗口。隐藏文字后保留中央准星。地图有外围围挡，第三人称方向固定在世界坐标，左键拖动转视角，右键拖动云台，松开释放鼠标。

本机 NVIDIA 启动命令：

```bash
cd /home/kaiser/robomaster/rl-lab
mujoco-gpu --validation /home/kaiser/miniconda3/envs/rl-lab/bin/python run_operator.py --variant rmuc2027 --fullscreen --fps 30
```

详细地图说明、来源和几何限制见 [RMUC2027 README](terrain_rmuc2027/README.md)。2025/2026 的转换和适配代码也保留，运行地图需另行安装；它们的本机缓存不属于此次上传。

离线克隆可直接使用已提交的 `firmware/vehicle` 控制器镜像及校准记录，无需另克隆 code914。存在原始车端工作区时仍按原规则同步；仿真 jump46 补丁只应用到生成目录，不修改原始车端代码或镜像。

```bash
# 平地诊断与逐 tick 回放
python run_lqr.py --headless --pose encoder --seconds 5 \
  --record out/native/recovery.jsonl --report out/native/recovery.json
python tools/replay_firmware.py out/native/recovery.jsonl
python tests/test_native_firmware.py
```

## Shared C viewer controls

### RM operator console (GPU rendering, VNC-friendly)

```bash
mujoco-gpu /home/kaiser/miniconda3/envs/rl-lab/bin/python run_operator.py
```

This opens the RM key/mouse console with first-person yaw-head and third-person
world-direction views. It starts paused at the precise encoder-zero pose. `P` starts the
real C recovery chain; `R` restores that pose and a fresh C runtime, then pauses.
The configured `mujoco-gpu` launcher uses NVIDIA/VirtualGL in the current
KasmVNC session; Tk displays one 960×540 frame at a time. The entry point detects
the inherited task lock and does not nest another `heavy_run.sh`. It uses the
launcher's 3000 MB startup guard and the current training service's shared lock
at jointpd `.heavy/heavy.lock`. `python run_operator.py` also automatically
selects this launcher when installed. Without the local launchers, a bundled Linux guard uses EGL, a user-cache
operator lock and a 2500 MiB available-memory floor checked every 100 ms. Close the
window to release the lock. A running console cannot share that lock with another
simulation/rendering job. Available RAM below 2 GiB pauses rendering and physics.

For this user's validation work alongside training, use
`mujoco-gpu --validation /home/kaiser/miniconda3/envs/rl-lab/bin/python run_operator.py`.
This holds a separate validation lock and monitors available RAM every 100 ms;
below 2500 MiB it stops only its own process group. Validation jobs remain serial.

| Key | Action |
| --- | --- |
| W / S | Forward / backward along the robot's longitudinal axis |
| A / D or right-mouse drag | Turn the yaw head; the real C Follow loop turns the chassis |
| Shift / Ctrl | Full stick / 20% stick; normal movement is 55% stick |
| E / Q | Spin / return to Follow; C retains the real Spin translation lockout |
| Z / X | LOW / MID leg selection |
| C / B | Actual DR16 HIGH switch: bump request; its C command owner forces LOW while bump owns the action; use Z/X to release |
| Space | Original DR16 jump-wheel edge, including C readiness/cooldown gates |
| Tab | First / third person |
| P / R | Run-pause / reset precise encoder pose and pause |
| Hold right mouse / release | Drag to aim / release the pointer and stop mouse input, without pausing |
| [ / ] | Decrease / increase mouse sensitivity (0.25–2×); default is one quarter of the original sensitivity |
| Left mouse drag (third person) | Rotate the world-fixed third-person view |
| H | Show/hide all text; help is visible at startup; the reticle always remains |
| F11 | Fullscreen / restore window |
| Esc | Exit fullscreen, release all inputs and pause |
| Backspace / Enter | Relax / enable |

Losing window focus pauses and clears held keys, mouse input and jump pulses.
The two-wheel chassis has no lateral translation; A/D steer the yaw head.
Mouse vertical movement adjusts only the first-person view. Pitch and shooting
remain outside this yaw-only simulation; F/G and fire buttons have no weapon action.

The active C velocity envelope is **2.30 / 1.85 / 1.20 m/s** for LOW/MID/HIGH.
C also reduces it according to measured leg geometry, applies its input ramps,
and owns jump/bump/spin limits. The current HIGH remote switch starts bump and
forces LOW; the console follows that command behavior rather than bypassing it
to select a free-standing HIGH pose. The HUD reads the actual limited reference and
limit from C Page39 rather than deriving a separate Python speed governor.
W is mapped to positive C vx and S to negative C vx. Both cameras face the CAD
gun muzzle (yaw-head local +X); the first-person view previously faced the
opposite raw IMU reference axis. Physical movement and camera orientation are
checked against the muzzle geometry. The HUD's measured speed is positive
along the chassis muzzle direction, and it shows the gun's world heading.
Limits describe commanded speed, not a
physical clamp on sliding, momentum, falls or externally applied motion.

The console explicitly selects `coherent_candidate`: the wheel and complete yaw
gear/IMU mapping from the direction audit. This remains an **installation
candidate**, pending real-vehicle and broader behavior validation. The console
also explicitly uses `--yaw-damping 0`: the generic MJCF yaw damping of
0.35 Nm s/rad was copied from the joint defaults, and was not identified by
the leg profile. At the C Spin target of 10.5 rad/s it needs 3.675 Nm, exceeding
the original yaw rate-loop's 2.4 Nm cap and dragging the gun around. Zero
viscous bearing damping is an **uncalibrated assumption**, recorded in metadata;
it is not a real yaw-friction fit. `--yaw-damping 0.35` reproduces the old model.
Original C, PID gains, the identified leg parameters, and the default
`run_lqr.py` adapter retain their values.
Use `python run_operator.py --interface-mapping original` for the original
diagnostic interface. `--variant full/platform` selects the existing collision
models; `--variant rmuc2027` selects the bundled official 2027 arena.

```bash
# GPU demo: recovery, forward/reverse, yaw turn, both views, per-tick C replay.
python run_operator.py --demo-dir out/native/operator_demo_new --fps 20
```

The demo writes a streamed video, first/third-person PNGs, complete command
recording and report. Choose a new directory to preserve previous evidence.
The 2026-10-14 candidate demo replayed all 10,000 C ticks exactly and used about
653 MiB peak RSS. This verifies the console path and these short maneuvers;
it is not full physical 1:1 or high-speed/jump/bump acceptance.

The corrected camera/Spin audit is in
`out/native/operator_direction_spin_20261014/`; its 15 s GPU video and complete
C replay cover driving and a 5 s Spin hold. The world-yaw regression requires
more than seven chassis rotations, less than 4° maximum gun drift, and less
than 2° final drift under the recorded zero-damping assumption.

### Original diagnostic viewer

- `Space` run/pause; `R` reset; `F` camera follow.
- `1` relax; `2` RC control; `T` spin; `X` relax.
- `3` saved standing initial pose; `5` precise encoder-zero initial pose.
- Arrows drive and gimbal yaw; `J` real RC jump request.
- `7/8` LOW/MID; `9` or `B` real HIGH + bump-switch semantics.
- `--variant full/platform` selects full collisions or the platform terrain.
- The identified plant is the default; `--plant nominal` selects original XML physics. Plant profiles never change C gains.

## Legacy Python viewer controls

Run `python run_lqr.py --controller legacy` for the historical controls below.

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

## Shared firmware validation

```bash
python tools/sync_firmware.py --check
python tests/test_native_firmware.py
python tools/validate_vehicle_trial.py
python tools/catalog_vehicle_samples.py
python tools/package_vehicle_controller.py
```

Physical trials report contact, attitude, actual firmware phases and recovery faults. A finite trace or a replay match does not establish a successful physical maneuver.

## Historical Python regression battery

```bash
python tests/test_vmc_calibration.py       # VMC anchors and Jacobians
python tests/test_firmware_recovery.py     # encoder zero → recovery → LQR
python tests/test_calibrated_lqr.py        # 0.15 m stand + pitch pulse
python tests/test_drive_controls.py        # drive/yaw commands
python tests/test_firmware_jump.py         # physically actuated jump
python tests/test_turn_stability.py        # low-leg hard-turn stability
python tests/test_inertia_integrity.py     # CAD inertia guards
python tests/test_sysid.py                 # identified-parameter pipeline
python tests/smoke_test.py                 # finite-value sanity
```

These tests preserve the legacy Python path; they do not validate the native C controller. After a change to that path or the shared model, rerun affected checks; the recovery
and jump tests additionally rerun against the full collision model:

```bash
export MUJOCO_MODEL="$PWD/model/whole_body_lqr.xml"
python tests/test_firmware_recovery.py
python tests/test_firmware_jump.py
unset MUJOCO_MODEL
```

## Historical Python plant profiles

The regression battery runs on the **nominal** plant — the XML as
committed — which is what the firmware tuning was validated against.
Running on the identified plant is one explicit argument:

```python
from balance_lab.plant import load_model

model, applied = load_model("fast", plant="identified_2026-09-04")
model, applied = load_model("fast", plant="identified_2026-09-21")  # asymmetric adopted plant
# applied: record of every parameter written; empty for plant="nominal"
```

See [docs/SYSID.md](docs/SYSID.md) for the parameter→consumer mapping and
the current identification status (PRBS actuator fit, the 2026-09-07
scalar coupled-mechanics pass, and the adopted 2026-09-20 tensioned
**asymmetric** per-side fit).

## Historical Python calibration and tuning

- Encoder zero **Z_new** (2026-09-13, user-confirmed: chassis belly flat,
  both legs against the rear stops, both 58 mm tires on one ground plane;
  `E = Q − Z_new`): `l0=0.332557082 m` on both legs, per-side
  `phi0 = 17.070469 deg` (left) / `18.716243 deg` (right)
  (`encoder_zero_pose.json`). The migration is a pure re-anchor — the FK
  on every pose is unchanged, proven by `tests/test_vmc_calibration.py`
  (worst Δl0 = 6e-17 m), following rl-serial-balance
  `docs/RL_ZERO_UNIFICATION_PLAN.md` §4 (direct-encoder form).
- Vertical calibrated range: `0.116399057..0.332557082 m` at `phi0=90 deg`.
- Normal stance: `l0=0.150 m`, `phi0=90 deg`, level chassis, wheels down
  (unchanged by the zero migration — the stand pose file is deliberately
  not the zero posture).
- Knee travel (`l_leg`/`r_leg` MJCF ranges): measured leg-limit envelope
  ±0.01 rad MuJoCo/PhysX softness margin — `[-0.4155, 0.7598]` /
  `[-0.7611, 0.41]` rad (rl-serial-balance
  `configs/calibration/leg_limits_2026-09-21.json`; replaces the old
  `±1.35`-derived endpoints that over-folded ~2× past the real lower stop
  and blocked extension past `l_leg=0`).
- VMC length PID (2026-09-21 port, **the locked rl-serial per-leg table**,
  cloud recipe `vmc_per_leg_gains: True`): support mass
  `IDENTIFIED_SUPPORT_MASS_KG = 13.592626 kg` (feedforward **66.6718 N/leg**),
  `kp = 2039.2593 / 1960.7407 N/m` and `kd = 15.9257 / 28.0743 N·s/m`
  (l/r), mean anchored at the hardware-validated 2000 N/m design point
  (the firmware's 450 N/m measured only 4.5 N at 1 cm), code_611
  variable-integration-rate + output-clamp anti-windup, theta-sync damping
  `0.60 → 2.0` (3.1 Hz yaw-cycle fix). Scope: the VMC support path
  (standing + descent-edge + lateral load transfer). The firmware
  jump/landing/off-ground laws keep `CHASSIS_SUPPORT_MASS = 12.801 kg` —
  their preload ready window sits a few mm from the equilibrium and
  re-anchoring it times the jump out in preload (measured; see
  `control/jump.py`). Derivation in `balance_lab/control/vmc_gains.py`,
  table pinned in `tests/test_sysid.py`.
- `code_zero_pose.json` is a retained legacy diagnostic; it is not used by
  recovery or the accepted LQR path.

## Historical scope (Windows origin)

This project was developed on Windows (`D:\User\balance_infantry\...`);
the notes keep their original reproduce commands where relevant. All
current development happens on Linux/conda.
