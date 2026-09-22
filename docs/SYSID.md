# System identification pipeline

Status: **three identification generations are on file; the 2026-09-20
tensioned *asymmetric* fit is the adopted plant (2026-09-21 user ruling).**
Generations: `identified_2026-09-04` (PRBS actuator fit, bundled default),
`identified_2026-09-07_v2` (scalar coupled-mechanics pass), and
`identified_2026-09-21` (per-side equivalent plant: damping/friction/
armature/gains/passive chain/mass scale — adopted). This document records
what is identified, where each parameter lands in the simulator, and how to
add the next measurement set.

## The loop

```
real vehicle (PRBS excitation, state==3 faultless windows)
        │  raw CSV logs
        ▼
rl-serial-balance  scripts/identify_real2sim.py + identification/
        │  fits actuator params; backfills nominal.yaml → identified_*.yaml
        ▼
configs/model/identified_<date>.yaml          (fingerprint = provenance)
        │  copy the file into balance_lab/sysid/profiles/
        ▼
balance_lab  load_profile() → apply to MjModel + controller
        │
        ▼
regression battery (nominal) + identified-plant smoke tests
```

The file format, schema version, and the fingerprint function are shared
with `serial_balance_isaac.parameters` (rl-serial-balance), so a profile
produced there loads here unchanged and keeps **the same fingerprint**.
The fingerprint is the answer to "which measured parameter set am I
running on?"; `tests/test_sysid.py` pins it, so a silently edited YAML
fails the battery instead of quietly changing the plant.

## What the bundled profile contains

`balance_lab/sysid/profiles/identified_2026-09-04.yaml`
(fingerprint `495989a22f4a6201`):

| Section | Value | Origin |
| --- | --- | --- |
| `actuators.hip.strength_scale` | 0.971 | PRBS first-order actuator fit |
| `actuators.wheel.strength_scale` | 0.807 | PRBS first-order actuator fit |
| `actuators.*.viscous_friction_nms_rad` | 0.35 / 0.08 | = MJCF nominal parity |
| `contacts.*` | 1.0 / 1.0 / 1.2 | = nominal parity |
| `latency.actuator_ms` | 28 | identified, **not yet simulated** |
| `theta_compensation.*` | poly + dx offsets | identified; dx 0.0182 differs from the controller's legacy 0.013 |
| `gas_springs.force_n` | 300 | = nominal parity |
| `fit.status` | `identified` | dataset id + covariance metadata |

## Adopted asymmetric profile (2026-09-21)

`balance_lab/sysid/profiles/identified_2026-09-21.yaml`
(fingerprint `b3d77732f0b5d7ee`), ported from rl-serial-balance:

| Source | sha256 (first 16) |
| --- | --- |
| `configs/model/leg_friction_damping_2026-09-21_asymmetric.json` | `a5fa76890366bf6f` |
| `configs/model/physical_fk_2026-09-18_v2.yaml` (wheel viscous 0.0082 + left-leg mirror fix) | `df35b20cdfc01813` |

| Section | Value | Origin / notes |
| --- | --- | --- |
| `legs.motors.*.viscous/friction/armature/strength` | per-motor (l_j1 damping 0.739 vs r_j1 0.0389 — 19× apart) | 2026-09-20 tensioned fit, P3 held out; **never average the sides** |
| `legs.passive_joints.{l,r}` | 0.0254 / 0.3023 N·m·s/rad (+ Coulomb) | passive chain, right 12× left (least-constrained equivalent) |
| `legs.mass_inertia_scale` | l 1.2438, r 1.5128 | missing CNC limit stops + gas-spring mass; applied to every `l_*`/`r_*` body (source profile's body set, wheels included) |
| `rigid_bodies.total_mass_kg` | **13.836367** | what THIS asset weighs after the scale (MATLAB wheel layout); recorded as `plant_total_mass_kg`, **not** the feedforward anchor |
| `rigid_bodies.support_mass_kg` | **13.592626** | 2026-09-21 physical ruling → controller support feedforward (**66.6718 N/leg**) and the per-leg PID anchor (`TraditionalLQR.apply_identified`) |
| `actuators.wheel.viscous_friction_nms_rad` | 0.0082 | identified mean of 0.00699 / 0.00948; strength 0.807 measured harmful → 1.0 |
| `geometry.encoder_zero_rad` | Z_new (2026-09-13) | matches `tuning.VMC_ENCODER_ZERO_CAD_Q` |

Asset prerequisites ported together with the profile (both in
`model/*.xml`, all 10 variants): the **left-leg mirror fix** (`l_j1`
−23.31 g, `l_leg` −3.09 g dropped by the URDF export → total
12.801 → 12.827403 kg, guarded by `tests/test_inertia_integrity.py`) and
the **measured knee ranges** (see the calibration note in README).

Recorded divergences (not hidden):

- rl-serial-balance reaches 13.592626 kg on *its* asset; this repo's
  deliberate MATLAB wheel rebalance (wheels 0.577 vs 0.254858 kg, base
  offset by the same 0.644284 kg) makes the same body-set rule land on
  13.836367 kg. The two numbers are therefore separate fields:
  `total_mass_kg` describes the asset, `support_mass_kg` (13.592626) is the
  physical support ruling the feedforward and the PID anchor use — so
  rl-lab and rl-serial run the **same locked table** (2039.2593/1960.7407,
  15.9257/28.0743) while each asset keeps its own measured total.
- The source fit's runbook items stay open: optimizer `success=false` at
  `nfev=max_nfev`, 16 warm-fold RMSEs span 0.0167–0.0471 rad, fit ran on
  the knee-unlimited derived XML. Adopted anyway on the 2026-09-21 ruling.

Controller side: `support_mass` defaults to
`tuning.IDENTIFIED_SUPPORT_MASS_KG = 13.592626` (the 2026-09-21 ruling),
i.e. a fresh controller already carries the locked rl-serial table;
`apply_identified` only re-anchors it when a profile ships an explicit
`rigid_bodies.support_mass_kg` (the bundled legacy profile does not, so it
can never silently re-anchor). The per-leg derivation
(`balance_lab/control/vmc_gains.py`) keeps the mean stiffness anchored at
`LENGTH_KP = 2000 N/m` for any supported mass.

```python
model, applied = load_model("fast", plant="identified_2026-09-21")
controller = TraditionalLQR(model)
controller.apply_identified(load_profile("identified_2026-09-21"))
```

## Parameter → consumer mapping

| Profile field | Applied by | Target | Notes |
| --- | --- | --- | --- |
| `actuators.*.strength_scale` | `apply_model_parameters` | `model.actuator_gainprm[act,0]` | motor torque = gain × ctrl; measured weakness becomes gain |
| `actuators.*.viscous_friction_nms_rad` | `apply_model_parameters` | `model.dof_damping` on **chain** hinges | loop-closure hinges stay 0 (pure constraints) |
| `contacts.wheel_dynamic_friction` | `apply_model_parameters` | wheel collision geoms, sliding | MuJoCo has no static/dynamic split |
| `contacts.ground_static_friction` | `apply_model_parameters` | ground/platform geoms, sliding | platform layers included |
| `gas_springs.force_n × scale` | `TraditionalLQR.apply_identified` | `controller.gas_spring_force` | survives `reset()` |
| `theta_compensation` poly/center/scale/dx | `TraditionalLQR.apply_identified` | instance overrides used by `_theta_compensation` | asymmetric dx offsets rejected (not representable) |
| `actuators.*.torque_limit_nm` | — | **not applied** | torque envelopes are controller/ctrlrange authority (the jump window needs the 40 Nm XML range) |
| `contacts.restitution` | — | **not applied** | nominal 0; MuJoCo encodes restitution via negative solref — deferred until a non-zero value is identified |
| `latency.actuator_ms` | — | **not yet simulated** | TODO: bounded torque first-order lag on the actuator path; must stay opt-in |
| `sensors.*`, `identity`, `fit` | — | metadata | logged, not simulated |
| `legs.motors.*` | `apply_model_parameters` | per-motor `gainprm`/`dof_damping`/`dof_frictionloss`/`dof_armature` | overrides the scalar `actuators.hip` path; armature bounded [0, 0.08] like the fixture validator |
| `legs.passive_joints.*` | `apply_model_parameters` | `dof_damping`/`dof_frictionloss` on the side's passive leg hinges | motor hinges excluded |
| `legs.mass_inertia_scale` | `apply_model_parameters` | `body_mass`/`body_inertia` of `l_*`/`r_*` bodies | multiplicative **from the pristine baseline** (idempotent); solver inverse-weight hints stay compile-time |
| `rigid_bodies.support_mass_kg` (optional) | `TraditionalLQR.apply_identified` | `controller.support_mass` + per-leg length gains | VMC support path (standing/descent-edge/lateral load transfer) only; jump/landing/off-ground keep firmware `CHASSIS_SUPPORT_MASS` (preload window, measured); absent → current value kept |

The **coupled-mechanics** results of the PRBS fitter
(`hip_damping_scale`, `hip_frictionloss_nm`, `hip_armature_kg_m2`,
`wheel_damping_scale`, `wheel_frictionloss_nm`) are the real2sim output
and are **not yet accepted** (a preliminary run exists in
rl-serial-balance's `identify_real2sim_report.json`). The application
hook is ready and rule-exact:

```python
from balance_lab.sysid import MechanicsFit, load_profile
from balance_lab.plant import load_model

fit = MechanicsFit(
    hip_damping_scale=0.6, hip_frictionloss_nm=0.3,
    hip_armature_kg_m2=0.0098,
    wheel_damping_scale=1.5, wheel_frictionloss_nm=0.05,
    source="<report>",
)
model, applied = load_model("fast", plant=(load_profile(), fit))
```

Application rules (mirroring `identify_real2sim.py`):

- chain damping = `hip_damping_scale × 0.35` (absolute write, so the
  overlay order does not matter); wheel damping = `wheel_damping_scale × 0.08`;
- Coulomb friction on **every** leg hinge including closure hinges (their
  bearings exist on the real machine);
- reflected rotor inertia on the four actuated hinges only.

## Plant selection

```python
from balance_lab.plant import load_model

model, applied = load_model("fast")                     # nominal (battery default)
model, applied = load_model("fast", plant="identified_2026-09-04")
model, applied = load_model("fast", plant="identified_2026-09-21")  # adopted asymmetric
model, applied = load_model("full")                     # mesh-collision regression plant
model, applied = load_model("platform")                 # 600 mm stair terrain
```

`applied` is a human-readable record of every field written and is empty
for the nominal plant. Idempotence is tested.

## Adding the next identified profile

1. Produce `identified_<date>.yaml` via rl-serial-balance's
   `backfill_from_identification` (never edit the nominal file; geometry
   is immutable).
2. Copy it into `balance_lab/sysid/profiles/`.
3. Update `BUNDLED_PROFILE` in `balance_lab/sysid/apply.py` if it should
   become the default named profile.
4. Re-pin `FINGERPRINT` in `tests/test_sysid.py` and note the new
   dataset/run in the table above.
5. Rerun the battery (nominal must stay green) plus
   `python tests/test_sysid.py`, then the identified-plant acceptance of
   the controllers you intend to trust (recovery + jump, per
   [docs/RECOVERY_PORT_NOTES.md](RECOVERY_PORT_NOTES.md) § system
   identification).



## Real2Sim V2 Identification Profile (2026-09-07)

Status: **Real2Sim V2 coupled-mechanics pass completed and validated via Leave-One-Pass-Out Cross-Validation on temp4.csv (50.0 Hz DMA telemetry).**

`balance_lab/sysid/profiles/identified_2026-09-07_v2.yaml` (fingerprint `96beb674a89118c1`):

| Section | Value | Origin |
| --- | --- | --- |
| `actuators.hip.viscous_friction_nms_rad` | **0.4604** | Real2Sim V2 Nelder-Mead Cross-Validation (`temp4.csv`) |
| `actuators.hip.coulomb_friction_nm` | **0.2202** | Real2Sim V2 Coulomb friction parameter |
| `actuators.hip.static_friction_nm` | **0.3184** | Stribeck low-speed peak static friction |
| `actuators.hip.stribeck_velocity_rad_s` | **0.0310** | Stribeck transition characteristic velocity |
| `actuators.hip.armature_kg_m2` | **0.0035** | DM4310/4340 rotor reflected inertia (`armature`) |
| `latency.actuator_ms` | **20.0** | 实测 50.0 Hz DMA 零阻塞遥测步长 |
| `fit.metrics` | Chirp RMSE **0.0757 rad**, Square corr **0.9194** | 留一轮交叉验证测试集 (Pass 3 hold-out) |

### Usage in balance_lab

```python
from balance_lab.sysid import MechanicsFit, load_profile
from balance_lab.plant import load_model

# 1. Load with identified model parameters (damping, gains, contacts)
model, applied = load_model("fast", plant="identified_2026-09-07_v2")

# 2. Or load with full coupled-mechanics overlay (including coulomb friction and rotor armature)
v2_profile = load_profile("identified_2026-09-07_v2")
v2_fit = MechanicsFit(
    hip_damping_scale=0.4604 / 0.35,  # 1.3154
    hip_frictionloss_nm=0.2202,
    hip_armature_kg_m2=0.0035,
    wheel_damping_scale=1.0,
    wheel_frictionloss_nm=0.0,
    source="temp4.csv (Real2Sim V2)",
)
model, applied = load_model("fast", plant=(v2_profile, v2_fit))
```

## Known gaps (tracked)

- actuator latency (28 ms) is not simulated anywhere in rl-lab yet.
- sensors (gyro/encoder noise, bias) are not simulated; the platform is
  deterministic.
- geometry/CAD parameters are calibration-review-only in both repos and
  can never come from a fit.
