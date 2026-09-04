# System identification pipeline

Status: **PRBS actuator-level identification done; coupled-mechanics
real2sim pass not yet accepted.** This document records what is
identified, where each parameter lands in the simulator, and how to add
the next measurement set.

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

## Known gaps (tracked)

- actuator latency (28 ms) is not simulated anywhere in rl-lab yet.
- sensors (gyro/encoder noise, bias) are not simulated; the platform is
  deterministic.
- geometry/CAD parameters are calibration-review-only in both repos and
  can never come from a fit.
