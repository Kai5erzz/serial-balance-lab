# Physical jump-control record

Last updated: 2026-08-26

## Scope

This record covers the simulation jump controller after the airborne-pitch and
landing work. The controller uses only the four hip actuators, the two drive
wheels, the calibrated five-bar VMC, wheel contact, and measured body/leg
states. It does not apply a force to `base_link`, reset `qpos`, or prescribe a
body trajectory.

The preload, thrust, flight, landing, timeout, and safety-state structure is
the firmware jump route. The gains and phase allocation below are intentionally
more aggressive than the current real-vehicle settings because this pass was
authorized for simulation tuning.

## Actuator envelope

- Normal standing and driving keep the hip-motor limit at `20 Nm`.
- Recovery keeps its existing source-specific torque limits.
- Only preload/thrust/flight/landing may use the real drive's `40 Nm` peak.
- Both MuJoCo models expose `-40..+40 Nm` on the four hip actuators so the
  controller, rather than the XML actuator clamp, owns this envelope.
- Wheel torque remains limited to `3.69 Nm`; it was not enlarged for jumping.

The accepted stationary jump peaks at `29.8 Nm` and remains above the `20 Nm`
rating for about `0.135 s`. The `40 Nm` hardware peak remains a safety clamp,
not a normal airborne command.

## Control allocation

1. **Preload and thrust:** the VMC length route compresses and thrusts toward
   `0.305 m`. The firmware readiness line `0.145 m` (preload from
   `LEN_LEN_LOW=0.130 m`) is raised to `0.155 m` here because this sim's
   user-calibrated LOW stand is `0.150 m` and the MATLAB-wheel plant settles
   near `0.154 m`; the preload confirms a settled leg length instead of
   demanding a deep compression the heavier wheels cannot reach in the
   firmware's `0.85 s` window. While wheel contact still exists, a pitch-rate
   loop establishes the angular momentum needed to cancel the later leg
   motion. The stationary target is `1.48 rad/s`; moving launch subtracts a
   term proportional to measured launch velocity.
2. **Compact ascent:** the virtual-leg reference moves to `0.130 m` at
   `3.50 m/s`. The compact interval is `0.23 s`, which includes the apex.
   Airborne control is explicitly split by physical role: hips hold the
   virtual-leg direction (`theta Kp/Kd=30/4`, tuck/deploy weights `0.8/1.5`),
   while the two wheel rotors own body pitch (`Kp/Kd=35/5`). The old controller
   asked the hip common axis to hold pitch and leg direction simultaneously;
   after restoring the CAD inertias this made virtual `Tp` alternate between
   `+40/-40 Nm` at the `5 ms` control period. Removing the competing hip-pitch
   loop eliminates that airborne chatter. Tuck/deploy pitch references are
   `12 deg / 4 deg`; the measured body pitch remains close to zero because the
   leg-retraction reaction is opposite to this rotor reference.
3. **Landing deployment:** the length target returns to `0.270 m`; leg-angle
   authority rises after the compact interval so both wheels point toward the
   ground before contact. Reaction-wheel taper starts `0.10 s` after the tuck
   interval and lasts `0.20 s`. The stationary first-contact pitch is now
   `-0.42 deg`, compared with `-5.38 deg` before this retune.
4. **Touchdown capture and buffer:** the first raw wheel contact immediately
   starts a `40 ms` capture window. The wheel path stops using the airborne
   reaction-wheel request and approaches wheel pitch PD (`Kp=10`, `Kd=1`) with
   a bounded command step. The hip path keeps the airborne leg controller until
   both wheels pass the existing eight-sample contact debounce, then slews into
   independent landing leg-angle PD (`Kp=30`, `Kd=6`). The axial path blends
   from the airborne force into the landing buffer over the same capture
   window. Contact debounce now decides only the state transition; it no longer
   leaves all actuators in air mode after a real wheel has touched the ground.
   Hold/return timing was tightened for the heavier plant (hold `0.12..0.30 s`,
   return ref rate `1.20 m/s`, exit-stable `0.12 s`): measured
   touch-to-LQR-handoff is about `0.5 s`.
5. **Pre-jump settle and position capture:** the jump entry waits `3 s` for the
   LOW equilibrium to finish its leg-length transient. After wheel touchdown,
   the landing-only position-observer distance ratio is `6.0` instead of the
   ordinary driving value `3.0`. This stops the stationary landing at the
   takeoff point without changing daily stand/drive LQR gains.

## Acceptance results

Plant: MATLAB wheel inertia (`mw=0.577 kg`, `Iw=mw*R^2=0.001941 kg m^2`),
restored CAD inertias and original leg stops / 40 Nm hip envelope.

| Model / launch | Base rise | Apex wheel clearance | Apex pitch | Max air pitch | First wheel pitch | Touchdown-window pitch | Final pitch/roll | Final launch-point error | Non-wheel ground contact |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Fast, stationary | `0.319 m` | `0.305 m` | `-2.21 deg` | `6.28 deg` | `-0.42 deg` | `3.81 deg` | `+0.13/+0.21 deg` | `-0.014 m` | none |
| Fast, `+0.50 m/s` | `0.322 m` | `0.301 m` | `-6.16 deg` | `6.91 deg` | `-1.89 deg` | `7.57 deg` | `+0.62/+0.20 deg` | moving launch | none |

Both runs traverse `PRELOAD -> THRUST -> FLIGHT -> LANDING -> IDLE`, finish
with `fault=none`, keep both wheels grounded after settling, and avoid leg,
chassis, gimbal, and recovery-skid contact after takeoff. The stationary run
also enforces a `0.05 m` launch-point-error limit.

The ordinary calibrated stand, pitch-pulse recovery, encoder-zero recovery,
forward drive, and yaw regression tests remain unchanged and pass after the
jump changes.

## Reproduce

```powershell
cd D:\User\balance_infantry\mujoco_lqr_whole_body
python test_firmware_jump.py
$env:JUMP_FORWARD_TARGET = '0.20'
python test_firmware_jump.py
Remove-Item Env:JUMP_FORWARD_TARGET
$env:MUJOCO_MODEL = (Resolve-Path 'model\whole_body_lqr.xml')
python test_firmware_jump.py
Remove-Item Env:MUJOCO_MODEL
```

For interactive verification, run `python run_lqr.py`, press `Space`, and
press `J` after the robot reaches the standing LQR state. `Up/Down` selects
moving launch speed, `Left/Right` requests the reduced-rate turn, `X` stops
movement, and `F` toggles the following camera.

## Remaining sim-to-real work

Before using these aggressive gains on the real chassis, identify hip torque
rise/delay and speed saturation, wheel reflected inertia, linkage friction,
and impact compliance. Re-run this regression after each plant update. The
controller should be retuned against the identified plant; the 40 Nm window
must not be copied to hardware without checking the motor/driver thermal and
peak-duration limits.

## 2026-09-05 night session: landing handoff and attitude quantified

Prompted by "landing takes seconds to feel normal again" and "jump
attitude is not great". New probe: `tools/probes/landing_timeline.py`
(touchdown-anchored timeline of buffer phases, gains, position hold,
drive-response latency, cooldown) plus a 25 ms-resolution attitude
profile across all jump phases.

Findings:

1. The state machine is NOT the delay: touchdown -> idle in 0.145 s.
2. **Buffer handoff timings diverged from the current real firmware.**
   The sim shipped the aggressive sim-tuned set (return 1.20 m/s,
   PID_RETURN 0.20 s, EXIT_STABLE 0.12 s, HOLD_MIN 0.12 s); code_611
   rm_config.h has since been retuned gentler (0.80 / 0.30 / 0.18 /
   0.18). All four ported. The buffer exit is now a single smooth pitch
   hill instead of a double oscillation, and the jump regression stays
   green (forward_delta -0.391 m, inside the +-1 m line; the deeper
   backward drift is the trade for the gentler exit).
3. **Jump cooldown runs from jump ACCEPT, 3 s** — same as the firmware
   (`jump_last_accept_ms` stamped in `jump_control_try_start`). The
   ~2 s post-landing lockout is firmware-faithful behaviour, not a sim
   bug. Relaxing it would be a deliberate sim deviation.
4. **Thrust-phase nose-up is design intent.** Pitch climbs to +6.7 deg
   (79 deg/s) during thrust — the controller is parking angular momentum
   at the firmware's 1.48 rad/s pitch-rate target — and the airborne
   wheel-rotor loop spends it to level out (air pitch -0.3..+2.4 deg
   after the recovery). The visible nose-up at liftoff will not go away
   without retuning `JUMP_THRUST_PITCH_RATE_TARGET_ZERO`, which is a
   firmware value.
5. Remaining measurable wart: after buffer exit the leg rebuild
   (0.12 -> 0.176 m overshoot, gas-spring driven) couples into a
   backward pitch drift to -3.75 deg and a ~0.4 m/s backward slide,
   giving ~1.3 s forward-drive latency after landing. Next lever: make
   the post-exit length reference start from the actual length and slew
   down (sim deviation), or slow the rebuild; not attempted tonight.

## 2026-09-22 — spring-only launch experiment (motor never extends)

User ruling: at takeoff the motor must never help the gas spring extend
(it shock-loads the rod); it may only compress the spring, or be idle, and
keeps only the swing (theta) channel.

Implementation (`jump._jump_supports`): the THRUST phase clips its
leg-axial output to `F0 <= 0` (extension-side force removed: static share,
the PID chase toward `JUMP_THRUST_LENGTH = 0.305 m`, and the retired
`+70 N` boost / `120 N` floor). PRELOAD keeps its weight-bearing share —
it folds the leg (`dl < 0`), so a positive `F0` there does *negative* work
on extension (absorb/hold), never "helping it straighten". Landing/air
laws untouched.

Measured (fast model, flat scene, 3 s settle):

| quantity | spring-only (new) | motor-assisted (before) |
| --- | ---: | ---: |
| thrust `F0` (sampled) | **0.00 N** throughout | ~140–200 N/leg |
| takeoff | **none** (`fault=no_takeoff`, 0.70 s and 5.0 s windows) | `flight` @3.385 s |
| airborne time | **0.000 s** | 0.43 s |
| max wheel clearance | **0.0038 m** | 0.308 m |
| body Δz | +0.15…0.17 m (wheels planted — a stand-up, not a jump) | +0.331 m |
| leg length range | 0.111 → ~0.30–0.316 m | 0.125 → flight |

Why zero (per-leg load `0.5·12.801·9.81 = 62.79 N` vs the control-side
virtual spring `_gas_virtual_support(l)`):

| l0 (m) | 0.115 | 0.125 | 0.150 | 0.200 | 0.2225 | 0.240 | 0.300 | 0.330 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| spring (N) | 24.5 | 28.5 | 38.3 | 56.3 | **62.8** | 69.3 | 86.7 | 94.7 |

- At the preload release point the spring carries only 45 % of one leg's
  load; with `F0 <= 0` the region below the 0.2225 m crossover cannot
  produce net lift, so the launch can never be initiated from the
  compressed pose.
- The observed stand-up stalls around `l0 ≈ 0.30 m` where the *physical*
  spring's virtual force reaches the leg load (the control-side curve
  still reads ~85 N there — the feedforward overestimates the physical
  spring near full extension; logged as a sysid follow-up), and at the
  stall there is no residual kinetic energy → the wheels never unload.
- Energy accounting: the net-positive region above the crossover holds
  ∫(spring − load) dl ≈ 2–3 J/leg (v ≤ ~0.6 m/s ⇒ a few cm even under
  perfect staging); harvesting the *whole* spring range
  (∫ spring dl ≈ 11.5 J/leg) would cap around 0.10–0.20 m — and the
  0.111→0.2225 m traversal it requires is precisely the motor extension
  work the new rule forbids.
- The previous 0.33 m apex was paid for by the motor: thrust floor 120 N
  + boost 70 N + PID toward 0.305 m ≈ 270 N/leg net ≈ 49 J.

Verdict: **spring-only takeoff height on this plant = 0** (no takeoff);
the state machine and all safety/landing behaviour remain intact. A
non-zero spring-only jump needs a hardware change (higher spring precharge
/ better lever arm / more travel so that `spring_virtual > load` at the
*compressed* pose), or a relaxation of the no-extension-work rule.

**Closed the same day.** The premise of the no-extension rule was then
checked against manufacturer guidance and the mechanism geometry and did
not hold:

- a gas spring's in-stroke extension is its normal working mode; the
  documented damage modes are (a) using the full stroke / bottoming out
  (manufacturer spec: leave ~8 mm of stroke margin), (b) side/inclined
  loads bending the rod into the seal, (c) rod-surface damage -- not
  "being extended" per se.  Pull-type applications use a different
  product line (ACE tension series GZ, LQD/ELQD), confirming standard
  QD springs are push-only *as force elements*, which is unrelated to
  in-stroke length changes.
- the mechanism's own stops bound the installed gas-spring length to
  **[0.154, ~0.205] m** (measured at real closed-chain poses: 0.1542 at
  l0=0.111, 0.2023 at the 0.3326 extension stop, ~0.2045 at the new knee
  range top), i.e. the spring stays on the compression side of its stroke
  at all times and cannot be over-extended by motor assist -- the leg
  stops hit first.  (Hardware check still recommended:实物全伸安装距 −
  8 mm ≥ ~0.205 m.)

Ruling (user, 2026-09-22): the constraint is retracted; the thrust law is
restored to the original motor-assisted version (weight share + PID toward
0.305 m + 70 N boost / 120 N floor).  Battery green again; measured apex
returns to the ~0.33 m body rise / 0.31 m wheel clearance of the
pre-experiment machine.
