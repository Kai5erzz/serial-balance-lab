# Turn-roll compensation: real-vehicle port and the speed-runaway root cause

Session record, 2026-09-05 (overnight). Scope: "low leg length + hard
turning leans over" and the roll compensation found in the real-vehicle
code (`code_611/HNU_Embedded_Framework`, the actively developed copy —
its chassis_control.c mtime postdates the top-level checkout, and its
jump gates carry real-robot tuning comments).

## What the real vehicle has that the simulation lacked

`Leg_FN_Calculation` in `chassis_control.c` composes the differential
support (`support_l += F`, `support_r -= F`) from four channels; the sim
only had the first:

1. **Roll PD** — already ported (`ROLL_NORMAL_KP 3.5`, `KD 0.10`,
   l0-scheduled gain 1.0→2.09 across 0.140→0.260 m).
2. **Centrifugal (lateral inertial) feedforward** — `LATERAL_INERTIAL_FF_*`:
   predicts the lateral acceleration of commanded circular motion and
   pre-positions the support split through the load-transfer lever:
   `F = gain(l0) · m · (vx_ref · yaw_rate_ref) · l0_avg / (2 · 0.214)`,
   clamped ±50 N, slewed 100 N/s, dead-band 0.2 m/s², gain scheduled
   LOW 1.30 / MID 1.15 / HIGH 1.15 (mid ≥ 0.170 m, high ≥ 0.245 m).
   Gated to normal grounded driving. **Ported** to
   `TraditionalLQR._update_roll_lateral_ff`.
3. **Static roll trim** — integrator (`KI 1.2`, limit 8 N, applied 6 N
   slewed 12 N/s) that learns only while settled (speed/yaw/roll-rate
   dead-bands) and holds through motion. **Ported** to
   `_update_roll_static_trim` (NORMAL profile; the spin-mode variants
   stay firmware-only, the sim has no spin mode).
4. **Slope/geometry compensation + spin schedule** — NOT ported: the sim
   ground is flat and the viewer has no spin mode.

Sign resolution (empirical, closing the old "boundary-sign" question):
`chassis_vx` is UI-front-positive while a **positive common wheel output
drives the vehicle backward** (a railed +1.2 Nm trim settled a +0.5 m/s
request at −0.5 m/s). The feedforward therefore applies the firmware
formula against the commanded product directly and lets the roll-PD
polarity (already verified) fix the frame; the result correlated +0.94
with the measured lateral acceleration.

## The turn bench and what it exposed

`tools/probes/turn_roll_bench.py` drives steady turns
(0.5 s command ramp, like an RC push) and measures signed roll, slip,
and turn-entry peak. Baseline findings:

- (0.130 m, 0.5 m/s, 2.0 rad/s): steady lean 3.6°.
- (0.130 m, 1.0 m/s, ±2.0 rad/s) with a ramped command: the chassis
  **flips over** (|roll| → 179°) — pre-existing, not caused by the port.

The failure timeline (0.25 s resolution) shows the mechanism: the
six-state LQR's velocity row cannot regulate speed on the ideal
(frictionless-axle) plant, so vx crept from the 1.0 m/s request to
2.1+ m/s; the turn radius kept tightening past the roll channel's
authority; the right wheel saturated at 3.69 Nm; the yaw loop collapsed;
the chassis fell. This is the documented T1-A runaway, and it — not the
missing feedforward — is what kills hard low-leg turns.

## What was changed

1. **Roll compensation port** (above) — `balance_lab/control/tuning.py`
   section "Real-vehicle roll compensation port", methods on
   `TraditionalLQR`, applied in the normal-support branch only.
2. **Forward cruise governor armed** (`DRIVE_CRUISE_MIN_COMMAND 99 →
   0.10`, `DRIVE_CRUISE_LIMIT 1.20 → 2.20`): the PI outer loop that was
   left disarmed in the 2026-08-27 session now closes the commanded
   speed. Two additions beyond the old attempt:
   - anti-windup: the integral freezes while either wheel is saturated
     (the old 18.5° pitch falls were exactly stored trim releasing after
     a saturated wheel desaturated);
   - the trim is applied **negatively** to the wheel outputs (the sign
     resolution above); with the old direct application a +0.5 m/s
     request settled the vehicle at −0.5 m/s.
   Result: cmd 1.0 m/s now tracks at **err 0.000, std 0.005 m/s**,
   pitch ≤ 1.9°. Reverse commands are explicitly excluded (`cmd < 0.0`
   gate) — see the open issue below.
3. **Turn-stability regression** — `tests/test_turn_stability.py`: five
   turn cases must stay alive, ≤ ~8° entry peak, ≤ ~4° steady lean.
   Measured after the fix: no falls anywhere; (1.0, ±2.0) entry peaks
   6.2°/4.9° (was: flip at +2.0); steady lean ≤ 2.8° at the 0.130 m
   preset.

## Open issues (next session)

- **Reverse driving diverges.** With the governor holding forward
  speeds, a −1 m/s request still runs away. Trace: the six-state
  velocity row itself pushes AWAY from a negative setpoint (at vx=+1,
  cmd=−1 it contributes ~−3.3 Nm of forward push) — for negative
  commands the row is positive-feedback. Flipping `LQR_XDOT_SIGN`
  stabilizes the row but destabilizes pitch (14–16°), reproducing the
  2026-08-27 result. The row's plant sign is inverted somewhere past
  `wheel_sign` (mirrored wheel axes / gear path); a clean fix needs a
  full wheel-output sign audit, not another patch.
- Turn-entry peaks at the extreme cases ((1.5 m/s, 2.0 rad/s) and high
  leg) grew to ~9° because the vehicle now genuinely reaches the
  commanded speed (turns are tighter than the old creeping entries).
  If that reads as too aggressive, the LOW-schedule feedforward gain
  (1.30) is the tuning knob.
- The static trim integrator never engages during driving by design;
  its benefit (learning CAD trim asymmetry, e.g. the −1.333° pose roll)
  only shows in long straight runs.
