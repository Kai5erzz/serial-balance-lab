# Stair descent & stair-climb development record

Last updated: 2026-08-27 (night session)

## Target specification (user)

1. Descend the 3-step platform at essentially any reasonable speed, with
   heading error within +-30 deg of face-normal, driving forward OR backward,
   at mid/high leg lengths.  Low leg length is explicitly out of scope.
2. Try jumping UP the three tiers (experimental; mode 7 demo).
3. Documentation + a real-vehicle migration plan.

## Current status

| Item | State |
| --- | --- |
| Edge geometric probe | WORKING on corrected terrain (returns 0.94 m at deck centre) |
| Guard preview arming | WORKING (arms ~0.6 m before lip with velocity-true direction) |
| Approach brake | Engages and self-calibrates sign per 120 ms window; slows but does not yet fully stabilise the drop |
| Full descent SAFE | NOT YET - nose-over at the 200 mm lips remains the failure mode across 0.4-0.7 m/s |

## Root causes found tonight (all verified experimentally)

### R1. Two parallel descent-guard implementations diverged
`_update_off_ground` and `_update_descent_guard` EACH computed their own
heading/direction.  Fixing only one left the guard probing the receding
boundary.  Both now use measured travel direction:

    direction = heading * (-sign(chassis_vx))   # |vx| > 0.05
              = cmd-sign fallback otherwise

because in this CAD export the visible nose lies along `-forward_world`, so
a positive UI command physically drives against `+heading`.

### R2. The six-state LQR velocity row cannot regulate this plant at all
Speed-ladder probes (flat ground, no governor):

    cmd=0.2: vx drifts 0.23 -> 1.05 m/s over 9 s (never settles)
    cmd=0.4: reaches 1.7 m/s
    cmd=0.8: chaotic divergence past ~2.4 m/s

A P-only velocity row on an essentially frictionless rolling plant has no
torque-free equilibrium except err=0, and empirically it behaves as a slow
runaway.  The firmware polynomials presuppose motor/gear viscous droop that
MuJoCo's ideal axle does not reproduce.  Consequence: EVERY approach to a
ledge overshoots its commanded speed badly before any guard can act.

### R3. Sign-chain minefield around wheel output writing
`step()` writes `ctrl[wheel] = wheel_sign[side] * last_output[side][0] - yaw`
with `wheel_sign = {l:+1, r:-1}` -- a PER-SIDE mount mirror inside the common
output path.  Any externally injected torque term therefore interacts with
(a) that per-side mapping and (b) every frame translation documented in
`_state`.  All 6 attempted governor variants tonight behaved differently from
their intended polarity until this topology was mapped.  The approach brake
sidesteps the debate entirely by calibrating its own sign in-flight.

## Tonight's dead ends (kept for archaeology, all reverted/disarmed)

* Ownership-transfer cruise governor (`DRIVE_CRUISE_ZERO_REF`,
  PI outer loop): destabilised pitch (18.5 deg falls); disarmed via
  `DRIVE_CRUISE_MIN_COMMAND = 99`.
* `LQR_XDOT_SIGN = -1`: froze the axle under command (mean speed ~0),
  stand still fine.
* Synthetic drag injection (`CRUISE_DRAG_COEF`): removed oscillation but the
  inner row still set the cruise point ~2.5x over request.
All code paths remain present, switched off, with comments.

## Next-session plan for T1 completion

1. Frame-by-frame capture of one lip crossing (0.4 m/s, mid legs): dump hip
   torques, `landing_air_deploy_ref`, GBUF phase vs time.  Failure signature
   is nose-over AFTER first-contact, i.e. GBUF/deploy timing rather than
   approach braking is the remaining lever.
2. Tune pre-lip retract depth against measured deploy-capture window:
   `DESCENT_EDGE_RETRACT_LENGTH 0.130` may over-shorten for high presets;
   try preset-relative target `min(selected, retract_len)`.
3. Then run `speed x angle x direction x leg-length` matrix (runner spec
   below) and iterate constants until the user band passes.

Matrix runner spec (not yet written):

    python test_descent_matrix.py            # full grid, prints table
    env: SPEEDS="0.3 0.6 0.9" ANGLES="-30 0 30"
         DIRS="fwd back" LEGS="mid high"

acceptance per cell: survives to z<=0.25 m rest pose, max_pitch<=20 deg,
max_roll<=12 deg, chassis never grounds, both wheels regain support and
stay grounded >= 2 s.

## mode 7: stair-climb (item 2), feasibility memo

Single hop onto the 600 mm top deck requires takeoff vertical speed

    v = sqrt(2 g h) ~= sqrt(2 * 9.81 * 0.63) ~= 3.5 m/s

against today's measured ~2.06 m/s (0.42 s airborne).  Thrust stroke is only
~0.18 m, so mean acceleration would need ~33 m/s^2 => ~550 N of thrust
against a 200 N/leg axial support cap and a 29.7 Nm observed hip peak.
**Physically impossible inside the actuator envelope.**

Per-tier hops (+200 mm each) need v >= ~2.15 m/s -- marginally above the
current jump, so tonight's conclusion is "borderline, worth instrumenting":
`mode7_stair_climb_demo.py` executes chained jumps toward each tier and logs
the achieved clearance/position per attempt.  Landing on the 300 mm tread
rings additionally demands lateral precision the current open-loop hop does
not aim for; treat every positive result as luck until aimed landing exists.

## Real-vehicle migration plan (see SIM_TO_REAL_MIGRATION.md)

Structure/logic that transfers directly:
* Guard preview + measured-direction rule (hardware equivalent: rely on the
  force observer; geometry preview is sim-only).
* Self-calibrating brake pattern: on hardware, calibrate the brake polarity
  once at bench (log wheel-encoder response to a fixed pulse) instead of
  in-flight flipping.
* Startup suppression window semantics (shadow-run Ground_Detect after
  recovery handoff; log first, enforce later).
Numbers (lookahead 0.6 m, probe 120 ms, KP 1.6) are MuJoCo-fit and must be
re-derived against identified plant data per RECOVERY_PORT_NOTES.

## mode 7 update (2026-08-27, climb session)

Reproducible tier-1 mount achieved headless:
`python mode7_climb_viewer.py --headless 0.30 2.05`
=> support=platform_step2 (0.2 m), both-wheel contact, pitch +8.3 deg.

Lessons that made it work:
1. The CAD nose faces `-forward_world`; the previous demo spawned the robot
   facing AWAY from the pyramid.  Climb spawn now sits on the diagonal with
   nose at centre (`spawn_facing_stairs`).
2. `begin_jump()` clears the drive command for landing safety, which killed
   run-up momentum.  The climb re-applies the command during preload/thrust
   and stops at flight.
3. The descent guard reads the uphill wall as a drop edge and brakes the
   run-up; climb mode sets `controller.disable_descent_guard = True`
   (new opt-out switch in `_update_descent_guard`).
4. Working window is narrow: approach 0.30 m/s, jump trigger 2.05 m from
   centre (wall at 1.6 m).  Closer triggers slam the riser mid-arc; farther
   ones land short.

Remaining for full acceptance (both wheels LEVEL on the tread):
* Landing settles at +8.3 deg (rear wheel straddling the lip); the second
  hop that would lift it is rejected by the firmware 8 deg pitch precheck.
  Next lever: land flatter (more carry) or null pitch with hip authority
  before the second hop.
* LQR approach speed still overshoots ~2x command (T1-A runaway); the jump
  trigger works around it by distance gating.

## REVERT (2026-08-27, later): descent back to the previous night's baseline

Tonight's approach-brake experiment (0.60 m lookahead, self-calibrating
wheel brake, measured-travel direction in both guard functions) made the
descent WORSE -- arming the guard 0.6 m early engaged the pre-edge retract
and brake at full approach speed, destabilising the chassis BEFORE the lip.
User call: restore the known state.  Reverted:

* DESCENT_GUARD_EDGE_LOOKAHEAD 0.60 -> 0.25
* _update_approach_brake method, call site, constants, state: removed
* direction selection in _update_off_ground / _update_descent_guard
  back to command-sign (last night's form)

Kept (inert by default): controller.disable_descent_guard opt-out switch
used by the climb scripts.  The nightly descent signature at 0.4 m/s is
again "no pre-lip guard action; face-plant on the next tread at ~5.3 s".

The T1 root causes (R1-R3 in this file) remain valid and are unaffected.

## 2026-08-27 evening: fall self-rescue + liberalised jumping

Ported from chassis_safety.c Security_Checking():
  hard gates (immediate): |pitch|>60 deg, |roll|>60 deg, phi0 outside
  [20, 160] deg; soft gates (grounded only, 0.08 s debounce): pitch>35,
  roll>30, |theta|>45 deg.  Confirmed danger with the machine idle hands
  control to begin_recovery() (the ported firmware DANGER->RECOVERY route;
  >=60 deg pitch takes the prone-torque path, else the cad-stand route).
Verified end-to-end on the platform descent: face-plant at t=5.37 s ->
soft_theta danger at t=6.24 s -> cad_stand_recovery -> stand at t=8.75 s,
chassis level again.  Caveat: the cad-stand handoff ends with legs at
l0=0.335 m (spread-eagle squat, chassis z=0.158) instead of the 0.150 m
stance - the normal length loop cannot retract against the gas spring at
that geometry.  Next-session item.

Jump policy (simulation-only, per user): moving jumps allowed at ANY
ground speed - precheck vx gate bound raised to 10 m/s (effectively off),
attitude gates 25 deg, in-flight abort 40/30 deg, landing gates 12/10 deg.
Test acceptance now scales the airborne-pitch bound with forward target
and tolerates chassis brush inside the touchdown window (belly-flop before
first wheel contact remains a hard fail).  RC speed step raised to 1.00 m/s
in run_lqr.py.  Moving jump verified at 0.2 and 1.0 m/s (full state machine
traversal, 2.44 m carry at 1.0).

## 2026-08-27 night II: firmware recovery module port (REC_SIMPLE)

User call: follow the real chassis_recovery.c, do not invent.  Ported the
pose-agnostic recovery: begin_fall_recovery() (REC_SIMPLE_ENTER seeds the
current pose, no encoder-zero gate), HOLD three-way classification
(inverted >=105 deg pitch / legs-not-forward-zero -> FORWARD_ZERO / else
SWING), the INVERTED sub-machine (CAPTURE 0.2 s -> PRELOAD 0.25 s -> TURN
with 105 deg/s sweep, sync Kp 1.0 corr<=37.5 deg, wait-leg 12/6 deg, TP
36/2.6 lim 9.5 + bias 4.0, contact force 12 N -> SETTLE 0.5 s), phase-10
FORWARD_ZERO (110 deg/s scaled to shared completion, TP 16/2.2 lim 12 Nm --
the firmware comment about the old 3.75 Nm dead-point stall matched our
first attempt exactly), FAULT with 1.5 s retry (documented deviation from
the latching firmware fault).  Danger handler now routes to this machine.

Verified: classification and routing run correctly from the descent crash
pose (phi 151 deg, chassis flat).  Baseline battery 4/4 PASS.

OPEN ITEM (the legs-up jam): from the belly-flat fallen pose this CAD
export cannot execute the sweep in either direction -- swinging down arcs
the 0.335 m wheels into the ground, and the four-bar range tops out near
163 deg so the firmware's positive +360 sweep is unreachable (we widened
the side-pose window 150 -> 165 deg so the reachable signed short path
runs; it still needs belly clearance the model does not have).  The real
robot executes phase 10 because its leg travel and belly clearance differ.
Next levers: pre-shorten l0 before the sweep (smaller wheel arc), or lift
the belly with a hip-prime pulse, matching the real robot's geometry.
Acceptance must use controller.is_properly_standing() (wheels bearing,
phi~90, l0<=0.22, z>=0.19, level) -- pitch/roll alone read zero for a
belly-flat chassis.

## 2026-08-27 night III: step-up gait investigation (walk-up attempt)

Built stepup_gait.py: W1 LQR approach + wall stall (works), W2 compress/
extend seat against the riser (works - chassis leans nose-on-wall at
z=0.264, pitch -24), W3 wheel placement FAILS.  Root cause chain:

1. Forward swing is kinematically capped: the four-bar forward stop sits at
   phi~20 deg (encoder-zero 18.5 deg is effectively the dead point).  Probe:
   commanding phi 138 from stand drove the leg the OTHER way and pinned it
   at 22 deg.  Wheels can never be placed forward-up onto a tread.
2. Rear swing IS wide (151-163 deg reachable) but the wheel arc height is
   not: with the hip at best ~0.40 m (legs at max extension), a rear wheel
   at phi 130 sits at ~0.17 m - below the 0.29 m needed to rest ON a
   200 mm tread.  Static placement is out of reach both ways.

Conclusion: with a single axle, a 20..163 deg leg envelope and 0.33 m max
leg, STRICT stepping onto a 200 mm stair is kinematically impossible -
which is exactly why the real machine jumps.  The remaining walk-ish
option is the hybrid: lean the nose on the lip (W2, already working) and
execute a SMALL hop from that posture - the lip support halves the required
apex height (~0.10-0.15 m instead of 0.2+) and the wall bounds the landing
window.  That hybrid is the recommended next build; the jump machinery
(preload/thrust/landing) already exists to execute it.

## 2026-08-27 night III (cont.): the unified blocker is a SIGN/polarity map

A finer probe (compress -> extend-lean -> sweep, per-0.4 s prints) showed
the legs fold to the forward stop (phi 26 deg) during compress and CANNOT
be pulled back even with 30 Nm: in this closed-chain branch POSITIVE hip TP
drives phi DOWN (into the stop), while the fw_swing encoder-zero route had
positive TP driving phi UP.  The TP->phi sign is branch-dependent (per-side
mirrored motors x closed-chain branch), and with the legs pinned at the
stop the l0 PD just folds the chassis to the z=0.158 belly rest - wheels
never leave the ground.

Consequence: every open-loop gait sequence gambles on polarity.  The
prerequisite for ANY stepping/rescue gait is a systematic polarity map:
sample poses across (l0, phi) space, apply short +-2 Nm hip pulses, record
phi response direction - one calibration script, ~30 minutes.  With that
map (as a lookup the gait uses to pick TP signs per branch), the lean +
mini-hop hybrid becomes executable, and the same map fixes the wedge-pose
rescue from night II.

## 2026-08-27 night III (final): belly-rest is an inescapable attractor

stepup_v2 added an adaptive relay sign for the phi channel (flip when the
error is not shrinking).  Result: identical freeze at the belly-rest state
(z=0.158, phi 23-26 deg, wheel_z 0.058).  The relay cannot help because
the pinning is not a polarity problem: once LQR releases and the chassis
collapses (~0.5 s), the gas springs plus the ground geometry press the
folded legs against the forward stop with more than the 24-30 Nm the hip
motors can generate.  Every direct-VMC sequence that passes through
belly-rest is dead.

Design law extracted: a gait must never pass through belly-rest.  It must
operate in the ~0.5 s window between wall-stall and collapse, or actively
hold a propped pose the way FW recovery HOLD does (leg forces sized to the
pose).  The two viable continuations:

1. Lean-hop hybrid executed by the JUMP machine (proven forces): from the
   wall-stall, a reduced-amplitude preload/thrust with the nose against
   the lip halves the apex requirement and the wall bounds the landing.
2. Drive deliberately into a HIGH propped pose (nose on lip, legs long,
   actively held like recovery HOLD) and run the fw recovery SWING ->
   RETRACT from there - the recovery's own force profile is already tuned
   to stand from a propped pose.

Both reuse proven machinery; neither needs a new sign map.  Baseline
regressions remain green after all tonight's additions.

## 2026-08-27 night IV: chained climb attempt (climb_chain.py)

Clean 3-tier chain built: per tier - LQR approach at set speed, jump at a
distance trigger, re-drive through preload/thrust for carry, post-landing
creep, support-level acceptance + is_properly_standing verdict.

Data: the landing window is razor thin.  (0.30, 2.05) landed vx=0.51 ->
just short (support=ground); the same nominal parameters in the earlier
script landed vx=0.55 -> straddled ON step2 (support=platform_step2,
pitch 8.3).  A 0.04 m/s takeoff-speed difference flips success/failure;
(0.40, 2.05) landed short again, (0.50, 2.05) nose-dived into the riser
(pitch -34.8).  The takeoff speed is not repeatable because the approach
velocity loop is still the open T1-A runaway.

Verdict: the jump-climb mechanism works mechanically (straddle reproduced,
precheck no longer blocks the follow-up hop); reliability is gated by
commanded-speed regulation.  T1-A (cruise governor on the identified
plant, with the polarity map) is the prerequisite - after it lands, the
climb is a parameter sweep away.
