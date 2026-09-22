# Encoder-zero recovery port record

Last updated: 2026-09-22

This note records the calibrated encoder-zero pose, the embedded recovery
state machine port, every intentional simulation-only deviation, and the
measurements needed to remove those deviations later. It is the handoff
record for the next system-identification pass.

## Scope and source of truth

- Embedded source:
  `HNU_Embedded_Framework/src/task/chassis/recovery/chassis_recovery.c`.
- Simulation controller: `traditional_lqr.py`.
- Closed, calibrated reset pose: `encoder_zero_pose.json`.
- The recovery route is `HOLD -> SWING -> RETRACT -> LQR HANDOFF -> STAND`.
- Wheel torque is exactly zero before LQR handoff, matching the embedded
  route. No wheel-based recovery assist or pitch-holding controller was
  added.
- The firmware 25 degree pitch fault gate remains enabled and unchanged.
- The existing firmware LOW gain schedule, position observer, theta
  compensation, torque signs, and VMC Jacobian are used after handoff.

## Verified calibration anchors

| Anchor | Swing | Compression | VMC l0 | VMC phi0 |
| --- | ---: | ---: | ---: | ---: |
| Real encoder zero / prone recovery start (Z_new, 2026-09-13) | -77.929530 / -76.283757 deg (l/r) | +27.000 deg | 0.332557082 m | 17.070469 / 18.716243 deg (l/r) |
| Vertical maximum length | -5.000 deg | +27.000 deg | 0.332557082 m | 90.000 deg |
| Vertical minimum length | -5.000 deg | -20.000 deg | 0.116399057 m | 90.000 deg |
| Normal stand target | -5.000 deg | -10.573590 deg | 0.150000000 m | 90.000 deg |

The encoder-zero active CAD hinges saved in `encoder_zero_pose.json` are
the unified Z_new (2026-09-13):

- left J1 `+0.888888 rad`, left J2 `+1.831366 rad`;
- right J1 `-0.860164 rad`, right J2 `-1.802642 rad`
  (`E = Q − Z_new`; legacy Z_old differs by `+0.024862750` rad left,
  `+0.003861417` rad right).

The pose is the user-confirmed flat posture (root height
`0.157445821 m`, both tires on the same ground, closure residual
≈2.2e-7 m). It replaces the earlier pose whose `-1.333 deg` roll workaround
hid an 8.0 mm left-tire / 1.2 mm right-tire ground mismatch. The VMC rod
angles are re-anchored per side so the FK on every pose stays bit-identical
to the legacy mapping (`tests/test_vmc_calibration.py` proves the identity).

The calibrated passive closure limits are:

- `l_leg`: `[-0.4155, +0.7598] rad`;
- `r_leg`: `[-0.7611, +0.4100] rad`
  (measured 2026-09-21 leg-limit travel ±0.01 rad softness margin;
  rl-serial-balance `configs/calibration/leg_limits_2026-09-21.json`).

These limits stay active during recovery. Calibration mode is the only mode
that temporarily disables leg limits.

## Intentional simulation-only differences

There are no controller-side force or motion-rate reductions in upright
recovery. The VMC length PD, gas-spring cancellation, retract bias, breakaway
force, reference speeds/accelerations, gain ramps, and torque reserves now use
the `chassis_recovery.c` values directly. Missing motor/friction dynamics must
be added to the plant model rather than hidden in controller multipliers.

The recovery target is `l0=0.150 m`, per the calibrated normal stance, rather
than the firmware build's `LEN_LEN_LOW=0.130 m`.

An invisible `recovery_skid` contact surface represents the real chassis
resting on the floor in the prone encoder-zero posture. It is a passive
collision surface, not a constraint or controller. At the normal standing
pose its measured ground clearance is `52.589 mm`; the LQR regression confirms
that neither it, the chassis, nor a leg contacts the floor while balancing.

The earlier port also omitted the firmware transition which sets both retract
force gains to `1.0` when the 180 ms breakaway envelope releases. The flag was
initialized but never consumed, so force fell back to the slow `0.9/s` ramp.
That transition and the firmware force-calculation order are now restored.
The recovery regression separately requires each leg to reach `l0 < 0.170 m`
during `FW_RETRACT`, before LQR handoff.

## System-identification work needed

Progress (2026-09-22): items 1 and 2 now have a first accepted
implementation — `identified_2026-09-21.yaml` carries the tensioned
per-side motor damping/friction/armature/gains and the passive-chain
friction/damping from the 2026-09-20 real2sim fit (see docs/SYSID.md),
and the VMC length PID was re-derived against those identified plants.
The remaining items (gas spring, contact footprint, encoder/IMU timing)
are still open.

To further close the sim-to-real gap, identify and add:

1. DM-J8009P output-side reflected inertia, torque rise time, command delay,
   torque/speed saturation, viscous drag, Coulomb friction, and stiction.
2. Passive and driven linkage-joint friction, backlash, and stop compliance.
3. Gas-spring force versus stroke, mounting geometry, hysteresis, and damping.
4. Chassis prone contact footprint, contact compliance, and floor friction.
5. Link/chassis mass, centre of mass, and pitch inertia.
6. Encoder/IMU filtering and timing relative to the 5 ms chassis task.

Recommended identification order is motor output dynamics, gas spring, joint
friction, then contact. Do not reintroduce controller scaling to compensate;
rerun both recovery models plus the ordinary LQR regression after each update.

## Current acceptance results

Fast model, 12 s headless run:

- handoff at about `1.100 s`, stand at about `1.185 s`;
- minimum `FW_RETRACT` `l0=0.106716/0.106695 m` before handoff;
- maximum recovery pitch `10.430 deg`, below the firmware 25 degree gate;
- maximum hip-motor torque `24.000 Nm` during the source breakaway envelope;
- final `l0=0.153855/0.152490 m`;
- final `phi0=84.624/84.208 deg`;
- final pitch `-0.133 deg`.

Full collision model:

- handoff at about `1.130 s`, stand at about `1.215 s`;
- minimum `FW_RETRACT` `l0=0.106535/0.106727 m` before handoff;
- maximum recovery pitch `12.046 deg`;
- maximum hip-motor torque `24.000 Nm` during the source breakaway envelope;
- final `l0=0.153252/0.153195 m`;
- final `phi0=84.538/84.044 deg`;
- final pitch `-0.119 deg`.

The ordinary calibrated LQR regression is unchanged: with the pitch pulse,
maximum pitch is about `0.538 deg`, wheel saturation is zero, and no chassis,
leg, or recovery-skid ground contact occurs.

## Viewer deadlock fix

The intermittent apparent crash after opening `run_lqr.py` was a viewer
deadlock, not an LQR or recovery failure. `update_info()` was called while an
outer `viewer.lock()` was held, but this MuJoCo viewer's `set_texts()` enters
the native viewer mutex internally. When its timing overlapped the GLFW
render thread, the main thread remained blocked in `viewer.py:set_texts`.

The outer lock has been removed, telemetry text is limited to 10 Hz, and
`viewer.sync()` remains the state/render synchronization point. A clean test
kept the paused viewer responsive for 30 seconds and then ran encoder-zero
recovery plus LQR for more than 26 simulated seconds without deadlocking.
The camera now tracks `base_link`, so longitudinal wheel motion does not make
the robot disappear from a fixed camera view.

Run the checks from this project directory:

```powershell
python test_vmc_calibration.py
python test_firmware_recovery.py
$env:MUJOCO_MODEL = (Resolve-Path 'model\whole_body_lqr.xml')
python test_firmware_recovery.py
Remove-Item Env:MUJOCO_MODEL
python test_calibrated_lqr.py
```
