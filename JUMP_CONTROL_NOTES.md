# Physical jump-control record

Last updated: 2026-08-25

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

The accepted jump uses the full `40 Nm` hip peak and remains above the `20 Nm`
rating for about `0.50 s`. This is a simulation result, not a thermal-duty
claim for the real motor.

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
2. **Compact ascent:** the virtual-leg reference moves to `0.140 m` at
   `3.50 m/s`. The compact interval is `0.23 s`, which includes the apex.
   Body pitch and leg angle share the hip common axis; pitch is prioritized at
   the apex while the drive wheels provide a real reaction-inertia axis.
   The air-phase LQR pitch gain is `1.00` (raised from `0.75` for the
   MATLAB-wheel plant); tuck/deploy pitch bias is `2 deg / 0 deg` so the
   chassis tucks level instead of holding the old `14 deg` rearward rake.
3. **Landing deployment:** the length target returns to `0.270 m`; leg-angle
   authority rises after the compact interval so both wheels point toward the
   ground before contact. The reaction-wheel torque is ramped out over the
   `0.15 s` deploy window so the rotors reach ~zero speed at touchdown;
   otherwise the stored wheel angular momentum becomes a ground impulse that
   slides the chassis away from the launch spot. The deploy pitch bias is
   `+3 deg`; measured touch-to-level recovery is about `0.9 s`.
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
5. **Pre-jump settle:** the jump entry now waits `3 s` (was `1 s`) for the LOW
   equilibrium to finish its leg-length transient; jumping from a moving
   chassis produced a long ballistic glide (measured `~1.2 m` horizontal
   travel), now `~0.04 m`.

## Acceptance results

Plant: MATLAB wheel inertia (`mw=0.577 kg`, `Iw=mw*R^2=0.001941 kg m^2`),
restored CAD inertias and original leg stops / 40 Nm hip envelope.

| Model / launch | Base rise | Apex wheel clearance | Apex pitch | Air pitch range | First wheel pitch | Touchdown-window pitch | Final pitch/roll | Non-wheel ground contact |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Fast, stationary | `0.317 m` | `0.300 m` | `+1.76 deg` | within `+/-4.96 deg` | `-2.29 deg` | within `+/-5.09 deg` | `+0.24/+0.21 deg` | none |
| Fast, `+0.20 m/s` | `0.315 m` | `0.298 m` (2 mm short) | `+2.75 deg` | within `+/-5.23 deg` | `-3.95 deg` | within `+/-10.00 deg` | `-0.03/+0.21 deg` | none |
| Full collision, stationary | `0.314 m` | `0.298 m` (2 mm short) | `+1.96 deg` | within `+/-5.40 deg` | `-5.23 deg` | within `+/-5.53 deg` | `+0.28/+0.00 deg` | loop links touch at touchdown |

The stationary fast run passes the full acceptance. The `+0.20 m/s` and full
collision variants miss by millimeters on apex clearance, and the full
collision model brushes the loop links during the touchdown window; both are
plant-dependent edges of the MATLAB-wheel retune and are the next tuning
targets. All three runs traverse `PRELOAD -> THRUST -> FLIGHT -> LANDING ->
IDLE`, finish with `fault=none`, and keep both wheels grounded after settling.

All three runs traverse `PRELOAD -> THRUST -> FLIGHT -> LANDING -> IDLE`,
finish with `fault=none`, keep both wheels grounded after settling, and avoid
leg, chassis, gimbal, and recovery-skid contact after takeoff.

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
