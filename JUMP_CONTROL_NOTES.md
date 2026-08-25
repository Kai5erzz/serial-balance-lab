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

1. **Preload and thrust:** the VMC length route compresses to `0.125 m` and
   thrusts toward `0.305 m`. While wheel contact still exists, a pitch-rate
   loop establishes the angular momentum needed to cancel the later leg
   motion. The stationary target is `1.48 rad/s`; moving launch subtracts a
   term proportional to measured launch velocity.
2. **Compact ascent:** the virtual-leg reference moves to `0.140 m` at
   `3.50 m/s`. The compact interval is `0.23 s`, which includes the apex.
   Body pitch and leg angle share the hip common axis; pitch is prioritized at
   the apex while the drive wheels provide a real reaction-inertia axis.
3. **Landing deployment:** the length target returns to `0.270 m`; leg-angle
   authority rises after the compact interval so both wheels point toward the
   ground before contact. The wheel reaction term includes feed-forward from
   the commanded leg swing, rather than waiting for body pitch to diverge.
4. **Touchdown capture and buffer:** the first raw wheel contact immediately
   starts a `40 ms` capture window. The wheel path stops using the airborne
   reaction-wheel request and approaches wheel pitch PD (`Kp=10`, `Kd=1`) with
   a bounded command step. The hip path keeps the airborne leg controller until
   both wheels pass the existing eight-sample contact debounce, then slews into
   independent landing leg-angle PD (`Kp=30`, `Kd=6`). The axial path blends
   from the airborne force into the landing buffer over the same capture
   window. Contact debounce now decides only the state transition; it no longer
   leaves all actuators in air mode after a real wheel has touched the ground.
   The buffer then hands all three paths back to the original LOW LQR.

## Acceptance results

| Model / launch | Base rise | Apex wheel clearance | Apex pitch | Air pitch range | First wheel pitch | Touchdown-window pitch | Final pitch/roll | Non-wheel ground contact |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | --- |
| Fast, stationary | `0.342 m` | `0.330 m` | `+0.64 deg` | within `+/-7.23 deg` | `+3.68 deg` | within `+/-6.11 deg` | `+0.09/+0.22 deg` | none |
| Fast, `+0.20 m/s` | `0.342 m` | `0.329 m` | `+2.03 deg` | within `+/-8.79 deg` | `+6.99 deg` | within `+/-8.79 deg` | `-0.10/+0.22 deg` | none |
| Full collision, stationary | `0.346 m` | `0.330 m` | `-2.05 deg` | within `+/-7.22 deg` | `-2.56 deg` | within `+/-6.93 deg` | `+0.23/+0.00 deg` | none |

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
