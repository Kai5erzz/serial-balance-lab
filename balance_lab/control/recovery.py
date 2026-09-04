"""Fall and encoder-zero recovery state machines ported from chassis_recovery.c.

Mixin part of :class:`balance_lab.control.controller.TraditionalLQR`; every
method accesses the shared controller state through ``self``.
"""

from __future__ import annotations

import mujoco  # noqa: F401
import numpy as np

from .tuning import *  # noqa: F401,F403
from .kinematics import RecoverySwingAxis, VmcKinematics, unit


class RecoveryMixin:
    "Fall and encoder-zero recovery state machines ported from chassis_recovery.c."

    def capture_recovery_target(self, data: mujoco.MjData) -> None:
        """Capture the CAD q=0 target associated with the encoder screenshot."""
        self.recovery_target = {side: self.vmc_kinematics(data, side) for side in ("l", "r")}
        self.recovery_physical_length = {
            side: self._leg_measurement(data, self.sides[side])[1]
            for side in ("l", "r")
        }

    def _start_cad_stand(self, data: mujoco.MjData, enable_lqr: bool) -> None:
        """Initialise the CAD-zero VMC trim, optionally arming LQR now."""
        self.enabled = enable_lqr
        self.stance_enabled = True
        self.gas_enabled = True
        # Direct CAD-zero enablement is already inside the validated local
        # basin.  Start wheel feedback at full gain and enter the VMC blend
        # near its standing end.  Starting both ramps from zero leaves almost
        # no restoring torque for the first 0.2 s, which is long enough for
        # the chassis to hit the floor.  The delayed recovery route below
        # still starts at zero and preserves its contact-gated semantics.
        self.lqr_gain = 1.0 if enable_lqr else 0.0
        self.recovery_state = "cad_stand_recovery"
        self.recovery_elapsed = CAD_DIRECT_START_ELAPSED_S if enable_lqr else 0.0
        self.recovery_stable_elapsed = 0.0
        self.prone_swing_elapsed = 0.0
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.recovery_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.cad_stand_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.cad_stand_wheel_origin = data.xpos[self.base].copy() if enable_lqr else None
        self.cad_stand_pitch_previous = self.body_pitch(data)
        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            self.cad_stand_last[side] = (kin.l0, kin.phi0)

    def begin_recovery(self, data: mujoco.MjData | None = None) -> None:
        """Run the retained legacy gas-trim recovery experiment."""
        if data is None:
            raise ValueError("Recovery requires the current MuJoCo state")
        self._start_cad_stand(data, enable_lqr=False)
        # When the free chassis is face-down, first execute the retained
        # torque-only swing pulse rather than handing LQR a flying body.
        if abs(self.body_pitch(data)) >= PRONE_ENTRY_PITCH:
            self.enabled = False
            self.cad_stand_wheel_origin = None
            self.recovery_state = "prone_swing"

    def begin_encoder_zero_recovery(self, data: mujoco.MjData) -> None:
        """Arm the firmware upright recovery from the calibrated encoder zero."""
        self.reset(data)
        errors: list[str] = []
        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            if abs(kin.l0 - VMC_ENCODER_ZERO_L0) > 2e-5:
                errors.append(f"{side}.l0={kin.l0:.9f}")
            if abs(self._wrap_pi(kin.phi0 - VMC_ENCODER_ZERO_PHI0)) > np.radians(0.02):
                errors.append(f"{side}.phi0={np.degrees(kin.phi0):.6f}deg")
            self.recovery_last_l0[side] = kin.l0
            self.recovery_last_phi0[side] = kin.phi0
            self.fw_recovery_l0_ref[side] = kin.l0
            self.fw_recovery_length_rate[side] = 0.0
        pitch = self.body_pitch(data)
        if abs(pitch) > np.radians(0.02):
            errors.append(f"pitch={np.degrees(pitch):.6f}deg")
        if errors:
            raise ValueError("Encoder-zero recovery start rejected: " + ", ".join(errors))

        self.enabled = False
        self.stance_enabled = False
        self.gas_enabled = True
        self.gimbal_locked = True
        self.lqr_leg_level = "low"
        self.lqr_gain = 0.0
        self.recovery_state = "fw_hold"
        self.recovery_elapsed = 0.0
        self.recovery_stable_elapsed = 0.0
        self.fw_recovery_axes = {}
        self.fw_recovery_angle_gain = 0.0
        self.fw_recovery_force_gain = {"l": 0.0, "r": 0.0}
        self.fw_recovery_breakaway_released = False
        self.recovery_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_support = {"l": 0.0, "r": 0.0}
        self.last_motor_torque = {"l": np.zeros(2), "r": np.zeros(2)}

    def begin_fall_recovery(self, data: mujoco.MjData) -> None:
        """REC_SIMPLE_ENTER port: seed the recovery from ANY fallen pose.

        The firmware recovery is pose-agnostic at entry -- HOLD classifies
        inverted / needs-forward-zero / already-upright and routes onward.
        This replaces the strict encoder-zero gate for live fall rescue.
        """
        self.reset(data)
        self.enabled = False
        self.stance_enabled = False
        self.gas_enabled = True
        self.gimbal_locked = True
        self.lqr_leg_level = "low"
        self.lqr_gain = 0.0
        self.recovery_state = "fw_hold"
        self.recovery_elapsed = 0.0
        self.recovery_stable_elapsed = 0.0
        self.fw_recovery_axes = {}
        self.fw_recovery_angle_gain = 0.0
        self.fw_recovery_force_gain = {"l": 0.0, "r": 0.0}
        self.fw_recovery_breakaway_released = False
        self.recovery_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_support = {"l": 0.0, "r": 0.0}
        self.last_motor_torque = {"l": np.zeros(2), "r": np.zeros(2)}
        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            self.recovery_last_l0[side] = kin.l0
            self.recovery_last_phi0[side] = kin.phi0
            self.fw_recovery_l0_ref[side] = kin.l0
            self.fw_recovery_length_rate[side] = 0.0
        # inverted-rescue scratch state
        self.fw_inverted_l0_ref = {
            side: self.fw_recovery_l0_ref[side] for side in ("l", "r")
        }
        self.fw_inverted_best_pitch = None
        self.fw_inverted_stall_timer = 0.0
        self.fw_inverted_wait_leg = 0
        self.fw_zero_targets = {"l": 0.0, "r": 0.0}
        self.fw_zero_stable_timer = 0.0

    def _fw_compute_zero_targets(self, kinematics) -> None:
        """Recovery_Simple_EnterForwardZero(): signed shortest path to zero."""
        wrapped = {
            side: self._wrap_pi(kinematics[side].phi0)
            for side in ("l", "r")
        }
        near_zero = {
            side: abs(wrapped[side]) < FW_RECOVERY_FORWARD_ZERO_PHI0_OK
            for side in ("l", "r")
        }
        side_pose = {
            side: (
                FW_RECOVERY_SIDE_POSE_MIN
                <= abs(wrapped[side])
                <= FW_RECOVERY_SIDE_POSE_MAX
            )
            for side in ("l", "r")
        }
        asymmetric = near_zero["l"] != near_zero["r"] and (
            (near_zero["l"] and side_pose["r"])
            or (near_zero["r"] and side_pose["l"])
        )
        symmetric = (
            side_pose["l"] and side_pose["r"]
            and wrapped["l"] * wrapped["r"] > 0.0
        )
        short_path = asymmetric or symmetric
        for side in ("l", "r"):
            phi0 = kinematics[side].phi0
            if near_zero[side]:
                self.fw_zero_targets[side] = phi0
            elif short_path:
                self.fw_zero_targets[side] = phi0 + self._wrap_pi(-phi0)
            else:
                delta = self._wrap_pi(-phi0)
                if delta < 0.0:
                    delta += 2.0 * np.pi
                self.fw_zero_targets[side] = phi0 + delta
            self.fw_recovery_l0_ref[side] = kinematics[side].l0

    @staticmethod
    def _capture_recovery_axis(actual: float) -> RecoverySwingAxis:
        return RecoverySwingAxis(
            actual=actual,
            start_actual=actual,
            last_raw=actual,
            ref=actual,
            target=FW_RECOVERY_PHI_FINAL,
        )

    def _update_recovery_axis(
        self,
        axis: RecoverySwingAxis,
        raw_angle: float,
        dt: float,
    ) -> None:
        delta = self._wrap_pi(raw_angle - axis.last_raw)
        axis.last_raw = raw_angle
        axis.actual += delta
        omega_raw = delta / dt
        alpha = dt / (FW_RECOVERY_OMEGA_FILTER_TAU + dt)
        axis.omega_actual += alpha * (omega_raw - axis.omega_actual)

    @staticmethod
    def _run_recovery_axis(
        axis: RecoverySwingAxis,
        speed_limit: float,
        accel_limit: float,
        dt: float,
    ) -> None:
        remaining = axis.direction * (axis.target - axis.ref)
        omega_target = 0.0
        if remaining > 0.0:
            speed = min(speed_limit, float(np.sqrt(2.0 * accel_limit * remaining)))
            omega_target = axis.direction * speed
        axis.omega_ref += float(np.clip(
            omega_target - axis.omega_ref,
            -accel_limit * dt,
            accel_limit * dt,
        ))
        axis.ref += axis.omega_ref * dt
        if axis.direction * (axis.ref - axis.target) >= 0.0:
            axis.ref = axis.target
            axis.omega_ref = 0.0

    @staticmethod
    def _recovery_breakaway_force(elapsed: float) -> float:
        if elapsed <= 0.0:
            return 0.0
        if elapsed < FW_RECOVERY_BREAKAWAY_RISE_TIME:
            progress = elapsed / FW_RECOVERY_BREAKAWAY_RISE_TIME
            progress = progress * progress * (3.0 - 2.0 * progress)
            return FW_RECOVERY_BREAKAWAY_FORCE * progress
        if elapsed < FW_RECOVERY_BREAKAWAY_TIME:
            progress = (
                (elapsed - FW_RECOVERY_BREAKAWAY_RISE_TIME) /
                (FW_RECOVERY_BREAKAWAY_TIME - FW_RECOVERY_BREAKAWAY_RISE_TIME)
            )
            progress = progress * progress * (3.0 - 2.0 * progress)
            return FW_RECOVERY_BREAKAWAY_FORCE + progress * (
                FW_RECOVERY_FORCE_LIMIT - FW_RECOVERY_BREAKAWAY_FORCE
            )
        return FW_RECOVERY_FORCE_LIMIT

    @staticmethod
    def _recovery_force_with_reserve(
        kin: VmcKinematics,
        force: float,
        tp: float,
        torque_reserve: float,
    ) -> float:
        tp_torque = kin.jacobian @ np.array((0.0, tp))
        force_torque = kin.jacobian @ np.array((force, 0.0))
        force_scale = 1.0
        for force_component, tp_component in zip(force_torque, tp_torque):
            if abs(tp_component) >= torque_reserve:
                return 0.0
            if abs(force_component) > 1e-4:
                limit_scale = (
                    (torque_reserve - tp_component) / force_component
                    if force_component > 0.0
                    else (-torque_reserve - tp_component) / force_component
                )
                force_scale = min(force_scale, float(limit_scale))
        return force * float(np.clip(force_scale, 0.0, 1.0))

    def _enter_fw_recovery_handoff(self, data: mujoco.MjData) -> None:
        self.fw_recovery_handoff_output = {
            side: self.recovery_output[side].copy() for side in ("l", "r")
        }
        captured_length = {
            side: self.vmc_kinematics(data, side).l0 for side in ("l", "r")
        }
        self.reset(data, preserve_recovery=True)
        self.enabled = True
        self.stance_enabled = True
        self.gas_enabled = True
        self.gimbal_locked = True
        self.lqr_leg_level = "low"
        self.lqr_gain = RECOVERY_LQR_INITIAL_GAIN
        self.recovery_state = "fw_handoff"
        self.recovery_elapsed = 0.0
        self.recovery_stable_elapsed = 0.0
        self.lqr_virtual_x = 0.0
        self.lqr_position_state = "drive"
        self.lqr_position_capture_timer = 0.0
        self.lqr_position_hold_x = 0.0
        self.lqr_position_brake_offset = 0.0
        self.theta_comp_apply_gain = 0.0
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.fw_recovery_length_hold_until = data.time + 3.0
        for side in ("l", "r"):
            self.length_reference[side] = captured_length[side]

    def _step_encoder_zero_recovery(self, data: mujoco.MjData, dt: float) -> None:
        """Port REC_SIMPLE HOLD -> SWING -> RETRACT for the upright zero pose."""
        self._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        self._apply_gimbal_lock(data)
        self.control_elapsed += dt
        update = self.control_elapsed + 1e-12 >= CONTROL_PERIOD_S
        if update:
            control_dt = self.control_elapsed
            self.control_elapsed = 0.0
            self.recovery_elapsed += control_dt
            kinematics = {
                side: self.vmc_kinematics(data, side) for side in ("l", "r")
            }
            length_rate: dict[str, float] = {}
            for side, kin in kinematics.items():
                raw_rate = (kin.l0 - self.recovery_last_l0[side]) / control_dt
                self.fw_recovery_length_rate[side] = (
                    0.5 * self.fw_recovery_length_rate[side] + 0.5 * raw_rate
                )
                length_rate[side] = self.fw_recovery_length_rate[side]

            if self.recovery_state == "fw_hold":
                if self.recovery_elapsed < FW_RECOVERY_HOLD_TIME_S:
                    pass
                else:
                    pitch_now = self.body_pitch(data)
                    long_legs = all(
                        kin.l0 > FW_RECOVERY_L0_LONG
                        for kin in kinematics.values()
                    )
                    phi_near_zero = all(
                        abs(self._wrap_pi(kin.phi0))
                        < FW_RECOVERY_PHI_NEAR_ZERO
                        for kin in kinematics.values()
                    )
                    if (
                        abs(pitch_now) > FW_RECOVERY_FORWARD_PITCH_MAX
                        and abs(pitch_now) < FW_RECOVERY_INVERTED_ENTER_PITCH
                    ):
                        # Firmware fault band between forward and inverted.
                        self.recovery_state = "fw_fault"
                        self.recovery_elapsed = 0.0
                    elif abs(pitch_now) >= FW_RECOVERY_INVERTED_ENTER_PITCH:
                        self.fw_inverted_l0_ref = {
                            side: kin.l0 for side, kin in kinematics.items()
                        }
                        self.fw_inverted_best_pitch = abs(
                            float(np.degrees(pitch_now))
                        )
                        self.fw_inverted_stall_timer = 0.0
                        self.recovery_state = "fw_inverted_capture"
                        self.recovery_elapsed = 0.0
                    elif not (long_legs and phi_near_zero):
                        self._fw_compute_zero_targets(kinematics)
                        self.fw_zero_stable_timer = 0.0
                        self.fw_recovery_axes = {
                            side: RecoverySwingAxis(
                                actual=kin.phi0,
                                start_actual=kin.phi0,
                                last_raw=kin.phi0,
                                ref=kin.phi0,
                                target=self.fw_zero_targets[side],
                            )
                            for side, kin in kinematics.items()
                        }
                        self.fw_recovery_angle_gain = 0.0
                        self.recovery_state = "fw_forward_zero"
                        self.recovery_elapsed = 0.0
                    else:
                        self.fw_recovery_axes = {
                            side: self._capture_recovery_axis(kin.phi0)
                            for side, kin in kinematics.items()
                        }
                        # REC_SIMPLE_CHECK_ZERO seeds the source controller at
                        # full angle gain before entering SWING.  Starting this
                        # ramp from zero makes the simulated leg lag its profile
                        # and stores an artificial late torque impulse.
                        self.fw_recovery_angle_gain = 1.0
                        self.recovery_state = "fw_swing"
                        self.recovery_elapsed = 0.0

            elif self.recovery_state == "fw_fault":
                for side in ("l", "r"):
                    self.recovery_output[side] = np.zeros(2)
                    self.last_support[side] = 0.0
                    self.last_output[side] = np.zeros(2)
                    self.set_vmc_motor_torques(
                        data, side, 0.0, 0.0, torque_limit=HIP_TORQUE_LIMIT
                    )
                if self.recovery_elapsed >= FW_RECOVERY_FAULT_HOLD_TIME:
                    # Deviation from the firmware (which latches the fault
                    # until the operator retries): re-enter HOLD so an
                    # unattended fallen chassis keeps trying to right itself.
                    self.recovery_state = "fw_hold"
                    self.recovery_elapsed = 0.0

            elif self.recovery_state == "fw_inverted_capture":
                for side in ("l", "r"):
                    kin = kinematics[side]
                    force = 0.0
                    tp = 0.0
                    self.recovery_output[side] = np.array((force, tp))
                    self.last_support[side] = force
                    self.last_output[side] = np.array((0.0, tp))
                    self.set_vmc_motor_torques(
                        data, side, force, tp, torque_limit=HIP_TORQUE_LIMIT
                    )
                if self.recovery_elapsed >= FW_RECOVERY_INVERTED_CAPTURE_TIME:
                    for side in ("l", "r"):
                        self.fw_recovery_l0_ref[side] = (
                            self.fw_inverted_l0_ref[side]
                        )
                    self.recovery_state = "fw_inverted_preload"
                    self.recovery_elapsed = 0.0

            elif self.recovery_state == "fw_inverted_preload":
                for side in ("l", "r"):
                    kin = kinematics[side]
                    force = FW_RECOVERY_INVERTED_CONTACT_FORCE
                    tp = 0.0
                    self.recovery_output[side] = np.array((force, tp))
                    self.last_support[side] = force
                    self.last_output[side] = np.array((0.0, tp))
                    self.set_vmc_motor_torques(
                        data, side, force, tp, torque_limit=HIP_TORQUE_LIMIT
                    )
                if self.recovery_elapsed >= FW_RECOVERY_INVERTED_PRELOAD_TIME:
                    # Calibrated direction: both legs turn toward negative
                    # phi0 (REC_SIMPLE_INVERTED_PRELOAD tail).
                    left = kinematics["l"].phi0
                    right_rel = self._wrap_pi(
                        kinematics["r"].phi0 - left
                    )
                    right_unwrapped = left + right_rel
                    negative_lead = min(right_unwrapped, left)
                    sym_target = (
                        negative_lead - FW_RECOVERY_INVERTED_TURN_MAX_TRAVEL
                    )
                    self.fw_recovery_axes = {
                        "l": RecoverySwingAxis(
                            actual=left,
                            start_actual=left,
                            last_raw=left,
                            ref=left,
                            target=sym_target,
                        ),
                        "r": RecoverySwingAxis(
                            actual=right_unwrapped,
                            start_actual=right_unwrapped,
                            last_raw=kinematics["r"].phi0,
                            ref=right_unwrapped,
                            target=sym_target,
                        ),
                    }
                    self.fw_recovery_angle_gain = 0.0
                    self.fw_inverted_best_pitch = abs(float(np.degrees(
                        self.body_pitch(data)
                    )))
                    self.fw_inverted_stall_timer = 0.0
                    self.fw_inverted_wait_leg = 0
                    self.recovery_state = "fw_inverted_turn"
                    self.recovery_elapsed = 0.0

            elif self.recovery_state == "fw_inverted_turn":
                for side in ("l", "r"):
                    self._update_recovery_axis(
                        self.fw_recovery_axes[side],
                        kinematics[side].phi0,
                        control_dt,
                    )
                ax_l, ax_r = self.fw_recovery_axes["l"], self.fw_recovery_axes["r"]
                progress_l = ax_l.direction * (ax_l.actual - ax_l.start_actual)
                progress_r = ax_r.direction * (ax_r.actual - ax_r.start_actual)
                sync_error = progress_l - progress_r
                sync_corr = float(np.clip(
                    FW_RECOVERY_INVERTED_SYNC_KP * abs(sync_error),
                    0.0,
                    FW_RECOVERY_INVERTED_SYNC_CORR_LIMIT,
                ))
                speed_l = FW_RECOVERY_INVERTED_TURN_RATE
                speed_r = FW_RECOVERY_INVERTED_TURN_RATE
                if sync_error > 0.0:
                    speed_l -= sync_corr
                else:
                    speed_r -= sync_corr
                if self.fw_inverted_wait_leg == 1:
                    if sync_error <= FW_RECOVERY_INVERTED_WAIT_EXIT:
                        self.fw_inverted_wait_leg = 0
                elif self.fw_inverted_wait_leg == -1:
                    if sync_error >= -FW_RECOVERY_INVERTED_WAIT_EXIT:
                        self.fw_inverted_wait_leg = 0
                if self.fw_inverted_wait_leg == 0:
                    if sync_error >= FW_RECOVERY_INVERTED_WAIT_ENTER:
                        self.fw_inverted_wait_leg = 1
                    elif sync_error <= -FW_RECOVERY_INVERTED_WAIT_ENTER:
                        self.fw_inverted_wait_leg = -1
                if self.fw_inverted_wait_leg == 1:
                    speed_l = 0.0
                elif self.fw_inverted_wait_leg == -1:
                    speed_r = 0.0
                self._run_recovery_axis(
                    ax_l, speed_l, FW_RECOVERY_INVERTED_TURN_ACCEL, control_dt
                )
                self._run_recovery_axis(
                    ax_r, speed_r, FW_RECOVERY_INVERTED_TURN_ACCEL, control_dt
                )
                self.fw_recovery_angle_gain = min(
                    1.0,
                    self.fw_recovery_angle_gain
                    + FW_RECOVERY_INVERTED_TURN_GAIN_RATE * control_dt,
                )
                abs_pitch = abs(float(np.degrees(self.body_pitch(data))))
                if self.fw_inverted_best_pitch is None:
                    self.fw_inverted_best_pitch = abs_pitch
                if abs_pitch < self.fw_inverted_best_pitch - np.degrees(
                    FW_RECOVERY_INVERTED_PITCH_PROGRESS_EPS
                ):
                    self.fw_inverted_best_pitch = abs_pitch
                    self.fw_inverted_stall_timer = 0.0
                elif self.recovery_elapsed > 25.0:
                    self.fw_inverted_stall_timer += control_dt
                stalled = (
                    self.fw_inverted_stall_timer
                    > FW_RECOVERY_INVERTED_PITCH_STALL_TIME
                )
                timed_out = (
                    self.recovery_elapsed > FW_RECOVERY_INVERTED_TURN_TIMEOUT
                )
                if abs(self.body_pitch(data)) <= FW_RECOVERY_INVERTED_EXIT_PITCH:
                    self.fw_inverted_wait_leg = 0
                    self.recovery_state = "fw_inverted_settle"
                    self.recovery_elapsed = 0.0
                elif stalled or timed_out or not np.isfinite(
                    data.qpos
                ).all():
                    self.recovery_state = "fw_fault"
                    self.recovery_elapsed = 0.0
                else:
                    for side in ("l", "r"):
                        axis = self.fw_recovery_axes[side]
                        force = FW_RECOVERY_INVERTED_CONTACT_FORCE
                        err = axis.ref - axis.actual
                        tp = self.fw_recovery_angle_gain * float(np.clip(
                            FW_RECOVERY_INVERTED_TP_KP * err
                            + FW_RECOVERY_INVERTED_TP_KD * (
                                axis.omega_ref - axis.omega_actual
                            ),
                            -FW_RECOVERY_INVERTED_TP_LIMIT,
                            FW_RECOVERY_INVERTED_TP_LIMIT,
                        ))
                        tp += (
                            axis.direction * FW_RECOVERY_INVERTED_TP_BIAS
                        )
                        self.recovery_output[side] = np.array((force, tp))
                        self.last_support[side] = force
                        self.last_output[side] = np.array((0.0, tp))
                        self.set_vmc_motor_torques(
                            data, side, force, tp,
                            torque_limit=HIP_TORQUE_LIMIT,
                        )

            elif self.recovery_state == "fw_inverted_settle":
                for side in ("l", "r"):
                    self._update_recovery_axis(
                        self.fw_recovery_axes[side],
                        kinematics[side].phi0,
                        control_dt,
                    )
                    self._run_recovery_axis(
                        self.fw_recovery_axes[side], 0.0,
                        FW_RECOVERY_INVERTED_TURN_ACCEL, control_dt,
                    )
                    axis = self.fw_recovery_axes[side]
                    force = FW_RECOVERY_INVERTED_CONTACT_FORCE
                    tp = float(np.clip(
                        FW_RECOVERY_INVERTED_SETTLE_TP_KP * (
                            axis.ref - axis.actual
                        )
                        + FW_RECOVERY_INVERTED_SETTLE_TP_KD * (
                            axis.omega_ref - axis.omega_actual
                        ),
                        -FW_RECOVERY_INVERTED_SETTLE_TP_LIMIT,
                        FW_RECOVERY_INVERTED_SETTLE_TP_LIMIT,
                    ))
                    self.recovery_output[side] = np.array((force, tp))
                    self.last_support[side] = force
                    self.last_output[side] = np.array((0.0, tp))
                    self.set_vmc_motor_torques(
                        data, side, force, tp, torque_limit=HIP_TORQUE_LIMIT
                    )
                if abs(self.body_pitch(data)) > FW_RECOVERY_FORWARD_PITCH_MAX:
                    self.recovery_state = "fw_fault"
                    self.recovery_elapsed = 0.0
                elif (
                    self.recovery_elapsed
                    >= FW_RECOVERY_INVERTED_SETTLE_TIME
                ):
                    self._fw_compute_zero_targets(kinematics)
                    self.fw_zero_stable_timer = 0.0
                    self.fw_recovery_axes = {
                        side: RecoverySwingAxis(
                            actual=kin.phi0,
                            start_actual=kin.phi0,
                            last_raw=kin.phi0,
                            ref=kin.phi0,
                            target=self.fw_zero_targets[side],
                        )
                        for side, kin in kinematics.items()
                    }
                    self.fw_recovery_angle_gain = 0.0
                    self.recovery_state = "fw_forward_zero"
                    self.recovery_elapsed = 0.0

            elif self.recovery_state == "fw_forward_zero":
                for side in ("l", "r"):
                    self._update_recovery_axis(
                        self.fw_recovery_axes[side],
                        kinematics[side].phi0,
                        control_dt,
                    )
                travels = {
                    side: abs(axis.target - axis.start_actual)
                    for side, axis in self.fw_recovery_axes.items()
                }
                max_travel = max(travels.values())
                for side, axis in self.fw_recovery_axes.items():
                    scale = (
                        travels[side] / max_travel if max_travel > 1e-4 else 0.0
                    )
                    self._run_recovery_axis(
                        axis,
                        FW_RECOVERY_FORWARD_ZERO_RATE * scale,
                        FW_RECOVERY_FORWARD_ZERO_ACCEL * scale,
                        control_dt,
                    )
                self.fw_recovery_angle_gain = min(
                    1.0,
                    self.fw_recovery_angle_gain
                    + FW_RECOVERY_FORWARD_ZERO_GAIN_RATE * control_dt,
                )
                if abs(self.body_pitch(data)) > FW_RECOVERY_FORWARD_PITCH_MAX:
                    # A failed crossing can tip the body again mid-zero; the
                    # firmware reuses the inverted capture/turn path here.
                    self.fw_inverted_l0_ref = {
                        side: kin.l0 for side, kin in kinematics.items()
                    }
                    self.recovery_state = "fw_inverted_capture"
                    self.recovery_elapsed = 0.0
                elif (
                    self.recovery_elapsed > FW_RECOVERY_FORWARD_ZERO_TIMEOUT
                ):
                    self.recovery_state = "fw_fault"
                    self.recovery_elapsed = 0.0
                else:
                    for side, kin in kinematics.items():
                        axis = self.fw_recovery_axes[side]
                        force = 0.0
                        err = axis.ref - axis.actual
                        tp = FW_RECOVERY_FORWARD_ZERO_TP_SIGN * (
                            self.fw_recovery_angle_gain * float(np.clip(
                                FW_RECOVERY_FORWARD_ZERO_TP_KP * err
                                + FW_RECOVERY_FORWARD_ZERO_TP_KD * (
                                    axis.omega_ref - axis.omega_actual
                                ),
                                -FW_RECOVERY_FORWARD_ZERO_TP_LIMIT,
                                FW_RECOVERY_FORWARD_ZERO_TP_LIMIT,
                            ))
                        )
                        self.recovery_output[side] = np.array((force, tp))
                        self.last_support[side] = force
                        self.last_output[side] = np.array((0.0, tp))
                        self.set_vmc_motor_torques(
                            data, side, force, tp,
                            torque_limit=HIP_TORQUE_LIMIT,
                        )
                    phi_ok = all(
                        abs(self._wrap_pi(kin.phi0))
                        < FW_RECOVERY_FORWARD_ZERO_PHI0_OK
                        for kin in kinematics.values()
                    )
                    long_legs = all(
                        kin.l0 > FW_RECOVERY_L0_LONG
                        for kin in kinematics.values()
                    )
                    if (
                        phi_ok
                        and long_legs
                        and abs(self.body_pitch(data))
                            < FW_RECOVERY_ZERO_PITCH_MAX
                    ):
                        self.fw_zero_stable_timer += control_dt
                    else:
                        self.fw_zero_stable_timer = 0.0
                    if (
                        self.fw_zero_stable_timer
                        >= FW_RECOVERY_FORWARD_ZERO_STABLE_TIME
                    ):
                        self.fw_recovery_axes = {
                            side: self._capture_recovery_axis(kin.phi0)
                            for side, kin in kinematics.items()
                        }
                        self.fw_recovery_angle_gain = 1.0
                        self.recovery_state = "fw_swing"
                        self.recovery_elapsed = 0.0

            elif self.recovery_state in {"fw_swing", "fw_retract"}:
                retract = self.recovery_state == "fw_retract"
                for side, axis in self.fw_recovery_axes.items():
                    self._update_recovery_axis(
                        axis, kinematics[side].phi0, control_dt
                    )

                speed = FW_RECOVERY_SWING_RATE
                acceleration = FW_RECOVERY_SWING_ACCEL
                if retract and any(
                    kin.l0 > FW_RECOVERY_BREAKAWAY_RELEASE_L0
                    for kin in kinematics.values()
                ):
                    speed = FW_RECOVERY_DEADPOINT_SWING_RATE
                elif retract and all(
                    kin.l0 < FW_RECOVERY_COMPACT_L0
                    for kin in kinematics.values()
                ):
                    speed = FW_RECOVERY_COMPACT_SWING_RATE
                    acceleration = FW_RECOVERY_COMPACT_SWING_ACCEL
                for axis in self.fw_recovery_axes.values():
                    self._run_recovery_axis(axis, speed, acceleration, control_dt)

                gain_rate = (
                    FW_RECOVERY_TP_GAIN_RATE if retract
                    else FW_RECOVERY_SWING_GAIN_RATE
                )
                self.fw_recovery_angle_gain = min(
                    1.0,
                    self.fw_recovery_angle_gain + gain_rate * control_dt,
                )

                stable = retract
                breakaway_scales: dict[str, float] = {}
                if retract:
                    # Match Recovery_Simple_RetractControl ordering: ramp the
                    # axial loop and l0 references first, then release both
                    # sides to full gain as soon as the 180 ms breakaway
                    # envelope ends.  The previous port initialized the
                    # release flag but never executed this source transition.
                    for side, kin in kinematics.items():
                        self.fw_recovery_force_gain[side] = min(
                            1.0,
                            self.fw_recovery_force_gain[side] +
                            FW_RECOVERY_FORCE_GAIN_RATE * control_dt,
                        )
                        self.fw_recovery_l0_ref[side] = self._move_towards(
                            self.fw_recovery_l0_ref[side],
                            FW_RECOVERY_L0_TARGET,
                            FW_RECOVERY_L0_RATE * control_dt,
                        )
                        # Firmware retraction is one-way; never extend a leg
                        # which compressed past the ramped reference.
                        self.fw_recovery_l0_ref[side] = min(
                            self.fw_recovery_l0_ref[side], kin.l0
                        )
                        breakaway_scale = float(np.clip(
                            (kin.l0 - (
                                FW_RECOVERY_BREAKAWAY_RELEASE_L0 -
                                FW_RECOVERY_BREAKAWAY_RELEASE_BAND
                            )) / FW_RECOVERY_BREAKAWAY_RELEASE_BAND,
                            0.0,
                            1.0,
                        ))
                        breakaway_scale = (
                            breakaway_scale * breakaway_scale *
                            (3.0 - 2.0 * breakaway_scale)
                        )
                        if self.recovery_elapsed >= FW_RECOVERY_BREAKAWAY_TIME:
                            breakaway_scale = 0.0
                        elif self.recovery_elapsed < FW_RECOVERY_BREAKAWAY_MIN_TIME:
                            breakaway_scale = 1.0
                        breakaway_scales[side] = breakaway_scale

                    if (
                        not self.fw_recovery_breakaway_released
                        and all(scale <= 0.001 for scale in breakaway_scales.values())
                    ):
                        for side, kin in kinematics.items():
                            self.fw_recovery_l0_ref[side] = min(
                                self.fw_recovery_l0_ref[side], kin.l0
                            )
                            self.fw_recovery_force_gain[side] = 1.0
                        self.fw_recovery_breakaway_released = True

                for side, kin in kinematics.items():
                    axis = self.fw_recovery_axes[side]
                    force = 0.0
                    motor_limit = HIP_TORQUE_LIMIT
                    terminal_blend = 0.0
                    if retract:
                        spring_ff = float(np.clip(
                            self._gas_virtual_support(kin.l0),
                            0.0,
                            FW_RECOVERY_SPRING_FF_LIMIT,
                        ))
                        bias_scale = float(np.clip(
                            (kin.l0 - FW_RECOVERY_L0_TARGET) /
                            FW_RECOVERY_FORCE_BIAS_RANGE,
                            0.0,
                            1.0,
                        ))
                        # Exact firmware order: the gain multiplies the full
                        # length PD, gas-spring cancellation, and retract bias.
                        force = self.fw_recovery_force_gain[side] * (
                            FW_RECOVERY_L0_KP *
                            (self.fw_recovery_l0_ref[side] - kin.l0)
                            - FW_RECOVERY_L0_KD * length_rate[side]
                            - spring_ff
                            - FW_RECOVERY_FORCE_BIAS * bias_scale
                        )
                        terminal_blend = float(np.clip(
                            (FW_RECOVERY_SETTLE_L0_START - kin.l0) /
                            (FW_RECOVERY_SETTLE_L0_START -
                             FW_RECOVERY_SETTLE_L0_END),
                            0.0,
                            1.0,
                        ))

                    tp_kp = FW_RECOVERY_SWING_KP + terminal_blend * (
                        FW_RECOVERY_SETTLE_KP - FW_RECOVERY_SWING_KP
                    )
                    tp_kd = FW_RECOVERY_SWING_KD + terminal_blend * (
                        FW_RECOVERY_SETTLE_KD - FW_RECOVERY_SWING_KD
                    )
                    tp_limit = FW_RECOVERY_SWING_TP_LIMIT + terminal_blend * (
                        FW_RECOVERY_SETTLE_TP_LIMIT -
                        FW_RECOVERY_SWING_TP_LIMIT
                    )
                    tp = self.fw_recovery_angle_gain * (
                        tp_kp * (axis.ref - axis.actual)
                        + tp_kd * (axis.omega_ref - axis.omega_actual)
                    )
                    tp += (
                        self.fw_recovery_angle_gain
                        * float(np.clip(
                            axis.omega_ref / FW_RECOVERY_SWING_RATE,
                            -1.0,
                            1.0,
                        ))
                        * FW_RECOVERY_SWING_FF
                    )
                    tp = float(np.clip(tp, -tp_limit, tp_limit))
                    slew_rate = FW_RECOVERY_TP_SLEW_RATE + terminal_blend * (
                        FW_RECOVERY_TERMINAL_TP_SLEW_RATE -
                        FW_RECOVERY_TP_SLEW_RATE
                    )
                    axis.tp_cmd = self._move_towards(
                        axis.tp_cmd, tp, slew_rate * control_dt
                    )
                    tp = float(np.clip(axis.tp_cmd, -tp_limit, tp_limit))

                    if retract:
                        breakaway_scale = breakaway_scales[side]
                        breakaway_active = breakaway_scale > 0.001
                        breakaway_force = (
                            FW_RECOVERY_FORCE_LIMIT + breakaway_scale * (
                                self._recovery_breakaway_force(self.recovery_elapsed)
                                - FW_RECOVERY_FORCE_LIMIT
                            )
                        )
                        if breakaway_active and force > -breakaway_force:
                            force = -breakaway_force
                        force_limit = (
                            FW_RECOVERY_FORCE_LIMIT + breakaway_scale * (
                                FW_RECOVERY_BREAKAWAY_FORCE -
                                FW_RECOVERY_FORCE_LIMIT
                            )
                        )
                        force = float(np.clip(
                            force,
                            -force_limit,
                            FW_RECOVERY_FORCE_EXTEND_LIMIT,
                        ))
                        motor_limit = (
                            FW_RECOVERY_TORQUE_RESERVE + breakaway_scale * (
                                FW_RECOVERY_BREAKAWAY_TORQUE -
                                FW_RECOVERY_TORQUE_RESERVE
                            )
                        )
                        force = self._recovery_force_with_reserve(
                            kin, force, tp, motor_limit
                        )

                    self.recovery_output[side] = np.array((force, tp))
                    self.last_support[side] = force
                    self.last_output[side] = np.array((0.0, tp))
                    stable &= (
                        kin.l0 < FW_RECOVERY_L0_OK
                        and abs(length_rate[side]) < FW_RECOVERY_STABLE_L0_RATE
                        and abs(axis.omega_actual) < FW_RECOVERY_STABLE_PHI_RATE
                        and abs(axis.ref - axis.actual) <
                            FW_RECOVERY_STABLE_PHI_ERROR
                        and abs(force) < FW_RECOVERY_STABLE_FORCE
                        and abs(tp) < FW_RECOVERY_STABLE_TP
                    )
                    # Store the source's active motor reserve alongside the
                    # virtual command for the held 1 ms physics samples.
                    self.set_vmc_motor_torques(
                        data, side, force, tp, torque_limit=motor_limit
                    )

                if not retract:
                    prepose_ok = all(
                        axis.direction *
                        (FW_RECOVERY_PHI_PREPOSE - axis.actual)
                        <= FW_RECOVERY_PHI_PREPOSE_TOLERANCE
                        for axis in self.fw_recovery_axes.values()
                    )
                    if prepose_ok:
                        self.recovery_state = "fw_retract"
                        self.recovery_elapsed = 0.0
                        self.recovery_stable_elapsed = 0.0
                        self.fw_recovery_angle_gain = max(
                            self.fw_recovery_angle_gain, 0.30
                        )
                        self.fw_recovery_force_gain = {"l": 0.0, "r": 0.0}
                else:
                    if stable:
                        self.recovery_stable_elapsed += control_dt
                    else:
                        self.recovery_stable_elapsed = 0.0
                    if (
                        self.recovery_stable_elapsed >=
                            FW_RECOVERY_STABLE_TIME_S
                        or self.recovery_elapsed >=
                            FW_RECOVERY_RETRACT_TIMEOUT_S
                    ):
                        self._enter_fw_recovery_handoff(data)

                if (
                    self.recovery_state in {"fw_swing", "fw_retract"}
                    and (
                        abs(self.body_pitch(data)) > np.radians(25.0)
                        or any(kin.l0 < 0.095 for kin in kinematics.values())
                    )
                ):
                    self.recovery_state = "fw_fault"
                    self.enabled = False
                    self.stance_enabled = False
                    self.recovery_output = {
                        "l": np.zeros(2), "r": np.zeros(2)
                    }

            for side, kin in kinematics.items():
                self.recovery_last_l0[side] = kin.l0
                self.recovery_last_phi0[side] = kin.phi0

        if self.recovery_state == "fw_fault":
            data.ctrl[:] = 0.0
            data.qfrc_applied[:] = 0.0
            return

        # Hold the latest 5 ms VMC output over every 1 ms physics step.
        if self.recovery_state in {"fw_swing", "fw_retract"}:
            for side, ids in self.sides.items():
                force, tp = self.recovery_output[side]
                reserve = (
                    FW_RECOVERY_BREAKAWAY_TORQUE
                    if self.recovery_state == "fw_retract" and
                    self.recovery_elapsed < FW_RECOVERY_BREAKAWAY_TIME
                    else HIP_TORQUE_LIMIT
                )
                self.set_vmc_motor_torques(
                    data, side, float(force), float(tp), torque_limit=reserve
                )
                data.ctrl[ids.wheel_motor] = 0.0

    def cancel_recovery(self) -> None:
        self.recovery_state = "idle"
        self.recovery_elapsed = 0.0
        self.recovery_stable_elapsed = 0.0

    def _apply_gimbal_lock(self, data: mujoco.MjData) -> None:
        if not self.gimbal_locked:
            return
        yaw_torque = -GIMBAL_YAW_KP * data.qpos[self.gimbal_qpos] - GIMBAL_YAW_KD * data.qvel[self.gimbal_dof]
        data.ctrl[self.gimbal_motor] = float(np.clip(yaw_torque, -GIMBAL_YAW_TORQUE_LIMIT, GIMBAL_YAW_TORQUE_LIMIT))

    def _step_cad_stand_recovery(self, data: mujoco.MjData, dt: float) -> None:
        """Torque-only CAD-zero -> gas-trim recovery and local wheel LQR."""
        self._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        self._apply_gimbal_lock(data)

        self.control_elapsed += dt
        update = self.control_elapsed + 1e-12 >= CONTROL_PERIOD_S
        if update:
            control_dt = self.control_elapsed
            self.control_elapsed = 0.0
            self.recovery_elapsed += control_dt
            alpha = min(self.recovery_elapsed / CAD_STAND_RAMP_S, 1.0)
            if (
                not self.enabled
                and self.recovery_elapsed >= RECOVERY_CAD_STAND_LQR_DELAY_S
            ):
                self.cad_stand_wheel_origin = data.xpos[self.base].copy()
                self.enabled = True
                self.lqr_gain = 0.0
            if self.enabled:
                self.lqr_gain = min(self.recovery_elapsed / LQR_RAMP_TIME_S, 1.0)
            if alpha >= 1.0 and self.lqr_gain >= 1.0:
                self.recovery_state = "stand"

            for side in ("l", "r"):
                zero = CAD_ZERO_GAS_TRIM[side]
                trim = CAD_GAS_TRIM[side]
                length_ref = (1.0 - alpha) * zero["l0"] + alpha * trim["l0"]
                phi_ref = (1.0 - alpha) * zero["phi0"] + alpha * trim["phi0"]
                force_ff = (1.0 - alpha) * zero["force_ff"] + alpha * trim["force_ff"]
                tp_ff = (1.0 - alpha) * zero["tp_ff"] + alpha * trim["tp_ff"]
                kin = self.vmc_kinematics(data, side)
                previous_l0, previous_phi0 = self.cad_stand_last[side]
                length_rate = (kin.l0 - previous_l0) / control_dt
                phi_rate = self._wrap_pi(kin.phi0 - previous_phi0) / control_dt
                force = force_ff + CAD_STAND_LENGTH_KP * (kin.l0 - length_ref) + CAD_STAND_LENGTH_KD * length_rate
                tp = tp_ff + CAD_STAND_PHI_KP * self._wrap_pi(kin.phi0 - phi_ref) + CAD_STAND_PHI_KD * phi_rate
                self.cad_stand_output[side] = np.array((
                    float(np.clip(force, -80.0, 80.0)),
                    float(np.clip(tp, -12.0, 12.0)),
                ))
                self.recovery_output[side] = self.cad_stand_output[side]
                self.last_support[side] = self.cad_stand_output[side][0]
                self.cad_stand_last[side] = (kin.l0, kin.phi0)

            if self.cad_stand_wheel_origin is not None:
                _, forward, lateral = self._frame(data)
                state = np.array((
                    self.body_pitch(data) - CAD_STAND_PITCH_REFERENCE,
                    -float(np.dot(data.qvel[3:6], lateral)),
                    float(np.dot(data.xpos[self.base] - self.cad_stand_wheel_origin, forward)),
                    float(np.dot(data.qvel[:3], forward)),
                ))
                wheel_torque = float(np.clip(
                    -self.lqr_gain * (CAD_STAND_WHEEL_K @ state),
                    -WHEEL_TORQUE_LIMIT,
                    WHEEL_TORQUE_LIMIT,
                ))
            else:
                wheel_torque = 0.0
            for side in ("l", "r"):
                self.last_output[side] = np.array((wheel_torque, self.cad_stand_output[side][1]))

        # Hold the latest command over all 1 ms physics steps.  This is
        # crucial: applying a 5 ms control sample for only one physics step
        # would silently cut the effective wheel torque to one fifth.
        for side, ids in self.sides.items():
            force, tp = self.cad_stand_output[side]
            self.set_vmc_motor_torques(data, side, float(force), float(tp))
            data.ctrl[ids.wheel_motor] = self.cad_stand_wheel_sign[side] * self.last_output[side][0]

    def _step_prone_swing(self, data: mujoco.MjData, dt: float) -> None:
        """Physical face-down q=0 -> contact-gated upright handoff.

        A bounded bilateral VMC pulse folds/sweeps the leg linkage and uses
        the wheels as the ground reaction.  It was tested independently in
        both front/back prone orientations.  This method intentionally never
        changes qpos after the initial pose: all motion is torque and contact.
        """
        self._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        self._apply_gimbal_lock(data)
        self.prone_swing_elapsed += dt
        self.recovery_elapsed = self.prone_swing_elapsed
        for side, ids in self.sides.items():
            self.recovery_output[side] = np.array((PRONE_SWING_FORCE, PRONE_SWING_TP))
            self.cad_stand_output[side] = self.recovery_output[side]
            self.last_support[side] = PRONE_SWING_FORCE
            self.last_output[side] = np.array((PRONE_SWING_WHEEL_TORQUE, PRONE_SWING_TP))
            self.set_vmc_motor_torques(data, side, PRONE_SWING_FORCE, PRONE_SWING_TP)
            data.ctrl[ids.wheel_motor] = self.wheel_sign[side] * PRONE_SWING_WHEEL_TORQUE

        pitch_ok = abs(self.body_pitch(data)) <= PRONE_HANDOFF_PITCH
        contact_ok = data.ncon >= PRONE_HANDOFF_MIN_CONTACTS
        if self.prone_swing_elapsed >= PRONE_HANDOFF_MIN_TIME_S and pitch_ok and contact_ok:
            # Rebase position/derivative memories but retain the world-forward
            # calibration made at CAD zero.  begin_recovery then selects the
            # ordinary calibrated VMC trim and wheel-LQR route.
            self.reset(data)
            self.begin_recovery(data)
            self._step_cad_stand_recovery(data, dt)
        elif self.prone_swing_elapsed >= PRONE_SWING_TIMEOUT_S:
            # A missing contact is a setup problem, not a reason to continue
            # winding the joints.  Leave a visible safe fault state instead.
            self.recovery_state = "prone_fault"
            self.enabled = False
            self.stance_enabled = False
            self.gas_enabled = False
            data.ctrl[:] = 0.0
            data.qfrc_applied[:] = 0.0

    def _step_torque_recovery(self, data: mujoco.MjData, dt: float) -> None:
        """REC_SIMPLE_RETRACT equivalent with real VMC motor torques."""
        self._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        self._apply_gimbal_lock(data)
        self.control_elapsed += dt
        update = self.control_elapsed + 1e-12 >= CONTROL_PERIOD_S
        if update:
            control_dt = self.control_elapsed
            self.control_elapsed = 0.0
            self.recovery_elapsed += control_dt
            for side in ("l", "r"):
                kin = self.vmc_kinematics(data, side)
                d_l0 = (kin.l0 - self.recovery_last_l0[side]) / control_dt
                d_phi0 = self._wrap_pi(kin.phi0 - self.recovery_last_phi0[side]) / control_dt
                target = self.recovery_target[side]
                force = float(np.clip(
                    RECOVERY_LENGTH_KP * (target.l0 - kin.l0) - RECOVERY_LENGTH_KD * d_l0,
                    RECOVERY_FORCE_MIN,
                    RECOVERY_FORCE_MAX,
                ))
                tp = float(np.clip(
                    RECOVERY_TP_KP * self._wrap_pi(target.phi0 - kin.phi0) - RECOVERY_TP_KD * d_phi0,
                    -RECOVERY_TP_LIMIT,
                    RECOVERY_TP_LIMIT,
                ))
                self.recovery_output[side] = np.array((force, tp))
                self.recovery_last_l0[side] = kin.l0
                self.recovery_last_phi0[side] = kin.phi0

        stable = True
        for side in ("l", "r"):
            force, tp = self.recovery_output[side]
            self.set_vmc_motor_torques(data, side, float(force), float(tp))
            kin = self.vmc_kinematics(data, side)
            target = self.recovery_target[side]
            stable &= (
                abs(kin.l0 - target.l0) < RECOVERY_LENGTH_TOLERANCE and
                abs(self._wrap_pi(kin.phi0 - target.phi0)) < RECOVERY_PHI_TOLERANCE
            )
        if update and stable and self.recovery_elapsed > 0.20:
            self.recovery_stable_elapsed += control_dt
            if self.recovery_stable_elapsed >= RECOVERY_STABLE_S:
                # Equivalent to the firmware's handoff PID/LQR clear: never
                # differentiate the compact starting pose against the just-
                # reached standing pose on the first LQR sample.
                self.reset(data, preserve_recovery=True)
                self.recovery_state = "handoff"
                self.enabled = True
                self.lqr_gain = RECOVERY_LQR_INITIAL_GAIN
        elif update:
            self.recovery_stable_elapsed = 0.0
