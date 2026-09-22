"""Jump state machine: preload -> thrust -> flight -> touchdown capture -> landing buffer.

Gas-spring extension safety (investigated and CLOSED 2026-09-22): a
spring-only launch rule (thrust F0 <= 0) was implemented and measured --
it cannot take off at all (0 m; force/energy accounting in
docs/JUMP_CONTROL_NOTES.md).  The rule was then retracted because its
premise did not hold: the mechanism's stops keep the gas spring's
installed length inside [0.154, ~0.205] m at all times (compression-side
bounded, never over-extended past its stroke), so in-stroke extension --
including motor-assisted extension -- is normal working operation, not a
damage mode.  The original thrust law (weight share + PID toward
JUMP_THRUST_LENGTH + boost) is restored; the retained record of the
experiment lives in the notes.
Support-mass scope: the jump/landing support laws keep the firmware
``CHASSIS_SUPPORT_MASS`` (``TOTAL_MASS = 12.801 kg``) they were ported and
validated against -- the preload ready window sits a few millimetres from
the equilibrium, and re-anchoring the feedforward to the 2026-09-21 ruling
mass (13.592626 kg) moves it past ``JUMP_PRELOAD_READY_LENGTH``
(measured: the jump then times out in preload).  The ruling mass applies to
the VMC support path (controller._apply_vmc_support / lateral load
transfer), i.e. the 66.6718 N/leg standing feedforward; on the real robot
the firmware's stale 12.801 kg constant carries the same ~7.5 N deficit in
every channel, which the length integrator absorbs either way.

Mixin part of :class:`balance_lab.control.controller.TraditionalLQR`; every
method accesses the shared controller state through ``self``.
"""

from __future__ import annotations

import mujoco  # noqa: F401
import numpy as np

from .tuning import *  # noqa: F401,F403
from .kinematics import unit  # noqa: F401


class JumpControlMixin:
    "Jump state machine: preload -> thrust -> flight -> touchdown capture -> landing buffer."

    def _reset_jump_runtime(self, clear_cooldown: bool = False) -> None:
        self.jump_state = "idle"
        self.jump_fault = "none"
        self.jump_elapsed = 0.0
        if clear_cooldown:
            self.jump_last_accept_time = None
        self.jump_off_ground = False
        self.jump_takeoff_count = 0
        self.jump_landing_count = 0
        self.jump_takeoff_seen = False
        self.jump_landing_seen = False
        self.jump_length_target = self.normal_leg_length
        self.jump_air_profile_active = False
        self.jump_air_elapsed = 0.0
        self.jump_air_reference = {
            "l": self.jump_air_landing_length,
            "r": self.jump_air_landing_length,
        }
        self.jump_launch_vx = 0.0
        self.jump_takeoff_position = None
        self.jump_stationary_position_hold = False
        self.jump_thrust_theta_ref = 0.0
        self.jump_thrust_pitch_rate_lpf = 0.0
        self.jump_thrust_pitch_tp = 0.0
        self.jump_air_common_tp_lpf = 0.0
        self.jump_air_pitch_rate_lpf = 0.0
        self.jump_air_theta_rate_lpf = 0.0
        self.jump_air_wheel_torque = 0.0
        self.jump_raw_wheel_contact = {"l": False, "r": False}
        self.jump_landing_ground_elapsed = 0.0
        self.jump_landing_tp_handoff = {"l": 0.0, "r": 0.0}
        self.jump_landing_tp_handoff_elapsed = 0.0
        self.jump_landing_tp_command = {"l": 0.0, "r": 0.0}
        self.jump_landing_wheel_command = {"l": 0.0, "r": 0.0}
        self.offground_fn_raw = {"l": 0.0, "r": 0.0}
        self.offground_fn_filtered = {"l": 0.0, "r": 0.0}
        self.offground_fn_history = {"l": [0.0] * 4, "r": [0.0] * 4}
        self.offground_filter_index = 0
        self.offground_deriv_init = False
        self.offground_last_d_l0 = {"l": 0.0, "r": 0.0}
        self.offground_last_d_theta = {"l": 0.0, "r": 0.0}
        self.off_ground_cnt = 0
        self.off_ground = False
        self.off_ground_last = False
        self.airborne_wheel_shut = False
        self.airborne_theta_hold_tp = {"l": 0.0, "r": 0.0}
        self.airborne_soft_force = {"l": 0.0, "r": 0.0}
        self.airborne_lqr_tp = {"l": 0.0, "r": 0.0}
        self.airborne_length_sync_force = 0.0
        self.descent_guard_active = False
        self.descent_mode_active = False
        self.descent_guard_drop_seen = False
        self.descent_guard_preedge_elapsed = 0.0
        self.offground_wheel_torque = 0.0
        self.landing_air_deploy_ref = {
            "l": LANDING_AIR_DEPLOY_LENGTH,
            "r": LANDING_AIR_DEPLOY_LENGTH,
        }
        self.landing_airborne_elapsed = 0.0
        self.landing_buffer_phase = "inactive"
        self.landing_buffer_phase_elapsed = 0.0
        self.landing_buffer_elapsed = 0.0
        self.landing_exit_stable_elapsed = 0.0
        self.landing_length_reference = {
            "l": self.normal_leg_length,
            "r": self.normal_leg_length,
        }
        self.landing_support_handoff = {"l": 0.0, "r": 0.0}
        self.gbuf_phase = "inactive"
        self.gbuf_elapsed = 0.0
        self.gbuf_exit_stable = 0.0
        self.gbuf_length_ref = {
            "l": self.normal_leg_length,
            "r": self.normal_leg_length,
        }
        self.gbuf_absorb_ref = {
            "l": self.normal_leg_length,
            "r": self.normal_leg_length,
        }
        self.gbuf_contact_length = {
            "l": self.normal_leg_length,
            "r": self.normal_leg_length,
        }
        self.gbuf_contact_loss_count = 0

    def begin_jump(self, data: mujoco.MjData) -> tuple[bool, str]:
        """Request the firmware jump state machine from a standing LQR state."""
        contacts = self._wheel_contacts(data)
        average_length = 0.5 * sum(
            self.vmc_kinematics(data, side).l0 for side in ("l", "r")
        )
        reasons: list[str] = []
        if self.jump_state != "idle":
            reasons.append(f"already {self.jump_state}")
        if not (self.enabled and self.stance_enabled):
            reasons.append("LQR/VMC is not armed")
        if self.recovery_state not in {"idle", "stand", "fw_stand"}:
            reasons.append(f"recovery={self.recovery_state}")
        if self.jump_last_accept_time is not None and (
            data.time - self.jump_last_accept_time < JUMP_COOLDOWN_S
        ):
            reasons.append("cooldown")
        if not all(contacts.values()):
            reasons.append("both wheels are not grounded")
        if abs(self.body_pitch(data)) > JUMP_PRECHECK_PITCH_MAX:
            reasons.append("pitch")
        if abs(self.body_roll(data)) > JUMP_PRECHECK_ROLL_MAX:
            reasons.append("roll")
        # Near-stationary interlock.  With no operator speed command the
        # chassis must genuinely be parked.  A nonzero command makes the
        # launch intentional instead of accidental, so bind only the plant's
        # known steady-state tracking error on top of the requested speed
        # (the plain calibrated LQR settles a 0.20 m/s request near
        # ~0.32 m/s), capped so a hard-driven entry cannot become a launch.
        launch_vx_max = JUMP_SAFE_LAUNCH_VX_MAX
        commanded_speed = abs(self._drive_target_for_control())
        if commanded_speed > 1e-6:
            launch_vx_max = max(
                launch_vx_max,
                min(commanded_speed + JUMP_SAFE_LAUNCH_COMMAND_MARGIN,
                    JUMP_SAFE_LAUNCH_MOVING_MAX),
            )
        if abs(self.chassis_vx) > launch_vx_max:
            reasons.append("vx")
        if abs(self.yaw_rate) > JUMP_PRECHECK_YAW_RATE_MAX:
            reasons.append("yaw rate")
        if not (VMC_LENGTH_MIN <= average_length <= VMC_LENGTH_MAX):
            reasons.append(f"l0={average_length:.3f}")
        if reasons:
            self.jump_fault = "precheck"
            return False, ", ".join(reasons)

        self.jump_last_accept_time = float(data.time)
        self.jump_fault = "none"
        self.jump_state = "preload"
        self.jump_elapsed = 0.0
        self.jump_takeoff_seen = False
        self.jump_landing_seen = False
        self.jump_off_ground = False
        self.jump_takeoff_count = 0
        self.jump_landing_count = 0
        self.jump_length_target = JUMP_PRELOAD_LENGTH
        self.jump_air_profile_active = False
        self.jump_air_elapsed = 0.0
        self.jump_launch_vx = self.chassis_vx
        self.jump_takeoff_position = data.xpos[self.base].copy()
        # A moving-drive command is not safe to carry through the short
        # preload/touchdown handoff.  It can keep the speed loop active while
        # the wheels are unloading, then reapply the same command as soon as
        # contact is confirmed and produce the observed runaway reverse.
        # Require the operator to press an arrow again after the jump settles.
        self.set_drive_command(0.0, 0.0)
        # A jump's touchdown point is not a reliable position reference: the
        # vehicle can acquire a small ballistic translation before the wheels
        # recapture the ground.  Let the normal zero-speed observer brake the
        # touchdown velocity locally instead of commanding a long reverse
        # correction back to the takeoff point.
        self.jump_stationary_position_hold = False
        self.jump_thrust_pitch_rate_lpf = -float(self.last_state["l"][5])
        self.jump_air_common_tp_lpf = 0.0
        self.jump_air_pitch_rate_lpf = -float(self.last_state["l"][5])
        self.jump_air_theta_rate_lpf = 0.5 * sum(
            self.theta_rate_lpf.values()
        )
        self.jump_air_wheel_torque = 0.0
        self.jump_landing_ground_elapsed = 0.0
        self.landing_buffer_phase = "inactive"
        self.lqr_virtual_x = 0.0
        self.lqr_position_state = "drive"
        self.lqr_position_capture_timer = 0.0
        self.lqr_position_hold_x = 0.0
        self.lqr_position_brake_offset = 0.0
        self.length_integral = {"l": 0.0, "r": 0.0}
        self.jump_raw_wheel_contact = contacts
        for side in ("l", "r"):
            self.length_reference[side] = JUMP_PRELOAD_LENGTH
        return True, "preload"

    def _enter_jump_state(self, state: str, data: mujoco.MjData | None = None) -> None:
        self.jump_state = state
        self.jump_elapsed = 0.0
        self.jump_landing_ground_elapsed = 0.0
        self.jump_thrust_pitch_tp = 0.0
        self.length_integral = {"l": 0.0, "r": 0.0}
        if state == "thrust" and data is not None:
            pitch = self.body_pitch(data)
            self.jump_thrust_theta_ref = 0.5 * sum(
                np.pi / 2.0 - self.vmc_kinematics(data, side).phi0 + pitch
                for side in ("l", "r")
            )

    def _finish_jump(self, fault: str = "none") -> None:
        self.jump_fault = fault
        self.jump_state = "idle"
        self.jump_elapsed = 0.0
        self.jump_length_target = self.normal_leg_length
        # A fault can be raised while the chassis is physically airborne.
        # Keep the already-running tuck/deploy reference alive until contact;
        # otherwise one software-state edge would strand the legs at their
        # last reference even though the plant is still in flight.
        if not self.jump_off_ground:
            self.jump_air_profile_active = False
            self.jump_air_elapsed = 0.0
        self.jump_thrust_pitch_tp = 0.0
        self.jump_landing_ground_elapsed = 0.0
        # The landing buffer keeps its own captured reference while it
        # absorbs the touchdown.  Do not hand that temporary (often longer)
        # reference to the normal LQR when the jump state changes to idle:
        # doing so can extend both legs on the very next control tick and
        # create the apparent second jump / runaway reverse braking.
        for side in ("l", "r"):
            self.length_reference[side] = self.normal_leg_length

    def _start_jump_air_profile(self, data: mujoco.MjData) -> None:
        if self.jump_air_profile_active:
            return
        captured = 0.5 * sum(
            self.vmc_kinematics(data, side).l0 for side in ("l", "r")
        )
        captured = float(np.clip(
            captured,
            self.jump_air_landing_length,
            self.jump_air_capture_length_max,
        ))
        self.jump_air_reference = {"l": captured, "r": captured}
        self.jump_air_elapsed = 0.0
        self.jump_air_profile_active = True

    def _update_jump_air_profile(self, dt: float) -> None:
        if not self.jump_air_profile_active:
            return
        self.jump_air_elapsed += dt
        compact = self.jump_air_elapsed < (
            self.jump_air_tuck_time + self.jump_air_tuck_hold_time
        )
        target = (
            self.jump_air_tuck_length
            if compact
            else self.jump_air_landing_length
        )
        rate = (
            self.jump_air_tuck_ref_rate
            if compact
            else self.jump_air_deploy_ref_rate
        )
        for side in ("l", "r"):
            self.jump_air_reference[side] = self._move_towards(
                self.jump_air_reference[side], target, rate * dt
            )

    def _begin_landing_buffer(self, data: mujoco.MjData) -> None:
        if self.landing_buffer_phase != "inactive":
            return
        refs = {
            side: float(np.clip(
                self.vmc_kinematics(data, side).l0, 0.10, 0.34
            ))
            for side in ("l", "r")
        }
        ref_max = min(refs.values()) + LANDING_LOW_CONTACT_SPLIT_MAX
        # A touchdown reference may be longer than the operator's selected
        # standing length.  Clamp it before the first landing-support update;
        # otherwise the captured airborne geometry can command an immediate
        # leg extension at contact.
        refs = {
            side: min(value, ref_max, self.normal_leg_length)
            for side, value in refs.items()
        }
        self.landing_length_reference = refs
        # Raw wheel contact starts actuator capture immediately.  Ground-state
        # confirmation remains debounced below, but it must not leave the air
        # reaction-wheel and hip controller active against a real ground force.
        self.landing_buffer_phase = "capture"
        self.landing_buffer_phase_elapsed = 0.0
        self.landing_buffer_elapsed = 0.0
        self.landing_exit_stable_elapsed = 0.0
        self.landing_support_handoff = self.last_support.copy()
        self.jump_landing_tp_command = {
            side: float(self.last_output[side][1]) for side in ("l", "r")
        }
        self.jump_landing_wheel_command = {
            # ``step`` clears data.ctrl before contact processing, so capture
            # the previously applied logical air command rather than that
            # already-cleared actuator slot.
            side: float(self.wheel_sign[side] * self.jump_air_wheel_torque)
            for side in ("l", "r")
        }
        self.jump_air_wheel_torque = 0.0

    def _update_landing_buffer(self, dt: float) -> None:
        if self.landing_buffer_phase == "inactive":
            return
        self.landing_buffer_phase_elapsed += dt
        self.landing_buffer_elapsed += dt
        lengths = {
            side: self.vmc_kinematics(self._jump_data, side).l0
            for side in ("l", "r")
        }
        average_speed = 0.5 * sum(
            abs(self.length_rate_lpf[side]) for side in ("l", "r")
        )
        if self.landing_buffer_phase in {"capture", "hold"}:
            # Follow the compression stroke down without ever commanding the
            # captured, fully extended contact length back into the mechanism.
            # This removes the rebound that previously followed first impact.
            if not self.jump_off_ground:
                for side in ("l", "r"):
                    ratchet_target = min(
                        self.normal_leg_length,
                        lengths[side] + LANDING_LOW_RATCHET_MARGIN,
                    )
                    ratchet_target = min(
                        ratchet_target, self.landing_length_reference[side]
                    )
                    self.landing_length_reference[side] = self._move_towards(
                        self.landing_length_reference[side],
                        ratchet_target,
                        LANDING_LOW_RATCHET_REF_RATE * dt,
                    )
            if (
                self.landing_buffer_phase == "capture"
                and self.landing_buffer_phase_elapsed >= LANDING_CAPTURE_TIME
            ):
                self.landing_buffer_phase = "hold"
                self.landing_buffer_phase_elapsed = 0.0
            elif self.landing_buffer_phase == "hold":
                average_length = 0.5 * sum(lengths.values())
                average_ref = 0.5 * sum(self.landing_length_reference.values())
                settled = (
                    abs(average_length - average_ref)
                    <= LANDING_SETTLE_LENGTH_ERROR
                    and average_speed <= LANDING_SETTLE_LENGTH_SPEED
                )
                hold_done = (
                    self.landing_buffer_phase_elapsed >= LANDING_HOLD_MAX_TIME
                    or (
                        self.landing_buffer_phase_elapsed >= LANDING_HOLD_MIN_TIME
                        and settled
                    )
                )
                if hold_done:
                    self.landing_buffer_phase = "return"
                    self.landing_buffer_phase_elapsed = 0.0
        elif self.landing_buffer_phase == "return":
            for side in ("l", "r"):
                self.landing_length_reference[side] = self._move_towards(
                    self.landing_length_reference[side],
                    self.normal_leg_length,
                    LANDING_RETURN_REF_RATE_LOW * dt,
                )
            target_reached = all(
                abs(value - self.normal_leg_length) <= 0.0005
                for value in self.landing_length_reference.values()
            )
            stable_ground = not self.jump_off_ground and (
                average_speed <= LANDING_EXIT_LENGTH_SPEED
            )
            if target_reached and stable_ground:
                self.landing_exit_stable_elapsed += dt
            else:
                self.landing_exit_stable_elapsed = 0.0
            if (
                target_reached
                and self.landing_buffer_phase_elapsed >= LANDING_PID_RETURN_TIME
                and self.landing_exit_stable_elapsed >= LANDING_EXIT_STABLE_TIME
            ):
                self.jump_landing_tp_handoff = (
                    self.jump_landing_tp_command.copy()
                )
                self.jump_landing_tp_handoff_elapsed = 0.0
                self.landing_buffer_phase = "inactive"
        if self.landing_buffer_phase != "inactive":
            ref_max = min(self.landing_length_reference.values()) + (
                LANDING_LOW_CONTACT_SPLIT_MAX
            )
            for side in ("l", "r"):
                self.landing_length_reference[side] = min(
                    self.landing_length_reference[side], ref_max
                )
                self.length_reference[side] = self.landing_length_reference[side]

    def _update_jump_contacts(self, data: mujoco.MjData) -> None:
        contacts = self._wheel_contacts(data)
        self.jump_raw_wheel_contact = contacts
        if (
            self.jump_state == "idle"
            and self.landing_buffer_phase == "inactive"
            and not self.jump_off_ground
        ):
            return
        if not self.jump_off_ground:
            if not any(contacts.values()):
                self.jump_takeoff_count += 1
            else:
                self.jump_takeoff_count = 0
            if self.jump_takeoff_count >= JUMP_TAKEOFF_CONFIRM_COUNT:
                self.jump_off_ground = True
                self.jump_takeoff_seen = True
                self.jump_takeoff_count = 0
                self.jump_landing_count = 0
                if not self.jump_air_profile_active:
                    self._start_jump_air_profile(data)
                if self.jump_state in {"thrust", "abort_settle"}:
                    self._enter_jump_state("flight")
        else:
            if any(contacts.values()) and self.landing_buffer_phase == "inactive":
                self._begin_landing_buffer(data)
            if all(contacts.values()):
                self.jump_landing_count += 1
            else:
                self.jump_landing_count = 0
            if self.jump_landing_count >= JUMP_LANDING_CONFIRM_COUNT:
                self.jump_off_ground = False
                self.jump_landing_seen = True
                self.jump_landing_count = 0
                self.jump_landing_tp_handoff = {
                    side: float(self.last_output[side][1])
                    for side in ("l", "r")
                }
                self.jump_landing_tp_handoff_elapsed = 0.0
                if self.landing_buffer_phase == "inactive":
                    self._begin_landing_buffer(data)
                if self.jump_state == "flight":
                    self._enter_jump_state("landing")

    def _update_jump_state(self, data: mujoco.MjData, dt: float) -> None:
        self._jump_data = data
        self._update_jump_contacts(data)
        if self.landing_buffer_phase != "inactive":
            self._update_landing_buffer(dt)
        if self.jump_state == "idle":
            if self.jump_off_ground and self.jump_air_profile_active:
                self._update_jump_air_profile(dt)
            return
        self.jump_elapsed += dt
        if (
            abs(self.body_pitch(data)) > self.jump_abort_pitch_limit
            or abs(self.body_roll(data)) > self.jump_abort_roll_limit
        ):
            self._finish_jump("safety")
            return

        lengths = {
            side: self.vmc_kinematics(data, side).l0 for side in ("l", "r")
        }
        average_length = 0.5 * sum(lengths.values())
        average_rate = 0.5 * sum(
            abs(self.length_rate_lpf[side]) for side in ("l", "r")
        )
        if self.jump_state == "preload":
            self.jump_length_target = JUMP_PRELOAD_LENGTH
            if (
                average_length <= JUMP_PRELOAD_READY_LENGTH
                and average_rate <= JUMP_PRELOAD_READY_RATE
                and self.jump_elapsed >= JUMP_PRELOAD_MIN_TIME
            ):
                self._enter_jump_state("thrust", data)
                self.jump_length_target = JUMP_THRUST_LENGTH
            elif self.jump_elapsed >= JUMP_PRELOAD_MAX_TIME:
                self._finish_jump("preload_timeout")
        elif self.jump_state == "thrust":
            if self.jump_elapsed < JUMP_THRUST_BOOST_TIME:
                self.jump_length_target = JUMP_THRUST_LENGTH
            else:
                self._start_jump_air_profile(data)
                self._update_jump_air_profile(dt)
                self.jump_length_target = 0.5 * sum(
                    self.jump_air_reference.values()
                )
            if self.jump_elapsed >= JUMP_THRUST_MAX_TIME:
                self.jump_fault = "no_takeoff"
                self._enter_jump_state("abort_settle")
                self.jump_air_profile_active = False
                self.jump_length_target = float(np.clip(
                    average_length, JUMP_THRUST_LENGTH, 0.34
                ))
        elif self.jump_state == "flight":
            self.jump_length_target = self.normal_leg_length
            self._start_jump_air_profile(data)
            self._update_jump_air_profile(dt)
            if self.jump_elapsed >= JUMP_FLIGHT_TIMEOUT:
                self._finish_jump("flight_timeout")
        elif self.jump_state == "landing":
            self.jump_length_target = self.normal_leg_length
            stable = (
                not self.jump_off_ground
                and abs(self.body_pitch(data)) <= JUMP_LANDING_PITCH_MAX
                and abs(self.body_roll(data)) <= JUMP_LANDING_ROLL_MAX
                and abs(lengths["l"] - lengths["r"])
                <= JUMP_LANDING_LENGTH_DIFF_MAX
            )
            self.jump_landing_ground_elapsed = (
                self.jump_landing_ground_elapsed + dt if stable else 0.0
            )
            timeout_ok = (
                not self.jump_off_ground
                and abs(self.body_pitch(data)) <= JUMP_PRECHECK_PITCH_MAX
                and abs(self.body_roll(data)) <= JUMP_PRECHECK_ROLL_MAX
                and abs(lengths["l"] - lengths["r"])
                <= JUMP_LANDING_TIMEOUT_LENGTH_DIFF_MAX
            )
            if (
                self.jump_landing_ground_elapsed
                >= JUMP_LANDING_GROUND_CONFIRM_TIME
            ):
                self._finish_jump("none")
            elif self.jump_elapsed >= JUMP_LANDING_TIMEOUT:
                self._finish_jump("landing_timeout" if timeout_ok else "safety")
        elif self.jump_state == "abort_settle":
            if self.jump_elapsed >= JUMP_ABORT_HOLD_TIME:
                self.jump_length_target = self._move_towards(
                    self.jump_length_target,
                    self.normal_leg_length,
                    JUMP_ABORT_RETRACT_RATE * dt,
                )
            if self.jump_elapsed >= JUMP_ABORT_SETTLE_TIME:
                self._finish_jump("no_takeoff")
        if self.jump_state != "idle":
            for side in ("l", "r"):
                self.length_reference[side] = self.jump_length_target

    def _jump_air_supports(self, lengths: dict[str, float]) -> dict[str, float]:
        support = {}
        for side in ("l", "r"):
            spring = (
                self._gas_virtual_support(lengths[side])
                if self.gas_enabled else 0.0
            )
            support[side] = (
                self.jump_air_deploy_kp
                * (self.jump_air_reference[side] - lengths[side])
                - self.jump_air_deploy_kd * self.length_rate_lpf[side]
                - spring
            )
        common = float(np.clip(
            0.5 * (support["l"] + support["r"]),
            -self.jump_air_retract_force_limit,
            SUPPORT_FORCE_LIMIT,
        ))
        differential = 0.5 * (support["l"] - support["r"])
        support["l"] = common + differential
        support["r"] = common - differential
        sync = float(np.clip(
            JUMP_AIR_LENGTH_SYNC_KP * (lengths["r"] - lengths["l"])
            + JUMP_AIR_LENGTH_SYNC_KD
            * (self.length_rate_lpf["r"] - self.length_rate_lpf["l"]),
            -JUMP_AIR_LENGTH_SYNC_LIMIT,
            JUMP_AIR_LENGTH_SYNC_LIMIT,
        ))
        support["l"] += sync
        support["r"] -= sync
        return {
            side: float(np.clip(
                support[side],
                -self.jump_air_retract_force_peak,
                SUPPORT_FORCE_LIMIT,
            ))
            for side in ("l", "r")
        }

    def _landing_supports(self, lengths: dict[str, float]) -> dict[str, float]:
        return_gain = 0.0
        if self.landing_buffer_phase == "return":
            return_gain = float(np.clip(
                self.landing_buffer_phase_elapsed / LANDING_PID_RETURN_TIME,
                0.0,
                1.0,
            ))
        kp = LANDING_LENGTH_KP + return_gain * (LENGTH_KP - LANDING_LENGTH_KP)
        kd = LANDING_LENGTH_KD_LOW + return_gain * (
            LENGTH_KD_LOW - LANDING_LENGTH_KD_LOW
        )
        supports = {}
        for side in ("l", "r"):
            spring = (
                self._gas_virtual_support(lengths[side])
                if self.gas_enabled else 0.0
            )
            length_force = float(np.clip(
                kp * (self.landing_length_reference[side] - lengths[side])
                - kd * self.length_rate_lpf[side],
                -LANDING_LENGTH_OUTPUT_LIMIT_LOW,
                LANDING_LENGTH_OUTPUT_LIMIT_LOW,
            ))
            supports[side] = (
                0.5 * TOTAL_MASS * GRAVITY  # firmware CHASSIS_SUPPORT_MASS (see below)
                - spring
                + length_force
                + self.support_trim[side]
                + (self.roll_force if side == "l" else -self.roll_force)
            )
        sync = (1.0 - return_gain) * (
            JUMP_LANDING_SYNC_KP * (lengths["r"] - lengths["l"])
            + JUMP_LANDING_SYNC_KD
            * (self.length_rate_lpf["r"] - self.length_rate_lpf["l"])
        )
        sync = float(np.clip(
            sync, -JUMP_LANDING_SYNC_LIMIT, JUMP_LANDING_SYNC_LIMIT
        ))
        supports["l"] += sync
        supports["r"] -= sync
        return {
            side: float(np.clip(
                supports[side], -SUPPORT_FORCE_LIMIT, SUPPORT_FORCE_LIMIT
            ))
            for side in ("l", "r")
        }

    def _jump_supports(self, lengths: dict[str, float]) -> dict[str, float]:
        if self.jump_off_ground or (
            self.jump_state == "thrust" and self.jump_air_profile_active
        ):
            air = self._jump_air_supports(lengths)
            if self.landing_buffer_phase != "inactive":
                landing = self._landing_supports(lengths)
                blend = float(np.clip(
                    self.landing_buffer_elapsed / LANDING_SUPPORT_BLEND_TIME,
                    0.0,
                    1.0,
                ))
                return {
                    side: air[side] + blend * (landing[side] - air[side])
                    for side in ("l", "r")
                }
            return air
        if self.landing_buffer_phase != "inactive":
            return self._landing_supports(lengths)

        thrust = self.jump_state == "thrust"
        if thrust:
            kp = JUMP_LENGTH_KP_THRUST
            kd = JUMP_LENGTH_KD_THRUST
            output_limit = JUMP_LENGTH_LIMIT_THRUST
        else:
            kp = JUMP_LENGTH_KP_PRELOAD
            kd = JUMP_LENGTH_KD_PRELOAD
            output_limit = JUMP_LENGTH_LIMIT_PRELOAD
        supports = {}
        for side in ("l", "r"):
            spring = (
                self._gas_virtual_support(lengths[side])
                if self.gas_enabled else 0.0
            )
            length_force = float(np.clip(
                kp * (self.jump_length_target - lengths[side])
                - kd * self.length_rate_lpf[side],
                -output_limit,
                output_limit,
            ))
            supports[side] = (
                0.5 * TOTAL_MASS * GRAVITY  # firmware CHASSIS_SUPPORT_MASS (see below)
                - spring
                + length_force
                + self.support_trim[side]
                + (self.roll_force if side == "l" else -self.roll_force)
            )
        if thrust:
            sync = float(np.clip(
                JUMP_LENGTH_SYNC_KP * (lengths["r"] - lengths["l"])
                + JUMP_LENGTH_SYNC_KD
                * (self.length_rate_lpf["r"] - self.length_rate_lpf["l"]),
                -JUMP_LENGTH_SYNC_LIMIT,
                JUMP_LENGTH_SYNC_LIMIT,
            ))
            supports["l"] += sync
            supports["r"] -= sync
        boost_active = thrust and self.jump_elapsed < JUMP_THRUST_BOOST_TIME
        for side in ("l", "r"):
            if boost_active:
                supports[side] = max(
                    supports[side] + JUMP_THRUST_BOOST_FORCE,
                    JUMP_THRUST_SUPPORT_FLOOR,
                )
            if thrust:
                supports[side] = max(supports[side], 0.0)
            supports[side] = float(np.clip(
                supports[side], -SUPPORT_FORCE_LIMIT, SUPPORT_FORCE_LIMIT
            ))
        return supports

    def _apply_jump_hip_control(self, data: mujoco.MjData, dt: float) -> None:
        if self.jump_state == "thrust" and not self.jump_off_ground:
            pitch = self.body_pitch(data)
            pitch_rate = -float(self.last_state["l"][5])
            alpha = float(np.clip(
                dt / (JUMP_THRUST_PITCH_RATE_FILTER_TAU + dt), 0.0, 1.0
            ))
            self.jump_thrust_pitch_rate_lpf += alpha * (
                pitch_rate - self.jump_thrust_pitch_rate_lpf
            )
            average_length = 0.5 * sum(
                self.vmc_kinematics(data, side).l0 for side in ("l", "r")
            )
            rate_gain = float(np.clip(
                (average_length - self.jump_thrust_pitch_rate_start_l0)
                / (
                    self.jump_thrust_pitch_rate_full_l0
                    - self.jump_thrust_pitch_rate_start_l0
                ),
                0.0,
                1.0,
            ))
            # chassis_vx reads nose-positive after step()'s single frame
            # translation, so a faster forward launch INCREASES the pitch
            # rate the thrust must store here; the historical minus sign
            # silently assumed tail-positive and left ~5 deg of apex pitch.
            rate_target = float(np.clip(
                self.jump_thrust_pitch_rate_target
                + JUMP_THRUST_PITCH_RATE_VX_GAIN * self.jump_launch_vx,
                self.jump_thrust_pitch_rate_min,
                self.jump_thrust_pitch_rate_max,
            )) * rate_gain
            pitch_request = float(np.clip(
                -self.jump_thrust_pitch_kp * pitch
                - self.jump_thrust_pitch_kd
                * (self.jump_thrust_pitch_rate_lpf - rate_target),
                -self.jump_thrust_pitch_limit,
                self.jump_thrust_pitch_limit,
            ))
            self.jump_thrust_pitch_tp = self._move_towards(
                self.jump_thrust_pitch_tp,
                pitch_request,
                self.jump_thrust_pitch_tp_slew_rate * dt,
            )
            if self.jump_air_profile_active:
                self.jump_thrust_theta_ref = self._move_towards(
                    self.jump_thrust_theta_ref,
                    0.0,
                    JUMP_PRETUCK_THETA_REF_RATE * dt,
                )
            theta = {
                side: np.pi / 2.0
                - self.vmc_kinematics(data, side).phi0
                + pitch
                for side in ("l", "r")
            }
            theta_common = 0.5 * sum(theta.values())
            theta_rate_common = 0.5 * sum(self.theta_rate_lpf.values())
            theta_hold = float(np.clip(
                self.jump_thrust_theta_kp
                * (theta_common - self.jump_thrust_theta_ref)
                + self.jump_thrust_theta_kd * theta_rate_common,
                -self.jump_thrust_theta_limit,
                self.jump_thrust_theta_limit,
            ))
            tracking_error = abs(theta_common - self.jump_thrust_theta_ref)
            if (
                self.jump_thrust_theta_kp == 0.0
                and self.jump_thrust_theta_kd == 0.0
            ):
                pitch_authority = 1.0
            else:
                pitch_authority = float(np.clip(
                    (JUMP_THRUST_THETA_YIELD_END - tracking_error)
                    / (
                        JUMP_THRUST_THETA_YIELD_END
                        - JUMP_THRUST_THETA_YIELD_START
                    ),
                    0.0,
                    1.0,
                ))
            common = 0.5 * (
                self.last_output["l"][1] + self.last_output["r"][1]
            )
            differential = 0.5 * (
                self.last_output["l"][1] - self.last_output["r"][1]
            )
            common = pitch_authority * (
                JUMP_THRUST_THETA_LQR_GAIN * common
                + self.jump_thrust_pitch_tp
            ) + theta_hold
            self.last_output["l"][1] = common + differential
            self.last_output["r"][1] = common - differential
        elif (
            self.landing_buffer_phase != "inactive"
            and not self.jump_off_ground
        ):
            pitch = self.body_pitch(data)
            for side in ("l", "r"):
                theta = (
                    np.pi / 2.0
                    - self.vmc_kinematics(data, side).phi0
                    + pitch
                )
                target = float(np.clip(
                    self.jump_landing_theta_kp * theta
                    + self.jump_landing_theta_kd * self.theta_rate_lpf[side],
                    -self.jump_landing_theta_limit,
                    self.jump_landing_theta_limit,
                ))
                self.jump_landing_tp_command[side] = self._move_towards(
                    self.jump_landing_tp_command[side],
                    target,
                    LANDING_CAPTURE_TP_SLEW_RATE
                    * min(dt, CONTROL_PERIOD_S),
                )
                self.last_output[side][1] = self.jump_landing_tp_command[side]
            # Contact capture owns the wheel channel from the first raw
            # contact, so the airborne reaction-wheel request cannot become a
            # ground-drive impulse while contact confirmation is pending.
            self.jump_air_wheel_torque = 0.0
        elif self.jump_off_ground:
            pitch = self.body_pitch(data)
            pitch_rate_raw = -float(self.last_state["l"][5])
            theta_rate_raw = 0.5 * sum(self.theta_rate_lpf.values())
            if self.jump_air_rate_filter_tau > 0.0:
                rate_alpha = float(np.clip(
                    dt / (self.jump_air_rate_filter_tau + dt), 0.0, 1.0
                ))
                self.jump_air_pitch_rate_lpf += rate_alpha * (
                    pitch_rate_raw - self.jump_air_pitch_rate_lpf
                )
                self.jump_air_theta_rate_lpf += rate_alpha * (
                    theta_rate_raw - self.jump_air_theta_rate_lpf
                )
                pitch_rate = self.jump_air_pitch_rate_lpf
                theta_rate_common = self.jump_air_theta_rate_lpf
            else:
                self.jump_air_pitch_rate_lpf = pitch_rate_raw
                self.jump_air_theta_rate_lpf = theta_rate_raw
                pitch_rate = pitch_rate_raw
                theta_rate_common = theta_rate_raw
            compact = self.jump_air_elapsed < (
                self.jump_air_tuck_time + self.jump_air_tuck_hold_time
            )
            pitch_reference = float(np.clip(
                (
                    self.jump_air_tuck_pitch_bias
                    if compact
                    else self.jump_air_deploy_pitch_bias
                )
                + self.jump_air_moving_pitch_gain * self.drive_forward_target,
                -self.jump_air_pitch_reference_limit,
                self.jump_air_pitch_reference_limit,
            ))
            pitch_tp = float(np.clip(
                -self.jump_air_pitch_kp * (pitch - pitch_reference)
                - self.jump_air_pitch_kd * pitch_rate,
                -self.jump_air_pitch_limit,
                self.jump_air_pitch_limit,
            ))
            # The hip common axis cannot simultaneously hold body pitch and
            # point both legs at the ground.  In flight the two wheel rotors
            # are a second, real internal reaction axis.  Drive them with the
            # firmware common-wheel polarity; no ground force is assumed.
            wheel_pitch_feedback = (
                self.jump_air_wheel_pitch_kp * (pitch - pitch_reference)
                + self.jump_air_wheel_pitch_kd * pitch_rate
            )
            logical_wheel_rate = 0.5 * sum(
                self.wheel_sign[side]
                * float(data.qvel[
                    self.model.jnt_dofadr[self.sides[side].wheel_joint]
                ])
                for side in ("l", "r")
            )
            if self.jump_air_elapsed >= self.jump_air_wheel_speed_damping_start:
                wheel_pitch_feedback -= (
                    self.jump_air_wheel_speed_kd * logical_wheel_rate
                )
            theta = {}
            for side in ("l", "r"):
                theta[side] = (
                    np.pi / 2.0
                    - self.vmc_kinematics(data, side).phi0
                    + pitch
                )
            theta_common = 0.5 * sum(theta.values())
            theta_hold_common = float(np.clip(
                self.jump_air_theta_kp * theta_common
                + self.jump_air_theta_kd * theta_rate_common,
                -self.jump_air_theta_limit,
                self.jump_air_theta_limit,
            ))
            theta_weight = (
                self.jump_air_theta_weight_tuck
                if compact else self.jump_air_theta_weight_deploy
            )
            # Feed the commanded leg-swing reaction into the two real wheel
            # rotors.  This gives the airborne system its second internal
            # pitch axis: hips can point the legs at the landing surface while
            # wheel angular momentum cancels the equal body reaction.
            self.jump_air_wheel_torque = float(np.clip(
                self.jump_air_wheel_sign * (
                    wheel_pitch_feedback
                    + self.jump_air_wheel_theta_ff
                    * theta_weight
                    * theta_hold_common
                ),
                -self.jump_air_wheel_torque_limit,
                self.jump_air_wheel_torque_limit,
            ))
            # Deploy the reaction-wheel torque away before touchdown so the
            # rotors reach ~0 wheel speed at first contact; otherwise the
            # stored wheel angular momentum becomes a horizontal ground
            # impulse that slides the chassis away from the launch spot.
            air_elapsed = self.jump_air_elapsed
            tuck_total = self.jump_air_tuck_time + self.jump_air_tuck_hold_time
            taper_start = tuck_total + self.jump_air_wheel_taper_delay
            if air_elapsed > taper_start:
                deploy_progress = float(np.clip(
                    (air_elapsed - taper_start)
                    / self.jump_air_wheel_taper_time,
                    0.0,
                    1.0,
                ))
                self.jump_air_wheel_torque *= (1.0 - deploy_progress)
            lqr_differential = JUMP_AIR_LQR_TP_GAIN * 0.5 * (
                self.last_output["l"][1] - self.last_output["r"][1]
            )
            theta_differential = 0.5 * self.jump_air_theta_kp * (
                theta["l"] - theta["r"]
            ) + 0.5 * self.jump_air_theta_kd * (
                self.theta_rate_lpf["l"] - self.theta_rate_lpf["r"]
            )
            common = float(np.clip(
                pitch_tp + theta_weight * theta_hold_common,
                -self.jump_air_common_tp_limit,
                self.jump_air_common_tp_limit,
            ))
            if self.jump_air_common_tp_filter_tau > 0.0:
                common_alpha = float(np.clip(
                    dt / (self.jump_air_common_tp_filter_tau + dt),
                    0.0,
                    1.0,
                ))
                self.jump_air_common_tp_lpf += common_alpha * (
                    common - self.jump_air_common_tp_lpf
                )
                common = self.jump_air_common_tp_lpf
            else:
                self.jump_air_common_tp_lpf = common
            differential = lqr_differential + theta_weight * theta_differential
            differential_limit = max(
                self.jump_air_common_tp_limit - abs(common), 0.0
            )
            differential = float(np.clip(
                differential, -differential_limit, differential_limit
            ))
            self.last_output["l"][1] = common + differential
            self.last_output["r"][1] = common - differential
            if self.landing_buffer_phase != "inactive":
                self.jump_landing_tp_command = {
                    side: float(self.last_output[side][1])
                    for side in ("l", "r")
                }
        elif (
            self.jump_landing_tp_handoff_elapsed
            < JUMP_LANDING_TP_HANDOFF_TIME
            and self.jump_landing_seen
        ):
            gain = 1.0 - (
                self.jump_landing_tp_handoff_elapsed
                / JUMP_LANDING_TP_HANDOFF_TIME
            )
            for side in ("l", "r"):
                self.last_output[side][1] = (
                    gain * self.jump_landing_tp_handoff[side]
                    + (1.0 - gain) * self.last_output[side][1]
                )
            self.jump_landing_tp_handoff_elapsed += dt
            self.jump_air_wheel_torque = 0.0
        else:
            self.jump_air_wheel_torque = 0.0

    def _update_landing_wheel_control(
        self, data: mujoco.MjData, dt: float
    ) -> None:
        if self.landing_buffer_phase == "inactive":
            return
        pitch_reference = float(np.clip(
            self.jump_air_moving_pitch_gain * self.drive_forward_target,
            np.radians(-8.0),
            np.radians(8.0),
        ))
        pitch_rate = -float(self.last_state["l"][5])
        strong_capture = (
            self.landing_buffer_phase == "capture"
            or (
                self.landing_buffer_phase == "hold"
                and self.landing_buffer_phase_elapsed
                < self.jump_landing_strong_hold_time
            )
        )
        if strong_capture:
            kp = self.jump_landing_capture_wheel_pitch_kp
            kd = self.jump_landing_capture_wheel_pitch_kd
            torque_limit = self.jump_landing_capture_wheel_torque_limit
        else:
            kp = self.jump_landing_wheel_pitch_kp
            kd = self.jump_landing_wheel_pitch_kd
            torque_limit = self.jump_landing_wheel_torque_limit
        landing_torque = float(np.clip(
            kp * (self.body_pitch(data) - pitch_reference) + kd * pitch_rate,
            -torque_limit,
            torque_limit,
        ))
        slew_rate = (
            self.jump_landing_capture_wheel_slew_rate
            if self.landing_buffer_phase == "capture"
            else self.jump_landing_wheel_slew_rate
        )
        for side in ("l", "r"):
            target = self.wheel_sign[side] * landing_torque
            self.jump_landing_wheel_command[side] = self._move_towards(
                self.jump_landing_wheel_command[side],
                target,
                slew_rate * min(dt, CONTROL_PERIOD_S),
            )
