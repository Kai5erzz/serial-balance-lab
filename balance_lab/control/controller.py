"""TraditionalLQR: firmware six-state LQR + five-bar VMC support loop.

The class is composed from three concern mixins (jump, guards, recovery)
plus the core below; the split is purely by responsibility, all methods
share one controller state through ``self``.  The feedback law and gain
polynomials come from ``src/task/chassis/control/chassis_control.c``;
MuJoCo supplies the state instead of the IMU/encoder/Kalman pipeline, and
wheel and hip output signs stay explicit constants so they can be
calibrated against the assembled model.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import mujoco
import numpy as np

if TYPE_CHECKING:
    from ..sysid.parameters import ModelParameters

from .tuning import *  # noqa: F401,F403
from .kinematics import RecoverySwingAxis, SideIds, VmcKinematics, unit
from .jump import JumpControlMixin
from .guards import TerrainSafetyMixin
from .recovery import RecoveryMixin


class TraditionalLQR(JumpControlMixin, TerrainSafetyMixin, RecoveryMixin):
    """Firmware traditional LQR, position observer, and VMC support loop.

The firmware's wheel torque path is copied directly: left receives ``-T``
and right receives ``+T``.  The real DM multi-turn zero is translated by
``encoder_joint_reading``; feedback and closed-loop geometry stay in the
MJCF CAD hinge frame."""


    def __init__(self, model: mujoco.MjModel) -> None:
        self.model = model
        self.base = self._body("base_link")
        self.ground = self._geom("ground")
        # The platform scene replaces ``ground`` with ``ground_outer`` and
        # three named step geoms.  Keep the contact whitelist at the controller
        # boundary so jump prechecks and landing detection use the same ground
        # set in both scenes.
        self.ground_geoms = {
            geom_id for name in (
                "ground", "ground_outer", "platform_top",
                "platform_step1", "platform_step2", "platform_step3",
            )
            for geom_id in (self._geom(name),)
            if geom_id >= 0
        }
        # The CAD assembly is yawed inside its export frame.  The gimbal is
        # mounted on the vehicle centreline and its fixed mounting point is
        # the reliable longitudinal reference, unlike the very small
        # horizontal residual of a nominally vertical closed leg.
        self.gimbal_body = self._body("gimbal_yaw")
        self.gimbal_joint = self._joint("gimbal_yaw")
        self.gimbal_motor = self._actuator("gimbal_yaw_motor")
        self.gimbal_qpos = int(self.model.jnt_qposadr[self.gimbal_joint])
        self.gimbal_dof = int(self.model.jnt_dofadr[self.gimbal_joint])
        self.forward_body: np.ndarray | None = None
        self.forward_world: np.ndarray | None = None
        self.up_body: np.ndarray | None = None
        self.lateral_body: np.ndarray | None = None
        self.sides = {
            "l": SideIds(self._joint("l_j1"), self._joint("l_j2"), self._body("l_wheel"), self._joint("l_wheel"), self._actuator("l_wheel_motor"), (self._actuator("l_j1_motor"), self._actuator("l_j2_motor")), self._site("l_gas_thigh"), self._site("l_gas_leg")),
            "r": SideIds(self._joint("r_j1"), self._joint("r_j2"), self._body("r_wheel"), self._joint("r_wheel"), self._actuator("r_wheel_motor"), (self._actuator("r_j1_motor"), self._actuator("r_j2_motor")), self._site("r_gas_thigh"), self._site("r_gas_leg")),
        }
        self.length_reference: dict[str, float] = {}
        self.length_integral = {"l": 0.0, "r": 0.0}
        self.theta_rate_lpf = {"l": 0.0, "r": 0.0}
        self.length_rate_lpf = {"l": 0.0, "r": 0.0}
        self.jump_hip_motor_filter_tau = 0.0
        self.jump_hip_motor_torque_lpf = {
            "l": np.zeros(2), "r": np.zeros(2)
        }
        self.previous: dict[str, float] = {}
        self.base_position0 = np.zeros(3)
        self.chassis_vx = 0.0
        self.drive_speed_trim = 0.0
        self.offground_suppress_elapsed = 0.0
        self.cruise_integral = 0.0
        self.danger_reason = "none"
        self.danger_confirmed = False
        self.lqr_virtual_x = 0.0
        self.lqr_position_state = "drive"
        self.lqr_position_capture_timer = 0.0
        self.lqr_position_hold_x = 0.0
        self.lqr_position_brake_offset = 0.0
        self.theta_comp_apply_gain = 0.0
        # Start disarmed so the initial CAD pose can be inspected safely.
        self.enabled = False
        self.stance_enabled = True
        self.gas_enabled = True
        self.gimbal_locked = True
        self.lqr_gain = 0.0
        self.wheel_sign = {"l": 1.0, "r": -1.0}
        # The two wheel joint axes are opposite in the CAD assembly.  Testing
        # the complete CAD-zero startup (not merely its final settled pose)
        # selects this map: the reverse map lets gravity drive the chassis
        # into the floor before it eventually recovers.
        self.cad_stand_wheel_sign = {"l": 1.0, "r": -1.0}
        # vmc_kinematics now returns the exact CAD-actuator Jacobian, including
        # the J1/J2 crossing and left raw-angle sign.  Firmware Tp therefore
        # enters virtual work without an additional empirical sign.
        self.vmc_tp_sign = 1.0
        self.last_state = {"l": np.zeros(6), "r": np.zeros(6)}
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_support = {"l": 0.0, "r": 0.0}
        self.support_trim = {"l": 0.0, "r": 0.0}
        self.roll_force = 0.0
        self.roll_rate_lpf_deg_s = 0.0
        # Real-vehicle roll compensation channels (code_611 port): the
        # centrifugal feedforward and the learned static trim, feeding the
        # same left/right differential path as roll_force in normal support.
        self.lateral_inertial_ff = 0.0
        self.roll_static_trim_force = 0.0
        self.roll_static_trim_applied = 0.0
        self.roll_static_trim_timer = 0.0
        self.last_motor_torque = {"l": np.zeros(2), "r": np.zeros(2)}
        self.control_elapsed = 0.0
        self.chassis_vx = 0.0
        self.drive_speed_trim = 0.0
        self.offground_suppress_elapsed = 0.0
        self.cruise_integral = 0.0
        self.danger_reason = "none"
        self.danger_confirmed = False
        self.lqr_virtual_x = 0.0
        self.lqr_position_state = "drive"
        self.lqr_position_capture_timer = 0.0
        self.lqr_position_hold_x = 0.0
        self.lqr_position_brake_offset = 0.0
        self.theta_comp_apply_gain = 0.0
        self.lqr_leg_level = "mid"
        self.recovery_state = "idle"
        self.recovery_elapsed = 0.0
        self.recovery_stable_elapsed = 0.0
        self.recovery_target: dict[str, VmcKinematics] = {}
        self.recovery_physical_length: dict[str, float] = {}
        self.recovery_last_l0: dict[str, float] = {}
        self.recovery_last_phi0: dict[str, float] = {}
        self.recovery_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.fw_recovery_axes: dict[str, RecoverySwingAxis] = {}
        self.fw_recovery_l0_ref = {"l": 0.0, "r": 0.0}
        self.fw_recovery_length_rate = {"l": 0.0, "r": 0.0}
        self.fw_recovery_angle_gain = 0.0
        self.fw_recovery_force_gain = {"l": 0.0, "r": 0.0}
        self.fw_recovery_breakaway_released = False
        self.fw_recovery_handoff_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.fw_recovery_length_hold_until = 0.0
        # State retained for the legacy, currently locked LQR experiment.
        self.cad_stand_last: dict[str, tuple[float, float]] = {}
        self.cad_stand_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.cad_stand_wheel_origin: np.ndarray | None = None
        self.cad_stand_pitch_previous = 0.0
        self.prone_swing_elapsed = 0.0
        self.previous_base_position = np.zeros(3)
        self.drive_forward_target = 0.0
        self.drive_yaw_rate_target = 0.0
        # Keep the validated 0.150 m equilibrium as the library default.
        # The interactive runner selects LEG_LENGTH_DEFAULT explicitly so
        # changing the UI default does not invalidate headless calibration
        # tests or firmware-frame startup checks.
        self.normal_leg_length = VMC_NORMAL_STAND_LENGTH
        self.yaw_rate = 0.0
        self.yaw_output = 0.0
        self.yaw_integral = 0.0
        self.previous_heading = 0.0
        self.jump_state = "idle"
        self.jump_fault = "none"
        self.jump_elapsed = 0.0
        self.jump_last_accept_time: float | None = None
        self.jump_off_ground = False
        self.jump_takeoff_count = 0
        self.jump_landing_count = 0
        self.jump_takeoff_seen = False
        self.jump_landing_seen = False
        self.jump_length_target = self.normal_leg_length
        self.jump_air_profile_active = False
        self.jump_air_elapsed = 0.0
        self.jump_air_reference = {"l": JUMP_AIR_LANDING_LENGTH, "r": JUMP_AIR_LANDING_LENGTH}
        self.jump_launch_vx = 0.0
        self.jump_takeoff_position: np.ndarray | None = None
        self.jump_stationary_position_hold = False
        self.jump_thrust_theta_ref = 0.0
        self.jump_thrust_pitch_rate_lpf = 0.0
        self.jump_thrust_pitch_tp = 0.0
        self.jump_raw_wheel_contact = {"l": False, "r": False}
        self.jump_landing_ground_elapsed = 0.0
        self.jump_landing_tp_handoff = {"l": 0.0, "r": 0.0}
        self.jump_landing_tp_handoff_elapsed = 0.0
        self.jump_landing_tp_command = {"l": 0.0, "r": 0.0}
        self.jump_landing_wheel_command = {"l": 0.0, "r": 0.0}
        # Kept for telemetry/backward compatibility; jump touchdown no longer
        # uses a takeoff-point position correction because it can command a
        # large reverse wheel impulse after landing.
        self.jump_landing_position_x_ratio = 0.0
        self.jump_hip_motor_torque_lpf = {
            "l": np.zeros(2), "r": np.zeros(2)
        }
        # Real-vehicle off-ground port (chassis_safety.c): wheel vertical
        # contact force reconstruction + 4-tap filter + debounced flag.
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
        self.descent_edge_transition_active = False
        self.descent_edge_loss_count = 0
        self.descent_edge_profile_active = False
        self.descent_edge_contact_drive = False
        self.descent_edge_length_reference = {
            "l": self.normal_leg_length,
            "r": self.normal_leg_length,
        }
        self.offground_wheel_torque = 0.0
        self.landing_air_deploy_ref = {
            "l": LANDING_AIR_DEPLOY_LENGTH,
            "r": LANDING_AIR_DEPLOY_LENGTH,
        }
        self.landing_airborne_elapsed = 0.0
        # landing buffer for the generic airborne path (idle take-offs)
        self.gbuf_phase = "inactive"   # inactive/compress/hold/return
        self.gbuf_elapsed = 0.0
        self.gbuf_exit_stable = 0.0
        self.gbuf_length_ref = {"l": VMC_NORMAL_STAND_LENGTH, "r": VMC_NORMAL_STAND_LENGTH}
        self.gbuf_absorb_ref = {"l": VMC_NORMAL_STAND_LENGTH, "r": VMC_NORMAL_STAND_LENGTH}
        self.gbuf_contact_length = {"l": VMC_NORMAL_STAND_LENGTH, "r": VMC_NORMAL_STAND_LENGTH}
        self.gbuf_contact_loss_count = 0
        # Tunable air-attitude allocation.  These are deliberately instance
        # fields so headless sweeps can test the physical controller without
        # rewriting source between every run.
        # In free flight the hips point the legs and the wheel rotors own
        # chassis pitch.  Asking the hip common axis to do both made the two
        # loops fight and drove the motors between +/-40 Nm every control tick.
        self.jump_air_pitch_kp = 0.0
        self.jump_air_pitch_kd = 0.0
        self.jump_air_pitch_limit = 30.0
        self.jump_air_tuck_pitch_bias = np.radians(12.0)
        self.jump_air_deploy_pitch_bias = np.radians(4.0)
        self.jump_air_pitch_reference_limit = np.radians(15.0)
        self.jump_air_moving_pitch_gain = np.radians(3.0)
        self.jump_air_tuck_length = 0.130
        self.jump_air_tuck_time = JUMP_AIR_TUCK_TIME
        self.jump_air_tuck_hold_time = JUMP_AIR_TUCK_HOLD_TIME
        self.jump_air_tuck_ref_rate = JUMP_AIR_TUCK_REF_RATE
        self.jump_air_landing_length = JUMP_AIR_LANDING_LENGTH
        self.jump_air_capture_length_max = JUMP_AIR_CAPTURE_LENGTH_MAX
        self.jump_air_deploy_ref_rate = JUMP_AIR_DEPLOY_REF_RATE
        self.jump_air_deploy_kp = JUMP_AIR_DEPLOY_KP
        self.jump_air_deploy_kd = JUMP_AIR_DEPLOY_KD
        self.jump_air_retract_force_limit = JUMP_AIR_RETRACT_FORCE_LIMIT
        self.jump_air_retract_force_peak = JUMP_AIR_RETRACT_FORCE_PEAK
        self.jump_air_theta_kp = 30.0
        self.jump_air_theta_kd = 4.0
        self.jump_air_theta_limit = 22.0
        self.jump_air_theta_weight_tuck = 0.8
        self.jump_air_theta_weight_deploy = 1.5
        self.jump_air_common_tp_limit = 24.0
        self.jump_air_common_tp_filter_tau = 0.0
        self.jump_air_common_tp_lpf = 0.0
        self.jump_air_rate_filter_tau = 0.0
        self.jump_air_pitch_rate_lpf = 0.0
        self.jump_air_theta_rate_lpf = 0.0
        self.jump_air_wheel_pitch_kp = 35.0
        self.jump_air_wheel_pitch_kd = 5.0
        self.jump_air_wheel_speed_kd = 0.0
        self.jump_air_wheel_speed_damping_start = 0.0
        self.jump_air_wheel_theta_ff = 0.10
        self.jump_air_wheel_sign = 1.0
        self.jump_air_wheel_torque_limit = WHEEL_TORQUE_LIMIT
        self.jump_air_wheel_torque = 0.0
        self.jump_air_wheel_taper_delay = 0.10
        self.jump_air_wheel_taper_time = 0.20
        self.jump_landing_capture_wheel_pitch_kp = 10.0
        self.jump_landing_capture_wheel_pitch_kd = 1.0
        self.jump_landing_capture_wheel_torque_limit = WHEEL_TORQUE_LIMIT
        self.jump_landing_wheel_pitch_kp = 10.0
        self.jump_landing_wheel_pitch_kd = 1.0
        self.jump_landing_wheel_torque_limit = WHEEL_TORQUE_LIMIT
        self.jump_landing_strong_hold_time = 0.0
        self.jump_landing_capture_wheel_slew_rate = 1100.0
        self.jump_landing_wheel_slew_rate = 1000.0
        self.jump_landing_theta_kp = 30.0
        self.jump_landing_theta_kd = 6.0
        self.jump_landing_theta_limit = 30.0
        self.jump_abort_pitch_limit = JUMP_ABORT_PITCH_MAX
        self.jump_abort_roll_limit = JUMP_ABORT_ROLL_MAX
        self.jump_thrust_pitch_kp = JUMP_THRUST_PITCH_KP
        self.jump_thrust_pitch_kd = JUMP_THRUST_PITCH_KD
        self.jump_thrust_pitch_limit = JUMP_THRUST_PITCH_TP_LIMIT
        self.jump_thrust_pitch_rate_target = JUMP_THRUST_PITCH_RATE_TARGET_ZERO
        self.jump_thrust_pitch_rate_min = JUMP_THRUST_PITCH_RATE_TARGET_MIN
        self.jump_thrust_pitch_rate_max = JUMP_THRUST_PITCH_RATE_TARGET_MAX
        self.jump_thrust_pitch_rate_start_l0 = (
            JUMP_THRUST_PITCH_RATE_START_L0
        )
        self.jump_thrust_pitch_rate_full_l0 = JUMP_THRUST_PITCH_RATE_FULL_L0
        self.jump_thrust_pitch_tp_slew_rate = JUMP_THRUST_PITCH_TP_SLEW_RATE
        self.jump_thrust_theta_kp = JUMP_THRUST_THETA_HOLD_KP
        self.jump_thrust_theta_kd = JUMP_THRUST_THETA_HOLD_KD
        self.jump_thrust_theta_limit = JUMP_THRUST_THETA_HOLD_LIMIT
        self.landing_buffer_phase = "inactive"
        self.landing_buffer_phase_elapsed = 0.0
        self.landing_buffer_elapsed = 0.0
        self.landing_exit_stable_elapsed = 0.0
        self.landing_length_reference = {"l": VMC_NORMAL_STAND_LENGTH, "r": VMC_NORMAL_STAND_LENGTH}
        self.landing_support_handoff = {"l": 0.0, "r": 0.0}
        # Identified-parameter overrides (see balance_lab.sysid and
        # docs/SYSID.md).  They default to the firmware constants so a stock
        # controller is bit-identical to the pre-sysid behaviour;
        # apply_identified() replaces them with real-vehicle measurements.
        # reset() deliberately does not touch them: an identified profile
        # survives mode switches and recovery resets.
        self.gas_spring_force = GAS_SPRING_FORCE
        self.theta_comp_poly = (
            THETA_COMP_POLY_C0, THETA_COMP_POLY_C1, THETA_COMP_POLY_C2,
        )
        self.theta_comp_center_m = THETA_COMP_POLY_L0_CENTER
        self.theta_comp_scale_m = THETA_COMP_POLY_L0_SCALE
        self.theta_dx_offset_m = THETA_DX_OFFSET_M

    def _body(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)

    def _joint(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)

    def _actuator(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)

    def _site(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, name)

    def _geom(self, name: str) -> int:
        return mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, name)

    def _heading(self, data: mujoco.MjData) -> tuple[np.ndarray, float]:
        """Return the current ground-plane heading and yaw from reset."""
        if self.forward_body is None or self.forward_world is None:
            raise ValueError("Controller frame has not been calibrated")
        up, forward_zero, lateral_zero = self._frame(data)
        rotation = data.xmat[self.base].reshape(3, 3)
        heading = rotation @ self.forward_body
        heading -= up * np.dot(heading, up)
        if np.linalg.norm(heading) < 1e-8:
            heading = forward_zero.copy()
        else:
            heading = unit(heading)
        yaw = float(np.arctan2(
            np.dot(heading, lateral_zero),
            np.dot(heading, forward_zero),
        ))
        return heading, yaw

    def set_drive_command(self, forward_mps: float, yaw_rate_rad_s: float) -> None:
        """Set persistent keyboard-style velocity commands.

        ``forward_mps`` is positive toward the visible front of the chassis.
        The frame translation for this CAD export lives where chassis_vx is
        measured in ``step()``; ``_state`` consumes both terms in that same
        sign convention.
        """
        self.drive_forward_target = float(np.clip(
            forward_mps, -DRIVE_SPEED_MAX, DRIVE_SPEED_MAX
        ))
        self.drive_yaw_rate_target = float(np.clip(
            yaw_rate_rad_s, -DRIVE_YAW_RATE_MAX, DRIVE_YAW_RATE_MAX
        ))

    def set_leg_length(self, leg_length: float) -> float:
        """Select a supported standing length for the interactive controller.

        The normal-ground VMC reference slews toward this target in
        ``_apply_vmc_support``.  Deferring that slew keeps a live length
        button from generating an instantaneous axial-force step.
        """
        requested = float(leg_length)
        if not np.isfinite(requested):
            raise ValueError("leg length must be finite")
        selected = min(LEG_LENGTH_PRESETS, key=lambda value: abs(value - requested))
        if abs(selected - requested) > 1e-6:
            raise ValueError(
                "unsupported leg length "
                f"{requested:.6f}; choose one of {LEG_LENGTH_PRESETS}"
            )
        self.normal_leg_length = float(selected)
        if not self.descent_guard_active and not self.descent_edge_profile_active:
            for side in ("l", "r"):
                self.descent_edge_length_reference[side] = self.normal_leg_length
        return self.normal_leg_length

    def _drive_target_for_control(self) -> float:
        """Return the velocity reference currently allowed by safety logic."""
        if self.descent_guard_active:
            # Before the wheels leave the edge, reduce the commanded speed
            # instead of making the velocity reference jump to zero.  Once a
            # real takeoff has been observed, the guard owns the wheel brake
            # and the reference becomes zero for the landing sequence.
            if not self.descent_guard_drop_seen:
                return float(np.clip(
                    self.drive_forward_target,
                    -DESCENT_MODE_SPEED_LIMIT,
                    DESCENT_MODE_SPEED_LIMIT,
                ))
            return 0.0
        if self.descent_mode_active:
            return float(np.clip(
                self.drive_forward_target,
                -DESCENT_MODE_SPEED_LIMIT,
                DESCENT_MODE_SPEED_LIMIT,
            ))
        return self.drive_forward_target

    def stop_drive(self) -> None:
        self.set_drive_command(0.0, 0.0)
        # A completed ledge descent is intentionally latched safe.  An
        # explicit stop is the operator acknowledgement that it is safe to
        # resume ordinary ground driving.
        self.descent_guard_active = False
        self.descent_mode_active = False
        self.descent_guard_drop_seen = False
        self.descent_guard_preedge_elapsed = 0.0
        self.descent_edge_transition_active = False
        self.descent_edge_loss_count = 0
        self.descent_edge_profile_active = False
        self.descent_edge_contact_drive = False
        self.descent_edge_length_reference = {
            "l": self.normal_leg_length,
            "r": self.normal_leg_length,
        }

    def _wheel_contacts(self, data: mujoco.MjData) -> dict[str, bool]:
        touching = {"l": False, "r": False}
        wheel_side = {ids.wheel_body: side for side, ids in self.sides.items()}
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            contact_geoms = {int(contact.geom1), int(contact.geom2)}
            if not contact_geoms.intersection(self.ground_geoms):
                continue
            other_geom = next(
                geom for geom in contact_geoms
                if geom not in self.ground_geoms
            )
            side = wheel_side.get(int(self.model.geom_bodyid[other_geom]))
            if side is not None:
                touching[side] = True
        return touching

    def _wheel_support_contacts(self, data: mujoco.MjData) -> dict[str, bool]:
        """Return wheel contacts whose normal can carry chassis weight.

        A wheel touching a stair riser is a collision, not a landing.  The
        first row of MuJoCo's contact frame is the contact normal; requiring
        a substantial vertical component prevents a side scrape from being
        promoted to bilateral support.
        """
        supporting = {"l": False, "r": False}
        wheel_side = {ids.wheel_body: side for side, ids in self.sides.items()}
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            contact_geoms = {int(contact.geom1), int(contact.geom2)}
            if not contact_geoms.intersection(self.ground_geoms):
                continue
            other_geom = next(
                geom for geom in contact_geoms
                if geom not in self.ground_geoms
            )
            side = wheel_side.get(int(self.model.geom_bodyid[other_geom]))
            if side is None:
                continue
            normal = np.asarray(contact.frame, dtype=float).reshape(3, 3)[0]
            if abs(float(normal[2])) >= 0.65:
                supporting[side] = True
        return supporting

    def is_properly_standing(self, data: mujoco.MjData) -> bool:
        """True ONLY for a real stance: wheels bearing load, legs down near
        the calibrated geometry, chassis at standing height, body level.

        Pitch/roll alone are NOT acceptance criteria -- a chassis lying flat
        with the legs folded skyward also reads pitch=roll=0.
        """
        grounded = self._wheel_support_contacts(data)
        if not all(grounded.values()):
            return False
        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            if abs(np.degrees(kin.phi0) - 90.0) > 15.0:
                return False
            if kin.l0 > 0.22:
                return False
        if float(data.xpos[self.base][2]) < 0.19:
            return False
        return (
            abs(float(np.degrees(self.body_pitch(data)))) < 6.0
            and abs(float(np.degrees(self.body_roll(data)))) < 6.0
        )

    def reset(self, data: mujoco.MjData, preserve_recovery: bool = False, calibrate_frame: bool = False) -> None:
        mujoco.mj_forward(self.model, data)
        self.base_position0 = data.xpos[self.base].copy()
        up = np.array((0.0, 0.0, 1.0))
        # The export coordinate frame has the chassis yawed by about 45 deg,
        # and CAD q=0 is the virtual-leg vertical pose.  Therefore the
        # horizontal hip-to-wheel residual is neither a chassis axis nor a
        # numerically stable way to discover one.  Use the gimbal mounting
        # point instead: it lies on the vehicle longitudinal centreline and
        # faces the front.  Capture it in base coordinates so the reference
        # remains fixed while the free base moves and pitches.
        if self.forward_body is None or calibrate_frame:
            base_rotation = data.xmat[self.base].reshape(3, 3)
            gimbal_offset = data.xpos[self.gimbal_body] - data.xpos[self.base]
            # Body Y is the CAD-up direction after the fixed CAD-to-Z-up
            # rotation.  Project in base coordinates to discard the gimbal's
            # vertical elevation before normalising the longitudinal vector.
            gimbal_body = base_rotation.T @ gimbal_offset
            gimbal_body[1] = 0.0
            self.forward_body = unit(gimbal_body)
            forward_world = base_rotation @ self.forward_body
            forward_world -= up * np.dot(forward_world, up)
            forward_world = unit(forward_world)
            self.forward_world = forward_world
            self.forward_body = base_rotation.T @ forward_world
            # Store the complete calibrated vehicle frame in base-body
            # coordinates.  Reading gravity in this frame reproduces an IMU
            # roll/pitch measurement and, unlike projection onto fixed world
            # forward/lateral axes, cannot mix chassis yaw into tilt.
            self.up_body = unit(base_rotation.T @ up)
            # Vehicle-positive lateral points toward the physical left leg.
            # With this CAD assembly that is forward x up (the opposite of
            # ``_frame``'s historical wheel/pitch axis).
            self.lateral_body = unit(np.cross(self.forward_body, self.up_body))
        self.previous = {
            "x": 0.0,
            "pitch": self.body_pitch(data),
            "roll": self.body_roll(data),
        }
        self.previous_base_position = data.xpos[self.base].copy()
        _, self.previous_heading = self._heading(data)
        self.chassis_vx = 0.0
        self.offground_suppress_elapsed = 0.0
        self.cruise_integral = 0.0
        self.danger_reason = "none"
        self.danger_confirmed = False
        self.yaw_rate = 0.0
        self.yaw_output = 0.0
        self.yaw_integral = 0.0
        self.stop_drive()
        self._reset_jump_runtime(clear_cooldown=True)
        self.control_elapsed = 0.0
        if not preserve_recovery:
            self.cancel_recovery()
        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            # Exact drv_leg.c definition: theta = pi/2 - phi0 + ins.pitch.
            theta = np.pi / 2.0 - kin.phi0 + self.body_pitch(data)
            length = kin.l0
            self.previous[f"theta_{side}"] = theta
            self.previous[f"length_{side}"] = length
            self.previous[f"phi_{side}"] = kin.phi0
            self.length_reference[side] = length
            self.length_integral[side] = 0.0
            self.theta_rate_lpf[side] = 0.0
            self.length_rate_lpf[side] = 0.0

    def enable_cad_zero_lqr(self, data: mujoco.MjData) -> None:
        """Arm the legacy standing experiment only from CAD joint zero.

        The MJCF joint coordinates are geometric CAD coordinates, not the
        hardware's multi-turn encoder readings.  Refusing a nonzero start
        prevents an encoder offset or a saved recovery pose from silently
        becoming the reference.  The interactive UI deliberately locks this
        route until a new equilibrium is built at calibrated l0=0.150 m.
        """
        nonzero: list[str] = []
        for joint in range(self.model.njnt):
            if self.model.jnt_type[joint] == mujoco.mjtJoint.mjJNT_FREE:
                continue
            value = float(data.qpos[self.model.jnt_qposadr[joint]])
            if abs(value) > CAD_ZERO_JOINT_TOLERANCE_RAD:
                name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint)
                nonzero.append(f"{name}={value:+.9f}")
        if nonzero:
            raise ValueError("LQR CAD-zero start rejected: " + ", ".join(nonzero))
        if abs(self.body_pitch(data)) >= PRONE_ENTRY_PITCH:
            raise ValueError("LQR CAD-zero start rejected: base is not upright")
        self._start_cad_stand(data, enable_lqr=True)

    def enable_calibrated_lqr(self, data: mujoco.MjData) -> None:
        """Arm the firmware six-state LQR at the calibrated N equilibrium."""
        self.reset(data)
        errors: list[str] = []
        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            if abs(kin.l0 - VMC_NORMAL_STAND_LENGTH) > 2e-5:
                errors.append(f"{side}.l0={kin.l0:.9f}")
            if abs(self._wrap_pi(kin.phi0 - np.pi / 2.0)) > np.radians(0.02):
                errors.append(f"{side}.phi0={np.degrees(kin.phi0):.6f}deg")
        pitch = self.body_pitch(data)
        if abs(pitch) > np.radians(0.02):
            errors.append(f"pitch={np.degrees(pitch):.6f}deg")
        if errors:
            raise ValueError("Calibrated LQR start rejected: " + ", ".join(errors))

        self.enabled = True
        self.stance_enabled = True
        self.gas_enabled = True
        self.gimbal_locked = True
        self.lqr_leg_level = "low"
        self.lqr_gain = 1.0
        self.recovery_state = "idle"
        self.control_elapsed = 0.0
        self.base_position0 = data.xpos[self.base].copy()
        self.last_state = {"l": np.zeros(6), "r": np.zeros(6)}
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_support = {"l": 0.0, "r": 0.0}
        self.last_motor_torque = {"l": np.zeros(2), "r": np.zeros(2)}
        self.roll_force = 0.0
        self.roll_rate_lpf_deg_s = 0.0
        self.lateral_inertial_ff = 0.0
        self.roll_static_trim_force = 0.0
        self.roll_static_trim_applied = 0.0
        self.roll_static_trim_timer = 0.0
        self.length_integral = {"l": 0.0, "r": 0.0}
        self.theta_rate_lpf = {"l": 0.0, "r": 0.0}
        self.length_rate_lpf = {"l": 0.0, "r": 0.0}
        for side in ("l", "r"):
            self.length_reference[side] = VMC_NORMAL_STAND_LENGTH

    def apply_identified(self, params: "ModelParameters") -> dict[str, float]:
        """Apply the controller-side parameters of an identified profile.

        Only what the controller itself consumes is read here: gas-spring
        force and the theta-compensation fit.  Plant-side parameters
        (actuator strength, joint damping/friction/inertia, contact
        friction) belong to the MuJoCo model and are applied by
        :func:`balance_lab.sysid.apply.apply_model_parameters`.  See
        docs/SYSID.md for the full parameter -> consumer mapping.
        """
        applied: dict[str, float] = {}
        force = params.number("gas_springs", "force_n")
        scale = params.number("gas_springs", "scale")
        if force <= 0.0:
            raise ValueError(f"gas_springs.force_n must be positive, got {force}")
        self.gas_spring_force = force * scale
        applied["gas_spring_force_n"] = self.gas_spring_force

        convention = str(params.section("theta_compensation").get("convention", ""))
        if convention and convention != "theta_corrected_equals_theta_raw_minus_compensation":
            raise ValueError(f"unsupported theta_compensation convention: {convention!r}")
        poly = params.numbers("theta_compensation", "polynomial_rad")
        if len(poly) != 3:
            raise ValueError(
                f"theta_compensation.polynomial_rad must have 3 coefficients, got {len(poly)}"
            )
        left = params.number("theta_compensation", "dx_offset_left_m")
        right = params.number("theta_compensation", "dx_offset_right_m")
        if abs(left - right) > 1e-9:
            raise ValueError(
                "this controller applies one shared theta dx offset; the profile's "
                f"asymmetric offsets (l={left}, r={right}) are not representable"
            )
        self.theta_comp_poly = (poly[0], poly[1], poly[2])
        self.theta_comp_center_m = params.number("theta_compensation", "length_center_m")
        self.theta_comp_scale_m = params.number("theta_compensation", "length_scale_m")
        self.theta_dx_offset_m = left
        applied["theta_comp_center_m"] = self.theta_comp_center_m
        applied["theta_comp_scale_m"] = self.theta_comp_scale_m
        applied["theta_dx_offset_m"] = self.theta_dx_offset_m
        return applied

    def gain(self, leg_length: float) -> np.ndarray:
        powers = np.array((leg_length ** 3, leg_length ** 2, leg_length, 1.0))
        coefficients = LOW_COEFFICIENTS if self.lqr_leg_level == "low" else MID_COEFFICIENTS
        return coefficients @ powers

    def _frame(self, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        up = np.array((0.0, 0.0, 1.0))
        if self.forward_world is None:
            raise ValueError("Controller frame has not been calibrated")
        # Keep the vehicle longitudinal reference fixed in the world after
        # the CAD-zero calibration.  Reprojecting a body-fixed axis works
        # near upright but becomes singular when the chassis is prone (the
        # exact recovery state that still needs an unambiguous pitch sign).
        forward = self.forward_world
        lateral = unit(np.cross(up, forward))
        return up, forward, lateral

    @staticmethod
    def _wrap_pi(value: float) -> float:
        return float((value + np.pi) % (2.0 * np.pi) - np.pi)

    @staticmethod
    def _wrap_2pi(value: float) -> float:
        return float(value % (2.0 * np.pi))

    def vmc_kinematics(self, data: mujoco.MjData, side: str) -> VmcKinematics:
        """Exact calibrated port of ``drv_leg.c::vmc_calc`` and its Jacobian.

        The firmware's raw encoder zero is the kinematic anchor.  The user's
        three measured poses establish its CAD counterpart and reveal that
        the exported CAD names are crossed relative to the firmware arms:
        CAD J2 is phi1/FRONT and CAD J1 is phi4/BACK.  The left raw angles
        enter ``drv_leg.c`` with the opposite sign.  Keeping those facts here
        makes l0/phi0 and the virtual-work motor Jacobian one consistent map.
        """
        ids = self.sides[side]
        q_j1 = float(data.qpos[self.model.jnt_qposadr[ids.j1]])
        q_j2 = float(data.qpos[self.model.jnt_qposadr[ids.j2]])
        q0_j1 = VMC_ENCODER_ZERO_CAD_Q[f"{side}_j1"]
        q0_j2 = VMC_ENCODER_ZERO_CAD_Q[f"{side}_j2"]
        raw_sign = -1.0 if side == "l" else 1.0
        phi1 = self._wrap_2pi(VMC_DM_ZERO_PHI1 + raw_sign * (q_j2 - q0_j2))
        phi4 = self._wrap_2pi(VMC_DM_ZERO_PHI4 + raw_sign * (q_j1 - q0_j1))

        xb, yb = VMC_L1 * np.cos(phi1), VMC_L1 * np.sin(phi1)
        xd, yd = VMC_L1 * np.cos(phi4), VMC_L1 * np.sin(phi4)
        dx, dy = xd - xb, yd - yb
        a0, b0 = 2.0 * VMC_L2 * dx, 2.0 * VMC_L2 * dy
        c0 = dx * dx + dy * dy
        discriminant = max(0.0, a0 * a0 + b0 * b0 - c0 * c0)
        phi2 = 2.0 * np.arctan2(b0 + np.sqrt(discriminant), a0 + c0)
        xc, yc = xb + VMC_L2 * np.cos(phi2), yb + VMC_L2 * np.sin(phi2)
        phi3 = np.arctan2((yb - yd) + VMC_L2 * np.sin(phi2), (xb - xd) + VMC_L2 * np.cos(phi2))
        l0 = max(float(np.hypot(xc, yc)), 1e-5)
        phi0 = float(np.arctan2(yc, xc))
        sin32 = np.sin(phi3 - phi2)
        if abs(sin32) < 1e-5:
            raise ValueError(f"VMC four-bar singularity on {side} leg")
        q00 = VMC_L1 * np.sin(phi0 - phi3) * np.sin(phi1 - phi2) / sin32
        q01 = VMC_L1 * np.cos(phi0 - phi3) * np.sin(phi1 - phi2) / (l0 * sin32)
        q10 = VMC_L1 * np.sin(phi0 - phi2) * np.sin(phi3 - phi4) / sin32
        q11 = VMC_L1 * np.cos(phi0 - phi2) * np.sin(phi3 - phi4) / (l0 * sin32)
        # drv_leg.c rows are [phi1 motor, phi4 motor].  CAD actuator order is
        # [J1=phi4, J2=phi1], and the left raw encoder derivatives are -1.
        firmware_jacobian = np.array(((q00, q01), (q10, q11)))
        actuator_jacobian = raw_sign * firmware_jacobian[[1, 0], :]
        return VmcKinematics(
            l0,
            phi0,
            float(phi1),
            float(phi2),
            float(phi3),
            float(phi4),
            actuator_jacobian,
        )

    def set_vmc_motor_torques(
        self,
        data: mujoco.MjData,
        side: str,
        support_force: float,
        tp: float,
        torque_limit: float = HIP_TORQUE_LIMIT,
    ) -> np.ndarray:
        """Map firmware [F0, Tp] to the two real MJCF drive-motor torques."""
        ids = self.sides[side]
        torque = self.vmc_kinematics(data, side).jacobian @ np.array((support_force, tp))
        torque = np.clip(torque, -torque_limit, torque_limit)
        if self.jump_off_ground and self.jump_hip_motor_filter_tau > 0.0:
            dt = float(self.model.opt.timestep)
            alpha = float(np.clip(
                dt / (self.jump_hip_motor_filter_tau + dt), 0.0, 1.0
            ))
            self.jump_hip_motor_torque_lpf[side] += alpha * (
                torque - self.jump_hip_motor_torque_lpf[side]
            )
            torque = self.jump_hip_motor_torque_lpf[side].copy()
        else:
            self.jump_hip_motor_torque_lpf[side] = torque.copy()
        data.ctrl[ids.hip_motors[0]] = torque[0]
        data.ctrl[ids.hip_motors[1]] = torque[1]
        self.last_motor_torque[side] = torque
        return torque

    def _leg_measurement(self, data: mujoco.MjData, ids: SideIds) -> tuple[float, float]:
        up, forward, _ = self._frame(data)
        hip = data.xanchor[ids.j1]
        wheel = data.xpos[ids.wheel_body]
        leg = wheel - hip
        length = float(np.linalg.norm(leg))
        direction = leg / max(length, 1e-9)
        theta_world = float(np.arctan2(np.dot(direction, forward), -np.dot(direction, up)))
        return theta_world, length

    def body_pitch(self, data: mujoco.MjData) -> float:
        """Yaw-invariant calibrated pitch: backward lean is positive."""
        up, _, _ = self._frame(data)
        if self.forward_body is None or self.up_body is None:
            raise ValueError("Controller body frame has not been calibrated")
        base_rotation = data.xmat[self.base].reshape(3, 3)
        current_forward = base_rotation @ self.forward_body
        current_up = base_rotation @ self.up_body
        raw_pitch = float(np.arctan2(
            -np.dot(up, current_forward),
            np.dot(up, current_up),
        ))
        return PITCH_BACKWARD_SIGN * raw_pitch

    def body_pitch_rate(self, data: mujoco.MjData) -> float:
        """Return the calibrated backward-positive pitch rate from IMU omega.

        The finite-difference pitch used by the normal LQR is intentionally
        retained for its discrete observer.  It is a poor derivative source
        for the ledge handoff, though: a one-step contact impulse appears as a
        large alternating rate and makes the common hip command chatter.  The
        free-joint angular velocity is the MuJoCo equivalent of the IMU gyro;
        projecting it onto the current horizontal lateral axis avoids that
        contact-difference amplification while preserving the calibrated
        backward-positive sign.
        """
        up = np.array((0.0, 0.0, 1.0))
        heading, _ = self._heading(data)
        lateral = unit(np.cross(up, heading))
        return float(-np.dot(data.qvel[3:6], lateral))

    def body_roll(self, data: mujoco.MjData) -> float:
        """IMU-style roll in radians, zeroed by the calibrated CAD frame."""
        up, _, _ = self._frame(data)
        if self.lateral_body is None or self.up_body is None:
            raise ValueError("Controller body frame has not been calibrated")
        base_rotation = data.xmat[self.base].reshape(3, 3)
        current_lateral = base_rotation @ self.lateral_body
        current_up = base_rotation @ self.up_body
        return float(np.arctan2(np.dot(up, current_lateral), np.dot(up, current_up)))

    def _state(self, data: mujoco.MjData, side: str, dt: float) -> tuple[np.ndarray, float]:
        kin = self.vmc_kinematics(data, side)
        length = kin.l0
        pitch = self.body_pitch(data)
        # This is deliberately not reconstructed from CAD points.  The
        # embedded controller defines theta in the VMC frame, so using the
        # same phi0 expression preserves both its zero and feedback polarity.
        theta = np.pi / 2.0 - kin.phi0 + pitch
        theta_observed = theta - self._theta_compensation(data, length)
        theta_dot_raw = self._wrap_pi(theta - self.previous[f"theta_{side}"]) / dt
        # drv_leg.c::One_Older_LPF(..., 0.5f), used directly by LQR obs[1].
        self.theta_rate_lpf[side] = (
            VMC_RATE_LPF_RETAIN * self.theta_rate_lpf[side]
            + (1.0 - VMC_RATE_LPF_RETAIN) * theta_dot_raw
        )
        theta_dot = self.theta_rate_lpf[side]
        x = self.lqr_virtual_x
        # ``step()`` forms chassis_vx as -(displacement . heading): after that
        # single frame translation a positive value means motion toward the
        # visible front, matching the UI sign of set_drive_command.  The
        # velocity reference therefore enters directly.  Negating it here a
        # second time reverses every drive command: a +0.40 m/s request then
        # tracks tail-ward at ~0.48 m/s and fails the drive regression.
        if DRIVE_CRUISE_ZERO_REF and (
            abs(self.drive_forward_target) >= DRIVE_CRUISE_MIN_COMMAND
        ):
            # Cruise handoff: see DRIVE_CRUISE_ZERO_REF note above the
            # constants.  Keeping a raw nonzero reference here made the
            # firmware velocity row behave as a slow runaway under any
            # command; a zero reference turns it into a plain mild damper.
            firmware_vx_reference = 0.0
        else:
            firmware_vx_reference = self._drive_target_for_control()
            # chassis_control.c LQRXRefBuf clamp: the drive ceiling follows
            # the live leg geometry (HIGH 1.20 / MID 1.85 / LOW 2.30 m/s).
            # Without it the sim drives long legs past the firmware's own
            # envelope and the theta loop diverges exactly as the real
            # engineers predicted when they set these caps.
            avg_length = 0.5 * sum(
                self.vmc_kinematics(data, side).l0 for side in ("l", "r")
            )
            if avg_length >= CHASSIS_V_HIGH_LEG_L0:
                vx_limit = CHASSIS_V_MAX_HIGH
            elif avg_length >= CHASSIS_V_MID_LEG_L0:
                vx_limit = CHASSIS_V_MAX_MID
            else:
                vx_limit = CHASSIS_V_MAX_LOW
            firmware_vx_reference = float(np.clip(
                firmware_vx_reference, -vx_limit, vx_limit
            ))
        # Calibration switch: see LQR_XDOT_SIGN note above the constants.
        x_state = LQR_XDOT_SIGN * (self.chassis_vx - firmware_vx_reference)
        pitch_dot = (pitch - self.previous["pitch"]) / dt
        # Firmware LQR uses obs[4:6] = [-ins.pitch, -ins.pitch_gyro].
        state = np.array((theta_observed, theta_dot, x, x_state, -pitch, -pitch_dot))
        return state, length

    def _theta_compensation(self, data: mujoco.MjData, length: float) -> float:
        """Static geometry + fitted residual copied from chassis_control.c."""
        safe_length = max(length, 0.05)
        average_length = 0.5 * sum(
            self.vmc_kinematics(data, side).l0 for side in ("l", "r")
        )
        fitted_length = float(np.clip(
            average_length,
            THETA_COMP_POLY_L0_MIN,
            THETA_COMP_POLY_L0_MAX,
        ))
        normalized = (
            fitted_length - self.theta_comp_center_m
        ) / self.theta_comp_scale_m
        poly_c0, poly_c1, poly_c2 = self.theta_comp_poly
        polynomial = poly_c0 + normalized * (poly_c1 + normalized * poly_c2)
        geometry = float(np.arcsin(np.clip(
            self.theta_dx_offset_m / safe_length,
            -1.0,
            1.0,
        )))
        return geometry + self.theta_comp_apply_gain * polynomial

    @staticmethod
    def _move_towards(value: float, target: float, max_step: float) -> float:
        return value + float(np.clip(target - value, -max_step, max_step))

    def _update_lqr_position_observer(self, chassis_vx: float, dt: float) -> None:
        """Port the firmware's zero-command DRIVE/BRAKE/HOLD observer."""
        x_ratio = LQR_POSITION_X_RATIO
        if self.lqr_position_state == "drive":
            self.lqr_position_state = "brake"
            self.lqr_position_capture_timer = 0.0
            self.lqr_position_hold_x = 0.0
            self.lqr_position_brake_offset = 0.0

        if self.lqr_position_state == "brake":
            self.lqr_virtual_x = float(np.clip(
                self.lqr_virtual_x + chassis_vx * dt * x_ratio,
                -LQR_POSITION_LIMIT,
                LQR_POSITION_LIMIT,
            ))
            if abs(chassis_vx) < LQR_POSITION_CAPTURE_VX_MAX:
                self.lqr_position_capture_timer += dt
                if self.lqr_position_capture_timer >= LQR_POSITION_CAPTURE_TIME:
                    self.lqr_position_state = "hold"
                    self.lqr_position_capture_timer = LQR_POSITION_CAPTURE_TIME
                    self.lqr_position_brake_offset = self.lqr_virtual_x
                    self.lqr_position_hold_x = 0.0
            else:
                self.lqr_position_capture_timer = 0.0
            return

        self.lqr_position_brake_offset = self._move_towards(
            self.lqr_position_brake_offset,
            0.0,
            LQR_POSITION_BRAKE_RELEASE_RATE * dt,
        )
        self.lqr_position_hold_x = float(np.clip(
            self.lqr_position_hold_x + chassis_vx * dt * x_ratio,
            -LQR_POSITION_LIMIT,
            LQR_POSITION_LIMIT,
        ))
        self.lqr_virtual_x = float(np.clip(
            self.lqr_position_hold_x + self.lqr_position_brake_offset,
            -LQR_POSITION_LIMIT,
            LQR_POSITION_LIMIT,
        ))

    def _apply_gas_springs(self, data: mujoco.MjData) -> None:
        data.qfrc_applied[:] = 0.0
        if not self.gas_enabled:
            return
        for ids in self.sides.values():
            thigh = data.site_xpos[ids.gas_thigh_site]
            leg = data.site_xpos[ids.gas_leg_site]
            direction = unit(leg - thigh)
            thigh_body = int(self.model.site_bodyid[ids.gas_thigh_site])
            leg_body = int(self.model.site_bodyid[ids.gas_leg_site])
            # Green-polarity direction verified in the assembled visual model.
            mujoco.mj_applyFT(self.model, data, -self.gas_spring_force * direction, np.zeros(3), thigh, thigh_body, data.qfrc_applied)
            mujoco.mj_applyFT(self.model, data, self.gas_spring_force * direction, np.zeros(3), leg, leg_body, data.qfrc_applied)

    def _gas_virtual_support(self, leg_length: float) -> float:
        """Gas-spring force expressed in the firmware's virtual-leg l0 axis."""
        force = self.gas_spring_force
        denominator = 2.0 * VMC_L1 * VMC_L2
        cosine = np.clip((VMC_L1**2 + VMC_L2**2 - leg_length**2) / denominator, -1.0, 1.0)
        theta3 = float(np.arccos(cosine))
        sine3 = max(float(np.sin(theta3)), 1e-5)
        ls = float(np.sqrt(GAS_L6**2 + GAS_LS2**2 - 2.0 * GAS_L6 * GAS_LS2 * np.cos(theta3 - GAS_ALPHA)))
        return force * leg_length * GAS_L6 * GAS_LS2 * np.sin(theta3 - GAS_ALPHA) / (ls * VMC_L1 * VMC_L2 * sine3)

    def _apply_drive_speed_trim(self, dt: float) -> None:
        """Add a bounded normal-ground speed trim outside the LQR matrix.

        The firmware's six-state gain is intentionally retained unchanged;
        this trim is a separate, low-authority common wheel channel so speed
        limiting does not increase the LQR x-dot gain and destabilize recovery.
        It is disabled for recovery, jump, airborne and landing-buffer phases.
        """
        if self.descent_guard_active:
            if self.descent_edge_transition_active:
                # The first wheel has reached a vertical tread while the
                # other is still crossing the riser.  Stop both the normal
                # speed trim and the wheel LQR for this contact-mode change;
                # leaving either active makes the supported leg chase a
                # transient contact point and creates the sudden skew seen at
                # the edge.  The VMC axial/hip support remains active below.
                self.drive_speed_trim = self._move_towards(
                    self.drive_speed_trim, 0.0, DRIVE_SPEED_TRIM_SLEW * dt
                )
                return
            # The user command remains latched so the vehicle can continue
            # after the landing has settled, but the LQR reference is held at
            # zero around the drop.  Before the wheels leave the edge, use the
            # calibrated common braking direction to remove residual momentum.
            # The sign is specific to the assembled wheel motor polarity and
            # has been checked against the flat-ground drive response.
            if not self.descent_guard_drop_seen and self.gbuf_phase == "inactive":
                # Roll across the edge at a deliberately low speed.  A zero
                # reference is not enough here: it makes the LQR stop with
                # the front wheel against the riser, while a full drive
                # command can carry too much horizontal momentum into the
                # drop.  The trim is common-mode and removed immediately
                # once the force/contact detector confirms takeoff.
                # Same negated firmware-frame sense as the main trim above.
                ui_speed = -self.chassis_vx
                creep_target = float(np.clip(
                    self.drive_forward_target,
                    -DESCENT_EDGE_CREEP_SPEED,
                    DESCENT_EDGE_CREEP_SPEED,
                ))
                error = creep_target - ui_speed
                if abs(error) <= DRIVE_SPEED_DEADBAND:
                    error = 0.0
                target = float(np.clip(
                    DRIVE_SPEED_KP * error,
                    -DESCENT_EDGE_CREEP_TRIM_LIMIT,
                    DESCENT_EDGE_CREEP_TRIM_LIMIT,
                ))
                self.drive_speed_trim = self._move_towards(
                    self.drive_speed_trim,
                    target,
                    DRIVE_SPEED_TRIM_SLEW * dt,
                )
                self.last_output["l"][0] += self.drive_speed_trim
                self.last_output["r"][0] += self.drive_speed_trim
            else:
                self.drive_speed_trim = self._move_towards(
                    self.drive_speed_trim, 0.0, DRIVE_SPEED_TRIM_SLEW * dt
                )
            if self.gbuf_phase != "inactive":
                # The LOW landing buffer is a vertical impact absorber.  Its
                # normal LQR wheel row can turn the captured contact into a
                # forward impulse while the leg is still shortening; keep
                # the wheel row passive until the buffer's velocity damper has
                # handed support back.
                self.last_output["l"][0] = 0.0
                self.last_output["r"][0] = 0.0
            # Before takeoff, _drive_target_for_control() already limits the
            # normal LQR reference to the slow edge-crossing speed.  Do not
            # add a full reverse torque here: that stops at the riser instead
            # of allowing the wheel to roll over it.
            return

        enabled = (
            self.recovery_state == "idle"
            and self.jump_state == "idle"
            and not self.jump_off_ground
            and not self.off_ground
            and self.gbuf_phase == "inactive"
            and self.landing_buffer_phase == "inactive"
            and self.enabled
        )
        if not enabled:
            self.drive_speed_trim = self._move_towards(
                self.drive_speed_trim, 0.0, DRIVE_SPEED_TRIM_SLEW * dt
            )
            return
        cmd = self.drive_forward_target
        requested = abs(cmd)
        if (
            requested < DRIVE_CRUISE_MIN_COMMAND
            or cmd < 0.0  # reverse commands: see the velocity-row note below
            or abs(self.chassis_vx) > DRIVE_CRUISE_VX_BYPASS
        ):
            # Zero/near-zero commands stay entirely with the six-state LQR:
            # the pure velocity-error row holds a stand solid, and this
            # governor only exists because that same row alone turns every
            # nonzero request into an unbounded slow runaway (a 0.2 m/s
            # command reached ~1 m/s after nine seconds with no cruise).
            self.drive_speed_trim = self._move_towards(
                self.drive_speed_trim, 0.0, DRIVE_CRUISE_SLEW * dt
            )
            self.cruise_integral = 0.0
            return
        # Achieved speed resolved along the commanded direction.  chassis_vx
        # is UI-front-positive (see _state), while a POSITIVE common wheel
        # output drives the vehicle BACKWARD in this CAD export -- measured
        # tonight: a railed +1.2 Nm trim settled a +0.5 m/s request at
        # -0.5 m/s.  The error therefore keeps the natural sense here and
        # the application below negates the trim, closing the loop with
        # negative gain.  (Resolves the boundary-sign question that kept
        # this governor disarmed; the descent-edge creep branch above is
        # independently self-consistent with its negated ui_speed measure.)
        sign = np.sign(cmd)
        achieved = sign * self.chassis_vx
        error = requested - achieved
        if abs(error) <= DRIVE_SPEED_DEADBAND:
            error = 0.0
        # Anti-windup: freeze the integral while either wheel already sits at
        # its torque limit.  Integrating against a saturated wheel stored trim
        # that released as a pitch impulse once the wheel desaturated (the
        # 18.5 deg falls that first disarmed this governor).
        wheel_saturated = (
            abs(self.last_output["l"][0]) >= WHEEL_TORQUE_LIMIT - 1e-6
            or abs(self.last_output["r"][0]) >= WHEEL_TORQUE_LIMIT - 1e-6
        )
        if not wheel_saturated:
            self.cruise_integral = float(np.clip(
                self.cruise_integral + DRIVE_CRUISE_KI * error * dt,
                -DRIVE_CRUISE_I_LIMIT,
                DRIVE_CRUISE_I_LIMIT,
            ))
        target = float(np.clip(
            DRIVE_CRUISE_KP * error + self.cruise_integral,
            -DRIVE_CRUISE_LIMIT,
            DRIVE_CRUISE_LIMIT,
        ))
        self.drive_speed_trim = self._move_towards(
            self.drive_speed_trim,
            target,
            DRIVE_CRUISE_SLEW * dt,
        )
        # Negative application: see the sign note above.  A positive error
        # (too slow) must REDUCE the common wheel output to push the vehicle
        # toward the visible front.
        self.last_output["l"][0] -= self.drive_speed_trim
        self.last_output["r"][0] -= self.drive_speed_trim

        # OPEN ISSUE (next session, T1-A): with the governor holding forward
        # speeds, a REVERSE request still diverges.  Trace: the six-state
        # velocity row itself pushes AWAY from a negative setpoint (at
        # vx=+1, cmd=-1 the row contributes ~-3.3 Nm of forward push), so
        # the governor's 2.2 Nm authority loses that fight.  Flipping
        # LQR_XDOT_SIGN stabilizes the row but destabilizes pitch (14-16
        # deg) -- the same result the 2026-08-27 session recorded, so the
        # row's plant sign is inverted somewhere past wheel_sign, likely in
        # the mirrored wheel axes/gear path, and a clean fix needs the full
        # wheel-output sign audit rather than another patch here.

    def _update_yaw_control(self, dt: float) -> None:
        target = (
            0.0
            if self.jump_state != "idle" or self.landing_buffer_phase != "inactive"
            else self.drive_yaw_rate_target
        )
        if abs(target) > 1e-4:
            error = target - self.yaw_rate
            self.yaw_integral = float(np.clip(
                self.yaw_integral + YAW_RATE_FB_KI * error * dt,
                -YAW_RATE_I_LIMIT,
                YAW_RATE_I_LIMIT,
            ))
            raw = (
                np.copysign(YAW_RATE_FF, target)
                + YAW_RATE_FB_KP * error
                + self.yaw_integral
            )
        else:
            self.yaw_integral = 0.0
            # Keep the accepted straight-line equilibrium bit-identical when
            # no turn is requested.  The vehicle's full yaw lock also uses
            # gimbal/IMU states that this viewer does not model.
            raw = 0.0
        self.yaw_output = self._move_towards(
            self.yaw_output,
            float(np.clip(raw, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT)),
            YAW_OUTPUT_SLEW_RATE * dt,
        )

    def _roll_auxiliary_active(self) -> bool:
        """True while normal ground control owns the support forces.

        Firmware equivalent: the two compensation channels run inside
        Leg_FN_Calculation, i.e. only during ordinary grounded driving --
        not airborne, not in a landing buffer, not during a descent-edge
        transition, and not while a recovery state machine owns the legs.
        """
        return (
            not self.off_ground
            and self.gbuf_phase == "inactive"
            and self.jump_state == "idle"
            and not self.jump_off_ground
            and self.landing_buffer_phase == "inactive"
            and not (self.descent_edge_profile_active and self.descent_guard_active)
            and not self.descent_edge_transition_active
            and self.recovery_state in ("idle", "fw_stand")
        )

    def _update_roll_lateral_ff(self, lengths: dict[str, float], dt: float) -> None:
        """Centrifugal roll feedforward from the commanded motion (code_611).

        Predicts the lateral acceleration of commanded circular motion and
        pre-positions differential support through the load-transfer lever
        arm, so the roll PD regulates around the banked turn equilibrium
        instead of fighting it.  Firmware sign: positive vx*yaw yields
        positive differential force (more support on the left wheel).
        """
        if not self._roll_auxiliary_active():
            self.lateral_inertial_ff = 0.0
            return
        lateral_accel = (
            self._drive_target_for_control() * self.drive_yaw_rate_target
        )
        if abs(lateral_accel) < LATERAL_INERTIAL_FF_ACCEL_DB:
            target = 0.0
        else:
            average_length = 0.5 * sum(lengths.values())
            if average_length >= LATERAL_INERTIAL_FF_HIGH_L0_MIN:
                gain = LATERAL_INERTIAL_FF_GAIN_HIGH
            elif average_length >= LATERAL_INERTIAL_FF_MID_L0_MIN:
                gain = LATERAL_INERTIAL_FF_GAIN_MID
            else:
                gain = LATERAL_INERTIAL_FF_GAIN_LOW
            target = gain * TOTAL_MASS * lateral_accel * average_length / (
                2.0 * LATERAL_INERTIAL_FF_HALF_TRACK_M
            )
            target = float(np.clip(
                target, -LATERAL_INERTIAL_FF_LIMIT, LATERAL_INERTIAL_FF_LIMIT
            ))
        self.lateral_inertial_ff = self._move_towards(
            self.lateral_inertial_ff, target, LATERAL_INERTIAL_FF_SLEW_RATE * dt
        )

    def _update_roll_static_trim(
        self, roll_deg: float, roll_rate_deg: float, dt: float
    ) -> None:
        """Static roll trim integrator, NORMAL profile (code_611 port).

        Learns only while the chassis is settled (low speed, no yaw/roll
        motion, length reference at target), then holds the learned trim
        during motion; the slewed applied copy keeps the support force
        step-free while the estimate changes.
        """
        if not self._roll_auxiliary_active():
            self.roll_static_trim_force = 0.0
            self.roll_static_trim_applied = 0.0
            self.roll_static_trim_timer = 0.0
            return
        length_change_done = all(
            abs(self.length_reference[side] - self.normal_leg_length) < 1e-9
            for side in ("l", "r")
        )
        settled = (
            length_change_done
            and abs(self.drive_forward_target) <= ROLL_STATIC_TRIM_VX_CMD_DB
            and abs(self.chassis_vx) <= ROLL_STATIC_TRIM_VX_FDB_DB
            and abs(self.drive_yaw_rate_target) <= ROLL_STATIC_TRIM_YAW_RATE_DB
            and abs(self.yaw_rate) <= ROLL_STATIC_TRIM_YAW_RATE_DB
            and abs(roll_rate_deg) <= ROLL_STATIC_TRIM_ROLL_RATE_DB
        )
        if not settled:
            self.roll_static_trim_timer = 0.0
        else:
            self.roll_static_trim_timer += dt
            if (
                self.roll_static_trim_timer >= ROLL_STATIC_TRIM_SETTLE_TIME
                and abs(roll_deg) > ROLL_STATIC_TRIM_DEADBAND_DEG
            ):
                self.roll_static_trim_force -= ROLL_STATIC_TRIM_KI * roll_deg * dt
                self.roll_static_trim_force = float(np.clip(
                    self.roll_static_trim_force,
                    -ROLL_STATIC_TRIM_LIMIT,
                    ROLL_STATIC_TRIM_LIMIT,
                ))
        applied_target = float(np.clip(
            self.roll_static_trim_force,
            -ROLL_STATIC_TRIM_APPLY_LIMIT,
            ROLL_STATIC_TRIM_APPLY_LIMIT,
        ))
        self.roll_static_trim_applied = self._move_towards(
            self.roll_static_trim_applied,
            applied_target,
            ROLL_STATIC_TRIM_APPLY_SLEW_RATE * dt,
        )

    def _apply_vmc_support(self, data: mujoco.MjData, control_dt: float, update: bool) -> None:
        """Firmware VMC support, mapped to the two actual drive motors."""
        if not self.stance_enabled:
            self.last_support = {"l": 0.0, "r": 0.0}
            return
        lengths = {
            side: self.vmc_kinematics(data, side).l0 for side in ("l", "r")
        }
        if update:
            roll = self.body_roll(data)
            roll_rate = self._wrap_pi(roll - self.previous["roll"]) / control_dt
            roll_deg = float(np.degrees(roll))
            roll_rate_deg_raw = float(np.degrees(roll_rate))
            derivative_alpha = control_dt / (ROLL_DERIVATIVE_LPF_RC_S + control_dt)
            self.roll_rate_lpf_deg_s += derivative_alpha * (
                roll_rate_deg_raw - self.roll_rate_lpf_deg_s
            )
            average_length = 0.5 * sum(lengths.values())
            roll_progress = float(np.clip(
                (average_length - ROLL_GAIN_L0_START) /
                (ROLL_GAIN_L0_END - ROLL_GAIN_L0_START),
                0.0,
                1.0,
            ))
            roll_gain_scale = 1.0 + roll_progress * (ROLL_GAIN_MASS_SCALE - 1.0)
            self.roll_force = float(np.clip(
                # Firmware polarity (force opposes measured roll).  The WIP
                # +roll/+roll-rate variant looked equivalent on a pure stand
                # but turned the drive/derivative coupling marginally
                # unstable: forward cruise lost ~0.13 m of travel and the
                # recovery final length drifted to 0.27 m.
                roll_gain_scale * (
                    ROLL_KP_N_PER_DEG * (0.0 - roll_deg)
                    - ROLL_KD_N_PER_DEG_S * self.roll_rate_lpf_deg_s
                ),
                -ROLL_FORCE_LIMIT,
                ROLL_FORCE_LIMIT,
            ))
            self._update_roll_lateral_ff(lengths, control_dt)
            self._update_roll_static_trim(
                roll_deg, self.roll_rate_lpf_deg_s, control_dt
            )
            self.previous["roll"] = roll
            for side in ("l", "r"):
                length_rate_raw = (
                    lengths[side] - self.previous[f"length_{side}"]
                ) / control_dt
                self.length_rate_lpf[side] = (
                    VMC_RATE_LPF_RETAIN * self.length_rate_lpf[side]
                    + (1.0 - VMC_RATE_LPF_RETAIN) * length_rate_raw
                )

            if self.off_ground or self.gbuf_phase != "inactive":
                # The generic safety path has already replaced support_force
                # with the real airborne/deploy or LOW landing-buffer law.
                # Do not overwrite it with the normal standing PID here.
                pass
            elif (
                self.jump_state != "idle"
                or self.jump_off_ground
                or self.landing_buffer_phase != "inactive"
            ):
                self.last_support = self._jump_supports(lengths)
            elif (
                self.descent_edge_profile_active
                and self.descent_guard_active
                and not self.descent_guard_drop_seen
            ):
                # Start the clearance motion in the geometric look-ahead
                # window, while both wheels still carry a real vertical
                # contact.  Waiting for contact loss is too late: the upper
                # tread then catches the long leg and the resulting impulse
                # pitches the base into the next riser.  Keep the reference
                # bilateral so a one-sample contact asymmetry cannot skew the
                # two four-bars.
                if update:
                    mean_reference = 0.5 * sum(
                        self.descent_edge_length_reference.values()
                    )
                    mean_reference = self._move_towards(
                        mean_reference,
                        DESCENT_EDGE_RETRACT_LENGTH,
                        DESCENT_EDGE_RETRACT_RATE * control_dt,
                    )
                    for side in ("l", "r"):
                        self.descent_edge_length_reference[side] = mean_reference
                mean_spring = 0.5 * sum(
                    self._gas_virtual_support(lengths[side])
                    for side in ("l", "r")
                ) if self.gas_enabled else 0.0
                mean_reference = 0.5 * sum(
                    self.descent_edge_length_reference.values()
                )
                mean_length = 0.5 * sum(lengths.values())
                mean_length_rate = 0.5 * sum(
                    self.length_rate_lpf.values()
                )
                edge_length_force = float(np.clip(
                    LENGTH_KP * (mean_reference - mean_length)
                    - LENGTH_KD_LOW * mean_length_rate,
                    -LENGTH_OUTPUT_LIMIT,
                    LENGTH_OUTPUT_LIMIT,
                ))
                transition_support = (
                    0.5 * TOTAL_MASS * GRAVITY
                    - mean_spring
                    + edge_length_force
                )
                self.last_support = {
                    side: float(np.clip(
                        transition_support
                        + (self.roll_force if side == "l" else -self.roll_force),
                        -SUPPORT_FORCE_LIMIT,
                        SUPPORT_FORCE_LIMIT,
                    ))
                    for side in ("l", "r")
                }
                for side in ("l", "r"):
                    self.length_integral[side] = 0.0
            elif self.descent_edge_transition_active:
                # Keep a common length loop while one wheel is crossing the
                # riser.  The old edge path held only a fixed gas-compensated
                # load, which let the gas springs extend both legs into the
                # l0~=0.33 m kinematic singularity before airborne control
                # took over.  Use the mean length/rate for the axial PID so a
                # different upper/lower tread height cannot make the two
                # linkage references fight and skew the chassis.
                mean_spring = 0.5 * sum(
                    self._gas_virtual_support(lengths[side])
                    for side in ("l", "r")
                ) if self.gas_enabled else 0.0
                edge_length_reference = 0.5 * sum(
                    self.descent_edge_length_reference.values()
                )
                mean_length = 0.5 * sum(lengths.values())
                mean_length_rate = 0.5 * sum(
                    self.length_rate_lpf.values()
                )
                edge_length_force = float(np.clip(
                    LENGTH_KP * (edge_length_reference - mean_length)
                    - LENGTH_KD_LOW * mean_length_rate,
                    -LENGTH_OUTPUT_LIMIT,
                    LENGTH_OUTPUT_LIMIT,
                ))
                transition_support = (
                    0.5 * TOTAL_MASS * GRAVITY
                    - mean_spring
                    + edge_length_force
                )
                self.last_support = {
                    side: float(np.clip(
                        transition_support
                        + (self.roll_force if side == "l" else -self.roll_force),
                        -SUPPORT_FORCE_LIMIT,
                        SUPPORT_FORCE_LIMIT,
                    ))
                    for side in ("l", "r")
                }
                for side in ("l", "r"):
                    self.length_integral[side] = 0.0
            else:
                for side in ("l", "r"):
                    length = lengths[side]
                    length_rate = self.length_rate_lpf[side]
                    # Full rm_config.h low-leg PID, including the source's
                    # integral sign-reset and integral/output clamps.
                    self.length_reference[side] = self._move_towards(
                        self.length_reference[side],
                        self.normal_leg_length,
                        LEG_LENGTH_SWITCH_RATE * control_dt,
                    )
                    length_error = self.length_reference[side] - length
                    if length_error * self.length_integral[side] < 0.0:
                        self.length_integral[side] = 0.0
                    self.length_integral[side] = float(np.clip(
                        self.length_integral[side]
                        + LENGTH_KI * length_error * control_dt,
                        -LENGTH_INTEGRAL_LIMIT,
                        LENGTH_INTEGRAL_LIMIT,
                    ))
                    length_pid = float(np.clip(
                        LENGTH_KP * length_error
                        + self.length_integral[side]
                        - LENGTH_KD_LOW * length_rate,
                        -LENGTH_OUTPUT_LIMIT,
                        LENGTH_OUTPUT_LIMIT,
                    ))
                    spring_support = (
                        self._gas_virtual_support(length)
                        if self.gas_enabled else 0.0
                    )
                    roll_auxiliary = (
                        self.lateral_inertial_ff + self.roll_static_trim_applied
                    )
                    support = (
                        0.5 * TOTAL_MASS * GRAVITY
                        - spring_support
                        + length_pid
                        + self.support_trim[side]
                        + (
                            self.roll_force + roll_auxiliary
                            if side == "l"
                            else -self.roll_force - roll_auxiliary
                        )
                    )
                    support = float(np.clip(
                        support, -SUPPORT_FORCE_LIMIT, SUPPORT_FORCE_LIMIT
                    ))
                    if self.recovery_state == "fw_handoff":
                        recovery_force = float(
                            self.fw_recovery_handoff_output[side][0]
                        )
                        support = recovery_force + self.lqr_gain * (
                            support - recovery_force
                        )
                    self.last_support[side] = support

        for side in ("l", "r"):
            support = self.last_support[side]
            tp = self.vmc_tp_sign * (
                float(self.last_output[side][1]) if self.enabled else 0.0
            )
            if self.recovery_state == "fw_handoff":
                recovery_tp = float(self.fw_recovery_handoff_output[side][1])
                tp = recovery_tp + self.lqr_gain * (tp - recovery_tp)
            motor_limit = (
                JUMP_HIP_MOTOR_TORQUE_LIMIT
                if self.jump_state != "idle"
                or self.jump_off_ground
                or self.landing_buffer_phase != "inactive"
                else HIP_TORQUE_LIMIT
            )
            self.set_vmc_motor_torques(
                data, side, support, tp, torque_limit=motor_limit
            )

    def passive_step(self, data: mujoco.MjData) -> None:
        """Remove every controller-applied force for free-swing calibration."""
        data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}
        self.last_support = {"l": 0.0, "r": 0.0}
        self.lqr_gain = 0.0
        self.cancel_recovery()

    def step(self, data: mujoco.MjData, dt: float) -> None:
        if self.recovery_state in {
            "fw_hold", "fw_swing", "fw_retract", "fw_fault"
        }:
            self._step_encoder_zero_recovery(data, dt)
            return
        if self.recovery_state == "prone_swing":
            self._step_prone_swing(data, dt)
            return
        if self.recovery_state in {"cad_stand_recovery", "stand"}:
            self._step_cad_stand_recovery(data, dt)
            return
        if self.recovery_state == "retract":
            self._step_torque_recovery(data, dt)
            return
        self._apply_gas_springs(data)
        data.ctrl[:] = 0.0
        self._apply_gimbal_lock(data)

        self.control_elapsed += dt
        update = self.control_elapsed + 1e-12 >= CONTROL_PERIOD_S
        control_dt = self.control_elapsed if update else CONTROL_PERIOD_S
        if update:
            self.control_elapsed = 0.0

            if self.recovery_state in {"handoff", "fw_handoff"}:
                self.lqr_gain = min(1.0, self.lqr_gain + RECOVERY_LQR_HANDOFF_RATE * control_dt)
                if self.lqr_gain >= 1.0:
                    self.recovery_state = (
                        "fw_stand"
                        if self.recovery_state == "fw_handoff"
                        else "stand"
                    )

            if (
                self.recovery_state in {"fw_handoff", "fw_stand"}
                and data.time >= self.fw_recovery_length_hold_until
            ):
                for side in ("l", "r"):
                        self.length_reference[side] = self._move_towards(
                            self.length_reference[side],
                            self.normal_leg_length,
                            FW_RECOVERY_L0_RATE * control_dt,
                        )

        if self.enabled:
            if update:
                if self.recovery_state == "idle":
                    self.lqr_gain = min(1.0, self.lqr_gain + control_dt / LQR_RAMP_TIME_S)
                heading, heading_angle = self._heading(data)
                displacement = data.xpos[self.base] - self.previous_base_position
                self.chassis_vx = -float(np.dot(
                    displacement, heading
                )) / control_dt
                self.yaw_rate = self._wrap_pi(
                    heading_angle - self.previous_heading
                ) / control_dt
                control_drive_target = self._drive_target_for_control()
                if (
                    abs(control_drive_target) < 1e-6
                    and self.jump_state == "idle"
                    and not self.descent_guard_active
                ):
                    if (
                        self.jump_landing_seen
                        and self.jump_stationary_position_hold
                        and self.jump_takeoff_position is not None
                    ):
                        launch_error = -float(np.dot(
                            data.xpos[self.base] - self.jump_takeoff_position,
                            self.forward_world,
                        ))
                        self.lqr_virtual_x = float(np.clip(
                            self.jump_landing_position_x_ratio * launch_error,
                            -LQR_POSITION_LIMIT,
                            LQR_POSITION_LIMIT,
                        ))
                        self.lqr_position_state = "hold"
                        self.lqr_position_capture_timer = (
                            LQR_POSITION_CAPTURE_TIME
                        )
                        self.lqr_position_hold_x = self.lqr_virtual_x
                        self.lqr_position_brake_offset = 0.0
                    else:
                        self._update_lqr_position_observer(
                            self.chassis_vx, control_dt
                        )
                elif self.descent_guard_active:
                    # Do not accumulate a position-hold error while the
                    # vehicle is crossing a drop.  The horizontal speed
                    # error must be the only wheel-LQR drive term during this
                    # short guarded interval; a growing virtual position
                    # error can reverse the brake into propulsion at the edge.
                    self.lqr_position_state = "drive"
                    self.lqr_position_capture_timer = 0.0
                    self.lqr_position_hold_x = 0.0
                    self.lqr_position_brake_offset = 0.0
                    self.lqr_virtual_x = 0.0
                else:
                    self.lqr_position_state = "drive"
                    self.lqr_position_capture_timer = 0.0
                    self.lqr_position_hold_x = 0.0
                    self.lqr_position_brake_offset = 0.0
                    self.lqr_virtual_x = 0.0
                self._update_jump_state(data, control_dt)
                self.theta_comp_apply_gain = min(
                    1.0,
                    self.theta_comp_apply_gain + THETA_COMP_APPLY_RATE * control_dt,
                )
                for side in ("l", "r"):
                    state, length = self._state(data, side, control_dt)
                    output = self.lqr_gain * (self.gain(length) @ (-state))
                    wheel_torque = float(np.clip(
                        output[0],
                        -WHEEL_TORQUE_LIMIT,
                        WHEEL_TORQUE_LIMIT,
                    ))
                    hip_torque = float(-output[1])
                    self.last_state[side] = state
                    self.last_output[side] = np.array((wheel_torque, hip_torque))
                # Firmware YAW_LQR_COMMON_MIX_ENABLE=1: balancing torque is
                # common mode.  Removing the differential component prevents
                # small left/right VMC errors from turning the free base.
                common_wheel = 0.5 * (
                    self.last_output["l"][0] + self.last_output["r"][0]
                )
                self.last_output["l"][0] = common_wheel
                self.last_output["r"][0] = common_wheel

                # The same differential theta synchronizer used after the
                # firmware LQR keeps one four-bar from opening while the
                # other closes.  It contains no common body-pitch torque.
                theta_diff = float(
                    self.last_state["l"][0] - self.last_state["r"][0]
                )
                theta_rate_diff = float(
                    self.last_state["l"][1] - self.last_state["r"][1]
                )
                average_length = 0.5 * sum(
                    self.vmc_kinematics(data, side).l0 for side in ("l", "r")
                )
                sync_progress = float(np.clip(
                    (average_length - THETA_SYNC_GAIN_L0_START) /
                    (THETA_SYNC_GAIN_L0_END - THETA_SYNC_GAIN_L0_START),
                    0.0,
                    1.0,
                ))
                sync_progress = sync_progress * sync_progress * (3.0 - 2.0 * sync_progress)
                sync_kp = THETA_SYNC_KP_LOW + sync_progress * (
                    THETA_SYNC_KP_HIGH - THETA_SYNC_KP_LOW
                )
                sync_kd = THETA_SYNC_KD_LOW + sync_progress * (
                    THETA_SYNC_KD_HIGH - THETA_SYNC_KD_LOW
                )
                sync_torque = sync_kp * theta_diff + sync_kd * theta_rate_diff
                guard_progress = float(np.clip(
                    (abs(theta_diff) - THETA_SYNC_GUARD_START) /
                    (THETA_SYNC_GUARD_END - THETA_SYNC_GUARD_START),
                    0.0,
                    1.0,
                ))
                guard_progress = guard_progress * guard_progress * (3.0 - 2.0 * guard_progress)
                sync_torque += float(np.copysign(
                    THETA_SYNC_GUARD_LIMIT * guard_progress,
                    theta_diff,
                ))
                self.last_output["l"][1] += sync_torque
                self.last_output["r"][1] -= sync_torque

                self._apply_jump_hip_control(data, control_dt)
                self._apply_drive_speed_trim(control_dt)
                self._update_landing_wheel_control(data, control_dt)
                self._update_yaw_control(control_dt)
                self._update_danger_check(data, control_dt)
                self._update_off_ground(data, control_dt)

            for side, ids in self.sides.items():
                if self.landing_buffer_phase != "inactive":
                    wheel_command = self.jump_landing_wheel_command[side]
                elif self.jump_off_ground:
                    wheel_command = (
                        self.wheel_sign[side] * self.jump_air_wheel_torque
                    )
                elif self.off_ground:
                    # Use the wheels as an internal reaction axis while the
                    # generic airborne leg profile deploys.  They are not
                    # used as a ground drive unless a real vertical stair
                    # tread has already been observed.  That contact-gated
                    # exception keeps the vehicle moving across a lower
                    # tread without applying propulsion while airborne.
                    contact_drive = (
                        -np.copysign(
                            DESCENT_OFFGROUND_CONTACT_DRIVE_TORQUE,
                            self.drive_forward_target,
                        )
                        if self.descent_edge_contact_drive
                        and abs(self.drive_forward_target) > 1e-6
                        else 0.0
                    )
                    wheel_command = self.wheel_sign[side] * (
                        self.offground_wheel_torque + contact_drive
                    )
                else:
                    yaw_command = (
                        0.0
                        if self.jump_state != "idle"
                        or self.landing_buffer_phase != "inactive"
                        else self.yaw_output
                    )
                    wheel_command = (
                        self.wheel_sign[side] * self.last_output[side][0]
                        - yaw_command
                    )
                data.ctrl[ids.wheel_motor] = float(np.clip(
                    wheel_command, -WHEEL_TORQUE_LIMIT, WHEEL_TORQUE_LIMIT
                ))
        else:
            self.lqr_gain = 0.0
            self.last_output = {"l": np.zeros(2), "r": np.zeros(2)}

        self._apply_vmc_support(data, control_dt, update)

        if update:
            self.previous["x"] = float(np.dot(data.xpos[self.base] - self.base_position0, self._frame(data)[1]))
            self.previous["pitch"] = self.body_pitch(data)
            self.previous_base_position = data.xpos[self.base].copy()
            _, self.previous_heading = self._heading(data)
            for side, ids in self.sides.items():
                kin = self.vmc_kinematics(data, side)
                theta = np.pi / 2.0 - kin.phi0 + self.body_pitch(data)
                length = kin.l0
                self.previous[f"theta_{side}"] = theta
                self.previous[f"length_{side}"] = length
                self.previous[f"phi_{side}"] = kin.phi0
