"""Traditional LQR port for the whole-body MuJoCo model.

The feedback law and the medium-leg gain-polynomials come directly from
``src/task/chassis/control/chassis_control.c``.  MuJoCo supplies the state
instead of the IMU/encoder/Kalman pipeline; wheel and hip output signs remain
explicit constants so they can be calibrated against the assembled model.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import mujoco
import numpy as np


GRAVITY = 9.81
TOTAL_MASS = 12.801
WHEEL_TORQUE_LIMIT = 3.69
HIP_TORQUE_LIMIT = 20.0
GIMBAL_YAW_TORQUE_LIMIT = 1.2
GIMBAL_YAW_KP = 28.0
GIMBAL_YAW_KD = 1.8
GAS_SPRING_FORCE = 300.0
SUPPORT_FORCE_LIMIT = 200.0
LQR_RAMP_TIME_S = 0.8
LENGTH_KP = 450.0
LENGTH_KI = 100.0
LENGTH_INTEGRAL_LIMIT = 35.0
LENGTH_OUTPUT_LIMIT = 150.0
# rm_config.h normal low-leg derivative gain.
LENGTH_KD_LOW = 18.0
VMC_RATE_LPF_RETAIN = 0.5
ROLL_KP_N_PER_DEG = 3.5
ROLL_KD_N_PER_DEG_S = 0.10
ROLL_FORCE_LIMIT = 40.0
ROLL_DERIVATIVE_LPF_RC_S = 0.020
ROLL_GAIN_L0_START = 0.140
ROLL_GAIN_L0_END = 0.260
ROLL_GAIN_MASS_SCALE = TOTAL_MASS / 6.120
THETA_SYNC_KP_LOW = 15.0
THETA_SYNC_KP_HIGH = 19.0
THETA_SYNC_KD_LOW = 0.60
THETA_SYNC_KD_HIGH = 0.90
THETA_SYNC_GAIN_L0_START = 0.160
THETA_SYNC_GAIN_L0_END = 0.240
THETA_SYNC_GUARD_START = 0.150
THETA_SYNC_GUARD_END = 0.300
THETA_SYNC_GUARD_LIMIT = 2.50

# Firmware jump profile from chassis_control.c / chassis_safety.c.  The only
# geometry translation is the final LOW reference: this simulation's
# user-calibrated standing length is 0.150 m, while the current vehicle build
# names 0.130 m as LEN_LEN_LOW.  The preload, thrust, tuck, and deploy targets
# below are copied without scaling.
JUMP_COOLDOWN_S = 3.0
JUMP_PRELOAD_LENGTH = 0.125
JUMP_THRUST_LENGTH = 0.305
JUMP_PRELOAD_MIN_TIME = 0.22
JUMP_PRELOAD_MAX_TIME = 0.85
JUMP_PRELOAD_READY_LENGTH = 0.155
JUMP_PRELOAD_READY_RATE = 0.25
JUMP_THRUST_BOOST_FORCE = 70.0
JUMP_THRUST_BOOST_TIME = 0.26
JUMP_THRUST_SUPPORT_FLOOR = 120.0
JUMP_THRUST_MAX_TIME = 0.70
JUMP_ABORT_HOLD_TIME = 0.12
JUMP_ABORT_SETTLE_TIME = 0.60
JUMP_ABORT_RETRACT_RATE = 0.35
JUMP_FLIGHT_TIMEOUT = 1.50
JUMP_LANDING_GROUND_CONFIRM_TIME = 0.15
JUMP_LANDING_TIMEOUT = 2.00
JUMP_LANDING_PITCH_MAX = np.radians(12.0)
JUMP_LANDING_ROLL_MAX = np.radians(10.0)
JUMP_LANDING_LENGTH_DIFF_MAX = 0.030
JUMP_LANDING_TIMEOUT_LENGTH_DIFF_MAX = 0.060
# Simulation-bold precheck: attitude gates are wide open and the launch
# speed gate is effectively removed -- a moving jump is allowed at ANY
# ground speed.  Only the physical sanity gates remain (state machine idle,
# cooldown, wheels grounded, legs inside range).
JUMP_PRECHECK_PITCH_MAX = np.radians(25.0)
JUMP_PRECHECK_ROLL_MAX = np.radians(25.0)
# Launch speed is no longer gated: simulation-only policy decision.  The
# constant survives as a huge bound so the precheck expression stays valid.
JUMP_SAFE_LAUNCH_VX_MAX = 10.0
JUMP_SAFE_LAUNCH_COMMAND_MARGIN = 10.0
JUMP_SAFE_LAUNCH_MOVING_MAX = 10.0
JUMP_PRECHECK_YAW_RATE_MAX = 2.50
JUMP_ABORT_PITCH_MAX = np.radians(40.0)
JUMP_ABORT_ROLL_MAX = np.radians(30.0)
JUMP_LENGTH_KP_PRELOAD = 450.0
JUMP_LENGTH_KD_PRELOAD = 28.0
JUMP_LENGTH_LIMIT_PRELOAD = 70.0
JUMP_LENGTH_KP_THRUST = 600.0
JUMP_LENGTH_KD_THRUST = 16.0
JUMP_LENGTH_LIMIT_THRUST = 120.0
JUMP_LENGTH_SYNC_KP = 400.0
JUMP_LENGTH_SYNC_KD = 70.0
JUMP_LENGTH_SYNC_LIMIT = 30.0
JUMP_THRUST_PITCH_KP = 8.0
JUMP_THRUST_PITCH_KD = 20.0
JUMP_THRUST_PITCH_TP_LIMIT = 40.0
JUMP_THRUST_PITCH_TP_SLEW_RATE = 100.0
JUMP_THRUST_PITCH_RATE_TARGET_ZERO = 1.48
JUMP_THRUST_PITCH_RATE_VX_GAIN = 1.80
JUMP_THRUST_PITCH_RATE_TARGET_MIN = -0.25
JUMP_THRUST_PITCH_RATE_TARGET_MAX = 1.48
JUMP_THRUST_PITCH_RATE_START_L0 = 0.16
JUMP_THRUST_PITCH_RATE_FULL_L0 = 0.26
JUMP_THRUST_PITCH_RATE_FILTER_TAU = 0.060
JUMP_THRUST_THETA_HOLD_KP = 0.0
JUMP_THRUST_THETA_HOLD_KD = 0.0
JUMP_THRUST_THETA_HOLD_LIMIT = 22.0
JUMP_PRETUCK_THETA_REF_RATE = 2.0
JUMP_THRUST_THETA_YIELD_START = 0.04
JUMP_THRUST_THETA_YIELD_END = 0.16
JUMP_THRUST_THETA_LQR_GAIN = 0.75
JUMP_LANDING_TP_HANDOFF_TIME = 0.060
JUMP_AIR_LQR_TP_GAIN = 1.00
JUMP_AIR_TP_LIMIT = 22.0
# --- Security_Checking port (chassis_safety.c) ---
# Real-vehicle fall detection: hard attitude/leg limits are immediate faults,
# soft candidates debounce for DANGER_SOFT_CONFIRM_TIME while grounded.  The
# firmware routes the confirmed danger state to the recovery module; here the
# same confirmation hands control to begin_recovery() (the ported self-rescue
# state machine).
DANGER_HARD_ATTITUDE_DEG = 60.0
DANGER_SOFT_PITCH_DEG = 35.0
DANGER_SOFT_ROLL_DEG = 30.0
DANGER_SOFT_THETA_RAD = np.radians(45.0)
DANGER_SOFT_CONFIRM_TIME = 0.08
DANGER_LEG_SAFE_AREA_DEG = 20.0
# --- REC_SIMPLE inverted self-rescue (chassis_recovery.c) ---
# Ported values for the pose-agnostic fall recovery: HOLD classifies the
# fallen pose, INVERTED sweeps both legs toward negative phi0 to right the
# body, FORWARD_ZERO drives the legs to zero before the normal SWING ->
# RETRACT -> HANDOFF route.  Torque/limits are the firmware defines.
FW_RECOVERY_FORWARD_PITCH_MAX = np.radians(80.0)
FW_RECOVERY_INVERTED_ENTER_PITCH = np.radians(105.0)
FW_RECOVERY_INVERTED_EXIT_PITCH = np.radians(40.0)
FW_RECOVERY_INVERTED_CAPTURE_TIME = 0.20
FW_RECOVERY_INVERTED_PRELOAD_TIME = 0.25
FW_RECOVERY_INVERTED_TURN_RATE = np.radians(105.0)
FW_RECOVERY_INVERTED_TURN_ACCEL = np.radians(240.0)
FW_RECOVERY_INVERTED_TURN_MAX_TRAVEL = np.radians(720.0)
FW_RECOVERY_INVERTED_SYNC_KP = 1.0
FW_RECOVERY_INVERTED_SYNC_CORR_LIMIT = np.radians(37.5)
FW_RECOVERY_INVERTED_WAIT_ENTER = np.radians(12.0)
FW_RECOVERY_INVERTED_WAIT_EXIT = np.radians(6.0)
FW_RECOVERY_INVERTED_TP_KP = 36.0
FW_RECOVERY_INVERTED_TP_KD = 2.6
FW_RECOVERY_INVERTED_TP_LIMIT = 9.5
FW_RECOVERY_INVERTED_TP_BIAS = 4.0
FW_RECOVERY_INVERTED_TURN_GAIN_RATE = 3.0
FW_RECOVERY_INVERTED_CONTACT_FORCE = 12.0
FW_RECOVERY_INVERTED_PITCH_PROGRESS_EPS = 0.10
FW_RECOVERY_INVERTED_PITCH_STALL_TIME = 0.75
FW_RECOVERY_INVERTED_TURN_TIMEOUT = 100.0
FW_RECOVERY_INVERTED_SETTLE_TIME = 0.50
FW_RECOVERY_INVERTED_SETTLE_TP_KP = 12.0
FW_RECOVERY_INVERTED_SETTLE_TP_KD = 2.0
FW_RECOVERY_INVERTED_SETTLE_TP_LIMIT = 3.0
FW_RECOVERY_FORWARD_ZERO_RATE = np.radians(110.0)
FW_RECOVERY_FORWARD_ZERO_ACCEL = np.radians(220.0)
FW_RECOVERY_FORWARD_ZERO_GAIN_RATE = 4.0
FW_RECOVERY_FORWARD_ZERO_TP_KP = 16.0
FW_RECOVERY_FORWARD_ZERO_TP_KD = 2.2
# Firmware note: the old 3.75 Nm clip could not cross the dead-point pose;
# phase 10 saturates at 12 Nm per hip motor.
FW_RECOVERY_FORWARD_ZERO_TP_LIMIT = 12.0
FW_RECOVERY_FORWARD_ZERO_TP_SIGN = 1.0
FW_RECOVERY_FORWARD_ZERO_PHI0_OK = 0.25
FW_RECOVERY_FORWARD_ZERO_STABLE_TIME = 0.15
FW_RECOVERY_FORWARD_ZERO_TIMEOUT = 20.0
FW_RECOVERY_SIDE_POSE_MIN = np.radians(45.0)
# Firmware uses 150 deg; this CAD export's four-bar tops out near 163 deg,
# so a 151-deg fallen pose just missed the short-path window and the
# firmware's positive +360 sweep is unreachable here.  Widened to 165 deg so
# the reachable signed sweep runs (documented sim deviation).
FW_RECOVERY_SIDE_POSE_MAX = np.radians(165.0)
FW_RECOVERY_L0_LONG = 0.30
FW_RECOVERY_PHI_NEAR_ZERO = 0.70
FW_RECOVERY_ZERO_PITCH_MAX = np.radians(15.0)
FW_RECOVERY_FAULT_HOLD_TIME = 1.5
# --- generic off-ground port (chassis_safety.c / rm_config.h) ---
OFFGROUND_TAKEOFF_FN_LOW = 30.0
OFFGROUND_JUMP_TAKEOFF_SIDE_MAX = 50.0
OFFGROUND_TAKEOFF_CONFIRM_LOW = 5
# The reconstructed wheel force reads a phantom unload around every controller
# reset: recovery SWING/RETRACT compresses the legs and releases them into the
# LOW stand, and that axial transient looks exactly like a takeoff in the
# four-sample filter.  Observed consequence when Ground_Detect is allowed to
# fire during it: the generic airborne deploy profile extends the legs from
# 0.11 m to full length, bounces the chassis off the floor, and the machine
# then hops between misjudged takeoffs and landing buffers indefinitely.
# Hold the detector at grounded through this window after each reset.  Five
# seconds covers the stand-up transient plus the first drive-command settle
# bounce (observed wheel unload signatures until ~3.3 s on the platform
# scene); releasing earlier latched descent_guard.drop_seen without any real
# ledge ever being crossed.
OFFGROUND_STARTUP_SUPPRESS_S = 5.0
OFFGROUND_LANDING_FN_LOW = 45.0
OFFGROUND_LANDING_CONFIRM_LOW = 8
OFFGROUND_UNSPRUNG_MASS = 0.60
OFFGROUND_SOFT_FORCE = 20.0
OFFGROUND_LQR_TP_GAIN = 0.50
OFFGROUND_LQR_TP_LIMIT = 40.0
OFFGROUND_LENGTH_SYNC_KP = 300.0
OFFGROUND_LENGTH_SYNC_KD = 12.0
OFFGROUND_LENGTH_SYNC_LIMIT = 18.0
OFFGROUND_AIR_RETRACT_LIMIT = 120.0
OFFGROUND_SPRING_COMP_LIMIT = SUPPORT_FORCE_LIMIT
OFFGROUND_THETA_HOLD_KP = 8.5
OFFGROUND_THETA_HOLD_KD = 0.75
OFFGROUND_THETA_HOLD_TP_LIMIT = 4.5
# The real firmware leaves the generic airborne body-pitch row disabled.  The
# MuJoCo model has no motor/IMU delay or structural compliance, so a small
# bounded common pitch allocation is needed for a hard 200 mm ledge drop; it
# is used only by the generic off-ground path, never by the active jump path.
OFFGROUND_PITCH_KP = 80.0
OFFGROUND_PITCH_KD = 20.0
OFFGROUND_PITCH_TP_LIMIT = 40.0
OFFGROUND_PITCH_TP_EXTRA_KP_NM_PER_DEG = 0.0
OFFGROUND_PITCH_TP_EXTRA_KD_NM_PER_DEG_S = 0.0
OFFGROUND_PITCH_TP_EXTRA_LIMIT = 8.0
OFFGROUND_WHEEL_PITCH_KP = 35.0
OFFGROUND_WHEEL_PITCH_KD = 5.0
OFFGROUND_WHEEL_TORQUE_LIMIT = WHEEL_TORQUE_LIMIT
# During a ledge transition one wheel can be the only real vertical support
# for a short interval.  Use a deliberately low differential axial load to
# damp roll through that supported leg; this is a load-transfer term, not a
# pose teleport or a hard chassis-angle clamp.
OFFGROUND_ROLL_KP_N_PER_DEG = 1.20
OFFGROUND_ROLL_KD_N_PER_DEG_S = 0.06
OFFGROUND_ROLL_FORCE_LIMIT = 28.0
# During a unilateral edge transition the axial load path alone can be too
# weak to arrest roll before the second wheel reaches the lower tread.  The
# differential hip row supplies a bounded attitude moment through the leg
# linkage; it is enabled only in the generic airborne path.
OFFGROUND_ROLL_TP_KP_NM_PER_DEG = 0.0
OFFGROUND_ROLL_TP_KD_NM_PER_DEG_S = 0.0
OFFGROUND_ROLL_TP_LIMIT = 12.0
# A single tread contact may be the only available load path at first
# touchdown.  It is accepted only after the IMU and vertical velocity are
# already inside a small basin; a side scrape with a large body motion cannot
# satisfy this condition.
OFFGROUND_LANDING_ATTITUDE_MAX = np.radians(12.0)
OFFGROUND_LANDING_VERTICAL_SPEED_MAX = 0.80
LANDING_AIR_DEPLOY_LENGTH = 0.27
LANDING_AIR_DEPLOY_REF_RATE = 0.55
LANDING_AIR_DEPLOY_KP = 350.0
LANDING_AIR_DEPLOY_KD = 20.0
LANDING_AIR_DEPLOY_FORCE_LIMIT = 60.0
GBUF_COMPRESS_TIME = 0.20
LANDING_COMPRESS_REF_RATE = 0.60
GBUF_HOLD_MIN_TIME = 0.18
GBUF_HOLD_MAX_TIME = 0.45
GBUF_RETURN_REF_RATE = 0.80
GBUF_PID_RETURN_TIME = 0.30
GBUF_EXIT_STABLE_TIME = 0.18
GBUF_SETTLE_LENGTH_ERROR = 0.015
GBUF_SETTLE_LENGTH_SPEED = 0.10
GBUF_PITCH_KP = 20.0
GBUF_PITCH_KD = 6.0
GBUF_PITCH_TP_LIMIT = 10.0
# The landing buffer damps residual translation but does not clamp both drive
# motors to their full reverse torque.  Full clamping makes the wheel behave
# like a locked skid on a tread edge and creates the forward-pitch impulse.
GBUF_BRAKE_KP = 6.0
GBUF_BRAKE_LIMIT = WHEEL_TORQUE_LIMIT
GBUF_SUPPORT_FLOOR = 35.0
# A landing buffer can span more than one 200 mm tread when the vehicle has
# residual forward speed.  If both wheels lose a vertical support normal
# during that buffer, hand control back to the airborne branch instead of
# pretending the wheels are still carrying the chassis.
GBUF_SUPPORT_LOSS_CONFIRM = 20
LANDING_BUFFER_MID_LENGTH = 0.21
JUMP_AIR_THETA_HOLD_KP = 18.0
JUMP_AIR_THETA_HOLD_KD = 8.0
JUMP_AIR_THETA_HOLD_LIMIT = 22.0
JUMP_AIR_TUCK_LENGTH = 0.14
JUMP_AIR_TUCK_TIME = 0.11
JUMP_AIR_TUCK_HOLD_TIME = 0.12
JUMP_AIR_TUCK_REF_RATE = 3.50
JUMP_AIR_LANDING_LENGTH = 0.27
JUMP_AIR_CAPTURE_LENGTH_MAX = 0.37
JUMP_AIR_DEPLOY_REF_RATE = 1.20
JUMP_AIR_DEPLOY_KP = 1000.0
JUMP_AIR_DEPLOY_KD = 26.0
JUMP_AIR_RETRACT_FORCE_LIMIT = 200.0
JUMP_AIR_RETRACT_FORCE_PEAK = 260.0
JUMP_AIR_LENGTH_SYNC_KP = 380.0
JUMP_AIR_LENGTH_SYNC_KD = 24.0
JUMP_AIR_LENGTH_SYNC_LIMIT = 30.0
JUMP_TAKEOFF_CONFIRM_COUNT = 5
JUMP_LANDING_CONFIRM_COUNT = 8
# The real hip drives are rated 20 Nm and allow 40 Nm peak.  Normal standing
# remains at HIP_TORQUE_LIMIT=20; only the short jump/landing window may use
# the peak envelope.
JUMP_HIP_MOTOR_TORQUE_LIMIT = 40.0

# LOW landing buffer constants.  The air-attitude/leg profile now delivers
# wheel-only contact close to zero pitch; this buffer absorbs the remaining
# axial impact before handing support and wheel LQR back to normal standing.
LANDING_LOW_CONTACT_SPLIT_MAX = 0.008
LANDING_LOW_RATCHET_MARGIN = 0.035
LANDING_LOW_RATCHET_REF_RATE = 1.20
LANDING_HOLD_MIN_TIME = 0.12
LANDING_HOLD_MAX_TIME = 0.30
LANDING_SETTLE_LENGTH_ERROR = 0.015
LANDING_SETTLE_LENGTH_SPEED = 0.10
LANDING_RETURN_REF_RATE_LOW = 1.20
LANDING_PID_RETURN_TIME = 0.20
LANDING_EXIT_STABLE_TIME = 0.12
LANDING_EXIT_LENGTH_SPEED = 0.12
LANDING_CAPTURE_TIME = 0.040
LANDING_SUPPORT_BLEND_TIME = LANDING_CAPTURE_TIME
LANDING_CAPTURE_TP_SLEW_RATE = 6000.0
LANDING_CAPTURE_WHEEL_SLEW_RATE = 1100.0
LANDING_WHEEL_SLEW_RATE = 1000.0
LANDING_LENGTH_KP = 650.0
LANDING_LENGTH_KD_LOW = 45.0
LANDING_LENGTH_OUTPUT_LIMIT_LOW = 120.0
JUMP_LANDING_SYNC_KP = 180.0
JUMP_LANDING_SYNC_KD = 70.0
JUMP_LANDING_SYNC_LIMIT = 32.0

# Keyboard drive commands use the same LOW speed/yaw envelopes and the same
# wheel yaw feedback law as the firmware.  UI-positive forward is translated
# to the firmware velocity sign at the LQR reference boundary.
DRIVE_SPEED_MAX = 2.30
DRIVE_YAW_RATE_MAX = 3.80
DRIVE_SPEED_KP = 6.0
DRIVE_SPEED_TRIM_LIMIT = 3.00
DRIVE_SPEED_TRIM_SLEW = 16.0
DRIVE_SPEED_DEADBAND = 0.02
# Cruise governor (drive-speed channel above them kept for reference).  The
# six-state LQR's velocity row alone behaves as a slow runaway under any
# nonzero request, so this small common-wheel loop closes commanded speed.
# It is deliberately subordinate: capped far below wheel saturation, slewed,
# and bypassed at stand so it can never fight the zero-command balance.
LQR_XDOT_SIGN = -1.0
LQR_XDOT_SIGN = +1.0
# Ownership transfer: while a real command is active the six-state velocity
# row receives a ZERO reference (its own equilibrium is unreachable and ran
# away), and the cruise governor below closes the true measured speed alone.
DRIVE_CRUISE_ZERO_REF = False
DRIVE_CRUISE_KP = 3.0
DRIVE_CRUISE_KI = 6.0
DRIVE_CRUISE_I_LIMIT = 1.5
DRIVE_CRUISE_LIMIT = 1.20
DRIVE_CRUISE_SLEW = 8.0
DRIVE_CRUISE_MIN_COMMAND = 99.0  # governor disarmed pending boundary-sign resolution (see DESCENT_NOTES)
DRIVE_CRUISE_VX_BYPASS = 2.60
# Ledge-descent guard.  A falling wheel is visible in the reconstructed
# normal force a few control ticks before the debounced off-ground edge.  Use
# that lead time to remove forward speed, then keep a bounded common braking
# torque through the LOW landing buffer.  The guard is transparent on level
# ground and only releases after a complete drop has settled.
DESCENT_GUARD_FN_TRIGGER = 49.5
DESCENT_GUARD_MIN_SPEED = 0.08
DESCENT_GUARD_BRAKE_LIMIT = WHEEL_TORQUE_LIMIT
DESCENT_MODE_SPEED_LIMIT = 0.20
DESCENT_EDGE_CREEP_SPEED = 0.20
DESCENT_EDGE_CREEP_TRIM_LIMIT = 1.10
DESCENT_EDGE_WHEEL_TORQUE = 0.35
DESCENT_OFFGROUND_CONTACT_DRIVE_TORQUE = 0.55
DESCENT_GUARD_PREEDGE_TIMEOUT = 0.80
DESCENT_GUARD_EDGE_LOOKAHEAD = 0.25
DESCENT_GUARD_EDGE_HOLD = 0.05
# Shorten the linkage before the wheel reaches the ledge.  This is the
# mechanically useful clearance motion of the real vehicle: keeping a long
# leg extended while the chassis drops lets the upper tread catch the leg
# body, which turns a normal edge crossing into a pitch impulse.  The
# operator's selected length is restored by the ordinary ground loop after
# the landing buffer has settled.
DESCENT_EDGE_RETRACT_LENGTH = 0.130
DESCENT_EDGE_RETRACT_RATE = 0.80
# The wheel support disappears before the base has moved past the upper
# tread.  A small common hip moment, applied while the geometric edge guard
# is active, counters the repeatable forward-pitch impulse of that handoff;
# it is a torque feed-forward, not a kinematic pose correction.
DESCENT_EDGE_PITCH_BIAS = 10.0
# A geometric edge loss is already a physical confirmation in the platform
# scene: both wheel contact masks are empty, so waiting several more control
# ticks leaves the normal ground VMC applying its old support load during the
# first part of the drop.  Handoff on the next control update instead; the
# force observer remains available for non-geometric takeoffs.
DESCENT_EDGE_TAKEOFF_CONFIRM = 1
# Kept configurable for regression sweeps: the strict force observer remains
# the default authority, while the edge-transition handoff can be compared
# against it without editing the controller.
DESCENT_UNILATERAL_TAKEOFF_ENABLE = True
YAW_RATE_FF = 0.34
YAW_RATE_FB_KP = 0.80
YAW_RATE_FB_KI = 0.60
YAW_RATE_I_LIMIT = 0.80
YAW_OUTPUT_SLEW_RATE = 25.0

# Directly copied from leg_config_t in chassis_task.c.  They are used only to
# subtract the already-modelled gas-spring contribution from VMC support.
VMC_L1 = 0.210
VMC_L2 = 0.250
GAS_L6 = 0.05030
GAS_LS2 = 0.20209
GAS_ALPHA = 0.214218
# Exact motor-zero conversion in ``drv_leg.c`` / ``rm_config.h``.  At four
# raw motor angles of zero the firmware VMC evaluates to
# l0=0.3325570820266586 m and phi0=18.495 deg on both legs.
VMC_DM_ZERO_PHI1 = np.radians(67.19)
VMC_DM_ZERO_PHI4 = np.radians(-30.20)
VMC_ENCODER_ZERO_L0 = 0.33255708202665857
VMC_ENCODER_ZERO_PHI0 = np.radians(18.495)

# User-verified CAD calibration.  J1/J2 in the viewer are the opposite VMC
# arm order: CAD J2 feeds firmware phi1 (FRONT), CAD J1 feeds phi4 (BACK).
# The first visual placement was swing=-76 deg.  Anchoring it to the exact
# firmware phi0 above and the two verified vertical poses refines the common
# swing coordinate by -0.505 deg, so both vertical endpoints solve to 90 deg.
VMC_ENCODER_ZERO_SWING = np.radians(-76.505)
VMC_ENCODER_ZERO_COMPRESSION = np.radians(27.0)
VMC_ENCODER_ZERO_CAD_Q = {
    "r_j1": VMC_ENCODER_ZERO_SWING + VMC_ENCODER_ZERO_COMPRESSION,
    "r_j2": VMC_ENCODER_ZERO_SWING - VMC_ENCODER_ZERO_COMPRESSION,
    "l_j1": -VMC_ENCODER_ZERO_SWING - VMC_ENCODER_ZERO_COMPRESSION,
    "l_j2": -VMC_ENCODER_ZERO_SWING + VMC_ENCODER_ZERO_COMPRESSION,
}
VMC_VERTICAL_SWING = np.radians(-5.0)
VMC_COMPRESSION_MIN = np.radians(-20.0)
VMC_COMPRESSION_MAX = np.radians(27.0)
VMC_LENGTH_MIN = 0.11639905741242806
VMC_LENGTH_MAX = VMC_ENCODER_ZERO_L0
VMC_NORMAL_STAND_LENGTH = 0.150
# Interactive low-leg presets.  The calibrated 0.150 m pose remains the
# controller's safe initialization pose; the active standing reference can be
# switched gradually to one of these three operator-selected lengths.
LEG_LENGTH_PRESETS = (0.130, 0.240, 0.300)
LEG_LENGTH_DEFAULT = LEG_LENGTH_PRESETS[0]
LEG_LENGTH_SWITCH_RATE = 0.30
VMC_NORMAL_STAND_COMPRESSION = np.radians(-10.573589924936163)
# User calibration: backward body pitch is positive.  The embedded LQR uses
# the negated IMU pitch in state element 4.
PITCH_BACKWARD_SIGN = -1.0
JOINT_FORWARD_SIGN = {"r_j1": 1.0, "r_j2": 1.0, "l_j1": -1.0, "l_j2": -1.0}
# Real DM multi-turn readings captured in the user's verified code-zero
# screenshot.  They are encoder offsets at the CAD geometric q=0 pose, not
# MJCF hinge coordinates.  Inserting these 5 rad values into qpos folds the
# legs and moves both wheel contacts about 0.27 m ahead of the centre of mass.
CODE_ZERO_ENCODER_RAD = {
    "r_j1": 5.349,
    "r_j2": 4.615,
    "l_j1": -5.310,
    "l_j2": -4.593,
}
# Keep the earlier multi-turn readings as the display translation at CAD
# q=0.  They are not firmware VMC zeros; the refined firmware-zero pose is
# ``VMC_ENCODER_ZERO_CAD_Q`` above.
CODE_ZERO_POSE_PATH = Path(__file__).resolve().parent / "code_zero_pose.json"
ENCODER_ZERO_POSE_PATH = Path(__file__).resolve().parent / "encoder_zero_pose.json"
RECOVERY_CROUCH_POSE_PATH = Path(__file__).resolve().parent / "recovery_crouch_pose.json"
CALIBRATED_STAND_POSE_PATH = Path(__file__).resolve().parent / "calibrated_stand_pose.json"
WHEEL_RADIUS_M = 0.058
# The chassis task in the embedded project runs at 5 ms.  Physics stays at
# MuJoCo's 1 ms step; feedback is sampled and held at this task rate.
CONTROL_PERIOD_S = 0.005
# chassis_control.c integrates the held-position state with X_RATIO=3 while
# feeding the unscaled velocity estimate into state 3.  Firmware-positive vx
# is opposite this MJCF export's calibrated world-forward direction.
LQR_POSITION_X_RATIO = 3.0
LQR_POSITION_CAPTURE_VX_MAX = 0.05
LQR_POSITION_CAPTURE_TIME = 0.10
LQR_POSITION_LIMIT = 1.50
LQR_POSITION_BRAKE_RELEASE_RATE = 1.50
THETA_DX_OFFSET_M = 0.013
THETA_COMP_APPLY_RATE = 1.5
THETA_COMP_POLY_L0_MIN = 0.130
THETA_COMP_POLY_L0_MAX = 0.300
THETA_COMP_POLY_L0_CENTER = 0.215
THETA_COMP_POLY_L0_SCALE = 0.085
THETA_COMP_POLY_C0 = 0.010312509
THETA_COMP_POLY_C1 = -0.009055911
THETA_COMP_POLY_C2 = 0.001545024
RECOVERY_SETTLE_S = 0.50
RECOVERY_STABLE_S = 0.15
RECOVERY_LQR_INITIAL_GAIN = 0.35
RECOVERY_LQR_HANDOFF_RATE = 8.0
RECOVERY_LENGTH_KP = 1200.0
RECOVERY_LENGTH_KD = 30.0
RECOVERY_FORCE_MIN = -100.0
RECOVERY_FORCE_MAX = 20.0
RECOVERY_TP_KP = 32.0
RECOVERY_TP_KD = 5.5
RECOVERY_TP_LIMIT = 10.0
RECOVERY_LENGTH_TOLERANCE = 0.006
RECOVERY_PHI_TOLERANCE = np.radians(2.0)

# Upright encoder-zero recovery, copied from chassis_recovery.c's active
# RECOVERY_SIMPLE path.  The simulated normal stand is the user-calibrated
# 0.150 m pose, so that replaces firmware LEN_LEN_LOW (0.130 m); all motion
# profiles, virtual-force gains, torque reserves, and handoff gains retain
# their source values.
FW_RECOVERY_HOLD_TIME_S = 0.22
FW_RECOVERY_PHI_PREPOSE = np.radians(30.0)
FW_RECOVERY_PHI_FINAL = np.radians(80.0)
FW_RECOVERY_PHI_PREPOSE_TOLERANCE = np.radians(4.0)
FW_RECOVERY_SWING_RATE = np.radians(135.0)
FW_RECOVERY_DEADPOINT_SWING_RATE = np.radians(70.0)
FW_RECOVERY_COMPACT_SWING_RATE = np.radians(150.0)
FW_RECOVERY_SWING_ACCEL = np.radians(360.0)
FW_RECOVERY_COMPACT_SWING_ACCEL = np.radians(420.0)
FW_RECOVERY_SWING_KP = 70.0
FW_RECOVERY_SWING_KD = 1.6
FW_RECOVERY_SWING_FF = 3.0
FW_RECOVERY_SWING_TP_LIMIT = 15.0
FW_RECOVERY_SWING_GAIN_RATE = 1.5
FW_RECOVERY_TP_GAIN_RATE = 2.0
FW_RECOVERY_TP_SLEW_RATE = 120.0
FW_RECOVERY_TERMINAL_TP_SLEW_RATE = 320.0
FW_RECOVERY_OMEGA_FILTER_TAU = 0.025
FW_RECOVERY_L0_TARGET = VMC_NORMAL_STAND_LENGTH
FW_RECOVERY_L0_RATE = 0.25
FW_RECOVERY_L0_OK = 0.170
FW_RECOVERY_L0_KP = 1200.0
FW_RECOVERY_L0_KD = 30.0
FW_RECOVERY_FORCE_GAIN_RATE = 0.9
FW_RECOVERY_FORCE_LIMIT = 100.0
FW_RECOVERY_FORCE_EXTEND_LIMIT = 20.0
FW_RECOVERY_FORCE_BIAS = 40.0
FW_RECOVERY_FORCE_BIAS_RANGE = 0.08
FW_RECOVERY_SPRING_FF_LIMIT = 140.0
FW_RECOVERY_BREAKAWAY_RELEASE_L0 = 0.285
FW_RECOVERY_BREAKAWAY_RELEASE_BAND = 0.020
FW_RECOVERY_BREAKAWAY_MIN_TIME = 0.050
FW_RECOVERY_BREAKAWAY_RISE_TIME = 0.020
FW_RECOVERY_BREAKAWAY_TIME = 0.180
FW_RECOVERY_BREAKAWAY_FORCE = 180.0
FW_RECOVERY_BREAKAWAY_TORQUE = 24.0
FW_RECOVERY_TORQUE_RESERVE = 19.0
FW_RECOVERY_COMPACT_L0 = 0.200
FW_RECOVERY_SETTLE_L0_START = 0.200
FW_RECOVERY_SETTLE_L0_END = 0.180
FW_RECOVERY_SETTLE_KP = 32.0
FW_RECOVERY_SETTLE_KD = 5.5
FW_RECOVERY_SETTLE_TP_LIMIT = 10.0
FW_RECOVERY_STABLE_L0_RATE = 0.06
FW_RECOVERY_STABLE_PHI_RATE = 1.20
FW_RECOVERY_STABLE_PHI_ERROR = np.radians(10.0)
FW_RECOVERY_STABLE_FORCE = 35.0
FW_RECOVERY_STABLE_TP = 5.0
FW_RECOVERY_STABLE_TIME_S = 0.015
FW_RECOVERY_RETRACT_TIMEOUT_S = 5.0
# Upright recovery now uses the real firmware force and trajectory values
# directly.  Missing motor/friction dynamics belong in the plant model, not
# in controller-side force or motion multipliers.
# Legacy gas-spring trim from before the three-point VMC calibration.  It is
# retained only so the old diagnostic scripts still import; it is not an
# accepted LQR equilibrium and the UI keeps that LQR route locked.
CAD_GAS_TRIM = {
    "l": {"l0": 0.24317031410024417, "phi0": 2.086879401680555,
          "force_ff": -0.57013264, "tp_ff": -2.67492262},
    "r": {"l0": 0.24130135403914826, "phi0": 2.1049236671263576,
          "force_ff": -5.73459966, "tp_ff": 2.73806389},
}
CAD_ZERO_GAS_TRIM = {
    "l": {"l0": 0.24333433303611274, "phi0": np.pi / 2.0,
          "force_ff": -3.1085, "tp_ff": 0.3357},
    "r": {"l0": 0.24294728564005058, "phi0": np.pi / 2.0,
          "force_ff": -0.6160, "tp_ff": -0.4079},
}
CAD_STAND_RAMP_S = 1.5
RECOVERY_CAD_STAND_LQR_DELAY_S = 2.5
CAD_DIRECT_START_ELAPSED_S = 1.2
CAD_STAND_LENGTH_KP = 30.0
CAD_STAND_LENGTH_KD = 2.0
CAD_STAND_PHI_KP = 6.0
CAD_STAND_PHI_KD = 0.2
CAD_ZERO_JOINT_TOLERANCE_RAD = 1e-7
# Discrete wheel LQR from a 5 ms finite-difference linearisation at
# CAD_GAS_TRIM. State order is [backward pitch, pitch rate, x, x rate].
# The 2x authority was validated from the actual no-contact CAD reset: it
# keeps the complete startup transient below 4.5 degrees in both symmetric
# pulse directions instead of allowing the chassis to fall onto the floor.
CAD_STAND_WHEEL_K = 2.0 * np.array((-5.63055, 0.14467, -1.66558, -0.44693))
CAD_STAND_PITCH_REFERENCE = np.radians(-0.008966215935189006)

# Empirically validated physical self-righting pulse.  These are VMC-space
# commands, passed through the same Jacobian/torque limits as every other leg
# command.  They are held only while the chassis is prone; completion requires
# both an upright pitch window and real floor contact before LQR is enabled.
PRONE_SWING_FORCE = -80.0
PRONE_SWING_TP = -12.0
PRONE_SWING_WHEEL_TORQUE = -WHEEL_TORQUE_LIMIT
PRONE_ENTRY_PITCH = np.radians(60.0)
PRONE_HANDOFF_PITCH = np.radians(20.0)
PRONE_HANDOFF_MIN_TIME_S = 1.50
PRONE_HANDOFF_MIN_CONTACTS = 2
PRONE_SWING_TIMEOUT_S = 8.0

# Firmware normal-control, LEG_MID, l0-polynomial coefficients.
# Each entry is [l0^3, l0^2, l0, 1], in LQR state order
# [theta, theta_dot, x, x_dot, body_pitch, body_pitch_dot].
MID_COEFFICIENTS = np.array([
    [[43.142026, -33.525025, -0.505906, -19.051889], [8.915141, -7.894468, -0.024500, -4.673016], [0.562547, -0.400972, -0.021748, -0.516287], [0.839415, -0.608516, -0.043039, -1.507059], [46.589231, -35.674413, -0.263574, 7.487381], [7.210225, -5.537124, -0.034589, 1.242157]],
    [[9.296017, -8.700764, 0.800062, 7.554579], [7.709849, -6.260654, 0.157116, 1.469995], [0.602449, -0.515614, 0.027895, 0.096545], [1.931830, -1.588498, 0.058392, 0.293536], [-15.062438, 9.980861, 0.775197, 35.935301], [-2.608729, 1.781326, 0.109786, 5.458628]],
], dtype=float)

# Firmware normal-control LEG_LOW gain polynomial.  Recovery hands off into
# this set, rather than the medium-leg controller above.
LOW_COEFFICIENTS = np.array([
    [[44.916491, -35.029826, -0.513056, -19.580009], [9.437026, -8.422520, -0.013035, -4.818016], [0.595311, -0.423460, -0.023375, -0.528522], [1.071533, -0.777102, -0.051666, -1.640241], [51.616394, -39.448117, -0.298388, 8.137242], [8.735194, -6.686198, -0.045734, 1.468985]],
    [[10.686609, -9.829563, 0.846726, 7.607636], [8.038049, -6.525741, 0.168430, 1.483438], [0.657245, -0.556778, 0.028162, 0.101608], [2.198227, -1.799988, 0.064107, 0.325382], [-16.337782, 10.735298, 0.878556, 39.530768], [-2.982729, 1.999001, 0.141819, 6.638457]],
], dtype=float)


def unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < 1e-9:
        raise ValueError("A required controller direction is degenerate.")
    return vector / length


def encoder_joint_reading(model: mujoco.MjModel, data: mujoco.MjData, joint_name: str) -> float:
    """Translate one MJCF hinge coordinate into the calibrated DM frame."""
    if joint_name not in CODE_ZERO_ENCODER_RAD:
        raise KeyError(f"No calibrated encoder zero for {joint_name}")
    joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    q = float(data.qpos[model.jnt_qposadr[joint]])
    return CODE_ZERO_ENCODER_RAD[joint_name] + JOINT_FORWARD_SIGN[joint_name] * q


def apply_saved_pose(model: mujoco.MjModel, data: mujoco.MjData, pose_path: Path) -> None:
    """Load a complete loop-closed pose and place its lowest wheel on ground."""
    if not pose_path.is_file():
        raise FileNotFoundError(f"Missing saved pose: {pose_path}")
    pose = json.loads(pose_path.read_text(encoding="utf-8"))
    base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    base_address = int(model.jnt_qposadr[base_joint])
    data.qpos[base_address:base_address + 7] = pose["base_free"]
    for joint in range(1, model.njnt):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint)
        if name in pose:
            data.qpos[model.jnt_qposadr[joint]] = pose[name]
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    wheel_heights = []
    for side in ("l", "r"):
        wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel")
        wheel_heights.append(float(data.xpos[wheel, 2] - WHEEL_RADIUS_M))
    # The calibrated joint pose is independent of global translation. Put its
    # lowest wheel exactly on the z=0 ground plane before starting a mode.
    data.qpos[base_address + 2] -= min(wheel_heights)
    mujoco.mj_forward(model, data)


def apply_code_zero_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Load the CAD pose corresponding to the calibrated encoder screenshot.

    At this pose the MJCF drive hinges are zero while ``encoder_joint_reading``
    reports r_j1/r_j2=+5.349/+4.615 and l_j1/l_j2=-5.310/-4.593 rad.
    """
    data.qpos[:] = model.qpos0
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    wheel_heights = []
    for side in ("l", "r"):
        wheel = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_wheel")
        wheel_heights.append(float(data.xpos[wheel, 2] - WHEEL_RADIUS_M))
    base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    data.qpos[int(model.jnt_qposadr[base_joint]) + 2] -= min(wheel_heights)
    mujoco.mj_forward(model, data)


def apply_encoder_zero_pose(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Load the calibrated real-encoder zero with both four-bars closed.

    This is the verified ``l0=0.332557082 m, phi0=18.495 deg`` pose.  It is
    intentionally separate from the legacy CAD-q-zero experiment above,
    which predates the user's three-pose VMC calibration.
    """
    apply_saved_pose(model, data, ENCODER_ZERO_POSE_PATH)


def _axis_angle_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    """Return the active 3-D rotation about a unit-independent axis."""
    axis = unit(axis)
    skew = np.array(((0.0, -axis[2], axis[1]),
                     (axis[2], 0.0, -axis[0]),
                     (-axis[1], axis[0], 0.0)))
    return np.eye(3) + np.sin(angle) * skew + (1.0 - np.cos(angle)) * (skew @ skew)


def apply_prone_zero_pose(model: mujoco.MjModel, data: mujoco.MjData, controller: "TraditionalLQR", direction: float = -1.0) -> None:
    """Place CAD q=0 on its chassis face for a physical self-righting test.

    Only the initial free-base pose is set.  Every following recovery motion
    is produced by torque actuators, the gas springs, and mesh contacts.
    ``direction`` accepts either front/back prone orientation; both were
    validated against the same pulse and contact-gated LQR handoff.
    """
    apply_code_zero_pose(model, data)
    controller.reset(data, calibrate_frame=True)
    _, _, lateral = controller._frame(data)
    base_rotation = data.xmat[controller.base].reshape(3, 3)
    quaternion = np.empty(4)
    mujoco.mju_mat2Quat(
        quaternion,
        (_axis_angle_rotation(lateral, float(np.sign(direction) or -1.0) * np.pi / 2.0) @ base_rotation).ravel(),
    )
    base_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base_free")
    base_address = int(model.jnt_qposadr[base_joint])
    data.qpos[base_address + 3:base_address + 7] = quaternion
    # The exterior is asymmetric.  These values place the two face-down
    # orientations at a 2 mm mesh-contact preload, not embedded in the floor.
    data.qpos[base_address + 2] = 0.2365 if direction < 0.0 else 0.3605
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    controller.reset(data)  # retain the calibrated fixed world-forward axis


@dataclass
class SideIds:
    j1: int
    j2: int
    wheel_body: int
    wheel_joint: int
    wheel_motor: int
    hip_motors: tuple[int, int]
    gas_thigh_site: int
    gas_leg_site: int


@dataclass
class VmcKinematics:
    """The same four-bar variables and VMC Jacobian used by drv_leg.c."""

    l0: float
    phi0: float
    phi1: float
    phi2: float
    phi3: float
    phi4: float
    jacobian: np.ndarray


@dataclass
class RecoverySwingAxis:
    actual: float
    start_actual: float
    last_raw: float
    ref: float
    target: float
    omega_actual: float = 0.0
    omega_ref: float = 0.0
    tp_cmd: float = 0.0

    @property
    def direction(self) -> float:
        return 1.0 if self.target >= self.start_actual else -1.0


class TraditionalLQR:
    """Firmware traditional LQR, position observer, and VMC support loop.

    The firmware's wheel torque path is copied directly: left receives ``-T``
    and right receives ``+T``.  The real DM multi-turn zero is translated by
    ``encoder_joint_reading``; feedback and closed-loop geometry stay in the
    MJCF CAD hinge frame.
    """

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

    def _ground_edge_distance_ahead(
        self, data: mujoco.MjData, direction: np.ndarray
    ) -> float | None:
        """Return the nearest contacted box-terrain edge along ``direction``.

        A force-only ledge detector is too noisy for this model: suspension
        motion on a flat tread can briefly reduce the reconstructed force by
        the same amount as a real drop.  The platform terrain is represented
        by fixed box geoms, so use the actual contacted box boundary as a
        geometric confirmation before applying the high-authority brake.
        ``None`` means the current ground is a plane or another terrain type
        without a usable box boundary.
        """
        direction = np.asarray(direction, dtype=float)
        norm = float(np.linalg.norm(direction[:2]))
        if norm <= 1e-9:
            return None
        direction = direction / norm
        wheel_bodies = {
            ids.wheel_body for ids in self.sides.values()
        }
        nearest: float | None = None
        for contact_index in range(data.ncon):
            contact = data.contact[contact_index]
            geom_ids = (int(contact.geom1), int(contact.geom2))
            terrain_ids = [
                geom_id for geom_id in geom_ids
                if geom_id in self.ground_geoms
                and self.model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_BOX
            ]
            wheel_ids = [
                geom_id for geom_id in geom_ids
                if int(self.model.geom_bodyid[geom_id]) in wheel_bodies
            ]
            if not terrain_ids or not wheel_ids:
                continue
            wheel_body = int(self.model.geom_bodyid[wheel_ids[0]])
            wheel_position = data.xpos[wheel_body]
            for terrain_id in terrain_ids:
                rotation = data.geom_xmat[terrain_id].reshape(3, 3)
                local_position = rotation.T @ (
                    wheel_position - data.geom_xpos[terrain_id]
                )
                local_direction = rotation.T @ direction
                distances: list[float] = []
                for axis in range(2):
                    component = float(local_direction[axis])
                    if component > 1e-6:
                        distances.append(
                            (self.model.geom_size[terrain_id, axis]
                             - local_position[axis]) / component
                        )
                    elif component < -1e-6:
                        distances.append(
                            (-self.model.geom_size[terrain_id, axis]
                             - local_position[axis]) / component
                        )
                if not distances:
                    continue
                distance = min(distances)
                if distance >= -0.05 and (
                    nearest is None or distance < nearest
                ):
                    nearest = max(0.0, float(distance))
        return nearest

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

    def _base_vertical_accel_raw(self, data: mujoco.MjData) -> float:
        """World-vertical acceleration of the free base (incl. gravity)."""
        if getattr(self, "base_lin_z_adr", None) is None:
            try:
                adr = self.model.jnt_qveladr[self._joint("base_free")]
            except Exception:
                adr = 0
            self.base_lin_z_adr = int(adr) + 5
        return float(data.qacc[self.base_lin_z_adr])

    def _begin_generic_airborne_profile(self, data: mujoco.MjData) -> None:
        """Start the non-jump airborne leg profile used by the firmware."""
        self.landing_airborne_elapsed = 0.0
        for side in ("l", "r"):
            length = self.vmc_kinematics(data, side).l0
            if not np.isfinite(length):
                length = self.normal_leg_length
            length = float(np.clip(length, 0.10, 0.34))
            # Do not add the virtual 20 N preload to the measured geometry at
            # the instant of takeoff.  On a ledge the measured length can
            # already be near the four-bar's 0.33 m extension singularity;
            # adding preload there commands still more extension just as the
            # upper tread is leaving the leg.  The air profile may deploy
            # toward 0.27 m later, but it must start no longer than that.
            self.landing_air_deploy_ref[side] = float(np.clip(
                min(length, LANDING_AIR_DEPLOY_LENGTH),
                0.10,
                LANDING_AIR_DEPLOY_LENGTH,
            ))

    def _update_generic_airborne_profile(self, dt: float) -> None:
        if not self.off_ground or dt <= 0.0:
            return
        self.landing_airborne_elapsed += dt
        for side in ("l", "r"):
            self.landing_air_deploy_ref[side] = self._move_towards(
                self.landing_air_deploy_ref[side],
                LANDING_AIR_DEPLOY_LENGTH,
                LANDING_AIR_DEPLOY_REF_RATE * dt,
            )

    def _begin_descent_guard(self) -> None:
        """Arm the edge-speed brake used for an uncommanded ledge drop."""
        self.descent_guard_active = True
        self.descent_guard_drop_seen = False
        self.descent_guard_preedge_elapsed = 0.0
        # Do not change the operator-selected leg length before the wheel has
        # actually crossed the edge.  With a wheel constrained on the upper
        # tread, shortening the four-bar lowers the chassis into that tread;
        # the leg collision then happens before the wheel can drop.  The
        # edge mode below keeps the selected length reference common-mode and
        # hands the wheel a small, direction-aware roll-over torque instead.
        self.descent_edge_profile_active = False
        for side in ("l", "r"):
            self.descent_edge_length_reference[side] = self.normal_leg_length

    def _update_descent_guard(
        self,
        data: mujoco.MjData,
        dt: float,
        fn_avg: float,
        raw_wheel_contacts: dict[str, bool],
        takeoff_edge: bool,
        landing_edge: bool,
    ) -> None:
        """Stop horizontal motion around a generic off-ground event.

        A normal LQR has no terrain preview, so its speed reference remains
        active right up to the ledge.  The force observer provides a short
        preview of a wheel unloading; this guard changes the reference to
        zero before the drop and applies a common wheel brake after contact.
        After a real drop it remains latched until an explicit stop/reset,
        preventing the next step from being reached with residual momentum.
        """
        if dt <= 0.0:
            return
        if getattr(self, "disable_descent_guard", False):
            # Opt-out for climbing scenarios: the uphill wall otherwise
            # reads as a drop edge and the approach brake kills the run-up.
            return

        # chassis_vx is nose-positive after step()'s single frame
        # translation, so the guard's speed sense matches the UI value.  The
        # previous negation made a genuine forward approach read as reverse
        # and the force-preview brake never armed before the ledge.
        ui_speed = self.chassis_vx
        requested_speed = self.drive_forward_target
        moving_towards_command = (
            abs(requested_speed) >= DESCENT_GUARD_MIN_SPEED
            and (
                abs(ui_speed) < 0.02
                or requested_speed * ui_speed >= 0.0
            )
        )
        heading, _ = self._heading(data)
        direction = heading if requested_speed >= 0.0 else -heading
        edge_distance = self._ground_edge_distance_ahead(data, direction)
        force_preview = (
            not self.off_ground
            and not self.descent_guard_active
            and moving_towards_command
            and edge_distance is not None
            and edge_distance <= DESCENT_GUARD_EDGE_LOOKAHEAD
        )
        if force_preview:
            self._begin_descent_guard()
        if takeoff_edge:
            if not self.descent_guard_active:
                self._begin_descent_guard()
            self.descent_mode_active = True
            self.descent_guard_drop_seen = True
        if landing_edge:
            if not self.descent_guard_active:
                self._begin_descent_guard()
            self.descent_guard_drop_seen = True

        if not self.descent_guard_active:
            return

        if self.off_ground:
            self.descent_mode_active = True
            self.descent_guard_drop_seen = True
            return

        if not self.descent_guard_drop_seen:
            self.descent_guard_preedge_elapsed += dt
            # A short force dip can occur without a real ledge.  Do not hold
            # the brake forever in that case; the next force dip will arm it
            # again if the vehicle reaches an actual edge.  A geometric edge
            # is different: while it is still in the look-ahead window the
            # guard must remain armed, otherwise the normal speed loop and
            # the preview guard fight each other and the vehicle dithers in
            # front of the riser.
            edge_still_near = (
                edge_distance is not None
                and edge_distance <= 1.5 * DESCENT_GUARD_EDGE_LOOKAHEAD
            )
            if (
                self.descent_guard_preedge_elapsed
                >= DESCENT_GUARD_PREEDGE_TIMEOUT
                and abs(ui_speed) <= 0.08
                and not edge_still_near
            ):
                self.descent_guard_active = False
                self.descent_guard_preedge_elapsed = 0.0
                self.descent_edge_profile_active = False
            return

        # Once a real drop has been observed, keep the safety latch active.
        # Returning the original drive command automatically is unsafe here:
        # it can reverse the vehicle back toward the last riser before the
        # operator has confirmed the new support surface.
        return

    def _generic_landing_supports(
        self, lengths: dict[str, float]
    ) -> dict[str, float]:
        """Landing-buffer axial support using the real LOW-leg gains."""
        supports: dict[str, float] = {}
        for side in ("l", "r"):
            length = lengths[side]
            spring = self._gas_virtual_support(length) if self.gas_enabled else 0.0
            length_force = float(np.clip(
                LANDING_LENGTH_KP * (self.gbuf_length_ref[side] - length)
                - LANDING_LENGTH_KD_LOW * self.length_rate_lpf[side],
                -LANDING_LENGTH_OUTPUT_LIMIT_LOW,
                LANDING_LENGTH_OUTPUT_LIMIT_LOW,
            ))
            support = (
                0.5 * TOTAL_MASS * GRAVITY
                - spring
                + length_force
                + self.support_trim[side]
                + (self.roll_force if side == "l" else -self.roll_force)
            )
            # A gas spring can make the raw LOW-leg force negative while the
            # wheel has just moved to a lower tread and the chassis is still
            # above it.  Letting that negative force through removes the
            # vertical support exactly when the body is crossing the riser.
            # Keep a small bilateral support floor for the short landing
            # buffer; the normal standing PID resumes after it exits.
            supports[side] = float(np.clip(
                max(support, GBUF_SUPPORT_FLOOR),
                -SUPPORT_FORCE_LIMIT,
                SUPPORT_FORCE_LIMIT,
            ))
        return supports

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

    def _update_danger_check(self, data: mujoco.MjData, dt: float) -> None:
        """Port of chassis_safety.c Security_Checking().

        Hard attitude/leg-range violations are immediate; soft pitch/roll/
        theta candidates debounce while the chassis is grounded.  A confirmed
        danger state with the machine otherwise idle hands control to the
        recovery module (begin_recovery), the ported equivalent of the
        firmware DANGER -> CHASSIS_RECOVERY takeover.
        """
        if dt <= 0.0:
            return
        if getattr(self, "disable_danger_check", False):
            # Opt-out for scripted gaits that intentionally fold the legs.
            return
        pitch_deg = float(np.degrees(self.body_pitch(data)))
        roll_deg = float(np.degrees(self.body_roll(data)))
        reason = "none"
        if abs(pitch_deg) > DANGER_HARD_ATTITUDE_DEG:
            reason = "hard_pitch"
        elif abs(roll_deg) > DANGER_HARD_ATTITUDE_DEG:
            reason = "hard_roll"
        else:
            low = DANGER_LEG_SAFE_AREA_DEG
            high = 180.0 - low
            for side in ("l", "r"):
                phi = float(np.degrees(self.vmc_kinematics(data, side).phi0))
                if phi <= low or phi >= high:
                    reason = f"hard_{side}_phi"
                    break
        # The soft candidate keeps accumulating through touch-and-go
        # bounces: a fall that tumbles across terrain must still trigger the
        # instant the chassis finally settles grounded, not only when the
        # whole 0.08 s window happens to fit inside one contact spell.
        if reason == "none":
            candidate = "none"
            theta_bad = False
            for side in ("l", "r"):
                kin = self.vmc_kinematics(data, side)
                theta = np.pi / 2.0 - kin.phi0 + self.body_pitch(data)
                if abs(theta) > DANGER_SOFT_THETA_RAD:
                    theta_bad = True
                    break
            if abs(pitch_deg) > DANGER_SOFT_PITCH_DEG:
                candidate = "soft_pitch"
            elif abs(roll_deg) > DANGER_SOFT_ROLL_DEG:
                candidate = "soft_roll"
            elif theta_bad:
                candidate = "soft_theta"
            if candidate != "none":
                if candidate != getattr(self, "_danger_candidate", "none"):
                    self._danger_candidate = candidate
                    self._danger_timer = 0.0
                self._danger_timer = getattr(self, "_danger_timer", 0.0) + dt
                if self._danger_timer >= DANGER_SOFT_CONFIRM_TIME:
                    reason = candidate
            else:
                self._danger_candidate = "none"
                self._danger_timer = 0.0

        if reason != "none":
            self.danger_reason = reason
            self.danger_confirmed = True
            if (
                self.recovery_state == "idle"
                and self.jump_state == "idle"
                and self.enabled
                and not self.off_ground
                and not self.jump_off_ground
            ):
                # The firmware chassis task routes DANGER to the recovery
                # module; do the same with the ported self-rescue machine.
                self.begin_fall_recovery(data)
        else:
            self.danger_reason = "none"
            self.danger_confirmed = False

    def _update_off_ground(self, data: mujoco.MjData, dt: float) -> None:
        """Real-vehicle off-ground detection and airborne handling.

        Port of chassis_safety.c: each wheel's vertical contact force is
        reconstructed from the commanded VMC axial force, gas-spring support,
        leg torque and the unsprung-mass vertical acceleration, filtered over
        four samples, and debounced before setting the off-ground flag.
        While airborne the wheels are shut, the hips use the real bounded LQR
        plus theta-hold command, the axial force follows the real 0.27 m
        deploy profile, and the integrators are frozen.  A landing edge starts
        the real LOW-leg COMPRESS->HOLD->RETURN buffer.

        Only active when the jump machine is idle (stair / ledge drops).
        """
        if self.jump_state != "idle":
            return
        if not self.enabled:
            return
        if dt <= 0.0:
            return

        # The start-up window only disarms the Ground_Detect decision below.
        # The support-force reconstruction itself keeps running: freezing the
        # four-sample history instead would leave cold-start values in the
        # filter, and the first live evaluation after the window would then
        # misread a genuine stand-up transient as takeoff (observed as an
        # immediate airborne deploy hop at t~3 s on the platform scene).
        self.offground_suppress_elapsed += dt

        pitch = self.body_pitch(data)
        base_az = self._base_vertical_accel_raw(data)
        raw_wheel_contacts = self._wheel_contacts(data)

        for side in ("l", "r"):
            kin = self.vmc_kinematics(data, side)
            theta = np.pi / 2.0 - kin.phi0 + pitch
            l0 = max(kin.l0, 0.08)
            d_l0 = self.length_rate_lpf[side]
            d_theta = self.theta_rate_lpf[side]
            if self.offground_deriv_init:
                dd_l0 = (d_l0 - self.offground_last_d_l0[side]) / dt
                dd_theta = (d_theta - self.offground_last_d_theta[side]) / dt
            else:
                dd_l0 = 0.0
                dd_theta = 0.0
            self.offground_last_d_l0[side] = d_l0
            self.offground_last_d_theta[side] = d_theta
            leg_vertical_accel = (
                base_az
                - dd_l0 * np.cos(theta)
                + 2.0 * d_l0 * d_theta * np.sin(theta)
                + l0 * dd_theta * np.sin(theta)
                + l0 * d_theta * d_theta * np.cos(theta)
            )
            support = self.last_support[side]
            tp = self.last_output[side][1]
            fn = (
                support * np.cos(theta)
                + self._gas_virtual_support(l0)
                + tp * np.sin(theta) / l0
                + OFFGROUND_UNSPRUNG_MASS * leg_vertical_accel
            )
            self.offground_fn_raw[side] = (
                fn if np.isfinite(fn) else 0.5 * TOTAL_MASS * GRAVITY
            )
            self.offground_fn_history[side][self.offground_filter_index] = (
                self.offground_fn_raw[side]
            )
        self.offground_filter_index = (self.offground_filter_index + 1) % 4
        self.offground_deriv_init = True
        for side in ("l", "r"):
            self.offground_fn_filtered[side] = (
                sum(self.offground_fn_history[side]) / 4.0
            )
        fn_avg = 0.5 * sum(self.offground_fn_filtered.values())
        support_contacts = self._wheel_support_contacts(data)
        # The generic airborne branch normally shuts the drive wheels and
        # uses them only as a reaction axis.  During a guarded stair descent,
        # however, a wheel may regain a real vertical tread contact while the
        # body is still outside the conservative landing attitude basin.  In
        # that short interval a small common drive torque is physically
        # meaningful and lets the wheel roll off the next tread; it is never
        # enabled for a riser scrape or while both wheels are unsupported.
        self.descent_edge_contact_drive = bool(
            self.descent_guard_active
            and self.off_ground
            and any(support_contacts.values())
            and self.gbuf_phase == "inactive"
        )
        heading, _ = self._heading(data)
        direction = (
            heading if self.drive_forward_target >= 0.0 else -heading
        )
        edge_distance = self._ground_edge_distance_ahead(data, direction)
        edge_hold_preview = (
            self.descent_guard_active
            and not self.off_ground
            and edge_distance is not None
            and edge_distance <= DESCENT_GUARD_EDGE_HOLD
        )
        edge_contact_loss = (
            self.descent_guard_active
            and self.descent_edge_transition_active
            and not any(support_contacts.values())
            and not any(raw_wheel_contacts.values())
        )
        if edge_contact_loss:
            self.descent_edge_loss_count += 1
        else:
            self.descent_edge_loss_count = 0
        if self.gbuf_phase != "inactive":
            if all(support_contacts.values()):
                self.gbuf_contact_loss_count = 0
            else:
                self.gbuf_contact_loss_count += 1
        self.descent_edge_transition_active = bool(
            self.descent_guard_active
            and not self.off_ground
            and (
                self.descent_edge_transition_active
                or not all(support_contacts.values())
                or edge_hold_preview
            )
        )

        # debounced state machine (Ground_Detect)
        detector_armed = (
            self.offground_suppress_elapsed >= OFFGROUND_STARTUP_SUPPRESS_S
        )
        if not detector_armed:
            self.off_ground = False
            self.off_ground_cnt = 0
        elif (
            not self.off_ground
            and self.gbuf_phase == "inactive"
        ):
            force_takeoff_candidate = (
                fn_avg < OFFGROUND_TAKEOFF_FN_LOW
                and self.offground_fn_filtered["l"] < OFFGROUND_JUMP_TAKEOFF_SIDE_MAX
                and self.offground_fn_filtered["r"] < OFFGROUND_JUMP_TAKEOFF_SIDE_MAX
            )
            # During the geometric pre-edge clearance motion the gas-spring
            # cancellation can make the reconstructed virtual force negative
            # even though both wheels are still visibly carrying the chassis.
            # That is a support-model transient, not takeoff.  Let the real
            # contact geometry own this particular handoff; otherwise the
            # controller declares airborne while parked at the ledge.
            if (
                self.descent_edge_profile_active
                and self.descent_guard_active
                and not self.descent_guard_drop_seen
            ):
                force_takeoff_candidate = False
            # A ledge drop is often unilateral at first: one wheel is still
            # on the upper tread while the other has already crossed the
            # edge.  Waiting for both reconstructed forces to become small
            # leaves the normal ground VMC in charge during that interval;
            # its wheel/leg loop then treats the riser scrape as a pose error
            # and winds the chassis into roll.  Once the geometric edge guard
            # is active, a real vertical tread on only one wheel is enough to
            # hand the transition to the airborne profile.  The same normal
            # and contact checks are retained for the eventual landing.
            unilateral_edge_candidate = (
                DESCENT_UNILATERAL_TAKEOFF_ENABLE
                and
                self.descent_guard_active
                and self.descent_edge_transition_active
                and any(support_contacts.values())
                and not all(support_contacts.values())
                and any(raw_wheel_contacts.values())
                and edge_distance is not None
                and edge_distance <= DESCENT_GUARD_EDGE_HOLD
            )
            # Once both wheels have actually left the vertical treads, the
            # force observer still contains the previous axial command and
            # can take several hundred milliseconds to fall below its
            # threshold.  Keep the normal LQR from chasing that stale force:
            # the guarded edge transition plus three consecutive contact-free
            # samples is a physical takeoff confirmation for this scene.
            geometry_edge_candidate = (
                self.descent_edge_loss_count
                >= DESCENT_EDGE_TAKEOFF_CONFIRM
            )
            takeoff_candidate = (
                force_takeoff_candidate
                or unilateral_edge_candidate
                or geometry_edge_candidate
            )
            if takeoff_candidate:
                self.off_ground_cnt += 1
                if self.off_ground_cnt >= OFFGROUND_TAKEOFF_CONFIRM_LOW:
                    self.off_ground = True
                    self.off_ground_cnt = 0
            else:
                self.off_ground_cnt = 0
        elif self.off_ground:
            self.descent_edge_transition_active = False
            landing_attitude_stable = (
                abs(pitch) <= OFFGROUND_LANDING_ATTITUDE_MAX
                and abs(self.body_roll(data)) <= OFFGROUND_LANDING_ATTITUDE_MAX
                and abs(float(data.qvel[2]))
                <= OFFGROUND_LANDING_VERTICAL_SPEED_MAX
            )
            force_landing = (
                self.offground_fn_filtered["l"] > OFFGROUND_LANDING_FN_LOW
                and self.offground_fn_filtered["r"] > OFFGROUND_LANDING_FN_LOW
                and all(support_contacts.values())
                and landing_attitude_stable
            )
            if force_landing:
                self.off_ground_cnt += 1
                if self.off_ground_cnt >= OFFGROUND_LANDING_CONFIRM_LOW:
                    self.off_ground = False
                    self.off_ground_cnt = 0
            else:
                self.off_ground_cnt = 0
        else:
            # The LOW landing buffer owns the contact transition.  Its
            # captured support force can dip below the takeoff threshold as
            # the legs compress; allowing Ground_Detect to run again here
            # would re-enter airborne mode in the middle of a landing and
            # produce the observed repeated hop / runaway drive.
            self.off_ground_cnt = 0
            self.off_ground = False
            self.descent_edge_transition_active = False

        # The generic landing buffer is allowed to absorb the first tread,
        # but it must not mask a second edge.  A short debounce filters the
        # occasional missing contact normal from solver contact churn; a
        # sustained loss means the wheels are on a riser or airborne again.
        # Re-open the airborne path before calculating the edge transition so
        # the next landing can create a fresh buffer of its own.
        if (
            detector_armed
            and self.gbuf_phase != "inactive"
            and self.descent_guard_active
            and self.gbuf_contact_loss_count >= GBUF_SUPPORT_LOSS_CONFIRM
        ):
            self.gbuf_phase = "inactive"
            self.gbuf_elapsed = 0.0
            self.gbuf_exit_stable = 0.0
            self.gbuf_contact_loss_count = 0
            self.off_ground = True
            self.off_ground_cnt = 0
            self.descent_edge_transition_active = False
            self.descent_edge_loss_count = 0

        takeoff_edge = self.off_ground and not self.off_ground_last
        landing_edge = (not self.off_ground) and self.off_ground_last
        self.off_ground_last = self.off_ground

        if takeoff_edge:
            self._begin_generic_airborne_profile(data)
        self._update_generic_airborne_profile(dt)

        # landing buffer state machine (generic airborne path)
        if self.gbuf_phase != "inactive":
            self.gbuf_elapsed += dt
            lengths = {
                side: self.vmc_kinematics(data, side).l0 for side in ("l", "r")
            }
            average_speed = 0.5 * sum(
                abs(self.length_rate_lpf[side]) for side in ("l", "r")
            )
            if self.gbuf_phase == "compress":
                # Keep the captured contact geometry during compression.  The
                # real LOW buffer lets the physical leg shorten against that
                # fixed reference; it does not immediately pull the reference
                # back to the normal standing length.
                for side in ("l", "r"):
                    self.gbuf_length_ref[side] = self._move_towards(
                        self.gbuf_length_ref[side],
                        self.gbuf_absorb_ref[side],
                        LANDING_COMPRESS_REF_RATE * dt,
                    )
                if self.gbuf_elapsed >= GBUF_COMPRESS_TIME:
                    self.gbuf_phase = "hold"
                    self.gbuf_elapsed = 0.0
            elif self.gbuf_phase == "hold":
                for side in ("l", "r"):
                    ratchet_target = min(
                        lengths[side] + LANDING_LOW_RATCHET_MARGIN,
                        self.normal_leg_length,
                    )
                    # Never command the reference back beyond the captured
                    # contact geometry while the body is still absorbing the
                    # drop.  Otherwise the measured leg extension ratchets
                    # the reference upward, launches the chassis, and turns
                    # the landing buffer into a second jump.
                    ratchet_target = min(
                        ratchet_target, self.gbuf_length_ref[side]
                    )
                    self.gbuf_length_ref[side] = self._move_towards(
                        self.gbuf_length_ref[side],
                        ratchet_target,
                        LANDING_LOW_RATCHET_REF_RATE * dt,
                    )
                settled = (
                    abs(0.5 * sum(lengths.values())
                        - 0.5 * sum(self.gbuf_length_ref.values()))
                    <= GBUF_SETTLE_LENGTH_ERROR
                    and average_speed <= GBUF_SETTLE_LENGTH_SPEED
                )
                if (
                    self.gbuf_elapsed >= GBUF_HOLD_MAX_TIME
                    or (
                        self.gbuf_elapsed >= GBUF_HOLD_MIN_TIME and settled
                    )
                ):
                    self.gbuf_phase = "return"
                    self.gbuf_elapsed = 0.0
            elif self.gbuf_phase == "return":
                for side in ("l", "r"):
                    self.gbuf_length_ref[side] = self._move_towards(
                        self.gbuf_length_ref[side],
                        self.normal_leg_length,
                        GBUF_RETURN_REF_RATE * dt,
                    )
                target_reached = all(
                    abs(v - self.normal_leg_length) <= 0.0005
                    for v in self.gbuf_length_ref.values()
                )
                if (
                    target_reached
                    and average_speed <= GBUF_SETTLE_LENGTH_SPEED
                ):
                    self.gbuf_exit_stable += dt
                else:
                    self.gbuf_exit_stable = 0.0
                if (
                    target_reached
                    and self.gbuf_elapsed >= GBUF_PID_RETURN_TIME
                    and self.gbuf_exit_stable >= GBUF_EXIT_STABLE_TIME
                ):
                    self.gbuf_phase = "inactive"
                    self.gbuf_elapsed = 0.0
                    self.gbuf_exit_stable = 0.0

        # A ledge guard is scoped to one physical edge.  Once the captured
        # landing reference has returned and the IMU/velocity are settled,
        # release the guard so the operator's still-held command can approach
        # the next step at the normal controlled speed.  An explicit stop is
        # still sufficient to clear it immediately.
        if (
            self.descent_guard_active
            and self.descent_guard_drop_seen
            and not self.off_ground
            and self.gbuf_phase == "inactive"
            and abs(self.chassis_vx) <= DESCENT_GUARD_MIN_SPEED
            and abs(pitch) <= OFFGROUND_LANDING_ATTITUDE_MAX
            and abs(self.body_roll(data)) <= OFFGROUND_LANDING_ATTITUDE_MAX
        ):
            self.descent_guard_active = False
            self.descent_mode_active = False
            self.descent_guard_drop_seen = False
            self.descent_guard_preedge_elapsed = 0.0
            self.descent_edge_profile_active = False

        if landing_edge:
            self.descent_edge_transition_active = False
            self.gbuf_phase = "compress"
            self.gbuf_elapsed = 0.0
            self.gbuf_exit_stable = 0.0
            self.gbuf_contact_loss_count = 0
            refs = {
                side: float(np.clip(
                    self.vmc_kinematics(data, side).l0, 0.10, 0.34
                ))
                for side in ("l", "r")
            }
            ref_max = min(refs.values()) + LANDING_LOW_CONTACT_SPLIT_MAX
            for side in ("l", "r"):
                self.gbuf_contact_length[side] = self.vmc_kinematics(
                    data, side
                ).l0
                refs[side] = min(refs[side], ref_max)
                self.gbuf_length_ref[side] = refs[side]
                self.gbuf_absorb_ref[side] = refs[side]

        self._update_descent_guard(
            data,
            dt,
            fn_avg,
            raw_wheel_contacts,
            takeoff_edge,
            landing_edge,
        )

        if (
            self.descent_edge_transition_active
            and not self.off_ground
            and self.gbuf_phase == "inactive"
        ):
            # The upper tread can stop carrying one wheel a few samples
            # before Ground_Detect declares takeoff.  During that interval a
            # normal hip row is still based on the old ground equilibrium and
            # lets the chassis rotate into the riser.  Apply the same bounded
            # common pitch hold used immediately after takeoff, but keep the
            # axial support and wheel shutoff in their contact-safe modes.
            pitch_rate = self.body_pitch_rate(data)
            edge_pitch_tp = float(np.clip(
                DESCENT_EDGE_PITCH_BIAS
                - OFFGROUND_PITCH_KP * pitch
                - OFFGROUND_PITCH_KD * pitch_rate,
                -OFFGROUND_PITCH_TP_LIMIT,
                OFFGROUND_PITCH_TP_LIMIT,
            ))
            for side in ("l", "r"):
                self.last_output[side][1] = edge_pitch_tp

        # --- airborne / buffer output override ---
        if self.descent_edge_transition_active and self.gbuf_phase == "inactive":
            # The normal LQR can spike when the wheel contact point reaches a
            # sharp box edge.  Pass only a small common roll-over torque to
            # the motors; this keeps the wheel moving over the edge without
            # allowing the contact impulse to become a propulsion command.
            direction_sign = (
                1.0 if self.drive_forward_target >= 0.0 else -1.0
            )
            edge_wheel = -direction_sign * DESCENT_EDGE_WHEEL_TORQUE
            self.last_output["l"][0] = edge_wheel
            self.last_output["r"][0] = edge_wheel
        self.airborne_wheel_shut = self.off_ground
        if self.off_ground:
            # The wheels are still useful in free flight as a reaction wheel:
            # use their angular momentum to oppose chassis pitch while the
            # hips deploy the legs.  This is internal attitude control only;
            # it does not assume a wheel-ground contact.
            pitch_rate = self.body_pitch_rate(data)
            self.offground_wheel_torque = float(np.clip(
                OFFGROUND_WHEEL_PITCH_KP * pitch
                + OFFGROUND_WHEEL_PITCH_KD * pitch_rate,
                -OFFGROUND_WHEEL_TORQUE_LIMIT,
                OFFGROUND_WHEEL_TORQUE_LIMIT,
            ))
            lengths = {
                side: self.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            }
            air_support = {}
            raw_air_tp = {}
            for side in ("l", "r"):
                kin = self.vmc_kinematics(data, side)
                theta = np.pi / 2.0 - kin.phi0 + pitch
                # The real generic airborne branch zeros the wheel row and
                # retains only the hip theta/theta-rate rows.  Do not reuse
                # the already-computed normal-ground Tp here: it also contains
                # body-pitch feedback, which the firmware explicitly removes
                # for non-jump airborne motion.
                lqr_row = self.gain(kin.l0)[1]
                lqr_tp_raw = float(
                    lqr_row[0] * self.last_state[side][0]
                    + lqr_row[1] * self.last_state[side][1]
                )
                lqr_tp = float(np.clip(
                    OFFGROUND_LQR_TP_GAIN * lqr_tp_raw,
                    -OFFGROUND_LQR_TP_LIMIT,
                    OFFGROUND_LQR_TP_LIMIT,
                ))
                theta_hold = float(np.clip(
                    OFFGROUND_THETA_HOLD_KP * theta
                    + OFFGROUND_THETA_HOLD_KD * self.theta_rate_lpf[side],
                    -OFFGROUND_THETA_HOLD_TP_LIMIT,
                    OFFGROUND_THETA_HOLD_TP_LIMIT,
                ))
                pitch_rate = self.body_pitch_rate(data)
                pitch_tp = float(np.clip(
                    -OFFGROUND_PITCH_KP * pitch
                    - OFFGROUND_PITCH_KD * pitch_rate,
                    -OFFGROUND_PITCH_TP_LIMIT,
                    OFFGROUND_PITCH_TP_LIMIT,
                ))
                raw_air_tp[side] = lqr_tp + theta_hold + pitch_tp
                spring = self._gas_virtual_support(kin.l0) if self.gas_enabled else 0.0
                air_force = OFFGROUND_SOFT_FORCE - spring
                deploy_force = float(np.clip(
                    LANDING_AIR_DEPLOY_KP * (
                        self.landing_air_deploy_ref[side] - kin.l0
                    )
                    - LANDING_AIR_DEPLOY_KD * self.length_rate_lpf[side]
                    - OFFGROUND_SOFT_FORCE,
                    -LANDING_AIR_DEPLOY_FORCE_LIMIT,
                    LANDING_AIR_DEPLOY_FORCE_LIMIT,
                ))
                air_support[side] = air_force + deploy_force

            common_support = 0.5 * (air_support["l"] + air_support["r"])
            differential_support = 0.5 * (
                air_support["l"] - air_support["r"]
            )
            self.airborne_length_sync_force = float(np.clip(
                OFFGROUND_LENGTH_SYNC_KP * (lengths["r"] - lengths["l"])
                + OFFGROUND_LENGTH_SYNC_KD * (
                    self.length_rate_lpf["r"] - self.length_rate_lpf["l"]
                ),
                -OFFGROUND_LENGTH_SYNC_LIMIT,
                OFFGROUND_LENGTH_SYNC_LIMIT,
            ))
            air_support["l"] = common_support + differential_support + self.airborne_length_sync_force
            air_support["r"] = common_support - differential_support - self.airborne_length_sync_force
            for side in ("l", "r"):
                self.last_output[side][0] = 0.0
                self.airborne_lqr_tp[side] = float(
                    np.clip(raw_air_tp[side], -OFFGROUND_LQR_TP_LIMIT,
                            OFFGROUND_LQR_TP_LIMIT)
                )
                self.airborne_theta_hold_tp[side] = float(
                    raw_air_tp[side] - lqr_tp
                )
                self.last_output[side][1] = self.airborne_lqr_tp[side]
                self.last_support[side] = float(np.clip(
                    air_support[side],
                    -OFFGROUND_AIR_RETRACT_LIMIT,
                    OFFGROUND_SPRING_COMP_LIMIT,
                ))
            if self.descent_guard_active and any(support_contacts.values()):
                # Apply the load-transfer term after the bilateral air-force
                # clamp.  Before this point both sides can sit at the
                # negative retract limit, which silently discards the entire
                # differential correction and lets the supported wheel tip
                # the chassis.  A riser scrape is excluded by the vertical
                # contact mask.
                edge_roll_force = float(np.clip(
                    OFFGROUND_ROLL_KP_N_PER_DEG
                    * np.degrees(self.body_roll(data))
                    + OFFGROUND_ROLL_KD_N_PER_DEG_S
                    * self.roll_rate_lpf_deg_s,
                    -OFFGROUND_ROLL_FORCE_LIMIT,
                    OFFGROUND_ROLL_FORCE_LIMIT,
                ))
                self.last_support["l"] = float(np.clip(
                    self.last_support["l"] + edge_roll_force,
                    -OFFGROUND_AIR_RETRACT_LIMIT,
                    OFFGROUND_SPRING_COMP_LIMIT,
                ))
                self.last_support["r"] = float(np.clip(
                    self.last_support["r"] - edge_roll_force,
                    -OFFGROUND_AIR_RETRACT_LIMIT,
                    OFFGROUND_SPRING_COMP_LIMIT,
                ))
            air_roll_tp = float(np.clip(
                -OFFGROUND_ROLL_TP_KP_NM_PER_DEG
                * np.degrees(self.body_roll(data))
                - OFFGROUND_ROLL_TP_KD_NM_PER_DEG_S
                * self.roll_rate_lpf_deg_s,
                -OFFGROUND_ROLL_TP_LIMIT,
                OFFGROUND_ROLL_TP_LIMIT,
            ))
            pitch_rate = self.body_pitch_rate(data)
            air_pitch_extra_tp = float(np.clip(
                -OFFGROUND_PITCH_TP_EXTRA_KP_NM_PER_DEG
                * np.degrees(pitch)
                - OFFGROUND_PITCH_TP_EXTRA_KD_NM_PER_DEG_S
                * pitch_rate,
                -OFFGROUND_PITCH_TP_EXTRA_LIMIT,
                OFFGROUND_PITCH_TP_EXTRA_LIMIT,
            ))
            self.last_output["l"][1] += air_roll_tp
            self.last_output["r"][1] -= air_roll_tp
            self.last_output["l"][1] += air_pitch_extra_tp
            self.last_output["r"][1] += air_pitch_extra_tp
        elif self.gbuf_phase != "inactive":
            self.offground_wheel_torque = 0.0
            # Contact is already confirmed by the landing edge.  Keep the
            # wheel LQR alive while the LOW axial landing reference returns.
            lengths = {
                side: self.vmc_kinematics(data, side).l0
                for side in ("l", "r")
            }
            self.last_support = self._generic_landing_supports(lengths)
            for side in ("l", "r"):
                self.length_reference[side] = self.gbuf_length_ref[side]
            if self.descent_guard_active:
                pitch_rate = self.body_pitch_rate(data)
                gbuf_pitch_tp = float(np.clip(
                    -GBUF_PITCH_KP * pitch - GBUF_PITCH_KD * pitch_rate,
                    -GBUF_PITCH_TP_LIMIT,
                    GBUF_PITCH_TP_LIMIT,
                ))
                gbuf_brake = float(np.clip(
                    # A brake must oppose the measured nose-positive
                    # Kept in the firmware-frame sense of the rest of this
                    # port: the accepted stationary-jump dynamics couple
                    # through this exact expression.
                    -GBUF_BRAKE_KP * (-self.chassis_vx),
                    -GBUF_BRAKE_LIMIT,
                    GBUF_BRAKE_LIMIT,
                ))
                for side in ("l", "r"):
                    self.last_output[side][0] = gbuf_brake
                    # The landing buffer owns the hip row.  Adding its
                    # pitch correction on top of the previous normal-LQR
                    # output and theta synchronizer can leave a stale
                    # command of tens of Nm in ``last_output``; the actuator
                    # then saturates and the buffer itself becomes a second
                    # attitude impulse.  Keep only the bounded common pitch
                    # damper while the captured support reference settles.
                    self.last_output[side][1] = gbuf_pitch_tp
            # freeze leg-length / theta integrators (firmware behavior)
        else:
            self.offground_wheel_torque = 0.0
        if self.off_ground or self.gbuf_phase != "inactive":
            for side in ("l", "r"):
                if hasattr(self, "length_integral"):
                    self.length_integral[side] = 0.0

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
        self.length_integral = {"l": 0.0, "r": 0.0}
        self.theta_rate_lpf = {"l": 0.0, "r": 0.0}
        self.length_rate_lpf = {"l": 0.0, "r": 0.0}
        for side in ("l", "r"):
            self.length_reference[side] = VMC_NORMAL_STAND_LENGTH

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
            fitted_length - THETA_COMP_POLY_L0_CENTER
        ) / THETA_COMP_POLY_L0_SCALE
        polynomial = THETA_COMP_POLY_C0 + normalized * (
            THETA_COMP_POLY_C1 + normalized * THETA_COMP_POLY_C2
        )
        geometry = float(np.arcsin(np.clip(
            THETA_DX_OFFSET_M / safe_length,
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
            mujoco.mj_applyFT(self.model, data, -GAS_SPRING_FORCE * direction, np.zeros(3), thigh, thigh_body, data.qfrc_applied)
            mujoco.mj_applyFT(self.model, data, GAS_SPRING_FORCE * direction, np.zeros(3), leg, leg_body, data.qfrc_applied)

    @staticmethod
    def _gas_virtual_support(leg_length: float) -> float:
        """Gas-spring force expressed in the firmware's virtual-leg l0 axis."""
        denominator = 2.0 * VMC_L1 * VMC_L2
        cosine = np.clip((VMC_L1**2 + VMC_L2**2 - leg_length**2) / denominator, -1.0, 1.0)
        theta3 = float(np.arccos(cosine))
        sine3 = max(float(np.sin(theta3)), 1e-5)
        ls = float(np.sqrt(GAS_L6**2 + GAS_LS2**2 - 2.0 * GAS_L6 * GAS_LS2 * np.cos(theta3 - GAS_ALPHA)))
        return GAS_SPRING_FORCE * leg_length * GAS_L6 * GAS_LS2 * np.sin(theta3 - GAS_ALPHA) / (ls * VMC_L1 * VMC_L2 * sine3)

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
                0.5 * TOTAL_MASS * GRAVITY
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
                0.5 * TOTAL_MASS * GRAVITY
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
        # Achieved speed resolved along the commanded direction; chassis_vx
        # is nose-positive, so a reverse command just flips the projection.
        sign = np.sign(cmd)
        achieved = sign * self.chassis_vx
        error = requested - achieved
        if abs(error) <= DRIVE_SPEED_DEADBAND:
            error = 0.0
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
        self.last_output["l"][0] += self.drive_speed_trim
        self.last_output["r"][0] += self.drive_speed_trim

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
                    support = (
                        0.5 * TOTAL_MASS * GRAVITY
                        - spring_support
                        + length_pid
                        + self.support_trim[side]
                        + (self.roll_force if side == "l" else -self.roll_force)
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
