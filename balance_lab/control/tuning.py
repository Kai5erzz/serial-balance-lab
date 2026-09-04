"""Firmware tuning constants for the whole-body controller.

Every constant keeps the name and value it has in the embedded source
(``chassis_task.c`` / ``chassis_control.c`` / ``chassis_recovery.c`` /
``chassis_safety.c`` / ``rm_config.h``) unless a comment documents a
simulation-only deviation.  The controller modules import this module with
``from .tuning import *`` on purpose: the tuning surface is one shared,
flat namespace of firmware names, which keeps the port diffable against the
C source.  Add new constants here, never in the behavioural modules.
"""

from __future__ import annotations

import numpy as np

from ..paths import (
    CALIBRATED_STAND_POSE_PATH,
    CODE_ZERO_POSE_PATH,
    ENCODER_ZERO_POSE_PATH,
    RECOVERY_CROUCH_POSE_PATH,
)

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

DRIVE_CRUISE_MIN_COMMAND = 99.0  # governor disarmed pending boundary-sign resolution (see docs/DESCENT_NOTES.md)

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

__all__ = [
    "GRAVITY",
    "TOTAL_MASS",
    "WHEEL_TORQUE_LIMIT",
    "HIP_TORQUE_LIMIT",
    "GIMBAL_YAW_TORQUE_LIMIT",
    "GIMBAL_YAW_KP",
    "GIMBAL_YAW_KD",
    "GAS_SPRING_FORCE",
    "SUPPORT_FORCE_LIMIT",
    "LQR_RAMP_TIME_S",
    "LENGTH_KP",
    "LENGTH_KI",
    "LENGTH_INTEGRAL_LIMIT",
    "LENGTH_OUTPUT_LIMIT",
    "LENGTH_KD_LOW",
    "VMC_RATE_LPF_RETAIN",
    "ROLL_KP_N_PER_DEG",
    "ROLL_KD_N_PER_DEG_S",
    "ROLL_FORCE_LIMIT",
    "ROLL_DERIVATIVE_LPF_RC_S",
    "ROLL_GAIN_L0_START",
    "ROLL_GAIN_L0_END",
    "ROLL_GAIN_MASS_SCALE",
    "THETA_SYNC_KP_LOW",
    "THETA_SYNC_KP_HIGH",
    "THETA_SYNC_KD_LOW",
    "THETA_SYNC_KD_HIGH",
    "THETA_SYNC_GAIN_L0_START",
    "THETA_SYNC_GAIN_L0_END",
    "THETA_SYNC_GUARD_START",
    "THETA_SYNC_GUARD_END",
    "THETA_SYNC_GUARD_LIMIT",
    "JUMP_COOLDOWN_S",
    "JUMP_PRELOAD_LENGTH",
    "JUMP_THRUST_LENGTH",
    "JUMP_PRELOAD_MIN_TIME",
    "JUMP_PRELOAD_MAX_TIME",
    "JUMP_PRELOAD_READY_LENGTH",
    "JUMP_PRELOAD_READY_RATE",
    "JUMP_THRUST_BOOST_FORCE",
    "JUMP_THRUST_BOOST_TIME",
    "JUMP_THRUST_SUPPORT_FLOOR",
    "JUMP_THRUST_MAX_TIME",
    "JUMP_ABORT_HOLD_TIME",
    "JUMP_ABORT_SETTLE_TIME",
    "JUMP_ABORT_RETRACT_RATE",
    "JUMP_FLIGHT_TIMEOUT",
    "JUMP_LANDING_GROUND_CONFIRM_TIME",
    "JUMP_LANDING_TIMEOUT",
    "JUMP_LANDING_PITCH_MAX",
    "JUMP_LANDING_ROLL_MAX",
    "JUMP_LANDING_LENGTH_DIFF_MAX",
    "JUMP_LANDING_TIMEOUT_LENGTH_DIFF_MAX",
    "JUMP_PRECHECK_PITCH_MAX",
    "JUMP_PRECHECK_ROLL_MAX",
    "JUMP_SAFE_LAUNCH_VX_MAX",
    "JUMP_SAFE_LAUNCH_COMMAND_MARGIN",
    "JUMP_SAFE_LAUNCH_MOVING_MAX",
    "JUMP_PRECHECK_YAW_RATE_MAX",
    "JUMP_ABORT_PITCH_MAX",
    "JUMP_ABORT_ROLL_MAX",
    "JUMP_LENGTH_KP_PRELOAD",
    "JUMP_LENGTH_KD_PRELOAD",
    "JUMP_LENGTH_LIMIT_PRELOAD",
    "JUMP_LENGTH_KP_THRUST",
    "JUMP_LENGTH_KD_THRUST",
    "JUMP_LENGTH_LIMIT_THRUST",
    "JUMP_LENGTH_SYNC_KP",
    "JUMP_LENGTH_SYNC_KD",
    "JUMP_LENGTH_SYNC_LIMIT",
    "JUMP_THRUST_PITCH_KP",
    "JUMP_THRUST_PITCH_KD",
    "JUMP_THRUST_PITCH_TP_LIMIT",
    "JUMP_THRUST_PITCH_TP_SLEW_RATE",
    "JUMP_THRUST_PITCH_RATE_TARGET_ZERO",
    "JUMP_THRUST_PITCH_RATE_VX_GAIN",
    "JUMP_THRUST_PITCH_RATE_TARGET_MIN",
    "JUMP_THRUST_PITCH_RATE_TARGET_MAX",
    "JUMP_THRUST_PITCH_RATE_START_L0",
    "JUMP_THRUST_PITCH_RATE_FULL_L0",
    "JUMP_THRUST_PITCH_RATE_FILTER_TAU",
    "JUMP_THRUST_THETA_HOLD_KP",
    "JUMP_THRUST_THETA_HOLD_KD",
    "JUMP_THRUST_THETA_HOLD_LIMIT",
    "JUMP_PRETUCK_THETA_REF_RATE",
    "JUMP_THRUST_THETA_YIELD_START",
    "JUMP_THRUST_THETA_YIELD_END",
    "JUMP_THRUST_THETA_LQR_GAIN",
    "JUMP_LANDING_TP_HANDOFF_TIME",
    "JUMP_AIR_LQR_TP_GAIN",
    "JUMP_AIR_TP_LIMIT",
    "DANGER_HARD_ATTITUDE_DEG",
    "DANGER_SOFT_PITCH_DEG",
    "DANGER_SOFT_ROLL_DEG",
    "DANGER_SOFT_THETA_RAD",
    "DANGER_SOFT_CONFIRM_TIME",
    "DANGER_LEG_SAFE_AREA_DEG",
    "FW_RECOVERY_FORWARD_PITCH_MAX",
    "FW_RECOVERY_INVERTED_ENTER_PITCH",
    "FW_RECOVERY_INVERTED_EXIT_PITCH",
    "FW_RECOVERY_INVERTED_CAPTURE_TIME",
    "FW_RECOVERY_INVERTED_PRELOAD_TIME",
    "FW_RECOVERY_INVERTED_TURN_RATE",
    "FW_RECOVERY_INVERTED_TURN_ACCEL",
    "FW_RECOVERY_INVERTED_TURN_MAX_TRAVEL",
    "FW_RECOVERY_INVERTED_SYNC_KP",
    "FW_RECOVERY_INVERTED_SYNC_CORR_LIMIT",
    "FW_RECOVERY_INVERTED_WAIT_ENTER",
    "FW_RECOVERY_INVERTED_WAIT_EXIT",
    "FW_RECOVERY_INVERTED_TP_KP",
    "FW_RECOVERY_INVERTED_TP_KD",
    "FW_RECOVERY_INVERTED_TP_LIMIT",
    "FW_RECOVERY_INVERTED_TP_BIAS",
    "FW_RECOVERY_INVERTED_TURN_GAIN_RATE",
    "FW_RECOVERY_INVERTED_CONTACT_FORCE",
    "FW_RECOVERY_INVERTED_PITCH_PROGRESS_EPS",
    "FW_RECOVERY_INVERTED_PITCH_STALL_TIME",
    "FW_RECOVERY_INVERTED_TURN_TIMEOUT",
    "FW_RECOVERY_INVERTED_SETTLE_TIME",
    "FW_RECOVERY_INVERTED_SETTLE_TP_KP",
    "FW_RECOVERY_INVERTED_SETTLE_TP_KD",
    "FW_RECOVERY_INVERTED_SETTLE_TP_LIMIT",
    "FW_RECOVERY_FORWARD_ZERO_RATE",
    "FW_RECOVERY_FORWARD_ZERO_ACCEL",
    "FW_RECOVERY_FORWARD_ZERO_GAIN_RATE",
    "FW_RECOVERY_FORWARD_ZERO_TP_KP",
    "FW_RECOVERY_FORWARD_ZERO_TP_KD",
    "FW_RECOVERY_FORWARD_ZERO_TP_LIMIT",
    "FW_RECOVERY_FORWARD_ZERO_TP_SIGN",
    "FW_RECOVERY_FORWARD_ZERO_PHI0_OK",
    "FW_RECOVERY_FORWARD_ZERO_STABLE_TIME",
    "FW_RECOVERY_FORWARD_ZERO_TIMEOUT",
    "FW_RECOVERY_SIDE_POSE_MIN",
    "FW_RECOVERY_SIDE_POSE_MAX",
    "FW_RECOVERY_L0_LONG",
    "FW_RECOVERY_PHI_NEAR_ZERO",
    "FW_RECOVERY_ZERO_PITCH_MAX",
    "FW_RECOVERY_FAULT_HOLD_TIME",
    "OFFGROUND_TAKEOFF_FN_LOW",
    "OFFGROUND_JUMP_TAKEOFF_SIDE_MAX",
    "OFFGROUND_TAKEOFF_CONFIRM_LOW",
    "OFFGROUND_STARTUP_SUPPRESS_S",
    "OFFGROUND_LANDING_FN_LOW",
    "OFFGROUND_LANDING_CONFIRM_LOW",
    "OFFGROUND_UNSPRUNG_MASS",
    "OFFGROUND_SOFT_FORCE",
    "OFFGROUND_LQR_TP_GAIN",
    "OFFGROUND_LQR_TP_LIMIT",
    "OFFGROUND_LENGTH_SYNC_KP",
    "OFFGROUND_LENGTH_SYNC_KD",
    "OFFGROUND_LENGTH_SYNC_LIMIT",
    "OFFGROUND_AIR_RETRACT_LIMIT",
    "OFFGROUND_SPRING_COMP_LIMIT",
    "OFFGROUND_THETA_HOLD_KP",
    "OFFGROUND_THETA_HOLD_KD",
    "OFFGROUND_THETA_HOLD_TP_LIMIT",
    "OFFGROUND_PITCH_KP",
    "OFFGROUND_PITCH_KD",
    "OFFGROUND_PITCH_TP_LIMIT",
    "OFFGROUND_PITCH_TP_EXTRA_KP_NM_PER_DEG",
    "OFFGROUND_PITCH_TP_EXTRA_KD_NM_PER_DEG_S",
    "OFFGROUND_PITCH_TP_EXTRA_LIMIT",
    "OFFGROUND_WHEEL_PITCH_KP",
    "OFFGROUND_WHEEL_PITCH_KD",
    "OFFGROUND_WHEEL_TORQUE_LIMIT",
    "OFFGROUND_ROLL_KP_N_PER_DEG",
    "OFFGROUND_ROLL_KD_N_PER_DEG_S",
    "OFFGROUND_ROLL_FORCE_LIMIT",
    "OFFGROUND_ROLL_TP_KP_NM_PER_DEG",
    "OFFGROUND_ROLL_TP_KD_NM_PER_DEG_S",
    "OFFGROUND_ROLL_TP_LIMIT",
    "OFFGROUND_LANDING_ATTITUDE_MAX",
    "OFFGROUND_LANDING_VERTICAL_SPEED_MAX",
    "LANDING_AIR_DEPLOY_LENGTH",
    "LANDING_AIR_DEPLOY_REF_RATE",
    "LANDING_AIR_DEPLOY_KP",
    "LANDING_AIR_DEPLOY_KD",
    "LANDING_AIR_DEPLOY_FORCE_LIMIT",
    "GBUF_COMPRESS_TIME",
    "LANDING_COMPRESS_REF_RATE",
    "GBUF_HOLD_MIN_TIME",
    "GBUF_HOLD_MAX_TIME",
    "GBUF_RETURN_REF_RATE",
    "GBUF_PID_RETURN_TIME",
    "GBUF_EXIT_STABLE_TIME",
    "GBUF_SETTLE_LENGTH_ERROR",
    "GBUF_SETTLE_LENGTH_SPEED",
    "GBUF_PITCH_KP",
    "GBUF_PITCH_KD",
    "GBUF_PITCH_TP_LIMIT",
    "GBUF_BRAKE_KP",
    "GBUF_BRAKE_LIMIT",
    "GBUF_SUPPORT_FLOOR",
    "GBUF_SUPPORT_LOSS_CONFIRM",
    "LANDING_BUFFER_MID_LENGTH",
    "JUMP_AIR_THETA_HOLD_KP",
    "JUMP_AIR_THETA_HOLD_KD",
    "JUMP_AIR_THETA_HOLD_LIMIT",
    "JUMP_AIR_TUCK_LENGTH",
    "JUMP_AIR_TUCK_TIME",
    "JUMP_AIR_TUCK_HOLD_TIME",
    "JUMP_AIR_TUCK_REF_RATE",
    "JUMP_AIR_LANDING_LENGTH",
    "JUMP_AIR_CAPTURE_LENGTH_MAX",
    "JUMP_AIR_DEPLOY_REF_RATE",
    "JUMP_AIR_DEPLOY_KP",
    "JUMP_AIR_DEPLOY_KD",
    "JUMP_AIR_RETRACT_FORCE_LIMIT",
    "JUMP_AIR_RETRACT_FORCE_PEAK",
    "JUMP_AIR_LENGTH_SYNC_KP",
    "JUMP_AIR_LENGTH_SYNC_KD",
    "JUMP_AIR_LENGTH_SYNC_LIMIT",
    "JUMP_TAKEOFF_CONFIRM_COUNT",
    "JUMP_LANDING_CONFIRM_COUNT",
    "JUMP_HIP_MOTOR_TORQUE_LIMIT",
    "LANDING_LOW_CONTACT_SPLIT_MAX",
    "LANDING_LOW_RATCHET_MARGIN",
    "LANDING_LOW_RATCHET_REF_RATE",
    "LANDING_HOLD_MIN_TIME",
    "LANDING_HOLD_MAX_TIME",
    "LANDING_SETTLE_LENGTH_ERROR",
    "LANDING_SETTLE_LENGTH_SPEED",
    "LANDING_RETURN_REF_RATE_LOW",
    "LANDING_PID_RETURN_TIME",
    "LANDING_EXIT_STABLE_TIME",
    "LANDING_EXIT_LENGTH_SPEED",
    "LANDING_CAPTURE_TIME",
    "LANDING_SUPPORT_BLEND_TIME",
    "LANDING_CAPTURE_TP_SLEW_RATE",
    "LANDING_CAPTURE_WHEEL_SLEW_RATE",
    "LANDING_WHEEL_SLEW_RATE",
    "LANDING_LENGTH_KP",
    "LANDING_LENGTH_KD_LOW",
    "LANDING_LENGTH_OUTPUT_LIMIT_LOW",
    "JUMP_LANDING_SYNC_KP",
    "JUMP_LANDING_SYNC_KD",
    "JUMP_LANDING_SYNC_LIMIT",
    "DRIVE_SPEED_MAX",
    "DRIVE_YAW_RATE_MAX",
    "DRIVE_SPEED_KP",
    "DRIVE_SPEED_TRIM_LIMIT",
    "DRIVE_SPEED_TRIM_SLEW",
    "DRIVE_SPEED_DEADBAND",
    "LQR_XDOT_SIGN",
    "LQR_XDOT_SIGN",
    "DRIVE_CRUISE_ZERO_REF",
    "DRIVE_CRUISE_KP",
    "DRIVE_CRUISE_KI",
    "DRIVE_CRUISE_I_LIMIT",
    "DRIVE_CRUISE_LIMIT",
    "DRIVE_CRUISE_SLEW",
    "DRIVE_CRUISE_MIN_COMMAND",
    "DRIVE_CRUISE_VX_BYPASS",
    "DESCENT_GUARD_FN_TRIGGER",
    "DESCENT_GUARD_MIN_SPEED",
    "DESCENT_GUARD_BRAKE_LIMIT",
    "DESCENT_MODE_SPEED_LIMIT",
    "DESCENT_EDGE_CREEP_SPEED",
    "DESCENT_EDGE_CREEP_TRIM_LIMIT",
    "DESCENT_EDGE_WHEEL_TORQUE",
    "DESCENT_OFFGROUND_CONTACT_DRIVE_TORQUE",
    "DESCENT_GUARD_PREEDGE_TIMEOUT",
    "DESCENT_GUARD_EDGE_LOOKAHEAD",
    "DESCENT_GUARD_EDGE_HOLD",
    "DESCENT_EDGE_RETRACT_LENGTH",
    "DESCENT_EDGE_RETRACT_RATE",
    "DESCENT_EDGE_PITCH_BIAS",
    "DESCENT_EDGE_TAKEOFF_CONFIRM",
    "DESCENT_UNILATERAL_TAKEOFF_ENABLE",
    "YAW_RATE_FF",
    "YAW_RATE_FB_KP",
    "YAW_RATE_FB_KI",
    "YAW_RATE_I_LIMIT",
    "YAW_OUTPUT_SLEW_RATE",
    "VMC_L1",
    "VMC_L2",
    "GAS_L6",
    "GAS_LS2",
    "GAS_ALPHA",
    "VMC_DM_ZERO_PHI1",
    "VMC_DM_ZERO_PHI4",
    "VMC_ENCODER_ZERO_L0",
    "VMC_ENCODER_ZERO_PHI0",
    "VMC_ENCODER_ZERO_SWING",
    "VMC_ENCODER_ZERO_COMPRESSION",
    "VMC_ENCODER_ZERO_CAD_Q",
    "VMC_VERTICAL_SWING",
    "VMC_COMPRESSION_MIN",
    "VMC_COMPRESSION_MAX",
    "VMC_LENGTH_MIN",
    "VMC_LENGTH_MAX",
    "VMC_NORMAL_STAND_LENGTH",
    "LEG_LENGTH_PRESETS",
    "LEG_LENGTH_DEFAULT",
    "LEG_LENGTH_SWITCH_RATE",
    "VMC_NORMAL_STAND_COMPRESSION",
    "PITCH_BACKWARD_SIGN",
    "JOINT_FORWARD_SIGN",
    "CODE_ZERO_ENCODER_RAD",
    "WHEEL_RADIUS_M",
    "CONTROL_PERIOD_S",
    "LQR_POSITION_X_RATIO",
    "LQR_POSITION_CAPTURE_VX_MAX",
    "LQR_POSITION_CAPTURE_TIME",
    "LQR_POSITION_LIMIT",
    "LQR_POSITION_BRAKE_RELEASE_RATE",
    "THETA_DX_OFFSET_M",
    "THETA_COMP_APPLY_RATE",
    "THETA_COMP_POLY_L0_MIN",
    "THETA_COMP_POLY_L0_MAX",
    "THETA_COMP_POLY_L0_CENTER",
    "THETA_COMP_POLY_L0_SCALE",
    "THETA_COMP_POLY_C0",
    "THETA_COMP_POLY_C1",
    "THETA_COMP_POLY_C2",
    "RECOVERY_SETTLE_S",
    "RECOVERY_STABLE_S",
    "RECOVERY_LQR_INITIAL_GAIN",
    "RECOVERY_LQR_HANDOFF_RATE",
    "RECOVERY_LENGTH_KP",
    "RECOVERY_LENGTH_KD",
    "RECOVERY_FORCE_MIN",
    "RECOVERY_FORCE_MAX",
    "RECOVERY_TP_KP",
    "RECOVERY_TP_KD",
    "RECOVERY_TP_LIMIT",
    "RECOVERY_LENGTH_TOLERANCE",
    "RECOVERY_PHI_TOLERANCE",
    "FW_RECOVERY_HOLD_TIME_S",
    "FW_RECOVERY_PHI_PREPOSE",
    "FW_RECOVERY_PHI_FINAL",
    "FW_RECOVERY_PHI_PREPOSE_TOLERANCE",
    "FW_RECOVERY_SWING_RATE",
    "FW_RECOVERY_DEADPOINT_SWING_RATE",
    "FW_RECOVERY_COMPACT_SWING_RATE",
    "FW_RECOVERY_SWING_ACCEL",
    "FW_RECOVERY_COMPACT_SWING_ACCEL",
    "FW_RECOVERY_SWING_KP",
    "FW_RECOVERY_SWING_KD",
    "FW_RECOVERY_SWING_FF",
    "FW_RECOVERY_SWING_TP_LIMIT",
    "FW_RECOVERY_SWING_GAIN_RATE",
    "FW_RECOVERY_TP_GAIN_RATE",
    "FW_RECOVERY_TP_SLEW_RATE",
    "FW_RECOVERY_TERMINAL_TP_SLEW_RATE",
    "FW_RECOVERY_OMEGA_FILTER_TAU",
    "FW_RECOVERY_L0_TARGET",
    "FW_RECOVERY_L0_RATE",
    "FW_RECOVERY_L0_OK",
    "FW_RECOVERY_L0_KP",
    "FW_RECOVERY_L0_KD",
    "FW_RECOVERY_FORCE_GAIN_RATE",
    "FW_RECOVERY_FORCE_LIMIT",
    "FW_RECOVERY_FORCE_EXTEND_LIMIT",
    "FW_RECOVERY_FORCE_BIAS",
    "FW_RECOVERY_FORCE_BIAS_RANGE",
    "FW_RECOVERY_SPRING_FF_LIMIT",
    "FW_RECOVERY_BREAKAWAY_RELEASE_L0",
    "FW_RECOVERY_BREAKAWAY_RELEASE_BAND",
    "FW_RECOVERY_BREAKAWAY_MIN_TIME",
    "FW_RECOVERY_BREAKAWAY_RISE_TIME",
    "FW_RECOVERY_BREAKAWAY_TIME",
    "FW_RECOVERY_BREAKAWAY_FORCE",
    "FW_RECOVERY_BREAKAWAY_TORQUE",
    "FW_RECOVERY_TORQUE_RESERVE",
    "FW_RECOVERY_COMPACT_L0",
    "FW_RECOVERY_SETTLE_L0_START",
    "FW_RECOVERY_SETTLE_L0_END",
    "FW_RECOVERY_SETTLE_KP",
    "FW_RECOVERY_SETTLE_KD",
    "FW_RECOVERY_SETTLE_TP_LIMIT",
    "FW_RECOVERY_STABLE_L0_RATE",
    "FW_RECOVERY_STABLE_PHI_RATE",
    "FW_RECOVERY_STABLE_PHI_ERROR",
    "FW_RECOVERY_STABLE_FORCE",
    "FW_RECOVERY_STABLE_TP",
    "FW_RECOVERY_STABLE_TIME_S",
    "FW_RECOVERY_RETRACT_TIMEOUT_S",
    "CAD_GAS_TRIM",
    "CAD_ZERO_GAS_TRIM",
    "CAD_STAND_RAMP_S",
    "RECOVERY_CAD_STAND_LQR_DELAY_S",
    "CAD_DIRECT_START_ELAPSED_S",
    "CAD_STAND_LENGTH_KP",
    "CAD_STAND_LENGTH_KD",
    "CAD_STAND_PHI_KP",
    "CAD_STAND_PHI_KD",
    "CAD_ZERO_JOINT_TOLERANCE_RAD",
    "CAD_STAND_WHEEL_K",
    "CAD_STAND_PITCH_REFERENCE",
    "PRONE_SWING_FORCE",
    "PRONE_SWING_TP",
    "PRONE_SWING_WHEEL_TORQUE",
    "PRONE_ENTRY_PITCH",
    "PRONE_HANDOFF_PITCH",
    "PRONE_HANDOFF_MIN_TIME_S",
    "PRONE_HANDOFF_MIN_CONTACTS",
    "PRONE_SWING_TIMEOUT_S",
    "MID_COEFFICIENTS",
    "LOW_COEFFICIENTS",
    "CALIBRATED_STAND_POSE_PATH",
    "CODE_ZERO_POSE_PATH",
    "ENCODER_ZERO_POSE_PATH",
    "RECOVERY_CROUCH_POSE_PATH",
]
