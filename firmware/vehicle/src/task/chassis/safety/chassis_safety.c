/**
 * @file chassis_safety.c
 * @brief Off-ground detection, airborne leg control, and posture protection.
 */
#include "chassis_safety.h"
#include "../control/jump_kinematic_rate_observer.h"
#include "../control/jump_local_rate_feedback.h"
#include "../control/jump_air_profile.h"
#include "../control/jump_landing_ownership.h"
#include "../control/jump_landing_fast_exit.h"
#include "../control/jump_landing_cushion.h"
#include "../control/lateral_inertial_ff.h"
#include "../control/bump_control_helpers.h"
#include "debug/chassis_debug.h"
#include "rm_config.h"
#include "offground_detection_helpers.h"
#include "drv_dwt.h"
#include <math.h>

#define LEFT CHASSIS_SIDE_LEFT
#define RIGHT CHASSIS_SIDE_RIGHT
#define FORCE_LIMIT 200.0f
static const chassis_safety_context_t *safety_context = NULL;
#define leg (safety_context->leg)
#define ins (*safety_context->ins)
#define chassis_cmd (*safety_context->cmd)
#define chassis_fdb_data (*safety_context->fdb)
#define theta_pid (safety_context->theta_pid)
#define roll_pid (safety_context->roll_pid)
#define L_length_pid (safety_context->length_pid[LEFT])
#define R_length_pid (safety_context->length_pid[RIGHT])
#define LQROutBuf (safety_context->lqr_output)
#define lqr_virtual_x_l (*safety_context->lqr_virtual_x[LEFT])
#define lqr_virtual_x_r (*safety_context->lqr_virtual_x[RIGHT])
#define yaw_target (*safety_context->yaw_target)
#define dm_obs (safety_context->dm_obs)
#define dbg_wheel_speed_phys_l (*safety_context->wheel_speed_phys[LEFT])
#define dbg_wheel_speed_phys_r (*safety_context->wheel_speed_phys[RIGHT])
#define chassis_dt (safety_context->control_dt_s)
#define OFFGROUND_FILTER_LENGTH 4U
#define OFFGROUND_UNSPRUNG_MASS  0.60f
#define OFFGROUND_L0_MIN         0.08f
#define OFFGROUND_DT_MIN         0.0005f
#define OFFGROUND_DT_MAX         0.0200f
#define BUMP_TRUE_AIR_ACCEL_THRESHOLD_MPS2 4.0f
#define BUMP_TRUE_AIR_CONFIRM_S             0.020f
#define BUMP_TRUE_AIR_IMU_MAX_AGE_MS          20U
#define BUMP_TRUE_AIR_ACCEL_FILTER_TAU_S       0.010f
#define BUMP_GROUND_RECOVER_ACCEL_MIN_MPS2     6.0f
#define BUMP_GROUND_RECOVER_ACCEL_MAX_MPS2    13.0f
#define BUMP_GROUND_RECOVER_CONFIRM_S          0.100f
#define BUMP_PHI0_SAFE_MIN_RAD                 (-25.0f * DEGREE_2_RAD)
#define BUMP_PHI0_SAFE_MAX_RAD                  (110.0f * DEGREE_2_RAD)

typedef struct {
    float takeoff_side_threshold;
    float takeoff_mean_threshold;
    float landing_threshold;
    uint8_t jump_takeoff_mode;
    uint8_t takeoff_side_low[CHASSIS_SIDE_COUNT];
    uint8_t takeoff_mean_pass;
    uint8_t takeoff_length_gate_pass;
    uint8_t touch_side_high[CHASSIS_SIDE_COUNT];
    uint8_t touch_inhibit_ready;
    uint8_t condition;
} offground_condition_debug_t;

typedef struct {
    uint8_t capture_valid;
    uint8_t detector_valid;
    uint8_t runtime_valid;
    uint32_t runtime_flags;
    uint32_t impact_flags;
    float detect_time_s;
    float capture_period_s;
    float reconstruction_dt_s;
    float support_candidate[CHASSIS_SIDE_COUNT];
    float reconstructed_raw[CHASSIS_SIDE_COUNT];
    float filtered_force[CHASSIS_SIDE_COUNT];
    float actual_length[CHASSIS_SIDE_COUNT];
    float theta[CHASSIS_SIDE_COUNT];
    float inertial_force[CHASSIS_SIDE_COUNT];
    uint8_t reconstructed_finite[CHASSIS_SIDE_COUNT];
    uint8_t reconstructed_fallback[CHASSIS_SIDE_COUNT];
    float pitch;
    float roll;
    float body_z_accel;
    uint8_t mode;
    uint8_t leg_level;
    uint8_t stand_state;
    float takeoff_side_threshold;
    float takeoff_mean_threshold;
    float landing_threshold;
    uint8_t takeoff_side_low[CHASSIS_SIDE_COUNT];
    uint8_t takeoff_mean_pass;
    uint8_t takeoff_length_gate_pass;
    uint8_t touch_side_high[CHASSIS_SIDE_COUNT];
    uint8_t touch_inhibit_ready;
    uint8_t enter_condition;
    uint8_t touch_condition;
    uint8_t touch_evaluated;
    uint8_t takeoff_progress;
    uint8_t landing_progress;
    uint8_t takeoff_required;
    uint8_t landing_required;
    float airborne_elapsed_s;
} offground_debug_snapshot_t;

#define OFFGROUND_DIAG_COUNTER_MAX 16777215UL
static offground_debug_snapshot_t offground_debug_snapshot;
static float offground_debug_last_capture_ms;
static uint8_t offground_debug_capture_clock_valid;
static uint32_t offground_debug_takeoff_events;
static uint32_t offground_debug_landing_events;
static uint32_t offground_debug_buffer_begins;
/* Counts clear/reset function calls, not physical landing events. */
static uint32_t offground_debug_clear_calls;
static uint8_t offground_debug_last_takeoff_source_valid;
static uint8_t offground_debug_last_takeoff_forced;
static uint8_t offground_debug_impact_eval_valid;
static uint8_t offground_debug_impact_left;
static uint8_t offground_debug_impact_right;
static uint8_t offground_debug_impact_result;
static uint8_t offground_debug_impact_count;

static void landing_buffer_reset(void);

static void offground_debug_count_increment(uint32_t *counter)
{
    if (*counter < OFFGROUND_DIAG_COUNTER_MAX) {
        (*counter)++;
    }
}

static void offground_debug_impact_clear(void)
{
    offground_debug_impact_eval_valid = 0U;
    offground_debug_impact_left = 0U;
    offground_debug_impact_right = 0U;
    offground_debug_impact_result = 0U;
    offground_debug_impact_count = 0U;
    offground_debug_snapshot.impact_flags = 0U;
}

static float FN_Average;       // filtered left/right mean, diagnostic only
static float offground_fn_raw[CHASSIS_SIDE_COUNT];
static float offground_fn_filtered[CHASSIS_SIDE_COUNT];
static float offground_fn_history[CHASSIS_SIDE_COUNT][OFFGROUND_FILTER_LENGTH];
static float offground_fn_measured_raw[CHASSIS_SIDE_COUNT];
static float offground_fn_measured_filtered[CHASSIS_SIDE_COUNT];
static float offground_fn_measured_history[CHASSIS_SIDE_COUNT][OFFGROUND_FILTER_LENGTH];
static uint8_t offground_filter_index = 0U;
static uint8_t offground_filter_initialized = 0U;
static uint8_t offground_measured_filter_index = 0U;
static uint8_t offground_measured_filter_initialized = 0U;
static float offground_last_d_l0[CHASSIS_SIDE_COUNT];
static float offground_last_d_theta[CHASSIS_SIDE_COUNT];
static uint8_t offground_derivative_initialized = 0U;
static offground_kin_rate_observer_t offground_position_observer;
static offground_kin_rate_view_t offground_position_view = {
    .position_age_ms = INFINITY,
    .rate_age_ms = INFINITY
};
static offground_confirmation_state_t offground_confirmation;
static uint64_t offground_confirmation_last_us;
static uint8_t offground_confirmation_clock_valid;
static float offground_feedback_max_age_ms = -1.0f;
static uint8_t offground_position_feedback_fresh;
static uint8_t offground_ins_feedback_fresh;
static uint8_t offground_measured_fn_valid;
static uint8_t offground_confirmation_evidence_valid;
static float offground_last_position_sample_dt_ms;
static uint8_t offground_legacy_scope_active = 1U;
static uint8_t offground_scope_initialized;

static uint8_t Off_Ground_Cnt = 0 ;
static uint8_t Touch_Ground_Cnt = 0;
static uint8_t  Off_Ground_Flag = 0;
static uint8_t Wheel_Shut_Flag = 0;
static uint8_t  Touch_Ground_Flag;
static uint8_t offground_shadow_takeoff_active = 0;
/* 由 BUMP 主状态机逐拍设置；不复用 jump_state。 */
static uint8_t bump_active = 0U;
static uint8_t bump_action_active = 0U;
static uint8_t bump_airborne_lockout = 0U;
static float bump_low_accel_elapsed_s = 0.0f;
static float bump_accel_norm_filtered = 0.0f;
static uint8_t bump_accel_norm_filter_initialized = 0U;
static float bump_ground_recover_elapsed_s = 0.0f;

/* ==================== 离地/重新落地处理 ==================== */
#define LANDING_YAW_HOLD_TIME   0.15f   /* 落地后 yaw 锁定缓冲时间 (s) */

static uint8_t chassis_off_ground = 0;
static uint8_t chassis_off_ground_last = 0;
static uint8_t chassis_takeoff_edge = 0;
static uint8_t chassis_landing_edge = 0;
static float landing_yaw_hold_timer = 0.0f;

typedef enum {
    LANDING_BUFFER_INACTIVE = 0,
    LANDING_BUFFER_COMPRESS,
    LANDING_BUFFER_HOLD,
    LANDING_BUFFER_RETURN
} landing_buffer_phase_e;

static landing_buffer_phase_e landing_buffer_phase =
    LANDING_BUFFER_INACTIVE;
static float landing_buffer_phase_timer = 0.0f;
static float landing_buffer_elapsed = 0.0f;
static float landing_precontact_elapsed = 0.0f;
static uint8_t landing_precontact_compact_count = 0U;
static float landing_exit_stable_elapsed = 0.0f;
static float landing_update_last_ms = 0.0f;
static float landing_impact_rearm_timer = 0.0f;
static uint8_t landing_contact_confirmed = 0U;
static uint8_t landing_buffer_jump_origin = 0U;
/* Preserve the jump source across a command-mode handoff.  The operator can
 * release the jump command while the chassis is still airborne; using the
 * current mode at touchdown would otherwise classify the landing as normal. */
static uint8_t landing_jump_origin_pending = 0U;
static jump_landing_axial_handoff_t landing_fast_axial_handoff;
static uint8_t landing_fast_pid_seed_pending = 0U;
static jump_landing_fast_exit_confirm_t landing_fast_exit_confirm;
static uint8_t landing_fast_exit_ready_latched = 0U;
static jump_landing_cushion_t landing_cushion;
static float landing_postbuffer_ref[CHASSIS_SIDE_COUNT] = {
    LANDING_BUFFER_MID_LENGTH, LANDING_BUFFER_MID_LENGTH
};
static float landing_postbuffer_elapsed_s = 0.0f;
static float landing_postbuffer_fade_elapsed_s = 0.0f;
static uint8_t landing_postbuffer_active = 0U;
static float landing_airborne_guard_elapsed_s = 0.0f;
static uint8_t landing_airborne_guard_armed = 0U;
static uint8_t landing_impact_armed = 0U;
static uint8_t landing_impact_count = 0U;
static float landing_air_length_sync_force = 0.0f;
static float landing_length_ref[CHASSIS_SIDE_COUNT] = {
    LANDING_BUFFER_MID_LENGTH, LANDING_BUFFER_MID_LENGTH
};
static float landing_absorb_ref[CHASSIS_SIDE_COUNT] = {
    LANDING_BUFFER_MID_LENGTH, LANDING_BUFFER_MID_LENGTH
};
static float landing_air_deploy_ref[CHASSIS_SIDE_COUNT] = {
    LANDING_AIR_DEPLOY_LENGTH, LANDING_AIR_DEPLOY_LENGTH
};
/* Normal MID/HIGH flight gets a guarded differential correction only when the
 * two measured legs split well beyond the takeoff geometry. */
static offground_normal_split_guard_t offground_normal_split_guard;
static float landing_airborne_elapsed = 0.0f;
/* The jump flight profile starts at the commanded end of the thrust pulse,
 * before the filtered contact estimator is expected to confirm takeoff. */
static uint8_t jump_air_profile_prestarted = 0U;
/* Set by the control chain while the jump has released its launch impulse and
 * the legs are coasting/retracting, i.e. no longer driven into the ground.  The
 * contact estimator excludes the internal spring/hip terms for this whole
 * window (see chassis_safety_module_capture_raw_support). */
static uint8_t jump_legs_unloaded = 0U;
/* 2026-09-23: airborne leg-angle reference.  Captured at takeoff by the control
 * chain and driven toward JUMP_AIR_THETA_REF_RAD; the airborne joint controller
 * holds theta against it. */
static float jump_air_theta_ref = 0.0f;
/* Legacy descent verdict exposed for existing diagnostics; actual compact
 * dwell is never shortened by descent. */
static uint8_t jump_air_profile_descent = 0U;
static jump_air_profile_state_t jump_air_profile_state;
static uint8_t jump_air_position_feedback_fresh = 0U;
static float landing_requested_ref[CHASSIS_SIDE_COUNT] = {
    LANDING_BUFFER_MID_LENGTH, LANDING_BUFFER_MID_LENGTH
};

/* ==================== 空中 LQR 姿态保持 ==================== */
#define OFFGROUND_LQR_TP_GAIN          0.50f
#define JUMP_AIR_LQR_TP_GAIN           1.00f
#define OFFGROUND_LQR_TP_LIMIT        40.0f
#define OFFGROUND_SYNC_LIMIT           3.5f
#define JUMP_AIR_TP_LIMIT              28.0f
#define JUMP_AIR_THETA_SYNC_LIMIT      18.0f
#define OFFGROUND_SOFT_FORCE           (20.0f)
#define OFFGROUND_LENGTH_SYNC_KP       300.0f
#define OFFGROUND_LENGTH_SYNC_KD        12.0f
#define OFFGROUND_LENGTH_SYNC_LIMIT     18.0f
#define OFFGROUND_SPRING_COMP_LIMIT    FORCE_LIMIT
#define OFFGROUND_AIR_RETRACT_LIMIT     45.0f
/* Airborne leg-rod hold: theta = 0 means a vertical leg rod. */
#define OFFGROUND_THETA_HOLD_KP         8.5f
#define OFFGROUND_THETA_HOLD_KD         0.75f
#define OFFGROUND_THETA_HOLD_TP_LIMIT   4.5f
#define JUMP_AIR_THETA_HOLD_KP          80.0f
#define JUMP_AIR_THETA_HOLD_KD          12.0f
#define JUMP_AIR_THETA_HOLD_TP_LIMIT    40.0f
/* JUMP_THETA_ERR_CLAMP (the wrap-safe error bound) lives in rm_config.h because
 * the grounded thrust channel in chassis_control.c uses the same limit. */
#define JUMP_AIR_PITCH_YIELD_START       0.04f
#define JUMP_AIR_PITCH_YIELD_END         0.16f

/* Temporary airborne polarity calibration. Set to 0 after the phi0 directions are recorded. */
#define OFFGROUND_TP_POLARITY_TEST_ENABLE  0
#define OFFGROUND_TP_TEST_RUN_LEFT          0
#define OFFGROUND_TP_TEST_NEUTRALIZE_AXIAL_FORCE 1
#define OFFGROUND_TP_TEST_CAPTURE_TIME     0.30f
#define OFFGROUND_TP_TEST_PULSE_TIME       0.30f
#define OFFGROUND_TP_TEST_SETTLE_TIME      0.35f
#define OFFGROUND_TP_TEST_LEVEL            6.0f

typedef enum {
    OFFGROUND_TP_TEST_CAPTURE = 0,
    OFFGROUND_TP_TEST_LEFT_POS,
    OFFGROUND_TP_TEST_LEFT_POS_SETTLE,
    OFFGROUND_TP_TEST_LEFT_NEG,
    OFFGROUND_TP_TEST_LEFT_NEG_SETTLE,
    OFFGROUND_TP_TEST_RIGHT_POS,
    OFFGROUND_TP_TEST_RIGHT_POS_SETTLE,
    OFFGROUND_TP_TEST_RIGHT_NEG,
    OFFGROUND_TP_TEST_DONE
} offground_tp_test_phase_e;

static offground_tp_test_phase_e offground_tp_test_phase = OFFGROUND_TP_TEST_CAPTURE;
static offground_tp_test_phase_e dbg_offground_tp_test_applied_phase = OFFGROUND_TP_TEST_CAPTURE;
static float offground_tp_test_timer = 0.0f;
static float dbg_offground_tp_test_L = 0.0f;
static float dbg_offground_tp_test_R = 0.0f;

typedef enum {
    DANGER_NONE = 0,
    DANGER_PITCH,
    DANGER_ROLL,
    DANGER_LEFT_PHI_LOW,
    DANGER_LEFT_PHI_HIGH,
    DANGER_RIGHT_PHI_LOW,
    DANGER_RIGHT_PHI_HIGH,
    DANGER_LEFT_THETA,
    DANGER_RIGHT_THETA
} danger_reason_e;

static danger_reason_e dbg_danger_reason = DANGER_NONE;
static danger_reason_e danger_candidate_last = DANGER_NONE;
static float danger_candidate_timer = 0.0f;
/* 2026-09-23: latched copy of the last real danger verdict.  `dbg_danger_reason`
 * is rewritten every cycle, so once the jump state machine aborts and the
 * following cycle reports DANGER_NONE the cause was lost (page 56 recorded
 * "16" = SAFETY + PITCH for a jump whose logged pitch never exceeded 5.2 deg).
 * The latch keeps the verdict that actually tripped until the next jump start
 * clears it, so the operator always reads the true trigger. */
static danger_reason_e danger_reason_latched = DANGER_NONE;

#define DANGER_HARD_ATTITUDE_DEG         60.0f
#define DANGER_SOFT_PITCH_DEG            35.0f
#define DANGER_SOFT_ROLL_DEG             30.0f
#define DANGER_SOFT_THETA_RAD            (45.0f * DEGREE_2_RAD)
#define DANGER_SOFT_CONFIRM_TIME          0.08f

/* 离地/空中调试变量 */
static uint8_t jump_support_feedback_valid;
static float dbg_raw_support_L = 0.0f;
static float dbg_raw_support_R = 0.0f;
static float dbg_raw_fn_avg = 0.0f;
static float dbg_ground_detect_fn_avg = 0.0f;
static float jump_length_rate_snapshot[CHASSIS_SIDE_COUNT];
static float jump_theta_rate_snapshot[CHASSIS_SIDE_COUNT];
static float jump_rate_snapshot_age_ms = INFINITY;
static uint8_t jump_rate_snapshot_usable;
static uint8_t jump_rate_snapshot_fresh;
static float jump_landing_length_rate_snapshot[CHASSIS_SIDE_COUNT];
static uint8_t jump_landing_length_rate_snapshot_valid;
static float dbg_offground_lqr_tp_L = 0.0f;
static float dbg_offground_lqr_tp_R = 0.0f;
static float dbg_offground_theta_hold_tp_L = 0.0f;
static float dbg_offground_theta_hold_tp_R = 0.0f;
static float dbg_offground_theta_hold_tp_common_raw = 0.0f;

static float move_towards(float current, float target, float max_step)
{
    const float error = target - current;
    if (error > max_step) {
        return current + max_step;
    }
    if (error < -max_step) {
        return current - max_step;
    }
    return target;
}

static uint8_t offground_requires_legacy_fn(void)
{
    return offground_legacy_scope_required(
        chassis_cmd.ctrl_mode == CHASSIS_JUMP,
        landing_jump_origin_pending, landing_buffer_jump_origin,
        landing_postbuffer_active, landing_airborne_guard_armed);
}

static void offground_measured_filter_reset(void)
{
    offground_measured_filter_index = 0U;
    offground_measured_filter_initialized = 0U;
    offground_measured_fn_valid = 0U;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; ++side) {
        offground_fn_measured_raw[side] = 0.0f;
        offground_fn_measured_filtered[side] = 0.0f;
        for (uint8_t tap = 0U; tap < OFFGROUND_FILTER_LENGTH; ++tap) {
            offground_fn_measured_history[side][tap] = 0.0f;
        }
    }
}

static uint8_t offground_ins_sample_fresh(void)
{
    if (safety_context == NULL || !isfinite(ins.motion_accel_b[2])) {
        return 0U;
    }
    return lateral_inertial_ff_sample_fresh(
        HAL_GetTick(), ins.publish_tick_ms, ins.publish_count,
        LATERAL_FF_INS_MAX_AGE_MS);
}

static void offground_confirmation_clear_candidates(void)
{
    offground_confirmation.takeoff_progress_ms = 0.0f;
    offground_confirmation.takeoff_weak_progress_ms = 0.0f;
    offground_confirmation.takeoff_strong_progress_ms = 0.0f;
    offground_confirmation.takeoff_candidate_kind = 0U;
    offground_confirmation.landing_progress_ms = 0.0f;
    offground_confirmation.landing_candidate_ms = 0.0f;
    offground_confirmation.landing_dropout_ms = 0.0f;
    offground_confirmation.landing_candidate_active = 0U;
}

static float offground_confirmation_cycle_dt_ms(void)
{
    const uint64_t now_us = dwt_get_time_us();
    float dt_ms = 0.0f;
    if (offground_confirmation_clock_valid) {
        if (now_us < offground_confirmation_last_us) {
            offground_confirmation_last_us = now_us;
            offground_confirmation_clear_candidates();
            return INFINITY;
        }
        dt_ms = (float)(now_us - offground_confirmation_last_us) * 0.001f;
    }
    offground_confirmation_last_us = now_us;
    offground_confirmation_clock_valid = 1U;
    offground_confirmation_advance_post_landing(
        &offground_confirmation, dt_ms);
    return dt_ms;
}

static void offground_source_scope_update(void)
{
    const uint8_t legacy_scope = offground_requires_legacy_fn();
    if (offground_source_scope_transition(
            &offground_scope_initialized, &offground_legacy_scope_active,
            legacy_scope)) {
        offground_kin_observer_reset(&offground_position_observer);
        offground_position_view = (offground_kin_rate_view_t){
            .position_age_ms = INFINITY,
            .rate_age_ms = INFINITY
        };
        offground_measured_filter_reset();
        offground_confirmation_reset(&offground_confirmation);
        offground_last_position_sample_dt_ms = 0.0f;
        offground_confirmation_evidence_valid = 0U;
        Off_Ground_Cnt = 0U;
        Touch_Ground_Cnt = 0U;
    }
}

static float offground_selected_fn(chassis_side_e side)
{
    if (side >= CHASSIS_SIDE_COUNT) {
        return 0.0f;
    }
    return offground_legacy_scope_active ?
        offground_fn_filtered[side] : offground_fn_measured_filtered[side];
}

static float Offground_TakeoffThreshold(void)
{
    switch (chassis_cmd.leg_level) {
    case LEG_HIG:
        return OFFGROUND_TAKEOFF_FN_HIGH;
    case LEG_MID:
        return OFFGROUND_TAKEOFF_FN_MID;
    case LEG_LOW:
    default:
        return OFFGROUND_TAKEOFF_FN_LOW;
    }
}

static uint8_t Offground_TakeoffConfirmCount(void)
{
    if (chassis_cmd.ctrl_mode == CHASSIS_JUMP || bump_active) {
        return OFFGROUND_JUMP_TAKEOFF_CONFIRM;
    }
    switch (chassis_cmd.leg_level) {
    case LEG_HIG:
        return OFFGROUND_TAKEOFF_CONFIRM_HIGH;
    case LEG_MID:
        return OFFGROUND_TAKEOFF_CONFIRM_MID;
    case LEG_LOW:
    default:
        return OFFGROUND_TAKEOFF_CONFIRM_LOW;
    }
}

static float Offground_LandingThreshold(void)
{
    switch (chassis_cmd.leg_level) {
    case LEG_HIG:
        return OFFGROUND_LANDING_FN_HIGH;
    case LEG_MID:
        return OFFGROUND_LANDING_FN_MID;
    case LEG_LOW:
    default:
        return OFFGROUND_LANDING_FN_LOW;
    }
}

static uint8_t Offground_LandingConfirmCount(void)
{
    switch (chassis_cmd.leg_level) {
    case LEG_HIG:
        return OFFGROUND_LANDING_CONFIRM_HIGH;
    case LEG_MID:
        return OFFGROUND_LANDING_CONFIRM_MID;
    case LEG_LOW:
    default:
        return OFFGROUND_LANDING_CONFIRM_LOW;
    }
}

static uint8_t Offground_EnterCondition(
    offground_condition_debug_t *debug)
{
    const uint8_t jump_mode = chassis_cmd.ctrl_mode == CHASSIS_JUMP;
    const float threshold = jump_mode ? OFFGROUND_JUMP_TAKEOFF_SIDE_MAX :
                                        Offground_TakeoffThreshold();
    const uint8_t left_low =
        offground_selected_fn(LEFT) < threshold;
    const uint8_t right_low =
        offground_selected_fn(RIGHT) < threshold;
    uint8_t length_gate_pass = 1U;
    uint8_t mean_pass = 0U;

    if (debug != NULL) {
        debug->takeoff_side_threshold = threshold;
        debug->takeoff_mean_threshold = jump_mode ?
            OFFGROUND_JUMP_TAKEOFF_MEAN_MAX : -1.0f;
        debug->jump_takeoff_mode = jump_mode;
        debug->takeoff_side_low[LEFT] = left_low;
        debug->takeoff_side_low[RIGHT] = right_low;
        debug->takeoff_length_gate_pass = 1U;
    }

    if (jump_mode) {
        /* During a jump, the robot crouches down in PRELOAD (~0.12 m) and
         * then thrusts outward to ~0.285 m. Crouch down-acceleration causes
         * a brief normal-force dip while firmly grounded. Require legs to
         * extend past 0.26 m or the boost timer to prestart air profile
         * before takeoff can ever be evaluated. */
        const float avg_length = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        length_gate_pass = (uint8_t)!(
            !jump_air_profile_prestarted && avg_length < 0.26f);
        const float mean_force = 0.5f *
            (offground_selected_fn(LEFT) +
             offground_selected_fn(RIGHT));
        mean_pass = mean_force < OFFGROUND_JUMP_TAKEOFF_MEAN_MAX;
    }

    if (debug != NULL) {
        debug->takeoff_length_gate_pass = length_gate_pass;
        debug->takeoff_mean_pass = mean_pass;
    }

    if (jump_mode) {
        return length_gate_pass && mean_pass && left_low && right_low;
    }
    if (bump_active) {
        /* BUMP 不走跳跃的 0.26 m 预压豁免，直接使用通用离地判据；
         * 此分支与下方通用路径等价，保留以对应 611 的逐字移植。 */
        return left_low && right_low;
    }
    return left_low && right_low;
}

static uint8_t Offground_TouchCondition(
    offground_condition_debug_t *debug)
{
    const float threshold = Offground_LandingThreshold();
    const uint8_t left_high =
        offground_selected_fn(LEFT) > threshold;
    const uint8_t right_high =
        offground_selected_fn(RIGHT) > threshold;
    uint8_t inhibit_ready = 1U;

    if (debug != NULL) {
        debug->landing_threshold = threshold;
        debug->touch_side_high[LEFT] = left_high;
        debug->touch_side_high[RIGHT] = right_high;
    }
    if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        /* Physical jump flight time is ~300 ms. Mid-air retractions generate
         * large inertial normal force spikes when braking. Touchdown can ONLY
         * be detected after the unchanged 0.22 s touch/impact inhibit; this
         * fixed guard is independent of the measured compact dwell/deploy. */
        if (landing_airborne_elapsed <
            (JUMP_AIR_TUCK_TIME + JUMP_AIR_TUCK_HOLD_TIME)) {
            inhibit_ready = 0U;
        }
    }
    if (debug != NULL) {
        debug->touch_inhibit_ready = inhibit_ready;
    }
    /* Extension and braking also create large internal leg forces in flight.
     * A jump contact needs current motor load, an IMU impact, and compression
     * of BOTH measured legs after deployment; a time/state label is not load. */
    if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        inhibit_ready = (uint8_t)(inhibit_ready &&
            jump_air_profile_state.deploy_latched &&
            jump_support_feedback_valid && offground_ins_sample_fresh() &&
            jump_rate_snapshot_usable &&
            isfinite(ins.accel[2]) &&
            ins.accel[2] > JUMP_TOUCH_SPECIFIC_FORCE_MIN &&
            jump_length_rate_snapshot[LEFT] < JUMP_TOUCH_COMPRESSION_RATE_MAX &&
            jump_length_rate_snapshot[RIGHT] < JUMP_TOUCH_COMPRESSION_RATE_MAX);
    }
    return inhibit_ready && left_high && right_high;
}

static void offground_debug_capture_detector(
    const offground_condition_debug_t *enter_debug,
    const offground_condition_debug_t *touch_debug,
    uint8_t touch_evaluated, uint8_t touch_condition,
    uint8_t takeoff_progress, uint8_t landing_progress,
    uint8_t takeoff_required, uint8_t landing_required)
{
    offground_debug_snapshot_t *snapshot = &offground_debug_snapshot;
    snapshot->detector_valid = 0U;
    snapshot->runtime_valid = 0U;
    snapshot->detect_time_s = dwt_get_time_ms() / 1000.0f;
    snapshot->mode = (uint8_t)chassis_cmd.ctrl_mode;
    snapshot->leg_level = (uint8_t)chassis_cmd.leg_level;
    snapshot->stand_state = (uint8_t)chassis_fdb_data.stand_state;
    snapshot->takeoff_side_threshold =
        enter_debug->takeoff_side_threshold;
    snapshot->takeoff_mean_threshold =
        enter_debug->takeoff_mean_threshold;
    snapshot->landing_threshold = touch_debug->landing_threshold;
    snapshot->takeoff_side_low[LEFT] = enter_debug->takeoff_side_low[LEFT];
    snapshot->takeoff_side_low[RIGHT] = enter_debug->takeoff_side_low[RIGHT];
    snapshot->takeoff_mean_pass = enter_debug->takeoff_mean_pass;
    snapshot->takeoff_length_gate_pass =
        enter_debug->takeoff_length_gate_pass;
    snapshot->touch_side_high[LEFT] =
        touch_evaluated ? touch_debug->touch_side_high[LEFT] : 0U;
    snapshot->touch_side_high[RIGHT] =
        touch_evaluated ? touch_debug->touch_side_high[RIGHT] : 0U;
    snapshot->touch_inhibit_ready = touch_evaluated ?
        touch_debug->touch_inhibit_ready : 0U;
    snapshot->enter_condition = enter_debug->condition;
    snapshot->touch_condition = touch_condition;
    snapshot->touch_evaluated = touch_evaluated;
    snapshot->takeoff_progress = takeoff_progress;
    snapshot->landing_progress = landing_progress;
    snapshot->takeoff_required = takeoff_required;
    snapshot->landing_required = landing_required;
    snapshot->airborne_elapsed_s = landing_airborne_elapsed;
    snapshot->detector_valid = 1U;
}

static float Offground_TakeoffConfirmMs(void)
{
    if (bump_active) {
        return 3.0f;
    }
    switch (chassis_cmd.leg_level) {
    case LEG_HIG:
        return 15.0f;
    case LEG_MID:
    case LEG_LOW:
    default:
        return 5.0f;
    }
}

static float Offground_LandingConfirmMs(void)
{
    switch (chassis_cmd.leg_level) {
    case LEG_HIG:
        return 12.0f;
    case LEG_MID:
        return 10.0f;
    case LEG_LOW:
    default:
        return 8.0f;
    }
}

static uint8_t offground_progress_debug_count(float progress_ms)
{
    if (!isfinite(progress_ms) || progress_ms <= 0.0f) {
        return 0U;
    }
    if (progress_ms >= 254.5f) {
        return 255U;
    }
    return (uint8_t)(progress_ms + 0.5f);
}

static void offground_measured_detect(
    offground_condition_debug_t *enter_debug,
    offground_condition_debug_t *touch_debug,
    uint8_t enter_condition,
    uint8_t *touch_evaluated,
    uint8_t *touch_condition,
    uint8_t evidence_allowed,
    float cycle_dt_ms)
{
    uint8_t confirmed = 0U;
    const uint8_t valid = (uint8_t)(offground_measured_fn_valid &&
                                    offground_position_view.valid &&
                                    offground_ins_feedback_fresh);
    if (!valid) {
        offground_confirmation_clear_candidates();
        Off_Ground_Cnt = 0U;
        Touch_Ground_Cnt = 0U;
        offground_shadow_takeoff_active = 0U;
        return;
    }

    offground_shadow_takeoff_active = (uint8_t)(
        !Off_Ground_Flag && enter_condition);
    if (!evidence_allowed) {
        return;
    }

    const float dt_ms = cycle_dt_ms;
    if (Off_Ground_Flag == 0U) {
        const uint8_t strong_candidate = (uint8_t)(
            offground_selected_fn(LEFT) < OFFGROUND_POST_LANDING_STRONG_FN_MAX_N &&
            offground_selected_fn(RIGHT) < OFFGROUND_POST_LANDING_STRONG_FN_MAX_N);
        offground_confirmation_step_takeoff(
            &offground_confirmation, valid, enter_condition,
            strong_candidate, dt_ms, Offground_TakeoffConfirmMs(),
            OFFGROUND_POST_LANDING_WEAK_CONFIRM_MS, &confirmed);
        Off_Ground_Cnt = offground_progress_debug_count(
            offground_confirmation.takeoff_progress_ms);
        Touch_Ground_Cnt = 0U;
        if (confirmed) {
            Off_Ground_Flag = 1U;
            Touch_Ground_Flag = 0U;
            Off_Ground_Cnt = 0U;
            Touch_Ground_Cnt = 0U;
            offground_confirmation.landing_candidate_active = 0U;
            offground_confirmation.landing_progress_ms = 0.0f;
        }
        return;
    }

    *touch_condition = Offground_TouchCondition(touch_debug);
    *touch_evaluated = 1U;
    const float hold_threshold =
        Offground_LandingThreshold() - OFFGROUND_LANDING_RELEASE_MARGIN_N;
    const uint8_t hold_condition = (uint8_t)(
        offground_selected_fn(LEFT) > hold_threshold &&
        offground_selected_fn(RIGHT) > hold_threshold);
    offground_confirmation_step_landing(
        &offground_confirmation, valid, *touch_condition,
        hold_condition, dt_ms, Offground_LandingConfirmMs(), &confirmed);
    Touch_Ground_Cnt = offground_progress_debug_count(
        offground_confirmation.landing_progress_ms);
    Off_Ground_Cnt = 0U;
    if (confirmed) {
        Off_Ground_Flag = 0U;
        Touch_Ground_Flag = 1U;
        Off_Ground_Cnt = 0U;
        Touch_Ground_Cnt = 0U;
        offground_confirmation_landing(&offground_confirmation);
    }
    (void)enter_debug;
}

/*
 * @brief 判断是否离地
 * @note 左右腿分别经四点滑动平均，两腿同时低于单一阈值判定离地。
 * */
static void Ground_Detect(void){
    offground_condition_debug_t enter_debug = {0};
    offground_condition_debug_t touch_debug = {0};
    uint8_t enter_condition;
    uint8_t touch_condition = 0U;
    uint8_t touch_evaluated = 0U;
    const float confirmation_cycle_dt_ms =
        offground_confirmation_cycle_dt_ms();
    const uint8_t confirmation_evidence_allowed = (uint8_t)(
        isfinite(confirmation_cycle_dt_ms) &&
        confirmation_cycle_dt_ms > 0.0f &&
        confirmation_cycle_dt_ms <= OFFGROUND_CONFIRM_MAX_SAMPLE_DT_MS);
    uint8_t takeoff_progress = 0U;
    uint8_t landing_progress = 0U;
    const uint8_t takeoff_confirm_count =
        Offground_TakeoffConfirmCount();
    const uint8_t landing_confirm_count =
        Offground_LandingConfirmCount();

    offground_source_scope_update();
    offground_ins_feedback_fresh = offground_ins_sample_fresh();
    offground_confirmation_evidence_valid = (uint8_t)(
        !offground_legacy_scope_active && offground_measured_fn_valid &&
        offground_position_view.valid &&
        offground_ins_feedback_fresh &&
        confirmation_evidence_allowed);

    enter_condition = Offground_EnterCondition(&enter_debug);
    enter_debug.condition = enter_condition;
    /* Capture the selected threshold even on cycles where the touch helper is
     * not called because takeoff remains the active side of the detector. */
    touch_debug.landing_threshold = Offground_LandingThreshold();

    if (!offground_legacy_scope_active) {
        if (!confirmation_evidence_allowed) {
            offground_confirmation_clear_candidates();
            Off_Ground_Cnt = 0U;
            Touch_Ground_Cnt = 0U;
        }
        offground_measured_detect(&enter_debug, &touch_debug,
                                  enter_condition, &touch_evaluated,
                                  &touch_condition,
                                  confirmation_evidence_allowed,
                                  confirmation_cycle_dt_ms);
#if !OFFGROUND_DETECTION_ENABLE
        Off_Ground_Flag = 0U;
        Touch_Ground_Flag = 0U;
        Wheel_Shut_Flag = 0U;
#endif
        offground_debug_capture_detector(
            &enter_debug, &touch_debug, touch_evaluated, touch_condition,
            Off_Ground_Cnt, Touch_Ground_Cnt,
            takeoff_confirm_count, landing_confirm_count);
        return;
    }

#if !OFFGROUND_DETECTION_ENABLE
    /* 标定时只观察单阈值影子判定，绝不进入空中控制或关闭轮端。 */
    Off_Ground_Flag = 0;
    Touch_Ground_Flag = 0;
    Touch_Ground_Cnt = 0;
    Wheel_Shut_Flag = 0;

    if (enter_condition) {
        if (Off_Ground_Cnt < takeoff_confirm_count) {
            Off_Ground_Cnt++;
        }
        takeoff_progress = Off_Ground_Cnt;
    } else {
        Off_Ground_Cnt = 0;
    }
    offground_shadow_takeoff_active =
        (Off_Ground_Cnt >= takeoff_confirm_count);
    offground_debug_capture_detector(
        &enter_debug, &touch_debug, 0U, 0U,
        takeoff_progress, Touch_Ground_Cnt,
        takeoff_confirm_count, landing_confirm_count);
    return;
#endif

    offground_shadow_takeoff_active = 0;

    if (Off_Ground_Flag == 0) {
        if (enter_condition) {
            if (Off_Ground_Cnt < takeoff_confirm_count) {
                Off_Ground_Cnt++;
            }
            takeoff_progress = Off_Ground_Cnt;
            if (Off_Ground_Cnt >= takeoff_confirm_count) {
                Off_Ground_Flag = 1;
                Touch_Ground_Flag = 0;
                Off_Ground_Cnt = 0;
                Touch_Ground_Cnt = 0;
            }
        } else {
            Off_Ground_Cnt = 0;
        }
    } else {
        touch_condition = Offground_TouchCondition(&touch_debug);
        touch_evaluated = 1U;
        if (touch_condition) {
            if (Touch_Ground_Cnt < landing_confirm_count) {
                Touch_Ground_Cnt++;
            }
            landing_progress = Touch_Ground_Cnt;
            if (Touch_Ground_Cnt >= landing_confirm_count) {
                Off_Ground_Flag = 0;
                Touch_Ground_Flag = 1;
                Off_Ground_Cnt = 0;
                Touch_Ground_Cnt = 0;
            }
        } else {
            Touch_Ground_Cnt = 0;
        }
    }
    offground_debug_capture_detector(
        &enter_debug, &touch_debug, touch_evaluated, touch_condition,
        takeoff_progress, landing_progress,
        takeoff_confirm_count, landing_confirm_count);
}

static void Offground_TpPolarityTest_Reset(void)
{
    offground_tp_test_phase = OFFGROUND_TP_TEST_CAPTURE;
    dbg_offground_tp_test_applied_phase = OFFGROUND_TP_TEST_CAPTURE;
    offground_tp_test_timer = 0.0f;
    dbg_offground_tp_test_L = 0.0f;
    dbg_offground_tp_test_R = 0.0f;
}

static void Offground_TpPolarityTest_Apply(float dt, float *tp_l, float *tp_r)
{
    dbg_offground_tp_test_L = 0.0f;
    dbg_offground_tp_test_R = 0.0f;
    dbg_offground_tp_test_applied_phase = offground_tp_test_phase;
    offground_tp_test_timer += dt;

    switch (offground_tp_test_phase) {
    case OFFGROUND_TP_TEST_CAPTURE:
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_CAPTURE_TIME) {
#if OFFGROUND_TP_TEST_RUN_LEFT
            offground_tp_test_phase = OFFGROUND_TP_TEST_LEFT_POS;
#else
            offground_tp_test_phase = OFFGROUND_TP_TEST_RIGHT_POS;
#endif
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_LEFT_POS:
        dbg_offground_tp_test_L = OFFGROUND_TP_TEST_LEVEL;
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_PULSE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_LEFT_POS_SETTLE;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_LEFT_POS_SETTLE:
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_SETTLE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_LEFT_NEG;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_LEFT_NEG:
        dbg_offground_tp_test_L = -OFFGROUND_TP_TEST_LEVEL;
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_PULSE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_LEFT_NEG_SETTLE;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_LEFT_NEG_SETTLE:
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_SETTLE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_RIGHT_POS;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_RIGHT_POS:
        dbg_offground_tp_test_R = OFFGROUND_TP_TEST_LEVEL;
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_PULSE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_RIGHT_POS_SETTLE;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_RIGHT_POS_SETTLE:
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_SETTLE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_RIGHT_NEG;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_RIGHT_NEG:
        dbg_offground_tp_test_R = -OFFGROUND_TP_TEST_LEVEL;
        if (offground_tp_test_timer >= OFFGROUND_TP_TEST_PULSE_TIME) {
            offground_tp_test_phase = OFFGROUND_TP_TEST_DONE;
            offground_tp_test_timer = 0.0f;
        }
        break;

    case OFFGROUND_TP_TEST_DONE:
    default:
        break;
    }

    *tp_l = dbg_offground_tp_test_L;
    *tp_r = dbg_offground_tp_test_R;
}
static uint8_t chassis_airborne_allowed(void)
{
    const uint8_t normal_stand_state =
        chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK;
    const uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL;
    const uint8_t jump_mode =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPING;
    const uint8_t jump_origin_pending =
        landing_jump_origin_pending &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER &&
        (Off_Ground_Flag || chassis_off_ground ||
         landing_buffer_phase != LANDING_BUFFER_INACTIVE);
    const uint8_t jump_landing_handoff =
        jump_landing_owner_mode_allowed(
            landing_buffer_phase != LANDING_BUFFER_INACTIVE,
            landing_buffer_jump_origin,
            chassis_fdb_data.stand_state != CHASSIS_IS_DANGER,
            0U,
            jump_landing_jump_stand_allowed(
                chassis_cmd.ctrl_mode == CHASSIS_JUMP,
                0U,
                chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK));
    /* Keep the conservative ground detector alive for the short normal-axial
     * handoff after the jump-origin buffer has been released. */
    const uint8_t fast_exit_handoff =
        jump_landing_fast_exit_preserve_ground_detection(
            landing_fast_axial_handoff.active,
            chassis_cmd.ctrl_mode == CHASSIS_JUMP,
            chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);

    const uint8_t bump_action_scope =
        (bump_action_active || bump_airborne_lockout) &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER;

    return (normal_mode && normal_stand_state) || jump_mode ||
           jump_origin_pending ||
           jump_landing_handoff || fast_exit_handoff ||
           bump_action_scope || (bump_active && normal_stand_state);
}

static void offground_debug_capture_runtime(uint8_t airborne_allowed)
{
    uint32_t runtime_flags = 0U;
    uint32_t impact_flags = 0U;
    const uint8_t impact_count = offground_debug_impact_count > 15U ?
        15U : offground_debug_impact_count;

    if (Off_Ground_Flag) runtime_flags |= 1UL << 0;
    if (chassis_off_ground) runtime_flags |= 1UL << 1;
    if (Wheel_Shut_Flag) runtime_flags |= 1UL << 2;
    if (airborne_allowed) runtime_flags |= 1UL << 3;
    if (bump_active) runtime_flags |= 1UL << 4;
    if (jump_legs_unloaded) runtime_flags |= 1UL << 5;
    if (jump_air_profile_prestarted) runtime_flags |= 1UL << 6;
    runtime_flags |= ((uint32_t)landing_buffer_phase & 0x3UL) << 7;
    if (landing_contact_confirmed) runtime_flags |= 1UL << 9;
    if (landing_postbuffer_active) runtime_flags |= 1UL << 10;
    if (landing_airborne_guard_armed) runtime_flags |= 1UL << 11;
    if (offground_debug_last_takeoff_forced) runtime_flags |= 1UL << 12;
    if (offground_debug_last_takeoff_source_valid) runtime_flags |= 1UL << 13;
    if (offground_debug_snapshot.capture_valid) runtime_flags |= 1UL << 14;
    if (offground_debug_snapshot.detector_valid) runtime_flags |= 1UL << 15;

    if (offground_debug_impact_left) impact_flags |= 1UL << 14;
    if (offground_debug_impact_right) impact_flags |= 1UL << 15;
    if (offground_debug_impact_result) impact_flags |= 1UL << 16;
    if (landing_impact_armed) impact_flags |= 1UL << 17;
    if (offground_debug_impact_eval_valid) impact_flags |= 1UL << 18;
    impact_flags |= ((uint32_t)impact_count & 0xFUL) << 19;
#if OFFGROUND_DETECTION_ENABLE
    impact_flags |= 1UL << 23;
#endif

    offground_debug_snapshot.runtime_flags = runtime_flags;
    offground_debug_snapshot.impact_flags = impact_flags;
    offground_debug_snapshot.runtime_valid = 1U;
}

static float landing_real_dt(void)
{
    const float now_ms = dwt_get_time_ms();
    float dt = 0.001f;
    if (landing_update_last_ms > 0.0f) {
        dt = (now_ms - landing_update_last_ms) * 0.001f;
    }
    landing_update_last_ms = now_ms;
    if (!isfinite(dt) || dt < OFFGROUND_DT_MIN || dt > OFFGROUND_DT_MAX) {
        dt = 0.001f;
    }
    return dt;
}

static uint8_t bump_true_airborne_update(float dt_s)
{
    uint8_t accel_fresh = 0U;
    float accel_norm = NAN;

    if (safety_context != NULL &&
        (uint32_t)(HAL_GetTick() - ins.publish_tick_ms) <=
            BUMP_TRUE_AIR_IMU_MAX_AGE_MS &&
        isfinite(ins.accel[0]) && isfinite(ins.accel[1]) &&
        isfinite(ins.accel[2])) {
        accel_norm = sqrtf(ins.accel[0] * ins.accel[0] +
                           ins.accel[1] * ins.accel[1] +
                           ins.accel[2] * ins.accel[2]);
        accel_fresh = isfinite(accel_norm);
    }

    if (accel_fresh) {
        if (!bump_accel_norm_filter_initialized ||
            !isfinite(bump_accel_norm_filtered)) {
            bump_accel_norm_filtered = accel_norm;
            bump_accel_norm_filter_initialized = 1U;
        } else {
            float alpha = dt_s /
                (BUMP_TRUE_AIR_ACCEL_FILTER_TAU_S + dt_s);
            LIMIT_MIN_MAX(alpha, 0.0f, 1.0f);
            bump_accel_norm_filtered += alpha *
                (accel_norm - bump_accel_norm_filtered);
        }
    } else {
        bump_accel_norm_filter_initialized = 0U;
        bump_low_accel_elapsed_s = 0.0f;
    }

    if (bump_airborne_lockout) {
        const uint8_t supported_body = (uint8_t)(
            (landing_buffer_phase == LANDING_BUFFER_INACTIVE ||
             !landing_contact_confirmed) &&
            accel_fresh &&
            bump_accel_norm_filtered >=
                BUMP_GROUND_RECOVER_ACCEL_MIN_MPS2 &&
            bump_accel_norm_filtered <=
                BUMP_GROUND_RECOVER_ACCEL_MAX_MPS2);
        if (supported_body) {
            bump_ground_recover_elapsed_s += dt_s;
            if (bump_ground_recover_elapsed_s >=
                    BUMP_GROUND_RECOVER_CONFIRM_S) {
                if (landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
                    !landing_contact_confirmed) {
                    const uint8_t jump_origin_pending =
                        landing_jump_origin_pending;
                    landing_buffer_reset();
                    landing_jump_origin_pending = jump_origin_pending;
                }
                bump_airborne_lockout = 0U;
                bump_ground_recover_elapsed_s = 0.0f;
            }
        } else {
            bump_ground_recover_elapsed_s = 0.0f;
        }
    } else {
        bump_ground_recover_elapsed_s = 0.0f;
    }

    if (!bump_action_active) {
        bump_low_accel_elapsed_s = 0.0f;
        bump_accel_norm_filter_initialized = 0U;
        return 0U;
    }

    return bump_true_airborne_step_rising(
        chassis_off_ground, accel_fresh ? bump_accel_norm_filtered : NAN,
        BUMP_TRUE_AIR_ACCEL_THRESHOLD_MPS2, dt_s,
        BUMP_TRUE_AIR_CONFIRM_S, &bump_low_accel_elapsed_s,
        0U, &bump_airborne_lockout);
}

static uint8_t bump_phi0_within_safety_window(float phi0,
                                              uint8_t action_active)
{
    if (!isfinite(phi0)) {
        return 0U;
    }
    if (action_active) {
        return (uint8_t)(phi0 >= BUMP_PHI0_SAFE_MIN_RAD &&
                         phi0 <= BUMP_PHI0_SAFE_MAX_RAD);
    }
    return (uint8_t)(phi0 > LEG_SAFE_AREA * DEGREE_2_RAD &&
                     phi0 < PI - LEG_SAFE_AREA * DEGREE_2_RAD);
}

static float landing_buffer_average_length_speed(void)
{
    /* Jump-origin landing gates must use the same paired, timestamped rate
     * that feeds the landing PID.  A missing pair is explicitly unstable;
     * falling back to the older control-cycle observer could release RETURN
     * while the actual landing-rate input is stale. */
    if (landing_buffer_jump_origin || landing_postbuffer_active) {
        return jump_landing_average_abs_length_rate(
            jump_landing_length_rate_snapshot_valid,
            jump_landing_length_rate_snapshot[LEFT],
            jump_landing_length_rate_snapshot[RIGHT]);
    }
    const uint8_t jump_origin_scope = landing_buffer_jump_origin;
    const float legacy_left = leg[LEFT]->d_l0_lpf;
    const float legacy_right = leg[RIGHT]->d_l0_lpf;
    jump_local_rate_selection_t left = jump_local_rate_select(
        JUMP_LOCAL_RATE_FEEDBACK_ENABLE,
        JUMP_KINEMATIC_RATE_OBSERVER_ENABLE,
        jump_origin_scope,
        jump_rate_snapshot_usable,
        jump_length_rate_snapshot[LEFT],
        legacy_left);
    jump_local_rate_selection_t right = jump_local_rate_select(
        JUMP_LOCAL_RATE_FEEDBACK_ENABLE,
        JUMP_KINEMATIC_RATE_OBSERVER_ENABLE,
        jump_origin_scope,
        jump_rate_snapshot_usable,
        jump_length_rate_snapshot[RIGHT],
        legacy_right);
    return jump_local_rate_pair_average_abs(left, right);
}

static float jump_length_rate(chassis_side_e side)
{
    if (!jump_rate_snapshot_usable || side >= CHASSIS_SIDE_COUNT ||
        (!jump_rate_snapshot_fresh &&
         jump_rate_snapshot_age_ms > JUMP_KIN_RATE_HOLD_MAX_AGE_MS)) {
        return 0.0f;
    }
    return jump_length_rate_snapshot[side];
}

static float airborne_length_rate(chassis_side_e side)
{
    return chassis_cmd.ctrl_mode == CHASSIS_JUMP ?
        jump_length_rate(side) : leg[side]->d_l0_lpf;
}

static uint8_t landing_jump_origin_active(void)
{
    return (uint8_t)(landing_jump_origin_pending ||
                     chassis_cmd.ctrl_mode == CHASSIS_JUMP);
}

static void landing_air_deploy_begin(void)
{
    if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        landing_jump_origin_pending = 1U;
    }
    const uint8_t preserve_prestarted = (uint8_t)(
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        jump_air_profile_prestarted);
    jump_air_profile_begin(&jump_air_profile_state, preserve_prestarted);
    if (preserve_prestarted) {
        /* The release profile is already running. Keep its elapsed time,
         * references, and actual-compact dwell through the delayed edge. */
        jump_air_profile_prestarted = 0U;
        return;
    }

    landing_airborne_elapsed = 0.0f;
    if (chassis_cmd.ctrl_mode == CHASSIS_JUMP || bump_active) {
        offground_normal_split_guard_reset(&offground_normal_split_guard);
        /* Start from one common, measured takeoff length.  The update routine
         * then retracts both sides with the same bounded reference before it
         * deploys them toward the landing geometry. */
        float captured_common =
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        if (!isfinite(captured_common)) {
            captured_common = JUMP_AIR_LANDING_LENGTH;
        }
        LIMIT_MIN_MAX(captured_common,
                      JUMP_AIR_LANDING_LENGTH,
                      JUMP_AIR_CAPTURE_LENGTH_MAX);
        landing_air_deploy_ref[LEFT] = captured_common;
        landing_air_deploy_ref[RIGHT] = captured_common;
        return;
    }

    if (chassis_cmd.leg_level != LEG_LOW) {
        offground_normal_split_guard_begin(
            &offground_normal_split_guard, leg[LEFT]->l0, leg[RIGHT]->l0);
    } else {
        offground_normal_split_guard_reset(&offground_normal_split_guard);
    }

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        float length = leg[side]->l0;
        if (!isfinite(length)) {
            length = landing_requested_ref[side];
        }
        LIMIT_MIN_MAX(length, LANDING_LENGTH_REF_MIN,
                      LANDING_LENGTH_REF_MAX);
        /* Bumpless handoff from the existing 20 N airborne preload.  This is
         * an internal force-generating reference, so it may temporarily sit
         * above the physical leg range by the preload-matching offset.  The
         * previous 0.27 m upper clamp created an immediate 30-80 mm retract
         * step whenever takeoff was detected near natural extension. */
        landing_air_deploy_ref[side] = length +
            OFFGROUND_SOFT_FORCE / LANDING_AIR_DEPLOY_KP;
        LIMIT_MIN_MAX(landing_air_deploy_ref[side],
                      LANDING_LENGTH_REF_MIN,
                      LANDING_LENGTH_REF_MAX +
                          OFFGROUND_SOFT_FORCE /
                              LANDING_AIR_DEPLOY_KP);
    }
}

static void landing_air_deploy_update(float dt)
{
    if ((chassis_cmd.leg_level != LEG_LOW && !bump_active) ||
        landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        return;
    }

    landing_airborne_elapsed += dt;
    /* BUMP 复用跳跃的离地/落地缓冲参数，但不改变 jump_state。 */
    const uint8_t active_air_buffer =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP || bump_active;
    float target = LANDING_AIR_DEPLOY_LENGTH;
    float ref_rate = LANDING_AIR_DEPLOY_REF_RATE;
    if (active_air_buffer) {
        /* Keep the tuck reference until both measured legs have reached the
         * compact threshold and stayed there for the full dwell. The deadline
         * starts deployment even if the feedback is invalid or the legs never
         * track the compact target. */
        const uint8_t compact_phase =
            jump_air_profile_actual_compact_phase(
                &jump_air_profile_state, landing_airborne_elapsed, dt,
                jump_air_position_feedback_fresh,
                leg[LEFT]->l0, leg[RIGHT]->l0,
                landing_air_deploy_ref[LEFT], landing_air_deploy_ref[RIGHT],
                JUMP_AIR_ACTUAL_COMPACT_LENGTH,
                JUMP_AIR_TUCK_LENGTH,
                JUMP_AIR_ACTUAL_COMPACT_REF_TOL,
                chassis_cmd.ctrl_mode == CHASSIS_JUMP ?
                    JUMP_FLIGHT_COMPACT_HOLD : JUMP_AIR_ACTUAL_COMPACT_HOLD,
                chassis_cmd.ctrl_mode == CHASSIS_JUMP ?
                    JUMP_FLIGHT_COMPACT_HARD_MAX : JUMP_AIR_COMPACT_HARD_MAX);
        target = compact_phase ? JUMP_AIR_TUCK_LENGTH :
                                 JUMP_AIR_LANDING_LENGTH;
        ref_rate = compact_phase ? JUMP_AIR_TUCK_REF_RATE :
            (chassis_cmd.ctrl_mode == CHASSIS_JUMP ?
                JUMP_FLIGHT_DEPLOY_REF_RATE : JUMP_AIR_DEPLOY_REF_RATE);
    }
    const float max_step = ref_rate * dt;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        landing_air_deploy_ref[side] = move_towards(
            landing_air_deploy_ref[side], target, max_step);
    }
}

static void landing_airborne_guard_arm(void)
{
#if LANDING_AIRBORNE_GUARD_ENABLE
    landing_airborne_guard_elapsed_s = 0.0f;
    landing_airborne_guard_armed = 1U;
#else
    landing_airborne_guard_elapsed_s = 0.0f;
    landing_airborne_guard_armed = 0U;
#endif
}

static void landing_airborne_guard_clear(void)
{
    landing_airborne_guard_elapsed_s = 0.0f;
    landing_airborne_guard_armed = 0U;
}

/* Advanced once per control cycle from update_ground(), which also runs while
 * the robot is airborne.  Expires on its own so a normal standing robot is
 * never left with jump-grade authority. */
static void landing_airborne_guard_step(float dt_s)
{
    if (!landing_airborne_guard_armed) {
        return;
    }
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.050f) {
        return;
    }
    landing_airborne_guard_elapsed_s += dt_s;
    if (landing_airborne_guard_elapsed_s >= LANDING_AIRBORNE_GUARD_WINDOW_S) {
        landing_airborne_guard_clear();
    }
}

/* v18 (jump101 #2): a post-landing bounce keeps the jump attitude channels. */
uint8_t chassis_safety_module_landing_airborne_guard_active(void)
{
    return landing_airborne_guard_armed;
}

static void landing_postbuffer_reset(void)
{
    landing_postbuffer_active = 0U;
    landing_postbuffer_elapsed_s = 0.0f;
    landing_postbuffer_fade_elapsed_s = 0.0f;
}

/* A compact pair of legs before touchdown is still part of the jump flight.
 * Latching the shared buffer owner here prevents the generic impact path from
 * handing axial and attitude authority back to the grounded controller while
 * the chassis is falling.  Two consecutive samples reject a single noisy
 * length estimate without adding a long delay to the 0.22 s tuck deadline. */
static uint8_t landing_precontact_compact_latched(void)
{
    /* A real compact dwell belongs to FLIGHT until measured touchdown. */
    if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        landing_precontact_compact_count = 0U;
        return 0U;
    }
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE ||
        !chassis_off_ground ||
        !landing_jump_origin_active() ||
        !isfinite(landing_airborne_elapsed) ||
        landing_airborne_elapsed < LANDING_PRECONTACT_LATCH_TIME ||
        !isfinite(leg[LEFT]->l0) || !isfinite(leg[RIGHT]->l0)) {
        landing_precontact_compact_count = 0U;
        return 0U;
    }

    /* Jump flight already maintains a timestamped pair of length rates for
     * the landing D path.  Prefer that pair here as well; the legacy VMC LPF
     * can lag or change sign during the high acceleration tuck and then miss
     * the only short pre-contact window. */
    float left_rate = leg[LEFT]->d_l0_lpf;
    float right_rate = leg[RIGHT]->d_l0_lpf;
    if (jump_rate_snapshot_fresh && jump_rate_snapshot_usable) {
        const float observed_left = jump_length_rate(LEFT);
        const float observed_right = jump_length_rate(RIGHT);
        if (isfinite(observed_left) && isfinite(observed_right)) {
            left_rate = observed_left;
            right_rate = observed_right;
        }
    }

    if (leg[LEFT]->l0 <= LANDING_PRECONTACT_COMPACT_LENGTH &&
        leg[RIGHT]->l0 <= LANDING_PRECONTACT_COMPACT_LENGTH &&
        isfinite(left_rate) && isfinite(right_rate) &&
        left_rate < LANDING_PRECONTACT_COMPACT_RATE_MAX &&
        right_rate < LANDING_PRECONTACT_COMPACT_RATE_MAX) {
        if (landing_precontact_compact_count <
                LANDING_PRECONTACT_CONFIRM_CNT) {
            landing_precontact_compact_count++;
        }
    } else {
        landing_precontact_compact_count = 0U;
    }
    return (uint8_t)(landing_precontact_compact_count >=
                     LANDING_PRECONTACT_CONFIRM_CNT);
}

/* Landing RETURN may release the jump owner only after body attitude and
 * angular velocity are quiet.  Leg speed and contact alone are insufficient:
 * jump3 reached phase RETURN with pitch already past +35 deg, then the soft
 * danger path cleared the owner and the body continued to rotate. */
static uint8_t landing_buffer_attitude_velocity_settled(void)
{
    if (!isfinite(ins.pitch) || !isfinite(ins.roll) ||
        !isfinite(ins.gyro[0]) || !isfinite(ins.gyro[1])) {
        return 0U;
    }
    return (uint8_t)(
        fabsf(ins.pitch) <= LANDING_EXIT_ATTITUDE_PITCH_MAX_DEG &&
        fabsf(ins.roll) <= LANDING_EXIT_ATTITUDE_ROLL_MAX_DEG &&
        fabsf(ins.gyro[0]) <= LANDING_EXIT_ATTITUDE_RATE_MAX_RAD_S &&
        fabsf(ins.gyro[1]) <= LANDING_EXIT_ATTITUDE_RATE_MAX_RAD_S);
}

uint8_t chassis_safety_module_landing_postbuffer_active(void)
{
    return (uint8_t)(landing_postbuffer_active &&
                     !chassis_safety_module_bump_pose_owner_active());
}

/* 1.0 = keep the landing length-PID tuning, 0.0 = standing tuning. */
float chassis_safety_module_landing_postbuffer_gain(void)
{
#if LANDING_POSTBUFFER_REF_ENABLE
    if (!landing_postbuffer_active ||
        chassis_safety_module_bump_pose_owner_active()) {
        return 0.0f;
    }
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE ||
        landing_postbuffer_elapsed_s <= LANDING_POSTBUFFER_REF_HOLD_S) {
        return 1.0f;
    }
    float gain = 1.0f -
        landing_postbuffer_fade_elapsed_s / LANDING_POSTBUFFER_GAIN_FADE_S;
    LIMIT_MIN_MAX(gain, 0.0f, 1.0f);
    return gain;
#else
    return 0.0f;
#endif
}

/* Runs every cycle.  While the landing buffer is live it simply follows the
 * buffer reference; once the fast-exit releases the buffer it keeps a one-way
 * ratchet (follow compression with a small margin, never extend) for a bounded
 * hold, then fades back to the operator target.  Without this the leg target
 * snapped straight to 0.13 m and the length loop pulled the leg inward while
 * the robot was still compressing. */
static void landing_postbuffer_update(float dt_s)
{
    if (!landing_postbuffer_active) {
        return;
    }
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.050f) {
        dt_s = 0.001f;
    }
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            landing_postbuffer_ref[side] = landing_length_ref[side];
        }
        landing_postbuffer_elapsed_s = 0.0f;
        return;
    }
    /* A rebound must not consume the hold clock or ratchet the reference. */
    if (chassis_off_ground) {
        return;
    }
    landing_postbuffer_elapsed_s += dt_s;
    if (landing_postbuffer_elapsed_s <= LANDING_POSTBUFFER_REF_HOLD_S) {
        const float max_step = LANDING_LOW_RATCHET_REF_RATE * dt_s;
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            float target = leg[side]->l0 + LANDING_LOW_RATCHET_MARGIN;
            if (target < landing_requested_ref[side]) {
                target = landing_requested_ref[side];
            }
            if (target > landing_postbuffer_ref[side]) {
                target = landing_postbuffer_ref[side];
            }
            landing_postbuffer_ref[side] = move_towards(
                landing_postbuffer_ref[side], target, max_step);
        }
        return;
    }
    landing_postbuffer_fade_elapsed_s += dt_s;
    const float max_step = LANDING_POSTBUFFER_FADE_RATE * dt_s;
    uint8_t reached = 1U;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        landing_postbuffer_ref[side] = move_towards(
            landing_postbuffer_ref[side], landing_requested_ref[side],
            max_step);
        if (fabsf(landing_postbuffer_ref[side] -
                  landing_requested_ref[side]) > 0.0005f) {
            reached = 0U;
        }
    }
    /* Keep the window (and therefore the landing PID tuning) until the gain
     * blend has finished too, otherwise the gains would step back early. */
    if (reached &&
        landing_postbuffer_fade_elapsed_s >= LANDING_POSTBUFFER_GAIN_FADE_S) {
        landing_postbuffer_active = 0U;
    }
}

static void landing_buffer_reset(void)
{
    landing_buffer_phase = LANDING_BUFFER_INACTIVE;
    landing_buffer_jump_origin = 0U;
    landing_buffer_phase_timer = 0.0f;
    landing_buffer_elapsed = 0.0f;
    landing_precontact_elapsed = 0.0f;
    landing_precontact_compact_count = 0U;
    landing_exit_stable_elapsed = 0.0f;
    landing_contact_confirmed = 0U;
    if (!chassis_off_ground) {
        landing_jump_origin_pending = 0U;
    }
    /* The cushion keeps serving its own window across this reset; only the
     * fast-exit confirmation belongs to the buffer's lifetime. */
    jump_landing_fast_exit_confirm_reset(&landing_fast_exit_confirm);
    landing_fast_exit_ready_latched = 0U;
    offground_normal_split_guard_reset(&offground_normal_split_guard);
}

static void landing_buffer_begin(uint8_t contact_confirmed)
{
    offground_debug_count_increment(&offground_debug_buffer_begins);
    /* The landing buffer owns both leg axes from this point onward. */
    offground_normal_split_guard_reset(&offground_normal_split_guard);
    jump_air_profile_reset(&jump_air_profile_state);
    landing_buffer_jump_origin = landing_jump_origin_active();
    if (contact_confirmed) {
        landing_jump_origin_pending = 0U;
    }
    if (landing_buffer_jump_origin) {
        /* Arm the post-landing airborne guard at the jump-origin touchdown.
         * It survives this buffer's fast-exit reset. */
        landing_airborne_guard_arm();
    }
    float contact_length[CHASSIS_SIDE_COUNT];
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        contact_length[side] = leg[side]->l0;
        if (!isfinite(contact_length[side])) {
            contact_length[side] = LANDING_BUFFER_MID_LENGTH;
        }
        LIMIT_MIN_MAX(contact_length[side],
                      LANDING_LENGTH_REF_MIN,
                      LANDING_LENGTH_REF_MAX);
    }

    /* One wheel often touches first after a jump.  Do not preserve a large
     * left/right reference split throughout the impact phase.  For LOW, keep
     * the shorter leg untouched (never inject extension energy) and shorten
     * only the longer reference until the split is bounded. */
    float low_contact_ref_max = fminf(contact_length[LEFT],
                                     contact_length[RIGHT]) +
                                LANDING_LOW_CONTACT_SPLIT_MAX;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        float contact_ref = contact_length[side];
        if (chassis_cmd.leg_level == LEG_LOW &&
            contact_ref > low_contact_ref_max) {
            contact_ref = low_contact_ref_max;
        }
        landing_length_ref[side] = contact_ref;
        const float absorb_target =
            (chassis_cmd.leg_level == LEG_LOW) ?
            landing_requested_ref[side] : LANDING_BUFFER_MID_LENGTH;
        landing_absorb_ref[side] = fminf(contact_ref, absorb_target);
    }
#if LANDING_POSTBUFFER_REF_ENABLE
    if (landing_buffer_jump_origin) {
        /* Seed the post-buffer hold from the captured contact geometry. */
        landing_postbuffer_ref[LEFT] = landing_length_ref[LEFT];
        landing_postbuffer_ref[RIGHT] = landing_length_ref[RIGHT];
        landing_postbuffer_elapsed_s = 0.0f;
        landing_postbuffer_fade_elapsed_s = 0.0f;
        landing_postbuffer_active = 1U;
    } else {
        landing_postbuffer_active = 0U;
    }
#else
    landing_postbuffer_active = 0U;
#endif
    landing_buffer_phase = LANDING_BUFFER_COMPRESS;
    landing_precontact_compact_count = 0U;
    landing_buffer_phase_timer = 0.0f;
    landing_buffer_elapsed = 0.0f;
    landing_precontact_elapsed = 0.0f;
    landing_exit_stable_elapsed = 0.0f;
    landing_contact_confirmed = contact_confirmed ? 1U : 0U;
}

static void landing_buffer_update(float dt)
{
    if (landing_buffer_phase == LANDING_BUFFER_INACTIVE) {
        return;
    }

    landing_buffer_phase_timer += dt;
    landing_buffer_elapsed += dt;
    if (!landing_contact_confirmed) {
        landing_precontact_elapsed += dt;
    }

    switch (landing_buffer_phase) {
    case LANDING_BUFFER_COMPRESS: {
        const float max_step = LANDING_COMPRESS_REF_RATE * dt;
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            landing_length_ref[side] = move_towards(
                landing_length_ref[side], landing_absorb_ref[side],
                max_step);
        }
        const uint8_t absorb_reference_reached =
            fabsf(landing_length_ref[LEFT] - landing_absorb_ref[LEFT]) <
                0.001f &&
            fabsf(landing_length_ref[RIGHT] - landing_absorb_ref[RIGHT]) <
                0.001f;
        if (absorb_reference_reached ||
            landing_buffer_phase_timer >= LANDING_COMPRESS_MAX_TIME) {
            landing_buffer_phase = LANDING_BUFFER_HOLD;
            landing_buffer_phase_timer = 0.0f;
        }
        break;
    }

    case LANDING_BUFFER_HOLD: {
        /* A one-wheel impact may begin the buffer before the conservative
         * two-wheel ground flag clears.  jump27 showed the LOW ratchet then
         * following the still-airborne leg motion from about 0.29 m down to
         * 0.16 m, removing nearly all landing stroke before real contact.
         * Keep the captured first-contact geometry until landing is actually
         * confirmed; only the confirmed impact may consume that stroke. */
        if (!landing_contact_confirmed) {
            break;
        }
        if (chassis_cmd.leg_level == LEG_LOW) {
            /* A one-way reference follows real compression with a small
             * spring margin, but never rises with rebound.  It preserves
             * braking force without returning stored spring energy. */
            const float max_step = LANDING_LOW_RATCHET_REF_RATE * dt;
            for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
                float ratchet_target =
                    leg[side]->l0 + LANDING_LOW_RATCHET_MARGIN;
                if (ratchet_target < landing_requested_ref[side]) {
                    ratchet_target = landing_requested_ref[side];
                }
                if (ratchet_target > landing_length_ref[side]) {
                    ratchet_target = landing_length_ref[side];
                }
                landing_length_ref[side] = move_towards(
                    landing_length_ref[side], ratchet_target, max_step);
            }
        } else {
            const float max_step = LANDING_COMPRESS_REF_RATE * dt;
            for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
                landing_length_ref[side] = move_towards(
                    landing_length_ref[side], landing_absorb_ref[side],
                    max_step);
            }
        }

        const float average_length =
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        const float average_absorb_ref =
            (chassis_cmd.leg_level == LEG_LOW) ?
            0.5f * (landing_length_ref[LEFT] + landing_length_ref[RIGHT]) :
            0.5f * (landing_absorb_ref[LEFT] + landing_absorb_ref[RIGHT]);
        const float average_length_speed =
            landing_buffer_average_length_speed();
        const uint8_t settled =
            fabsf(average_length - average_absorb_ref) <=
                LANDING_SETTLE_LENGTH_ERROR &&
            average_length_speed <= LANDING_SETTLE_LENGTH_SPEED;
        /* Rebound is part of the same impact, not permission to leave the
         * buffer.  The previous early-release condition ended LOW buffering
         * while the chassis was still airborne and made the reference toggle
         * between the captured contact length and 0.27 m. */
        if ((landing_buffer_phase_timer >= LANDING_HOLD_MIN_TIME && settled) ||
            landing_buffer_phase_timer >= LANDING_HOLD_MAX_TIME) {
            landing_buffer_phase = LANDING_BUFFER_RETURN;
            landing_buffer_phase_timer = 0.0f;
            landing_exit_stable_elapsed = 0.0f;
        }
        break;
    }

    case LANDING_BUFFER_RETURN: {
        const float return_rate =
            (chassis_cmd.leg_level == LEG_LOW) ?
            LANDING_RETURN_REF_RATE_LOW : LANDING_RETURN_REF_RATE;
        const float max_step = return_rate * dt;
        /* A jump-origin LOW landing is still dissipating impact energy here.
         * The old RETURN path always chased landing_requested_ref (normally
         * 0.13 m), so a leg that had compressed to about 0.11 m was actively
         * extended during rebound.  That second upward impulse is exactly the
         * ``landing takeoff'' seen in jump1.csv.  Keep this reference one-way:
         * it may follow further measured compression with a small margin, but
         * it can never rise when the leg rebounds.  The postbuffer owns the
         * later, slow return to the operator target after stable contact. */
        const uint8_t jump_low_ratchet =
            landing_buffer_jump_origin &&
            chassis_cmd.leg_level == LEG_LOW;
        uint8_t target_reached = 1U;
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            float target = landing_requested_ref[side];
            if (jump_low_ratchet) {
                float ratchet_target = leg[side]->l0 +
                    LANDING_LOW_RATCHET_MARGIN;
                if (!isfinite(ratchet_target)) {
                    ratchet_target = landing_length_ref[side];
                }
                /* Never inject extension energy during this phase. */
                if (ratchet_target > landing_length_ref[side]) {
                    ratchet_target = landing_length_ref[side];
                }
                target = ratchet_target;
            }
            landing_length_ref[side] = move_towards(
                landing_length_ref[side], target, max_step);
            if (!jump_low_ratchet &&
                fabsf(landing_length_ref[side] - target) > 0.0005f) {
                target_reached = 0U;
            }
        }
        const float average_length_speed =
            landing_buffer_average_length_speed();
        const uint8_t attitude_settled =
            !landing_buffer_jump_origin ||
            landing_buffer_attitude_velocity_settled();
        const uint8_t stable_ground =
            !chassis_off_ground &&
            average_length_speed <= LANDING_EXIT_LENGTH_SPEED &&
            attitude_settled;
        landing_exit_stable_elapsed =
            jump_landing_exit_stable_elapsed_step(
                landing_exit_stable_elapsed, dt,
                (uint8_t)(target_reached && stable_ground));
        if (landing_contact_confirmed && target_reached &&
            landing_buffer_phase_timer >= LANDING_PID_RETURN_TIME &&
            landing_exit_stable_elapsed >= LANDING_EXIT_STABLE_TIME) {
            landing_buffer_reset();
        }
        break;
    }

    case LANDING_BUFFER_INACTIVE:
    default:
        landing_buffer_reset();
        break;
    }

    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
        chassis_cmd.leg_level == LEG_LOW) {
        /* The LOW ratchet is intentionally one-way, but independent ratchets
         * can recreate a large split after first contact.  Continue bounding
         * only the longer reference; never extend the shorter side. */
        const float ref_max = fminf(landing_length_ref[LEFT],
                                    landing_length_ref[RIGHT]) +
                              LANDING_LOW_CONTACT_SPLIT_MAX;
        if (landing_length_ref[LEFT] > ref_max) {
            landing_length_ref[LEFT] = ref_max;
        }
        if (landing_length_ref[RIGHT] > ref_max) {
            landing_length_ref[RIGHT] = ref_max;
        }
    }
}

static uint8_t landing_impact_condition(void)
{
    if (!offground_legacy_scope_active &&
        (!offground_measured_fn_valid || !offground_ins_feedback_fresh)) {
        return 0U;
    }
    const float force_threshold = Offground_LandingThreshold();
    const float length_speed_threshold =
        (chassis_cmd.leg_level == LEG_LOW) ?
        LANDING_IMPACT_LENGTH_SPEED_LOW :
        LANDING_IMPACT_LENGTH_SPEED;
    const uint8_t jump_mode = chassis_cmd.ctrl_mode == CHASSIS_JUMP || bump_active;
    /* For jump flight, the robot tucks then deploys. Impact can only occur
     * when the legs have actually deployed past the tuck position. */
    const float min_extension =
        jump_mode ? (JUMP_AIR_TUCK_LENGTH + 0.05f) : 0.0f;

    const uint8_t left_impact =
        offground_selected_fn(LEFT) > force_threshold &&
        leg[LEFT]->d_l0_lpf < length_speed_threshold &&
        leg[LEFT]->l0 >= min_extension;
    const uint8_t right_impact =
        offground_selected_fn(RIGHT) > force_threshold &&
        leg[RIGHT]->d_l0_lpf < length_speed_threshold &&
        leg[RIGHT]->l0 >= min_extension;
    offground_debug_impact_left = left_impact;
    offground_debug_impact_right = right_impact;
    offground_debug_impact_result = left_impact || right_impact;
    offground_debug_impact_eval_valid = 1U;
    return offground_debug_impact_result;
}

static void landing_impact_update(float dt)
{
    offground_debug_impact_clear();
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        /* A speculative impact can start the buffer before the filtered
         * ground detector confirms touchdown.  While still airborne, freeze
         * the captured references and phase timers: advancing COMPRESS/HOLD/
         * RETURN here can consume the landing stroke in mid-air, then apply
         * an extension command when the chassis finally touches down. */
        if (chassis_off_ground) {
            landing_exit_stable_elapsed =
                jump_landing_exit_stable_elapsed_step(
                    landing_exit_stable_elapsed, dt, 0U);
            return;
        }
        landing_buffer_update(dt);
        if (!landing_contact_confirmed &&
            landing_precontact_elapsed >= LANDING_PRECONTACT_TIMEOUT) {
            landing_buffer_reset();
            landing_impact_count = 0U;
            landing_impact_rearm_timer = 0.0f;
        }
        return;
    }

    /* The compact airborne profile can finish before force based impact
     * detection becomes reliable.  Claim the landing owner after two compact
     * samples, while keeping contact_confirmed clear so the off-ground path
     * still uses reaction-wheel and theta authority. */
    if (landing_precontact_compact_latched()) {
        landing_buffer_begin(0U);
        landing_impact_armed = 0U;
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
        pid_clear(L_length_pid);
        pid_clear(R_length_pid);
        return;
    }

    if (!offground_legacy_scope_active &&
        (!offground_measured_fn_valid || !offground_ins_feedback_fresh)) {
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
        return;
    }

    if (!landing_impact_armed) {
        /* During active jump flight, do not arm impact detection while the robot
         * is in the initial tuck and hold phases. Mid-air retractions generate
         * high virtual forces that must not trigger premature landing buffering. */
        if (landing_jump_origin_active() || bump_active) {
            const float min_flight_time =
                JUMP_AIR_TUCK_TIME + JUMP_AIR_TUCK_HOLD_TIME;
            if (landing_airborne_elapsed < min_flight_time) {
                landing_impact_rearm_timer = 0.0f;
                landing_impact_armed = 0U;
                landing_impact_count = 0U;
                return;
            }
        }
        if (!landing_impact_condition()) {
            landing_impact_rearm_timer += dt;
            if (landing_impact_rearm_timer >= LANDING_IMPACT_REARM_TIME) {
                landing_impact_armed = 1U;
                landing_impact_count = 0U;
            }
        } else {
            landing_impact_rearm_timer = 0.0f;
        }
        return;
    }

    if (landing_impact_condition()) {
        if (landing_impact_count < LANDING_IMPACT_CONFIRM_CNT) {
            landing_impact_count++;
        }
        offground_debug_impact_count = landing_impact_count;
    } else {
        landing_impact_count = 0U;
    }

    if (landing_impact_count >= LANDING_IMPACT_CONFIRM_CNT) {
        const uint8_t jump_origin = landing_jump_origin_active();
        if (!jump_landing_impact_admitted(
                jump_origin, chassis_off_ground)) {
            /* Wait for the real landing edge to open a jump-origin buffer;
             * airborne force reconstruction must not start precontact HOLD. */
            landing_impact_count = 0U;
            landing_impact_rearm_timer = 0.0f;
            return;
        }
        landing_buffer_begin(0U);
        landing_impact_armed = 0U;
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
        pid_clear(L_length_pid);
        pid_clear(R_length_pid);
    }
}

static void airborne_state_clear(void)
{
    offground_debug_count_increment(&offground_debug_clear_calls);
    Off_Ground_Flag = 0;
    Off_Ground_Cnt = 0;
    Touch_Ground_Cnt = 0;
    Touch_Ground_Flag = 0;
    Wheel_Shut_Flag = 0;
    offground_shadow_takeoff_active = 0;
    chassis_off_ground = 0;
    chassis_off_ground_last = 0;
    chassis_takeoff_edge = 0;
    chassis_landing_edge = 0;
    landing_yaw_hold_timer = 0.0f;
    landing_buffer_reset();
    landing_impact_armed = 0U;
    landing_impact_count = 0U;
    landing_impact_rearm_timer = 0.0f;
    landing_air_length_sync_force = 0.0f;
    offground_normal_split_guard_reset(&offground_normal_split_guard);
    jump_air_profile_prestarted = 0U;
    jump_air_profile_reset(&jump_air_profile_state);
    jump_legs_unloaded = 0U;
    landing_airborne_elapsed = 0.0f;
    jump_landing_cushion_reset(&landing_cushion);
    landing_airborne_guard_clear();
    landing_postbuffer_reset();
}

static void Security_Checking(void)
{
    danger_reason_e soft_candidate = DANGER_NONE;
    float dt = chassis_dt;
    if (!isfinite(dt) || dt < OFFGROUND_DT_MIN || dt > OFFGROUND_DT_MAX) {
        dt = 0.001f;
    }

    dbg_danger_reason = DANGER_NONE;
    /* Mechanical limits and a fully overturned body remain immediate faults.
     * The lower fall thresholds below are debounced so a landing impact or a
     * running jump cannot enter RECOVERY from one noisy sample. */
    if (fabsf(ins.pitch) > DANGER_HARD_ATTITUDE_DEG) {
        dbg_danger_reason = DANGER_PITCH;
    } else if (fabsf(ins.roll) > DANGER_HARD_ATTITUDE_DEG) {
        dbg_danger_reason = DANGER_ROLL;
    } else if (!bump_phi0_within_safety_window(
                   leg[LEFT]->phi0, bump_action_active) &&
               (!isfinite(leg[LEFT]->phi0) ||
               (bump_action_active ?
                    leg[LEFT]->phi0 < BUMP_PHI0_SAFE_MIN_RAD :
                    leg[LEFT]->phi0 <= LEG_SAFE_AREA * DEGREE_2_RAD))) {
        dbg_danger_reason = DANGER_LEFT_PHI_LOW;
    } else if (!bump_phi0_within_safety_window(
                   leg[LEFT]->phi0, bump_action_active)) {
        dbg_danger_reason = DANGER_LEFT_PHI_HIGH;
    } else if (!bump_phi0_within_safety_window(
                   leg[RIGHT]->phi0, bump_action_active) &&
               (!isfinite(leg[RIGHT]->phi0) ||
               (bump_action_active ?
                    leg[RIGHT]->phi0 < BUMP_PHI0_SAFE_MIN_RAD :
                    leg[RIGHT]->phi0 <= LEG_SAFE_AREA * DEGREE_2_RAD))) {
        dbg_danger_reason = DANGER_RIGHT_PHI_LOW;
    } else if (!bump_phi0_within_safety_window(
                   leg[RIGHT]->phi0, bump_action_active)) {
        dbg_danger_reason = DANGER_RIGHT_PHI_HIGH;
    }

    /* A grounded chassis can fall while phi0 remains inside its broad
     * mechanical range.  Detect the actual body/leg-rod pose as well.  Airborne
     * theta excursions are handled by JUMP/landing and are not classified as
     * a fall until contact returns.
     *
     * 2026-10-07 (bump4.csv): BUMP 期间必须豁免 theta 类判据。
     *
     * BUMP 的爬台阶动作本来就要把腿摆到远超 DANGER_SOFT_THETA_RAD(45°) 的角度：
     *   · 接触瞬间实测 theta 已达 46~52°；
     *   · 摆动阶段 SWING_POSE 的目标 phi0=30°（照搬 recovery），
     *     对应 theta = 90° + pitch − 30° ≈ 60~85°。
     * 实测后果（三次尝试全部失败）：t=93.230 theta_R 到 45.4° → 约 16 ms 后
     * dbg_danger_reason = DANGER_RIGHT_THETA → stand_state = CHASSIS_IS_DANGER
     * → dr16 模式阶梯因 stand_state 不是 STAND/JUMPOK 而落到最后 else
     * → ctrl_mode = CHASSIS_RECOVERY → dr16 里 previous==RECOVERY 又把
     * bump_mode 清 0 → bump 立刻退出到 IDLE。尝试1 的 theta 到 69.6°，同理。
     * 注意确认时间实际只有约 16 ms 而非标称的 80 ms —— chassis_dt=0.005 而
     * 控制循环是 1 ms，定时器跑快了 5 倍。
     *
     * 处理方式与 jump 一致（jump 在离地时同样豁免 theta 判据）：bump_active
     * 时跳过 theta 候选。姿态保护仍由 pitch/roll 阈值和 phi0 机械限位承担，
     * 所以真正倒地依然会被判出来。 */
    /* A jump-origin landing buffer deliberately allows the rods to swing
     * through the ordinary grounded theta band while it absorbs impact.  The
     * airborne theta guard is already owner-gated in the jump controller; the
     * grounded soft danger check must use the same ownership window or it will
     * set CHASSIS_IS_DANGER on the first rebound and clear the buffer.  Keep
     * hard pitch/roll and phi0 mechanical limits active below. */
    const uint8_t jump_landing_pose_owner =
        jump_landing_pose_owner_active(
            chassis_safety_module_jump_landing_owner_active(),
            landing_postbuffer_active,
            landing_buffer_phase != LANDING_BUFFER_INACTIVE,
            landing_buffer_jump_origin);
    const uint8_t jump_landing_attitude_owner =
        (uint8_t)(landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
                  landing_buffer_jump_origin &&
                  landing_contact_confirmed && !chassis_off_ground);
    if (dbg_danger_reason == DANGER_NONE && !chassis_off_ground &&
        !bump_active) {
        /* A confirmed jump landing may pass through the soft 35/30 deg band
         * while the wheel and leg-angle owners are still correcting it.  Do
         * not convert that transient into DANGER and clear the buffer; the
         * hard 60 deg attitude limit above remains active. */
        if (fabsf(ins.pitch) > DANGER_SOFT_PITCH_DEG &&
            !jump_landing_attitude_owner) {
            soft_candidate = DANGER_PITCH;
        } else if (fabsf(ins.roll) > DANGER_SOFT_ROLL_DEG &&
                   !jump_landing_attitude_owner) {
            soft_candidate = DANGER_ROLL;
        } else if (!jump_landing_pose_owner &&
                   fabsf(leg[LEFT]->theta) > DANGER_SOFT_THETA_RAD) {
            soft_candidate = DANGER_LEFT_THETA;
        } else if (!jump_landing_pose_owner &&
                   fabsf(leg[RIGHT]->theta) > DANGER_SOFT_THETA_RAD) {
            soft_candidate = DANGER_RIGHT_THETA;
        }
    }

    if (dbg_danger_reason == DANGER_NONE &&
        soft_candidate != DANGER_NONE) {
        if (soft_candidate != danger_candidate_last) {
            danger_candidate_last = soft_candidate;
            danger_candidate_timer = 0.0f;
        }
        danger_candidate_timer += dt;
        if (danger_candidate_timer >= DANGER_SOFT_CONFIRM_TIME) {
            dbg_danger_reason = soft_candidate;
        }
    } else {
        danger_candidate_last = DANGER_NONE;
        danger_candidate_timer = 0.0f;
    }

    if (dbg_danger_reason != DANGER_NONE) {
        danger_reason_latched = dbg_danger_reason;
        chassis_fdb_data.stand_state = CHASSIS_IS_DANGER;
    }
}

void chassis_safety_module_bind(const chassis_safety_context_t *context)
{
    safety_context = context;
}

void chassis_safety_module_set_position_feedback(
    const float length[CHASSIS_SIDE_COUNT],
    const float theta[CHASSIS_SIDE_COUNT],
    const uint32_t rx_count[4], uint8_t feedback_fresh,
    float max_feedback_age_ms)
{
    if (safety_context == NULL || length == NULL || theta == NULL ||
        rx_count == NULL) {
        return;
    }

    offground_source_scope_update();
    offground_position_feedback_fresh = feedback_fresh ? 1U : 0U;
    offground_ins_feedback_fresh = offground_ins_sample_fresh();
    offground_feedback_max_age_ms =
        (feedback_fresh && isfinite(max_feedback_age_ms) &&
         max_feedback_age_ms >= 0.0f) ? max_feedback_age_ms : -1.0f;
    if (offground_legacy_scope_active) {
        offground_position_view = (offground_kin_rate_view_t){
            .position_age_ms = INFINITY,
            .rate_age_ms = INFINITY
        };
        offground_measured_fn_valid = 0U;
        return;
    }

    offground_position_view = offground_kin_observer_update(
        &offground_position_observer, length, theta, rx_count,
        (uint8_t)(feedback_fresh && offground_ins_feedback_fresh),
        dwt_get_time_us());
    if (offground_position_view.new_position_sample) {
        offground_last_position_sample_dt_ms =
            offground_position_view.position_sample_dt_ms;
    }
    if (!offground_position_view.valid) {
        offground_measured_filter_reset();
        offground_confirmation_clear_candidates();
        Off_Ground_Cnt = 0U;
        Touch_Ground_Cnt = 0U;
    }
    if (!offground_ins_feedback_fresh) {
        offground_measured_filter_reset();
        offground_confirmation_clear_candidates();
        Off_Ground_Cnt = 0U;
        Touch_Ground_Cnt = 0U;
    }
}

void chassis_safety_module_reset(void)
{
    offground_debug_count_increment(&offground_debug_clear_calls);
    offground_debug_snapshot.capture_valid = 0U;
    offground_debug_snapshot.detector_valid = 0U;
    offground_debug_snapshot.runtime_valid = 0U;
    offground_debug_snapshot.runtime_flags = 0U;
    offground_debug_snapshot.impact_flags = 0U;
    offground_debug_capture_clock_valid = 0U;
    offground_debug_impact_eval_valid = 0U;
    offground_debug_impact_left = 0U;
    offground_debug_impact_right = 0U;
    offground_debug_impact_result = 0U;
    offground_debug_impact_count = 0U;
    offground_kin_observer_reset(&offground_position_observer);
    offground_confirmation_reset(&offground_confirmation);
    offground_confirmation_clock_valid = 0U;
    offground_confirmation_last_us = 0U;
    offground_position_view = (offground_kin_rate_view_t){
        .position_age_ms = INFINITY,
        .rate_age_ms = INFINITY
    };
    offground_feedback_max_age_ms = -1.0f;
    offground_position_feedback_fresh = 0U;
    offground_ins_feedback_fresh = 0U;
    offground_confirmation_evidence_valid = 0U;
    offground_last_position_sample_dt_ms = 0.0f;
    offground_scope_initialized = 0U;
    offground_measured_filter_reset();
    jump_air_profile_reset(&jump_air_profile_state);
    jump_air_position_feedback_fresh = 0U;
    jump_landing_length_rate_snapshot[CHASSIS_SIDE_LEFT] = 0.0f;
    jump_landing_length_rate_snapshot[CHASSIS_SIDE_RIGHT] = 0.0f;
    jump_landing_length_rate_snapshot_valid = 0U;
    jump_landing_axial_handoff_cancel(&landing_fast_axial_handoff);
    landing_fast_pid_seed_pending = 0U;
    jump_landing_fast_exit_confirm_reset(&landing_fast_exit_confirm);
    landing_fast_exit_ready_latched = 0U;
    jump_landing_cushion_reset(&landing_cushion);
    landing_airborne_guard_clear();
    landing_postbuffer_reset();
    landing_jump_origin_pending = 0U;
    if (safety_context != NULL) {
        bump_active = 0U;
        bump_action_active = 0U;
        bump_airborne_lockout = 0U;
        bump_low_accel_elapsed_s = 0.0f;
        bump_accel_norm_filtered = 0.0f;
        bump_accel_norm_filter_initialized = 0U;
        bump_ground_recover_elapsed_s = 0.0f;
        danger_candidate_last = DANGER_NONE;
        danger_candidate_timer = 0.0f;
        offground_filter_initialized = 0U;
        offground_filter_index = 0U;
        offground_derivative_initialized = 0U;
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            offground_fn_raw[side] = 0.0f;
            offground_fn_filtered[side] = 0.0f;
            offground_last_d_l0[side] = 0.0f;
            offground_last_d_theta[side] = 0.0f;
            for (uint8_t tap = 0U; tap < OFFGROUND_FILTER_LENGTH; tap++) {
                offground_fn_history[side][tap] = 0.0f;
            }
        }
        airborne_state_clear();
    }
}

static uint8_t landing_fast_exit_mode_allowed(void)
{
    const uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL;
    const uint8_t normal_stand =
        chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK;
    const uint8_t jump_mode = chassis_cmd.ctrl_mode == CHASSIS_JUMP;
    const uint8_t jump_stand =
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPING ||
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK;
    return (uint8_t)((normal_mode && normal_stand) ||
                     (jump_mode && jump_stand));
}

uint8_t chassis_safety_module_landing_fast_exit_ready(
    float left_length, float right_length)
{
    if (safety_context == NULL ||
        chassis_safety_module_bump_pose_owner_active()) {
        jump_landing_fast_exit_confirm_reset(&landing_fast_exit_confirm);
        landing_fast_exit_ready_latched = 0U;
        return 0U;
    }
    /* One evaluation per control cycle: the eligibility has to hold for
     * JUMP_LANDING_FAST_EXIT_CONFIRM_CYCLES consecutive cycles, so the caller
     * cannot re-evaluate inside the same cycle. */
    /* HOLD entry is still part of the impact transient.  Require the phase
     * dwell, low common compression speed, and both legs near their current
     * buffer references before allowing the short normal-control handoff. */
    const uint8_t buffer_hold =
        landing_buffer_phase == LANDING_BUFFER_HOLD;
    const uint8_t buffer_settled = jump_landing_fast_exit_hold_settled(
        buffer_hold, landing_buffer_phase_timer, LANDING_HOLD_MIN_TIME,
        landing_buffer_average_length_speed(), LANDING_SETTLE_LENGTH_SPEED,
        left_length, right_length,
        landing_length_ref[LEFT], landing_length_ref[RIGHT],
        LANDING_SETTLE_LENGTH_ERROR);
    const uint8_t eligible = jump_landing_fast_exit_eligible(
        landing_buffer_phase != LANDING_BUFFER_INACTIVE,
        buffer_hold, buffer_settled,
        landing_buffer_jump_origin, landing_contact_confirmed,
        chassis_off_ground, landing_fast_exit_mode_allowed(),
        left_length, right_length);
    landing_fast_exit_ready_latched = jump_landing_fast_exit_confirm_step(
        &landing_fast_exit_confirm, eligible);
    return landing_fast_exit_ready_latched;
}

uint8_t chassis_safety_module_landing_fast_exit_begin(void)
{
    if (safety_context == NULL ||
        chassis_safety_module_bump_pose_owner_active() ||
        !landing_fast_exit_ready_latched ||
        !isfinite(leg[LEFT]->support_force) ||
        !isfinite(leg[RIGHT]->support_force)) {
        return 0U;
    }
    if (!jump_landing_axial_handoff_begin(
            &landing_fast_axial_handoff,
            leg[LEFT]->support_force, leg[RIGHT]->support_force)) {
        return 0U;
    }
    /* The caller finishes JUMP first, preserving theta/wheel owner history.
     * Reset only the axial buffer state after capturing its last support. */
    landing_fast_pid_seed_pending = 1U;
    landing_buffer_reset();
    return 1U;
}

uint8_t chassis_safety_module_landing_fast_exit_take_pid_seed(void)
{
    const uint8_t pending = landing_fast_pid_seed_pending;
    landing_fast_pid_seed_pending = 0U;
    return pending;
}

uint8_t chassis_safety_module_landing_fast_exit_active(void)
{
    return landing_fast_axial_handoff.active;
}

void chassis_safety_module_landing_fast_exit_cancel_if_unsafe(void)
{
    const uint8_t safe_ground = (uint8_t)(
        !chassis_off_ground && safety_context != NULL &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER &&
        chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY &&
        chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY &&
        chassis_cmd.ctrl_mode != CHASSIS_RELAX &&
        chassis_cmd.ctrl_mode != CHASSIS_INIT &&
        chassis_cmd.ctrl_mode != CHASSIS_RECOVERY &&
        landing_fast_exit_mode_allowed());
    if (!safe_ground) {
        jump_landing_axial_handoff_cancel(&landing_fast_axial_handoff);
        landing_fast_pid_seed_pending = 0U;
        jump_landing_fast_exit_confirm_reset(&landing_fast_exit_confirm);
        landing_fast_exit_ready_latched = 0U;
    }
}

void chassis_safety_module_landing_fast_exit_finish_cycle(void)
{
    chassis_safety_module_landing_fast_exit_cancel_if_unsafe();
    jump_landing_axial_handoff_finish_cycle(
        &landing_fast_axial_handoff,
        landing_fast_axial_handoff.active);
}

void chassis_safety_module_landing_fast_exit_apply_support(
    float dt_s, float *left_force, float *right_force)
{
    if (safety_context == NULL || left_force == NULL || right_force == NULL) {
        return;
    }
    if (chassis_safety_module_bump_pose_owner_active()) {
        return;
    }
    chassis_safety_module_landing_fast_exit_cancel_if_unsafe();
    const uint8_t safe_ground = landing_fast_axial_handoff.active;
    jump_landing_axial_handoff_apply(
        &landing_fast_axial_handoff, dt_s, safe_ground,
        *left_force, *right_force, left_force, right_force);
}

/* Length rate used by the landing cushion.  The timestamp kinematic observer is
 * preferred because the legacy VMC derivative is computed with the fixed 5 ms
 * control_dt_s while the loop runs at ~1 kHz, i.e. it reads about a fifth of
 * the real rate.  Falling back to it only weakens the cushion; it never
 * strengthens it or changes its sign. */
static float landing_cushion_length_rate(chassis_side_e side)
{
    if (side >= CHASSIS_SIDE_COUNT) {
        return 0.0f;
    }
    if (landing_buffer_jump_origin || landing_postbuffer_active) {
        return jump_landing_length_rate_snapshot_valid &&
               isfinite(jump_landing_length_rate_snapshot[side]) ?
            jump_landing_length_rate_snapshot[side] : 0.0f;
    }
    if (jump_rate_snapshot_usable && isfinite(jump_length_rate_snapshot[side])) {
        return jump_length_rate_snapshot[side];
    }
    return isfinite(leg[side]->d_l0_lpf) ? leg[side]->d_l0_lpf : 0.0f;
}

/* Symmetric motor cushion for the touchdown compression.  Applied on the axial
 * support channel after the raw-support capture (so the ground detector keeps
 * seeing the unmodified candidate) and before the final VMC mapping. */
void chassis_safety_module_landing_cushion_apply(
    float dt_s, float *left_force, float *right_force)
{
    if (left_force == NULL || right_force == NULL) {
        return;
    }
    if (chassis_safety_module_bump_pose_owner_active()) {
        return;
    }
    if (safety_context == NULL) {
        jump_landing_cushion_reset(&landing_cushion);
        return;
    }

    const uint8_t safe_state =
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER &&
        chassis_cmd.ctrl_mode != CHASSIS_RELAX &&
        chassis_cmd.ctrl_mode != CHASSIS_INIT &&
        chassis_cmd.ctrl_mode != CHASSIS_RECOVERY;
    const uint8_t jump_origin_ok =
        !LANDING_CUSHION_JUMP_ORIGIN_ONLY || landing_buffer_jump_origin;
    const uint8_t arm_request =
        landing_contact_confirmed && !chassis_off_ground && safe_state &&
        jump_origin_ok;
    const uint8_t keep_alive = !chassis_off_ground && safe_state;

    float target = 0.0f;
#if LANDING_CUSHION_ENABLE
    if (jump_landing_cushion_window_step(
            &landing_cushion, dt_s, arm_request, keep_alive,
            LANDING_CUSHION_WINDOW_S)) {
        /* v19 (jump102): no one-shot.  Every compression episode gets the
         * cushion; the fast release rate is what stops it pushing into a
         * rebound, so a bounce cannot be pumped and the deeper second collapse
         * is still braked. */
        target = jump_landing_cushion_target(
            landing_cushion_length_rate(LEFT),
            landing_cushion_length_rate(RIGHT),
            LANDING_CUSHION_KD, LANDING_CUSHION_FORCE_MAX,
            LANDING_CUSHION_RATE_DEADBAND);
    }
#else
    (void)arm_request;
    (void)keep_alive;
    jump_landing_cushion_reset(&landing_cushion);
#endif
    const uint8_t jump_origin_rate_valid =
        !(landing_buffer_jump_origin || landing_postbuffer_active) ||
        jump_landing_length_rate_snapshot_valid;
    if (!jump_origin_rate_valid ||
        ((landing_buffer_jump_origin || landing_postbuffer_active) &&
         target <= 0.0f)) {
        /* No stale derivative and no residual cushion after common compression
         * ends: extension/rebound must receive zero axial cushion immediately. */
        landing_cushion.applied_force = 0.0f;
    } else {
        landing_cushion.applied_force = jump_landing_cushion_slew(
            landing_cushion.applied_force, target, dt_s,
            LANDING_CUSHION_RISE_RATE, LANDING_CUSHION_FALL_RATE);
    }

    float cushion = landing_cushion.applied_force;
    if (!isfinite(cushion) || cushion <= 0.0f) {
        return;
    }
    /* One common-mode force for both legs: no split, no roll moment.  It only
     * fills the headroom left by the landing length loop up to the total limit,
     * so "loop + cushion" can never pin the 200 N FORCE_LIMIT the way v17 did. */
    const float loop_force = fmaxf(*left_force, *right_force);
    float headroom = LANDING_CUSHION_TOTAL_LIMIT - loop_force;
    LIMIT_MIN_MAX(headroom, 0.0f, LANDING_CUSHION_FORCE_MAX);
    if (cushion > headroom) {
        cushion = headroom;
    }
    if (cushion <= 0.0f) {
        return;
    }
    *left_force += cushion;
    *right_force += cushion;
    LIMIT_MIN_MAX(*left_force, -FORCE_LIMIT, FORCE_LIMIT);
    LIMIT_MIN_MAX(*right_force, -FORCE_LIMIT, FORCE_LIMIT);
}

static void offground_measured_force_update(
    const float support_command[CHASSIS_SIDE_COUNT])
{
    float candidate[CHASSIS_SIDE_COUNT];
    offground_ins_feedback_fresh = offground_ins_sample_fresh();
    if (offground_legacy_scope_active || !offground_position_view.valid ||
        !offground_ins_feedback_fresh) {
        offground_measured_filter_reset();
        offground_confirmation_clear_candidates();
        return;
    }

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; ++side) {
        const float theta = leg[side]->theta;
        const float sin_theta = sinf(theta);
        const float cos_theta = cosf(theta);
        const float l0 = fmaxf(leg[side]->l0, OFFGROUND_L0_MIN);
        const float d_l0 = offground_position_view.length_rate[side];
        const float d_theta = offground_position_view.theta_rate[side];
        const float dd_l0 = offground_position_view.length_accel[side];
        const float dd_theta = offground_position_view.theta_accel[side];
        const float leg_vertical_accel =
            ins.motion_accel_b[2] - dd_l0 * cos_theta +
            2.0f * d_l0 * d_theta * sin_theta +
            l0 * dd_theta * sin_theta +
            l0 * d_theta * d_theta * cos_theta;
        candidate[side] = support_command[side] * cos_theta +
            leg[side]->F_Vertical + leg[side]->Tp * sin_theta / l0 +
            OFFGROUND_UNSPRUNG_MASS * leg_vertical_accel;
        if (!isfinite(theta) || !isfinite(support_command[side]) ||
            !isfinite(leg[side]->F_Vertical) ||
            !isfinite(leg[side]->Tp) || !isfinite(leg_vertical_accel) ||
            !isfinite(candidate[side])) {
            offground_measured_filter_reset();
            offground_confirmation_clear_candidates();
            return;
        }
    }

    if (!offground_measured_filter_initialized) {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; ++side) {
            offground_fn_measured_raw[side] = candidate[side];
            offground_fn_measured_filtered[side] = candidate[side];
            for (uint8_t tap = 0U; tap < OFFGROUND_FILTER_LENGTH; ++tap) {
                offground_fn_measured_history[side][tap] = candidate[side];
            }
        }
        offground_measured_filter_initialized = 1U;
    } else {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; ++side) {
            offground_fn_measured_raw[side] = candidate[side];
            offground_fn_measured_history[side][offground_measured_filter_index] =
                candidate[side];
        }
        offground_measured_filter_index = (uint8_t)(
            (offground_measured_filter_index + 1U) % OFFGROUND_FILTER_LENGTH);
    }

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; ++side) {
        float sum = 0.0f;
        for (uint8_t tap = 0U; tap < OFFGROUND_FILTER_LENGTH; ++tap) {
            sum += offground_fn_measured_history[side][tap];
        }
        offground_fn_measured_filtered[side] =
            sum / (float)OFFGROUND_FILTER_LENGTH;
        if (!isfinite(offground_fn_measured_filtered[side])) {
            offground_measured_filter_reset();
            offground_confirmation_clear_candidates();
            return;
        }
    }
    offground_measured_fn_valid = 1U;
}

/* Invert the same front/back VMC Jacobian used by the motor output, using
 * actual CAN torque feedback. A grounded candidate computed earlier in this
 * tick is discarded by air control and cannot establish airborne contact. */
static uint8_t jump_feedback_support(chassis_side_e side, float *force, float *tp)
{
    const unsigned front_i = side == LEFT ? 3U : 2U;
    const unsigned back_i = side == LEFT ? 0U : 1U;
    dm_motor_object_t *front_motor = safety_context->dm_motor[front_i];
    dm_motor_object_t *back_motor = safety_context->dm_motor[back_i];
    const uint32_t now = HAL_GetTick();
    if (front_motor == NULL || back_motor == NULL ||
        !front_motor->measure.rx_count || !back_motor->measure.rx_count ||
        (uint32_t)(now - front_motor->measure.last_rx_ms) > 12U ||
        (uint32_t)(now - back_motor->measure.last_rx_ms) > 12U) {
        return 0U;
    }
    const float l0 = leg[side]->l0;
    const float s32 = sinf(leg[side]->phi3 - leg[side]->phi2);
    if (!isfinite(l0) || l0 < OFFGROUND_L0_MIN ||
        !isfinite(s32) || fabsf(s32) < 0.00001f) return 0U;
    const float s12 = sinf(leg[side]->phi1 - leg[side]->phi2);
    const float s34 = sinf(leg[side]->phi3 - leg[side]->phi4);
    const float a = leg[side]->l1 * sinf(leg[side]->phi0 - leg[side]->phi3) * s12 / s32;
    const float b = leg[side]->l1 * cosf(leg[side]->phi0 - leg[side]->phi3) * s12 / (l0 * s32);
    const float c = leg[side]->l4 * sinf(leg[side]->phi0 - leg[side]->phi2) * s34 / s32;
    const float d = leg[side]->l4 * cosf(leg[side]->phi0 - leg[side]->phi2) * s34 / (l0 * s32);
    const float determinant = a * d - b * c;
    const float sign = side == LEFT ? -1.0f : 1.0f;
    const float front = sign * front_motor->measure.torque;
    const float back = sign * back_motor->measure.torque;
    if (!isfinite(determinant) || fabsf(determinant) < 0.00001f ||
        !isfinite(front) || !isfinite(back)) return 0U;
    *force = (front * d - back * b) / determinant;
    *tp = (a * back - c * front) / determinant;
    return (uint8_t)(isfinite(*force) && isfinite(*tp));
}

void chassis_safety_module_capture_raw_support(float left_force, float right_force)
{
    if (safety_context == NULL) {
        return;
    }

    const float support_command[CHASSIS_SIDE_COUNT] = {
        left_force, right_force
    };
    const float capture_ms = dwt_get_time_ms();
    float capture_period_s = 0.0f;
    float dt = chassis_dt;
    offground_debug_snapshot.capture_valid = 0U;
    if (offground_debug_capture_clock_valid) {
        const float capture_delta_ms = capture_ms -
                                       offground_debug_last_capture_ms;
        if (isfinite(capture_delta_ms) && capture_delta_ms >= 0.0f) {
            capture_period_s = capture_delta_ms * 0.001f;
        }
    }
    offground_debug_last_capture_ms = capture_ms;
    offground_debug_capture_clock_valid = 1U;
    if (!isfinite(dt) || dt < OFFGROUND_DT_MIN || dt > OFFGROUND_DT_MAX) {
        dt = 0.001f;
    }
    offground_debug_snapshot.capture_period_s = capture_period_s;
    offground_debug_snapshot.reconstruction_dt_s = dt;
    offground_debug_snapshot.pitch = ins.pitch;
    offground_debug_snapshot.roll = ins.roll;
    offground_debug_snapshot.body_z_accel = ins.motion_accel_b[2];
    offground_debug_snapshot.mode = (uint8_t)chassis_cmd.ctrl_mode;
    offground_debug_snapshot.leg_level = (uint8_t)chassis_cmd.leg_level;
    offground_debug_snapshot.stand_state =
        (uint8_t)chassis_fdb_data.stand_state;

    dbg_raw_support_L = left_force;
    dbg_raw_support_R = right_force;
    dbg_raw_fn_avg = 0.5f * (left_force + right_force);

    jump_support_feedback_valid = (uint8_t)(chassis_cmd.ctrl_mode == CHASSIS_JUMP && chassis_off_ground);

    /* Reconstruct each wheel's vertical contact force from the commanded VMC
     * axial force, gas-spring support, leg torque and leg inertial terms.  This
     * is the same model structure used by SENTRY_LEG; filtered derivatives are
     * used here because they are already available in this VMC implementation. */
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        const float theta = leg[side]->theta;
        const float sin_theta = sinf(theta);
        const float cos_theta = cosf(theta);
        const float l0 = fmaxf(leg[side]->l0, OFFGROUND_L0_MIN);
        const float d_l0 = leg[side]->d_l0_lpf;
        const float d_theta = leg[side]->d_theta_lpf;
        float dd_l0 = 0.0f;
        float dd_theta = 0.0f;

        if (offground_derivative_initialized) {
            dd_l0 = (d_l0 - offground_last_d_l0[side]) / dt;
            dd_theta =
                (d_theta - offground_last_d_theta[side]) / dt;
        }
        offground_last_d_l0[side] = d_l0;
        offground_last_d_theta[side] = d_theta;

        const float leg_vertical_accel =
            ins.motion_accel_b[2] - dd_l0 * cos_theta +
            2.0f * d_l0 * d_theta * sin_theta +
            l0 * dd_theta * sin_theta +
            l0 * d_theta * d_theta * cos_theta;
        offground_debug_snapshot.support_candidate[side] =
            support_command[side];
        offground_debug_snapshot.actual_length[side] = leg[side]->l0;
        offground_debug_snapshot.theta[side] = theta;
        offground_debug_snapshot.inertial_force[side] =
            OFFGROUND_UNSPRUNG_MASS * leg_vertical_accel;

        /* 2026-09-23 (jump11.csv forensics): the gas spring and the virtual hip
         * torque are INTERNAL leg forces, so they must not appear in the wheel's
         * contact reconstruction once the leg is no longer being driven into the
         * ground.  This used to key off `jump_air_profile_prestarted`, which
         * `landing_air_deploy_begin()` clears the instant takeoff is confirmed:
         * the spring (~95 N at 0.33 m) then came straight back into the estimate
         * and re-armed the ground verdict, which is exactly how the measured
         * pre-tuck reading of ~100 N mean survived the old 45 N gate for the
         * whole flight.  Key it off the control chain's "thrust released" state
         * instead, which stays true from the end of the boost window until the
         * jump leaves THRUST. */
        const uint8_t jump_unloaded =
            (chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
             (jump_air_profile_prestarted || jump_legs_unloaded));
        const float tp_vertical_term = jump_unloaded ?
            0.0f : (leg[side]->Tp * sin_theta / l0);
        const float spring_term = jump_unloaded ?
            0.0f : leg[side]->F_Vertical;
        float applied_force = support_command[side];
        float applied_spring = spring_term;
        float applied_tp_term = tp_vertical_term;
        if (chassis_cmd.ctrl_mode == CHASSIS_JUMP && chassis_off_ground) {
            float measured_tp = 0.0f;
            if (jump_feedback_support(side, &applied_force, &measured_tp)) {
                applied_spring = leg[side]->F_Vertical;
                applied_tp_term = measured_tp * sin_theta / l0;
            } else {
                jump_support_feedback_valid = 0U;
            }
        }
        const float reconstructed_force =
            applied_force * cos_theta +
            applied_spring +
            applied_tp_term +
            OFFGROUND_UNSPRUNG_MASS * leg_vertical_accel;

        offground_debug_snapshot.reconstructed_finite[side] =
            isfinite(reconstructed_force) ? 1U : 0U;
        offground_debug_snapshot.reconstructed_fallback[side] =
            isfinite(reconstructed_force) ? 0U : 1U;
        offground_fn_raw[side] = isfinite(reconstructed_force) ?
            reconstructed_force : (0.5f * CHASSIS_SUPPORT_MASS * g);
        offground_debug_snapshot.reconstructed_raw[side] =
            offground_fn_raw[side];
    }
    offground_derivative_initialized = 1U;

    if (!offground_filter_initialized) {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            for (uint8_t tap = 0U; tap < OFFGROUND_FILTER_LENGTH; tap++) {
                offground_fn_history[side][tap] = offground_fn_raw[side];
            }
        }
        offground_filter_initialized = 1U;
    } else {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            offground_fn_history[side][offground_filter_index] =
                offground_fn_raw[side];
        }
        offground_filter_index =
            (uint8_t)((offground_filter_index + 1U) % OFFGROUND_FILTER_LENGTH);
    }

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        float sum = 0.0f;
        for (uint8_t tap = 0U; tap < OFFGROUND_FILTER_LENGTH; tap++) {
            sum += offground_fn_history[side][tap];
        }
        offground_fn_filtered[side] = sum / (float)OFFGROUND_FILTER_LENGTH;
        offground_debug_snapshot.filtered_force[side] =
            offground_fn_filtered[side];
    }
    const float support_candidate[CHASSIS_SIDE_COUNT] = {
        left_force, right_force
    };
    offground_measured_force_update(support_candidate);
    FN_Average = 0.5f *
        (offground_fn_filtered[LEFT] + offground_fn_filtered[RIGHT]);
    dbg_ground_detect_fn_avg = FN_Average;
    offground_debug_snapshot.capture_valid = 1U;
}

void chassis_safety_module_freeze_integrators_if_airborne(void)
{
    if (chassis_safety_module_bump_pose_owner_active()) {
        return;
    }
    if (safety_context != NULL && chassis_off_ground) {
        L_length_pid->Iout = 0.0f;
        R_length_pid->Iout = 0.0f;
        theta_pid->Iout = 0.0f;
        roll_pid->Iout = 0.0f;
    } else if (safety_context != NULL && jump_air_profile_prestarted) {
        /* The axial pre-tuck bypasses the grounded length PID, but Pitch/Roll
         * and theta synchronization are deliberately still ground-owned. */
        L_length_pid->Iout = 0.0f;
        R_length_pid->Iout = 0.0f;
    }
}

void chassis_safety_module_update_ground(void)
{
    uint8_t off_now;
    uint8_t airborne_allowed;
    float landing_dt;

    if (safety_context == NULL) {
        return;
    }

    offground_debug_impact_clear();
    offground_debug_snapshot.runtime_valid = 0U;
    landing_dt = landing_real_dt();
    landing_airborne_guard_step(landing_dt);

    Ground_Detect();
    /* Latch the source before evaluating airborne_allowed.  The filtered
     * takeoff edge can arrive one cycle before the jump state publishes
     * CHASSIS_IS_JUMPING; without this early latch that first cycle would
     * clear the detector and lose the jump owner. */
    if (Off_Ground_Flag && !chassis_off_ground_last &&
        chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        landing_jump_origin_pending = 1U;
    }
    airborne_allowed = chassis_airborne_allowed();
    if (!airborne_allowed) {
        airborne_state_clear();
        offground_debug_capture_runtime(airborne_allowed);
        return;
    }

    off_now = Off_Ground_Flag;
    chassis_takeoff_edge = off_now && !chassis_off_ground_last;
    chassis_landing_edge = !off_now && chassis_off_ground_last;
    if (chassis_takeoff_edge) {
        offground_debug_count_increment(&offground_debug_takeoff_events);
        offground_debug_last_takeoff_source_valid = 1U;
        offground_debug_last_takeoff_forced = 0U;
        if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
            landing_jump_origin_pending = 1U;
        }
    }
    if (chassis_landing_edge) {
        offground_debug_count_increment(&offground_debug_landing_events);
    }
    chassis_off_ground_last = off_now;
    chassis_off_ground = off_now;
    if (chassis_off_ground) {
        chassis_safety_module_landing_fast_exit_cancel_if_unsafe();
    }

    /* BUMP's leg-off-ground signal can precede whole-body flight by seconds.
     * Discard only speculative contact buffering during the action; the
     * confirmed landing buffer remains owned by the shared safety path. */
    if (bump_precontact_buffer_stale(
            chassis_safety_module_bump_pose_owner_active(),
            landing_buffer_phase != LANDING_BUFFER_INACTIVE,
            landing_contact_confirmed)) {
        const uint8_t jump_origin_pending = landing_jump_origin_pending;
        landing_buffer_reset();
        landing_jump_origin_pending = jump_origin_pending;
    }

    const uint8_t bump_airborne_rising =
        bump_true_airborne_update(landing_dt);
    if (bump_airborne_rising) {
        /* A generic off-ground edge may have started the profile while only a
         * leg was unloaded. Restart from measured geometry on true flight,
         * unless a confirmed landing/rebound buffer already owns its refs. */
        if (landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
            !landing_contact_confirmed) {
            const uint8_t jump_origin_pending = landing_jump_origin_pending;
            landing_buffer_reset();
            landing_jump_origin_pending = jump_origin_pending;
        }
        if (!landing_contact_confirmed) {
            landing_air_deploy_begin();
        }
        landing_impact_armed = 0U;
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
    }

    if (chassis_takeoff_edge &&
        landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
        landing_contact_confirmed && landing_buffer_jump_origin &&
        (!bump_action_active || bump_airborne_lockout)) {
        /* A confirmed jump landing that immediately loses contact is a
         * rebound, not stable ground.  Keeping the grounded buffer alive here
         * leaves apply_airborne_control on its soft-force fallback while the
         * legs are already extending (jump2: 0.13 m -> 0.28 m in 90 ms).
         * Re-arm the measured air profile so the reaction-wheel and axial
         * flight paths regain authority for this short second flight. */
        landing_buffer_reset();
        landing_postbuffer_reset();
        landing_air_deploy_begin();
        landing_impact_armed = 0U;
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
        pid_clear(theta_pid);
        pid_clear(roll_pid);
        pid_clear(L_length_pid);
        pid_clear(R_length_pid);
    } else if (chassis_takeoff_edge &&
               landing_buffer_phase == LANDING_BUFFER_INACTIVE &&
               (!bump_action_active || bump_airborne_lockout)) {
        /* A first takeoff starts the airborne deployment. */
        landing_air_deploy_begin();
        landing_impact_armed = 0U;
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
        pid_clear(theta_pid);
        pid_clear(roll_pid);
        pid_clear(L_length_pid);
        pid_clear(R_length_pid);
    }

    if (chassis_landing_edge) {
        const uint8_t buffer_was_active =
            landing_buffer_phase != LANDING_BUFFER_INACTIVE;
        landing_yaw_hold_timer = LANDING_YAW_HOLD_TIME;
        if (buffer_was_active) {
            landing_contact_confirmed = 1U;
            landing_precontact_elapsed = 0.0f;
            if (landing_jump_origin_active()) {
                landing_buffer_jump_origin = 1U;
                landing_airborne_guard_arm();
            }
            landing_jump_origin_pending = 0U;
        } else {
            landing_buffer_begin(1U);
        }
        landing_impact_armed = 0U;
        landing_impact_count = 0U;
        landing_impact_rearm_timer = 0.0f;
        Wheel_Shut_Flag = 0U;
        yaw_target = ins.yaw_total_angle * DEGREE_2_RAD;
        lqr_virtual_x_l = 0.0f;
        lqr_virtual_x_r = 0.0f;
        pid_clear(theta_pid);
        pid_clear(roll_pid);
        if (!buffer_was_active) {
            pid_clear(L_length_pid);
            pid_clear(R_length_pid);
        }
    } else if (chassis_off_ground) {
        /* RETURN exit requires uninterrupted stable ground contact. The
         * airborne path skips landing_buffer_update(), so clear its timer here
         * rather than carrying pre-rebound time across the gap. */
        landing_exit_stable_elapsed =
            jump_landing_exit_stable_elapsed_step(
                landing_exit_stable_elapsed, landing_dt, 0U);
        if (!bump_action_active || bump_airborne_lockout) {
            landing_air_deploy_update(landing_dt);
            landing_impact_update(landing_dt);
        }
    } else if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        landing_buffer_update(landing_dt);
    }
    if (!chassis_off_ground && !chassis_landing_edge &&
        landing_buffer_phase == LANDING_BUFFER_INACTIVE &&
        chassis_cmd.ctrl_mode != CHASSIS_JUMP) {
        landing_jump_origin_pending = 0U;
    }
    landing_postbuffer_update(landing_dt);
    offground_debug_capture_runtime(airborne_allowed);
}

uint8_t chassis_safety_module_apply_airborne_control(void)
{
    if (safety_context == NULL) {
        return 0;
    }
    if (chassis_safety_module_bump_pose_owner_active()) {
        dbg_offground_lqr_tp_L = 0.0f;
        dbg_offground_lqr_tp_R = 0.0f;
        dbg_offground_theta_hold_tp_L = 0.0f;
        dbg_offground_theta_hold_tp_R = 0.0f;
        dbg_offground_theta_hold_tp_common_raw = 0.0f;
        landing_air_length_sync_force = 0.0f;
        return 0U;
    }

    const float buffered_support_l = leg[LEFT]->support_force;
    const float buffered_support_r = leg[RIGHT]->support_force;

    if (!chassis_off_ground) {
#if OFFGROUND_TP_POLARITY_TEST_ENABLE
        Offground_TpPolarityTest_Reset();
#endif
        dbg_offground_lqr_tp_L = 0.0f;
        dbg_offground_lqr_tp_R = 0.0f;
        dbg_offground_theta_hold_tp_L = 0.0f;
        dbg_offground_theta_hold_tp_R = 0.0f;
        dbg_offground_theta_hold_tp_common_raw = 0.0f;
        landing_air_length_sync_force = 0.0f;

        if (jump_air_profile_prestarted &&
            chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
            /* The timed pre-tuck owns only the axial channel.  jump36 showed
             * that treating this interval as fully airborne also shut the
             * wheels and replaced the grounded hip controller; Pitch reached
             * -15 deg before takeoff was confirmed.  Keep both grounded
             * attitude channels and replace only the two axial commands. */
            float air_force_l = JUMP_AIR_DEPLOY_KP *
                    (landing_air_deploy_ref[LEFT] - leg[LEFT]->l0) -
                JUMP_AIR_DEPLOY_KD * jump_length_rate(LEFT) -
                leg[LEFT]->F_Vertical;
            float air_force_r = JUMP_AIR_DEPLOY_KP *
                    (landing_air_deploy_ref[RIGHT] - leg[RIGHT]->l0) -
                JUMP_AIR_DEPLOY_KD * jump_length_rate(RIGHT) -
                leg[RIGHT]->F_Vertical;

            float common_force = 0.5f * (air_force_l + air_force_r);
            const float differential_force =
                0.5f * (air_force_l - air_force_r);
            LIMIT_MIN_MAX(common_force,
                          -JUMP_FLIGHT_RETRACT_PEAK,
                           OFFGROUND_SPRING_COMP_LIMIT);
            air_force_l = common_force + differential_force;
            air_force_r = common_force - differential_force;

            landing_air_length_sync_force =
                JUMP_AIR_LENGTH_SYNC_KP *
                    (leg[RIGHT]->l0 - leg[LEFT]->l0) +
                JUMP_AIR_LENGTH_SYNC_KD *
                    (jump_length_rate(RIGHT) - jump_length_rate(LEFT));
            LIMIT_MIN_MAX(landing_air_length_sync_force,
                          -JUMP_AIR_LENGTH_SYNC_LIMIT,
                           JUMP_AIR_LENGTH_SYNC_LIMIT);
            float air_roll = JUMP_FLIGHT_ROLL_KP * ins.roll -
                             JUMP_FLIGHT_ROLL_KD * ins.gyro[1];
            LIMIT_MIN_MAX(air_roll, -JUMP_FLIGHT_ROLL_LIMIT, JUMP_FLIGHT_ROLL_LIMIT);
            air_force_l -= air_roll;
            air_force_r += air_roll;
            air_force_l += landing_air_length_sync_force;
            air_force_r -= landing_air_length_sync_force;
            LIMIT_MIN_MAX(air_force_l,
                          -JUMP_FLIGHT_RETRACT_PEAK,
                           OFFGROUND_SPRING_COMP_LIMIT);
            LIMIT_MIN_MAX(air_force_r,
                          -JUMP_FLIGHT_RETRACT_PEAK,
                           OFFGROUND_SPRING_COMP_LIMIT);
            leg[LEFT]->support_force = air_force_l;
            leg[RIGHT]->support_force = air_force_r;
        }

        /* Hand axial-force authority from the proven 20 N airborne command to
         * the landing length controller over its own short support window. */
        if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
            float blend = landing_buffer_elapsed / LANDING_SUPPORT_BLEND_TIME;
            LIMIT_MIN_MAX(blend, 0.0f, 1.0f);
            float air_force_l =
                OFFGROUND_SOFT_FORCE - leg[LEFT]->F_Vertical;
            float air_force_r =
                OFFGROUND_SOFT_FORCE - leg[RIGHT]->F_Vertical;
            LIMIT_MIN_MAX(air_force_l, -OFFGROUND_SPRING_COMP_LIMIT,
                          OFFGROUND_SPRING_COMP_LIMIT);
            LIMIT_MIN_MAX(air_force_r, -OFFGROUND_SPRING_COMP_LIMIT,
                          OFFGROUND_SPRING_COMP_LIMIT);
            leg[LEFT]->support_force = air_force_l + blend *
                (leg[LEFT]->support_force - air_force_l);
            leg[RIGHT]->support_force = air_force_r + blend *
                (leg[RIGHT]->support_force - air_force_r);
        }
        return 0;
    }

    const uint8_t landing_buffered_contact =
        landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
        landing_contact_confirmed;
    if (!landing_buffered_contact) {
        Wheel_Shut_Flag = 1;
        LQROutBuf[LEFT][0] = 0.0f;
        LQROutBuf[RIGHT][0] = 0.0f;
    }

    const uint8_t active_jump =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP ||
        chassis_safety_module_landing_airborne_guard_active();
    /* 仅在缓冲控制中复用跳跃参数；BUMP 不进入 jump_state。
     * active_jump 仍单独用于跳跃专属通道（theta 参考与运动学速率观测），
     * 使 BUMP 走 611 的 theta 目标 0 与 d_theta_lpf 路径。 */
    const uint8_t active_air_buffer = active_jump || bump_active;
    const float theta_sync_limit = active_air_buffer ?
        JUMP_AIR_THETA_SYNC_LIMIT : OFFGROUND_SYNC_LIMIT;
    float sync_output = theta_pid->Output;
    LIMIT_MIN_MAX(sync_output, -theta_sync_limit, theta_sync_limit);

    const float lqr_tp_gain = active_air_buffer ?
        JUMP_AIR_LQR_TP_GAIN : OFFGROUND_LQR_TP_GAIN;
    dbg_offground_lqr_tp_L = lqr_tp_gain * (-LQROutBuf[LEFT][1]);
    dbg_offground_lqr_tp_R = lqr_tp_gain * (-LQROutBuf[RIGHT][1]);
    const float theta_hold_kp = active_air_buffer ?
        JUMP_AIR_THETA_HOLD_KP : OFFGROUND_THETA_HOLD_KP;
    const float theta_hold_kd = active_air_buffer ?
        JUMP_AIR_THETA_HOLD_KD : OFFGROUND_THETA_HOLD_KD;
    const float theta_hold_limit = active_air_buffer ?
        JUMP_AIR_THETA_HOLD_TP_LIMIT : OFFGROUND_THETA_HOLD_TP_LIMIT;
    /* 2026-09-23: hold the leg angle against an explicit reference instead of
     * zero.  The previous hard-coded 0.0 target meant the airborne joint channel
     * was a spring pulling the rods to vertical, which fought the landing
     * geometry and left the body reaction uncontrolled.  The reference is
     * captured at takeoff and driven to JUMP_AIR_THETA_REF_RAD by the control
     * chain (see chassis_safety_module_set_jump_air_theta_ref).
     *
     * SIGN -- POSITIVE.  Settled two independent ways on 2026-09-23.
     *
     * (a) FIRMWARE GEOMETRY (no simulation needed).  chassis_task.c:96-104
     *     passes motor_distance = 0.0 and drv_leg.c:42-43 forces l4 = l1,
     *     l3 = l2, so the linkage degenerates to phi0 == (phi1 + phi4)/2
     *     exactly.  The Jacobian entries are therefore q01 = q11 = +0.5 in every
     *     pose, and drv_leg.c:516-517 gives Tmotor[0] = q00*F + 0.5*Tp and
     *     Tmotor[1] = q10*F + 0.5*Tp.  So +Tp increases phi0, which DECREASES
     *     theta = pi/2 - phi0 + pitch (drv_leg.c:71).  Negative feedback
     *     therefore requires Tp = +(kp*err + kd*dtheta) -- the positive sign.
     *     NOTE: an earlier version of this comment claimed the opposite
     *     ("a positive leg-angle torque drives theta up").  That premise was
     *     factually inverted and is exactly what introduced the wrap bug.
     *
     * (b) 1:1 SIMULATION (/home/kaiser/robomaster/rl-lab), 2x2 experiment:
     *       theta +80/+12, wheel ON  -> full jump, max|theta| 0.123, no fault
     *       theta -80/-12, wheel ON  -> max|theta| 5.519, pitch 152, SAFETY
     *       theta +80/+12, wheel OFF -> full jump, max|theta| 0.123, no fault
     *       theta -80/-12, wheel OFF -> max|theta| 5.553, pitch 159, SAFETY
     *     The negated cases reproduce the real robot exactly: theta reaching
     *     ~5.5 rad (a full turn = the wrap) and the body past 150 deg -- the
     *     "legs spin a full turn in the air" failure seen in jump18/19/20.
     *
     * Do not re-invert this.  If you doubt it, re-run either experiment above.
     *
     * The error clamp is kept: with Kp 80 a 0.20 rad band already reaches ~16 Nm,
     * and the sim's own max error is 0.118 rad, so the clamp never bites in
     * normal operation but does bound the demand if theta ever does wrap. */
    const float theta_ref = active_jump ? jump_air_theta_ref : 0.0f;
    float theta_err_L = leg[LEFT]->theta - theta_ref;
    float theta_err_R = leg[RIGHT]->theta - theta_ref;
    LIMIT_MIN_MAX(theta_err_L, -JUMP_THETA_ERR_CLAMP, JUMP_THETA_ERR_CLAMP);
    LIMIT_MIN_MAX(theta_err_R, -JUMP_THETA_ERR_CLAMP, JUMP_THETA_ERR_CLAMP);
    const jump_local_rate_selection_t theta_rate_selection_l =
        jump_local_rate_select(
            JUMP_LOCAL_RATE_FEEDBACK_ENABLE,
            JUMP_KINEMATIC_RATE_OBSERVER_ENABLE,
            active_jump, jump_rate_snapshot_usable,
            jump_theta_rate_snapshot[LEFT], leg[LEFT]->d_theta_lpf);
    const jump_local_rate_selection_t theta_rate_selection_r =
        jump_local_rate_select(
            JUMP_LOCAL_RATE_FEEDBACK_ENABLE,
            JUMP_KINEMATIC_RATE_OBSERVER_ENABLE,
            active_jump, jump_rate_snapshot_usable,
            jump_theta_rate_snapshot[RIGHT], leg[RIGHT]->d_theta_lpf);
    const float theta_rate_l = theta_rate_selection_l.valid ?
        theta_rate_selection_l.rate : 0.0f;
    const float theta_rate_r = theta_rate_selection_r.valid ?
        theta_rate_selection_r.rate : 0.0f;
    dbg_offground_theta_hold_tp_L =
        theta_hold_kp * theta_err_L +
        theta_hold_kd * theta_rate_l;
    dbg_offground_theta_hold_tp_R =
        theta_hold_kp * theta_err_R +
        theta_hold_kd * theta_rate_r;
    LIMIT_MIN_MAX(dbg_offground_theta_hold_tp_L,
                  -theta_hold_limit, theta_hold_limit);
    LIMIT_MIN_MAX(dbg_offground_theta_hold_tp_R,
                  -theta_hold_limit, theta_hold_limit);
    dbg_offground_theta_hold_tp_common_raw = 0.5f *
        (dbg_offground_theta_hold_tp_L +
         dbg_offground_theta_hold_tp_R);

    if (active_air_buffer) {
        /* One common hip axis cannot independently control body Pitch and the
         * absolute direction of both leg rods.  jump33 showed the airborne
         * common Pitch/LQR term repeatedly opposing theta hold and carrying
         * the rods farther rearward.  Honour the landing-geometry invariant:
         * allocate the common axis only to vertical-leg hold in jump flight,
         * while retaining the LQR differential part for left/right symmetry. */
        const float lqr_common = 0.5f *
            (dbg_offground_lqr_tp_L + dbg_offground_lqr_tp_R);
        const float lqr_differential = 0.5f *
            (dbg_offground_lqr_tp_L - dbg_offground_lqr_tp_R);
        (void)lqr_common;
        dbg_offground_lqr_tp_L =
            lqr_differential;
        dbg_offground_lqr_tp_R =
            -lqr_differential;
    }

    float air_Tp_L = dbg_offground_lqr_tp_L - sync_output +
                     dbg_offground_theta_hold_tp_L;
    float air_Tp_R = dbg_offground_lqr_tp_R + sync_output +
                     dbg_offground_theta_hold_tp_R;
#if OFFGROUND_TP_POLARITY_TEST_ENABLE
    Offground_TpPolarityTest_Apply(chassis_dt, &air_Tp_L, &air_Tp_R);
    dbg_offground_lqr_tp_L = 0.0f;
    dbg_offground_lqr_tp_R = 0.0f;
    dbg_offground_theta_hold_tp_L = 0.0f;
    dbg_offground_theta_hold_tp_R = 0.0f;
#endif
    const float air_tp_limit = active_air_buffer ?
        JUMP_AIR_TP_LIMIT : OFFGROUND_LQR_TP_LIMIT;
    if (active_air_buffer) {
        /* Allocate the bounded common command first, then spend only the
         * remaining headroom on left/right differential correction.  The
         * common command is now primarily the leg-vertical hold once the leg
         * rods leave their small Pitch-assist window. */
        float common_tp = 0.5f * (air_Tp_L + air_Tp_R);
        float differential_tp = 0.5f * (air_Tp_L - air_Tp_R);
        LIMIT_MIN_MAX(common_tp, -air_tp_limit, air_tp_limit);
        const float differential_limit =
            fmaxf(air_tp_limit - fabsf(common_tp), 0.0f);
        LIMIT_MIN_MAX(differential_tp,
                      -differential_limit, differential_limit);
        air_Tp_L = common_tp + differential_tp;
        air_Tp_R = common_tp - differential_tp;
    } else {
        LIMIT_MIN_MAX(air_Tp_L, -air_tp_limit, air_tp_limit);
        LIMIT_MIN_MAX(air_Tp_R, -air_tp_limit, air_tp_limit);
    }

    float air_F_L = OFFGROUND_SOFT_FORCE - leg[LEFT]->F_Vertical;
    float air_F_R = OFFGROUND_SOFT_FORCE - leg[RIGHT]->F_Vertical;
    if ((chassis_cmd.leg_level == LEG_LOW || bump_active) &&
        landing_buffer_phase == LANDING_BUFFER_INACTIVE) {
        const float deploy_kp = active_jump && !jump_air_profile_state.deploy_latched ?
            JUMP_FLIGHT_TUCK_KP : (active_jump ? JUMP_FLIGHT_DEPLOY_KP :
            (active_air_buffer ? JUMP_AIR_DEPLOY_KP : LANDING_AIR_DEPLOY_KP));
        const float deploy_kd = active_jump ?
            (jump_air_profile_state.deploy_latched ? JUMP_FLIGHT_DEPLOY_KD : JUMP_FLIGHT_LENGTH_KD) :
            (active_air_buffer ? JUMP_AIR_DEPLOY_KD : LANDING_AIR_DEPLOY_KD);
        const float deploy_limit = active_air_buffer ?
            JUMP_AIR_DEPLOY_FORCE_LIMIT :
            LANDING_AIR_DEPLOY_FORCE_LIMIT;
        float deploy_force_l =
            deploy_kp *
                (landing_air_deploy_ref[LEFT] - leg[LEFT]->l0) -
            deploy_kd * airborne_length_rate(LEFT) -
            OFFGROUND_SOFT_FORCE;
        float deploy_force_r =
            deploy_kp *
                (landing_air_deploy_ref[RIGHT] - leg[RIGHT]->l0) -
            deploy_kd * airborne_length_rate(RIGHT) -
            OFFGROUND_SOFT_FORCE;
        LIMIT_MIN_MAX(deploy_force_l, -deploy_limit, deploy_limit);
        LIMIT_MIN_MAX(deploy_force_r, -deploy_limit, deploy_limit);
        air_F_L += deploy_force_l;
        air_F_R += deploy_force_r;
    }
#if OFFGROUND_TP_POLARITY_TEST_ENABLE && OFFGROUND_TP_TEST_NEUTRALIZE_AXIAL_FORCE
    if (offground_tp_test_phase != OFFGROUND_TP_TEST_DONE) {
        air_F_L = -leg[LEFT]->F_Vertical;
        air_F_R = -leg[RIGHT]->F_Vertical;
    }
#endif
    LIMIT_MIN_MAX(air_F_L, -OFFGROUND_SPRING_COMP_LIMIT, OFFGROUND_SPRING_COMP_LIMIT);
    LIMIT_MIN_MAX(air_F_R, -OFFGROUND_SPRING_COMP_LIMIT, OFFGROUND_SPRING_COMP_LIMIT);
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        float blend = landing_buffer_elapsed / LANDING_SUPPORT_BLEND_TIME;
        LIMIT_MIN_MAX(blend, 0.0f, 1.0f);
        air_F_L += blend * (buffered_support_l - air_F_L);
        air_F_R += blend * (buffered_support_r - air_F_R);
    }
    /* Bound the left/right length split both during nominal flight and during
     * a small landing rebound.  This channel is intentionally much weaker
     * than the thrust synchronizer; it only prevents one leg deploying while
     * the other collapses. */
    const float length_sync_kp = active_jump ? JUMP_FLIGHT_LENGTH_SYNC_KP :
        (active_air_buffer ? JUMP_AIR_LENGTH_SYNC_KP : OFFGROUND_LENGTH_SYNC_KP);
    const float length_sync_kd = active_jump ? JUMP_FLIGHT_LENGTH_SYNC_KD :
        (active_air_buffer ? JUMP_AIR_LENGTH_SYNC_KD : OFFGROUND_LENGTH_SYNC_KD);
    const float length_sync_limit = active_jump ? JUMP_FLIGHT_LENGTH_SYNC_LIMIT :
        (active_air_buffer ? JUMP_AIR_LENGTH_SYNC_LIMIT : OFFGROUND_LENGTH_SYNC_LIMIT);
    if (active_air_buffer) {
        /* Limit common retraction before adding the differential synchronizer.
         * Independent per-leg clipping previously forced both sides to the
         * same -80 N and erased the very correction needed by the slow left
         * mechanism. */
        const float spring_common = active_jump ?
            0.5f * (leg[LEFT]->F_Vertical + leg[RIGHT]->F_Vertical) : 0.0f;
        float common_force = 0.5f * (air_F_L + air_F_R) + spring_common;
        const float differential_force = 0.5f * (air_F_L - air_F_R);
        LIMIT_MIN_MAX(common_force, -JUMP_AIR_RETRACT_FORCE_LIMIT,
                      OFFGROUND_SPRING_COMP_LIMIT);
        air_F_L = common_force + differential_force - spring_common;
        air_F_R = common_force - differential_force - spring_common;
    }

    const uint8_t normal_split_guard_scope = (uint8_t)(
        !active_air_buffer &&
        chassis_cmd.leg_level != LEG_LOW &&
        landing_buffer_phase == LANDING_BUFFER_INACTIVE);
    if (normal_split_guard_scope) {
        /* MID/HIGH normal flight keeps its existing common 20 N preload.  A
         * separate guard corrects only a large split, with a bounded and
         * slewed differential command. */
        landing_air_length_sync_force =
            offground_normal_split_guard_update(
                &offground_normal_split_guard,
                leg[LEFT]->l0, leg[RIGHT]->l0,
                leg[LEFT]->d_l0_lpf, leg[RIGHT]->d_l0_lpf,
                chassis_dt);
    } else if (active_air_buffer ||
               chassis_cmd.leg_level == LEG_LOW) {
        landing_air_length_sync_force =
            length_sync_kp * (leg[RIGHT]->l0 - leg[LEFT]->l0) +
            length_sync_kd * (active_jump ?
                (jump_length_rate(RIGHT) - jump_length_rate(LEFT)) :
                (leg[RIGHT]->d_l0_lpf - leg[LEFT]->d_l0_lpf));
        LIMIT_MIN_MAX(landing_air_length_sync_force,
                      -length_sync_limit, length_sync_limit);
    } else {
        landing_air_length_sync_force = 0.0f;
        offground_normal_split_guard_reset(&offground_normal_split_guard);
    }
    if (active_jump) {
        float air_roll = JUMP_FLIGHT_ROLL_KP * ins.roll -
                         JUMP_FLIGHT_ROLL_KD * ins.gyro[1];
        LIMIT_MIN_MAX(air_roll, -JUMP_FLIGHT_ROLL_LIMIT, JUMP_FLIGHT_ROLL_LIMIT);
        air_F_L -= air_roll;
        air_F_R += air_roll;
    }
    air_F_L += landing_air_length_sync_force;
    air_F_R -= landing_air_length_sync_force;
    const float air_retract_limit = active_jump ? JUMP_FLIGHT_RETRACT_PEAK :
        (active_air_buffer ? JUMP_AIR_RETRACT_FORCE_PEAK : OFFGROUND_AIR_RETRACT_LIMIT);
    LIMIT_MIN_MAX(air_F_L, -air_retract_limit,
                  OFFGROUND_SPRING_COMP_LIMIT);
    LIMIT_MIN_MAX(air_F_R, -air_retract_limit,
                  OFFGROUND_SPRING_COMP_LIMIT);
    leg[LEFT]->support_force = air_F_L;
    leg[RIGHT]->support_force = air_F_R;
    leg[LEFT]->Tp = air_Tp_L;
    leg[RIGHT]->Tp = air_Tp_R;
    return 1;
}

void chassis_safety_module_check_danger(void)
{
    if (safety_context != NULL) {
        Security_Checking();
    }
}

uint8_t chassis_safety_module_airborne_allowed(void)
{
    return safety_context != NULL && chassis_airborne_allowed();
}

uint8_t chassis_safety_module_off_ground(void)
{
    return chassis_off_ground;
}

uint8_t chassis_safety_module_ground_contact_confirmed(void)
{
    const float landing_threshold = Offground_LandingThreshold();
    const uint8_t source_valid = offground_legacy_scope_active ?
        offground_filter_initialized :
        (offground_measured_fn_valid && offground_ins_feedback_fresh);
    return (uint8_t)(safety_context != NULL && source_valid &&
        !chassis_off_ground &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER &&
        isfinite(landing_threshold) &&
        offground_selected_fn(CHASSIS_SIDE_LEFT) > landing_threshold &&
        offground_selected_fn(CHASSIS_SIDE_RIGHT) > landing_threshold);
}

uint8_t chassis_safety_module_wheel_shut(void)
{
    return Wheel_Shut_Flag;
}

void chassis_safety_module_set_wheel_shut(uint8_t wheel_shut)
{
    Wheel_Shut_Flag = wheel_shut ? 1U : 0U;
}

void chassis_safety_module_set_bump_active(uint8_t active)
{
    bump_active = active ? 1U : 0U;
}

void chassis_safety_module_set_bump_action_active(uint8_t active)
{
    const uint8_t was_active = bump_action_active;
    bump_action_active = active ? 1U : 0U;
    if (bump_action_active && !was_active && !bump_airborne_lockout) {
        bump_low_accel_elapsed_s = 0.0f;
        bump_accel_norm_filter_initialized = 0U;
    }
}

uint8_t chassis_safety_module_bump_airborne_lockout(void)
{
    return bump_airborne_lockout;
}

uint8_t chassis_safety_module_bump_pose_owner_active(void)
{
    return (uint8_t)(safety_context != NULL && bump_action_active &&
                     !bump_airborne_lockout);
}

uint8_t chassis_safety_module_takeoff_edge(void)
{
    return chassis_takeoff_edge;
}

uint8_t chassis_safety_module_landing_edge(void)
{
    return chassis_landing_edge;
}

void chassis_safety_module_force_airborne(void)
{
    if (safety_context == NULL) {
        return;
    }
    if (!Off_Ground_Flag) {
        offground_debug_impact_clear();
        offground_debug_snapshot.detector_valid = 0U;
        offground_debug_snapshot.runtime_valid = 0U;
        offground_debug_snapshot.runtime_flags = 0U;
        offground_debug_count_increment(&offground_debug_takeoff_events);
        offground_debug_last_takeoff_source_valid = 1U;
        offground_debug_last_takeoff_forced = 1U;
        Off_Ground_Flag = 1;
        Touch_Ground_Flag = 0;
        Off_Ground_Cnt = 0;
        Touch_Ground_Cnt = 0;
        chassis_off_ground = 1U;
        chassis_takeoff_edge = 1U;
        chassis_off_ground_last = 1U;
        if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
            landing_jump_origin_pending = 1U;
        }
        if (landing_buffer_phase == LANDING_BUFFER_INACTIVE) {
            landing_air_deploy_begin();
            landing_impact_armed = 0U;
            landing_impact_count = 0U;
            landing_impact_rearm_timer = 0.0f;
            pid_clear(theta_pid);
            pid_clear(roll_pid);
            pid_clear(L_length_pid);
            pid_clear(R_length_pid);
        }
    }
}

float chassis_safety_module_landing_gain(void)
{
    if (chassis_safety_module_bump_pose_owner_active()) {
        return 1.0f;
    }
    if (safety_context == NULL ||
        landing_buffer_phase == LANDING_BUFFER_INACTIVE) {
        return 1.0f;
    }
    return landing_contact_confirmed ? 1.0f : 0.0f;
}

uint8_t chassis_safety_module_landing_buffer_active(void)
{
    return landing_buffer_phase != LANDING_BUFFER_INACTIVE;
}

uint8_t chassis_safety_module_landing_buffer_jump_origin(void)
{
    return (landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
            landing_buffer_jump_origin) ? 1U : 0U;
}

uint8_t chassis_safety_module_jump_landing_owner_active(void)
{
    const uint8_t normal_owner_mode =
        (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
         chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL) &&
        (chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
         chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);
    const uint8_t jump_handoff_mode = jump_landing_jump_stand_allowed(
        chassis_cmd.ctrl_mode == CHASSIS_JUMP,
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPING,
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);
    return (uint8_t)(safety_context != NULL &&
        jump_landing_owner_mode_allowed(
            landing_buffer_phase != LANDING_BUFFER_INACTIVE,
            landing_buffer_jump_origin,
            chassis_fdb_data.stand_state != CHASSIS_IS_DANGER,
            normal_owner_mode, jump_handoff_mode));
}

uint8_t chassis_safety_module_danger_reason(void)
{
    return (uint8_t)dbg_danger_reason;
}

uint8_t chassis_safety_module_danger_reason_latched(void)
{
    return (uint8_t)danger_reason_latched;
}

void chassis_safety_module_clear_danger_latch(void)
{
    danger_reason_latched = DANGER_NONE;
}

uint8_t chassis_safety_module_landing_buffer_phase(void)
{
    return (uint8_t)landing_buffer_phase;
}

float chassis_safety_module_air_length_sync_force(void)
{
    return landing_air_length_sync_force;
}

float chassis_safety_module_air_lqr_tp_common(void)
{
    return 0.5f *
        (dbg_offground_lqr_tp_L + dbg_offground_lqr_tp_R);
}

float chassis_safety_module_air_theta_hold_tp_common(void)
{
    return dbg_offground_theta_hold_tp_common_raw;
}

float chassis_safety_module_air_length_target(chassis_side_e side)
{
    if (side != CHASSIS_SIDE_LEFT && side != CHASSIS_SIDE_RIGHT) {
        return 0.0f;
    }
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        return landing_length_ref[side];
    }
    return landing_air_deploy_ref[side];
}

float chassis_safety_module_airborne_elapsed(void)
{
    return landing_airborne_elapsed;
}

float chassis_safety_module_offground_filtered_force(chassis_side_e side)
{
    if (side != CHASSIS_SIDE_LEFT && side != CHASSIS_SIDE_RIGHT) {
        return 0.0f;
    }
    return offground_selected_fn(side);
}

float chassis_safety_module_raw_support_candidate(chassis_side_e side)
{
    if (side != CHASSIS_SIDE_LEFT && side != CHASSIS_SIDE_RIGHT) {
        return 0.0f;
    }
    return side == CHASSIS_SIDE_LEFT ? dbg_raw_support_L : dbg_raw_support_R;
}

uint8_t chassis_safety_module_jump_legs_unloaded(void)
{
    return jump_legs_unloaded;
}

void chassis_safety_module_jump_air_profile_prestart(float elapsed_s)
{
    if (safety_context == NULL ||
        chassis_cmd.ctrl_mode != CHASSIS_JUMP ||
        chassis_off_ground ||
        landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
        return;
    }

    if (!isfinite(elapsed_s) || elapsed_s < 0.0f) {
        elapsed_s = 0.0f;
    }

    if (!jump_air_profile_prestarted) {
        landing_air_deploy_begin();
        jump_air_profile_prestarted = 1U;
    }

    /* Drive the reference from absolute release time, not loop count. This
     * keeps the release-to-takeoff handoff continuous if task timing shifts. */
    float profile_dt = elapsed_s - landing_airborne_elapsed;
    if (profile_dt > 0.0f) {
        if (profile_dt > 0.050f) {
            profile_dt = 0.050f;
        }
        landing_air_deploy_update(profile_dt);
    }
}

void chassis_safety_module_jump_air_profile_cancel(void)
{
    jump_air_profile_prestarted = 0U;
    jump_air_profile_descent = 0U;
    jump_air_profile_reset(&jump_air_profile_state);
    jump_legs_unloaded = 0U;
    landing_airborne_elapsed = 0.0f;
}

void chassis_safety_module_set_jump_legs_unloaded(uint8_t unloaded)
{
    jump_legs_unloaded = unloaded ? 1U : 0U;
}

void chassis_safety_module_set_jump_kinematic_rates(
    float length_rate_left, float length_rate_right,
    float theta_rate_left, float theta_rate_right,
    uint8_t usable, uint8_t fresh, float valid_rate_age_ms)
{
    const uint8_t values_finite =
        isfinite(length_rate_left) && isfinite(length_rate_right) &&
        isfinite(theta_rate_left) && isfinite(theta_rate_right);
    jump_length_rate_snapshot[CHASSIS_SIDE_LEFT] =
        values_finite ? length_rate_left : 0.0f;
    jump_length_rate_snapshot[CHASSIS_SIDE_RIGHT] =
        values_finite ? length_rate_right : 0.0f;
    jump_theta_rate_snapshot[CHASSIS_SIDE_LEFT] =
        values_finite ? theta_rate_left : 0.0f;
    jump_theta_rate_snapshot[CHASSIS_SIDE_RIGHT] =
        values_finite ? theta_rate_right : 0.0f;
    jump_rate_snapshot_age_ms = isfinite(valid_rate_age_ms) &&
        valid_rate_age_ms >= 0.0f ? valid_rate_age_ms : INFINITY;
    jump_rate_snapshot_usable = (uint8_t)(
        jump_local_rate_snapshot_usable(
            usable, jump_rate_snapshot_age_ms,
            JUMP_KIN_RATE_HOLD_MAX_AGE_MS) && values_finite);
    jump_rate_snapshot_fresh = (uint8_t)(
        fresh && jump_rate_snapshot_usable);
}

void chassis_safety_module_set_jump_landing_length_rates(
    float length_rate_left, float length_rate_right, uint8_t valid)
{
    const uint8_t values_finite =
        isfinite(length_rate_left) && isfinite(length_rate_right);
    jump_landing_length_rate_snapshot_valid =
        (uint8_t)(valid && values_finite);
    jump_landing_length_rate_snapshot[CHASSIS_SIDE_LEFT] =
        jump_landing_length_rate_snapshot_valid ? length_rate_left : 0.0f;
    jump_landing_length_rate_snapshot[CHASSIS_SIDE_RIGHT] =
        jump_landing_length_rate_snapshot_valid ? length_rate_right : 0.0f;
}

void chassis_safety_module_set_jump_air_theta_ref(float theta_ref)
{
    jump_air_theta_ref = isfinite(theta_ref) ? theta_ref : 0.0f;
}

float chassis_safety_module_jump_air_theta_ref(void)
{
    return jump_air_theta_ref;
}

void chassis_safety_module_jump_air_profile_set_descent(uint8_t descending)
{
    jump_air_profile_descent = descending ? 1U : 0U;
}

uint8_t chassis_safety_module_jump_air_profile_active(void)
{
    return jump_air_profile_prestarted;
}

uint8_t chassis_safety_module_jump_air_profile_descent(void)
{
    return jump_air_profile_descent;
}

uint8_t chassis_safety_module_jump_air_compact_phase(void)
{
    return jump_air_profile_compact_phase_active(
        &jump_air_profile_state,
        (uint8_t)(safety_context != NULL &&
                  chassis_cmd.ctrl_mode == CHASSIS_JUMP),
        (uint8_t)(landing_buffer_phase != LANDING_BUFFER_INACTIVE));
}

void chassis_safety_module_set_jump_position_feedback_fresh(uint8_t fresh)
{
    jump_air_position_feedback_fresh = fresh ? 1U : 0U;
}

uint8_t chassis_safety_module_jump_air_actual_compact(void)
{
    return jump_air_profile_state.actual_compact;
}

uint8_t chassis_safety_module_jump_air_deploy_started(void)
{
    return jump_air_profile_state.deploy_latched;
}

float chassis_safety_module_jump_air_actual_compact_hold(void)
{
    return jump_air_profile_state.actual_compact_hold_s;
}

float chassis_safety_module_landing_pid_return_gain(void)
{
    if (chassis_safety_module_bump_pose_owner_active()) {
        return 1.0f;
    }
    if (landing_buffer_phase == LANDING_BUFFER_INACTIVE) {
        return 1.0f;
    }
    if (landing_buffer_phase != LANDING_BUFFER_RETURN) {
        return 0.0f;
    }

    /* Keep the reference/buffer latched through rebound, but fade the impact
     * gains independently.  Holding the high derivative gain until perfect
     * stability sustained the measured 10 Hz whole-body mode indefinitely. */
    float gain = landing_buffer_phase_timer / LANDING_PID_RETURN_TIME;
    LIMIT_MIN_MAX(gain, 0.0f, 1.0f);
    return gain;
}

void chassis_safety_module_landing_length_override(
    float nominal_left, float nominal_right,
    float *output_left, float *output_right)
{
    const float nominal[CHASSIS_SIDE_COUNT] = {
        nominal_left, nominal_right
    };

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (isfinite(nominal[side])) {
            landing_requested_ref[side] = nominal[side];
            LIMIT_MIN_MAX(landing_requested_ref[side],
                          LANDING_LENGTH_REF_MIN,
                          LANDING_LENGTH_REF_MAX);
        }
    }

    if (chassis_safety_module_bump_pose_owner_active()) {
        if (output_left != NULL) *output_left = nominal_left;
        if (output_right != NULL) *output_right = nominal_right;
        return;
    }

    if (output_left != NULL) {
        if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
            *output_left = landing_length_ref[LEFT];
        } else if (landing_postbuffer_active) {
            *output_left = landing_postbuffer_ref[LEFT];
        } else {
            *output_left = nominal_left;
        }
    }
    if (output_right != NULL) {
        if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) {
            *output_right = landing_length_ref[RIGHT];
        } else if (landing_postbuffer_active) {
            *output_right = landing_postbuffer_ref[RIGHT];
        } else {
            *output_right = nominal_right;
        }
    }
}

void chassis_safety_module_start_landing_yaw_hold(void)
{
    landing_yaw_hold_timer = LANDING_YAW_HOLD_TIME;
}

uint8_t chassis_safety_module_step_landing_yaw_hold(float dt)
{
    if (landing_yaw_hold_timer <= 0.0f) {
        return 0;
    }

    landing_yaw_hold_timer -= dt;
    if (landing_yaw_hold_timer < 0.0f) {
        landing_yaw_hold_timer = 0.0f;
    }
    return 1;
}

static void safety_debug_page_13(void)
{
    /* Off-ground/landing calibration, exactly ch0~ch15:
     * ch0=time; ch1/ch2=actual L/R l0; ch3/ch4=active L/R length target;
     * ch5/ch6=four-sample filtered contact L/R; ch7=takeoff threshold;
     * ch8=flags; ch9/ch10=enter/exit counters; ch11/ch12=wheel speeds;
     * ch13=landing threshold; ch14=wheel authority (0=airborne, 1=confirmed);
     * ch15=mode+10*level+100*ramping+1000*landing phase+10000*precontact.
     * Flag bit9 marks precontact buffering, bits12/13 are the two landing
     * comparisons, bit14 is their AND, and bit15 means the landing buffer is
     * active. */
    uint16_t flags = 0u;
    const float takeoff_threshold = Offground_TakeoffThreshold();
    const float landing_threshold = Offground_LandingThreshold();
    const uint8_t air_deploy_active =
        chassis_off_ground && chassis_cmd.leg_level == LEG_LOW &&
        landing_buffer_phase == LANDING_BUFFER_INACTIVE;
    const float target_left =
        (landing_buffer_phase != LANDING_BUFFER_INACTIVE) ?
        landing_length_ref[LEFT] :
        (air_deploy_active ? landing_air_deploy_ref[LEFT] :
                             leg[LEFT]->length_ref);
    const float target_right =
        (landing_buffer_phase != LANDING_BUFFER_INACTIVE) ?
        landing_length_ref[RIGHT] :
        (air_deploy_active ? landing_air_deploy_ref[RIGHT] :
                             leg[RIGHT]->length_ref);

    if (Off_Ground_Flag) flags |= 0x0001u;
    if (chassis_off_ground) flags |= 0x0002u;
    if (chassis_takeoff_edge) flags |= 0x0004u;
    if (chassis_landing_edge) flags |= 0x0008u;
    if (Wheel_Shut_Flag) flags |= 0x0010u;
    if (chassis_airborne_allowed()) flags |= 0x0020u;
    if (offground_selected_fn(LEFT) < takeoff_threshold) flags |= 0x0040u;
    if (offground_selected_fn(RIGHT) < takeoff_threshold) flags |= 0x0080u;
    if (Offground_EnterCondition(NULL)) flags |= 0x0100u;
    if (offground_shadow_takeoff_active ||
        (landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
         !landing_contact_confirmed)) flags |= 0x0200u;
#if OFFGROUND_DETECTION_ENABLE
    flags |= 0x0400u;
#endif
    if (Touch_Ground_Flag) flags |= 0x0800u;
    if (offground_selected_fn(LEFT) > landing_threshold)
        flags |= 0x1000u;
    if (offground_selected_fn(RIGHT) > landing_threshold)
        flags |= 0x2000u;
    if (Offground_TouchCondition(NULL)) flags |= 0x4000u;
    if (landing_buffer_phase != LANDING_BUFFER_INACTIVE) flags |= 0x8000u;

    chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
    chassis_debug_set(1, leg[LEFT]->l0);
    chassis_debug_set(2, leg[RIGHT]->l0);
    chassis_debug_set(3, target_left);
    chassis_debug_set(4, target_right);
    chassis_debug_set(5, offground_selected_fn(LEFT));
    chassis_debug_set(6, offground_selected_fn(RIGHT));
    chassis_debug_set(7, takeoff_threshold);
    chassis_debug_set(8, (float)flags);
    chassis_debug_set(9, (float)Off_Ground_Cnt);
    chassis_debug_set(10, (float)Touch_Ground_Cnt);
    chassis_debug_set(11, dbg_wheel_speed_phys_l);
    chassis_debug_set(12, dbg_wheel_speed_phys_r);
    chassis_debug_set(13, landing_threshold);
    chassis_debug_set(14, chassis_safety_module_landing_gain());
    chassis_debug_set(15, (float)chassis_cmd.ctrl_mode +
                           10.0f * (float)chassis_cmd.leg_level +
                           100.0f * (float)chassis_cmd.leg_leng_change +
                           1000.0f * (float)landing_buffer_phase +
                           10000.0f * (float)(
                               landing_buffer_phase != LANDING_BUFFER_INACTIVE &&
                               !landing_contact_confirmed));
}

/* Page 63 combines the latest capture/detector sample with the runtime and
 * impact state cached at update_ground() exit. Wheel_Shut_Flag is therefore
 * the pre-apply_airborne_control value for that cycle. I31 counts calls to
 * reset/airborne_state_clear; it is not a physical landing count. */
static void safety_debug_page_63(void)
{
    const offground_debug_snapshot_t *snapshot = &offground_debug_snapshot;
    uint32_t detector_flags = snapshot->impact_flags;

    if (snapshot->takeoff_side_low[LEFT]) detector_flags |= 1UL << 0;
    if (snapshot->takeoff_side_low[RIGHT]) detector_flags |= 1UL << 1;
    if (snapshot->mode == CHASSIS_JUMP && snapshot->takeoff_mean_pass)
        detector_flags |= 1UL << 2;
    if (snapshot->takeoff_length_gate_pass) detector_flags |= 1UL << 3;
    if (snapshot->touch_side_high[LEFT]) detector_flags |= 1UL << 4;
    if (snapshot->touch_side_high[RIGHT]) detector_flags |= 1UL << 5;
    if (snapshot->touch_inhibit_ready) detector_flags |= 1UL << 6;
    if (snapshot->enter_condition) detector_flags |= 1UL << 7;
    if (snapshot->touch_condition) detector_flags |= 1UL << 8;
    if (snapshot->touch_evaluated) detector_flags |= 1UL << 9;
    if (snapshot->reconstructed_finite[LEFT]) detector_flags |= 1UL << 10;
    if (snapshot->reconstructed_finite[RIGHT]) detector_flags |= 1UL << 11;
    if (snapshot->reconstructed_fallback[LEFT]) detector_flags |= 1UL << 12;
    if (snapshot->reconstructed_fallback[RIGHT]) detector_flags |= 1UL << 13;

    chassis_debug_set(0, snapshot->detect_time_s);
    chassis_debug_set(1, (float)snapshot->mode +
                         10.0f * (float)snapshot->leg_level +
                         100.0f * (float)snapshot->stand_state);
    chassis_debug_set(2, (float)(snapshot->runtime_flags |
                                 ((uint32_t)snapshot->runtime_valid << 16)));
    chassis_debug_set(3, (float)detector_flags);
    chassis_debug_set(4, snapshot->support_candidate[LEFT]);
    chassis_debug_set(5, snapshot->support_candidate[RIGHT]);
    chassis_debug_set(6, snapshot->reconstructed_raw[LEFT]);
    chassis_debug_set(7, snapshot->reconstructed_raw[RIGHT]);
    chassis_debug_set(8, snapshot->filtered_force[LEFT]);
    chassis_debug_set(9, snapshot->filtered_force[RIGHT]);
    chassis_debug_set(10, snapshot->takeoff_side_threshold);
    chassis_debug_set(11, snapshot->takeoff_mean_threshold);
    chassis_debug_set(12, snapshot->landing_threshold);
    chassis_debug_set(13, (float)snapshot->takeoff_progress);
    chassis_debug_set(14, (float)snapshot->landing_progress);
    chassis_debug_set(15, (float)snapshot->takeoff_required +
                          256.0f * (float)snapshot->landing_required);
    chassis_debug_set(16, (float)offground_debug_takeoff_events);
    chassis_debug_set(17, (float)offground_debug_landing_events);
    chassis_debug_set(18, (float)offground_debug_buffer_begins);
    chassis_debug_set(19, snapshot->capture_period_s);
    chassis_debug_set(20, snapshot->reconstruction_dt_s);
    chassis_debug_set(21, snapshot->actual_length[LEFT]);
    chassis_debug_set(22, snapshot->actual_length[RIGHT]);
    chassis_debug_set(23, snapshot->theta[LEFT]);
    chassis_debug_set(24, snapshot->theta[RIGHT]);
    chassis_debug_set(25, snapshot->inertial_force[LEFT]);
    chassis_debug_set(26, snapshot->inertial_force[RIGHT]);
    chassis_debug_set(27, snapshot->pitch);
    chassis_debug_set(28, snapshot->roll);
    chassis_debug_set(29, snapshot->body_z_accel);
    chassis_debug_set(30, snapshot->airborne_elapsed_s);
    chassis_debug_set(31, (float)offground_debug_clear_calls);
}

static void safety_debug_page_64(void)
{
    uint32_t flags = 0U;
    const uint32_t takeoff_count =
        offground_debug_takeoff_events > 255U ? 255U :
        offground_debug_takeoff_events;
    const uint32_t landing_count =
        offground_debug_landing_events > 255U ? 255U :
        offground_debug_landing_events;
    const uint32_t buffer_count =
        offground_debug_buffer_begins > 255U ? 255U :
        offground_debug_buffer_begins;
    const uint8_t strong_candidate =
        offground_confirmation.takeoff_candidate_kind == 2U;
    const uint32_t ordinary_progress = (uint32_t)fmaxf(
        0.0f, fminf(511.0f,
            floorf((strong_candidate ? 0.0f :
                    offground_confirmation.takeoff_progress_ms) * 10.0f)));
    const uint32_t strong_progress = (uint32_t)fmaxf(
        0.0f, fminf(511.0f,
            floorf((strong_candidate ?
                    offground_confirmation.takeoff_strong_progress_ms :
                    0.0f) * 10.0f)));
    const uint32_t packed_takeoff_progress = ordinary_progress |
        (strong_progress << 9);
    const uint8_t detector_valid = offground_legacy_scope_active ?
        offground_filter_initialized :
        (offground_measured_fn_valid && offground_ins_feedback_fresh);

    if (offground_legacy_scope_active) flags |= 1UL << 0;
    if (offground_position_feedback_fresh) flags |= 1UL << 1;
    if (offground_position_view.valid) flags |= 1UL << 2;
    if (offground_position_view.new_rate_sample) flags |= 1UL << 3;
    flags |= ((uint32_t)offground_position_view.rx_advanced_mask & 0x0FUL) << 4;
    if (Off_Ground_Flag) flags |= 1UL << 8;
    if (Wheel_Shut_Flag) flags |= 1UL << 9;
    flags |= ((uint32_t)landing_buffer_phase & 0x03UL) << 10;
    if (bump_active) flags |= 1UL << 12;
    if (detector_valid) flags |= 1UL << 13;
    if (offground_confirmation_evidence_valid) flags |= 1UL << 14;
    if (offground_confirmation.landing_candidate_active) flags |= 1UL << 15;
    if (offground_confirmation.post_landing_active) flags |= 1UL << 16;
    if (offground_debug_impact_eval_valid) flags |= 1UL << 17;
    if (chassis_takeoff_edge) flags |= 1UL << 18;
    if (chassis_landing_edge) flags |= 1UL << 19;
    if (landing_contact_confirmed) flags |= 1UL << 20;
    if (landing_buffer_jump_origin) flags |= 1UL << 21;
    if (landing_airborne_guard_armed) flags |= 1UL << 22;
    if (offground_ins_feedback_fresh) flags |= 1UL << 23;

    chassis_debug_set(0, offground_debug_snapshot.detect_time_s);
    chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                         10.0f * (float)chassis_cmd.leg_level +
                         100.0f * (float)chassis_fdb_data.stand_state +
                         1000.0f * (float)bump_active);
    chassis_debug_set(2, (float)flags);
    chassis_debug_set(3,
        offground_debug_snapshot.capture_period_s * 1000.0f);
    chassis_debug_set(4, offground_feedback_max_age_ms);
    chassis_debug_set(5, offground_last_position_sample_dt_ms);
    chassis_debug_set(6, leg[LEFT]->d_l0_lpf);
    chassis_debug_set(7, leg[RIGHT]->d_l0_lpf);
    chassis_debug_set(8, offground_position_view.valid ?
        offground_position_view.length_rate[LEFT] : 0.0f);
    chassis_debug_set(9, offground_position_view.valid ?
        offground_position_view.length_rate[RIGHT] : 0.0f);
    chassis_debug_set(10, offground_position_view.valid ?
        offground_position_view.length_accel[LEFT] : 0.0f);
    chassis_debug_set(11, offground_position_view.valid ?
        offground_position_view.length_accel[RIGHT] : 0.0f);
    chassis_debug_set(12, offground_position_view.valid ?
        offground_position_view.theta_rate[LEFT] : 0.0f);
    chassis_debug_set(13, offground_position_view.valid ?
        offground_position_view.theta_rate[RIGHT] : 0.0f);
    chassis_debug_set(14, offground_position_view.valid ?
        offground_position_view.theta_accel[LEFT] : 0.0f);
    chassis_debug_set(15, offground_position_view.valid ?
        offground_position_view.theta_accel[RIGHT] : 0.0f);
    chassis_debug_set(16, offground_selected_fn(LEFT));
    chassis_debug_set(17, offground_selected_fn(RIGHT));
    chassis_debug_set(18, leg[LEFT]->l0);
    chassis_debug_set(19, leg[RIGHT]->l0);
    chassis_debug_set(20, leg[LEFT]->support_force);
    chassis_debug_set(21, leg[RIGHT]->support_force);
    chassis_debug_set(24, ins.pitch);
    chassis_debug_set(25, ins.roll);
    chassis_debug_set(26, (float)packed_takeoff_progress);
    chassis_debug_set(27, offground_confirmation.landing_progress_ms);
    chassis_debug_set(28, offground_confirmation.landing_dropout_ms);
    chassis_debug_set(29, offground_confirmation.landing_candidate_ms);
    chassis_debug_set(30, offground_confirmation.post_landing_ms);
    chassis_debug_set(31, (float)(takeoff_count |
        (landing_count << 8) | (buffer_count << 16)));
}

uint8_t chassis_safety_module_fill_debug_page(uint8_t page)
{
    if (safety_context == NULL) {
        return 0;
    }

    if (page == 6) {
        chassis_debug_set(0, (float)chassis_off_ground);
        chassis_debug_set(1, (float)Off_Ground_Flag);
        chassis_debug_set(2, (float)chassis_takeoff_edge);
        chassis_debug_set(3, (float)chassis_landing_edge);
        chassis_debug_set(4, (float)chassis_airborne_allowed());
        chassis_debug_set(5, dbg_raw_support_L);
        chassis_debug_set(6, dbg_raw_support_R);
        chassis_debug_set(7, dbg_raw_fn_avg);
        chassis_debug_set(8, dbg_ground_detect_fn_avg);
        chassis_debug_set(9, leg[LEFT]->support_force);
        chassis_debug_set(10, leg[RIGHT]->support_force);
        chassis_debug_set(11, LQROutBuf[LEFT][0]);
        chassis_debug_set(12, LQROutBuf[RIGHT][0]);
        chassis_debug_set(13, (float)Off_Ground_Cnt);
        chassis_debug_set(14, (float)Touch_Ground_Cnt);
        chassis_debug_set(15, landing_yaw_hold_timer);
        return 1;
    }

    if (page == 7) {
        chassis_debug_set(0, (float)Off_Ground_Flag);
        chassis_debug_set(1, (float)chassis_airborne_allowed());
        chassis_debug_set(2, (float)Wheel_Shut_Flag);
        chassis_debug_set(3, (float)dbg_danger_reason);
        chassis_debug_set(4, leg[LEFT]->theta);
        chassis_debug_set(5, leg[LEFT]->d_theta_lpf);
        chassis_debug_set(6, dbg_offground_lqr_tp_L);
        chassis_debug_set(7, dbg_offground_theta_hold_tp_L);
        chassis_debug_set(8, leg[LEFT]->Tp);
        chassis_debug_set(9, leg[RIGHT]->theta);
        chassis_debug_set(10, leg[RIGHT]->d_theta_lpf);
        chassis_debug_set(11, dbg_offground_lqr_tp_R);
        chassis_debug_set(12, dbg_offground_theta_hold_tp_R);
        chassis_debug_set(13, leg[RIGHT]->Tp);
        chassis_debug_set(14, leg[LEFT]->support_force);
        chassis_debug_set(15, leg[RIGHT]->support_force);
        return 1;
    }

    if (page == 9) {
        chassis_debug_set(0, (float)chassis_off_ground);
        chassis_debug_set(1, (float)chassis_airborne_allowed());
        chassis_debug_set(2, ins.pitch);
        chassis_debug_set(3, ins.gyro[1]);
        chassis_debug_set(4, leg[LEFT]->phi0);
        chassis_debug_set(5, leg[LEFT]->theta);
        chassis_debug_set(6, leg[LEFT]->d_theta_lpf);
        chassis_debug_set(7, dbg_offground_theta_hold_tp_L);
        chassis_debug_set(8, leg[LEFT]->Tp);
        chassis_debug_set(9, leg[RIGHT]->phi0);
        chassis_debug_set(10, leg[RIGHT]->theta);
        chassis_debug_set(11, leg[RIGHT]->d_theta_lpf);
        chassis_debug_set(12, dbg_offground_theta_hold_tp_R);
        chassis_debug_set(13, leg[RIGHT]->Tp);
        chassis_debug_set(14, dbg_offground_lqr_tp_L);
        chassis_debug_set(15, dbg_offground_lqr_tp_R);
        return 1;
    }

    if (page == 10) {
        chassis_debug_set(0, (float)chassis_off_ground);
        chassis_debug_set(1, (float)dbg_offground_tp_test_applied_phase);
        chassis_debug_set(2, offground_tp_test_timer);
        chassis_debug_set(3, dbg_offground_tp_test_L);
        chassis_debug_set(4, leg[LEFT]->phi0);
        chassis_debug_set(5, leg[LEFT]->d_phi0);
        chassis_debug_set(6, leg[LEFT]->theta);
        chassis_debug_set(7, dm_obs[3]);
        chassis_debug_set(8, dm_obs[0]);
        chassis_debug_set(9, dbg_offground_tp_test_R);
        chassis_debug_set(10, leg[RIGHT]->phi0);
        chassis_debug_set(11, leg[RIGHT]->d_phi0);
        chassis_debug_set(12, leg[RIGHT]->theta);
        chassis_debug_set(13, dm_obs[2]);
        chassis_debug_set(14, dm_obs[1]);
        chassis_debug_set(15, (float)(offground_tp_test_phase != OFFGROUND_TP_TEST_DONE));
        return 1;
    }

    if (page == 13) {
        safety_debug_page_13();
        return 1;
    }

    if (page == 63) {
        safety_debug_page_63();
        return 1;
    }

    if (page == 64U) {
        safety_debug_page_64();
        return 1U;
    }

    return 0;
}
