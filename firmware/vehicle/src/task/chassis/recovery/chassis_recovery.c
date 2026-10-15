/**
 * @file chassis_recovery.c
 * @brief Torque-controlled recovery state machine and its diagnostics.
 */
#include "chassis_recovery.h"
#include "debug/chassis_debug.h"
#include "rm_config.h"

#include <math.h>

#define LEFT  CHASSIS_SIDE_LEFT
#define RIGHT CHASSIS_SIDE_RIGHT

static const chassis_recovery_context_t *recovery_context = NULL;

#define leg                  (recovery_context->leg)
#define ins                  (*recovery_context->ins)
#define chassis_cmd          (*recovery_context->cmd)
#define chassis_fdb_data     (*recovery_context->fdb)
#define theta_pid            (recovery_context->theta_pid)
#define roll_pid             (recovery_context->roll_pid)
#define L_length_pid         (recovery_context->length_pid[LEFT])
#define R_length_pid         (recovery_context->length_pid[RIGHT])
#define vmc_out_l            (recovery_context->vmc_out[LEFT])
#define vmc_out_r            (recovery_context->vmc_out[RIGHT])
#define LQRXObsBuf           (recovery_context->lqr_x_obs)
#define LQROutBuf            (recovery_context->lqr_output)
#define lqr_virtual_x_l      (*recovery_context->lqr_virtual_x[LEFT])
#define lqr_virtual_x_r      (*recovery_context->lqr_virtual_x[RIGHT])
#define chassis_dt           (recovery_context->control_dt_s)
#define LEN_LEN_LOW          (recovery_context->low_leg_length)

#define RECOVERY_LQR_HANDOFF_RAMP_RATE 8.0f
#define RECOVERY_LQR_HANDOFF_INITIAL_GAIN 0.35f
/* Recovery retract 调试变量 */
static float dbg_retract_l0_ref_L = 0, dbg_retract_l0_ref_R = 0;
static float dbg_retract_phi0_ref_L = 0, dbg_retract_phi0_ref_R = 0;
static float dbg_retract_F_L = 0, dbg_retract_F_R = 0;
static float dbg_retract_Tp_L = 0, dbg_retract_Tp_R = 0;
static float dbg_retract_spring_ff_L = 0.0f, dbg_retract_spring_ff_R = 0.0f;
static float dbg_retract_d_l0_L = 0.0f, dbg_retract_d_l0_R = 0.0f;
static float dbg_retract_gain = 0;
static uint8_t dbg_retract_pose_ok = 0;
static uint8_t dbg_retract_state = 0;
static uint8_t dbg_retract_stable_mask = 0;

/* Recovery debug 变量 */
/* Recovery 条件 flag 变量 */
/* Recovery crash/retract capture variables for VOFA page 0. */
#define RECOVERY_DBG_LATCH_PITCH_HIGH    0x01u
#define RECOVERY_DBG_LATCH_PHI0_FORWARD  0x02u
#define RECOVERY_DBG_LATCH_RETRACT_CMD   0x04u
#define RECOVERY_DBG_LATCH_FORCE_LIMIT   0x08u
#define RECOVERY_DBG_LATCH_TP_LIMIT      0x10u
#define RECOVERY_DBG_PITCH_HIGH_DEG      8.0f
#define RECOVERY_DBG_PHI0_FORWARD_THRESH (45.0f * DEGREE_2_RAD)
#define RECOVERY_DBG_L0_RETRACT_THRESH   0.20f
#define RECOVERY_DBG_LIMIT_RATIO         0.90f
static uint16_t dbg_recovery_event_latch = 0;
static float dbg_recovery_phi0_ref_active_L = 0.0f;
static float dbg_recovery_phi0_ref_active_R = 0.0f;
static float dbg_recovery_l0_ref_active_L = 0.0f;
static float dbg_recovery_l0_ref_active_R = 0.0f;

#define RECOVERY_SIMPLE_ENABLE 1

typedef enum {
    REC_SIMPLE_ENTER = 0,
    REC_SIMPLE_HOLD,
    REC_SIMPLE_CHECK_ZERO,
    REC_SIMPLE_SWING,
    REC_SIMPLE_RETRACT,
    REC_SIMPLE_RETRACT_HOLD,
    REC_SIMPLE_LQR_PREP,
    REC_SIMPLE_LQR_HANDOFF,
    REC_SIMPLE_DONE,
    REC_SIMPLE_FAULT,
    REC_SIMPLE_FORWARD_ZERO = 10,
    REC_SIMPLE_INVERTED_CAPTURE = 11,
    REC_SIMPLE_INVERTED_PRELOAD = 12,
    REC_SIMPLE_INVERTED_ALIGN = 13,
    REC_SIMPLE_INVERTED_TURN = 14,
    REC_SIMPLE_INVERTED_SETTLE = 15
} recovery_simple_state_e;

#define REC_SIMPLE_HOLD_TIME            0.22f
#define REC_SIMPLE_ZERO_PITCH_MAX       15.0f
#define REC_SIMPLE_FORWARD_PITCH_MAX    80.0f
#define REC_SIMPLE_INVERTED_ENTER_PITCH_MIN 105.0f
#define REC_SIMPLE_INVERTED_EXIT_PITCH_MAX  40.0f
#define REC_SIMPLE_INVERTED_CAPTURE_TIME    0.20f
#define REC_SIMPLE_INVERTED_PRELOAD_TIME    0.25f
#define REC_SIMPLE_INVERTED_ALIGN_OK        (8.0f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_ALIGN_TIMEOUT   6.0f
#define REC_SIMPLE_INVERTED_TURN_TIMEOUT    100.0f
#define REC_SIMPLE_INVERTED_TURN_MAX_PHI_TRAVEL (720.0f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_TURN_PHI_RATE  (105.0f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_TURN_PHI_ACCEL (240.0f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_SYNC_KP          1.0f
#define REC_SIMPLE_INVERTED_SYNC_CORR_LIMIT (37.5f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_WAIT_LEG_ENTER  (12.0f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_WAIT_LEG_EXIT    (6.0f * DEGREE_2_RAD)
#define REC_SIMPLE_INVERTED_WAIT_NONE        0
#define REC_SIMPLE_INVERTED_WAIT_LEFT        1
#define REC_SIMPLE_INVERTED_WAIT_RIGHT      -1
#define REC_SIMPLE_INVERTED_TURN_TP_KP      36.0f
#define REC_SIMPLE_INVERTED_TURN_TP_KD       2.6f
#define REC_SIMPLE_INVERTED_TURN_TP_LIMIT    9.5f
#define REC_SIMPLE_INVERTED_TURN_TP_BIAS     4.0f
#define REC_SIMPLE_INVERTED_TURN_GAIN_RATE   3.0f
#define REC_SIMPLE_INVERTED_CONTACT_FORCE    12.0f
#define REC_SIMPLE_INVERTED_PITCH_PROGRESS_EPS 0.10f
#define REC_SIMPLE_INVERTED_PITCH_STALL_ARM_TIME 25.0f
#define REC_SIMPLE_INVERTED_PITCH_STALL_TIME 0.75f
#define REC_SIMPLE_INVERTED_SETTLE_TIME      0.50f
#define REC_SIMPLE_INVERTED_SETTLE_TP_KP    12.0f
#define REC_SIMPLE_INVERTED_SETTLE_TP_KD     2.0f
#define REC_SIMPLE_INVERTED_SETTLE_TP_LIMIT  3.0f
#define REC_SIMPLE_FORWARD_ZERO_TIMEOUT 20.0f
/* Phase 10 must hand off near the real zero point; too wide a gate makes phase 3 cross underbody. */
#define REC_SIMPLE_FORWARD_ZERO_PHI0_OK 0.25f
#define REC_SIMPLE_FORWARD_ZERO_STABLE_TIME 0.15f
#define REC_SIMPLE_FORWARD_ZERO_TP_KP  16.0f
#define REC_SIMPLE_FORWARD_ZERO_TP_KD   2.2f
/* Phase 10 previously saturated at only about 3.75 N.m per hip motor.  That
 * was not enough to cross the measured static-friction/dead-point pose, so a
 * stalled leg only continued after being moved by hand. */
#define REC_SIMPLE_FORWARD_ZERO_TP_LIMIT 12.0f
#define REC_SIMPLE_FORWARD_ZERO_TP_SIGN 1.0f
#define REC_SIMPLE_FORWARD_ZERO_GAIN_RATE 4.0f
#define REC_SIMPLE_FORWARD_ZERO_REF_RATE (110.0f * DEGREE_2_RAD)
#define REC_SIMPLE_FORWARD_ZERO_REF_ACCEL (220.0f * DEGREE_2_RAD)
#define REC_SIMPLE_FORWARD_ZERO_D_L0_MAX 1.50f
#define REC_SIMPLE_FORWARD_ZERO_MOTION_DEBOUNCE_TIME 0.030f
/* 正置回零选路在入口锁存：任一腿 phi0 > 90° 优先走身上路径；
 * 否则按 pitch 符号决定。注意本机 ins.pitch 是"前倾为正"
 * （LQR 的 pitch 状态量用的是 -ins.pitch，见 chassis_control.c），所以：
 *   后仰 = ins.pitch <= 0 -> 身上路径（phi0 正向，末段从身上侧接近零位）
 *   前倾 = ins.pitch >  0 -> 身下路径（最短有符号）
 * 实测证据：pitch=-31.7° 的躺姿走身下时 phi0 递减把机身带倒到 -77°；该姿态要
 * 扶正必须让 phi0 递增。旧的 45°/150° 侧姿分支已删除，回零途中不再重新选路。 */
#define REC_SIMPLE_OVER_BODY_PHI0_MIN (90.0f * DEGREE_2_RAD)
/* 进入自救时的初始姿态门：两腿实测长度都低于此值就跳过"回零"和预备摆腿，
 * 直接进收腿。只在 REC_SIMPLE_ENTER 判断一次；收腿阶段的交接门槛仍由
 * REC_SIMPLE_RETRACT_L0_OK(0.17 m) 决定，不随本值变化。 */
#define REC_SIMPLE_SKIP_ZERO_L0   0.20f
#define REC_SIMPLE_SWING_TIMEOUT        8.0f
#define REC_SIMPLE_SWING_PHI0_OK        (4.0f * DEGREE_2_RAD)
#define REC_SIMPLE_SWING_GAIN_RATE      1.5f
#define REC_SIMPLE_SWING_PHI0_RATE      (135.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_SWING_PHI0_RATE (135.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_DEADPOINT_PHI0_RATE (70.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_COMPACT_PHI0_RATE (150.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_COMPACT_L0   0.200f
#define REC_SIMPLE_SWING_PHI0_ACCEL     (360.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_COMPACT_PHI0_ACCEL (420.0f * DEGREE_2_RAD)
#define REC_SIMPLE_SWING_REF_RATE_OK    (3.0f * DEGREE_2_RAD)
#define REC_SIMPLE_SWING_ACTUAL_RATE_OK (6.0f * DEGREE_2_RAD)
#define REC_SIMPLE_SWING_TP_KP          70.0f
#define REC_SIMPLE_SWING_TP_KD           1.6f
#define REC_SIMPLE_SWING_TP_FF           3.0f
#define REC_SIMPLE_SWING_TP_LIMIT       15.0f
#define REC_SIMPLE_SWING_OMEGA_FILTER_TAU 0.025f
#define REC_SIMPLE_SWING_TP_SLEW_RATE   120.0f
#define REC_SIMPLE_RETRACT_TERMINAL_TP_SLEW_RATE 320.0f
#define REC_SIMPLE_CONTROL_DT_DEFAULT   0.005f
#define REC_SIMPLE_CONTROL_DT_MIN       0.0005f
#define REC_SIMPLE_CONTROL_DT_MAX       0.010f
#define REC_SIMPLE_RETRACT_L0_TARGET   LEN_LEN_LOW
#define REC_SIMPLE_RETRACT_L0_RATE     0.25f
#define REC_SIMPLE_RETRACT_L0_OK       0.170f
#define REC_SIMPLE_RETRACT_STABLE_TIME 0.015f
#define REC_SIMPLE_RETRACT_TIMEOUT     5.0f
#define REC_SIMPLE_RETRACT_F_GAIN_RATE_L 0.9f
#define REC_SIMPLE_RETRACT_F_GAIN_RATE_R 0.9f
#define REC_SIMPLE_RETRACT_TP_GAIN_INIT 0.30f
#define REC_SIMPLE_RETRACT_TP_GAIN_RATE 2.00f
/* Do not continue compressing after a leg has crossed below the body. */
#define REC_SIMPLE_RETRACT_UNDERBODY_GUARD (-8.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_F_LIMIT_L     100.0f
#define REC_SIMPLE_RETRACT_F_LIMIT_R     100.0f
#define REC_SIMPLE_RETRACT_F_EXTEND_LIMIT 20.0f
#define REC_SIMPLE_RETRACT_TORQUE_RESERVE_L 19.0f
#define REC_SIMPLE_RETRACT_TORQUE_RESERVE_R 19.0f
#define REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_L0 0.285f
#define REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_BAND 0.020f
#define REC_SIMPLE_RETRACT_BREAKAWAY_MIN_TIME_L 0.050f
#define REC_SIMPLE_RETRACT_BREAKAWAY_TIME_L 0.180f
#define REC_SIMPLE_RETRACT_BREAKAWAY_RAMP_TIME_L 0.020f
#define REC_SIMPLE_RETRACT_BREAKAWAY_F_L   180.0f
#define REC_SIMPLE_RETRACT_BREAKAWAY_TORQUE_L 24.0f
#define REC_SIMPLE_RETRACT_BREAKAWAY_MIN_TIME 0.050f
#define REC_SIMPLE_RETRACT_BREAKAWAY_TIME 0.180f
#define REC_SIMPLE_RETRACT_BREAKAWAY_RAMP_TIME 0.020f
#define REC_SIMPLE_RETRACT_BREAKAWAY_F_R  180.0f
#define REC_SIMPLE_RETRACT_BREAKAWAY_TORQUE_R 24.0f
#define REC_SIMPLE_AUTO_HANDOFF_ENABLE 1
#define REC_SIMPLE_LQR_PREP_TIME       0.0f
#define REC_SIMPLE_LQR_LENGTH_HOLD_TIME 3.0f
#define REC_SIMPLE_HANDOFF_RATE        1.5f
/* After the low-leg LQR blend reaches full gain, it becomes responsible for
 * pitch convergence. Recovery only waits for the leg-length motion to settle. */
#define REC_SIMPLE_HANDOFF_D_L0_OK        0.10f
#define REC_SIMPLE_HANDOFF_STABLE_TIME    0.15f
#define REC_SIMPLE_F_LIMIT             360.0f
#define REC_SIMPLE_F_BIAS              40.0f
#define REC_SIMPLE_L0_KP               1200.0f
#define REC_SIMPLE_L0_KD               30.0f
#define REC_SIMPLE_PHI0_TARGET         (30.0f * DEGREE_2_RAD)  /* Wheel-ground limited prepose angle. */
#define REC_SIMPLE_PHI0_RATE           (20.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_PHI0_FINAL_TARGET (80.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_PHI0_L0_START     0.300f
#define REC_SIMPLE_RETRACT_PHI0_L0_FINAL     0.180f
#define REC_SIMPLE_RETRACT_PHI0_RATE         (45.0f * DEGREE_2_RAD)
#define REC_SIMPLE_RETRACT_SETTLE_L0_START   0.200f
#define REC_SIMPLE_RETRACT_TP_SETTLE_KP      32.0f
#define REC_SIMPLE_RETRACT_TP_SETTLE_KD      5.5f
#define REC_SIMPLE_RETRACT_TP_SETTLE_LIMIT  10.0f
#define REC_SIMPLE_SPRING_FF_GAIN      1.0f
#define REC_SIMPLE_SPRING_FF_LIMIT     140.0f
#define REC_SIMPLE_PUSH_BIAS_L0_RANGE  0.08f
#define REC_SIMPLE_FAULT_PITCH_MAX     25.0f
#define REC_SIMPLE_FAULT_L0_MIN        0.095f
#define REC_SIMPLE_FAULT_D_L0_MAX      0.70f
#define REC_SIMPLE_MOTION_FAULT_DEBOUNCE_TIME 0.020f
#define REC_SIMPLE_RETRACT_D_L0_EXTEND_MAX   1.50f
#define REC_SIMPLE_RETRACT_D_L0_COMPRESS_MAX 10.00f
#define REC_SIMPLE_FAULT_D_PHI0_MAX    18.0f
#define REC_SIMPLE_RETRACT_D_PHI0_MAX  35.0f
#define REC_SIMPLE_RETRACT_FAULT_DEBOUNCE_TIME 0.020f
#define REC_SIMPLE_RETRACT_MOTION_FAULT_ARM_TIME 0.80f
#define REC_SIMPLE_STABLE_D_L0_MAX     0.06f
#define REC_SIMPLE_STABLE_D_PHI0_MAX   1.20f
#define REC_SIMPLE_STABLE_D_THETA_MAX  1.00f
#define REC_SIMPLE_STABLE_PHI0_ERR_MAX (10.0f * DEGREE_2_RAD)
#define REC_SIMPLE_STABLE_F_MAX        35.0f
#define REC_SIMPLE_STABLE_TP_MAX       5.0f

static recovery_simple_state_e recovery_simple_state = REC_SIMPLE_ENTER;
typedef struct {
    float actual;
    float start_actual;
    float last_raw;
    float ref;
    float target;
    float omega_actual;
    float omega_ref;
    float tp_cmd;
    float direction;
    uint8_t initialized;
} recovery_swing_axis_t;

static recovery_swing_axis_t recovery_swing_axis[CHASSIS_SIDE_COUNT];
static float recovery_simple_control_dt = REC_SIMPLE_CONTROL_DT_DEFAULT;
/* Unclamped interval between two actual Recovery executions.  This is kept
 * separate from recovery_simple_control_dt so Page 48 can expose scheduler or
 * CAN-induced stalls that the controller's 10 ms safety clamp would hide. */
static float recovery_simple_raw_dt = REC_SIMPLE_CONTROL_DT_DEFAULT;
static float dbg_swing_tp_raw_L = 0.0f;
static float dbg_swing_tp_raw_R = 0.0f;
static float dbg_swing_sync_error = 0.0f;
static float recovery_simple_timer = 0.0f;
static float recovery_simple_stable_timer = 0.0f;
static float recovery_simple_l0_ref_L = 0.0f;
static float recovery_simple_l0_ref_R = 0.0f;
static float recovery_simple_phi0_ref_L = 0.0f;
static float recovery_simple_phi0_ref_R = 0.0f;
static float recovery_simple_force_gain_L = 0.0f;
static float recovery_simple_force_gain_R = 0.0f;
static uint8_t recovery_simple_breakaway_released = 0;
static float recovery_simple_retract_fault_timer = 0.0f;
static float recovery_simple_handoff_F_L = 0.0f;
static float recovery_simple_handoff_F_R = 0.0f;
static float recovery_simple_handoff_Tp_L = 0.0f;
static float recovery_simple_handoff_Tp_R = 0.0f;
static float recovery_simple_forward_zero_target_L = 0.0f;
static float recovery_simple_forward_zero_target_R = 0.0f;
static float recovery_simple_motion_fault_timer = 0.0f;
static float recovery_simple_forward_zero_motion_timer = 0.0f;
static float recovery_simple_forward_zero_stable_timer = 0.0f;
static float recovery_simple_inverted_phi_target_L = 0.0f;
static float recovery_simple_inverted_phi_target_R = 0.0f;
static float recovery_simple_inverted_entry_pitch = 0.0f;
/* 翻身前对齐：两条腿共同的收敛终点，以及右腿的展开角（供翻身捕获复用）。 */
static float recovery_simple_inverted_align_target = 0.0f;
static float recovery_simple_inverted_right_unwrapped = 0.0f;
static float recovery_simple_inverted_best_abs_pitch = 0.0f;
static float recovery_simple_inverted_pitch_stall_timer = 0.0f;
static int8_t recovery_simple_inverted_turn_sign_L = 0;
static int8_t recovery_simple_inverted_turn_sign_R = 0;
/* +1: hold left for right to catch up; -1: hold right for left to catch up. */
static int8_t recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
/* phase 10 fault code: 1=pitch, 2=leg length, 3=motion, 4=timeout. */
static uint8_t dbg_forward_zero_fault_reason = 0;
/* simple fault code: tens=phase, ones=reason; reason 1=pitch, 2=l0, 3=motion, 4=timeout. */
static uint8_t dbg_simple_fault_reason = 0;
static uint8_t recovery_simple_zero_ok = 0;
/* 正置回零路径锁存：1=身上路径(phi0 正向，末段从身上侧接近零位)，
 * 0=身下路径(最短有符号)。入口优先判断腿角，再判断 pitch，途中不再重选。 */
static uint8_t recovery_simple_over_body = 0;
/* 入口紧凑跳过回零的事件锁存（供 0 页 I3 bit8 观察）。 */
static uint8_t recovery_simple_skip_zero_fired = 0;
/* 1 = 本次自救由"入口紧凑跳过回零"进入（直接进 RETRACT）。用于放行收腿阶段
 * 的双向摆角收敛：正常路径的"只许朝站立方向前进、不许回拉"规则在这里会把
 * 起始摆角大于 80° 的腿锁死，永远进不了交接稳定窗。 */
static uint8_t recovery_simple_compact_entry = 0;

/* Recovery retract 状态变量（文件作用域，便于 reset） */
static float recovery_simple_angle_gain = 0.0f;
/* Recovery assist shorten 状态变量 */
static float    recovery_lqr_handoff_gain = 0.0f;
static uint8_t  recovery_lqr_handoff_last_active = 0;
static float    recovery_simple_lqr_length_hold_L = 0.0f;
static float    recovery_simple_lqr_length_hold_R = 0.0f;
static float    recovery_simple_lqr_length_hold_timer = 0.0f;
static float    recovery_simple_lqr_length_hold_start_ms = 0.0f;
static uint8_t  recovery_simple_lqr_length_hold_active = 0;
static float    recovery_simple_handoff_stable_timer = 0.0f;
/* Recovery 目标斜坡状态变量 */
static uint8_t recovery_active_last = 0;

/* Recovery 预备位判定参数 */
#define RECOVERY_PREPOSE_PHI_ERR_OK      (10.0f * DEGREE_2_RAD)
#define RECOVERY_PREPOSE_L0_LONG_MIN     0.31f
#define RECOVERY_PREPOSE_DPHI_OK         1.2f
#define RECOVERY_PREPOSE_DL0_OK          0.30f
#define RECOVERY_PREPOSE_OK_TIME         0.15f
#define RECOVERY_PREPOSE_MIN_TIME        0.35f
static float recovery_min_time = 0.0f;


static void Recovery_Simple_Reset(void);
static void Recovery_Simple_StateMachine(float dt);
static void Recovery_Simple_RetractControl(void);
static void Recovery_Simple_ApplyLqrLengthHold(void);
static void Recovery_DebugCaptureUpdate(void);
static inline uint8_t Recovery_IsZeroPose(void);

static void Recovery_SeedCurrentPose(void)
{
    leg[LEFT]->support_force = 0.0f;
    leg[RIGHT]->support_force = 0.0f;
    leg[LEFT]->Tp = 0.0f;
    leg[RIGHT]->Tp = 0.0f;
}

static void Recovery_Pose_ResetOnEnter(void)
{
    dbg_recovery_event_latch = 0;
    recovery_lqr_handoff_gain = 0.0f;
    recovery_lqr_handoff_last_active = 0;
    Recovery_Simple_Reset();
    /* 回零路径在入口只锁存一次：任一腿朝身前(phi0 > 90°)时，
     * 无论 pitch 正负都走身上路径；否则后仰(pitch<=0)走身上，前倾(pitch>0)走身下。
     * 之后无论中途姿态怎么变（包括倒置翻身翻回来之后的姿态），本次自救都
     * 沿用这一个方向，不再重新选路。 */
    const uint8_t leg_forward =
        (isfinite(leg[LEFT]->phi0) &&
         leg[LEFT]->phi0 > REC_SIMPLE_OVER_BODY_PHI0_MIN) ||
        (isfinite(leg[RIGHT]->phi0) &&
         leg[RIGHT]->phi0 > REC_SIMPLE_OVER_BODY_PHI0_MIN);
    recovery_simple_over_body = (leg_forward || ins.pitch <= 0.0f) ? 1u : 0u;
    Recovery_SeedCurrentPose();
}

static void Recovery_Pose_ResetOnExit(void)
{
    Recovery_SeedCurrentPose();
    recovery_lqr_handoff_gain = 0.0f;
    recovery_lqr_handoff_last_active = 0;

    /* Keep the compact-leg hold during the automatic RECOVERY -> OPEN_LOOP handoff. */
    if (recovery_simple_state != REC_SIMPLE_DONE ||
        !recovery_simple_lqr_length_hold_active) {
        recovery_simple_lqr_length_hold_active = 0;
        recovery_simple_lqr_length_hold_timer = 0.0f;
        recovery_simple_lqr_length_hold_start_ms = 0.0f;
    }
}

static inline float positive_angle_delta_f32(float target, float current)
{
    float delta = fmodf(target - current, PI2);
    if (delta < 0.0f) {
        delta += PI2;
    }
    return delta;
}
static float slope_to_target_f32(float cur, float target, float max_step)
{
    if (cur < target) {
        cur += max_step;
        if (cur > target) cur = target;
    } else if (cur > target) {
        cur -= max_step;
        if (cur < target) cur = target;
    }
    return cur;
}

static float ramp_to(float cur, float target, float max_step)
{
    if (cur < target) {
        cur += max_step;
        if (cur > target) cur = target;
    } else if (cur > target) {
        cur -= max_step;
        if (cur < target) cur = target;
    }
    return cur;
}

static inline float wrap_pi_f32(float x)
{
    while (x > PI) {
        x -= 2.0f * PI;
    }
    while (x < -PI) {
        x += 2.0f * PI;
    }
    return x;
}

static inline float angle_err_abs_f32(float a, float b)
{
    return fabsf(wrap_pi_f32(a - b));
}

static void Recovery_SwingAxisCapture(recovery_swing_axis_t *axis,
                                      float raw_angle,
                                      float actual_angle,
                                      float target_angle)
{
    axis->actual = actual_angle;
    axis->start_actual = actual_angle;
    axis->last_raw = raw_angle;
    axis->ref = actual_angle;
    axis->target = target_angle;
    axis->omega_actual = 0.0f;
    axis->omega_ref = 0.0f;
    axis->tp_cmd = 0.0f;
    axis->direction = (target_angle >= actual_angle) ? 1.0f : -1.0f;
    axis->initialized = 1;
}

static void Recovery_SwingAxisUpdateActual(recovery_swing_axis_t *axis,
                                           float raw_angle,
                                           float dt)
{
    float delta;
    float omega_raw;
    float alpha;

    if (!axis->initialized) {
        Recovery_SwingAxisCapture(axis, raw_angle, raw_angle, raw_angle);
        return;
    }

    delta = wrap_pi_f32(raw_angle - axis->last_raw);
    axis->last_raw = raw_angle;
    axis->actual += delta;

    omega_raw = delta / dt;
    alpha = dt / (REC_SIMPLE_SWING_OMEGA_FILTER_TAU + dt);
    axis->omega_actual += alpha * (omega_raw - axis->omega_actual);
}

static void Recovery_SwingAxisRun(recovery_swing_axis_t *axis,
                                  float speed_limit,
                                  float accel_limit,
                                  float dt)
{
    float remaining = axis->direction * (axis->target - axis->ref);
    float omega_target = 0.0f;

    if (remaining > 0.0f) {
        float brake_speed = sqrtf(2.0f * accel_limit * remaining);
        float speed = speed_limit;

        if (brake_speed < speed) {
            speed = brake_speed;
        }
        omega_target = axis->direction * speed;
    }

    axis->omega_ref = ramp_to(axis->omega_ref, omega_target, accel_limit * dt);
    axis->ref += axis->omega_ref * dt;

    if (axis->direction * (axis->ref - axis->target) >= 0.0f) {
        axis->ref = axis->target;
        axis->omega_ref = 0.0f;
    }
}

static void Recovery_SwingAxisBrake(recovery_swing_axis_t *axis,
                                    float accel_limit,
                                    float dt)
{
    axis->omega_ref = ramp_to(axis->omega_ref, 0.0f, accel_limit * dt);
    axis->ref += axis->omega_ref * dt;
}

#if RECOVERY_SIMPLE_ENABLE
static void Recovery_Simple_Reset(void)
{
    for (uint8_t side = 0; side < CHASSIS_SIDE_COUNT; side++) {
        recovery_swing_axis[side] = (recovery_swing_axis_t){0};
    }
    recovery_simple_control_dt = REC_SIMPLE_CONTROL_DT_DEFAULT;
    dbg_swing_tp_raw_L = 0.0f;
    dbg_swing_tp_raw_R = 0.0f;
    dbg_swing_sync_error = 0.0f;
    recovery_simple_state = REC_SIMPLE_ENTER;
    recovery_simple_timer = 0.0f;
    recovery_simple_stable_timer = 0.0f;
    recovery_simple_l0_ref_L = 0.0f;
    recovery_simple_l0_ref_R = 0.0f;
    recovery_simple_phi0_ref_L = 0.0f;
    recovery_simple_phi0_ref_R = 0.0f;
    recovery_simple_force_gain_L = 0.0f;
    recovery_simple_force_gain_R = 0.0f;
    recovery_simple_breakaway_released = 0;
    recovery_simple_retract_fault_timer = 0.0f;
    recovery_simple_handoff_F_L = 0.0f;
    recovery_simple_handoff_F_R = 0.0f;
    recovery_simple_handoff_Tp_L = 0.0f;
    recovery_simple_handoff_Tp_R = 0.0f;
    recovery_simple_lqr_length_hold_L = 0.0f;
    recovery_simple_lqr_length_hold_R = 0.0f;
    recovery_simple_lqr_length_hold_timer = 0.0f;
    recovery_simple_lqr_length_hold_start_ms = 0.0f;
    recovery_simple_lqr_length_hold_active = 0;
    recovery_simple_handoff_stable_timer = 0.0f;
    recovery_simple_forward_zero_target_L = 0.0f;
    recovery_simple_forward_zero_target_R = 0.0f;
    recovery_simple_motion_fault_timer = 0.0f;
    recovery_simple_forward_zero_motion_timer = 0.0f;
    recovery_simple_forward_zero_stable_timer = 0.0f;
    recovery_simple_inverted_phi_target_L = 0.0f;
    recovery_simple_inverted_phi_target_R = 0.0f;
    recovery_simple_inverted_entry_pitch = 0.0f;
    recovery_simple_inverted_align_target = 0.0f;
    recovery_simple_inverted_right_unwrapped = 0.0f;
    recovery_simple_inverted_best_abs_pitch = 0.0f;
    recovery_simple_inverted_pitch_stall_timer = 0.0f;
    recovery_simple_inverted_turn_sign_L = 0;
    recovery_simple_inverted_turn_sign_R = 0;
    recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
    dbg_forward_zero_fault_reason = 0;
    dbg_simple_fault_reason = 0;
    recovery_simple_zero_ok = 0;
    recovery_simple_angle_gain = 0.0f;
    recovery_lqr_handoff_gain = 0.0f;
    recovery_lqr_handoff_last_active = 0;
    dbg_retract_l0_ref_L = 0.0f;
    dbg_retract_l0_ref_R = 0.0f;
    dbg_retract_phi0_ref_L = 0.0f;
    dbg_retract_phi0_ref_R = 0.0f;
    dbg_retract_F_L = 0.0f;
    dbg_retract_F_R = 0.0f;
    dbg_retract_Tp_L = 0.0f;
    dbg_retract_Tp_R = 0.0f;
    dbg_retract_spring_ff_L = 0.0f;
    dbg_retract_spring_ff_R = 0.0f;
    dbg_retract_d_l0_L = 0.0f;
    dbg_retract_d_l0_R = 0.0f;
    dbg_retract_gain = 0.0f;
    dbg_retract_state = (float)REC_SIMPLE_ENTER;
    dbg_retract_pose_ok = 0;
    dbg_retract_stable_mask = 0;
    recovery_simple_over_body = 0;
    recovery_simple_skip_zero_fired = 0;
    recovery_simple_compact_entry = 0;
    leg[LEFT]->support_force = 0.0f;
    leg[RIGHT]->support_force = 0.0f;
    leg[LEFT]->Tp = 0.0f;
    leg[RIGHT]->Tp = 0.0f;
}

static void Recovery_Simple_ApplyLqrLengthHold(void)
{
    float elapsed;

    if (!recovery_simple_lqr_length_hold_active) {
        return;
    }

    /* RELAX/INIT explicitly reset the recovery state. Other modes may be the
     * normal OPEN_LOOP handoff published immediately after a successful stand. */
    if (chassis_cmd.ctrl_mode == CHASSIS_RELAX ||
        chassis_cmd.ctrl_mode == CHASSIS_INIT) {
        recovery_simple_lqr_length_hold_active = 0;
        recovery_simple_lqr_length_hold_timer = 0.0f;
        recovery_simple_lqr_length_hold_start_ms = 0.0f;
        return;
    }

    elapsed = (dwt_get_time_ms() - recovery_simple_lqr_length_hold_start_ms) / 1000.0f;
    if (elapsed < 0.0f) {
        elapsed = 0.0f;
    }
    if (elapsed < REC_SIMPLE_LQR_LENGTH_HOLD_TIME) {
        leg[LEFT]->length_ref = recovery_simple_lqr_length_hold_L;
        leg[RIGHT]->length_ref = recovery_simple_lqr_length_hold_R;
        /* 正常腿长调度在本周期前半段会尝试走向 LOW，并将状态标成
         * LENGTH_CHANGE。保持逻辑覆盖目标后必须同步纠正状态，否则
         * 静摩擦补偿会把静止保持误判成腿长切换并持续蓄力。 */
        chassis_cmd.leg_leng_change = LENGTH_STAY;
        recovery_simple_lqr_length_hold_timer = elapsed;
    } else {
        recovery_simple_lqr_length_hold_timer = REC_SIMPLE_LQR_LENGTH_HOLD_TIME;
        recovery_simple_lqr_length_hold_active = 0;
    }
}

static uint8_t Recovery_Simple_MotionFault(void)
{
    float d_l0_L = leg[LEFT]->d_l0_lpf;
    float d_l0_R = leg[RIGHT]->d_l0_lpf;
    uint8_t unsafe;

    if (isnan(d_l0_L) || isinf(d_l0_L)) {
        d_l0_L = leg[LEFT]->d_l0;
    }
    if (isnan(d_l0_R) || isinf(d_l0_R)) {
        d_l0_R = leg[RIGHT]->d_l0;
    }

    unsafe = (fabsf(d_l0_L) > REC_SIMPLE_FAULT_D_L0_MAX ||
              fabsf(d_l0_R) > REC_SIMPLE_FAULT_D_L0_MAX ||
              fabsf(leg[LEFT]->d_phi0) > REC_SIMPLE_FAULT_D_PHI0_MAX ||
              fabsf(leg[RIGHT]->d_phi0) > REC_SIMPLE_FAULT_D_PHI0_MAX);

    if (unsafe) {
        recovery_simple_motion_fault_timer += recovery_simple_control_dt;
    } else {
        recovery_simple_motion_fault_timer = 0.0f;
    }

    return recovery_simple_motion_fault_timer >=
           REC_SIMPLE_MOTION_FAULT_DEBOUNCE_TIME;
}

static uint8_t Recovery_Simple_RetractMotionFault(void)
{
    float d_l0_L = leg[LEFT]->d_l0_lpf;
    float d_l0_R = leg[RIGHT]->d_l0_lpf;
    uint8_t unsafe;

    if (recovery_simple_state == REC_SIMPLE_RETRACT &&
        recovery_simple_timer < REC_SIMPLE_RETRACT_MOTION_FAULT_ARM_TIME) {
        recovery_simple_retract_fault_timer = 0.0f;
        return 0;
    }

    if (isnan(d_l0_L) || isinf(d_l0_L)) {
        d_l0_L = leg[LEFT]->d_l0;
    }
    if (isnan(d_l0_R) || isinf(d_l0_R)) {
        d_l0_R = leg[RIGHT]->d_l0;
    }

    /* Compression is commanded in this phase; only reject sustained rebound/runaway motion. */
    unsafe = (d_l0_L > REC_SIMPLE_RETRACT_D_L0_EXTEND_MAX ||
              d_l0_R > REC_SIMPLE_RETRACT_D_L0_EXTEND_MAX ||
              fabsf(leg[LEFT]->d_phi0) > REC_SIMPLE_RETRACT_D_PHI0_MAX ||
              fabsf(leg[RIGHT]->d_phi0) > REC_SIMPLE_RETRACT_D_PHI0_MAX);

    if (unsafe) {
        recovery_simple_retract_fault_timer += recovery_simple_control_dt;
    } else {
        recovery_simple_retract_fault_timer = 0.0f;
    }

    return recovery_simple_retract_fault_timer >= REC_SIMPLE_RETRACT_FAULT_DEBOUNCE_TIME;
}

static uint8_t Recovery_Simple_ForwardZeroMotionFault(void)
{
    float d_l0_L = leg[LEFT]->d_l0_lpf;
    float d_l0_R = leg[RIGHT]->d_l0_lpf;

    if (isnan(d_l0_L) || isinf(d_l0_L)) {
        d_l0_L = leg[LEFT]->d_l0;
    }
    if (isnan(d_l0_R) || isinf(d_l0_R)) {
        d_l0_R = leg[RIGHT]->d_l0;
    }

    /* phase 10 is an intentional over-body swing, so d_phi0 is not a fault signal here. */
    if (fabsf(d_l0_L) > REC_SIMPLE_FORWARD_ZERO_D_L0_MAX ||
        fabsf(d_l0_R) > REC_SIMPLE_FORWARD_ZERO_D_L0_MAX) {
        recovery_simple_forward_zero_motion_timer += recovery_simple_control_dt;
    } else {
        recovery_simple_forward_zero_motion_timer = 0.0f;
    }

    return recovery_simple_forward_zero_motion_timer >=
           REC_SIMPLE_FORWARD_ZERO_MOTION_DEBOUNCE_TIME;
}

/* Preserve the angle-control torque and give leg force only the remaining joint torque. */
static float Recovery_Simple_ForceWithTorqueReserve(leg_obj_t *leg_obj, float force, float Tp, float torque_reserve)
{
    float tp_torque[2] = {0.0f, 0.0f};
    float force_torque[2] = {0.0f, 0.0f};
    float force_scale = 1.0f;

    leg_obj->support_force = 0.0f;
    leg_obj->Tp = Tp;
    leg_obj->vmc_cal_T(leg_obj, tp_torque);

    leg_obj->support_force = force;
    leg_obj->Tp = 0.0f;
    leg_obj->vmc_cal_T(leg_obj, force_torque);

    for (uint8_t i = 0; i < 2; i++) {
        float force_component = force_torque[i];
        float tp_component = tp_torque[i];

        if (fabsf(tp_component) >= torque_reserve) {
            force_scale = 0.0f;
            break;
        }

        if (fabsf(force_component) > 1e-4f) {
            float limit_scale = (force_component > 0.0f) ?
                (torque_reserve - tp_component) / force_component :
                (-torque_reserve - tp_component) / force_component;
            if (limit_scale < force_scale) {
                force_scale = limit_scale;
            }
        }
    }

    LIMIT_MIN_MAX(force_scale, 0.0f, 1.0f);
    return force * force_scale;
}

static float Recovery_Simple_BreakawayForce(float elapsed,
                                            float rise_time,
                                            float total_time,
                                            float peak_force,
                                            float sustain_force)
{
    float progress;

    if (elapsed <= 0.0f) {
        return 0.0f;
    }
    if (elapsed < rise_time) {
        progress = elapsed / rise_time;
        progress = progress * progress * (3.0f - 2.0f * progress);
        return peak_force * progress;
    }
    if (elapsed < total_time && total_time > rise_time) {
        progress = (elapsed - rise_time) / (total_time - rise_time);
        progress = progress * progress * (3.0f - 2.0f * progress);
        return peak_force + progress * (sustain_force - peak_force);
    }
    return sustain_force;
}

static float Recovery_Simple_PhiTargetFromLength(float l0)
{
    float progress = (REC_SIMPLE_RETRACT_PHI0_L0_START - l0) /
                     (REC_SIMPLE_RETRACT_PHI0_L0_START - REC_SIMPLE_RETRACT_PHI0_L0_FINAL);

    LIMIT_MIN_MAX(progress, 0.0f, 1.0f);
    return REC_SIMPLE_PHI0_TARGET + progress *
           (REC_SIMPLE_RETRACT_PHI0_FINAL_TARGET - REC_SIMPLE_PHI0_TARGET);
}

static float Recovery_Simple_TerminalBlend(float l0)
{
    float blend = (REC_SIMPLE_RETRACT_SETTLE_L0_START - l0) /
                  (REC_SIMPLE_RETRACT_SETTLE_L0_START - REC_SIMPLE_RETRACT_PHI0_L0_FINAL);

    LIMIT_MIN_MAX(blend, 0.0f, 1.0f);
    return blend;
}

/* 正置回零方向：返回从当前 phi0 到等价零位(0 mod 2π)的角度增量。
 * over_body=1（任一腿入口 phi0>90°，或入口 pitch<=0）：强制正向回零。
 * over_body=0（两腿均未朝身前且入口 pitch>0）：最短有符号路径回零。 */
static float Recovery_Simple_ZeroDelta(float phi0, uint8_t over_body)
{
    if (over_body) {
        return positive_angle_delta_f32(0.0f, phi0);
    }
    return -wrap_pi_f32(phi0);
}

/* 入口初始姿态判断，只在 REC_SIMPLE_ENTER 调用一次：两腿实测长度都短于
 * REC_SIMPLE_SKIP_ZERO_L0，说明倒地时腿已经收在身下，不必再走"回零"和
 * 预备摆腿，直接进收腿。
 * 这里不设长度下限：DM 反馈缺失时 VMC 在编码器零位算出的 l0 约 0.33 m，天然
 * 落不进本条件；而真正折得很短的腿不该被下限挡住、退回去走回零。
 * 本判断只作用于入口；收腿阶段的交接门槛不受影响。 */
static uint8_t Recovery_Simple_EntryCompact(void)
{
    float l0_L = leg[LEFT]->l0;
    float l0_R = leg[RIGHT]->l0;

    return (isfinite(l0_L) && isfinite(l0_R) &&
            l0_L < REC_SIMPLE_SKIP_ZERO_L0 &&
            l0_R < REC_SIMPLE_SKIP_ZERO_L0) ? 1u : 0u;
}

/* 进入预备摆腿阶段的唯一入口：CHECK_ZERO 正常路径使用。
 * phi0_target 固定传 REC_SIMPLE_RETRACT_PHI0_FINAL_TARGET(80°)，沿用原有标定。 */
static void Recovery_Simple_EnterSwing(float phi0_target)
{
    recovery_simple_l0_ref_L = leg[LEFT]->l0;
    recovery_simple_l0_ref_R = leg[RIGHT]->l0;
    recovery_simple_phi0_ref_L = leg[LEFT]->phi0;
    recovery_simple_phi0_ref_R = leg[RIGHT]->phi0;
    Recovery_SwingAxisCapture(&recovery_swing_axis[LEFT],
                              leg[LEFT]->phi0,
                              leg[LEFT]->phi0,
                              phi0_target);
    Recovery_SwingAxisCapture(&recovery_swing_axis[RIGHT],
                              leg[RIGHT]->phi0,
                              leg[RIGHT]->phi0,
                              phi0_target);
    recovery_simple_stable_timer = 0.0f;
    recovery_simple_angle_gain = 1.0f;
    recovery_simple_force_gain_L = 0.0f;
    recovery_simple_force_gain_R = 0.0f;
    recovery_simple_breakaway_released = 0;
    recovery_simple_motion_fault_timer = 0.0f;
    recovery_simple_retract_fault_timer = 0.0f;
    recovery_lqr_handoff_gain = 0.0f;
    recovery_lqr_handoff_last_active = 0;
    pid_clear(theta_pid);
    pid_clear(roll_pid);
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
    chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
    recovery_simple_state = REC_SIMPLE_SWING;
    recovery_simple_timer = 0.0f;
}

/* 入口紧凑（两腿 l0 都 < REC_SIMPLE_SKIP_ZERO_L0）专用入口：跳过回零，直接进
 * 收腿，终点由 RETRACT 自己给出（l0 -> 0.13 m，phi0 -> 80°）。
 * 不能再经过 SWING：SWING 的退出判据是"到 30°±4°"，折叠姿态往往已经在 30° 以上，
 * 于是先把腿摆到 30°、再让收腿把它拉回 80°，白跑一趟；而收腿末端角度环只有
 * ~10 N.m，拉不回 80°，就永远进不了 phi0_err<10° 的交接稳定窗（实测卡死在
 * 36°/0.153 m 近 3 s）。直接进 RETRACT 则长度环和气弹簧前馈立刻工作，不会弹腿。 */
static void Recovery_Simple_EnterRetractCompact(void)
{
    recovery_simple_compact_entry = 1u;
    recovery_simple_l0_ref_L = leg[LEFT]->l0;
    recovery_simple_l0_ref_R = leg[RIGHT]->l0;
    recovery_simple_phi0_ref_L = leg[LEFT]->phi0;
    recovery_simple_phi0_ref_R = leg[RIGHT]->phi0;
    recovery_simple_stable_timer = 0.0f;
    recovery_simple_angle_gain = 1.0f;
    recovery_simple_force_gain_L = 0.0f;
    recovery_simple_force_gain_R = 0.0f;
    recovery_simple_breakaway_released = 0;
    recovery_simple_motion_fault_timer = 0.0f;
    recovery_simple_retract_fault_timer = 0.0f;
    recovery_lqr_handoff_gain = 0.0f;
    recovery_lqr_handoff_last_active = 0;
    pid_clear(theta_pid);
    pid_clear(roll_pid);
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
    chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
    recovery_simple_state = REC_SIMPLE_RETRACT;
    recovery_simple_timer = 0.0f;
}

/* 进入 LQR 准备阶段的唯一入口：收腿稳定窗、收腿超时、入口紧凑直接交接共用，
 * 保证三条路径的清场动作完全一致。 */
static void Recovery_Simple_EnterLqrPrep(void)
{
    recovery_simple_state = REC_SIMPLE_LQR_PREP;
    recovery_simple_timer = 0.0f;
    recovery_simple_stable_timer = 0.0f;
    recovery_simple_handoff_stable_timer = 0.0f;
    pid_clear(theta_pid);
    pid_clear(roll_pid);
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
}

/* Enter the established upright-side recovery path without re-running HOLD. */
static void Recovery_Simple_EnterForwardZero(void)
{
    float phi0_wrapped_L = wrap_pi_f32(leg[LEFT]->phi0);
    float phi0_wrapped_R = wrap_pi_f32(leg[RIGHT]->phi0);
    uint8_t left_near_zero =
        fabsf(phi0_wrapped_L) < REC_SIMPLE_FORWARD_ZERO_PHI0_OK;
    uint8_t right_near_zero =
        fabsf(phi0_wrapped_R) < REC_SIMPLE_FORWARD_ZERO_PHI0_OK;
    /* 方向已由 Recovery_Pose_ResetOnEnter() 在入口优先按腿角、再按 pitch 锁存。
     * 这里只读取，不重新判断——
     * 倒置缓和结束后再进本函数时也必须沿用入口方向。 */
    recovery_simple_l0_ref_L = leg[LEFT]->l0;
    recovery_simple_l0_ref_R = leg[RIGHT]->l0;
    recovery_simple_phi0_ref_L = 0.0f;
    recovery_simple_phi0_ref_R = 0.0f;
    /* 已经在零位容差内的一侧保持当前位置，不为了统一方向再绕一整圈。 */
    recovery_simple_forward_zero_target_L = left_near_zero ?
        leg[LEFT]->phi0 :
        leg[LEFT]->phi0 + Recovery_Simple_ZeroDelta(leg[LEFT]->phi0,
                                                    recovery_simple_over_body);
    recovery_simple_forward_zero_target_R = right_near_zero ?
        leg[RIGHT]->phi0 :
        leg[RIGHT]->phi0 + Recovery_Simple_ZeroDelta(leg[RIGHT]->phi0,
                                                     recovery_simple_over_body);
    Recovery_SwingAxisCapture(&recovery_swing_axis[LEFT],
                              leg[LEFT]->phi0,
                              leg[LEFT]->phi0,
                              recovery_simple_forward_zero_target_L);
    Recovery_SwingAxisCapture(&recovery_swing_axis[RIGHT],
                              leg[RIGHT]->phi0,
                              leg[RIGHT]->phi0,
                              recovery_simple_forward_zero_target_R);
    recovery_simple_motion_fault_timer = 0.0f;
    recovery_simple_forward_zero_motion_timer = 0.0f;
    recovery_simple_forward_zero_stable_timer = 0.0f;
    dbg_forward_zero_fault_reason = 0;
    recovery_simple_angle_gain = 1.0f;
    recovery_simple_timer = 0.0f;
    chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
    recovery_simple_state = REC_SIMPLE_FORWARD_ZERO;
}

static void Recovery_Simple_StateMachine(float dt)
{
    dbg_retract_state = (float)recovery_simple_state;
    switch (recovery_simple_state) {
    case REC_SIMPLE_ENTER:
        Recovery_SeedCurrentPose();
        recovery_simple_timer = 0.0f;
        /* 入口只判断一次初始姿态：两腿都已经收在身下就跳过回零和预备摆腿，
         * 直接进收腿（终点 l0=0.13 m / phi0=80°）→ LQR 交接；否则照常从 HOLD
         * 走完整流程。 */
        if (Recovery_Simple_EntryCompact()) {
            recovery_simple_skip_zero_fired = 1;
            Recovery_Simple_EnterRetractCompact();
            break;
        }
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY;
        recovery_simple_state = REC_SIMPLE_HOLD;
        break;

    case REC_SIMPLE_HOLD:
        recovery_simple_timer += dt;
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY;

        if (fabsf(ins.pitch) > REC_SIMPLE_FORWARD_PITCH_MAX &&
            fabsf(ins.pitch) < REC_SIMPLE_INVERTED_ENTER_PITCH_MIN) {
            dbg_simple_fault_reason = 11;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 12;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (Recovery_Simple_MotionFault()) {
            dbg_simple_fault_reason = 13;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }

        if (recovery_simple_timer >= REC_SIMPLE_HOLD_TIME &&
            fabsf(ins.pitch) >= REC_SIMPLE_INVERTED_ENTER_PITCH_MIN) {
            recovery_simple_inverted_entry_pitch = ins.pitch;
            recovery_simple_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_INVERTED_CAPTURE;
            break;
        }

        if (recovery_simple_timer >= REC_SIMPLE_HOLD_TIME &&
            !(leg[LEFT]->l0 > 0.30f && leg[RIGHT]->l0 > 0.30f &&
              fabsf(leg[LEFT]->phi0) < 0.70f && fabsf(leg[RIGHT]->phi0) < 0.70f)) {
            Recovery_Simple_EnterForwardZero();
            break;
        }

        if (recovery_simple_timer >= REC_SIMPLE_HOLD_TIME) {
            recovery_simple_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_CHECK_ZERO;
        }
        break;

    case REC_SIMPLE_INVERTED_CAPTURE:
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 112;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer >= REC_SIMPLE_INVERTED_CAPTURE_TIME) {
            recovery_simple_l0_ref_L = leg[LEFT]->l0;
            recovery_simple_l0_ref_R = leg[RIGHT]->l0;
            recovery_simple_phi0_ref_L = leg[LEFT]->phi0;
            recovery_simple_phi0_ref_R = leg[RIGHT]->phi0;
            recovery_simple_angle_gain = 0.0f;
            recovery_simple_force_gain_L = 0.0f;
            recovery_simple_force_gain_R = 0.0f;
            recovery_simple_motion_fault_timer = 0.0f;
            recovery_simple_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_INVERTED_PRELOAD;
        }
        break;

    case REC_SIMPLE_INVERTED_PRELOAD:
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 122;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (Recovery_Simple_MotionFault()) {
            dbg_simple_fault_reason = 123;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer >= REC_SIMPLE_INVERTED_PRELOAD_TIME) {
            float right_phi0_rel;
            float negative_lead_phi0;

            /* Calibrated direction: both legs turn toward negative phi0. */
            recovery_simple_inverted_turn_sign_L = -1;
            recovery_simple_inverted_turn_sign_R = -1;
            /* Place both legs on one unwrap branch before comparing them. */
            right_phi0_rel = wrap_pi_f32(leg[RIGHT]->phi0 - leg[LEFT]->phi0);
            recovery_simple_inverted_right_unwrapped =
                leg[LEFT]->phi0 + right_phi0_rel;
            negative_lead_phi0 =
                (recovery_simple_inverted_right_unwrapped < leg[LEFT]->phi0) ?
                recovery_simple_inverted_right_unwrapped : leg[LEFT]->phi0;
            /* 完全翻倒时两条腿可能相差半圈（实测 phi_L=+81.5°, phi_R=-83.9°）。
             * 旧代码直接给两腿同一个终点就开始转，于是整段翻身两腿始终差 150° 以上，
             * 等待腿只压参考速度、在重力下拦不住。先对齐到同一条腿（取 negative
             * lead，只动落后那一侧），再开始同步翻转。 */
            recovery_simple_inverted_align_target = negative_lead_phi0;
            Recovery_SwingAxisCapture(&recovery_swing_axis[LEFT],
                                      leg[LEFT]->phi0,
                                      leg[LEFT]->phi0,
                                      negative_lead_phi0);
            Recovery_SwingAxisCapture(&recovery_swing_axis[RIGHT],
                                      leg[RIGHT]->phi0,
                                      recovery_simple_inverted_right_unwrapped,
                                      negative_lead_phi0);
            recovery_simple_phi0_ref_L = leg[LEFT]->phi0;
            recovery_simple_phi0_ref_R = recovery_simple_inverted_right_unwrapped;
            recovery_simple_angle_gain = 1.0f;
            recovery_simple_motion_fault_timer = 0.0f;
            recovery_simple_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_INVERTED_ALIGN;
        }
        break;

    case REC_SIMPLE_INVERTED_ALIGN:
    {
        float align_err_L = angle_err_abs_f32(recovery_swing_axis[LEFT].actual,
                                              recovery_simple_inverted_align_target);
        float align_err_R = angle_err_abs_f32(recovery_swing_axis[RIGHT].actual,
                                              recovery_simple_inverted_align_target);
        uint8_t aligned = (align_err_L < REC_SIMPLE_INVERTED_ALIGN_OK) &&
                          (align_err_R < REC_SIMPLE_INVERTED_ALIGN_OK);

        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 132;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (Recovery_Simple_MotionFault()) {
            dbg_simple_fault_reason = 133;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }

        if (aligned || recovery_simple_timer > REC_SIMPLE_INVERTED_ALIGN_TIMEOUT) {
            float symmetric_turn_target;

            /* 对齐超时不干等：照旧开始翻身，行为不比原来差，原因记在故障码 134。 */
            dbg_simple_fault_reason = aligned ? 0 : 134;
            symmetric_turn_target = recovery_simple_inverted_align_target +
                recovery_simple_inverted_turn_sign_L * REC_SIMPLE_INVERTED_TURN_MAX_PHI_TRAVEL;
            recovery_simple_inverted_phi_target_L = symmetric_turn_target;
            recovery_simple_inverted_phi_target_R = symmetric_turn_target;
            recovery_simple_phi0_ref_L = recovery_swing_axis[LEFT].actual;
            recovery_simple_phi0_ref_R = recovery_swing_axis[RIGHT].actual;
            /* 用对齐后的展开角重新捕获，保持连续性。 */
            Recovery_SwingAxisCapture(&recovery_swing_axis[LEFT],
                                      leg[LEFT]->phi0,
                                      recovery_swing_axis[LEFT].actual,
                                      symmetric_turn_target);
            Recovery_SwingAxisCapture(&recovery_swing_axis[RIGHT],
                                      leg[RIGHT]->phi0,
                                      recovery_swing_axis[RIGHT].actual,
                                      symmetric_turn_target);
            recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
            recovery_simple_motion_fault_timer = 0.0f;
            recovery_simple_inverted_best_abs_pitch = fabsf(ins.pitch);
            recovery_simple_inverted_pitch_stall_timer = 0.0f;
            recovery_simple_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_INVERTED_TURN;
        }
        break;
    }

    case REC_SIMPLE_INVERTED_TURN:
    {
        float abs_pitch = fabsf(ins.pitch);

        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (abs_pitch <= REC_SIMPLE_INVERTED_EXIT_PITCH_MAX) {
            recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
            recovery_simple_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_INVERTED_SETTLE;
            break;
        }
        if (abs_pitch < recovery_simple_inverted_best_abs_pitch -
                            REC_SIMPLE_INVERTED_PITCH_PROGRESS_EPS) {
            recovery_simple_inverted_best_abs_pitch = abs_pitch;
            recovery_simple_inverted_pitch_stall_timer = 0.0f;
        } else if (recovery_simple_timer > REC_SIMPLE_INVERTED_PITCH_STALL_ARM_TIME) {
            recovery_simple_inverted_pitch_stall_timer += dt;
        }
        if (recovery_simple_inverted_pitch_stall_timer >
            REC_SIMPLE_INVERTED_PITCH_STALL_TIME) {
            dbg_simple_fault_reason = 145;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 142;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (Recovery_Simple_MotionFault()) {
            dbg_simple_fault_reason = 143;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer > REC_SIMPLE_INVERTED_TURN_TIMEOUT) {
            dbg_simple_fault_reason = 144;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        }
        break;
    }

    case REC_SIMPLE_INVERTED_SETTLE:
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (fabsf(ins.pitch) > REC_SIMPLE_FORWARD_PITCH_MAX) {
            dbg_simple_fault_reason = 151;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer >= REC_SIMPLE_INVERTED_SETTLE_TIME) {
            recovery_simple_inverted_turn_sign_L = 0;
            recovery_simple_inverted_turn_sign_R = 0;
            Recovery_Simple_EnterForwardZero();
        }
        break;

    case REC_SIMPLE_FORWARD_ZERO:
    {
        float phi0_err_L = wrap_pi_f32(-leg[LEFT]->phi0);
        float phi0_err_R = wrap_pi_f32(-leg[RIGHT]->phi0);
        uint8_t phi_ok =
            (fabsf(phi0_err_L) < REC_SIMPLE_FORWARD_ZERO_PHI0_OK) &&
            (fabsf(phi0_err_R) < REC_SIMPLE_FORWARD_ZERO_PHI0_OK);

        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;

        if (fabsf(ins.pitch) > REC_SIMPLE_FORWARD_PITCH_MAX) {
            /* A failed jump can enter Recovery while the body is still
             * tipping.  Stopping all hip control at 80 deg left it to fall
             * inverted and required another manual enable.  Reuse the
             * existing inverted capture/turn path automatically instead. */
            recovery_simple_inverted_entry_pitch = ins.pitch;
            recovery_simple_angle_gain = 0.0f;
            recovery_simple_motion_fault_timer = 0.0f;
            recovery_simple_inverted_pitch_stall_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_INVERTED_CAPTURE;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_forward_zero_fault_reason = 2;
            dbg_simple_fault_reason = 102;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (Recovery_Simple_ForwardZeroMotionFault()) {
            dbg_forward_zero_fault_reason = 3;
            dbg_simple_fault_reason = 103;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer > REC_SIMPLE_FORWARD_ZERO_TIMEOUT) {
            dbg_forward_zero_fault_reason = 4;
            dbg_simple_fault_reason = 104;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }

        uint8_t both_legs_zero =
            phi_ok &&
            leg[LEFT]->l0 > 0.30f && leg[RIGHT]->l0 > 0.30f &&
            fabsf(ins.pitch) < REC_SIMPLE_ZERO_PITCH_MAX;

        if (both_legs_zero) {
            recovery_simple_forward_zero_stable_timer += dt;
        } else {
            recovery_simple_forward_zero_stable_timer = 0.0f;
        }

        if (recovery_simple_forward_zero_stable_timer >=
            REC_SIMPLE_FORWARD_ZERO_STABLE_TIME) {
            recovery_simple_state = REC_SIMPLE_CHECK_ZERO;
            recovery_simple_timer = 0.0f;
            recovery_simple_forward_zero_stable_timer = 0.0f;
        }
        break;
    }

    case REC_SIMPLE_CHECK_ZERO:
        recovery_simple_zero_ok = Recovery_IsZeroPose();
        if (!recovery_simple_zero_ok || fabsf(ins.pitch) > REC_SIMPLE_ZERO_PITCH_MAX) {
            dbg_simple_fault_reason = 23;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }

        Recovery_Simple_EnterSwing(REC_SIMPLE_RETRACT_PHI0_FINAL_TARGET);
        break;

    case REC_SIMPLE_SWING:
    {
        float prepose_err_L = recovery_swing_axis[LEFT].direction *
            (REC_SIMPLE_PHI0_TARGET - recovery_swing_axis[LEFT].actual);
        float prepose_err_R = recovery_swing_axis[RIGHT].direction *
            (REC_SIMPLE_PHI0_TARGET - recovery_swing_axis[RIGHT].actual);
        uint8_t retract_entry_ok =
            (prepose_err_L <= REC_SIMPLE_SWING_PHI0_OK) &&
            (prepose_err_R <= REC_SIMPLE_SWING_PHI0_OK);

        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;

        if (fabsf(ins.pitch) > REC_SIMPLE_FAULT_PITCH_MAX) {
            dbg_simple_fault_reason = 31;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            recovery_simple_stable_timer = 0.0f;
            break;
        }
        if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
            leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 32;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            recovery_simple_stable_timer = 0.0f;
            break;
        }
        if (Recovery_Simple_MotionFault()) {
            dbg_simple_fault_reason = 33;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            recovery_simple_stable_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer > REC_SIMPLE_SWING_TIMEOUT) {
            dbg_simple_fault_reason = 34;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            recovery_simple_stable_timer = 0.0f;
            break;
        }

        if (retract_entry_ok) {
            recovery_simple_state = REC_SIMPLE_RETRACT;
            recovery_simple_timer = 0.0f;
            recovery_simple_stable_timer = 0.0f;
            if (recovery_simple_angle_gain < REC_SIMPLE_RETRACT_TP_GAIN_INIT) {
                recovery_simple_angle_gain = REC_SIMPLE_RETRACT_TP_GAIN_INIT;
            }
            recovery_simple_force_gain_L = 0.0f;
            recovery_simple_force_gain_R = 0.0f;
            recovery_simple_breakaway_released = 0;
            recovery_simple_motion_fault_timer = 0.0f;
            recovery_simple_retract_fault_timer = 0.0f;
        }
        break;
    }

    case REC_SIMPLE_RETRACT:
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (fabsf(ins.pitch) > REC_SIMPLE_FAULT_PITCH_MAX) {
            dbg_simple_fault_reason = 41;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        } else if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
                   leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 42;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        } else if (Recovery_Simple_RetractMotionFault()) {
            dbg_simple_fault_reason = 43;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        } else if (leg[LEFT]->phi0 < REC_SIMPLE_RETRACT_UNDERBODY_GUARD ||
                   leg[RIGHT]->phi0 < REC_SIMPLE_RETRACT_UNDERBODY_GUARD) {
            /* Stop the shortening force and restore both legs to the swing pose first. */
            recovery_simple_state = REC_SIMPLE_SWING;
            recovery_simple_timer = 0.0f;
            recovery_simple_stable_timer = 0.0f;
            Recovery_SwingAxisCapture(&recovery_swing_axis[LEFT],
                                      leg[LEFT]->phi0,
                                      leg[LEFT]->phi0,
                                      REC_SIMPLE_RETRACT_PHI0_FINAL_TARGET);
            Recovery_SwingAxisCapture(&recovery_swing_axis[RIGHT],
                                      leg[RIGHT]->phi0,
                                      leg[RIGHT]->phi0,
                                      REC_SIMPLE_RETRACT_PHI0_FINAL_TARGET);
            recovery_simple_angle_gain = 1.0f;
            recovery_simple_force_gain_L = 0.0f;
            recovery_simple_force_gain_R = 0.0f;
            recovery_simple_breakaway_released = 0;
            recovery_simple_motion_fault_timer = 0.0f;
            recovery_simple_retract_fault_timer = 0.0f;
        } else if (recovery_simple_timer > REC_SIMPLE_RETRACT_TIMEOUT) {
            /* A leg can mechanically stall before the stable gate. Keep the
             * current compact pose and let the low-leg LQR finish the stand. */
            Recovery_Simple_EnterLqrPrep();
        }
        break;

    case REC_SIMPLE_RETRACT_HOLD:
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (fabsf(ins.pitch) > REC_SIMPLE_FAULT_PITCH_MAX) {
            dbg_simple_fault_reason = 51;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        } else if (leg[LEFT]->l0 < REC_SIMPLE_FAULT_L0_MIN ||
                   leg[RIGHT]->l0 < REC_SIMPLE_FAULT_L0_MIN) {
            dbg_simple_fault_reason = 52;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        } else if (Recovery_Simple_RetractMotionFault()) {
            dbg_simple_fault_reason = 53;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
        }
        break;

    case REC_SIMPLE_LQR_PREP:
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY_READY;
        recovery_simple_timer += dt;
        if (Recovery_Simple_RetractMotionFault()) {
            dbg_simple_fault_reason = 63;
            recovery_simple_state = REC_SIMPLE_FAULT;
            recovery_simple_timer = 0.0f;
            break;
        }
        if (recovery_simple_timer >= REC_SIMPLE_LQR_PREP_TIME) {
            /* Retraction ends with a negative axial force. Do not spend the
             * first part of handoff blending through that force before LQR
             * can support and raise the body. */
            recovery_simple_handoff_F_L = dbg_retract_F_L > 0.0f ?
                dbg_retract_F_L : 0.0f;
            recovery_simple_handoff_F_R = dbg_retract_F_R > 0.0f ?
                dbg_retract_F_R : 0.0f;
            recovery_simple_handoff_Tp_L = dbg_retract_Tp_L;
            recovery_simple_handoff_Tp_R = dbg_retract_Tp_R;
            /* Freeze each length loop at the final retract pose before LQR takes over. */
            recovery_simple_lqr_length_hold_L = leg[LEFT]->l0;
            recovery_simple_lqr_length_hold_R = leg[RIGHT]->l0;
            recovery_simple_lqr_length_hold_timer = 0.0f;
            recovery_simple_lqr_length_hold_start_ms = dwt_get_time_ms();
            recovery_simple_lqr_length_hold_active = 1;
            recovery_lqr_handoff_gain = 0.0f;
            recovery_lqr_handoff_last_active = 0;
            recovery_simple_handoff_stable_timer = 0.0f;
            recovery_simple_state = REC_SIMPLE_LQR_HANDOFF;
            recovery_simple_timer = 0.0f;
            chassis_fdb_data.stand_state = CHASSIS_LEG_BACK_IS_OK;
        }
        break;

    case REC_SIMPLE_LQR_HANDOFF: {
        uint8_t length_rate_ok;

        chassis_fdb_data.stand_state = CHASSIS_LEG_BACK_IS_OK;
        recovery_simple_timer += dt;

        length_rate_ok = fabsf(leg[LEFT]->d_l0_lpf) < REC_SIMPLE_HANDOFF_D_L0_OK &&
                         fabsf(leg[RIGHT]->d_l0_lpf) < REC_SIMPLE_HANDOFF_D_L0_OK;

        if (recovery_lqr_handoff_gain >= 0.99f && length_rate_ok) {
            recovery_simple_handoff_stable_timer += dt;
        } else {
            recovery_simple_handoff_stable_timer = 0.0f;
        }

        if (recovery_simple_handoff_stable_timer >= REC_SIMPLE_HANDOFF_STABLE_TIME) {
            chassis_fdb_data.stand_state = CHASSIS_IS_STAND;
            recovery_simple_state = REC_SIMPLE_DONE;
        }
        break;
    }

    case REC_SIMPLE_DONE:
        chassis_fdb_data.stand_state = CHASSIS_IS_STAND;
        break;

    case REC_SIMPLE_FAULT:
    default:
        leg[LEFT]->support_force = 0.0f;
        leg[RIGHT]->support_force = 0.0f;
        leg[LEFT]->Tp = 0.0f;
        leg[RIGHT]->Tp = 0.0f;
        chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY;
        break;
    }
    dbg_retract_state = (float)recovery_simple_state;
}

static void Recovery_Simple_RetractControl(void)
{
    float dt;
    float swing_dt = recovery_simple_control_dt;
    int8_t inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
    float inverted_speed_L = REC_SIMPLE_INVERTED_TURN_PHI_RATE;
    float inverted_speed_R = REC_SIMPLE_INVERTED_TURN_PHI_RATE;

    uint8_t forward_zero_active = (recovery_simple_state == REC_SIMPLE_FORWARD_ZERO);
    uint8_t swing_active = (recovery_simple_state == REC_SIMPLE_SWING);
    uint8_t retract_swing_active = (recovery_simple_state == REC_SIMPLE_RETRACT);
    uint8_t forward_swing_motion_active = swing_active || retract_swing_active;
    uint8_t inverted_preload_active = (recovery_simple_state == REC_SIMPLE_INVERTED_PRELOAD);
    uint8_t inverted_align_active = (recovery_simple_state == REC_SIMPLE_INVERTED_ALIGN);
    uint8_t inverted_turn_active = (recovery_simple_state == REC_SIMPLE_INVERTED_TURN);
    uint8_t inverted_settle_active = (recovery_simple_state == REC_SIMPLE_INVERTED_SETTLE);
    uint8_t inverted_active = inverted_preload_active || inverted_align_active ||
                              inverted_turn_active || inverted_settle_active;
    uint8_t angle_only_active = forward_zero_active || swing_active ||
                                inverted_align_active ||
                                inverted_turn_active || inverted_settle_active;
    uint8_t swing_motion_active = angle_only_active || retract_swing_active;
    uint8_t retract_active =
        (recovery_simple_state == REC_SIMPLE_RETRACT ||
         recovery_simple_state == REC_SIMPLE_RETRACT_HOLD ||
         recovery_simple_state == REC_SIMPLE_LQR_PREP);
    float breakaway_scale_L = (leg[LEFT]->l0 -
        (REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_L0 -
         REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_BAND)) /
        REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_BAND;
    float breakaway_scale_R = (leg[RIGHT]->l0 -
        (REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_L0 -
         REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_BAND)) /
        REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_BAND;

    LIMIT_MIN_MAX(breakaway_scale_L, 0.0f, 1.0f);
    LIMIT_MIN_MAX(breakaway_scale_R, 0.0f, 1.0f);
    breakaway_scale_L = breakaway_scale_L * breakaway_scale_L *
        (3.0f - 2.0f * breakaway_scale_L);
    breakaway_scale_R = breakaway_scale_R * breakaway_scale_R *
        (3.0f - 2.0f * breakaway_scale_R);

    if (recovery_simple_state != REC_SIMPLE_RETRACT ||
        recovery_simple_timer >= REC_SIMPLE_RETRACT_BREAKAWAY_TIME_L) {
        breakaway_scale_L = 0.0f;
    } else if (recovery_simple_timer < REC_SIMPLE_RETRACT_BREAKAWAY_MIN_TIME_L) {
        breakaway_scale_L = 1.0f;
    }
    if (recovery_simple_state != REC_SIMPLE_RETRACT ||
        recovery_simple_timer >= REC_SIMPLE_RETRACT_BREAKAWAY_TIME) {
        breakaway_scale_R = 0.0f;
    } else if (recovery_simple_timer < REC_SIMPLE_RETRACT_BREAKAWAY_MIN_TIME) {
        breakaway_scale_R = 1.0f;
    }

    uint8_t left_breakaway_active = breakaway_scale_L > 0.001f;
    uint8_t right_breakaway_active = breakaway_scale_R > 0.001f;

    /* 正置摆腿/收腿使用实测周期，倒置自救维持已经回归验证过的时基。 */
    dt = (swing_active || retract_active) ?
        recovery_simple_control_dt : chassis_dt;

    if (swing_motion_active) {
        Recovery_SwingAxisUpdateActual(&recovery_swing_axis[LEFT], leg[LEFT]->phi0, swing_dt);
        Recovery_SwingAxisUpdateActual(&recovery_swing_axis[RIGHT], leg[RIGHT]->phi0, swing_dt);
    }

    if (inverted_turn_active) {
        float progress_L = recovery_swing_axis[LEFT].direction *
            (recovery_swing_axis[LEFT].actual - recovery_swing_axis[LEFT].start_actual);
        float progress_R = recovery_swing_axis[RIGHT].direction *
            (recovery_swing_axis[RIGHT].actual - recovery_swing_axis[RIGHT].start_actual);
        float sync_correction;

        /* Positive means the left leg is leading in the commanded turn direction. */
        dbg_swing_sync_error = progress_L - progress_R;
        sync_correction = REC_SIMPLE_INVERTED_SYNC_KP * fabsf(dbg_swing_sync_error);
        LIMIT_MIN_MAX(sync_correction, 0.0f, REC_SIMPLE_INVERTED_SYNC_CORR_LIMIT);
        if (dbg_swing_sync_error > 0.0f) {
            inverted_speed_L -= sync_correction;
        } else {
            inverted_speed_R -= sync_correction;
        }

        if (recovery_simple_inverted_wait_leg == REC_SIMPLE_INVERTED_WAIT_LEFT) {
            if (dbg_swing_sync_error <= REC_SIMPLE_INVERTED_WAIT_LEG_EXIT) {
                recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
            }
        } else if (recovery_simple_inverted_wait_leg == REC_SIMPLE_INVERTED_WAIT_RIGHT) {
            if (dbg_swing_sync_error >= -REC_SIMPLE_INVERTED_WAIT_LEG_EXIT) {
                recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_NONE;
            }
        }
        if (recovery_simple_inverted_wait_leg == REC_SIMPLE_INVERTED_WAIT_NONE) {
            if (dbg_swing_sync_error >= REC_SIMPLE_INVERTED_WAIT_LEG_ENTER) {
                recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_LEFT;
            } else if (dbg_swing_sync_error <= -REC_SIMPLE_INVERTED_WAIT_LEG_ENTER) {
                recovery_simple_inverted_wait_leg = REC_SIMPLE_INVERTED_WAIT_RIGHT;
            }
        }
        inverted_wait_leg = recovery_simple_inverted_wait_leg;
        if (inverted_wait_leg == REC_SIMPLE_INVERTED_WAIT_LEFT) {
            inverted_speed_L = 0.0f;
        } else if (inverted_wait_leg == REC_SIMPLE_INVERTED_WAIT_RIGHT) {
            inverted_speed_R = 0.0f;
        }
    } else {
        dbg_swing_sync_error = 0.0f;
    }

    if (forward_zero_active) {
        /* Reach zero together.  Driving both sides at the same absolute rate
         * makes the shorter-travel leg stop first; jump18 recorded the other
         * leg continuing alone while Pitch ran to 80 deg.  Scale speed and
         * acceleration by captured travel so both references have the same
         * nominal completion time. */
        const float travel_L = fabsf(recovery_swing_axis[LEFT].target -
                                     recovery_swing_axis[LEFT].start_actual);
        const float travel_R = fabsf(recovery_swing_axis[RIGHT].target -
                                     recovery_swing_axis[RIGHT].start_actual);
        const float max_travel = fmaxf(travel_L, travel_R);
        float scale_L = 0.0f;
        float scale_R = 0.0f;
        if (max_travel > 1e-4f) {
            scale_L = travel_L / max_travel;
            scale_R = travel_R / max_travel;
        }
        Recovery_SwingAxisRun(&recovery_swing_axis[LEFT],
                              REC_SIMPLE_FORWARD_ZERO_REF_RATE * scale_L,
                              REC_SIMPLE_FORWARD_ZERO_REF_ACCEL * scale_L,
                              swing_dt);
        Recovery_SwingAxisRun(&recovery_swing_axis[RIGHT],
                              REC_SIMPLE_FORWARD_ZERO_REF_RATE * scale_R,
                              REC_SIMPLE_FORWARD_ZERO_REF_ACCEL * scale_R,
                              swing_dt);
    } else if (forward_swing_motion_active) {
        float phi0_rate = retract_swing_active ?
            REC_SIMPLE_RETRACT_SWING_PHI0_RATE : REC_SIMPLE_SWING_PHI0_RATE;
        float phi0_accel = REC_SIMPLE_SWING_PHI0_ACCEL;

        if (retract_swing_active &&
            (leg[LEFT]->l0 > REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_L0 ||
             leg[RIGHT]->l0 > REC_SIMPLE_RETRACT_BREAKAWAY_RELEASE_L0)) {
            /* Keep moving, but prioritize compression while either leg is at the dead point. */
            phi0_rate = REC_SIMPLE_RETRACT_DEADPOINT_PHI0_RATE;
        } else if (retract_swing_active &&
                   leg[LEFT]->l0 < REC_SIMPLE_RETRACT_COMPACT_L0 &&
                   leg[RIGHT]->l0 < REC_SIMPLE_RETRACT_COMPACT_L0) {
            /* Short legs have enough clearance to finish the swing together with retraction. */
            phi0_rate = REC_SIMPLE_RETRACT_COMPACT_PHI0_RATE;
            phi0_accel = REC_SIMPLE_RETRACT_COMPACT_PHI0_ACCEL;
        }

        Recovery_SwingAxisRun(&recovery_swing_axis[LEFT],
                              phi0_rate,
                              phi0_accel,
                              swing_dt);
        Recovery_SwingAxisRun(&recovery_swing_axis[RIGHT],
                              phi0_rate,
                              phi0_accel,
                              swing_dt);
    } else if (inverted_align_active) {
        /* 对齐段：两腿都以同样的速率收敛到共同终点，不做同步等待。 */
        Recovery_SwingAxisRun(&recovery_swing_axis[LEFT],
                              REC_SIMPLE_INVERTED_TURN_PHI_RATE,
                              REC_SIMPLE_INVERTED_TURN_PHI_ACCEL,
                              swing_dt);
        Recovery_SwingAxisRun(&recovery_swing_axis[RIGHT],
                              REC_SIMPLE_INVERTED_TURN_PHI_RATE,
                              REC_SIMPLE_INVERTED_TURN_PHI_ACCEL,
                              swing_dt);
    } else if (inverted_turn_active) {
        Recovery_SwingAxisRun(&recovery_swing_axis[LEFT],
                              inverted_speed_L,
                              REC_SIMPLE_INVERTED_TURN_PHI_ACCEL,
                              swing_dt);
        Recovery_SwingAxisRun(&recovery_swing_axis[RIGHT],
                              inverted_speed_R,
                              REC_SIMPLE_INVERTED_TURN_PHI_ACCEL,
                              swing_dt);
    } else if (inverted_settle_active) {
        Recovery_SwingAxisBrake(&recovery_swing_axis[LEFT],
                                REC_SIMPLE_INVERTED_TURN_PHI_ACCEL,
                                swing_dt);
        Recovery_SwingAxisBrake(&recovery_swing_axis[RIGHT],
                                REC_SIMPLE_INVERTED_TURN_PHI_ACCEL,
                                swing_dt);
    }

    if (swing_motion_active) {
        recovery_simple_phi0_ref_L = recovery_swing_axis[LEFT].ref;
        recovery_simple_phi0_ref_R = recovery_swing_axis[RIGHT].ref;
    }

    if (!angle_only_active && !retract_active && !inverted_active) {
        leg[LEFT]->support_force = 0.0f;
        leg[RIGHT]->support_force = 0.0f;
        leg[LEFT]->Tp = 0.0f;
        leg[RIGHT]->Tp = 0.0f;
        leg[LEFT]->vmc_cal_T(leg[LEFT], vmc_out_l);
        leg[RIGHT]->vmc_cal_T(leg[RIGHT], vmc_out_r);
        return;
    }
    if (angle_only_active || retract_active) {
        float angle_gain_rate = forward_zero_active ? REC_SIMPLE_FORWARD_ZERO_GAIN_RATE :
            (swing_active ? REC_SIMPLE_SWING_GAIN_RATE :
             (retract_active ? REC_SIMPLE_RETRACT_TP_GAIN_RATE :
              REC_SIMPLE_INVERTED_TURN_GAIN_RATE));
        recovery_simple_angle_gain += angle_gain_rate * dt;
        if (recovery_simple_angle_gain > 1.0f) {
            recovery_simple_angle_gain = 1.0f;
        }
    }
    if (!angle_only_active) {
        recovery_simple_force_gain_L += REC_SIMPLE_RETRACT_F_GAIN_RATE_L * dt;
        recovery_simple_force_gain_R += REC_SIMPLE_RETRACT_F_GAIN_RATE_R * dt;
        if (recovery_simple_force_gain_L > 1.0f) {
            recovery_simple_force_gain_L = 1.0f;
        }
        if (recovery_simple_force_gain_R > 1.0f) {
            recovery_simple_force_gain_R = 1.0f;
        }
    }

    if (retract_active) {
        recovery_simple_l0_ref_L = slope_to_target_f32(
            recovery_simple_l0_ref_L,
            REC_SIMPLE_RETRACT_L0_TARGET,
            REC_SIMPLE_RETRACT_L0_RATE * dt);

        /* Retraction is one-way: never let the length loop pull a compressed leg back out. */
        if (recovery_simple_l0_ref_L > leg[LEFT]->l0) {
            recovery_simple_l0_ref_L = leg[LEFT]->l0;
        }
        if (recovery_simple_l0_ref_R > leg[RIGHT]->l0) {
            recovery_simple_l0_ref_R = leg[RIGHT]->l0;
        }
        recovery_simple_l0_ref_R = slope_to_target_f32(
            recovery_simple_l0_ref_R,
            REC_SIMPLE_RETRACT_L0_TARGET,
            REC_SIMPLE_RETRACT_L0_RATE * dt);
    }
    if (!recovery_simple_breakaway_released &&
        !left_breakaway_active && !right_breakaway_active && retract_active) {
        /* 爆发段退出时保持当前压缩方向，并无跳变切回持续收腿力。 */
        if (leg[LEFT]->l0 < recovery_simple_l0_ref_L) {
            recovery_simple_l0_ref_L = leg[LEFT]->l0;
        }
        if (leg[RIGHT]->l0 < recovery_simple_l0_ref_R) {
            recovery_simple_l0_ref_R = leg[RIGHT]->l0;
        }
        if (!swing_motion_active) {
            if (leg[LEFT]->phi0 > recovery_simple_phi0_ref_L) {
                recovery_simple_phi0_ref_L = leg[LEFT]->phi0;
            }
            if (leg[RIGHT]->phi0 > recovery_simple_phi0_ref_R) {
                recovery_simple_phi0_ref_R = leg[RIGHT]->phi0;
            }

        }
        recovery_simple_force_gain_L = 1.0f;
        recovery_simple_force_gain_R = 1.0f;
        recovery_simple_breakaway_released = 1;
    }

    if (!swing_motion_active) {
        float phi0_target_L = Recovery_Simple_PhiTargetFromLength(leg[LEFT]->l0);
        float phi0_target_R = Recovery_Simple_PhiTargetFromLength(leg[RIGHT]->l0);

        /* The recovery envelope can only advance towards standing; never pull back.
         * 入口紧凑路径例外：折叠姿态的起始摆角可能已经在 80° 之外，不回拉就永远
         * 满足不了 phi0_err<10° 的交接稳定窗，所以允许它双向收敛到同一个终点。 */
        if (phi0_target_L > recovery_simple_phi0_ref_L ||
            (recovery_simple_compact_entry &&
             phi0_target_L < recovery_simple_phi0_ref_L)) {
            recovery_simple_phi0_ref_L = slope_to_target_f32(
                recovery_simple_phi0_ref_L,
                phi0_target_L,
                REC_SIMPLE_RETRACT_PHI0_RATE * dt);
        }
        if (phi0_target_R > recovery_simple_phi0_ref_R ||
            (recovery_simple_compact_entry &&
             phi0_target_R < recovery_simple_phi0_ref_R)) {
            recovery_simple_phi0_ref_R = slope_to_target_f32(
                recovery_simple_phi0_ref_R,
                phi0_target_R,
                REC_SIMPLE_RETRACT_PHI0_RATE * dt);
        }
    }

    float l0_err_L = recovery_simple_l0_ref_L - leg[LEFT]->l0;
    float l0_err_R = recovery_simple_l0_ref_R - leg[RIGHT]->l0;
    float phi0_err_L = swing_motion_active ?
        (recovery_swing_axis[LEFT].ref - recovery_swing_axis[LEFT].actual) :
        wrap_pi_f32(recovery_simple_phi0_ref_L - leg[LEFT]->phi0);
    float phi0_err_R = swing_motion_active ?
        (recovery_swing_axis[RIGHT].ref - recovery_swing_axis[RIGHT].actual) :
        wrap_pi_f32(recovery_simple_phi0_ref_R - leg[RIGHT]->phi0);

    leg[LEFT]->F_Spring_to_F_Vertical(leg[LEFT]);
    leg[RIGHT]->F_Spring_to_F_Vertical(leg[RIGHT]);

    float spring_ff_L = leg[LEFT]->F_Vertical;
    float spring_ff_R = leg[RIGHT]->F_Vertical;
    if (isnan(spring_ff_L) || isinf(spring_ff_L) || spring_ff_L < 0.0f) {
        spring_ff_L = 0.0f;
    }
    if (isnan(spring_ff_R) || isinf(spring_ff_R) || spring_ff_R < 0.0f) {
        spring_ff_R = 0.0f;
    }
    LIMIT_MIN_MAX(spring_ff_L, 0.0f, REC_SIMPLE_SPRING_FF_LIMIT);
    LIMIT_MIN_MAX(spring_ff_R, 0.0f, REC_SIMPLE_SPRING_FF_LIMIT);

    float d_l0_L = leg[LEFT]->d_l0_lpf;
    float d_l0_R = leg[RIGHT]->d_l0_lpf;
    if (isnan(d_l0_L) || isinf(d_l0_L)) {
        d_l0_L = leg[LEFT]->d_l0;
    }
    if (isnan(d_l0_R) || isinf(d_l0_R)) {
        d_l0_R = leg[RIGHT]->d_l0;
    }
    dbg_retract_spring_ff_L = spring_ff_L;
    dbg_retract_spring_ff_R = spring_ff_R;
    dbg_retract_d_l0_L = d_l0_L;
    dbg_retract_d_l0_R = d_l0_R;

    float bias_scale_L = (leg[LEFT]->l0 - REC_SIMPLE_RETRACT_L0_TARGET) / REC_SIMPLE_PUSH_BIAS_L0_RANGE;
    float bias_scale_R = (leg[RIGHT]->l0 - REC_SIMPLE_RETRACT_L0_TARGET) / REC_SIMPLE_PUSH_BIAS_L0_RANGE;
    LIMIT_MIN_MAX(bias_scale_L, 0.0f, 1.0f);
    LIMIT_MIN_MAX(bias_scale_R, 0.0f, 1.0f);

    float F_L = 0.0f;
    float F_R = 0.0f;
    float settle_blend_L = angle_only_active ? 0.0f : Recovery_Simple_TerminalBlend(leg[LEFT]->l0);
    float settle_blend_R = angle_only_active ? 0.0f : Recovery_Simple_TerminalBlend(leg[RIGHT]->l0);
    float tp_kp_L = REC_SIMPLE_SWING_TP_KP +
                    (REC_SIMPLE_RETRACT_TP_SETTLE_KP - REC_SIMPLE_SWING_TP_KP) * settle_blend_L;
    float tp_kp_R = REC_SIMPLE_SWING_TP_KP +
                    (REC_SIMPLE_RETRACT_TP_SETTLE_KP - REC_SIMPLE_SWING_TP_KP) * settle_blend_R;
    float tp_kd_L = REC_SIMPLE_SWING_TP_KD +
                    (REC_SIMPLE_RETRACT_TP_SETTLE_KD - REC_SIMPLE_SWING_TP_KD) * settle_blend_L;
    float tp_kd_R = REC_SIMPLE_SWING_TP_KD +
                    (REC_SIMPLE_RETRACT_TP_SETTLE_KD - REC_SIMPLE_SWING_TP_KD) * settle_blend_R;
    float tp_limit_L = REC_SIMPLE_SWING_TP_LIMIT +
                       (REC_SIMPLE_RETRACT_TP_SETTLE_LIMIT - REC_SIMPLE_SWING_TP_LIMIT) * settle_blend_L;
    float tp_limit_R = REC_SIMPLE_SWING_TP_LIMIT +
                       (REC_SIMPLE_RETRACT_TP_SETTLE_LIMIT - REC_SIMPLE_SWING_TP_LIMIT) * settle_blend_R;

    if (forward_zero_active) {
        tp_kp_L = REC_SIMPLE_FORWARD_ZERO_TP_KP;
        tp_kp_R = REC_SIMPLE_FORWARD_ZERO_TP_KP;
        tp_kd_L = REC_SIMPLE_FORWARD_ZERO_TP_KD;
        tp_kd_R = REC_SIMPLE_FORWARD_ZERO_TP_KD;
        tp_limit_L = REC_SIMPLE_FORWARD_ZERO_TP_LIMIT;
        tp_limit_R = REC_SIMPLE_FORWARD_ZERO_TP_LIMIT;
    }
    if (inverted_turn_active || inverted_align_active) {
        tp_kp_L = REC_SIMPLE_INVERTED_TURN_TP_KP;
        tp_kp_R = REC_SIMPLE_INVERTED_TURN_TP_KP;
        tp_kd_L = REC_SIMPLE_INVERTED_TURN_TP_KD;
        tp_kd_R = REC_SIMPLE_INVERTED_TURN_TP_KD;
        tp_limit_L = REC_SIMPLE_INVERTED_TURN_TP_LIMIT;
        tp_limit_R = REC_SIMPLE_INVERTED_TURN_TP_LIMIT;
    }
    if (inverted_settle_active) {
        tp_kp_L = REC_SIMPLE_INVERTED_SETTLE_TP_KP;
        tp_kp_R = REC_SIMPLE_INVERTED_SETTLE_TP_KP;
        tp_kd_L = REC_SIMPLE_INVERTED_SETTLE_TP_KD;
        tp_kd_R = REC_SIMPLE_INVERTED_SETTLE_TP_KD;
        tp_limit_L = REC_SIMPLE_INVERTED_SETTLE_TP_LIMIT;
        tp_limit_R = REC_SIMPLE_INVERTED_SETTLE_TP_LIMIT;
    }

    float omega_ref_L = swing_motion_active ? recovery_swing_axis[LEFT].omega_ref : 0.0f;
    float omega_ref_R = swing_motion_active ? recovery_swing_axis[RIGHT].omega_ref : 0.0f;
    float omega_actual_L = swing_motion_active ?
        recovery_swing_axis[LEFT].omega_actual : leg[LEFT]->d_phi0;
    float omega_actual_R = swing_motion_active ?
        recovery_swing_axis[RIGHT].omega_actual : leg[RIGHT]->d_phi0;
    float Tp_L = tp_kp_L * phi0_err_L + tp_kd_L * (omega_ref_L - omega_actual_L);
    float Tp_R = tp_kp_R * phi0_err_R + tp_kd_R * (omega_ref_R - omega_actual_R);

    if (forward_zero_active) {
        Tp_L *= REC_SIMPLE_FORWARD_ZERO_TP_SIGN;
        Tp_R *= REC_SIMPLE_FORWARD_ZERO_TP_SIGN;
    }

    if (inverted_active) {
        /* Cancel the spring, then add a small outward force to keep ground contact. */
        F_L = REC_SIMPLE_INVERTED_CONTACT_FORCE - spring_ff_L;
        F_R = REC_SIMPLE_INVERTED_CONTACT_FORCE - spring_ff_R;
    } else if (!angle_only_active) {
        F_L = REC_SIMPLE_L0_KP * l0_err_L - REC_SIMPLE_L0_KD * d_l0_L;
        F_R = REC_SIMPLE_L0_KP * l0_err_R - REC_SIMPLE_L0_KD * d_l0_R;

        F_L -= REC_SIMPLE_SPRING_FF_GAIN * spring_ff_L + REC_SIMPLE_F_BIAS * bias_scale_L;
        F_R -= REC_SIMPLE_SPRING_FF_GAIN * spring_ff_R + REC_SIMPLE_F_BIAS * bias_scale_R;
    }

    if (!inverted_active) {
        F_L *= recovery_simple_force_gain_L;
        F_R *= recovery_simple_force_gain_R;
    }
    Tp_L *= recovery_simple_angle_gain;
    Tp_R *= recovery_simple_angle_gain;

    if (forward_swing_motion_active) {
        float swing_ff_scale_L = omega_ref_L / REC_SIMPLE_SWING_PHI0_RATE;
        float swing_ff_scale_R = omega_ref_R / REC_SIMPLE_SWING_PHI0_RATE;

        LIMIT_MIN_MAX(swing_ff_scale_L, -1.0f, 1.0f);
        LIMIT_MIN_MAX(swing_ff_scale_R, -1.0f, 1.0f);
        Tp_L += recovery_simple_angle_gain * swing_ff_scale_L * REC_SIMPLE_SWING_TP_FF;
        Tp_R += recovery_simple_angle_gain * swing_ff_scale_R * REC_SIMPLE_SWING_TP_FF;
    }

    if (inverted_turn_active) {
        float turn_bias_scale_L = omega_ref_L / REC_SIMPLE_INVERTED_TURN_PHI_RATE;
        float turn_bias_scale_R = omega_ref_R / REC_SIMPLE_INVERTED_TURN_PHI_RATE;

        LIMIT_MIN_MAX(turn_bias_scale_L, -1.0f, 1.0f);
        LIMIT_MIN_MAX(turn_bias_scale_R, -1.0f, 1.0f);
        Tp_L += recovery_simple_angle_gain * turn_bias_scale_L *
            REC_SIMPLE_INVERTED_TURN_TP_BIAS;
        Tp_R += recovery_simple_angle_gain * turn_bias_scale_R *
            REC_SIMPLE_INVERTED_TURN_TP_BIAS;
    }

    dbg_swing_tp_raw_L = Tp_L;
    dbg_swing_tp_raw_R = Tp_R;

    float breakaway_force_L = Recovery_Simple_BreakawayForce(
        recovery_simple_timer,
        REC_SIMPLE_RETRACT_BREAKAWAY_RAMP_TIME_L,
        REC_SIMPLE_RETRACT_BREAKAWAY_TIME_L,
        REC_SIMPLE_RETRACT_BREAKAWAY_F_L,
        REC_SIMPLE_RETRACT_F_LIMIT_L);
    float breakaway_force_R = Recovery_Simple_BreakawayForce(
        recovery_simple_timer,
        REC_SIMPLE_RETRACT_BREAKAWAY_RAMP_TIME,
        REC_SIMPLE_RETRACT_BREAKAWAY_TIME,
        REC_SIMPLE_RETRACT_BREAKAWAY_F_R,
        REC_SIMPLE_RETRACT_F_LIMIT_R);

    breakaway_force_L = REC_SIMPLE_RETRACT_F_LIMIT_L + breakaway_scale_L *
        (breakaway_force_L - REC_SIMPLE_RETRACT_F_LIMIT_L);
    breakaway_force_R = REC_SIMPLE_RETRACT_F_LIMIT_R + breakaway_scale_R *
        (breakaway_force_R - REC_SIMPLE_RETRACT_F_LIMIT_R);

    if (left_breakaway_active && F_L > -breakaway_force_L) {
        F_L = -breakaway_force_L;
    }
    if (right_breakaway_active && F_R > -breakaway_force_R) {
        F_R = -breakaway_force_R;
    }

    float force_limit_L = REC_SIMPLE_RETRACT_F_LIMIT_L + breakaway_scale_L *
        (REC_SIMPLE_RETRACT_BREAKAWAY_F_L - REC_SIMPLE_RETRACT_F_LIMIT_L);
    float force_limit_R = REC_SIMPLE_RETRACT_F_LIMIT_R + breakaway_scale_R *
        (REC_SIMPLE_RETRACT_BREAKAWAY_F_R - REC_SIMPLE_RETRACT_F_LIMIT_R);
    float force_extend_limit_L = angle_only_active ? force_limit_L :
                                                REC_SIMPLE_RETRACT_F_EXTEND_LIMIT;
    float force_extend_limit_R = angle_only_active ? force_limit_R :
                                                REC_SIMPLE_RETRACT_F_EXTEND_LIMIT;
    LIMIT_MIN_MAX(F_L, -force_limit_L, force_extend_limit_L);
    LIMIT_MIN_MAX(F_R, -force_limit_R, force_extend_limit_R);
    LIMIT_MIN_MAX(Tp_L, -tp_limit_L, tp_limit_L);
    LIMIT_MIN_MAX(Tp_R, -tp_limit_R, tp_limit_R);

    if (swing_motion_active) {
        const float tp_slew_rate_L = REC_SIMPLE_SWING_TP_SLEW_RATE +
            settle_blend_L *
            (REC_SIMPLE_RETRACT_TERMINAL_TP_SLEW_RATE -
             REC_SIMPLE_SWING_TP_SLEW_RATE);
        const float tp_slew_rate_R = REC_SIMPLE_SWING_TP_SLEW_RATE +
            settle_blend_R *
            (REC_SIMPLE_RETRACT_TERMINAL_TP_SLEW_RATE -
             REC_SIMPLE_SWING_TP_SLEW_RATE);
        recovery_swing_axis[LEFT].tp_cmd = ramp_to(
            recovery_swing_axis[LEFT].tp_cmd,
            Tp_L,
            tp_slew_rate_L * swing_dt);
        recovery_swing_axis[RIGHT].tp_cmd = ramp_to(
            recovery_swing_axis[RIGHT].tp_cmd,
            Tp_R,
            tp_slew_rate_R * swing_dt);
        Tp_L = recovery_swing_axis[LEFT].tp_cmd;
        Tp_R = recovery_swing_axis[RIGHT].tp_cmd;
        LIMIT_MIN_MAX(Tp_L, -tp_limit_L, tp_limit_L);
        LIMIT_MIN_MAX(Tp_R, -tp_limit_R, tp_limit_R);
        recovery_swing_axis[LEFT].tp_cmd = Tp_L;
        recovery_swing_axis[RIGHT].tp_cmd = Tp_R;
    } else {
        recovery_swing_axis[LEFT].tp_cmd = Tp_L;
        recovery_swing_axis[RIGHT].tp_cmd = Tp_R;
    }

    if (!angle_only_active || inverted_active) {
        float torque_reserve_L = REC_SIMPLE_RETRACT_TORQUE_RESERVE_L +
            breakaway_scale_L * (REC_SIMPLE_RETRACT_BREAKAWAY_TORQUE_L -
                                 REC_SIMPLE_RETRACT_TORQUE_RESERVE_L);
        float torque_reserve_R = REC_SIMPLE_RETRACT_TORQUE_RESERVE_R +
            breakaway_scale_R * (REC_SIMPLE_RETRACT_BREAKAWAY_TORQUE_R -
                                 REC_SIMPLE_RETRACT_TORQUE_RESERVE_R);

        F_L = Recovery_Simple_ForceWithTorqueReserve(
            leg[LEFT], F_L, Tp_L, torque_reserve_L);
        F_R = Recovery_Simple_ForceWithTorqueReserve(
            leg[RIGHT], F_R, Tp_R, torque_reserve_R);
    }

    leg[LEFT]->support_force = F_L;
    leg[RIGHT]->support_force = F_R;
    leg[LEFT]->Tp = Tp_L;
    leg[RIGHT]->Tp = Tp_R;

    leg[LEFT]->vmc_cal_T(leg[LEFT], vmc_out_l);
    leg[RIGHT]->vmc_cal_T(leg[RIGHT], vmc_out_r);

    dbg_retract_l0_ref_L = recovery_simple_l0_ref_L;
    dbg_retract_l0_ref_R = recovery_simple_l0_ref_R;
    dbg_retract_phi0_ref_L = recovery_simple_phi0_ref_L;
    dbg_retract_phi0_ref_R = recovery_simple_phi0_ref_R;
    dbg_retract_F_L = F_L;
    dbg_retract_F_R = F_R;
    dbg_retract_Tp_L = Tp_L;
    dbg_retract_Tp_R = Tp_R;
    dbg_retract_gain = angle_only_active ? recovery_simple_angle_gain :
        0.5f * (recovery_simple_force_gain_L + recovery_simple_force_gain_R);
    dbg_retract_state = (float)recovery_simple_state;
    uint8_t l0_ok =
        (leg[LEFT]->l0 < REC_SIMPLE_RETRACT_L0_OK) &&
        (leg[RIGHT]->l0 < REC_SIMPLE_RETRACT_L0_OK);
    uint8_t vel_ok =
        (fabsf(d_l0_L) < REC_SIMPLE_STABLE_D_L0_MAX) &&
        (fabsf(d_l0_R) < REC_SIMPLE_STABLE_D_L0_MAX);
    uint8_t leg_motion_ok =
        (fabsf(leg[LEFT]->d_phi0) < REC_SIMPLE_STABLE_D_PHI0_MAX) &&
        (fabsf(leg[RIGHT]->d_phi0) < REC_SIMPLE_STABLE_D_PHI0_MAX) &&
        (fabsf(leg[LEFT]->d_theta_lpf) < REC_SIMPLE_STABLE_D_THETA_MAX) &&
        (fabsf(leg[RIGHT]->d_theta_lpf) < REC_SIMPLE_STABLE_D_THETA_MAX);
    uint8_t phi_ok =
        (fabsf(phi0_err_L) < REC_SIMPLE_STABLE_PHI0_ERR_MAX) &&
        (fabsf(phi0_err_R) < REC_SIMPLE_STABLE_PHI0_ERR_MAX);
    uint8_t effort_ok =
        (fabsf(F_L) < REC_SIMPLE_STABLE_F_MAX) &&
        (fabsf(F_R) < REC_SIMPLE_STABLE_F_MAX) &&
        (fabsf(Tp_L) < REC_SIMPLE_STABLE_TP_MAX) &&
        (fabsf(Tp_R) < REC_SIMPLE_STABLE_TP_MAX);
    uint8_t stable_ok = l0_ok && vel_ok && leg_motion_ok && phi_ok && effort_ok;

    dbg_retract_stable_mask = 0;
    if (l0_ok) {
        dbg_retract_stable_mask |= 0x01u;
    }
    if (vel_ok) {
        dbg_retract_stable_mask |= 0x02u;
    }
    if (leg_motion_ok) {
        dbg_retract_stable_mask |= 0x04u;
    }
    if (phi_ok) {
        dbg_retract_stable_mask |= 0x08u;
    }
    if (effort_ok) {
        dbg_retract_stable_mask |= 0x10u;
    }

    if (stable_ok) {
        recovery_simple_stable_timer += dt;
    } else {
        recovery_simple_stable_timer = 0.0f;
    }

    dbg_retract_pose_ok = stable_ok;
    if (recovery_simple_state == REC_SIMPLE_RETRACT &&
        recovery_simple_stable_timer > REC_SIMPLE_RETRACT_STABLE_TIME) {
#if REC_SIMPLE_AUTO_HANDOFF_ENABLE
        Recovery_Simple_EnterLqrPrep();
#else
        recovery_simple_state = REC_SIMPLE_RETRACT_HOLD;
        recovery_simple_timer = 0.0f;
        recovery_simple_stable_timer = REC_SIMPLE_RETRACT_STABLE_TIME;
        recovery_lqr_handoff_gain = 0.0f;
        recovery_lqr_handoff_last_active = 0;
        pid_clear(theta_pid);
        pid_clear(roll_pid);
        pid_clear(L_length_pid);
        pid_clear(R_length_pid);
#endif
    }
}

#endif

static void Recovery_DebugCaptureUpdate(void)
{
    if (chassis_cmd.ctrl_mode != CHASSIS_RECOVERY) {
        dbg_recovery_event_latch = 0;
        dbg_recovery_phi0_ref_active_L = leg[LEFT]->phi0;
        dbg_recovery_phi0_ref_active_R = leg[RIGHT]->phi0;
        dbg_recovery_l0_ref_active_L = leg[LEFT]->l0;
        dbg_recovery_l0_ref_active_R = leg[RIGHT]->l0;
        return;
    }

    if (chassis_fdb_data.stand_state == CHASSIS_IS_RECOVERY_READY) {
        dbg_recovery_phi0_ref_active_L = dbg_retract_phi0_ref_L;
        dbg_recovery_phi0_ref_active_R = dbg_retract_phi0_ref_R;
        dbg_recovery_l0_ref_active_L = dbg_retract_l0_ref_L;
        dbg_recovery_l0_ref_active_R = dbg_retract_l0_ref_R;
    } else {
        dbg_recovery_phi0_ref_active_L = leg[LEFT]->phi0;
        dbg_recovery_phi0_ref_active_R = leg[RIGHT]->phi0;
        dbg_recovery_l0_ref_active_L = leg[LEFT]->l0;
        dbg_recovery_l0_ref_active_R = leg[RIGHT]->l0;
    }

    if (fabsf(ins.pitch) > RECOVERY_DBG_PITCH_HIGH_DEG) {
        dbg_recovery_event_latch |= RECOVERY_DBG_LATCH_PITCH_HIGH;
    }

    if (dbg_recovery_phi0_ref_active_L > RECOVERY_DBG_PHI0_FORWARD_THRESH ||
        dbg_recovery_phi0_ref_active_R > RECOVERY_DBG_PHI0_FORWARD_THRESH ||
        leg[LEFT]->phi0 > RECOVERY_DBG_PHI0_FORWARD_THRESH ||
        leg[RIGHT]->phi0 > RECOVERY_DBG_PHI0_FORWARD_THRESH) {
        dbg_recovery_event_latch |= RECOVERY_DBG_LATCH_PHI0_FORWARD;
    }

    if (recovery_simple_state == REC_SIMPLE_RETRACT ||
        recovery_simple_state == REC_SIMPLE_RETRACT_HOLD ||
        recovery_simple_state == REC_SIMPLE_LQR_PREP ||
        recovery_simple_state == REC_SIMPLE_LQR_HANDOFF ||
        dbg_recovery_l0_ref_active_L < RECOVERY_DBG_L0_RETRACT_THRESH ||
        dbg_recovery_l0_ref_active_R < RECOVERY_DBG_L0_RETRACT_THRESH) {
        dbg_recovery_event_latch |= RECOVERY_DBG_LATCH_RETRACT_CMD;
    }

    if (chassis_fdb_data.stand_state == CHASSIS_IS_RECOVERY_READY &&
        (fabsf(leg[LEFT]->support_force) >
             RECOVERY_DBG_LIMIT_RATIO * REC_SIMPLE_RETRACT_F_LIMIT_L ||
         fabsf(leg[RIGHT]->support_force) >
             RECOVERY_DBG_LIMIT_RATIO * REC_SIMPLE_RETRACT_F_LIMIT_R)) {
        dbg_recovery_event_latch |= RECOVERY_DBG_LATCH_FORCE_LIMIT;
    }

    float active_tp_limit = (recovery_simple_state == REC_SIMPLE_INVERTED_TURN) ?
        REC_SIMPLE_INVERTED_TURN_TP_LIMIT : REC_SIMPLE_SWING_TP_LIMIT;
    if (chassis_fdb_data.stand_state == CHASSIS_IS_RECOVERY_READY &&
        (fabsf(leg[LEFT]->Tp) >
             RECOVERY_DBG_LIMIT_RATIO * active_tp_limit ||
         fabsf(leg[RIGHT]->Tp) >
             RECOVERY_DBG_LIMIT_RATIO * active_tp_limit)) {
        dbg_recovery_event_latch |= RECOVERY_DBG_LATCH_TP_LIMIT;
    }
}

/* Recovery 进入/退出 reset 函数 */
static inline uint8_t Recovery_IsZeroPose(void)
{
    uint8_t pitch_ok = fabsf(ins.pitch) < 15.0f;

    uint8_t l0_long =
        (leg[LEFT]->l0  > 0.30f) &&
        (leg[RIGHT]->l0 > 0.30f);

    /* 只有双侧都 near 0 才算 zero pose。
     * 一腿 near 0、一腿 near ±π 的姿态必须走非零位路径，
     * 不允许跳过 overbody 直接 READY。
     */
    uint8_t both_near_zero =
        (fabsf(leg[LEFT]->phi0)  < 0.70f) &&
        (fabsf(leg[RIGHT]->phi0) < 0.70f);

    return pitch_ok && l0_long && both_near_zero;
}

/*
 * @brief Run the single, torque-controlled recovery state machine.
 */
static void Chassis_Recovery(void)
{
    static float recovery_dt_start = 0.0f;
    float rec_dt;

    if (recovery_dt_start != 0.0f) {
        rec_dt = (dwt_get_time_ms() - recovery_dt_start) / 1000.0f;
    } else {
        rec_dt = REC_SIMPLE_CONTROL_DT_DEFAULT;
    }
    recovery_dt_start = dwt_get_time_ms();
    recovery_simple_raw_dt = rec_dt;
    LIMIT_MIN_MAX(rec_dt, REC_SIMPLE_CONTROL_DT_MIN, REC_SIMPLE_CONTROL_DT_MAX);
    recovery_simple_control_dt = rec_dt;

    Recovery_Simple_StateMachine(rec_dt);
}

void chassis_recovery_module_bind(const chassis_recovery_context_t *context)
{
    recovery_context = context;
}

void chassis_recovery_module_reset(void)
{
    if (recovery_context != NULL) {
        Recovery_Simple_Reset();
    }
}

void chassis_recovery_module_clear_handoff(void)
{
    recovery_lqr_handoff_gain = 0.0f;
    recovery_lqr_handoff_last_active = 0;
}

void chassis_recovery_module_update_mode(void)
{
    uint8_t recovery_active_now;

    if (recovery_context == NULL) {
        return;
    }

    recovery_active_now = (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY);
    if (recovery_active_now && !recovery_active_last) {
        Recovery_Pose_ResetOnEnter();
    }
    if (!recovery_active_now && recovery_active_last) {
        Recovery_Pose_ResetOnExit();
    }
    recovery_active_last = recovery_active_now;
}

void chassis_recovery_module_step(void)
{
    if (recovery_context != NULL) {
        Chassis_Recovery();
    }
}

void chassis_recovery_module_apply_length_hold(void)
{
    if (recovery_context != NULL) {
        Recovery_Simple_ApplyLqrLengthHold();
    }
}

uint8_t chassis_recovery_module_apply_retract_control(void)
{
    if (recovery_context == NULL ||
        chassis_cmd.ctrl_mode != CHASSIS_RECOVERY ||
        chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY) {
        return 0;
    }

    Recovery_Simple_RetractControl();
    return 1;
}

void chassis_recovery_module_apply_handoff_blend(void)
{
#if RECOVERY_SIMPLE_ENABLE
    if (recovery_context == NULL ||
        chassis_cmd.ctrl_mode != CHASSIS_RECOVERY ||
        recovery_simple_state != REC_SIMPLE_LQR_HANDOFF) {
        return;
    }

    float blend = recovery_lqr_handoff_gain;

    /* Keep the compact-leg VMC support alive while the original LQR takes over. */
    leg[LEFT]->support_force = recovery_simple_handoff_F_L +
        blend * (leg[LEFT]->support_force - recovery_simple_handoff_F_L);
    leg[RIGHT]->support_force = recovery_simple_handoff_F_R +
        blend * (leg[RIGHT]->support_force - recovery_simple_handoff_F_R);
    leg[LEFT]->Tp = recovery_simple_handoff_Tp_L +
        blend * (leg[LEFT]->Tp - recovery_simple_handoff_Tp_L);
    leg[RIGHT]->Tp = recovery_simple_handoff_Tp_R +
        blend * (leg[RIGHT]->Tp - recovery_simple_handoff_Tp_R);

    dbg_retract_F_L = leg[LEFT]->support_force;
    dbg_retract_F_R = leg[RIGHT]->support_force;
    dbg_retract_Tp_L = leg[LEFT]->Tp;
    dbg_retract_Tp_R = leg[RIGHT]->Tp;
#endif
}

void chassis_recovery_module_capture_debug(void)
{
    if (recovery_context != NULL) {
        Recovery_DebugCaptureUpdate();
    }
}

uint8_t chassis_recovery_module_has_stood_up(void)
{
    return recovery_context != NULL &&
           chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
           chassis_fdb_data.stand_state == CHASSIS_IS_STAND;
}

uint8_t chassis_recovery_module_after_handoff(void)
{
    return recovery_context != NULL &&
           chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
           (chassis_fdb_data.stand_state == CHASSIS_LEG_BACK_IS_OK ||
            chassis_fdb_data.stand_state == CHASSIS_IS_STAND);
}

void chassis_recovery_module_update_handoff_lqr(void)
{
    uint8_t handoff_active;

    if (recovery_context == NULL) {
        return;
    }

    handoff_active = chassis_recovery_module_after_handoff();
    if (handoff_active && !recovery_lqr_handoff_last_active) {
        LQRXObsBuf[LEFT][2] = 0.0f;
        LQRXObsBuf[RIGHT][2] = 0.0f;
        lqr_virtual_x_l = 0.0f;
        lqr_virtual_x_r = 0.0f;
    }
    recovery_lqr_handoff_last_active = handoff_active;

    if (handoff_active) {
        recovery_lqr_handoff_gain =
            RECOVERY_LQR_HANDOFF_INITIAL_GAIN +
            RECOVERY_LQR_HANDOFF_RAMP_RATE * recovery_simple_timer;
        if (recovery_lqr_handoff_gain > 1.0f) {
            recovery_lqr_handoff_gain = 1.0f;
        }
    }
}

float chassis_recovery_module_handoff_gain(void)
{
    return recovery_lqr_handoff_gain;
}

uint8_t chassis_recovery_module_debug_state(void)
{
    return (uint8_t)recovery_simple_state;
}

uint8_t chassis_recovery_module_debug_fault_reason(void)
{
    return dbg_simple_fault_reason;
}

float chassis_recovery_module_dm_output_limit(float nominal_limit)
{
#if RECOVERY_SIMPLE_ENABLE
    if (recovery_context != NULL &&
        chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
        recovery_simple_state == REC_SIMPLE_RETRACT &&
        recovery_simple_timer < REC_SIMPLE_RETRACT_BREAKAWAY_TIME) {
        return REC_SIMPLE_RETRACT_BREAKAWAY_TORQUE_R;
    }
#endif
    return nominal_limit;
}

uint8_t chassis_recovery_module_fill_debug_page(uint8_t page)
{
    if (recovery_context == NULL) {
        return 0;
    }

    if (page == 48U) {
        /* Recovery state-machine overview, strictly I0-I15:
         * I0=time(s), I1=chassis mode, I2=stand state, I3=recovery state,
         * I4=state timer(s), I5=raw Recovery interval(ms), I6=pitch(deg),
         * I7=fault reason, I8-I9=left/right l0(m),
         * I10-I11=left/right phi0(rad), I12-I15=final MIT torque command
         * for CAN IDs 0x01, 0x02, 0x03, 0x04 (N.m). */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_fdb_data.stand_state);
        chassis_debug_set(3, (float)recovery_simple_state);
        chassis_debug_set(4, recovery_simple_timer);
        chassis_debug_set(5, recovery_simple_raw_dt * 1000.0f);
        chassis_debug_set(6, ins.pitch);
        chassis_debug_set(7, (float)dbg_simple_fault_reason);
        chassis_debug_set(8, leg[LEFT]->l0);
        chassis_debug_set(9, leg[RIGHT]->l0);
        chassis_debug_set(10, leg[LEFT]->phi0);
        chassis_debug_set(11, leg[RIGHT]->phi0);
        chassis_debug_set(12, recovery_context->dm_obs[0]);
        chassis_debug_set(13, recovery_context->dm_obs[1]);
        chassis_debug_set(14, recovery_context->dm_obs[2]);
        chassis_debug_set(15, recovery_context->dm_obs[3]);
        return 1;
    }

    if (page == 39U && chassis_cmd.ctrl_mode == CHASSIS_RECOVERY) {
        /* Recovery motion shaping, I0-I23:
         * I0=time; I1=3+10*state; I2=state timer; I3=angle gain;
         * I4-I8=left ref/actual/ref-rate/actual-rate/Tp;
         * I9-I13=right ref/actual/ref-rate/actual-rate/Tp;
         * I14=left-right progress error; I15=fault reason;
         * I16/I17=Pitch/Roll; I18-I21=L/R length and length rate;
         * I22/I23=left/right selected swing direction. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, 3.0f + 10.0f * (float)recovery_simple_state);
        chassis_debug_set(2, recovery_simple_timer);
        chassis_debug_set(3, recovery_simple_angle_gain);
        chassis_debug_set(4, recovery_swing_axis[LEFT].ref);
        chassis_debug_set(5, recovery_swing_axis[LEFT].actual);
        chassis_debug_set(6, recovery_swing_axis[LEFT].omega_ref);
        chassis_debug_set(7, recovery_swing_axis[LEFT].omega_actual);
        chassis_debug_set(8, dbg_retract_Tp_L);
        chassis_debug_set(9, recovery_swing_axis[RIGHT].ref);
        chassis_debug_set(10, recovery_swing_axis[RIGHT].actual);
        chassis_debug_set(11, recovery_swing_axis[RIGHT].omega_ref);
        chassis_debug_set(12, recovery_swing_axis[RIGHT].omega_actual);
        chassis_debug_set(13, dbg_retract_Tp_R);
        chassis_debug_set(14, dbg_swing_sync_error);
        chassis_debug_set(15, (float)dbg_simple_fault_reason);
        chassis_debug_set(16, ins.pitch);
        chassis_debug_set(17, ins.roll);
        chassis_debug_set(18, leg[LEFT]->l0);
        chassis_debug_set(19, leg[RIGHT]->l0);
        chassis_debug_set(20, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(21, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(22, recovery_swing_axis[LEFT].direction);
        chassis_debug_set(23, recovery_swing_axis[RIGHT].direction);
        return 1;
    }

    if (page == 0) {
        /* I3 位域：bit0=处于 RECOVERY；bit1=stand_state 为 READY；
         * bit2=已进入交接；bit3~6=自救子状态(8*state)；
         * bit7=回零路径锁存(1=身上路径/后仰，0=身下路径/前倾)；
         * bit8=入口两腿已短于 0.20 m，本次跳过回零直接接摆腿/收腿；bit9+=事件锁存。 */
        float recovery_flags_pack = 0.0f;
        if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY) {
            recovery_flags_pack += 1.0f;
        }
        if (chassis_fdb_data.stand_state == CHASSIS_IS_RECOVERY_READY) {
            recovery_flags_pack += 2.0f;
        }
        if (chassis_recovery_module_after_handoff()) {
            recovery_flags_pack += 4.0f;
        }
        recovery_flags_pack += 8.0f * (float)recovery_simple_state;
        if (recovery_simple_over_body) {
            recovery_flags_pack += 128.0f;
        }
        if (recovery_simple_skip_zero_fired) {
            recovery_flags_pack += 256.0f;
        }
        recovery_flags_pack += 512.0f * (float)dbg_recovery_event_latch;

        chassis_debug_set(0, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(1, (float)chassis_fdb_data.stand_state);
        chassis_debug_set(2, ins.pitch);
        chassis_debug_set(3, recovery_flags_pack);
        chassis_debug_set(4, dbg_recovery_phi0_ref_active_L);
        chassis_debug_set(5, leg[LEFT]->phi0);
        chassis_debug_set(6, dbg_recovery_phi0_ref_active_R);
        chassis_debug_set(7, leg[RIGHT]->phi0);
        chassis_debug_set(8, dbg_recovery_l0_ref_active_L);
        chassis_debug_set(9, leg[LEFT]->l0);
        chassis_debug_set(10, dbg_recovery_l0_ref_active_R);
        chassis_debug_set(11, leg[RIGHT]->l0);
        chassis_debug_set(12, dbg_retract_F_L);
        chassis_debug_set(13, dbg_retract_F_R);
        chassis_debug_set(14, dbg_retract_Tp_L);
        chassis_debug_set(15, dbg_retract_Tp_R);
        return 1;
    }

    if (page == 8) {
        /* Inverted/underbody recovery capture, strictly I0~I15:
         * I0=t, I1=state, I2=state timer, I3=fault reason,
         * I4=pitch(deg), I5=pitch rate, I6~I7=left/right l0,
         * I8~I10=left phi ref/actual/Tp, I11~I13=right phi ref/actual/Tp,
         * I14=left-right swing progress error, I15=wait leg (1=L, -1=R). */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)recovery_simple_state);
        chassis_debug_set(2, recovery_simple_timer);
        chassis_debug_set(3, (float)dbg_simple_fault_reason);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, ins.gyro[0]);
        chassis_debug_set(6, leg[LEFT]->l0);
        chassis_debug_set(7, leg[RIGHT]->l0);
        chassis_debug_set(8, recovery_swing_axis[LEFT].ref);
        chassis_debug_set(9, recovery_swing_axis[LEFT].actual);
        chassis_debug_set(10, dbg_retract_Tp_L);
        chassis_debug_set(11, recovery_swing_axis[RIGHT].ref);
        chassis_debug_set(12, recovery_swing_axis[RIGHT].actual);
        chassis_debug_set(13, dbg_retract_Tp_R);
        chassis_debug_set(14, dbg_swing_sync_error);
        chassis_debug_set(15, (float)recovery_simple_inverted_wait_leg);
        return 1;
    }

    if (page == 11) {
        if (recovery_simple_state == REC_SIMPLE_RETRACT ||
            recovery_simple_state == REC_SIMPLE_LQR_PREP) {
            float phi_err_L = wrap_pi_f32(dbg_retract_phi0_ref_L - leg[LEFT]->phi0);
            float phi_err_R = wrap_pi_f32(dbg_retract_phi0_ref_R - leg[RIGHT]->phi0);
            float force_abs_max = fabsf(dbg_retract_F_L);
            float tp_abs_max = fabsf(dbg_retract_Tp_L);

            if (fabsf(dbg_retract_F_R) > force_abs_max) {
                force_abs_max = fabsf(dbg_retract_F_R);
            }
            if (fabsf(dbg_retract_Tp_R) > tp_abs_max) {
                tp_abs_max = fabsf(dbg_retract_Tp_R);
            }

            chassis_debug_set(0, (float)recovery_simple_state);
            chassis_debug_set(1, recovery_simple_timer);
            chassis_debug_set(2, recovery_simple_stable_timer);
            chassis_debug_set(3, (float)dbg_retract_stable_mask);
            chassis_debug_set(4, leg[LEFT]->l0);
            chassis_debug_set(5, dbg_retract_d_l0_L);
            chassis_debug_set(6, phi_err_L);
            chassis_debug_set(7, leg[LEFT]->d_phi0);
            chassis_debug_set(8, leg[LEFT]->d_theta_lpf);
            chassis_debug_set(9, leg[RIGHT]->l0);
            chassis_debug_set(10, dbg_retract_d_l0_R);
            chassis_debug_set(11, phi_err_R);
            chassis_debug_set(12, leg[RIGHT]->d_phi0);
            chassis_debug_set(13, leg[RIGHT]->d_theta_lpf);
            chassis_debug_set(14, force_abs_max);
            chassis_debug_set(15, tp_abs_max);
        } else {
            chassis_debug_set(0, (float)recovery_simple_state);
            chassis_debug_set(1, recovery_simple_timer);
            chassis_debug_set(2, recovery_lqr_handoff_gain);
            chassis_debug_set(3, ins.pitch);
            chassis_debug_set(4, ins.gyro[1]);
            chassis_debug_set(5, leg[LEFT]->length_ref);
            chassis_debug_set(6, leg[LEFT]->l0);
            chassis_debug_set(7, leg[LEFT]->d_l0_lpf);
            chassis_debug_set(8, L_length_pid->Pout);
            chassis_debug_set(9, L_length_pid->Iout);
            chassis_debug_set(10, L_length_pid->Dout);
            chassis_debug_set(11, L_length_pid->Output);
            chassis_debug_set(12, leg[LEFT]->support_force);
            chassis_debug_set(13, leg[LEFT]->Tp);
            chassis_debug_set(14, vmc_out_l[0]); /* Front hip torque. */
            chassis_debug_set(15, vmc_out_l[1]); /* Back hip torque. */
        }
        return 1;
    }

    if (page == 18U) {
        uint8_t swing_motion_active =
            (recovery_simple_state == REC_SIMPLE_SWING) ||
            (recovery_simple_state == REC_SIMPLE_RETRACT) ||
            (recovery_simple_state == REC_SIMPLE_FORWARD_ZERO) ||
            (recovery_simple_state == REC_SIMPLE_INVERTED_TURN);
        float phi_ref_L = swing_motion_active ?
            recovery_swing_axis[LEFT].ref : dbg_retract_phi0_ref_L;
        float phi_ref_R = swing_motion_active ?
            recovery_swing_axis[RIGHT].ref : dbg_retract_phi0_ref_R;
        /* Recovery 左右腿破死点诊断，严格使用 I0~I15：
         * I0=t, I1=state, I2=state timer, I3=fault reason,
         * I4~I6=左腿长参考/实际/速度，I7~I9=右腿长参考/实际/速度，
         * I10=平均摆角参考，I11~I12=左右实际摆角，I13=机身 pitch，
         * I14~I15=左右实际轴向力。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)recovery_simple_state);
        chassis_debug_set(2, recovery_simple_timer);
        chassis_debug_set(3, (float)dbg_simple_fault_reason);
        chassis_debug_set(4, dbg_retract_l0_ref_L);
        chassis_debug_set(5, leg[LEFT]->l0);
        chassis_debug_set(6, dbg_retract_d_l0_L);
        chassis_debug_set(7, dbg_retract_l0_ref_R);
        chassis_debug_set(8, leg[RIGHT]->l0);
        chassis_debug_set(9, dbg_retract_d_l0_R);
        chassis_debug_set(10, 0.5f * (phi_ref_L + phi_ref_R));
        chassis_debug_set(11, leg[LEFT]->phi0);
        chassis_debug_set(12, leg[RIGHT]->phi0);
        chassis_debug_set(13, ins.pitch);
        chassis_debug_set(14, dbg_retract_F_L);
        chassis_debug_set(15, dbg_retract_F_R);
        return 1;
    }

    return 0;
}
