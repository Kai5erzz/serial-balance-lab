/**
 * @file chassis_control.c
 * @brief Normal chassis control, LQR, VMC, yaw and wheel mixing.
 */
#include "chassis_control.h"

#include "debug/chassis_debug.h"
#include "jump_kinematic_rate_observer.h"
#include "jump_local_rate_feedback.h"
#include "jump_air_profile.h"
#include "jump_reaction_wheel.h"
#include "jump_stroke_release.h"
#include "jump_theta_diff_d_helpers.h"
#include "recovery/chassis_recovery.h"
#include "safety/chassis_safety.h"
#include "jump_landing_pid.h"
#include "jump_landing_speed.h"
#include "jump_landing_ownership.h"
#include "jump_landing_fast_exit.h"
#include "jump_ground_recovery.h"
#include "bump_control_helpers.h"
#include "lateral_inertial_ff.h"
#include "rm_config.h"

#include <math.h>
#include <string.h>

#define LEFT  CHASSIS_SIDE_LEFT
#define RIGHT CHASSIS_SIDE_RIGHT
#define FRONT 0
#define BACK  1

typedef struct {
    uint32_t rx_count[4];
    uint32_t last_rx_ms[4];
    float angle_abs[4];
} jump_hip_feedback_snapshot_t;

static const chassis_control_context_t *control_context = NULL;
static jump_landing_pid_state_t jump_landing_pid_state[CHASSIS_SIDE_COUNT] = {{0}, {0}};
static jump_landing_vx_ref_state_t jump_motion_vx_ref_state = {0};
/* 611 的 chassis_feedback_update() 在 914 不存在；这里用 914 自己的
 * jump_hip_feedback_snapshot_fresh() 加 IMU 角速度有限性做等价门控。
 * 本次移植中只作为 BUMP 状态机的入口门控，不承担其他职责。 */
static uint8_t control_feedback_valid;
/* Updated by chassis_kf_update() before the BUMP APPROACH cycle runs. */
static float chassis_vx_filter;

#define ins              (*control_context->ins)
#define chassis_cmd      (*control_context->cmd)
#define chassis_fdb_data (*control_context->fdb)
#define chassis_dt       (control_context->control_dt_s)

/* Jump-specific measured cycle period.  The normal/recovery/safety chains keep
 * the legacy control_dt_s = 0.005 s because their loops were tuned on that
 * derivative scaling.  Jump local timers, slew rates and the vertical-speed
 * integral, however, need the real ~1 ms period. */
static float jump_cycle_dt_s = 0.001f;
static float jump_dt_last_ms;
static float jump_cycle_raw_dt_ms;
static uint8_t jump_cycle_dt_primed;
static uint8_t jump_cycle_dt_first_sample;
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
static jump_kin_rate_observer_t jump_kin_rate_observer[CHASSIS_SIDE_COUNT];
#endif
static jump_landing_rate_state_t jump_landing_length_rate_state[CHASSIS_SIDE_COUNT];
static float jump_landing_length_rate[CHASSIS_SIDE_COUNT];
static float jump_landing_length_rate_age_ms = INFINITY;
static uint8_t jump_landing_length_rate_valid;
static uint8_t jump_position_snapshot_fresh;
static float jump_kin_length_rate[CHASSIS_SIDE_COUNT];
static float jump_kin_theta_rate[CHASSIS_SIDE_COUNT];
static float jump_kin_rate_age_ms = INFINITY;
static uint8_t jump_kin_rates_fresh;
static uint8_t jump_kin_rates_usable;
static uint8_t jump_kin_geometry_valid = 1U;
static void jump_hip_feedback_snapshot_capture(
    jump_hip_feedback_snapshot_t *snapshot);
static uint8_t jump_hip_feedback_snapshot_fresh(
    const jump_hip_feedback_snapshot_t *snapshot, uint32_t now_tick,
    uint32_t *max_age_ms);
static void jump_landing_length_rate_update(
    float now_ms, uint32_t now_tick,
    const jump_hip_feedback_snapshot_t *snapshot,
    uint8_t feedback_fresh);
static jump_local_rate_selection_t jump_theta_rate_select(
    chassis_side_e side, float legacy_rate, uint8_t jump_owner_scope);
static float jump_theta_rate_or_zero(
    chassis_side_e side, float legacy_rate, uint8_t jump_owner_scope);
static inline uint8_t recovery_after_handoff(void);

static void jump_cycle_dt_update(void)
{
    const float now_ms = dwt_get_time_ms();
    float dt = 0.001f;
    jump_cycle_dt_first_sample = (uint8_t)!jump_cycle_dt_primed;
    if (jump_cycle_dt_primed) {
        jump_cycle_raw_dt_ms = now_ms - jump_dt_last_ms;
        dt = jump_cycle_raw_dt_ms * 0.001f;
    } else {
        jump_cycle_raw_dt_ms = 0.0f;
    }
    jump_cycle_dt_primed = 1U;
    jump_dt_last_ms = now_ms;
    if (!(dt > 0.0002f) || dt > 0.020f) {
        dt = 0.001f;
    }
    jump_cycle_dt_s = dt;
}
#if THETA_LENGTH_CALIBRATION_ENABLE
static float theta_level_comp[3] = {
    THETA_COMP_CAL_INIT_LOW,
    THETA_COMP_CAL_INIT_MID,
    THETA_COMP_CAL_INIT_HIG,
};
#else
static float theta_level_comp[3] = {0.0f, 0.0f, 0.0f};
#endif
static float theta_level_apply_gain = 0.0f;
static leg_level_e last_leg_level_for_comp = LEG_LOW;
static float leg_switch_freeze_timer = 0.0f;
static float theta_comp_learn_settle_timer = 0.0f;
static float theta_low_drift_trim = 0.0f;
static float theta_low_drift_apply_gain = 0.0f;
static float theta_low_drift_converge_timer = 0.0f;
static uint8_t theta_low_drift_latched = 0U;
static float theta_cal_vx_lpf = 0.0f;
static float theta_cal_vx_slow_lpf = 0.0f;
static float theta_cal_vx_trend = 0.0f;
static float theta_cal_leg_settle_timer = 0.0f;
static float theta_cal_zero_hold_timer = 0.0f;
static float theta_cal_latch_bad_timer = 0.0f;
static uint8_t theta_cal_value_latched[3] = {0U, 0U, 0U};
static leg_level_e theta_cal_last_level = LEG_LOW;

static uint8_t dbg_theta_level_apply_enable = 0;
static uint8_t dbg_theta_level_learn_enable = 0;
static float dbg_theta_level_comp_cur = 0.0f;
static float dbg_theta_level_comp_low = 0.0f;
static float dbg_theta_level_comp_mid = 0.0f;
static float dbg_theta_level_comp_hig = 0.0f;
static float dbg_theta_level_apply_gain = 0.0f;
static float dbg_theta_vx_filter = 0.0f;
static float dbg_theta_total_comp_L = 0.0f;
static float dbg_theta_total_comp_R = 0.0f;
static float dbg_theta_l0_average = 0.0f;
static float dbg_theta_poly_base = 0.0f;
static float dbg_theta_active_comp = 0.0f;
static float dbg_theta_active_trim = 0.0f;
static float dbg_theta_cal_flags = 0.0f;
static float dbg_theta_drift_x = 0.0f;
static float dbg_theta_strict_trim = 0.0f;


/*
 * @brief 斜坡函数
 * @param end目标值
 * @param begin初始值
 * @param measure 量测值
 * @param set 斜坡过程值
 * @param step_length 步长
 * @param safe_region 合理范围
 * */





#define LEN_LEN_LOW     0.13f // 单位：m
#define LEN_LEN_MID     0.21f // 单位：m
#define LEN_LEN_HIG     0.30f // 单位：m
/* 正常站立模式的腿长指令斜坡；0.16 m/s 时 LOW->MID 约需 0.5 s。 */
#define LEG_LENGTH_REF_SLEW_RATE 0.24f
/* Spin 中腿长切换需要更快完成，避免旋转期间长时间处于过渡几何。 */
#define LEG_LENGTH_REF_SLEW_RATE_SPIN 0.40f
#define LEG_LENGTH_REF_EPSILON   0.0001f
#define FORCE_Length_LIMIT 200.0f //
#define FORCE_LIMIT 200.0f // 支持力限幅,保护电机,防止起跳时电压过低电机保护

/* BUMP 台阶模式参数。长度单位 m、力矩单位 N m、时间单位 ms；
 * 接触阈值按本机 M3508_TOR_MAX=3.69 N m 留出余量，不能使用 4 N m。 */
/* 2026-10-07: 接近/完成腿长 0.34 -> 0.30 m（按实车要求降低 BUMP 工作高度）。
 * 该值同时是 APPROACH、EXTEND、DONE 三态的目标腿长，也是 CONTACT/RETRACT
 * 之外的基准高度；RETRACT(0.13) 与 ABORT(LEN_LEN_LOW) 是瞬态目标，未改。
 * 注意不要用本文件里的 LEN_LEN_HIG（=0.30f）代替：那个宏在本文件表示
 * 0.21~0.30 混合区间的上界，与 chassis_task.c 的 LEG_HIG 档位(0.34f) 语义不同。 */
#define BUMP_APPROACH_LENGTH             0.30f
#define BUMP_RETRACT_LENGTH              0.13f
#define BUMP_APPROACH_VX_RC_LIMIT       150.0f
#define BUMP_APPROACH_VX_MIN_MPS          0.08f /* chassis_vx_filter, m/s */
#define BUMP_APPROACH_WHEEL_TORQUE_LIMIT 2.20f
#define BUMP_CONTACT_TORQUE_ON          1.30f
#define BUMP_CONTACT_TORQUE_OFF         1.10f
#define BUMP_CONTACT_THETA_MIN_RAD      0.20f
#define BUMP_CONTACT_THETA_MAX_RAD      1.40f
#define BUMP_CONTACT_WHEEL_SPEED_MAX_RADPS 3.0f
#define BUMP_CONTACT_CONFIRM_TIME_MS    35U
#define BUMP_CONTACT_PAIR_WINDOW_MS    150U
#define BUMP_CONTACT_TIMEOUT_MS         900U
/* 2026-10-07: 已停用（保留宏供追溯）。APPROACH 不再有超时 —— 接触前就是
 * 正常行驶，无限等待。原值 1800 ms 会让「按拨杆+前进」在 1.8 s 后必然进
 * ABORT 并收腿摔倒，详见 BUMP_STATE_APPROACH 分支的注释。 */
/* #define BUMP_APPROACH_TIMEOUT_MS     1800U */
#define BUMP_APPROACH_STABLE_TIME_MS    80U
#define BUMP_APPROACH_LENGTH_TOLERANCE  0.020f
#define BUMP_APPROACH_RATE_MAX          0.080f
#define BUMP_LENGTH_REF_SLEW_RATE       0.30f
#define BUMP_CONTACT_HOLD_TIME_MS      60U
#define BUMP_RETRACT_TOLERANCE          0.015f
#define BUMP_RETRACT_MIN_STABLE_LENGTH  0.100f
#define BUMP_RETRACT_REACQUIRE_ANGLE_RAD (8.0f * DEGREE_2_RAD)
/* BUMP 角度目标跟踪每条腿各自的 phi0，并用真实控制周期推进。 */
#define BUMP_SWING_ANGLE_OK             (4.0f * DEGREE_2_RAD)
/* Start retract as soon as both measured angles enter this target band.
 * RETRACT angle reacquire retains the tighter 4° band. */
#define BUMP_SWING_OVER_ANGLE_OK        (6.0f * DEGREE_2_RAD)
/* bump16's loaded right leg held at 75.3-75.8 deg. Start LOW support
 * inside this band instead of waiting stationary for the 1 s timeout. */
#define BUMP_SWING_SUPPORT_ANGLE_OK     (6.0f * DEGREE_2_RAD)
#define BUMP_SWING_ANGLE_RATE_MAX       (0.10f)
#define BUMP_SWING_OVER_PHI0_RAD       (-10.0f * DEGREE_2_RAD)
#define BUMP_SWING_SUPPORT_PHI0_RAD     (80.0f * DEGREE_2_RAD)
#define BUMP_PHI0_MIN_RAD               (-25.0f * DEGREE_2_RAD)
#define BUMP_PHI0_MAX_RAD               (110.0f * DEGREE_2_RAD)
/* Both swing states keep a continuous 240 deg/s cruise reference. */
#define BUMP_SWING_PHI0_RATE            (240.0f * DEGREE_2_RAD)
#define BUMP_RETRACT_REACQUIRE_PHI0_RATE (25.0f * DEGREE_2_RAD)
#define BUMP_SWING_OVER_TIMEOUT_MS      6000U
/* If the measured target band is not reached within 1 s, hand control to
 * the normal LOW LQR so leg support can take over on the step. */
#define BUMP_SWING_SUPPORT_TIMEOUT_MS   1000U
#define BUMP_RETRACT_TIMEOUT_MS         1400U
#define BUMP_RETRACT_RATE                0.30f
#define BUMP_PHI0_RATE_FILTER_TAU         0.025f
#define BUMP_SWING_RATE_FILTER_TAU        0.005f
#define BUMP_PHI0_RATE_MAX               12.0f
/* Swing-only gains: the former 70/1.6 position loop repeatedly overtook
 * the moving reference, braked to a stop, then accelerated again.  Keep
 * the legacy hold/retract gains below separate from this velocity damping. */
#define BUMP_SWING_TRACK_TP_KP          32.0f
#define BUMP_SWING_TRACK_TP_KD           5.5f
/* At rest, restore position authority without removing SWING damping.
 * bump15 stalled 6.36 deg short of -10 deg with only -3.55 Nm at Kp=32,
 * outside the 6 deg RETRACT gate. Keep the softer Kp while cruising. */
#define BUMP_SWING_HOLD_TP_KP           70.0f
#define BUMP_SWING_TP_KP                70.0f  /* recovery REC_SIMPLE_SWING_TP_KP */
#define BUMP_SWING_TP_KD                 1.6f  /* recovery REC_SIMPLE_SWING_TP_KD */
#define BUMP_SWING_TP_FF                 3.0f  /* recovery REC_SIMPLE_SWING_TP_FF */
#define BUMP_SWING_TP_LIMIT             15.0f  /* recovery REC_SIMPLE_SWING_TP_LIMIT */
/* RETRACT angle reacquire is the terminal recovery pose.  Use damped gains
 * here; full swing gains made bump4's left leg cross its held phi0 while Tp
 * repeatedly hit the +/-15 limit. */
#define BUMP_RETRACT_REACQUIRE_TP_KP     32.0f
#define BUMP_RETRACT_REACQUIRE_TP_KD      5.5f
#define BUMP_RETRACT_REACQUIRE_TP_LIMIT  10.0f
/* 起立：把腿长目标从 bump 的 0.13 平滑混回 leg->length_ref（操作手档位），
 * 让机器人自己伸腿站起来；混完且腿长速度收敛即交接回普通控制。 */
#define BUMP_STAND_RATE                  1.5f
#define BUMP_STAND_SUPPORT_RATE          1.5f
#define BUMP_STAND_EXTEND_RATE            1.5f
#define BUMP_STAND_D_L0_OK               0.10f
#define BUMP_STAND_LENGTH_TOLERANCE      0.020f
#define BUMP_STAND_PITCH_OK_DEG         30.0f /* slope support handoff */
#define BUMP_STAND_ROLL_OK_DEG           6.0f
#define BUMP_STAND_PITCH_HARD_DEG       35.0f /* slope safety ceiling */
#define BUMP_STAND_ROLL_HARD_DEG        16.0f
#define BUMP_STAND_STABLE_TIME_MS       150U
#define BUMP_STAND_FINISH_TIME_MS       200U
#define BUMP_STAND_TIMEOUT_MS          5000U
#define BUMP_STAND_PHI0_SUPPORT_MIN_RAD (60.0f * DEGREE_2_RAD)
#define BUMP_STAND_PHI0_SUPPORT_MAX_RAD (110.0f * DEGREE_2_RAD)
#define BUMP_STAND_FINAL_THETA_OK_RAD   (8.0f * DEGREE_2_RAD)
#define BUMP_STAND_FINAL_THETA_RATE_MAX  0.50f
#define BUMP_STAND_FINAL_THETA_SYNC_RAD (8.0f * DEGREE_2_RAD)
#define BUMP_SPRING_COMP_LIMIT         120.0f
#define BUMP_EXTEND_TOLERANCE           0.020f /* DONE 判据：腿长回到名义高度的容差 */
#define BUMP_DONE_STABLE_TIME_MS        250U
#define BUMP_DONE_SPEED_MAX             0.035f
#define BUMP_ABORT_TIMEOUT_MS            900U
#define BUMP_ABORT_LENGTH               LEN_LEN_LOW
#define BUMP_LENGTH_KP                  520.0f
#define BUMP_LENGTH_KD                   24.0f
#define BUMP_LENGTH_OUTPUT_LIMIT         130.0f
#define BUMP_RETRACT_BREAKAWAY_RELEASE_L0 0.285f
#define BUMP_RETRACT_BREAKAWAY_RELEASE_BAND 0.020f
#define BUMP_RETRACT_BREAKAWAY_MIN_TIME  0.050f
#define BUMP_RETRACT_BREAKAWAY_RAMP_TIME 0.020f
#define BUMP_RETRACT_BREAKAWAY_FORCE     180.0f
#define BUMP_RETRACT_BREAKAWAY_HANDOFF_TIME 0.100f
#define BUMP_RETRACT_SUSTAIN_FORCE_LIMIT 200.0f
#define BUMP_RETRACT_LOWER_HOLD_FORCE    110.0f
#define BUMP_STAND_SHORT_LEG_RELEASE_M   0.280f
#define BUMP_STAND_SHORT_LEG_FORCE_LIMIT 40.0f
#define BUMP_RETRACT_BREAKAWAY_TORQUE_RESERVE 24.0f
#define BUMP_RETRACT_TORQUE_RESERVE      19.0f
#define BUMP_RETRACT_EXTEND_LIMIT       20.0f
#define BUMP_LANDING_WHEEL_KP             5.0f
#define BUMP_LANDING_WHEEL_KD             0.8f
#define BUMP_LANDING_WHEEL_LIMIT          2.5f
#define BUMP_LANDING_WHEEL_SLEW_RATE     20.0f
#define BUMP_LANDING_WHEEL_RELEASE_TIME   0.20f

/* Jump profile shared with the validated traditional_lqr simulation.  Ground
 * wheel LQR remains active through thrust; after takeoff the normal wheel row
 * is shut and a separate bounded reaction-wheel channel is applied. */
#define JUMP_COOLDOWN_MS                 3000U
#define JUMP_PRELOAD_LENGTH              0.125f
#define JUMP_THRUST_LENGTH               0.305f
#define JUMP_PRELOAD_MIN_TIME            0.22f
#define JUMP_PRELOAD_MAX_TIME             0.85f
#define JUMP_PRELOAD_READY_LENGTH        0.155f
#define JUMP_PRELOAD_READY_RATE          0.25f
#define JUMP_THRUST_BOOST_FORCE          70.0f
/* 2026-09-23 (jump21.csv): this is the TUCK-RELEASE instant, so it must coincide
 * with the moment the legs finish extending -- not overshoot it.
 *
 * History: 0.26 s was too short (jump16 aborted with the stroke unfinished), so
 * it was raised to 0.42 s.  jump21 shows 0.42 s is now too LONG: the legs reach
 * full extension (0.36 m) at 14.103 and the window did not close until 14.227,
 * i.e. ~0.13 s of boost was wasted pushing an already-bottomed leg (which only
 * decelerates the body).  The tuck therefore started at 14.231 while the robot
 * had already been airborne since 14.103, leaving the retract only the 79 ms
 * until touchdown -- at the measured -1.11 m/s that is 88 mm of travel against
 * the 100 mm needed to reach JUMP_AIR_TUCK_LENGTH.  The operator's report was
 * exactly this: "the legs don't tuck".
 *
 * 0.32 s puts the release at 14.127, right as the stroke completes, giving the
 * tuck 0.183 s of the 0.207 s physical flight (~0.20 m of travel) -- double what
 * it needs. */
#define JUMP_THRUST_BOOST_TIME           0.32f
#define JUMP_THRUST_SUPPORT_FLOOR       120.0f
#define JUMP_THRUST_MAX_TIME             0.85f
#define JUMP_ABORT_HOLD_TIME              0.12f
#define JUMP_ABORT_SETTLE_TIME            0.60f
#define JUMP_ABORT_RETRACT_RATE            0.35f
#define JUMP_FLIGHT_TIMEOUT               1.50f
#define JUMP_LANDING_GROUND_CONFIRM_TIME  0.15f
#define JUMP_LANDING_TIMEOUT              2.00f
#define JUMP_LANDING_PITCH_MAX_DEG       12.0f
#define JUMP_LANDING_ROLL_MAX_DEG        10.0f
#define JUMP_LANDING_LENGTH_DIFF_MAX      0.030f
#define JUMP_LANDING_TIMEOUT_LENGTH_DIFF_MAX 0.060f
#define JUMP_LANDING_THETA_HOLD_KP         22.0f
#define JUMP_LANDING_THETA_HOLD_KD          8.0f
#define JUMP_LANDING_THETA_HOLD_LIMIT      28.0f
#define JUMP_LANDING_THETA_DIFF_KP         22.0f
#define JUMP_LANDING_THETA_DIFF_KD          8.0f
#define JUMP_LANDING_THETA_DIFF_LIMIT      28.0f
/* 2026-09-23 item 1: landing is confirmed by real leg compression, not only by
 * the contact-force edge.  See jump_landing_entry_length. */
#define JUMP_LANDING_COMPRESS_MIN           0.012f
#define JUMP_LANDING_COMPRESS_LENGTH_MAX    0.24f
/* Transfer hip and wheel balance together once contact and posture admit it.
 * A speed-magnitude gate cannot govern this transfer: the pitch-only damper
 * cannot brake translation, so it can never satisfy that gate after a moving
 * touchdown. Keep finite-speed validation and the existing posture bounds. */
#define JUMP_LANDING_OWNER_PITCH_STABLE_DEG  8.0f
#define JUMP_LANDING_OWNER_ROLL_STABLE_DEG   8.0f
#define JUMP_LANDING_OWNER_LENGTH_DIFF_MAX   0.060f
#define JUMP_LANDING_OWNER_EARLY_GAIN_LIMIT  0.20f
/* 2026-09-23 item 2: global jump failsafe.  Every state already has its own
 * timeout, but their SUM can reach 0.85 + 0.70 + 1.50 + 2.00 s.  The total cap
 * still aborts an unresolved airborne sequence; once a jump-origin landing
 * owner has real ground contact, the landing buffer's own bounded exit logic
 * is allowed to finish instead of being cleared at the 1.50 s boundary. */
#define JUMP_TOTAL_TIMEOUT                  1.50f
/* The stroke release and the final length reference share the feasible 0.30 m
 * target. The former 0.36 m request was clamped to 0.30 m downstream, leaving
 * travel release unreachable until the timer had already consumed flight. */
#define JUMP_THRUST_TARGET_LENGTH           0.30f
#define JUMP_THRUST_FORCE_FADE_START        0.26f
/* Roll uses degrees for position and rad/s for angular rate. Grounded launch
 * damping includes the unequal leg-extension momentum below; the 60 N clamp
 * and final motor budget bound its authority. Air control owns flight forces. */
#define JUMP_ROLL_DIFF_KP                   8.0f
#define JUMP_ROLL_DIFF_KD                1000.0f
#define JUMP_ROLL_DIFF_LIMIT               60.0f
#define JUMP_ROLL_DIFF_DEADBAND_DEG         0.1f
/* Effective left/right leg inertia ratio from the identified plant. Regulate
 * launch momentum, not just body gyro: unequal extending legs can carry a
 * roll impulse even when the body is momentarily level. Gain is rad/m. */
#define JUMP_ROLL_MOMENTUM_RATE_GAIN         0.75f
#define JUMP_ROLL_LEFT_INERTIA_RATIO         0.82f
/* Grounded jump yaw authority uses wheel differential torque only.  The
 * measured leg-velocity term anticipates angular momentum released when the
 * moving legs tuck; damping only the current body gyro worsens flight yaw.
 * These launch gains are validated against the identified simulation plant. */
#define JUMP_GROUND_YAW_KP                   4.0f
#define JUMP_GROUND_YAW_KD                   2.0f
#define JUMP_GROUND_YAW_MOMENTUM_GAIN        10.0f
#define JUMP_GROUND_YAW_LEFT_INERTIA_RATIO    0.82f
#define JUMP_GROUND_YAW_TORQUE_LIMIT          1.0f
/* Jump gates re-aligned 2026-09-22 to the validated rl-lab sim
 * (precheck 25/25 deg, vx 10, abort 40/30, landing 12/10, preload
 * max 0.85 s, ready 0.155 m).  The earlier relaxation (12/12, 2.5,
 * 24/16, 5/4, 1.20, 0.145) papered over residual thrust/air attitude
 * oscillation; with the air block and theta-sync damping aligned the
 * stock gates are restored. */
#define JUMP_PRECHECK_PITCH_MAX_DEG      25.0f
#define JUMP_PRECHECK_ROLL_MAX_DEG       25.0f
#define JUMP_PRECHECK_VX_MAX             10.0f
#define JUMP_PRECHECK_YAW_RATE_MAX         2.50f
#define JUMP_ABORT_PITCH_MAX_DEG         40.0f
#define JUMP_ABORT_ROLL_MAX_DEG          30.0f
/* 2026-09-23 (jump18/jump19.csv): early leg-angle abort.  theta wraps at +-pi,
 * so once a rod passes vertical every downstream angle (and the theta loop
 * itself) becomes ambiguous -- jump19 had both rods run phi0 120 -> 174 -> -165
 * deg with theta wrapping -0.62 -> +5.29 rad, i.e. the legs physically rotated a
 * full turn before the phi0 guard (20 deg from horizontal, 80 ms confirm) could
 * react.  0.85 rad (49 deg) is well inside the safe band and leaves room for the
 * landing hold to still have authority. */
#define JUMP_ABORT_THETA_MAX_RAD          0.85f
#define JUMP_LENGTH_KP_PRELOAD           450.0f
#define JUMP_LENGTH_KD_PRELOAD            28.0f
#define JUMP_LENGTH_LIMIT_PRELOAD         70.0f
#define JUMP_LENGTH_KP_THRUST            600.0f
#define JUMP_LENGTH_KD_THRUST             16.0f
#define JUMP_LENGTH_LIMIT_THRUST         120.0f
#define JUMP_LENGTH_KP_RELEASE           700.0f
#define JUMP_LENGTH_KD_RELEASE            28.0f
#define JUMP_LENGTH_LIMIT_RELEASE        150.0f
#define JUMP_LENGTH_SYNC_KP              400.0f
#define JUMP_LENGTH_SYNC_KD               70.0f
#define JUMP_LENGTH_SYNC_LIMIT            30.0f
#define JUMP_LANDING_SYNC_KP             140.0f
#define JUMP_LANDING_SYNC_KD              24.0f
#define JUMP_LANDING_SYNC_LIMIT           18.0f
#define JUMP_LANDING_LENGTH_KD_LOW        30.0f  /* jump-origin external D */
#define JUMP_THRUST_PITCH_KP                8.0f
#define JUMP_THRUST_PITCH_KD               20.0f
#define JUMP_THRUST_PITCH_TP_LIMIT         40.0f
#define JUMP_THRUST_PITCH_TP_SLEW_RATE    100.0f
#define JUMP_THRUST_PITCH_RATE_TARGET_ZERO  1.48f
#define JUMP_THRUST_PITCH_RATE_VX_GAIN      1.80f
#define JUMP_THRUST_PITCH_RATE_TARGET_MIN  (-0.25f)
#define JUMP_THRUST_PITCH_RATE_TARGET_MAX   1.48f
#define JUMP_THRUST_PITCH_RATE_START_L0     0.16f
#define JUMP_THRUST_PITCH_RATE_FULL_L0      0.26f
#define JUMP_THRUST_PITCH_RATE_FILTER_TAU   0.060f
/* The common hip axis cannot independently hold body Pitch and keep both leg
 * rods vertical.  As takeoff extension builds, hand that axis to the leg
 * vertical controller and use the same law as the airborne controller.
 *
 * 2026-09-23 (jump17.csv): these were 0.0, which left the leg angle completely
 * free through the whole powered extension.  Measured consequence -- theta
 * common drifted from +0.19 to -0.63 rad while Pitch climbed +0.5 -> +18.6 deg,
 * with BOTH wheel motors pinned at +-8191 the entire time and still losing.
 * The body's COM sits behind the wheel contact, so letting the rods swing back
 * under a 0.13 -> 0.37 m extension is a pure nose-up moment that the wheels
 * cannot hold.  The jump then aborted on RIGHT_PHI_HIGH (phi0_R reached the
 * 160 deg mechanical limit) with the body already 20 deg nose-up.
 *
 * The airborne channel needed the same gain and the same negative-feedback sign
 * (see chassis_safety.c); enable the grounded one to match, so the rods stay
 * near the reference and the body never builds the nose-up rotation in the
 * first place. */
#define JUMP_THRUST_THETA_HOLD_KP            80.0f
#define JUMP_THRUST_THETA_HOLD_KD            12.0f
#define JUMP_THRUST_THETA_HOLD_LIMIT         40.0f
/* Half the per-leg hold gains produce the paired differential request:
 * left += K/2*(theta_l-theta_r), right -= the same. Normal theta PID unchanged. */
#define JUMP_THRUST_THETA_DIFF_KP (0.5f * JUMP_THRUST_THETA_HOLD_KP)
#define JUMP_THRUST_THETA_DIFF_KD (0.5f * JUMP_THRUST_THETA_HOLD_KD)
#define JUMP_PRETUCK_THETA_REF_RATE            2.00f
#define JUMP_THRUST_THETA_YIELD_START         0.10f
#define JUMP_THRUST_THETA_YIELD_END           0.30f
#define JUMP_THRUST_THETA_LQR_GAIN            0.75f
#define JUMP_LANDING_TP_HANDOFF_TIME         0.060f
#define JUMP_AIR_PITCH_K_GAIN              1.00f
#define JUMP_AIR_PITCH_D_GAIN              1.10f
#define JUMP_AIR_WHEEL_PITCH_KP           35.0f
#define JUMP_AIR_WHEEL_PITCH_KD            5.0f
#define JUMP_AIR_WHEEL_THETA_FF            0.10f
#define JUMP_AIR_WHEEL_TORQUE_LIMIT       M3508_TOR_MAX
#define JUMP_AIR_TUCK_PITCH_BIAS_RAD       (4.0f * DEGREE_2_RAD)
#define JUMP_AIR_DEPLOY_PITCH_BIAS_RAD     (4.0f * DEGREE_2_RAD)
#define JUMP_AIR_MOVING_PITCH_GAIN_RAD    (3.0f * DEGREE_2_RAD)
/* Landing-buffer wheel pitch damper (ported from the rl-lab jump landing
 * capture).  While the landing buffer absorbs the impact, the two wheel
 * rotors act as a soft, slew-limited body-Pitch damper instead of the full
 * standing LQR wheel row.  The same body-Pitch sign convention and left/right
 * split as the air reaction wheel keeps the air -> contact handoff
 * continuous, and the small gains cannot turn a touchdown into a horizontal
 * drive kick.  Full LQR wheel authority returns only after the buffer
 * releases, ramping in over JUMP_LANDING_WHEEL_RELEASE_TIME. */
#define JUMP_LANDING_WHEEL_ENABLE             1U
#define JUMP_LANDING_WHEEL_PITCH_KP          10.0f
#define JUMP_LANDING_WHEEL_PITCH_KD           1.0f
#define JUMP_LANDING_WHEEL_TORQUE_LIMIT      M3508_TOR_MAX
#define JUMP_LANDING_WHEEL_SLEW_RATE       1000.0f
#define JUMP_LANDING_WHEEL_RELEASE_TIME       0.25f
#define JUMP_THETA_SYNC_KP                  24.0f
#define JUMP_THETA_SYNC_DAMP_KD              2.80f
#define JUMP_THETA_SYNC_DAMP_LIMIT          14.0f
/* Centrifugal Roll feedforward uses commanded vx/yaw rate.  The previous
 * measured vx*yaw path was delayed and weakened the correctly phased Roll PD. */
#define LATERAL_INERTIAL_FF_GAIN_LOW    1.30f
#define LATERAL_INERTIAL_FF_GAIN_MID    1.15f
#define LATERAL_INERTIAL_FF_GAIN_HIGH   1.15f
#define LATERAL_INERTIAL_FF_LIMIT      50.0f
#define LATERAL_INERTIAL_FF_SLEW_RATE 500.0f
#define LATERAL_INERTIAL_FF_ACCEL_DB   0.20f

/* Restore the pre-Spin normal profile.  Spin keeps the stronger profile that
 * was verified together with its present yaw-rate and Roll compensation. */
#define LENGTH_PID_NORMAL_MID_KD       22.0f
#define LENGTH_PID_SPIN_EXTENDED_KP   700.0f
#define LENGTH_PID_SPIN_EXTENDED_KD    18.0f

/* Dynamic axial support feedforward.  The source signal follows SENTRY_LEG:
 * per-side gravity projected by leg theta minus the gas-spring vertical force.
 * The first vehicle test blends only a conservative fraction into the proven
 * legacy support baseline. */
#define SUPPORT_FORCE_FF_ENABLE          1
#define SUPPORT_FORCE_FF_BLEND           0.35f
#define SUPPORT_FORCE_FF_C0              1.10f
#define SUPPORT_FORCE_FF_C1              0.0004f
#define SUPPORT_FORCE_FF_C2              0.0001f
#define SUPPORT_FORCE_FF_LPF_RC          0.05f
#define SUPPORT_FORCE_FF_CORR_LIMIT      10.0f
#define SUPPORT_FORCE_FF_CORR_SLEW_RATE  60.0f

/* Roll 恢复为单一的左右支持力差动路径；腿长目标始终保持对称。 */
#define ROLL_LENGTH_TARGET_MIN    0.110f
#define ROLL_LENGTH_TARGET_MAX    0.300f

/* Slow geometric terrain compensation.  It is learned only while settled,
 * then held during motion so dynamic Roll is never mistaken for a slope. */
#define ROLL_GEOMETRY_COMP_ENABLE        1
#define ROLL_GEOMETRY_COMP_GAIN          0.35f
#define ROLL_GEOMETRY_OFFSET_LIMIT       0.004f
#define ROLL_GEOMETRY_OFFSET_SLEW_RATE   0.008f
#define ROLL_GEOMETRY_SETTLE_TIME        0.50f
#define ROLL_GEOMETRY_VX_REF_DB          0.10f
#define ROLL_GEOMETRY_VX_FDB_DB          0.10f
#define ROLL_GEOMETRY_YAW_RATE_DB        0.15f
#define ROLL_GEOMETRY_ROLL_RATE_DB       0.10f
#define ROLL_GEOMETRY_SLOPE_TAN_LIMIT    0.20f
#define ROLL_GEOMETRY_DENOM_MIN          0.05f

/* Static roll trim only learns after the chassis has settled. Dynamic roll
 * response remains entirely on the existing scheduled PD controller. */
#define ROLL_STATIC_TRIM_KI            1.80f
#define ROLL_STATIC_TRIM_LIMIT        12.00f
#define ROLL_STATIC_TRIM_APPLY_LIMIT  10.00f
#define ROLL_STATIC_TRIM_APPLY_SLEW_RATE 12.00f
#define ROLL_STATIC_TRIM_SETTLE_TIME   0.35f
#define ROLL_STATIC_TRIM_DEADBAND_DEG  0.20f
#define ROLL_STATIC_TRIM_VX_CMD_DB     20.0f
#define ROLL_STATIC_TRIM_VX_FDB_DB     0.12f
#define ROLL_STATIC_TRIM_YAW_RATE_DB   0.15f
#define ROLL_STATIC_TRIM_ROLL_RATE_DB  0.15f
/* Pre-Spin normal-mode trim profile.  Spin keeps the current profile above
 * and its dedicated target/integral compensation. */
#define ROLL_STATIC_TRIM_NORMAL_KI             1.20f
#define ROLL_STATIC_TRIM_NORMAL_LIMIT          8.00f
#define ROLL_STATIC_TRIM_NORMAL_APPLY_LIMIT    6.00f
#define ROLL_STATIC_TRIM_NORMAL_SETTLE_TIME    0.50f
#define ROLL_STATIC_TRIM_NORMAL_VX_FDB_DB      0.08f
#define ROLL_STATIC_TRIM_NORMAL_YAW_RATE_DB    0.12f
#define ROLL_STATIC_TRIM_NORMAL_ROLL_RATE_DB   0.10f

/* Sustained Spin adds a repeatable roll load at extended leg lengths.  Keep
 * ordinary driving and the already-stable low-leg Spin unchanged; Page 50
 * shows enough differential-force headroom for this small dedicated gain. */
#define ROLL_SPIN_EXTENDED_GAIN_SCALE  1.18f
/* Exact normal-mode Roll profile from the last chassis revision before the
 * Spin tuning series.  The current rm_config profile remains Spin-only. */
#define ROLL_NORMAL_KP                  3.5f
#define ROLL_NORMAL_KD                  0.10f
#define ROLL_NORMAL_MAX_FORCE          80.0f
#define ROLL_HIGH_YAW_RATE_MIN           4.0f
#define ROLL_HIGH_YAW_RELEASE_RATE       3.0f
#define ROLL_HIGH_YAW_RELEASE_ROLL_DEG   2.0f
#define ROLL_HIGH_YAW_RELEASE_ROLL_RATE  0.5f
#define ROLL_HIGH_YAW_KP                 4.5f
#define ROLL_HIGH_YAW_KD                 0.14f
#define ROLL_HIGH_YAW_MAX_FORCE         ROLL_NORMAL_MAX_FORCE
#define ROLL_FEEDBACK_PRIORITY_FORCE_START 32.0f
#define ROLL_FEEDBACK_PRIORITY_FORCE_END   40.0f
#define ROLL_NORMAL_GAIN_L0_START       0.140f
#define ROLL_NORMAL_GAIN_L0_END         0.260f
#define ROLL_NORMAL_GAIN_SCALE          (m_b_whole / m_b)

/* Spin roll feedforward.  Page 50 shows a repeatable positive roll bias in
 * the selected negative spin direction at every leg length.  The scheduled
 * target offset supplies the steady differential force that a pure PD cannot,
 * while a small bounded integral removes the remaining slow drift. */
#define ROLL_SPIN_TARGET_LOW_DEG          1.45f
#define ROLL_SPIN_TARGET_MID_DEG          1.85f
#define ROLL_SPIN_TARGET_HIGH_DEG         1.35f
#define ROLL_SPIN_SCHEDULE_L0_LOW         0.13f
#define ROLL_SPIN_SCHEDULE_L0_MID         0.21f
#define ROLL_SPIN_SCHEDULE_L0_HIGH        0.27f
#define ROLL_SPIN_TARGET_ENGAGE_RATE     12.0f
#define ROLL_SPIN_TARGET_RELEASE_RATE    16.0f
#define ROLL_SPIN_INTEGRAL_KI_LOW         4.0f
#define ROLL_SPIN_INTEGRAL_KI_MID         7.0f
#define ROLL_SPIN_INTEGRAL_KI_HIGH        7.5f
#define ROLL_SPIN_INTEGRAL_LIMIT_LOW     18.0f
#define ROLL_SPIN_INTEGRAL_LIMIT_MID     12.0f
#define ROLL_SPIN_INTEGRAL_LIMIT_HIGH    15.0f
#define ROLL_SPIN_INTEGRAL_APPLY_RATE    40.0f
#define ROLL_SPIN_INTEGRAL_RELEASE_RATE  30.0f
#define ROLL_SPIN_INTEGRAL_RATE_RATIO     0.65f
#define ROLL_SPIN_INTEGRAL_DEADBAND_DEG   0.08f
#define ROLL_SPIN_INTEGRAL_ROLL_RATE_MAX  1.0f
#define ROLL_SPIN_INTEGRAL_SETTLE_TIME    0.40f

/* 正常腿长控制的静摩擦补偿。仅在腿长误差明显且机构近似静止时介入，
 * 逐渐增加轴向力，机构开始运动后快速退出，避免解除死点时猛冲。 */
#define LEG_STUCK_ERROR_THRESHOLD 0.0025f
#define LEG_STUCK_SPEED_THRESHOLD 0.025f
#define LEG_STUCK_DETECT_TIME     0.120f
#define LEG_STUCK_FORCE_RAMP_RATE 16.0f
#define LEG_STUCK_FORCE_DECAY_RATE 32.0f
#define LEG_STUCK_FORCE_MAX       8.0f
#define LEG_STUCK_POST_RAMP_TIME  2.0f
#define LEG_STUCK_POST_RAMP_FORCE_MAX 6.0f

/* 实际腿长差超过分档阈值时，只撤销 roll 腿长偏置。左右腿仍由各自
 * 腿长 PID 恢复，不再额外叠加一套差动力，避免差动增益突变。 */
#define LEG_SPLIT_LIMIT_LOW         0.035f
#define LEG_SPLIT_LIMIT_MID         0.030f
#define LEG_SPLIT_LIMIT_HIGH        0.030f
#define LEG_SPLIT_RELEASE_HYSTERESIS 0.005f

static float leg_lenthchange_flag = 0;//倒地自起中腿长切换标志位

/* Theta 动态补偿 PI 控制器参数 */
#define theta_comp_Kp       0.00f
#define theta_comp_Ki       0.00f
#define theta_comp_Kd       0.0f
#define theta_comp_InteVal  0.030f
#define theta_comp_MaxVal   0.1f

#define Theta_Compensation_recovery  0.0f//-0.10924f
#define Theta_Compensation_leg_low   0.0f//-0.10924f//-0.115f
#define Theta_Compensation_leg_mid   0.0f//-0.80f
#define Theta_Compensation_leg_high  0.0f//-0.05f

/* 新云台补偿拟合完成前保留旧几何项作为安全基线，避免平衡点阶跃。 */
#define DX_OFFSET_LEGACY_NO_GIMBAL 0.0182f
#define DX_OFFSET                  DX_OFFSET_LEGACY_NO_GIMBAL

/* 按腿长分别记忆的 theta 微补偿参数 */
#define THETA_LEVEL_COMP_KI              0.040f
#define THETA_LEVEL_COMP_LIMIT_LOW       0.070f
#define THETA_LEVEL_COMP_LIMIT_MID       0.080f
#define THETA_LEVEL_COMP_LIMIT_HIG       0.080f
#define THETA_LEVEL_COMP_VX_DEADBAND     0.005f
#define THETA_LEVEL_COMP_VX_ENABLE_MAX   0.025f
#define THETA_LEVEL_COMP_SETTLE_TIME     0.80f
#define THETA_LEVEL_COMP_DTHETA_MAX      0.60f
#define THETA_LEVEL_COMP_PITCH_RATE_MAX  0.25f
#define THETA_LEVEL_COMP_X_ENABLE_MAX    0.060f

/* The fitted low-leg endpoint is currently biased. Once BRAKE has completed,
 * allow only LOW to learn a persistent drift with bounded rate and range. */
#define THETA_LOW_DRIFT_VX_ENABLE_MAX    0.60f
#define THETA_LOW_DRIFT_VX_ERROR_LIMIT   0.20f
#define THETA_LOW_DRIFT_KI               0.025f
#define THETA_LOW_DRIFT_TRIM_LIMIT       0.020f
#define THETA_LOW_DRIFT_CONVERGE_VX_MAX  0.030f
#define THETA_LOW_DRIFT_CONVERGE_TIME    0.50f
#define THETA_LOW_DRIFT_APPLY_RAMP_RATE  2.00f

/* Calibration must be able to correct a visible initial drift. Production
 * learning remains subject to the much tighter stationary conditions above. */
#define THETA_CAL_VX_ENABLE_MAX           0.60f
#define THETA_CAL_VX_ERROR_LIMIT          0.25f
#define THETA_CAL_KI                      0.004f
#define THETA_CAL_VX_LPF_RC               0.35f
#define THETA_CAL_VX_TREND_RC              1.50f
#define THETA_CAL_TREND_DAMP_TIME          2.00f
#define THETA_CAL_LEARN_VX_DEADBAND        0.003f
#define THETA_CAL_ZERO_VX_BAND             0.025f
#define THETA_CAL_ZERO_VX_TREND_MAX        0.004f
#define THETA_CAL_ZERO_HOLD_TIME           3.00f
#define THETA_CAL_LATCH_RELEASE_VX          0.050f
#define THETA_CAL_LATCH_RELEASE_TIME        2.00f
#define THETA_CAL_REF_TOLERANCE           0.002f
#define THETA_CAL_ACTUAL_LENGTH_ERROR_MAX 0.004f
#define THETA_CAL_LEG_SETTLE_TIME         3.00f

/* apply_gain 斜坡速率：从 0 恢复到 1 所需时间 ≈ 1/rate，0.67s */
#define THETA_LEVEL_APPLY_RAMP_RATE      1.5f

/* 腿长切换后冻结学习的时间 (秒)，防止切换瞬间的速度尖峰污染新槽 */
#define THETA_LEVEL_SWITCH_FREEZE_TIME   2.00f

/* Zero-command localization follows DRIVE -> BRAKE -> HOLD. Position is
 * captured only after wheel-speed feedback has settled, then retained without
 * the former 0.998-per-cycle leak. */
#define LQR_POSITION_CMD_DEADBAND        20.0f
#define LQR_POSITION_CAPTURE_VX_MAX      0.05f
#define LQR_POSITION_CAPTURE_TIME        0.10f
#define LQR_POSITION_LIMIT               1.50f
#define LQR_POSITION_BRAKE_RELEASE_RATE  1.50f

#define CHASSIS_V_MAX_LOW     2.30f
#define CHASSIS_V_MAX_MID     1.85f
#define CHASSIS_V_MAX_HIGH    1.20f
#define YAW_TURN_RATIO  0.08f  //单位应该为°,有关调节遥控器转向敏感度的系数,自行在安全范围内调节大小
static float Vx_Delta;
#define VX_DELTA_MAX 5.0f

/* ==================== Yaw 控制状态机参数 ==================== */
#define YAW_RC_DEADBAND             50.0f   /* 遥控器 yaw 输入死区 */
#define YAW_RATE_MAX_LOW_RADPS      3.80f
#define YAW_RATE_MAX_MID_RADPS      3.20f
#define YAW_RATE_MAX_HIGH_RADPS     2.50f
#define YAW_SPIN_RATE_MAX_LOW_RADPS 10.50f
#define YAW_SPIN_RATE_MAX_MID_RADPS  8.20f
#define YAW_SPIN_RATE_MAX_HIGH_RADPS 5.80f
#define YAW_FOLLOW_RATE_MAX_LOW     6.40f
#define YAW_FOLLOW_RATE_MAX_MID     5.60f
#define YAW_FOLLOW_RATE_MAX_HIGH    4.50f
#define YAW_FOLLOW_ANGLE_KP         5.00f
#define YAW_FOLLOW_ANGLE_SIGN       (-1.0f)
#define YAW_FOLLOW_ANGLE_DEADBAND   0.020f
#define YAW_FOLLOW_ANGLE_LIMIT      1.50f
#define YAW_FOLLOW_FF_RATE_MIN      0.10f
#define YAW_FOLLOW_ACTIVE_BRAKE_MARGIN 0.05f
#define YAW_OPERATOR_RATE_ACCEL     20.0f
#define YAW_OPERATOR_RATE_DECEL     32.0f
#define YAW_RATE_REF_DEADBAND       0.015f
#define YAW_RATE_SIGN               (1.0f)  /* 拨杆方向：实车左右反了改符号 */
#define YAW_GYRO_SIGN               (1.0f)  /* ins.gyro 已是 rad/s，仅在此处校正方向 */
#define YAW_RATE_FF                 0.34f   /* 拨杆前馈基础力矩 */
#define YAW_RATE_FB_KP              0.80f   /* 拨杆角速度反馈 Kp */
#define YAW_RATE_FB_KI              0.60f   /* 对称负载补偿：消除持续转向误差 */
#define YAW_RATE_I_LIMIT            0.80f   /* 积分补偿限幅，避免遮蔽机械卡滞 */
#define YAW_RATE_FB_ENABLE          1       /* 0=关反馈纯前馈测试, 1=正常前馈+反馈 */
#define YAW_RATE_CMD_MIN            0.06f   /* 拨杆非零时最小输出，防静摩擦 */
#define YAW_RATE_OVERSPEED_MARGIN   0.020f  /* actual 超过 target 上限后的允许余量 */
#define YAW_RATE_OVERSPEED_DAMP_KP  0.90f   /* yaw_dot 超速时的阻尼刹车 Kp */
#define YAW_COAST_DAMP_KP           0.45f   /* 松杆滑行阶段弱阻尼 Kp */
#define YAW_DAMP_KP                 0.80f   /* 正式阻尼阶段 Kp */
#define YAW_RELEASE_COAST_TIME      0.05f   /* 松杆后滑行时间 (s) */
#define YAW_LOCK_RATE_TH            0.045f  /* 锁角前允许的最大 yaw_rate ≈ 2.6 deg/s */
#define YAW_LOCK_ZERO_TIME          0.12f   /* 速率与残余输出均稳定后的捕获时间 (s) */
#define YAW_LOCK_OUTPUT_SETTLE_TH   0.030f  /* 捕获目标前允许的残余 yaw 输出 */
#define YAW_LOCK_CAPTURE_DEADBAND   0.012f  /* 锁角死区 ≈ 0.7 deg */
#define YAW_LOCK_KP                 2.00f   /* 锁角位置反馈 */
#define YAW_LOCK_KD                 0.35f   /* 锁角角速度阻尼 */
#define YAW_LOCK_OUTPUT_MAX         0.60f   /* 锁角阶段最大附加轮端力矩 */
#define YAW_BLOCK_RELEASE_HOLDOFF   0.20f   /* 卡住解除后延迟锁角 (s) */
#define YAW_OUTPUT_SLEW_RATE        25.0f   /* yaw 输出斜率限制 (单位/s) */

/* 卡住检测参数 */
#define YAW_BLOCK_ON_CNT            250     /* 1 kHz 下约 250 ms，避免中高腿起转慢被误判 */
#define YAW_BLOCK_OFF_CNT           15      /* 连续 N 周期才判定解除 */
#define YAW_BLOCK_HYST_RATIO        0.6f    /* 解除门限 = VX_DELTA_MAX * 此比例 */
#define YAW_BLOCK_CMD_MIN           0.20f   /* 有输出才检查卡住 */
#define YAW_BLOCK_RATE_LOW_TH       0.035f  /* 约 2 deg/s，yaw_rate 低于此值认为没转 */
#define YAW_BLOCK_OUTPUT_LIMIT      0.35f   /* 卡住时限最大输出 */

/* Yaw 混控调试参数 */
#define YAW_MIX_L_SIGN             (1.0f)  /* 左轮 yaw 混控符号 */
#define YAW_MIX_R_SIGN             (1.0f)  /* 右轮 yaw 混控符号 */



/* Yaw 转向优先级 */
#define YAW_PRIORITY_ENABLE        1       /* 1=开启防抵消保护 */
#define YAW_WHEEL_NET_MIN          0.08f   /* 转向时每侧最小净输出 */
#define YAW_LQR_COUNTER_RATIO_MAX  0.35f   /* LQR 最多抵消 yaw_cmd 的比例 */
#define YAW_PRIORITY_VX_CMD_DB     5.0f    /* 有直行意图时禁止 yaw 覆盖 LQR */
#define YAW_PRIORITY_VX_FDB_DB     0.08f   /* 已发生纵向漂移时必须让位置环完整接管 */
#define YAW_PRIORITY_SPIN_VX_FDB_DB 0.20f  /* Spin wheel-speed common mode contains a small geometry bias. */
#define YAW_PRIORITY_LQR_COMMON_MAX 1.50f  /* 中高腿常见共模下仍防止单轮抵消 */

/* 起转踢一脚 */
#define YAW_START_KICK_ENABLE      1
#define YAW_START_KICK_TIME        0.05f
#define YAW_START_KICK_GAIN        1.15f
#define YAW_START_RATE_LOW_TH      0.035f

/* 悬空混控方向测试（默认关闭，仅手扶/悬空使用） */
#define YAW_MIX_TEST_ENABLE        0

/* LQR 轮端共模化混控（1=开启：LQR 左右取均值再分给两轮，消除腿差引入的差速） */
#define YAW_LQR_COMMON_MIX_ENABLE  1
#define YAW_LQR_LOCAL_RESIDUAL_ENABLE 0
/* 局部 LQR 差量仅保留诊断值。实车确认其会与单方向 yaw 争用轮端力矩，
 * 因此默认不回注；中高腿 theta 同步改由髋关节差模环承担。 */
#define YAW_LQR_LOCAL_RESIDUAL_L0_START 0.160f
#define YAW_LQR_LOCAL_RESIDUAL_L0_END   0.240f
#define YAW_LQR_LOCAL_RESIDUAL_LIMIT    3.00f

typedef enum {
    YAW_DAMP_MODE = 0,  /* 正式角速度阻尼 */
    YAW_LOCK_MODE,      /* yaw 角度锁定 */
    YAW_RATE_MODE,      /* 拨杆角速度控制 */
    YAW_COAST_MODE      /* 松杆后滑行/弱阻尼，不锁角 */
} yaw_ctrl_state_e;

static yaw_ctrl_state_e yaw_ctrl_state = YAW_DAMP_MODE;
static float yaw_ctrl_output = 0.0f;
static float yaw_rate_iout = 0.0f;
static float yaw_zero_timer = 0.0f;
static uint8_t yaw_blocked = 0;
static uint8_t yaw_block_on_cnt = 0;
static uint8_t yaw_block_off_cnt = 0;
static float block_release_holdoff_timer = 0.0f;
static uint8_t last_rc_turning = 0;
static float yaw_release_coast_timer = 0.0f;
static float yaw_rate_ref_applied = 0.0f;

/* 调试变量 */
static uint8_t dbg_yaw_state = 0;
static uint8_t dbg_yaw_blocked = 0;
static float dbg_yaw_rate_ref = 0.0f;
static float dbg_yaw_rate_fdb = 0.0f;
static float dbg_yaw_ctrl_output = 0.0f;
static float dbg_yaw_ctrl_raw = 0.0f;
static float dbg_yaw_rate_iout = 0.0f;
static float dbg_yaw_target = 0.0f;
static float dbg_yaw_rate_desired = 0.0f;
static float dbg_follow_yaw_relative = 0.0f;
static float dbg_follow_yaw_error = 0.0f;
static float dbg_yaw_zero_timer = 0.0f;
static float dbg_vx_delta = 0.0f;
static float dbg_yaw_release_coast_timer = 0.0f;
static uint8_t dbg_last_rc_turning = 0;

/* 轮端混控调试变量 */
static float dbg_lqr_cmd_l = 0.0f;
static float dbg_lqr_cmd_r = 0.0f;
static float dbg_yaw_cmd_l = 0.0f;
static float dbg_yaw_cmd_r = 0.0f;
static float dbg_wheel_set_l = 0.0f;
static float dbg_wheel_set_r = 0.0f;
static float dbg_m3508_spd_l = 0.0f;
static float dbg_m3508_spd_r = 0.0f;
static float dbg_net_cmd_l = 0.0f;
static float dbg_net_cmd_r = 0.0f;
static float dbg_lqr_local_residual_raw = 0.0f;
static float dbg_lqr_local_residual_applied = 0.0f;

/* Yaw full chain sign debug 变量 (Page 5) */
static float dbg_yaw_raw_rc = 0.0f;             // chassis_cmd.vw_relative
static float dbg_yaw_raw_rc_set = 0.0f;         // chassis_cmd.vw_relative_set
static float dbg_yaw_vx_set = 0.0f;             // chassis_cmd.vx_set
static float dbg_yaw_lqr_common = 0.0f;         // 左右 LQR 轮端输出的共模项
static float dbg_yaw_now_rad = 0.0f;            // ins.yaw_total_angle * DEGREE_2_RAD
static float dbg_yaw_gyro_raw_radps = 0.0f;     // ins.gyro[2]，原始单位已经是 rad/s
static float dbg_yaw_gyro_signed_radps = 0.0f;  // YAW_GYRO_SIGN * ins.gyro[2]
static float dbg_yaw_rate_err = 0.0f;           // yaw_rate_ref - yaw_rate_fdb
static float dbg_yaw_rc_turning = 0.0f;         // rc_turning
static float dbg_yaw_invalid_flag = 0.0f;       // yaw_invalid
static float dbg_yaw_block_condition = 0.0f;    // block_condition
static float dbg_yaw_unblock_condition = 0.0f;  // unblock_condition
static float dbg_yaw_vx_l = 0.0f;               // chassis_kf_l.FilteredValue[0]
static float dbg_yaw_vx_r = 0.0f;               // chassis_kf_r.FilteredValue[0]
static float dbg_yaw_vx_delta_full = 0.0f;      // fabsf(vx_l - vx_r)
static float dbg_chassis_vx_limit = CHASSIS_V_MAX_LOW;
static float dbg_yaw_rate_limit = YAW_RATE_MAX_LOW_RADPS;

static float dbg_yaw_eff_l = 0.0f;              // 左轮 M3508_control_l 里经过 POS/NEG gain 后的 yaw_eff
static float dbg_yaw_eff_r = 0.0f;              // 右轮 M3508_control_r 里经过 POS/NEG gain 后的 yaw_eff
static float dbg_yaw_mix_l = 0.0f;              // 左轮 yaw_mix = -yaw_eff
static float dbg_yaw_mix_r = 0.0f;              // 右轮 yaw_mix = -yaw_eff
static float dbg_yaw_prio_l = 0.0f;             // 左轮 yaw_prio
static float dbg_yaw_prio_r = 0.0f;             // 右轮 yaw_prio

static float dbg_wheel_speed_raw_l = 0.0f;      // 左 M3508 measure.speed_aps
static float dbg_wheel_speed_raw_r = 0.0f;      // 右 M3508 measure.speed_aps
static float dbg_wheel_speed_phys_l = 0.0f;     // 按 chassis_kf_update 里的左右轮物理符号换算后的轮速 rad/s
static float dbg_wheel_speed_phys_r = 0.0f;     // 按 chassis_kf_update 里的左右轮物理符号换算后的轮速 rad/s
static float dbg_wheel_diff_phys = 0.0f;        // dbg_wheel_speed_phys_r - dbg_wheel_speed_phys_l

/* 积分过程中误差放大倍数，LQR位置的权重太高容易翻车，如果飘逸还是严重，增大此值来补救 */
#define X_RATIO 3.0f //速度积分比例

#define yaw_turn_region 50.0f

/*髋关节电机 DMJ8009P-2EC 实例*/
static dm_motor_object_t *dm_motor[4];

/* 驱动电机 3508 实例 */
static dji_motor_object_t *m3508_motor[2];

/*腿部支持力计算相关参数*/
static float F_bl_gravity ; //重力补偿前馈
static float F_bl_intertial ; //侧向惯性力矩补偿前馈
static float F_roll;
static float F_lateral;
static float dbg_lateral_ff_target;
static lateral_inertial_ff_state_t lateral_ff_state;
static uint8_t dbg_lateral_feedback_mode_allowed;
static uint8_t dbg_lateral_observer_ready;
static uint8_t dbg_lateral_feedback_fresh;
static float F_l_L;
static float F_l_R;
typedef struct {
    float ref_lpf;
    float last_ref_lpf;
    float ref_dot;
    float last_ref_dot;
    float ref_ddot;
    float correction;
    uint8_t initialized;
} support_force_ff_state_t;
static support_force_ff_state_t support_force_ff_state[CHASSIS_SIDE_COUNT];
static float dbg_support_ff_raw_ref[CHASSIS_SIDE_COUNT];
static float dbg_support_ff_correction[CHASSIS_SIDE_COUNT];
static float roll_length_offset;
static float roll_length_offset_target;
static float roll_geometry_settle_timer;
static float dbg_roll_slope_tan;
static float roll_length_target_l;
static float roll_length_target_r;
static float roll_static_trim_force;
static float roll_static_trim_applied;
static float roll_static_trim_timer;
static float dbg_roll_pd_output;
static float dbg_roll_pre_split_guard;
static float dbg_roll_pre_limit_force;
static float roll_spin_target_applied;
static float roll_spin_integral_force[3];
static float roll_spin_integral_applied;
static float roll_spin_integral_settle_timer;
static float spin_leg_transition_rate_limit;
static float spin_leg_transition_settle_timer;
static uint8_t spin_leg_transition_active;
/* Spin speed is selected once on the CHASSIS_SPIN entry edge.  The operator
 * may change leg_level while spinning, but that must never change the locked
 * yaw-rate profile until Spin is exited. */
static leg_level_e spin_entry_leg_level = LEG_LOW;
static float spin_entry_yaw_rate_limit = YAW_SPIN_RATE_MAX_LOW_RADPS;
static float spin_entry_elapsed_s;
static uint8_t spin_entry_latched;
static float spin_leg_slew_rate_applied = LEG_LENGTH_REF_SLEW_RATE;
static float leg_stuck_timer[2];
static float leg_stuck_force[2];
static int8_t leg_stuck_error_sign[2];
static float leg_stuck_last_ms;
static float leg_stuck_post_ramp_timer;
static uint8_t leg_stuck_burst_active[2];
static uint8_t leg_stuck_burst_used[2];
static uint8_t leg_length_reference_ramping_prev;
static uint8_t leg_split_target_guard_active;

#define LEN_JUDGE_REGION  0.02  //辅助判断腿长是否变化完成
#define ROLL_JUDGE_REGION  1.0  //辅助判断腿长是否变化完成,考虑到过单边桥腿长不一致
static int Leg_Left_len_JudgeOK;
static int Leg_right_len_JudgeOK;
static int Body_roll_JudgeOK;
static int Leg_len_JudgeOK;
#define Len_pid_output_LIMIT 80.0f

static pid_obj_t *theta_pid;  // 双腿角度协调控制
static pid_obj_t *theta_comp_pid; // 用于消除零漂的动态 theta 补偿
static pid_obj_t *yaw_pid;    // 航向角控制， 输出为vx的补偿，与vx期望累积
static pid_obj_t *roll_pid;   // 横滚角控制
static pid_obj_t *L_length_pid;  //腿长控制
static pid_obj_t *R_length_pid;  //腿长控制

static float yaw_target;
#define YAW_DELTA_MX 1.396263f //80.0* 0.01745329252 f


#define Location 0      //腿长位置环
#define Speed    1      //腿长速度环

#define WHEEL_RADIUS  0.058f      //轮子半径/m

static leg_obj_t * leg[2] = {NULL};
static float ft_l[2], ft_r[2];  // FT = [PendulumForce PendulumTorque]
static float vmc_out_l[2];  // vmc计算得出的扭矩值 左边腿
static float vmc_out_r[2];  // vmc计算得出的扭矩值 右边腿

typedef enum {
    JUMP_STATE_IDLE = 0,
    JUMP_STATE_PRELOAD,
    JUMP_STATE_THRUST,
    JUMP_STATE_FLIGHT,
    JUMP_STATE_LANDING,
    JUMP_STATE_ABORT_SETTLE,
} jump_state_e;

typedef enum {
    JUMP_FAULT_NONE = 0,
    JUMP_FAULT_COOLDOWN,
    JUMP_FAULT_PRECHECK,
    JUMP_FAULT_PRELOAD_TIMEOUT,
    JUMP_FAULT_NO_TAKEOFF,
    JUMP_FAULT_FLIGHT_TIMEOUT,
    JUMP_FAULT_SAFETY,
    JUMP_FAULT_LANDING_TIMEOUT,
} jump_fault_e;

static jump_state_e jump_state = JUMP_STATE_IDLE;
static jump_fault_e jump_fault = JUMP_FAULT_NONE;
static uint32_t jump_state_enter_ms;
static uint32_t jump_last_accept_ms;
static uint8_t jump_last_accept_valid;
static uint8_t jump_takeoff_seen;
static uint8_t jump_landing_seen;
static uint8_t jump_preload_ready;
static uint8_t jump_thrust_active;
static jump_stroke_release_reason_t jump_stroke_release_reason;
static float jump_stroke_release_elapsed_origin_s;
static uint8_t jump_request_seen;
static float jump_length_target = LEN_LEN_LOW;
static float jump_force_boost;
static float jump_length_sync_force;
static float jump_thrust_pitch_tp;
static float jump_thrust_pitch_rate_lpf;
static float jump_thrust_theta_hold_tp;
static float jump_thrust_theta_ref;
static float jump_thrust_theta_ref_rate;
static float jump_launch_vx;
static float jump_landing_tp_handoff[CHASSIS_SIDE_COUNT];
static float jump_landing_tp_handoff_timer;
static float jump_landing_theta_ref;
static uint8_t jump_landing_owner_theta_ref_valid;
static float jump_landing_theta_hold_tp;
static float jump_vertical_speed_estimate;
static float jump_landing_ground_timer;
/* 2026-09-23 (item 1): leg length captured when LANDING was entered.  The
 * landing confirm used to depend entirely on the filtered contact-force edge,
 * which oscillates during the touchdown rebound (jump13 toggled the off-ground
 * flag 0/1 through the whole capture window and the jump only left LANDING via
 * its 2 s timeout, by which time Pitch had reached -25 deg).  A real compression
 * of both legs from the captured length is direct physical evidence of ground
 * contact and is immune to contact-estimator bias. */
static float jump_landing_entry_length;
/* Roll differential PD force used during the jump (item 4).  Same law and gains
 * as the reference implementation: a bounded common-mode force pair that cannot
 * change the total support, only the left/right split. */
static float jump_roll_diff_force;
/* 2026-09-23 item 2: milliseconds since the current jump was accepted, spanning
 * every state.  See JUMP_TOTAL_TIMEOUT. */
static uint32_t jump_sequence_start_ms;
/* 2026-09-23: airborne leg-angle reference driven by the jump state machine and
 * consumed by the airborne joint controller. */
static float jump_air_theta_ref;
static uint8_t jump_air_theta_ref_valid;
/* Logical common reaction-wheel torque used only during active jump flight.
 * The normal LQR wheel row is still shut off in the air; this separate
 * channel mirrors the validated MuJoCo jump allocation. */
static float jump_air_wheel_torque;
/* Last source selected by the final M3508 output callbacks (0=zero, 1=LQR,
 * 2=reaction wheel, 3=landing damper), for the dedicated validation page. */
static uint8_t dbg_jump_wheel_output_source = 0U;
static uint8_t dbg_jump_handoff_wheel_source = 0U;
/* Page-56 diagnostics for the airborne wheel channel (see the assignment in
 * jump_air_wheel_control_update).  They are latched every cycle so the log shows
 * the channel's inputs even when its output is zero. */
static float jump_air_wheel_pitch_ref_rad;
static float jump_air_wheel_pitch_err_rad;
static float jump_air_wheel_pitch_rate;
/* Logical common landing-buffer wheel torque.  COMPRESS/HOLD/RETURN keep the
 * bounded body-pitch damper in charge; after the buffer releases, the normal
 * ground-LQR wheel row returns through the explicit handoff fade. */
static float jump_landing_wheel_torque;
static uint8_t jump_landing_return_ramp_active;
static float jump_landing_lqr_return_gain; /* 0..1 authority of the LQR wheel row */
static jump_landing_wheel_fade_t jump_landing_wheel_fade;
static float jump_landing_wheel_fade_duration_s =
    JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
static uint8_t jump_landing_buffer_owner_previous;
static uint8_t jump_landing_wheel_fade_started_this_cycle;
static uint8_t jump_landing_wheel_last_owner_command_valid[CHASSIS_SIDE_COUNT];
static float jump_landing_wheel_last_owner_command[CHASSIS_SIDE_COUNT];
static float jump_landing_base_tp[CHASSIS_SIDE_COUNT];
static float jump_landing_owner_gain = 1.0f;
/* Latest asynchronous M3508 callback blend coefficient shown on Page59. */
static float jump_landing_wheel_last_used_gain = 1.0f;
/* Published once per chassis cycle from the grounded hip-owner blend. The
 * asynchronous CAN callbacks use a completed-cycle value; zero makes first
 * ownership fail closed to the existing landing damper. */
static volatile float jump_landing_wheel_buffer_return_gain = 0.0f;
static jump_landing_balance_state_t jump_landing_balance_state;
static volatile float jump_landing_ground_balance_gain = 0.0f;
static jump_ground_recovery_t jump_ground_recovery_state;
static volatile float jump_ground_recovery_gain;
static volatile uint8_t jump_ground_recovery_scope_active;
static volatile float jump_ground_wheel_common_cache;
static volatile uint8_t jump_ground_wheel_cache_valid;
static void jump_control_fail_to_recovery(jump_fault_e fault);
static uint8_t jump_landing_balance_apply_lqr_refs(void);

/* The first call advances the independent posture clock once per chassis
 * cycle. A later safety landing edge may call with advance=0 to capture from
 * the newly confirmed state without consuming the cycle twice. */
static uint8_t jump_landing_balance_update(float dt_s, uint8_t advance)
{
    const uint8_t owner = chassis_safety_module_jump_landing_owner_active();
    const uint8_t buffer = chassis_safety_module_landing_buffer_active();
    const uint8_t off_ground = chassis_safety_module_off_ground();
    const uint8_t confirmed = (uint8_t)(owner && buffer &&
        chassis_safety_module_landing_gain() > 0.0f);
    jump_landing_balance_result_t result = jump_landing_balance_step(
        &jump_landing_balance_state, advance ? dt_s : 0.0f,
        owner, confirmed, off_ground,
        0.5f * (leg[LEFT]->theta + leg[RIGHT]->theta));
    if (result.invalid_capture) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        jump_landing_ground_balance_gain = 0.0f;
        return 0U;
    }
    if (owner && jump_landing_balance_state.capture_valid &&
        !isfinite(jump_landing_balance_state.captured_theta)) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        jump_landing_ground_balance_gain = 0.0f;
        return 0U;
    }
    if (result.captured_this_step) {
        jump_landing_theta_ref = result.captured_theta;
        jump_landing_owner_theta_ref_valid = 1U;
    }
    /* The geometric ramp reaches one in 40 ms, but the measured touchdown
     * data show that the body can still be at 15--16 degrees then.  Keep the
     * damper in charge until posture and speed have settled; otherwise the
     * normal LQR immediately drives the wheels into alternating saturation. */
    if (owner && confirmed && !off_ground) {
        const float length_diff = fabsf(leg[LEFT]->l0 - leg[RIGHT]->l0);
        const uint8_t posture_stable =
            isfinite(ins.pitch) && isfinite(ins.roll) &&
            fabsf(ins.pitch) <= JUMP_LANDING_OWNER_PITCH_STABLE_DEG &&
            fabsf(ins.roll) <= JUMP_LANDING_OWNER_ROLL_STABLE_DEG &&
            isfinite(length_diff) &&
            length_diff <= JUMP_LANDING_OWNER_LENGTH_DIFF_MAX;
        const float gain_limit = posture_stable && isfinite(chassis_vx_filter) ?
            1.0f :
            JUMP_LANDING_OWNER_EARLY_GAIN_LIMIT;
        jump_landing_ground_balance_gain =
            result.gain < gain_limit ? result.gain : gain_limit;
    } else {
        jump_landing_ground_balance_gain = result.gain;
    }
    return 1U;
}

static uint8_t jump_ground_recovery_update(float dt_s, uint8_t advance)
{
    const uint8_t normal_mode =
        (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
         chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL) &&
        (chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
         chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);
    const uint8_t jump_mode =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        (chassis_fdb_data.stand_state == CHASSIS_IS_JUMPING ||
         chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);
    const uint8_t safe_mode = (uint8_t)(normal_mode || jump_mode);
    const uint8_t contact =
        chassis_safety_module_jump_landing_owner_active() &&
        chassis_safety_module_landing_buffer_active() &&
        chassis_safety_module_landing_buffer_jump_origin() &&
        chassis_safety_module_landing_gain() > 0.0f;
#if JUMP_LOCAL_RATE_FEEDBACK_ENABLE && JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    const uint8_t observer_valid = (uint8_t)(jump_kin_rates_fresh &&
        jump_kin_rates_usable && isfinite(jump_kin_theta_rate[LEFT]) &&
        isfinite(jump_kin_theta_rate[RIGHT]));
#else
    const uint8_t observer_valid = 0U;
#endif
    jump_ground_recovery_result_t result = jump_ground_recovery_step(
        &jump_ground_recovery_state, advance ? dt_s : 0.0f, advance,
        contact, chassis_safety_module_landing_buffer_active(),
        !chassis_safety_module_off_ground(), safe_mode, observer_valid,
        jump_kin_theta_rate[LEFT], jump_kin_theta_rate[RIGHT]);
    if (result.invalid_observer) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        jump_ground_recovery_gain = 0.0f;
        jump_ground_recovery_scope_active = 0U;
        jump_ground_wheel_cache_valid = 0U;
        return 0U;
    }
    jump_ground_recovery_gain = result.gain;
    jump_ground_recovery_scope_active = jump_ground_recovery_state.latched;
    if (!jump_ground_recovery_state.latched) {
        jump_ground_wheel_cache_valid = 0U;
    }
    return 1U;
}

static uint8_t jump_ground_wheel_cache_publish(void);

static uint8_t jump_landing_wheel_handoff_mode_allowed(void)
{
    if (chassis_safety_module_off_ground() ||
        chassis_fdb_data.stand_state == CHASSIS_IS_DANGER) {
        return 0U;
    }
    const uint8_t normal_mode =
        (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
         chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL) &&
        (chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
         chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);
    const uint8_t jump_mode =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        (chassis_fdb_data.stand_state == CHASSIS_IS_JUMPING ||
         chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK);
    return (uint8_t)(normal_mode || jump_mode);
}

static uint8_t jump_landing_postbuffer_normal_base_active(void)
{
    return (uint8_t)(jump_landing_owner_theta_ref_valid &&
        !chassis_safety_module_landing_buffer_active() &&
        !chassis_safety_module_off_ground() &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER);
}

static uint8_t lateral_ff_feedback_fresh(void)
{
    if (control_context == NULL ||
        control_context->wheel_feedback == NULL) {
        return 0U;
    }
    const struct ins_msg *ins_sample = &ins;
    if (ins_sample == NULL) {
        return 0U;
    }

    const uint32_t now_ms = HAL_GetTick();
    if (!lateral_inertial_ff_sample_fresh(
            now_ms, ins_sample->publish_tick_ms,
            ins_sample->publish_count,
            LATERAL_FF_INS_MAX_AGE_MS)) {
        return 0U;
    }
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        const dji_motor_measure_t *wheel =
            &control_context->wheel_feedback[side];
        if (!lateral_inertial_ff_sample_fresh(
                now_ms, wheel->last_rx_ms, wheel->rx_count,
                LATERAL_FF_WHEEL_MAX_AGE_MS)) {
            return 0U;
        }
    }
    for (uint8_t motor = 0U; motor < 4U; motor++) {
        if (dm_motor[motor] == NULL ||
            !lateral_inertial_ff_sample_fresh(
                now_ms, dm_motor[motor]->measure.last_rx_ms,
                dm_motor[motor]->measure.rx_count,
                LATERAL_FF_HIP_MAX_AGE_MS)) {
            return 0U;
        }
    }
    return 1U;
}


/* BUMP 与主动跳跃完全独立；不要将这些状态加入 jump_state 或任何 LQR 档位索引。
 *
 * 2026-10-07 重构（按操作手指定的动作序列）：
 *   接触 -> phi0 到 -10° -> 保持角收腿到 0.13 -> phi0 到 80° -> 起立
 * 原来的 HIP_BACK/HIP_FRONT/EXTEND（先收腿再荡髋）被替换掉，因为实测
 * (bump3.csv) 证明「腿还斜着(θ≈0.35~0.65 rad)就收腿」会把机身顶翻：
 * t=38.903 接触 -> t=39.223 pitch 已到 24.6° 触发全局中止 -> 前扑倒地。
 * 反算 phi0 显示腿杆只转了 3.6°，而 theta/pitch 各转了约 20° —— 是机身绕髋
 * 前倾，腿杆原地不动。先把 theta 摆到 0（腿与机身竖直对齐）再收腿，就不会
 * 产生那个前倾力矩。
 *
 * 状态号语义已变，页 54 的 I2 对照见 chassis_control.c 页 54 填充处的注释。 */
typedef enum {
    BUMP_ABORT_NONE = 0,
    BUMP_ABORT_MODE,
    BUMP_ABORT_FEEDBACK,
    BUMP_ABORT_DANGER,
    BUMP_ABORT_ATTITUDE,
    BUMP_ABORT_PHI0_RANGE,
    BUMP_ABORT_SWING_OVER_TIMEOUT,
    BUMP_ABORT_RETRACT_TIMEOUT,
    BUMP_ABORT_SWING_SUPPORT_TIMEOUT,
    BUMP_ABORT_STAND_TIMEOUT,
} bump_abort_reason_e;

typedef struct {
    uint32_t rx_count[2];
    float last_phi0;
    float last_time_ms;
    float rate;
    float sample_age_ms;
    uint8_t primed;
    uint8_t valid;
} bump_phi0_rate_state_t;

typedef struct {
    float phi0[CHASSIS_SIDE_COUNT];
    float phi0_rate[CHASSIS_SIDE_COUNT];
    float theta[CHASSIS_SIDE_COUNT];
    float theta_rate[CHASSIS_SIDE_COUNT];
    float length[CHASSIS_SIDE_COUNT];
    float length_rate[CHASSIS_SIDE_COUNT];
    float hold_length_target[CHASSIS_SIDE_COUNT];
    float normal_length_target[CHASSIS_SIDE_COUNT];
    float pitch_deg;
    float roll_deg;
    float dt_s;
    uint8_t ground_contact;
    uint8_t feedback_valid;
} bump_stand_feedback_t;

typedef struct {
    float support_ready_ms;
    float handoff_ready_ms;
    float finish_ms;
    float extend_gain;
    float support_gain;
    float handoff_gain;
    uint8_t extension_started;
    uint8_t support_started;
    uint8_t handoff_started;
} bump_stand_handoff_state_t;

static bump_state_e bump_state = BUMP_STATE_IDLE;
static uint32_t bump_state_enter_ms;
static float bump_state_elapsed_s_accum;
static uint8_t bump_request_consumed;
static bump_abort_reason_e bump_abort_reason;
static uint8_t bump_contact_latched[CHASSIS_SIDE_COUNT];
static float bump_contact_timer[CHASSIS_SIDE_COUNT];
static uint32_t bump_contact_last_rx_count[CHASSIS_SIDE_COUNT];
static float bump_contact_pair_window_ms;
static uint8_t bump_approach_speed_seen;
static float bump_approach_stable_ms;
static float bump_hip_tp[CHASSIS_SIDE_COUNT];
static float bump_target_length = LEN_LEN_LOW;
static float bump_length_ref[CHASSIS_SIDE_COUNT] = {LEN_LEN_LOW, LEN_LEN_LOW};
static float bump_length_target_effective[CHASSIS_SIDE_COUNT] = {
    LEN_LEN_LOW, LEN_LEN_LOW
};
static float bump_done_length_target[CHASSIS_SIDE_COUNT] = {
    BUMP_APPROACH_LENGTH, BUMP_APPROACH_LENGTH
};
static uint8_t bump_length_ref_initialized;
static uint8_t bump_airborne_lockout_seen;
static uint8_t bump_retract_angle_reacquire;
static uint8_t bump_stand_angle_reacquire;
static uint8_t bump_wheel_handoff_active;
static uint8_t bump_buffer_seen;
static uint8_t bump_ins_feedback_fresh;
static float bump_wheel_handoff_gain = 1.0f;
static float bump_wheel_handoff_torque;
/* Per-leg phi0 references stay anchored to fresh measured joint geometry. */
static float bump_phi0_ref[CHASSIS_SIDE_COUNT];
static float bump_phi0_hold[CHASSIS_SIDE_COUNT];
static float bump_phi0_ref_rate[CHASSIS_SIDE_COUNT];
static float bump_angle_stable_timer[CHASSIS_SIDE_COUNT];
static float bump_length_stable_timer[CHASSIS_SIDE_COUNT];
static bump_phi0_rate_state_t bump_phi0_rate_state[CHASSIS_SIDE_COUNT];
static uint8_t bump_retract_breakaway_active[CHASSIS_SIDE_COUNT];
static float bump_retract_breakaway_elapsed[CHASSIS_SIDE_COUNT];
static uint8_t bump_retract_breakaway_handoff[CHASSIS_SIDE_COUNT];
static float bump_retract_breakaway_handoff_elapsed[CHASSIS_SIDE_COUNT];
static uint8_t bump_retract_lower_latched[CHASSIS_SIDE_COUNT];
static float bump_stand_support_gain = 1.0f;
static float bump_stand_handoff_gain = 1.0f;
static float bump_stand_extend_gain = 1.0f;
static bump_stand_handoff_state_t bump_stand_handoff;

static float move_towards_f32(float current, float target, float max_step);
static void bump_stand_lqr_seed(void);
static void bump_motion_reference_zero(void);
static void bump_state_enter(bump_state_e state);

static float bump_retract_breakaway_force(float elapsed, float length)
{
    if (!isfinite(elapsed) || elapsed <= 0.0f) {
        return 0.0f;
    }
    /* The old implementation dropped the whole pulse at a fixed 180 ms.
     * Keep the high force until the measured leg crosses the spring release
     * band; elapsed time only controls the initial 20 ms ramp. */
    float force = bump_retract_breakaway_force_magnitude(
        elapsed, BUMP_RETRACT_BREAKAWAY_RAMP_TIME,
        BUMP_RETRACT_BREAKAWAY_FORCE);
    if (elapsed >= BUMP_RETRACT_BREAKAWAY_RAMP_TIME &&
        isfinite(length) && length <= BUMP_RETRACT_BREAKAWAY_RELEASE_L0) {
        force = bump_retract_breakaway_taper_force(
            length, BUMP_RETRACT_BREAKAWAY_RELEASE_L0,
            BUMP_RETRACT_BREAKAWAY_RELEASE_BAND,
            BUMP_RETRACT_BREAKAWAY_FORCE, BUMP_LENGTH_OUTPUT_LIMIT);
    }
    return force;
}

/* Preserve the hip angle torque before applying the retract force.  This is
 * the same allocation rule used by recovery: both actuator torque components
 * must retain the requested reserve, so the axial force is scaled as a whole
 * instead of clipping one motor independently. */
static float bump_force_with_torque_reserve(leg_obj_t *leg_obj,
                                            float force,
                                            float Tp,
                                            float torque_reserve)
{
    float tp_torque[2] = {0.0f, 0.0f};
    float force_torque[2] = {0.0f, 0.0f};
    float force_scale = 1.0f;

    if (leg_obj == NULL || !isfinite(force) || !isfinite(Tp) ||
        !isfinite(torque_reserve) || torque_reserve < 0.0f) {
        return 0.0f;
    }

    leg_obj->support_force = 0.0f;
    leg_obj->Tp = Tp;
    leg_obj->vmc_cal_T(leg_obj, tp_torque);

    leg_obj->support_force = force;
    leg_obj->Tp = 0.0f;
    leg_obj->vmc_cal_T(leg_obj, force_torque);

    for (uint8_t i = 0U; i < 2U; i++) {
        const float tp_component = tp_torque[i];
        const float force_component = force_torque[i];
        if (fabsf(tp_component) >= torque_reserve) {
            force_scale = 0.0f;
            break;
        }
        if (fabsf(force_component) > 1e-4f) {
            const float limit_scale = (force_component > 0.0f) ?
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

static void bump_retract_breakaway_update(float dt)
{
    if (bump_state != BUMP_STATE_RETRACT || !isfinite(dt) || dt <= 0.0f) {
        memset(bump_retract_breakaway_active, 0,
               sizeof(bump_retract_breakaway_active));
        memset(bump_retract_breakaway_elapsed, 0,
               sizeof(bump_retract_breakaway_elapsed));
        memset(bump_retract_breakaway_handoff, 0,
               sizeof(bump_retract_breakaway_handoff));
        memset(bump_retract_breakaway_handoff_elapsed, 0,
               sizeof(bump_retract_breakaway_handoff_elapsed));
        /* Carry the lower-stop evidence through SWING_SUPPORT and the initial
         * STAND handoff.  Clearing it before support gain reaches one removes
         * the -110 N one-way hold and lets the gas spring reopen the leg. */
        if (bump_state != BUMP_STATE_SWING_SUPPORT &&
            bump_state != BUMP_STATE_STAND) {
            memset(bump_retract_lower_latched, 0,
                   sizeof(bump_retract_lower_latched));
        }
        return;
    }
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        const float length = leg[side] != NULL ? leg[side]->l0 : NAN;
        bump_retract_lower_latch_update(
            &bump_retract_lower_latched[side], length,
            BUMP_RETRACT_LENGTH - BUMP_RETRACT_TOLERANCE);
        if (bump_retract_lower_latched[side]) {
            bump_retract_breakaway_active[side] = 0U;
            bump_retract_breakaway_handoff[side] = 0U;
            bump_retract_breakaway_handoff_elapsed[side] = 0.0f;
            continue;
        }
        if (!bump_retract_breakaway_active[side]) {
            continue;
        }
        const uint8_t was_handoff = bump_retract_breakaway_handoff[side];
        bump_retract_breakaway_t state = {
            bump_retract_breakaway_active[side], 0U,
            bump_retract_breakaway_handoff[side],
            bump_retract_breakaway_elapsed[side],
            bump_retract_breakaway_handoff_elapsed[side],
        };
        bump_retract_breakaway_step(
            &state, 1U,
            chassis_safety_module_bump_airborne_lockout(),
            dt, length,
            BUMP_RETRACT_BREAKAWAY_RELEASE_L0,
            BUMP_RETRACT_BREAKAWAY_RELEASE_BAND,
            BUMP_RETRACT_BREAKAWAY_MIN_TIME,
            BUMP_RETRACT_BREAKAWAY_HANDOFF_TIME);
        bump_retract_breakaway_active[side] = state.active;
        bump_retract_breakaway_handoff[side] = state.handoff_active;
        bump_retract_breakaway_elapsed[side] = state.elapsed_s;
        bump_retract_breakaway_handoff_elapsed[side] =
            state.handoff_elapsed_s;
        if (!was_handoff && state.handoff_active) {
            /* A fast leg can outrun the reference ramp. Capture the
             * already-compressed geometry so the length PID cannot push
             * it back open at the force handoff. */
            bump_length_ref[side] = bump_retract_reference_capture(
                bump_length_ref[side], length, BUMP_RETRACT_LENGTH);
        }
    }
}

static uint8_t bump_pair_angle_targets_reached(
    const float target[2], float angle_tolerance)
{
    /* The next stage captures the measured pose. Do not wait for zero
     * angular speed or a dwell once both legs have reached the target. */
    const float actual[CHASSIS_SIDE_COUNT] = {
        leg[LEFT]->phi0, leg[RIGHT]->phi0
    };
    return (uint8_t)(bump_phi0_rate_state[LEFT].valid &&
                     bump_phi0_rate_state[RIGHT].valid &&
                     bump_pair_angle_gate(target, actual, angle_tolerance));
}

static void bump_lockout_resume_capture(void)
{
    if (!bump_airborne_lockout_seen ||
        chassis_safety_module_bump_airborne_lockout()) {
        return;
    }
    const uint8_t resume_stand = bump_state == BUMP_STATE_STAND;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        bump_length_ref[side] = isfinite(leg[side]->l0) ?
            leg[side]->l0 : BUMP_RETRACT_LENGTH;
        bump_phi0_ref[side] = isfinite(leg[side]->phi0) ?
            leg[side]->phi0 : bump_phi0_hold[side];
        bump_phi0_ref_rate[side] = 0.0f;
        bump_angle_stable_timer[side] = 0.0f;
        bump_length_stable_timer[side] = 0.0f;
    }
    bump_length_ref_initialized = 1U;
    bump_retract_angle_reacquire =
        (uint8_t)(bump_state == BUMP_STATE_RETRACT &&
            (!bump_angle_gate(bump_phi0_hold[LEFT], leg[LEFT]->phi0,
                              BUMP_SWING_ANGLE_OK) ||
             !bump_angle_gate(bump_phi0_hold[RIGHT], leg[RIGHT]->phi0,
                              BUMP_SWING_ANGLE_OK)));
    bump_stand_angle_reacquire = (uint8_t)(resume_stand &&
        (!bump_angle_gate(BUMP_SWING_SUPPORT_PHI0_RAD,
                          leg[LEFT]->phi0, BUMP_SWING_ANGLE_OK) ||
         !bump_angle_gate(BUMP_SWING_SUPPORT_PHI0_RAD,
                          leg[RIGHT]->phi0, BUMP_SWING_ANGLE_OK)));
    if (resume_stand) {
        memset(&bump_stand_handoff, 0, sizeof(bump_stand_handoff));
        bump_stand_extend_gain = 0.0f;
        bump_stand_support_gain = 0.0f;
        bump_stand_handoff_gain = 0.0f;
        bump_stand_lqr_seed();
    }
    bump_airborne_lockout_seen = 0U;
}

static uint8_t bump_control_active(void)
{
    /* STAND is a persistent switch latch only.  RETRACT has finished the
     * obstacle-specific motion there, so the ordinary LOW-length LQR must
     * regain all normal support, roll, and wheel-control paths immediately.
     * Keep the enum value itself alive so a held switch does not retrigger;
     * release handling below returns it to IDLE and arms the next edge. */
    return (uint8_t)(bump_state != BUMP_STATE_IDLE &&
                     bump_state != BUMP_STATE_STAND);
}

static uint8_t bump_action_active(void)
{
    return (uint8_t)(bump_state == BUMP_STATE_CONTACT ||
                     bump_state == BUMP_STATE_SWING_OVER_BODY ||
                     bump_state == BUMP_STATE_RETRACT ||
                     bump_state == BUMP_STATE_SWING_SUPPORT);
}

static float bump_state_elapsed_s(void)
{
    return bump_state_elapsed_s_accum;
}

static void bump_abort(bump_abort_reason_e reason)
{
    bump_abort_reason = reason;
    if (bump_state != BUMP_STATE_ABORT) {
        bump_state_enter(BUMP_STATE_ABORT);
    }
}

/* 2026-10-07 (操作手要求)：BUMP 在哪些阶段需要关断轮子。
 * APPROACH 保持驱动（要开过去撞台阶）；一旦检测到接触，其后整个爬台阶序列
 * （CONTACT/SWING_OVER/RETRACT/SWING_SUPPORT）都关轮子；STAND 交回低腿长普通控制；
 * DONE 交回普通控制；
 * ABORT 关断。 */
static uint8_t bump_wheel_shut_requested(void)
{
    return (uint8_t)(bump_state == BUMP_STATE_ABORT ||
                     bump_state == BUMP_STATE_CONTACT ||
                     bump_state == BUMP_STATE_SWING_OVER_BODY ||
                     bump_state == BUMP_STATE_RETRACT ||
                     bump_state == BUMP_STATE_SWING_SUPPORT);
}

static void bump_state_enter(bump_state_e state)
{
    bump_state = state;
    bump_state_enter_ms = HAL_GetTick();
    bump_state_elapsed_s_accum = 0.0f;
    memset(bump_contact_timer, 0, sizeof(bump_contact_timer));
    memset(bump_contact_last_rx_count, 0, sizeof(bump_contact_last_rx_count));
    bump_contact_pair_window_ms = 0.0f;
    bump_approach_stable_ms = 0.0f;
    memset(bump_retract_breakaway_active, 0,
           sizeof(bump_retract_breakaway_active));
    memset(bump_retract_breakaway_elapsed, 0,
           sizeof(bump_retract_breakaway_elapsed));
    memset(bump_retract_breakaway_handoff, 0,
           sizeof(bump_retract_breakaway_handoff));
    memset(bump_retract_breakaway_handoff_elapsed, 0,
           sizeof(bump_retract_breakaway_handoff_elapsed));
    if (state != BUMP_STATE_SWING_SUPPORT && state != BUMP_STATE_STAND) {
        memset(bump_retract_lower_latched, 0,
               sizeof(bump_retract_lower_latched));
    }
    if (state == BUMP_STATE_APPROACH) {
        bump_approach_speed_seen = 0U;
        bump_buffer_seen = 0U;
        bump_abort_reason = BUMP_ABORT_NONE;
        bump_airborne_lockout_seen = 0U;
        bump_retract_angle_reacquire = 0U;
        bump_stand_angle_reacquire = 0U;
        memset(bump_phi0_rate_state, 0, sizeof(bump_phi0_rate_state));
        memset(bump_contact_latched, 0, sizeof(bump_contact_latched));
        bump_target_length = BUMP_APPROACH_LENGTH;
        bump_length_ref_initialized = 1U;
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            const float current = leg[side] != NULL && isfinite(leg[side]->l0) ?
                leg[side]->l0 : BUMP_RETRACT_LENGTH;
            bump_length_ref[side] = current;
        }
    } else if (state == BUMP_STATE_RETRACT ||
               state == BUMP_STATE_SWING_SUPPORT) {
        bump_target_length = BUMP_RETRACT_LENGTH;
    } else if (state == BUMP_STATE_SWING_OVER_BODY) {
        /* Preserve each measured leg length while the hips swing over-body. */
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            if (leg[side] != NULL && isfinite(leg[side]->l0)) {
                bump_length_ref[side] = leg[side]->l0;
            }
        }
        bump_target_length = 0.5f *
            (bump_length_ref[LEFT] + bump_length_ref[RIGHT]);
    } else if (state == BUMP_STATE_STAND) {
        /* STAND is the terminal BUMP latch.  RETRACT has already established
         * the low-leg geometry; from this point the normal LOW LQR owns both
         * length and axial support.  Keep the references at LEN_LEN_LOW so a
         * held BUMP switch cannot re-open the legs or restart the sequence. */
        bump_target_length = LEN_LEN_LOW;
        bump_stand_support_gain = 1.0f;
        bump_stand_handoff_gain = 1.0f;
        bump_stand_extend_gain = 1.0f;
        memset(&bump_stand_handoff, 0, sizeof(bump_stand_handoff));
        bump_stand_angle_reacquire = 0U;
        bump_length_ref[LEFT] = isfinite(leg[LEFT]->l0) ?
            fminf(leg[LEFT]->l0, LEN_LEN_LOW) : LEN_LEN_LOW;
        bump_length_ref[RIGHT] = isfinite(leg[RIGHT]->l0) ?
            fminf(leg[RIGHT]->l0, LEN_LEN_LOW) : LEN_LEN_LOW;
        /* Reconfirm the lower-stop evidence at the SWING_SUPPORT -> STAND
         * handoff.  The latch is intentionally sticky across both states,
         * but this also recovers it when the first STAND sample arrives with
         * a short leg after a delayed state transition. */
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_retract_lower_latch_update(
                &bump_retract_lower_latched[side], leg[side]->l0,
                BUMP_RETRACT_LENGTH - BUMP_RETRACT_TOLERANCE);
        }
        chassis_cmd.leg_level = LEG_LOW;
        bump_stand_lqr_seed();
    } else if (state == BUMP_STATE_DONE) {
        bump_target_length = 0.5f *
            (leg[LEFT]->length_ref + leg[RIGHT]->length_ref);
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_done_length_target[side] = isfinite(leg[side]->length_ref) ?
                leg[side]->length_ref : BUMP_APPROACH_LENGTH;
            bump_length_ref[side] = bump_done_length_target[side];
        }
        bump_approach_stable_ms = 0.0f;
    } else if (state == BUMP_STATE_ABORT) {
        bump_target_length = BUMP_ABORT_LENGTH;
    }
    if (state == BUMP_STATE_SWING_OVER_BODY) {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_phi0_ref[side] = leg[side]->phi0;
            bump_phi0_hold[side] = leg[side]->phi0;
            bump_phi0_ref_rate[side] = 0.0f;
            bump_angle_stable_timer[side] = 0.0f;
        }
    } else if (state == BUMP_STATE_RETRACT) {
        bump_retract_angle_reacquire = 0U;
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_phi0_hold[side] = leg[side]->phi0;
            bump_phi0_ref[side] = bump_phi0_hold[side];
            bump_phi0_ref_rate[side] = 0.0f;
            bump_length_stable_timer[side] = 0.0f;
            /* A leg that is already below the release band has passed the
             * spring dead point; only the longer leg needs the high pulse. */
            bump_retract_breakaway_active[side] =
                (uint8_t)(isfinite(leg[side]->l0) &&
                    leg[side]->l0 >
                    (BUMP_RETRACT_BREAKAWAY_RELEASE_L0 -
                     BUMP_RETRACT_BREAKAWAY_RELEASE_BAND));
            bump_retract_breakaway_handoff[side] = 0U;
            bump_retract_breakaway_handoff_elapsed[side] = 0.0f;
            bump_retract_lower_latched[side] = 0U;
        }
    } else if (state == BUMP_STATE_SWING_SUPPORT) {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_phi0_ref[side] = leg[side]->phi0;
            bump_phi0_ref_rate[side] = 0.0f;
            bump_angle_stable_timer[side] = 0.0f;
        }
    }
    if (state != BUMP_STATE_SWING_OVER_BODY &&
        state != BUMP_STATE_RETRACT &&
        state != BUMP_STATE_SWING_SUPPORT &&
        state != BUMP_STATE_STAND) {
        bump_hip_tp[LEFT] = 0.0f;
        bump_hip_tp[RIGHT] = 0.0f;
    }
    if (state == BUMP_STATE_IDLE) {
        bump_buffer_seen = 0U;
        bump_wheel_handoff_active = 0U;
        bump_wheel_handoff_gain = 1.0f;
        bump_wheel_handoff_torque = 0.0f;
    }
}

static void bump_length_reference_update(void)
{
    if (!bump_control_active() || leg[LEFT] == NULL || leg[RIGHT] == NULL ||
        chassis_safety_module_bump_airborne_lockout()) {
        return;
    }

    float dt = jump_cycle_dt_s;
    if (!isfinite(dt) || dt < 0.0002f || dt > 0.020f) {
        dt = 0.001f;
    }
    if (!bump_length_ref_initialized) {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_length_ref[side] = isfinite(leg[side]->length_ref) ?
                leg[side]->length_ref : BUMP_RETRACT_LENGTH;
        }
        bump_length_ref_initialized = 1U;
    }
    const float max_step = BUMP_LENGTH_REF_SLEW_RATE * dt;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        const uint8_t retract_target = bump_state == BUMP_STATE_RETRACT;
        const float target = retract_target ? BUMP_RETRACT_LENGTH :
            (bump_state == BUMP_STATE_STAND ||
             bump_state == BUMP_STATE_SWING_OVER_BODY) ?
            bump_length_ref[side] :
            (bump_state == BUMP_STATE_DONE ?
                bump_done_length_target[side] : bump_target_length);
        bump_length_ref[side] = retract_target ? BUMP_RETRACT_LENGTH :
            move_towards_f32(bump_length_ref[side], target, max_step);
    }
}

static uint8_t bump_length_target_owner_active(void)
{
    /* After RETRACT, STAND is ordinary LOW operation.  Do not let the BUMP
     * reference owner keep overriding chassis_task's LOW target. */
    if (bump_state == BUMP_STATE_STAND) {
        return 0U;
    }
    if (chassis_safety_module_bump_pose_owner_active()) {
        return 1U;
    }
    const uint8_t higher_priority_owner =
        chassis_safety_module_off_ground() ||
        chassis_safety_module_landing_buffer_active() ||
        chassis_safety_module_landing_postbuffer_active() ||
        chassis_safety_module_jump_landing_owner_active() ||
        chassis_safety_module_bump_airborne_lockout() ||
        chassis_cmd.ctrl_mode == CHASSIS_JUMP;
    return (uint8_t)(bump_control_active() && !higher_priority_owner);
}

static void bump_length_target_select(
    uint8_t bump_owner,
    const float normal_target[CHASSIS_SIDE_COUNT],
    float selected_target[CHASSIS_SIDE_COUNT])
{
    if (normal_target == NULL || selected_target == NULL) {
        return;
    }
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        selected_target[side] = normal_target[side];
        if (bump_owner && bump_state != BUMP_STATE_STAND) {
            selected_target[side] = bump_state == BUMP_STATE_RETRACT ?
                BUMP_RETRACT_LENGTH :
                bump_length_ref[side];
        }
    }
}

static void bump_wheel_handoff_update(void)
{
    float dt = jump_cycle_dt_s;
    if (!isfinite(dt) || dt < 0.0002f || dt > 0.020f) {
        dt = 0.001f;
    }

    if (!bump_control_active() && !bump_wheel_handoff_active) {
        bump_wheel_handoff_gain = 1.0f;
        bump_wheel_handoff_torque = 0.0f;
        return;
    }

    const uint8_t airborne = chassis_safety_module_off_ground() &&
        (!bump_action_active() ||
         chassis_safety_module_bump_airborne_lockout());
    const uint8_t buffering =
        chassis_safety_module_landing_buffer_active() &&
        (!bump_action_active() ||
         chassis_safety_module_bump_airborne_lockout());
    if (airborne || buffering) {
        const float pitch = ins.pitch * DEGREE_2_RAD;
        const float pitch_rate = ins.pitch_rate_rad_s;
        float target = (isfinite(pitch) && isfinite(pitch_rate)) ?
            BUMP_LANDING_WHEEL_KP * pitch +
            BUMP_LANDING_WHEEL_KD * pitch_rate : 0.0f;
        LIMIT_MIN_MAX(target, -BUMP_LANDING_WHEEL_LIMIT,
                       BUMP_LANDING_WHEEL_LIMIT);
        const float step = BUMP_LANDING_WHEEL_SLEW_RATE * dt;
        bump_wheel_handoff_torque = move_towards_f32(
            bump_wheel_handoff_torque, target, step);
        bump_wheel_handoff_active = 1U;
        bump_wheel_handoff_gain = 0.0f;
        return;
    }

    if (bump_wheel_handoff_active) {
        bump_wheel_handoff_gain += dt / BUMP_LANDING_WHEEL_RELEASE_TIME;
        if (bump_wheel_handoff_gain >= 1.0f) {
            bump_wheel_handoff_active = 0U;
            bump_wheel_handoff_gain = 1.0f;
            bump_wheel_handoff_torque = 0.0f;
        }
    }
}

static float bump_wheel_handoff_mix(float normal_torque)
{
    if (!bump_wheel_handoff_active) {
        return normal_torque;
    }
    return bump_wheel_handoff_torque + bump_wheel_handoff_gain *
        (normal_torque - bump_wheel_handoff_torque);
}

static uint8_t bump_wheel_feedback_valid(uint8_t side)
{
    if (control_context == NULL || control_context->wheel_feedback == NULL ||
        side >= CHASSIS_SIDE_COUNT ||
        control_context->wheel_feedback[side].rx_count == 0U) {
        return 0U;
    }
    return (uint32_t)(HAL_GetTick() -
                      control_context->wheel_feedback[side].last_rx_ms) <= 30U;
}

static uint8_t bump_contact_condition(uint8_t side, float torque_threshold)
{
    if (!bump_wheel_feedback_valid(side) || leg[side] == NULL ||
        !isfinite(leg[side]->theta)) {
        return 0U;
    }
    const dji_motor_measure_t *wheel = &control_context->wheel_feedback[side];
    const float wheel_torque = fabsf((float)wheel->real_current /
                                     (float)M3508_TOR_TO_CUR);
    const float wheel_speed = fabsf(wheel->speed_aps /
        (side == LEFT ? M3508_READUCTION_RATIO_L : M3508_READUCTION_RATIO_R) *
        DEGREE_2_RAD);
    /* Real forward speed evidence is latched before impact; the remaining
     * gates reject static/rollback current and non-contact leg geometry. */
    return bump_contact_gate(
        bump_approach_speed_seen, wheel_speed, wheel_torque,
        leg[side]->theta, torque_threshold,
        BUMP_CONTACT_THETA_MIN_RAD, BUMP_CONTACT_THETA_MAX_RAD,
        BUMP_CONTACT_WHEEL_SPEED_MAX_RADPS);
}

static uint8_t bump_contacts_update(void)
{
    const float dt = (isfinite(jump_cycle_dt_s) && jump_cycle_dt_s > 0.0002f &&
                      jump_cycle_dt_s < 0.020f) ? jump_cycle_dt_s : 0.001f;
    uint8_t all_confirmed = 1U;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (bump_contact_latched[side]) {
            continue;
        }
        const dji_motor_measure_t *wheel = &control_context->wheel_feedback[side];
        const uint8_t new_sample = wheel->rx_count != bump_contact_last_rx_count[side];
        bump_contact_last_rx_count[side] = wheel->rx_count;
        if (new_sample && bump_contact_condition(side, BUMP_CONTACT_TORQUE_ON)) {
            bump_contact_timer[side] += dt;
            if (bump_contact_timer[side] * 1000.0f >=
                (float)BUMP_CONTACT_CONFIRM_TIME_MS) {
                bump_contact_latched[side] = 1U;
            }
        } else if (new_sample && !bump_contact_condition(side, BUMP_CONTACT_TORQUE_OFF)) {
            bump_contact_timer[side] = 0.0f;
        }
        if (!bump_contact_latched[side]) {
            all_confirmed = 0U;
        }
    }
    if ((bump_contact_latched[LEFT] || bump_contact_latched[RIGHT]) &&
        !all_confirmed) {
        /* Give the other side the existing bounded pairing window.  If it does
         * not arrive, bump_contact_pair_confirmed() accepts the already
         * debounced side instead of waiting forever in APPROACH. */
        bump_contact_pair_window_ms += dt * 1000.0f;
    }
    return bump_contact_pair_confirmed(
        bump_contact_latched[LEFT], bump_contact_latched[RIGHT],
        bump_contact_pair_window_ms, (float)BUMP_CONTACT_PAIR_WINDOW_MS);
}

static void bump_phi0_rate_update(float now_ms, uint32_t now_tick,
                                  const jump_hip_feedback_snapshot_t *snapshot,
                                  uint8_t feedback_fresh)
{
    if (snapshot == NULL) return;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        bump_phi0_rate_state_t *state = &bump_phi0_rate_state[side];
        const uint8_t base = (uint8_t)(side * 2U);
        const uint32_t age0 = (uint32_t)(now_tick - snapshot->last_rx_ms[base]);
        const uint32_t age1 = (uint32_t)(now_tick - snapshot->last_rx_ms[base + 1U]);
        const uint32_t age = (age0 > age1) ? age0 : age1;
        state->sample_age_ms = (float)age;
        const uint8_t sample_changed =
            snapshot->rx_count[base] != state->rx_count[0] &&
            snapshot->rx_count[base + 1U] != state->rx_count[1];
        if (!feedback_fresh || age > JUMP_LANDING_RATE_FEEDBACK_MAX_AGE_MS ||
            !isfinite(leg[side]->phi0)) {
            state->valid = 0U;
            state->rate = 0.0f;
            continue;
        }
        if (!sample_changed) {
            state->valid = (uint8_t)(state->primed &&
                state->sample_age_ms <= JUMP_LANDING_RATE_FEEDBACK_MAX_AGE_MS);
            if (!state->valid) state->rate = 0.0f;
            continue;
        }

        if (state->primed) {
            const float dt = (now_ms - state->last_time_ms) * 0.001f;
            if (dt >= 0.0002f && dt <= 0.020f) {
                const float raw_rate = bump_wrap_pi(
                    leg[side]->phi0 - state->last_phi0) / dt;
                if (isfinite(raw_rate) && fabsf(raw_rate) <= BUMP_PHI0_RATE_MAX) {
                    const uint8_t swinging =
                        bump_state == BUMP_STATE_SWING_OVER_BODY ||
                        bump_state == BUMP_STATE_SWING_SUPPORT;
                    const float tau = swinging ?
                        BUMP_SWING_RATE_FILTER_TAU : BUMP_PHI0_RATE_FILTER_TAU;
                    const float alpha = dt / (tau + dt);
                    state->rate += alpha * (raw_rate - state->rate);
                    state->valid = 1U;
                } else {
                    state->rate = 0.0f;
                    state->valid = 0U;
                }
            } else {
                state->rate = 0.0f;
                state->valid = 0U;
            }
        } else {
            state->rate = 0.0f;
            state->valid = 0U;
            state->primed = 1U;
        }
        state->last_phi0 = leg[side]->phi0;
        state->last_time_ms = now_ms;
        state->rx_count[0] = snapshot->rx_count[base];
        state->rx_count[1] = snapshot->rx_count[base + 1U];
    }
}

static float bump_angle_track_step(uint8_t side, float target, float dt,
                                   uint8_t move_reference)
{
    const uint8_t retract_reacquire =
        bump_state == BUMP_STATE_RETRACT && bump_retract_angle_reacquire;
    const uint8_t swinging =
        bump_state == BUMP_STATE_SWING_OVER_BODY ||
        bump_state == BUMP_STATE_SWING_SUPPORT;
    float tp_kp = retract_reacquire ?
        BUMP_RETRACT_REACQUIRE_TP_KP :
        (swinging ? BUMP_SWING_TRACK_TP_KP : BUMP_SWING_TP_KP);
    const float tp_kd = retract_reacquire ?
        BUMP_RETRACT_REACQUIRE_TP_KD :
        (swinging ? BUMP_SWING_TRACK_TP_KD : BUMP_SWING_TP_KD);
    const float tp_ff = retract_reacquire ? 0.0f : BUMP_SWING_TP_FF;
    const float tp_limit = retract_reacquire ?
        BUMP_RETRACT_REACQUIRE_TP_LIMIT : BUMP_SWING_TP_LIMIT;
    const float phi0_rate = retract_reacquire ?
        BUMP_RETRACT_REACQUIRE_PHI0_RATE : BUMP_SWING_PHI0_RATE;
    float reference_rate = 0.0f;
    if (move_reference) {
        const float error = bump_angle_error(target, bump_phi0_ref[side]);
        /* Include the final fractional step in the velocity command;
         * dropping it to zero early produces an unnecessary braking kick. */
        reference_rate = copysignf(fminf(phi0_rate, fabsf(error) / dt), error);
        bump_phi0_ref[side] = bump_move_reference(
            bump_phi0_ref[side], target, phi0_rate, dt);
    }
    /* Keep reference velocity unfiltered.  Fast measured-rate damping in
     * SWING prevents the accelerate/brake/catch-up cycle seen in bump12/14;
     * RETRACT retains its existing rate filter and angle-hold controller. */
    bump_phi0_ref_rate[side] = reference_rate;
    if (swinging && reference_rate == 0.0f) {
        tp_kp = BUMP_SWING_HOLD_TP_KP;
    }
    const float measured_rate = bump_phi0_rate_state[side].valid ?
        bump_phi0_rate_state[side].rate : 0.0f;
    float tp = tp_kp * bump_angle_error(
                   bump_phi0_ref[side], leg[side]->phi0) +
               tp_kd *
                   (bump_phi0_ref_rate[side] - measured_rate) +
               tp_ff *
                   (bump_phi0_ref_rate[side] / phi0_rate);
    LIMIT_MIN_MAX(tp, -tp_limit, tp_limit);
    return tp;
}

static uint8_t bump_pair_length_settled(const float target[CHASSIS_SIDE_COUNT],
                                        float tolerance,
                                        float dt)
{
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (isfinite(leg[side]->l0) && isfinite(leg[side]->d_l0_lpf) &&
            isfinite(target[side]) &&
            fabsf(leg[side]->l0 - target[side]) <= tolerance &&
            fabsf(leg[side]->d_l0_lpf) <= BUMP_STAND_D_L0_OK) {
            bump_length_stable_timer[side] += dt * 1000.0f;
        } else {
            bump_length_stable_timer[side] = 0.0f;
        }
    }
    return (uint8_t)(bump_length_stable_timer[LEFT] >=
                         BUMP_STAND_STABLE_TIME_MS &&
                     bump_length_stable_timer[RIGHT] >=
                         BUMP_STAND_STABLE_TIME_MS);
}

static uint8_t bump_retract_length_reached(void)
{
    /* Enter the support swing on the first in-band sample. Neither the
     * length derivative nor a 150 ms dwell should delay this handoff.
     * Sticky lower-stop latches cannot substitute for current position. */
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (!isfinite(leg[side]->l0) ||
            leg[side]->l0 < BUMP_RETRACT_MIN_STABLE_LENGTH ||
            leg[side]->l0 >
                BUMP_RETRACT_LENGTH + BUMP_RETRACT_TOLERANCE) {
            return 0U;
        }
    }
    return 1U;
}

static uint8_t bump_retract_reacquire_required(
    const float hold[CHASSIS_SIDE_COUNT],
    const float actual[CHASSIS_SIDE_COUNT])
{
    return (uint8_t)!bump_pair_angle_gate(
        hold, actual, BUMP_RETRACT_REACQUIRE_ANGLE_RAD);
}

static uint8_t bump_retract_finish_allowed(uint8_t reacquiring,
                                          uint8_t angle_settled,
                                          uint8_t length_reached)
{
    /* RETRACT only establishes the short-leg geometry.  Requiring the old
     * SWING_OVER hold angle here can consume the whole 1.4 s timeout when a
     * leg is loaded by the step.  SWING_SUPPORT re-seeds its reference from
     * the measured angle and can start the support swing immediately after
     * the legs reach the short target band. Keep the arguments for source/test ABI
     * compatibility; angle status is intentionally ignored at this handoff. */
    (void)reacquiring;
    (void)angle_settled;
    return length_reached;
}

static uint8_t bump_stand_handoff_step(
    bump_stand_handoff_state_t *state,
    const bump_stand_feedback_t *feedback)
{
    if (state == NULL || feedback == NULL) {
        return 0U;
    }
    float dt = feedback->dt_s;
    if (!isfinite(dt) || dt < 0.0002f || dt > 0.020f) {
        dt = 0.001f;
    }

    const uint8_t hard_attitude_valid =
        isfinite(feedback->pitch_deg) && isfinite(feedback->roll_deg) &&
        fabsf(feedback->pitch_deg) <= BUMP_STAND_PITCH_HARD_DEG &&
        fabsf(feedback->roll_deg) <= BUMP_STAND_ROLL_HARD_DEG;
    const uint8_t attitude_stable = hard_attitude_valid &&
        fabsf(feedback->pitch_deg) <= BUMP_STAND_PITCH_OK_DEG &&
        fabsf(feedback->roll_deg) <= BUMP_STAND_ROLL_OK_DEG;
    uint8_t kinematics_valid = feedback->feedback_valid;
    uint8_t support_window_valid = feedback->feedback_valid;
    uint8_t support_rates_stable = feedback->feedback_valid;
    uint8_t support_phi0_settled = feedback->feedback_valid;
    uint8_t length_rates_stable = feedback->feedback_valid;
    uint8_t final_theta_settled = feedback->feedback_valid;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (!isfinite(feedback->phi0[side]) ||
            !isfinite(feedback->phi0_rate[side]) ||
            !isfinite(feedback->length_rate[side])) {
            kinematics_valid = 0U;
            support_window_valid = 0U;
            support_rates_stable = 0U;
            length_rates_stable = 0U;
        } else {
            if (feedback->phi0[side] <
                    BUMP_STAND_PHI0_SUPPORT_MIN_RAD ||
                feedback->phi0[side] >
                    BUMP_STAND_PHI0_SUPPORT_MAX_RAD) {
                support_window_valid = 0U;
            }
            if (!bump_angle_gate(BUMP_SWING_SUPPORT_PHI0_RAD,
                                 feedback->phi0[side],
                                 BUMP_SWING_ANGLE_OK)) {
                support_phi0_settled = 0U;
            }
            if (fabsf(feedback->phi0_rate[side]) >
                BUMP_SWING_ANGLE_RATE_MAX) {
                support_rates_stable = 0U;
            }
            if (fabsf(feedback->length_rate[side]) >
                BUMP_STAND_D_L0_OK) {
                length_rates_stable = 0U;
            }
        }
        if (!isfinite(feedback->theta[side]) ||
            !isfinite(feedback->theta_rate[side]) ||
            fabsf(feedback->theta[side]) >
                BUMP_STAND_FINAL_THETA_OK_RAD ||
            fabsf(feedback->theta_rate[side]) >
                BUMP_STAND_FINAL_THETA_RATE_MAX) {
            final_theta_settled = 0U;
        }
    }
    if (!isfinite(feedback->theta[LEFT]) ||
        !isfinite(feedback->theta[RIGHT]) ||
        fabsf(feedback->theta[LEFT] - feedback->theta[RIGHT]) >
            BUMP_STAND_FINAL_THETA_SYNC_RAD) {
        final_theta_settled = 0U;
    }

    const uint8_t extension_ready = feedback->feedback_valid &&
        kinematics_valid && hard_attitude_valid && support_window_valid;
    if (!state->extension_started && extension_ready) {
        state->extension_started = 1U;
    }
    if (state->extension_started) {
        state->extend_gain = move_towards_f32(
            state->extend_gain, 1.0f, BUMP_STAND_EXTEND_RATE * dt);
    }

    /* Restore ordinary gravity support only after both feet have contact. */
    const uint8_t contact_support_ready = feedback->feedback_valid &&
        feedback->ground_contact && hard_attitude_valid &&
        support_window_valid && state->extension_started;
    if (!state->support_started) {
        if (contact_support_ready) {
            state->support_ready_ms += dt * 1000.0f;
            if (state->support_ready_ms >= BUMP_STAND_STABLE_TIME_MS) {
                state->support_started = 1U;
            }
        } else {
            state->support_ready_ms = 0.0f;
        }
    }
    state->support_gain = state->support_started ? move_towards_f32(
        state->support_gain, contact_support_ready ? 1.0f : 0.0f,
        BUMP_STAND_SUPPORT_RATE * dt) : 0.0f;

    const uint8_t normal_length_settled =
        bump_pair_length_gate(feedback->length,
                              feedback->normal_length_target,
                              BUMP_STAND_LENGTH_TOLERANCE) &&
        length_rates_stable;
    const uint8_t handoff_safety_valid = feedback->feedback_valid &&
        feedback->ground_contact && hard_attitude_valid &&
        support_window_valid;
    const uint8_t handoff_ready = handoff_safety_valid &&
        support_phi0_settled && support_rates_stable && attitude_stable &&
        normal_length_settled &&
        state->extend_gain >= 0.99f && state->support_gain >= 0.99f;
    if (!state->handoff_started) {
        if (handoff_ready) {
            state->handoff_ready_ms += dt * 1000.0f;
            if (state->handoff_ready_ms >= BUMP_STAND_STABLE_TIME_MS) {
                state->handoff_started = 1U;
            }
        } else {
            state->handoff_ready_ms = 0.0f;
        }
    }

    if (state->handoff_started) {
        state->handoff_gain = move_towards_f32(
            state->handoff_gain, handoff_safety_valid ? 1.0f : 0.0f,
            BUMP_STAND_RATE * dt);
    } else {
        state->handoff_gain = 0.0f;
    }

    /* Strict theta and synchronization checks apply after ownership blends. */
    const uint8_t finish_eligible = handoff_safety_valid &&
        attitude_stable && normal_length_settled && final_theta_settled &&
        state->extend_gain >= 0.99f && state->support_gain >= 0.99f &&
        state->handoff_gain >= 0.99f;
    state->finish_ms = bump_finish_dwell_update(
        state->finish_ms, finish_eligible, dt,
        BUMP_STAND_FINISH_TIME_MS);
    return (uint8_t)(state->finish_ms >= BUMP_STAND_FINISH_TIME_MS);
}

static void bump_control_prepare_cycle(void)
{
    const float dt = (isfinite(jump_cycle_dt_s) && jump_cycle_dt_s > 0.0002f &&
                      jump_cycle_dt_s < 0.020f) ? jump_cycle_dt_s : 0.001f;
    const uint8_t request = chassis_cmd.bump_mode != 0U;
    bump_lockout_resume_capture();
    const uint8_t bump_air_lockout =
        chassis_safety_module_bump_airborne_lockout();
    const uint8_t landing_buffer_active =
        chassis_safety_module_landing_buffer_active();
    if (bump_state != BUMP_STATE_IDLE && landing_buffer_active &&
        !bump_action_active()) {
        bump_buffer_seen = 1U;
    } else if (bump_buffer_seen && !landing_buffer_active && request &&
               bump_state != BUMP_STATE_IDLE &&
               bump_state != BUMP_STATE_DONE && !bump_action_active()) {
        /* 空中/落地缓冲结束后，BUMP 继续从已落地的 0.30 m 名义腿长
         * 交接，避免把缓冲期间的 .21/.13 参考重新抢回。
         *
         * 2026-10-07 (bump2.csv): 增加 `request` 条件。原来这里不带 request，
         * 于是它与下面「!request -> ABORT」的分支每拍互相打架：本分支先把
         * bump_length_ref 直接写成 0.30 并跳 DONE，紧接着请求分支又跳 ABORT
         * (目标 0.13)，bump_state_enter() 一拍被调两次 —— 实测 I22(状态计时)
         * 在 4537 拍里有 3801 拍为 0，腿长目标在 0.30/0.13 之间反复，就是操作手
         * 看到的「收腿->松开->循环」(l0 穿越 0.20 m 共 10 次)。
         * 交接只在拨杆仍按住时才有意义；松开就该走下面的平滑退出。 */
        bump_target_length = BUMP_APPROACH_LENGTH;
        bump_length_ref[LEFT] = BUMP_APPROACH_LENGTH;
        bump_length_ref[RIGHT] = BUMP_APPROACH_LENGTH;
        bump_state_enter(BUMP_STATE_DONE);
    }
    const uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN;

    const uint8_t request_release_can_cancel =
        bump_request_release_can_cancel(bump_state);
    if (!request && !bump_air_lockout && request_release_can_cancel) {
        /* 2026-10-07 (bump2.csv): 拨杆松开 = 操作手不想再爬了，应该**平滑回普通
         * 控制**，而不是走 ABORT。原实现把松开也送进 ABORT，而 ABORT 会把腿收到
         * 0.13(LEN_LEN_LOW) 并切断轮端 —— 站立时这就是一次受控摔倒，也是实测
         * 「松开后一直收腿/循环」的源头。
         *
         * 接触后的阶段不在这里取消：必须完成到支撑姿态并有序交还控制权，
         * 避免在负 phi0 或短腿姿态直接切回普通 LQR。 */
        if (bump_state != BUMP_STATE_IDLE) {
            bump_state_enter(BUMP_STATE_IDLE);
        }
        bump_request_consumed = 0U;
    } else if (bump_state == BUMP_STATE_IDLE && request &&
               !bump_request_consumed) {
        bump_request_consumed = 1U;
        const uint8_t low_or_mid_leg = chassis_cmd.leg_level == LEG_LOW ||
            chassis_cmd.leg_level == LEG_MID;
        if (low_or_mid_leg && normal_mode && control_feedback_valid &&
            bump_ins_feedback_fresh &&
            !chassis_safety_module_off_ground() &&
            chassis_fdb_data.stand_state != CHASSIS_IS_DANGER &&
            isfinite(leg[LEFT]->l0) && isfinite(leg[RIGHT]->l0)) {
            bump_state_enter(BUMP_STATE_APPROACH);
        }
        /* 2026-10-07: 入口条件不满足时**什么都不做**（保持 IDLE），不再进 ABORT。
         * 原实现走 ABORT，而 ABORT 会切轮端 + 把腿收到 0.13 —— 也就是说
         * 「离地时误碰拨杆」或「反馈短暂无效」这类完全不该有动作的情况，
         * 反而会触发一次受控摔倒。请求已在上面标记为已消费，需松开拨杆后
         * 才能再次触发。 */
    }

    /* Keep the terminal BUMP state in the ordinary LOW geometry while the
     * upper switch remains held.  Releasing the switch above has already
     * returned STAND to IDLE, so this cannot suppress a new rising edge. */
    if (bump_state == BUMP_STATE_STAND && request) {
        chassis_cmd.leg_level = LEG_LOW;
    }

    chassis_safety_module_set_bump_active(bump_control_active());
    chassis_safety_module_set_bump_action_active(bump_action_active());
    if (!bump_control_active()) {
        chassis_safety_module_set_wheel_shut(0U);
    }
    if (!bump_control_active()) {
        return;
    }

    if (!normal_mode) {
        bump_abort(BUMP_ABORT_MODE);
    } else if (!control_feedback_valid || !bump_ins_feedback_fresh) {
        bump_abort(BUMP_ABORT_FEEDBACK);
    } else if (chassis_fdb_data.stand_state == CHASSIS_IS_DANGER) {
        bump_abort(BUMP_ABORT_DANGER);
    } else {
        /* A step can leave the chassis on a stable slope while STAND is
         * taking over.  Keep the 24 deg action limit for approach, contact,
         * and swing states; only the support handoff uses the wider slope
         * ceiling.  Danger and off-ground checks remain independent. */
        const float bump_pitch_hard_limit = 24.0f;
        if (!isfinite(ins.pitch) || !isfinite(ins.roll) ||
            fabsf(ins.pitch) > bump_pitch_hard_limit ||
            fabsf(ins.roll) > 16.0f) {
            bump_abort(BUMP_ABORT_ATTITUDE);
        }
    }
    if (bump_control_active() && bump_action_active() &&
               (leg[LEFT]->phi0 < BUMP_PHI0_MIN_RAD ||
                leg[RIGHT]->phi0 < BUMP_PHI0_MIN_RAD ||
                leg[LEFT]->phi0 > BUMP_PHI0_MAX_RAD ||
                leg[RIGHT]->phi0 > BUMP_PHI0_MAX_RAD)) {
        bump_abort(BUMP_ABORT_PHI0_RANGE);
    }

    if (bump_action_active()) {
        chassis_cmd.vx = 0.0f;
        chassis_cmd.vx_set = 0.0f;
        chassis_cmd.vy = 0.0f;
        chassis_cmd.vy_set = 0.0f;
        chassis_cmd.vw_relative = 0.0f;
        chassis_cmd.vw_relative_set = 0.0f;
        bump_motion_reference_zero();
    }

    if (bump_action_active() && bump_air_lockout) {
        bump_airborne_lockout_seen = 1U;
        chassis_safety_module_set_wheel_shut(1U);
        chassis_safety_module_set_bump_active(1U);
        chassis_safety_module_set_bump_action_active(1U);
        return;
    }
    if (bump_control_active()) bump_state_elapsed_s_accum += dt;
    bump_retract_breakaway_update(dt);
    const float elapsed_ms = bump_state_elapsed_s_accum * 1000.0f;
    switch (bump_state) {
    case BUMP_STATE_APPROACH: {
        bump_target_length = BUMP_APPROACH_LENGTH;
        /* chassis_kf_update() runs before leg_calc(); require fresh hip/INS
         * and both wheel streams before latching real forward motion. */
        const uint8_t approach_speed_feedback_fresh =
            control_feedback_valid && bump_ins_feedback_fresh &&
            bump_wheel_feedback_valid(LEFT) &&
            bump_wheel_feedback_valid(RIGHT);
        bump_approach_speed_seen = bump_approach_speed_seen_step(
            bump_approach_speed_seen,
            approach_speed_feedback_fresh,
            chassis_vx_filter, BUMP_APPROACH_VX_MIN_MPS);
        if (fabsf(chassis_cmd.vx_set) > BUMP_APPROACH_VX_RC_LIMIT) {
            chassis_cmd.vx_set = copysignf(BUMP_APPROACH_VX_RC_LIMIT,
                                           chassis_cmd.vx_set);
        }
        /* 2026-10-07 (bump2.csv): 这里原本有一个 1800 ms 超时，超时即进 ABORT。
         * 那是错的：BUMP 的实际用法是「边正常行驶边找台阶」，1.8 s 根本开不到
         * 台阶前。实测 t=37.127 进入 APPROACH、t=38.927 就因超时进 ABORT，
         * 全程没碰到任何东西；而 ABORT 会把腿收到 0.13(LEN_LEN_LOW)，站立时
         * 直接把机器人放倒（pitch 1.48° -> 44° -> -116°，翻车）。
         * 又因为接触判据里的 |theta| >= 0.25 在平地上达不到（实测只到 0.177），
         * 这个超时是 100% 必然触发的 —— 也就是说只要按拨杆并前进，1.8 s 后
         * 必定摔倒，整个荡腿序列从未被执行过。
         *
         * 现在改为：接触前无限等待，就是正常行驶。退出只由两种情况触发 ——
         * 拨杆松开（回 IDLE 平滑交接）或全局故障判据（真故障才 ABORT）。 */
        if (bump_contacts_update()) {
            bump_state_enter(bump_sequence_transition(
                bump_state, 1U, 0U));
        }
        break;
    }
    case BUMP_STATE_CONTACT:
        bump_target_length = BUMP_APPROACH_LENGTH;
        if (bump_sequence_transition(
                bump_state, elapsed_ms >= BUMP_CONTACT_HOLD_TIME_MS,
                0U) != bump_state) {
            bump_state_enter(bump_sequence_transition(
                bump_state, 1U, 0U));
        }
        break;
    case BUMP_STATE_SWING_OVER_BODY: {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_hip_tp[side] = bump_angle_track_step(
                side, BUMP_SWING_OVER_PHI0_RAD, dt, 1U);
        }
        {
            const float over_body_target[CHASSIS_SIDE_COUNT] = {
                BUMP_SWING_OVER_PHI0_RAD, BUMP_SWING_OVER_PHI0_RAD
            };
            const uint8_t reached = bump_pair_angle_targets_reached(
                over_body_target, BUMP_SWING_OVER_ANGLE_OK);
            const bump_state_e next = bump_sequence_transition(
                bump_state, reached,
                elapsed_ms >= BUMP_SWING_OVER_TIMEOUT_MS);
            if (next == BUMP_STATE_ABORT) {
                bump_abort(BUMP_ABORT_SWING_OVER_TIMEOUT);
            } else if (next != bump_state) {
                bump_state_enter(next);
            }
        }
        break;
    }
    case BUMP_STATE_RETRACT:
        bump_target_length = BUMP_RETRACT_LENGTH;
        {
        const float actual_phi0[CHASSIS_SIDE_COUNT] = {
            leg[LEFT]->phi0, leg[RIGHT]->phi0
        };
        if (!bump_retract_angle_reacquire &&
            bump_retract_reacquire_required(bump_phi0_hold,
                                             actual_phi0)) {
            bump_retract_angle_reacquire = 1U;
            for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
                if (isfinite(leg[side]->l0)) {
                    bump_length_ref[side] = leg[side]->l0;
                }
                bump_angle_stable_timer[side] = 0.0f;
                bump_length_stable_timer[side] = 0.0f;
            }
        }
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_hip_tp[side] = bump_angle_track_step(
                side, bump_phi0_hold[side], dt,
                bump_retract_angle_reacquire);
        }
        const uint8_t angle_reached = bump_pair_angle_targets_reached(
            bump_phi0_hold, BUMP_SWING_ANGLE_OK);
        if (bump_retract_angle_reacquire && angle_reached) {
            bump_retract_angle_reacquire = 0U;
            bump_length_stable_timer[LEFT] = 0.0f;
            bump_length_stable_timer[RIGHT] = 0.0f;
        }
        {
            const uint8_t reached = bump_retract_finish_allowed(
                bump_retract_angle_reacquire, angle_reached,
                bump_retract_length_reached());
            const bump_state_e next = bump_sequence_transition(
                bump_state, reached,
                elapsed_ms >= BUMP_RETRACT_TIMEOUT_MS);
            if (next == BUMP_STATE_ABORT) {
                bump_abort(BUMP_ABORT_RETRACT_TIMEOUT);
            } else if (next != bump_state) {
                bump_state_enter(next);
            }
        }
        }
        break;
    case BUMP_STATE_SWING_SUPPORT: {
        for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
            bump_hip_tp[side] = bump_angle_track_step(
                side, BUMP_SWING_SUPPORT_PHI0_RAD, dt, 1U);
        }
        {
            const float support_target[CHASSIS_SIDE_COUNT] = {
                BUMP_SWING_SUPPORT_PHI0_RAD, BUMP_SWING_SUPPORT_PHI0_RAD
            };
            const uint8_t reached = bump_pair_angle_targets_reached(
                support_target, BUMP_SWING_SUPPORT_ANGLE_OK);
            const uint8_t timed_out =
                elapsed_ms >= BUMP_SWING_SUPPORT_TIMEOUT_MS;
            /* Restore normal LOW support on the first in-band sample;
             * no angular-speed gate or settling dwell before STAND.
             * Timeout remains a fallback for a slow loaded swing. */
            const bump_state_e next = bump_sequence_transition(
                bump_state, reached, timed_out);
            if (next != bump_state) {
                bump_state_enter(next);
            }
        }
        break;
    }
    case BUMP_STATE_STAND:
        /* RETRACT has completed the obstacle-specific work.  STAND is now a
         * persistent LOW-mode latch: the normal chassis LQR owns the legs and
         * wheels while the BUMP switch remains held. */
        chassis_cmd.leg_level = LEG_LOW;
        bump_target_length = LEN_LEN_LOW;
        bump_length_ref[LEFT] = LEN_LEN_LOW;
        bump_length_ref[RIGHT] = LEN_LEN_LOW;
        break;
    case BUMP_STATE_DONE:
        bump_target_length = BUMP_APPROACH_LENGTH;
        if (!chassis_safety_module_off_ground() &&
            !chassis_safety_module_landing_buffer_active() &&
            fabsf(leg[LEFT]->d_l0_lpf) <= BUMP_DONE_SPEED_MAX &&
            fabsf(leg[RIGHT]->d_l0_lpf) <= BUMP_DONE_SPEED_MAX &&
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0) >=
                BUMP_APPROACH_LENGTH - BUMP_EXTEND_TOLERANCE) {
            bump_approach_stable_ms += dt * 1000.0f;
            if (bump_approach_stable_ms >= BUMP_DONE_STABLE_TIME_MS &&
                !bump_wheel_handoff_active) {
                bump_state_enter(BUMP_STATE_IDLE);
            }
        } else {
            bump_approach_stable_ms = 0.0f;
        }
        break;
    case BUMP_STATE_ABORT:
        bump_target_length = BUMP_ABORT_LENGTH;
        /* 2026-10-07 (bump11.csv)：**ABORT 必须无条件超时退出，不能等腿长到位。**
         *
         * 原条件要求 avg_l0 <= 0.145 才回 IDLE。但机器人爬上台阶后车身被台阶
         * 撑住，腿**物理上收不到那个长度** —— bump11 实测卡在 0.264，于是 ABORT
         * 永远退不出去：
         *   · 状态停在 ABORT(=8)，bump_control_active() 恒为 1，普通控制被压住；
         *   · wheel_shut = 1（ABORT 关轮子），轮子不发力；
         *   · bump_target_length 恒为 0.13，腿一直被压着收。
         * 现象就是操作手描述的「归零之后就卡住了，啥反应都没，车身已经在台阶上」。
         * bump11 里 t=79.131 进 ABORT，到 t=83.466 手动失能才脱离（共 4.3 秒）。
         *
         * 现在改成：超时即回 IDLE，无条件把控制权交还普通模式。腿长是否到位
         * 不再作为退出门槛 —— 到位了更好，没到位也必须放行。 */
        if (elapsed_ms >= BUMP_ABORT_TIMEOUT_MS) {
            bump_state_enter(BUMP_STATE_IDLE);
        }
        break;
    case BUMP_STATE_IDLE:
    default:
        break;
    }
    /* 2026-10-07 (操作手要求)：**检测到碰撞后必须及时关掉轮子**。
     *
     * 原判据只在 ABORT 时关轮子，整个爬台阶序列（CONTACT/SWING_ZERO/RETRACT/
     * SWING_POSE/STAND）轮子都是通的 —— bump11 实测 I5(wheel_shut) 仅在 ABORT
     * 期间短暂为 1。操作手反馈：接触后轮子还在转，会把车身顶偏/顶下来。
     *
     * 现在：APPROACH 保持轮子驱动（要开过去撞台阶）；一旦进入 CONTACT 及其后
     * 的整个爬台阶序列就关轮子；DONE 交回普通控制时恢复。ABORT 仍然关断。 */
    chassis_safety_module_set_wheel_shut(bump_wheel_shut_requested());
    chassis_safety_module_set_bump_active(bump_control_active());
    chassis_safety_module_set_bump_action_active(bump_action_active());
}

static void Leg_FN_Calculation(float ROLL_TARGET, float L_TARGET_L, float L_TARGET_R);
static float jump_thrust_force_scale(float average_length);

/* --------------------------------- LQR控制相关 -------------------------------- */
static float a11[4];
static float a12[4];
static float a13[4];
static float a14[4];
static float a15[4];
static float a16[4];
static float a21[4];
static float a22[4];
static float a23[4];
static float a24[4];
static float a25[4];
static float a26[4];


static float MatLQR_K[2][6] = {0};//LQR运算中的反馈系数,在lqr_update参数更新函数中更新

/* 正常模式切换腿长档位时，对整张 K 矩阵做无扰交叉过渡。 */
#define LQR_K_TRANSITION_TIME 0.80f
#define LQR_K_TRANSITION_TAIL_START       0.80f
#define LQR_K_TRANSITION_L0_ERROR_MAX     0.012f
#define LQR_K_TRANSITION_L0_RATE_MAX      0.12f
#define LQR_K_TRANSITION_SETTLE_TIME      0.10f
/* 保持为 1.0，使固件使用 MATLAB 根据 Q/R 求出的完整 LQR K。
 * 若需改变 pitch-rate 权重，应重新调整 Q/R 并求解，而不是部署后缩放 K。 */
#define LQR_NORMAL_DPHI_GAIN 1.0f

/* The rigid-body LQR model does not include the measured 7-13 Hz chassis/leg
 * elastic mode. Attenuate only that band in the normal-mode phi-rate state;
 * raw IMU data remain available to recovery and safety logic. */
#define LQR_DPHI_NOTCH_F0_HZ       9.5f
#define LQR_DPHI_NOTCH_Q           1.5f
#define LQR_DPHI_NOTCH_RAW_BLEND   0.35f
#define LQR_DPHI_NOTCH_PI          3.14159265358979323846f

typedef struct {
    float x1;
    float x2;
    float y1;
    float y2;
    float output;
    uint8_t initialized;
} lqr_dphi_notch_t;

static lqr_dphi_notch_t lqr_dphi_notch = {0};
static float dbg_lqr_dphi_raw = 0.0f;
static float dbg_lqr_dphi_filtered = 0.0f;
static float lqr_k_transition_from[2][6] = {0};
static float lqr_k_transition_current[2][6] = {0};
static float lqr_k_transition_progress = 1.0f;
static leg_level_e lqr_k_transition_level = LEG_LOW;
static uint8_t lqr_k_transition_initialized = 0U;
static float lqr_k_transition_settle_timer = 0.0f;
static uint8_t lqr_k_transition_tail_released = 1U;
static uint8_t dbg_lqr_k_tail_waiting = 0U;


/* [T Tp(髋)] */
static float LQROutBuf[2][2]={0};

/* LQR 位置积分观测器（提升到文件作用域，便于 handoff 瞬间清零） */
typedef enum {
    LQR_POSITION_DRIVE = 0,
    LQR_POSITION_BRAKE,
    LQR_POSITION_HOLD,
} lqr_position_state_e;

static float lqr_virtual_x_l = 0.0f;
static float lqr_virtual_x_r = 0.0f;
static float lqr_position_capture_timer = 0.0f;
static float lqr_position_hold_x = 0.0f;
static float lqr_position_brake_offset = 0.0f;
static lqr_position_state_e lqr_position_state = LQR_POSITION_DRIVE;

static float LQRXerrorBuf[2][6]={0};
static float LQRXObsBuf[2][6]={0};
static float LQRXRefBuf[2][6]={0}; /*LQRXObsBuf[0][2] - LQRXRefBuf[0][2]*/

static void bump_stand_lqr_seed(void)
{
    lqr_position_state = LQR_POSITION_DRIVE;
    lqr_position_capture_timer = 0.0f;
    lqr_position_hold_x = 0.0f;
    lqr_position_brake_offset = 0.0f;
    lqr_virtual_x_l = 0.0f;
    lqr_virtual_x_r = 0.0f;
    yaw_target = ins.yaw_total_angle * DEGREE_2_RAD;
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        LQRXObsBuf[side][2] = 0.0f;
        LQROutBuf[side][0] = 0.0f;
    }
    bump_motion_reference_zero();
}

static void bump_motion_reference_zero(void)
{
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        LQRXRefBuf[side][2] = 0.0f;
        LQRXRefBuf[side][3] = 0.0f;
    }
}

static uint8_t jump_ground_wheel_cache_publish(void)
{
    const float raw_left = LQROutBuf[LEFT][0];
    const float raw_right = LQROutBuf[RIGHT][0];
    if (!isfinite(raw_left) || !isfinite(raw_right)) {
        jump_ground_wheel_common_cache = 0.0f;
        jump_ground_wheel_cache_valid = 0U;
        if (jump_ground_recovery_scope_active) {
            /* The asynchronous wheel callback may run before the next mode
             * transition. Clear only this failed scoped candidate so no NaN
             * survives into its int16 current conversion. */
            LQROutBuf[LEFT][0] = 0.0f;
            LQROutBuf[RIGHT][0] = 0.0f;
            jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
            return 0U;
        }
        return 1U;
    }
    jump_ground_wheel_common_cache = jump_ground_recovery_common(
        raw_left, raw_right, M3508_TOR_MAX,
        jump_ground_recovery_scope_active ? jump_ground_recovery_gain : 0.0f);
    jump_ground_wheel_cache_valid = 1U;
    return 1U;
}

static uint8_t jump_landing_balance_apply_lqr_refs(void)
{
    if (!jump_landing_balance_state.capture_valid ||
        !chassis_safety_module_jump_landing_owner_active() ||
        chassis_safety_module_off_ground() ||
        chassis_safety_module_landing_gain() <= 0.0f) {
        return 1U;
    }
    float beta = jump_landing_ground_balance_gain;
    if (!isfinite(beta)) beta = 0.0f;
    LIMIT_MIN_MAX(beta, 0.0f, 1.0f);
    const float theta_ref_left = jump_landing_balance_theta_ref(
        jump_landing_balance_state.captured_theta,
        dbg_theta_total_comp_L, beta);
    const float theta_ref_right = jump_landing_balance_theta_ref(
        jump_landing_balance_state.captured_theta,
        dbg_theta_total_comp_R, beta);
    if (!isfinite(theta_ref_left) || !isfinite(theta_ref_right)) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        return 0U;
    }
    LQRXRefBuf[LEFT][0] = theta_ref_left;
    LQRXRefBuf[RIGHT][0] = theta_ref_right;
    LQRXRefBuf[LEFT][1] = 0.0f;
    LQRXRefBuf[RIGHT][1] = 0.0f;
    if (!jump_motion_vx_ref_state.active) {
        LQRXRefBuf[LEFT][3] = 0.0f;
        LQRXRefBuf[RIGHT][3] = 0.0f;
    }
    return 1U;
}


/*
 * Matrix_LQRObs
 * [θ
 * θ_dot
 * x
 * x_dot
 * φ
 * φ_dot]
 */
static arm_matrix_instance_f32 MatLQRObs_L  = {6, 1, LQRXObsBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRObs_R  = {6, 1, LQRXObsBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRRef_L  = {6, 1, LQRXRefBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRRef_R  = {6, 1, LQRXRefBuf[RIGHT]};
static arm_matrix_instance_f32 MatLQRNegK   = {2, 6, (float*)MatLQR_K};/*TODO 后续需要测试是否可行*/
static arm_matrix_instance_f32 MatLQRErrX_L = {6, 1, LQRXerrorBuf[LEFT]};
static arm_matrix_instance_f32 MatLQRErrX_R = {6, 1, LQRXerrorBuf[RIGHT]};
/*
 * U
 * [T  驱动轮输出力矩
 * Tp]  髋关节输出力矩
 * */
static arm_matrix_instance_f32 MatLQROutU_L = {2, 1, LQROutBuf[LEFT]};
static arm_matrix_instance_f32 MatLQROutU_R = {2, 1, LQROutBuf[RIGHT]};


/*Calculate X. Output is u (T,Tp)`*/
/*TODO 加入对于theta和phi的死区,否则难以稳定下来*/
static void LQR_Cal(){


    //Calculate error
    /*cmsis dsp中的矩阵减法运算,error = ref(目标值) - obs(观测值) */
    arm_mat_sub_f32(&MatLQRRef_R,&MatLQRObs_R,&MatLQRErrX_R);

    //Calculate output value
    /*矩阵乘法,[2*6]*[6*1] */
    /*TODO 不同腿长下K矩阵会随之改变*/
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_R, &MatLQROutU_R);
    //Calculate error
    /*cmsis dsp中的矩阵减法运算,error = ref(目标值) - obs(观测值) */
    arm_mat_sub_f32(&MatLQRRef_L,&MatLQRObs_L, &MatLQRErrX_L);

    arm_mat_sub_f32(&MatLQRRef_R,&MatLQRObs_R,&MatLQRErrX_R);

    //Calculate output value
    /*矩阵乘法,[2*6]*[6*1] */
    arm_mat_mult_f32(&MatLQRNegK, &MatLQRErrX_L, &MatLQROutU_L);


}

/* ------------------------------- 打滑检测卡尔曼滤波部分 ------------------------------ */
//TODO:后续考虑移入观测线程，建立整车观测器
static KalmanFilter_t chassis_kf_l;
static KalmanFilter_t chassis_kf_r;
static float chassis_vx_l;
static float chassis_vx_r;
static float chassis_kf_dt_s = 0.003f;
static float jump_ground_vx_snapshot;
static uint8_t jump_ground_vx_snapshot_valid;
static uint8_t jump_kf_reacquire_waiting;
static uint8_t jump_kf_reacquire_contact_latched;
static uint8_t jump_kf_reacquire_contact_seen;
static uint8_t jump_kf_reacquire_active;
static float jump_kf_reacquire_elapsed_s;
static float jump_kf_reacquire_seed_vx;
/* Remains set if a safety/mode exit cancels the jump state machine midair. */
static uint8_t jump_kf_airborne_hold_latched;

static void jump_landing_speed_state_reset(void)
{
    jump_ground_vx_snapshot = 0.0f;
    jump_ground_vx_snapshot_valid = 0U;
    jump_kf_reacquire_waiting = 0U;
    jump_kf_reacquire_contact_latched = 0U;
    jump_kf_reacquire_contact_seen = 0U;
    jump_kf_reacquire_active = 0U;
    jump_kf_reacquire_elapsed_s = 0.0f;
    jump_kf_reacquire_seed_vx = 0.0f;
    jump_landing_vx_ref_cancel(&jump_motion_vx_ref_state, 0U);
}

#define KIN_RATE_WINDOW_SIZE 5U
#define KIN_RATE_LPF_TAU_S   0.010f

/* Normal-mode differential leg-angle damping.  The existing theta PID keeps
 * the static synchronizing stiffness; this bounded term only opposes relative
 * leg swing and therefore does not slow the common swing response. */
#define THETA_SYNC_GAIN_L0_START 0.160f
#define THETA_SYNC_GAIN_L0_END   0.240f
#define THETA_SYNC_KP_HIGH_NORMAL 19.0f
#define THETA_SYNC_KP_HIGH_SPIN   26.0f
#define THETA_SYNC_DAMP_KD_LOW             2.00f
#define THETA_SYNC_DAMP_KD_HIGH_NORMAL    2.00f
#define THETA_SYNC_DAMP_KD_HIGH_SPIN   1.30f
#define THETA_SYNC_DAMP_TP_LIMIT_NORMAL 3.00f
#define THETA_SYNC_DAMP_TP_LIMIT_SPIN   4.00f
#define THETA_SYNC_GUARD_START_NORMAL   0.150f
#define THETA_SYNC_GUARD_END_NORMAL     0.300f
#define THETA_SYNC_GUARD_LIMIT_NORMAL   2.50f
#define THETA_SYNC_GUARD_START_SPIN     0.090f
#define THETA_SYNC_GUARD_END_SPIN       0.210f
#define THETA_SYNC_GUARD_LIMIT_SPIN     4.00f

typedef struct {
    float theta_history[KIN_RATE_WINDOW_SIZE];
    float l0_history[KIN_RATE_WINDOW_SIZE];
    float time_ms_history[KIN_RATE_WINDOW_SIZE];
    float theta_rate;
    float l0_rate;
    uint8_t write_index;
    uint8_t sample_count;
} chassis_kin_rate_observer_t;

static chassis_kin_rate_observer_t kin_rate_observer[2];
static float dbg_theta_sync_p = 0.0f;
static float dbg_theta_sync_d = 0.0f;
static float dbg_theta_rate_diff = 0.0f;
static float dbg_theta_sync_kp = theta_Kp;
static float dbg_theta_sync_kd = THETA_SYNC_DAMP_KD_LOW;
static float dbg_theta_sync_guard = 0.0f;

static float chassis_angle_delta(float current, float previous)
{
    float delta = current - previous;
    while (delta > PI) {
        delta -= PI2;
    }
    while (delta < -PI) {
        delta += PI2;
    }
    return delta;
}

static void chassis_kin_rate_store_first(chassis_kin_rate_observer_t *observer,
                                         float theta,
                                         float l0,
                                         float now_ms)
{
    memset(observer, 0, sizeof(*observer));
    observer->theta_history[0] = theta;
    observer->l0_history[0] = l0;
    observer->time_ms_history[0] = now_ms;
    observer->write_index = 1U;
    observer->sample_count = 1U;
}

static void chassis_kin_rate_update(float now_ms)
{
    for (uint8_t side = 0U; side < 2U; side++) {
        chassis_kin_rate_observer_t *observer = &kin_rate_observer[side];
        float theta = leg[side]->theta;
        float l0 = leg[side]->l0;

        if (observer->sample_count < KIN_RATE_WINDOW_SIZE) {
            uint8_t index = observer->write_index;
            observer->theta_history[index] = theta;
            observer->l0_history[index] = l0;
            observer->time_ms_history[index] = now_ms;
            observer->write_index = (uint8_t)((index + 1U) % KIN_RATE_WINDOW_SIZE);
            observer->sample_count++;
            continue;
        }

        uint8_t oldest = observer->write_index;
        float window_dt_s = (now_ms - observer->time_ms_history[oldest]) * 0.001f;
        if (window_dt_s < 0.002f || window_dt_s > 0.050f) {
            chassis_kin_rate_store_first(observer, theta, l0, now_ms);
            continue;
        }

        float theta_rate_raw =
            chassis_angle_delta(theta, observer->theta_history[oldest]) /
            window_dt_s;
        float l0_rate_raw = (l0 - observer->l0_history[oldest]) / window_dt_s;
        float filter_dt_s = chassis_kf_dt_s;
        if (filter_dt_s < 0.0002f || filter_dt_s > 0.010f) {
            filter_dt_s = 0.001f;
        }
        float alpha = KIN_RATE_LPF_TAU_S /
                      (KIN_RATE_LPF_TAU_S + filter_dt_s);

        observer->theta_rate = alpha * observer->theta_rate +
                               (1.0f - alpha) * theta_rate_raw;
        observer->l0_rate = alpha * observer->l0_rate +
                            (1.0f - alpha) * l0_rate_raw;

        observer->theta_history[oldest] = theta;
        observer->l0_history[oldest] = l0;
        observer->time_ms_history[oldest] = now_ms;
        observer->write_index = (uint8_t)((oldest + 1U) % KIN_RATE_WINDOW_SIZE);
    }
}

static void chassis_theta_sync_gain_update(void)
{
    if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY) {
        dbg_theta_sync_kp = theta_Kp;
        dbg_theta_sync_kd = THETA_SYNC_DAMP_KD_LOW;
        theta_pid->Kp = dbg_theta_sync_kp;
        return;
    }

    /* 2026-09-22 alignment: the sim has no jump-specific sync override.
     * Its 3.1 Hz yaw/leg-swing limit-cycle fix raised the SHARED
     * damping to 2.0 Nm s/rad instead of stiffening P during jump;
     * jump now uses the same scheduled Kp (15..19) / flat Kd 2.0. */

    const float l0_average = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    float progress = (l0_average - THETA_SYNC_GAIN_L0_START) /
                     (THETA_SYNC_GAIN_L0_END - THETA_SYNC_GAIN_L0_START);
    LIMIT_MIN_MAX(progress, 0.0f, 1.0f);

    /* The delayed differential-rate observer must not dominate the 8.4 Hz
     * mechanical mode in ordinary driving.  Spin keeps its verified gains. */
    const float gain = progress * progress * (3.0f - 2.0f * progress);
    const uint8_t spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    const float theta_sync_kp_high = spin_mode ?
        THETA_SYNC_KP_HIGH_SPIN : THETA_SYNC_KP_HIGH_NORMAL;
    const float theta_sync_kd_high = spin_mode ?
        THETA_SYNC_DAMP_KD_HIGH_SPIN : THETA_SYNC_DAMP_KD_HIGH_NORMAL;
    dbg_theta_sync_kp = theta_Kp +
                        (theta_sync_kp_high - theta_Kp) * gain;
    dbg_theta_sync_kd = THETA_SYNC_DAMP_KD_LOW +
                        (theta_sync_kd_high - THETA_SYNC_DAMP_KD_LOW) * gain;
    theta_pid->Kp = dbg_theta_sync_kp;
}

static float chassis_theta_sync_damping_calculate(
    uint8_t *observer_rate_invalid)
{
    if (observer_rate_invalid != NULL) {
        *observer_rate_invalid = 0U;
    }
    dbg_theta_sync_d = 0.0f;
    dbg_theta_rate_diff = 0.0f;

    /* Jump motion previously had proportional split correction only.  During
     * the fast thrust/release stroke theta difference first reached 30-40
     * degrees, then crossed zero fast enough to open by the same amount in
     * the opposite direction after takeoff.  Use the local filtered rates;
     * the ordinary delayed observer remains reserved for driving. */
    const uint8_t grounded_jump =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        jump_state != JUMP_STATE_IDLE &&
        !chassis_safety_module_off_ground() &&
        !chassis_safety_module_jump_landing_owner_active() &&
        !jump_landing_postbuffer_normal_base_active();
    if (grounded_jump) {
        const float legacy_rate_diff =
            leg[LEFT]->d_theta_lpf - leg[RIGHT]->d_theta_lpf;
        float observer_rate_l = 0.0f;
        float observer_rate_r = 0.0f;
        uint8_t observer_usable = 0U;
        const uint8_t local_rate_scope =
            jump_state == JUMP_STATE_THRUST ||
            jump_state == JUMP_STATE_LANDING;
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
        observer_rate_l = jump_kin_theta_rate[LEFT];
        observer_rate_r = jump_kin_theta_rate[RIGHT];
        observer_usable = jump_kin_rates_usable;
#endif
        jump_theta_diff_d_selection_t rate_selection;
        if (JUMP_LOCAL_RATE_FEEDBACK_ENABLE &&
            JUMP_KINEMATIC_RATE_OBSERVER_ENABLE && local_rate_scope) {
            const jump_local_rate_selection_t left =
                jump_theta_rate_select(LEFT, leg[LEFT]->d_theta_lpf, 1U);
            const jump_local_rate_selection_t right =
                jump_theta_rate_select(RIGHT, leg[RIGHT]->d_theta_lpf, 1U);
            rate_selection.rate_diff = 0.0f;
            rate_selection.observer_selected = 1U;
            rate_selection.observer_valid = (uint8_t)(left.valid && right.valid);
            if (rate_selection.observer_valid) {
                rate_selection.rate_diff = left.rate - right.rate;
                if (!isfinite(rate_selection.rate_diff)) {
                    rate_selection.rate_diff = 0.0f;
                    rate_selection.observer_valid = 0U;
                }
            }
        } else {
            rate_selection = jump_theta_diff_d_select_rate_diff(
                (uint8_t)(JUMP_THRUST_THETA_DIFF_D_RATE_OBSERVER_ENABLE &&
                          JUMP_KINEMATIC_RATE_OBSERVER_ENABLE),
                (uint8_t)(chassis_cmd.ctrl_mode == CHASSIS_JUMP),
                (uint8_t)(jump_state == JUMP_STATE_THRUST),
                jump_thrust_active,
                chassis_safety_module_off_ground(),
                observer_usable,
                legacy_rate_diff,
                observer_rate_l,
                observer_rate_r);
        }
        if (rate_selection.observer_selected &&
            !rate_selection.observer_valid) {
            if (observer_rate_invalid != NULL) {
                *observer_rate_invalid = 1U;
            }
            return 0.0f;
        }
        const float rate_diff = rate_selection.rate_diff;
        if (isfinite(rate_diff)) {
            dbg_theta_rate_diff = rate_diff;
            const float sync_kd = jump_state == JUMP_STATE_THRUST ?
                JUMP_THRUST_THETA_DIFF_KD : THETA_SYNC_DAMP_KD_LOW;
            dbg_theta_sync_d = sync_kd * rate_diff;
            LIMIT_MIN_MAX(dbg_theta_sync_d,
                          -JUMP_THETA_SYNC_DAMP_LIMIT,
                           JUMP_THETA_SYNC_DAMP_LIMIT);
        }
        return dbg_theta_sync_d;
    }

#if CHASSIS_KINEMATIC_RATE_OBSERVER_ENABLE
    const uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_AUTO ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN ||
        chassis_safety_module_jump_landing_owner_active() ||
        (chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
         jump_landing_postbuffer_normal_base_active());
    const uint8_t observer_ready =
        kin_rate_observer[LEFT].sample_count >= KIN_RATE_WINDOW_SIZE &&
        kin_rate_observer[RIGHT].sample_count >= KIN_RATE_WINDOW_SIZE;

    if (normal_mode && observer_ready &&
        !chassis_safety_module_off_ground()) {
        const float rate_diff = kin_rate_observer[LEFT].theta_rate -
                                kin_rate_observer[RIGHT].theta_rate;
        if (isfinite(rate_diff)) {
            dbg_theta_rate_diff = rate_diff;
            dbg_theta_sync_d = dbg_theta_sync_kd * rate_diff;
            const float damping_limit =
                chassis_cmd.ctrl_mode == CHASSIS_SPIN ?
                THETA_SYNC_DAMP_TP_LIMIT_SPIN :
                THETA_SYNC_DAMP_TP_LIMIT_NORMAL;
            LIMIT_MIN_MAX(dbg_theta_sync_d,
                          -damping_limit,
                           damping_limit);
        }
    }
#endif

    return dbg_theta_sync_d;
}

static float chassis_theta_sync_guard_calculate(void)
{
    dbg_theta_sync_guard = 0.0f;

    const uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_AUTO ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    if (normal_mode && !chassis_safety_module_off_ground()) {
        const float theta_diff = leg[LEFT]->theta - leg[RIGHT]->theta;
        const float theta_diff_abs = fabsf(theta_diff);
        const uint8_t spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
        const float guard_start = spin_mode ?
            THETA_SYNC_GUARD_START_SPIN : THETA_SYNC_GUARD_START_NORMAL;
        const float guard_end = spin_mode ?
            THETA_SYNC_GUARD_END_SPIN : THETA_SYNC_GUARD_END_NORMAL;
        const float guard_limit = spin_mode ?
            THETA_SYNC_GUARD_LIMIT_SPIN : THETA_SYNC_GUARD_LIMIT_NORMAL;
        float progress = (theta_diff_abs - guard_start) /
                         (guard_end - guard_start);
        LIMIT_MIN_MAX(progress, 0.0f, 1.0f);

        /* This is a continuous extension of the same theta feedback path. */
        const float gain = progress * progress * (3.0f - 2.0f * progress);
        dbg_theta_sync_guard = copysignf(guard_limit * gain,
                                         theta_diff);
    }

    return dbg_theta_sync_guard;
}
/**
*   |    vx     |
*   |    ax     |
*/
static void chassis_kf_init(void)
{
    static float P_Init[4] =
            {
                    54.5802, 0.0054,
                    0.0054, 16.7869,
            };
    static float F_Init[4] =
            {
                    1, 0.003,
                    0, 1,

            };
    static float Q_Init[4] =
            {
                    50, 0,
                    0, 15,
            };

    // 设置最小方差
    static float state_min_variance[2] = {0.005, 0.1};/*TODO 初次修改怎么优化参数*/

    //  电机编码器测得位移，电机编码器测得速度，加速度计解算得出速度
    static uint8_t measurement_reference[2] = {1,2};
    static float measurement_degree[2] = {1,1};
    // 根据measurement_reference与measurement_degree生成H矩阵如下（在当前周期全部测量数据有效情况下）

//    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）
    static float mat_R_diagonal_elements[3] = {5,2}; // 方差即为影响置信度，方差越大置信度越低  /* 打滑检测：800000 1000 */1
    //根据mat_R_diagonal_elements生成R矩阵如下（在当前周期全部测量数据有效情况下）

    // 开启自动调整
    chassis_kf_l.UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf_l, 2, 0, 2);

    // 开启自动调整
    chassis_kf_r.UseAutoAdjustment = 1;
    Kalman_Filter_Init(&chassis_kf_r, 2, 0, 2);

    // 设置矩阵值
    memcpy(chassis_kf_l.P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf_l.F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf_l.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf_l.MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf_l.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf_l.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf_l.StateMinVariance, state_min_variance, sizeof(state_min_variance));

    // 设置矩阵值
    memcpy(chassis_kf_r.P_data, P_Init, sizeof(P_Init));
    memcpy(chassis_kf_r.F_data, F_Init, sizeof(F_Init));
    memcpy(chassis_kf_r.Q_data, Q_Init, sizeof(Q_Init));
    memcpy(chassis_kf_r.MeasurementMap, measurement_reference, sizeof(measurement_reference));
    memcpy(chassis_kf_r.MeasurementDegree, measurement_degree, sizeof(measurement_degree));
    memcpy(chassis_kf_r.MatR_DiagonalElements, mat_R_diagonal_elements, sizeof(mat_R_diagonal_elements));
    memcpy(chassis_kf_r.StateMinVariance, state_min_variance, sizeof(state_min_variance));


}
static float  speed_rads_ground_l;
static float  wheel_to_ground_l,speed_rads_ground_r,wheel_to_ground_r;
static float  wheel_to_ground_legacy_l, wheel_to_ground_legacy_r;
static float  wheel_to_ground_candidate_l, wheel_to_ground_candidate_r;
static void chassis_kf_update_pair(void *context)
{
    (void)context;
    Kalman_Filter_Update(&chassis_kf_l);
    Kalman_Filter_Update(&chassis_kf_r);
}

static void chassis_kf_seed_jump_vx(float vx)
{
    if (!isfinite(vx)) vx = 0.0f;
    chassis_kf_l.xhat_data[0] = vx;
    chassis_kf_r.xhat_data[0] = vx;
    chassis_kf_l.xhat_data[1] = 0.0f;
    chassis_kf_r.xhat_data[1] = 0.0f;
    chassis_kf_l.FilteredValue[0] = vx;
    chassis_kf_r.FilteredValue[0] = vx;
    chassis_kf_l.FilteredValue[1] = 0.0f;
    chassis_kf_r.FilteredValue[1] = 0.0f;
}

static void chassis_kf_update(void)
{
    static float _kf_start;
    float now_ms = dwt_get_time_ms();
    if(_kf_start != 0) // 避免第一次计算错误
        chassis_kf_dt_s = (now_ms - _kf_start) / 1000.0f;
    else
        chassis_kf_dt_s = 0.003f;
    _kf_start = now_ms;

    const uint8_t landing_origin_scope = (uint8_t)(
        chassis_safety_module_landing_buffer_jump_origin() ||
        chassis_safety_module_landing_postbuffer_active() ||
        chassis_safety_module_landing_airborne_guard_active());
    const uint8_t jump_kf_scope = (uint8_t)(
        jump_state != JUMP_STATE_IDLE || landing_origin_scope ||
        jump_kf_airborne_hold_latched || jump_kf_reacquire_active);
    float jump_kf_dt_s = chassis_kf_dt_s;
    if (!isfinite(jump_kf_dt_s) || jump_kf_dt_s < 0.0002f ||
        jump_kf_dt_s > 0.020f) {
        jump_kf_dt_s = 0.001f;
    }
    if (jump_kf_scope) {
        chassis_kf_dt_s = jump_kf_dt_s;
        chassis_kf_l.F_data[1] = jump_kf_dt_s;
        chassis_kf_r.F_data[1] = jump_kf_dt_s;
    } else {
        /* Preserve the tuned standing estimator's historical 3 ms model. */
        chassis_kf_l.F_data[1] = 0.003f;
        chassis_kf_r.F_data[1] = 0.003f;
    }

    chassis_kin_rate_update(now_ms);


    float wheel_rate_l =
        -(m3508_motor[LEFT]->measure.speed_aps /
          M3508_READUCTION_RATIO_L * DEGREE_2_RAD);
    float wheel_rate_r =
        (m3508_motor[RIGHT]->measure.speed_aps /
         M3508_READUCTION_RATIO_R * DEGREE_2_RAD);

    speed_rads_ground_l = wheel_rate_l - ins.gyro[0] - leg[LEFT]->d_phi0;
    speed_rads_ground_r = wheel_rate_r - ins.gyro[0] - leg[RIGHT]->d_phi0;
    wheel_to_ground_legacy_l =
        speed_rads_ground_l * WHEEL_RADIUS +
        leg[LEFT]->d_theta_lpf * leg[LEFT]->l0 *
            arm_cos_f32(leg[LEFT]->theta) +
        leg[LEFT]->d_l0 * arm_sin_f32(leg[LEFT]->theta);
    wheel_to_ground_legacy_r =
        speed_rads_ground_r * WHEEL_RADIUS +
        leg[RIGHT]->d_theta_lpf * leg[RIGHT]->l0 *
            arm_cos_f32(leg[RIGHT]->theta) +
        leg[RIGHT]->d_l0 * arm_sin_f32(leg[RIGHT]->theta);

    wheel_to_ground_candidate_l =
        (wheel_rate_l + kin_rate_observer[LEFT].theta_rate) * WHEEL_RADIUS +
        kin_rate_observer[LEFT].theta_rate * leg[LEFT]->l0 *
            arm_cos_f32(leg[LEFT]->theta) +
        kin_rate_observer[LEFT].l0_rate * arm_sin_f32(leg[LEFT]->theta);
    wheel_to_ground_candidate_r =
        (wheel_rate_r + kin_rate_observer[RIGHT].theta_rate) * WHEEL_RADIUS +
        kin_rate_observer[RIGHT].theta_rate * leg[RIGHT]->l0 *
            arm_cos_f32(leg[RIGHT]->theta) +
        kin_rate_observer[RIGHT].l0_rate * arm_sin_f32(leg[RIGHT]->theta);

#if CHASSIS_KINEMATIC_RATE_OBSERVER_ENABLE
    uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_AUTO ||
        chassis_cmd.ctrl_mode == CHASSIS_JUMP ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    wheel_to_ground_l = normal_mode && isfinite(wheel_to_ground_candidate_l) ?
                        wheel_to_ground_candidate_l : wheel_to_ground_legacy_l;
    wheel_to_ground_r = normal_mode && isfinite(wheel_to_ground_candidate_r) ?
                        wheel_to_ground_candidate_r : wheel_to_ground_legacy_r;
#else
    wheel_to_ground_l = wheel_to_ground_legacy_l;
    wheel_to_ground_r = wheel_to_ground_legacy_r;
#endif

    const uint8_t off_ground = chassis_safety_module_off_ground();
    const uint8_t jump_profile_unloaded = (uint8_t)(
        jump_state != JUMP_STATE_IDLE &&
        (chassis_safety_module_jump_legs_unloaded() ||
         chassis_safety_module_jump_air_profile_active()));
    const uint8_t landing_buffer_contact = (uint8_t)(
        chassis_safety_module_landing_postbuffer_active() ||
        chassis_safety_module_landing_gain() > 0.0f);
    const uint8_t trusted_ground_contact = (uint8_t)(
        chassis_safety_module_ground_contact_confirmed() ||
        recovery_after_handoff());
    const uint8_t landing_contact_confirmed =
        jump_landing_kf_contact_confirmed(
            off_ground, landing_origin_scope, landing_buffer_contact,
            jump_kf_airborne_hold_latched,
            (uint8_t)(jump_state == JUMP_STATE_IDLE),
            trusted_ground_contact,
            (uint8_t)(chassis_fdb_data.stand_state == CHASSIS_IS_DANGER));
    const uint8_t airborne_scope = jump_landing_kf_should_hold(
        (uint8_t)(jump_state == JUMP_STATE_FLIGHT),
        (uint8_t)(jump_state != JUMP_STATE_IDLE), off_ground,
        jump_profile_unloaded, landing_origin_scope,
        jump_kf_airborne_hold_latched,
        (uint8_t)(chassis_fdb_data.stand_state == CHASSIS_IS_DANGER),
        chassis_safety_module_landing_airborne_guard_active(),
        landing_contact_confirmed);
    const uint8_t safe_ground_sample = (uint8_t)(
        jump_state == JUMP_STATE_IDLE &&
        !off_ground && !jump_profile_unloaded &&
        !chassis_safety_module_landing_airborne_guard_active() &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER &&
        isfinite(chassis_vx_filter));
    /* Capture the launch speed before the jump state machine starts.  During
     * PRELOAD/THRUST the wheel/leg motion is intentionally large and its
     * kinematic speed is not chassis translation; updating this snapshot
     * there turns a small jump into a stored +/-1--2 m/s command that the
     * landing LQR then tries to reproduce. */
    const uint8_t jump_speed_snapshot_scope = (uint8_t)(
        jump_state != JUMP_STATE_IDLE || landing_origin_scope ||
        chassis_safety_module_landing_postbuffer_active() ||
        chassis_safety_module_landing_airborne_guard_active());
    if (safe_ground_sample && !jump_speed_snapshot_scope) {
        jump_ground_vx_snapshot = chassis_vx_filter;
        jump_ground_vx_snapshot_valid = 1U;
    }

    if (airborne_scope) {
        if (off_ground || jump_state == JUMP_STATE_FLIGHT) {
            jump_kf_airborne_hold_latched = 1U;
        }
        const uint8_t jump_motion_owned_scope = (uint8_t)(
            jump_state != JUMP_STATE_IDLE || landing_origin_scope ||
            chassis_safety_module_landing_airborne_guard_active());
        const float motion_seed_vx = jump_ground_vx_snapshot_valid ?
            jump_ground_vx_snapshot : chassis_vx_filter;
        (void)jump_landing_vx_ref_start(
            &jump_motion_vx_ref_state, jump_motion_owned_scope,
            motion_seed_vx);
        /* Keep the captured launch velocity reference fixed through flight
         * and every rebound. It may only slew again after confirmed contact. */
        jump_landing_vx_ref_mark_airborne(&jump_motion_vx_ref_state);
        jump_kf_reacquire_waiting = 1U;
        jump_kf_reacquire_contact_latched = 0U;
        jump_kf_reacquire_active = 0U;
        jump_kf_reacquire_elapsed_s = 0.0f;
        chassis_kf_l.MeasuredVector[0] = 0.0f;
        chassis_kf_l.MeasuredVector[1] = 0.0f;
        chassis_kf_r.MeasuredVector[0] = 0.0f;
        chassis_kf_r.MeasuredVector[1] = 0.0f;
        (void)jump_landing_kf_update_if_allowed(
            airborne_scope, chassis_kf_update_pair, NULL);
        return;
    }

    if ((landing_origin_scope || jump_kf_airborne_hold_latched) && off_ground) {
        /* Rebound invalidates the current contact ramp; restart only after the
         * landing detector confirms ground again. */
        jump_kf_reacquire_waiting = 1U;
        jump_kf_reacquire_contact_latched = 0U;
        jump_kf_reacquire_active = 0U;
        jump_kf_reacquire_elapsed_s = 0.0f;
    } else if (landing_contact_confirmed &&
               (jump_kf_reacquire_waiting ||
                !jump_kf_reacquire_contact_latched)) {
        const float launch_seed = jump_ground_vx_snapshot_valid ?
            jump_ground_vx_snapshot : chassis_vx_filter;
        jump_kf_reacquire_seed_vx = jump_landing_vx_contact_seed(
            launch_seed, chassis_vx_filter,
            jump_kf_reacquire_contact_seen);
        chassis_kf_seed_jump_vx(jump_kf_reacquire_seed_vx);
        jump_kf_reacquire_waiting = 0U;
        jump_kf_reacquire_contact_latched = 1U;
        jump_kf_reacquire_contact_seen = 1U;
        jump_kf_reacquire_active = 1U;
        jump_kf_reacquire_elapsed_s = 0.0f;
        jump_landing_vx_ref_confirm_contact(&jump_motion_vx_ref_state);
        jump_kf_airborne_hold_latched = 0U;
    } else if (!jump_kf_scope) {
        jump_kf_reacquire_waiting = 0U;
        jump_kf_reacquire_contact_latched = 0U;
        jump_kf_reacquire_contact_seen = 0U;
        jump_kf_reacquire_active = 0U;
        jump_kf_reacquire_elapsed_s = 0.0f;
    }

    if (jump_kf_reacquire_active) {
        float progress = jump_kf_reacquire_elapsed_s /
            JUMP_LANDING_VX_REACQUIRE_TIME_S;
        LIMIT_MIN_MAX(progress, 0.0f, 1.0f);
        wheel_to_ground_l = jump_landing_vx_reacquire_blend(
            jump_kf_reacquire_seed_vx, wheel_to_ground_l, progress);
        wheel_to_ground_r = jump_landing_vx_reacquire_blend(
            jump_kf_reacquire_seed_vx, wheel_to_ground_r, progress);
    }

    chassis_kf_l.MeasuredVector[0] = wheel_to_ground_l ;
    chassis_kf_l.MeasuredVector[1] = ins.motion_accel_b[1];//机体加速度作为整体的加速度

    chassis_kf_r.MeasuredVector[0] = wheel_to_ground_r ;
    chassis_kf_r.MeasuredVector[1] = ins.motion_accel_b[1];//机体加速度作为整体的加速度
    (void)jump_landing_kf_update_if_allowed(
        airborne_scope, chassis_kf_update_pair, NULL);

    /* Refresh the saved ground speed only after the landing estimator has
     * accepted wheel observations. If a rebound follows, the next contact
     * starts from this recent ground state instead of the original launch. */
    if (landing_contact_confirmed && isfinite(chassis_vx_filter)) {
        jump_ground_vx_snapshot = chassis_vx_filter;
        jump_ground_vx_snapshot_valid = 1U;
    }

    if (jump_kf_reacquire_active) {
        jump_kf_reacquire_elapsed_s += jump_kf_dt_s;
        if (jump_kf_reacquire_elapsed_s >=
            JUMP_LANDING_VX_REACQUIRE_TIME_S) {
            jump_kf_reacquire_elapsed_s =
                JUMP_LANDING_VX_REACQUIRE_TIME_S;
            jump_kf_reacquire_active = 0U;
        }
    }
}

/* --------------------------------- 底盘线程入口 --------------------------------- */

static void theta_level_comp_hard_clear(void)
{
#if THETA_LENGTH_CALIBRATION_ENABLE
    theta_level_comp[0] = THETA_COMP_CAL_INIT_LOW;
    theta_level_comp[1] = THETA_COMP_CAL_INIT_MID;
    theta_level_comp[2] = THETA_COMP_CAL_INIT_HIG;
#else
    theta_level_comp[0] = 0.0f;
    theta_level_comp[1] = 0.0f;
    theta_level_comp[2] = 0.0f;
#endif
    theta_level_apply_gain = 0.0f;
    leg_switch_freeze_timer = 0.0f;
    theta_comp_learn_settle_timer = 0.0f;
    theta_low_drift_trim = 0.0f;
    theta_low_drift_apply_gain = 0.0f;
    theta_low_drift_converge_timer = 0.0f;
    theta_low_drift_latched = 0U;
    theta_cal_vx_lpf = 0.0f;
    theta_cal_vx_slow_lpf = 0.0f;
    theta_cal_vx_trend = 0.0f;
    theta_cal_leg_settle_timer = 0.0f;
    theta_cal_zero_hold_timer = 0.0f;
    theta_cal_latch_bad_timer = 0.0f;
    theta_cal_last_level = LEG_LOW;
    memset(theta_cal_value_latched, 0,
           sizeof(theta_cal_value_latched));
    if (theta_comp_pid != NULL) {
        pid_clear(theta_comp_pid);
    }
}

static inline float theta_comp_poly_eval(float l0_average)
{
    float x;

    if (l0_average < THETA_COMP_POLY_L0_MIN) {
        l0_average = THETA_COMP_POLY_L0_MIN;
    } else if (l0_average > THETA_COMP_POLY_L0_MAX) {
        l0_average = THETA_COMP_POLY_L0_MAX;
    }

    x = (l0_average - THETA_COMP_POLY_L0_CENTER) /
        THETA_COMP_POLY_L0_SCALE;
    return THETA_COMP_POLY_C0 + x *
        (THETA_COMP_POLY_C1 + x * THETA_COMP_POLY_C2);
}

static float theta_comp_trim_interp(float l0_average)
{
    float blend;

    if (l0_average <= LEN_LEN_LOW) {
        return theta_level_comp[0];
    }
    if (l0_average < LEN_LEN_MID) {
        blend = (l0_average - LEN_LEN_LOW) / (LEN_LEN_MID - LEN_LEN_LOW);
        return theta_level_comp[0] +
               blend * (theta_level_comp[1] - theta_level_comp[0]);
    }
    if (l0_average < LEN_LEN_HIG) {
        blend = (l0_average - LEN_LEN_MID) / (LEN_LEN_HIG - LEN_LEN_MID);
        return theta_level_comp[1] +
               blend * (theta_level_comp[2] - theta_level_comp[1]);
    }
    return theta_level_comp[2];
}

static float apply_deadband_f32(float x, float deadband)
{
    if (fabsf(x) < deadband) {
        return 0.0f;
    }
    return x;
}

/* ---- Recovery 过渡状态判断 ---- */
static inline uint8_t recovery_has_stood_up(void)
{
    return chassis_recovery_module_has_stood_up();
}

static inline uint8_t recovery_after_handoff(void)
{
    return chassis_recovery_module_after_handoff();
}

static float move_towards_f32(float current, float target, float max_step)
{
    float error = target - current;
    if (error > max_step) {
        return current + max_step;
    }
    if (error < -max_step) {
        return current - max_step;
    }
    return target;
}

static uint8_t normal_leg_transition_mode(void)
{
    return (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
            chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
            chassis_cmd.ctrl_mode == CHASSIS_SPIN ||
            chassis_cmd.ctrl_mode == CHASSIS_AUTO);
}

/* When normal Roll feedback is large, an opposite inertial feedforward term
 * can cancel the correction that is trying to arrest body motion. Fade only
 * that opposing term continuously from 32 N to the fixed 40 N priority
 * boundary; same-direction and small-error commands retain their sum. */
static float roll_feedback_priority_lateral_force(
    float roll_force, float inertial_force, uint8_t normal_ground)
{
    const uint8_t feedforward_conflicts =
        roll_force != 0.0f && inertial_force != 0.0f &&
        roll_force * inertial_force < 0.0f;
    if (normal_ground && feedforward_conflicts &&
        fabsf(roll_force) > ROLL_FEEDBACK_PRIORITY_FORCE_START) {
        float opposing_gain =
            (ROLL_FEEDBACK_PRIORITY_FORCE_END - fabsf(roll_force)) /
            (ROLL_FEEDBACK_PRIORITY_FORCE_END -
             ROLL_FEEDBACK_PRIORITY_FORCE_START);
        LIMIT_MIN_MAX(opposing_gain, 0.0f, 1.0f);
        return roll_force + opposing_gain * inertial_force;
    }
    return roll_force + inertial_force;
}

static uint8_t roll_feedback_priority_scope_active(void)
{
    return (uint8_t)(
        (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
         chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL) &&
        !chassis_safety_module_off_ground() &&
        !chassis_safety_module_landing_buffer_active() &&
        !chassis_safety_module_landing_postbuffer_active() &&
        !chassis_safety_module_bump_pose_owner_active() &&
        !bump_control_active() &&
        !chassis_safety_module_wheel_shut());
}

static uint8_t roll_high_yaw_profile_active(void)
{
    static uint8_t high_yaw_hold = 0U;
    const uint8_t scope_valid =
        roll_feedback_priority_scope_active() &&
        jump_state == JUMP_STATE_IDLE &&
        chassis_cmd.bump_mode == 0U &&
        !chassis_safety_module_landing_airborne_guard_active() &&
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER;
    if (!scope_valid || !isfinite(dbg_yaw_rate_ref) ||
        !isfinite(ins.roll) || !isfinite(ins.gyro[1])) {
        /* A mode/safety transition must never carry high-yaw gains into a
         * jump, landing, BUMP, or danger owner. */
        high_yaw_hold = 0U;
        return 0U;
    }

    if (fabsf(dbg_yaw_rate_ref) >= ROLL_HIGH_YAW_RATE_MIN) {
        high_yaw_hold = 1U;
    } else if (high_yaw_hold &&
               fabsf(dbg_yaw_rate_ref) < ROLL_HIGH_YAW_RELEASE_RATE &&
               fabsf(ins.roll) < ROLL_HIGH_YAW_RELEASE_ROLL_DEG &&
               fabsf(ins.gyro[1]) < ROLL_HIGH_YAW_RELEASE_ROLL_RATE) {
        high_yaw_hold = 0U;
    }
    return high_yaw_hold;
}

static void support_force_ff_reset(void)
{
    memset(support_force_ff_state, 0, sizeof(support_force_ff_state));
    memset(dbg_support_ff_raw_ref, 0, sizeof(dbg_support_ff_raw_ref));
    memset(dbg_support_ff_correction, 0,
           sizeof(dbg_support_ff_correction));
}

static float support_force_ff_update(uint8_t side,
                                     float raw_ref,
                                     float legacy_support,
                                     float dt,
                                     uint8_t active)
{
    support_force_ff_state_t *state = &support_force_ff_state[side];
    dbg_support_ff_raw_ref[side] = raw_ref;

#if !SUPPORT_FORCE_FF_ENABLE
    (void)legacy_support;
    (void)dt;
    (void)active;
    memset(state, 0, sizeof(*state));
    dbg_support_ff_correction[side] = 0.0f;
    return 0.0f;
#else
    if (!active || !isfinite(raw_ref) || !isfinite(legacy_support)) {
        memset(state, 0, sizeof(*state));
        dbg_support_ff_correction[side] = 0.0f;
        return 0.0f;
    }

    if (!(dt > 0.0002f) || dt > 0.010f) {
        dt = 0.001f;
    }

    if (!state->initialized) {
        state->ref_lpf = raw_ref;
        state->last_ref_lpf = raw_ref;
        state->ref_dot = 0.0f;
        state->last_ref_dot = 0.0f;
        state->ref_ddot = 0.0f;
        state->correction = 0.0f;
        state->initialized = 1U;
    } else {
        const float alpha = dt / (SUPPORT_FORCE_FF_LPF_RC + dt);
        state->ref_lpf += alpha * (raw_ref - state->ref_lpf);
        state->ref_dot = (state->ref_lpf - state->last_ref_lpf) / dt;
        state->ref_ddot = (state->ref_dot - state->last_ref_dot) / dt;
    }

    float model_output = SUPPORT_FORCE_FF_C0 * state->ref_lpf +
                         SUPPORT_FORCE_FF_C1 * state->ref_dot +
                         SUPPORT_FORCE_FF_C2 * state->ref_ddot;
    float correction_target =
        SUPPORT_FORCE_FF_BLEND * (model_output - legacy_support);

    if (!isfinite(correction_target)) {
        memset(state, 0, sizeof(*state));
        dbg_support_ff_correction[side] = 0.0f;
        return 0.0f;
    }

    LIMIT_MIN_MAX(correction_target,
                  -SUPPORT_FORCE_FF_CORR_LIMIT,
                   SUPPORT_FORCE_FF_CORR_LIMIT);
    state->correction = move_towards_f32(
        state->correction, correction_target,
        SUPPORT_FORCE_FF_CORR_SLEW_RATE * dt);
    state->last_ref_lpf = state->ref_lpf;
    state->last_ref_dot = state->ref_dot;
    dbg_support_ff_correction[side] = state->correction;
    return state->correction;
#endif
}

static float leg_stuck_speed_measurement(uint8_t side)
{
    const chassis_kin_rate_observer_t *observer = &kin_rate_observer[side];

    /* The legacy single-cycle derivative is noisy enough to repeatedly reset
     * the stuck timer while the mechanism is physically stationary. Prefer
     * the timestamped five-sample observer once it is ready, but retain the
     * legacy signal during startup so compensation never uses stale data. */
    if (observer->sample_count >= KIN_RATE_WINDOW_SIZE &&
        isfinite(observer->l0_rate)) {
        return observer->l0_rate;
    }
    return leg[side]->d_l0_lpf;
}

static void leg_stuck_compensation_phase_begin(void)
{
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        leg_stuck_timer[side] = 0.0f;
        leg_stuck_error_sign[side] = 0;
        leg_stuck_burst_active[side] = 0U;
        leg_stuck_burst_used[side] = 0U;
    }
}

static void leg_stuck_compensation_reset(void)
{
    leg_stuck_compensation_phase_begin();
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        leg_stuck_force[side] = 0.0f;
    }
    leg_stuck_post_ramp_timer = 0.0f;
    leg_length_reference_ramping_prev = 0U;
    leg_stuck_last_ms = dwt_get_time_ms();
}

static float jump_state_elapsed_s(void)
{
    return (float)(uint32_t)(HAL_GetTick() - jump_state_enter_ms) * 0.001f;
}

static uint8_t jump_control_active(void)
{
    return jump_state != JUMP_STATE_IDLE;
}

static float jump_length_rate(chassis_side_e side)
{
    if (side >= CHASSIS_SIDE_COUNT) {
        return 0.0f;
    }
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    return jump_kin_rates_usable ? jump_kin_length_rate[side] : 0.0f;
#else
    return isfinite(leg[side]->d_l0_lpf) ? leg[side]->d_l0_lpf : 0.0f;
#endif
}

static jump_local_rate_selection_t jump_theta_rate_select(
    chassis_side_e side, float legacy_rate, uint8_t jump_owner_scope)
{
    const float observer_rate = side < CHASSIS_SIDE_COUNT ?
        jump_kin_theta_rate[side] : 0.0f;
    return jump_local_rate_select(
        JUMP_LOCAL_RATE_FEEDBACK_ENABLE,
        JUMP_KINEMATIC_RATE_OBSERVER_ENABLE,
        jump_owner_scope,
        jump_kin_rates_usable,
        observer_rate,
        legacy_rate);
}

static float jump_theta_rate_or_zero(
    chassis_side_e side, float legacy_rate, uint8_t jump_owner_scope)
{
    const jump_local_rate_selection_t selection =
        jump_theta_rate_select(side, legacy_rate, jump_owner_scope);
    return selection.valid ? selection.rate : 0.0f;
}

static void jump_kinematic_rate_update(float now_ms)
{
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    const float theta[CHASSIS_SIDE_COUNT] = {
        leg[LEFT]->theta, leg[RIGHT]->theta};
    const float length[CHASSIS_SIDE_COUNT] = {
        leg[LEFT]->l0, leg[RIGHT]->l0};
    jump_kin_rate_view_t view[CHASSIS_SIDE_COUNT];
    jump_kin_geometry_valid = 1U;

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (!isfinite(theta[side]) || !isfinite(length[side])) {
            jump_kin_geometry_valid = 0U;
        }
        (void)jump_kin_rate_observer_update(
            &jump_kin_rate_observer[side], theta[side], length[side], now_ms);
        view[side] = jump_kin_rate_observer_view(
            &jump_kin_rate_observer[side], now_ms);
    }

    jump_kin_rates_fresh = jump_kin_rate_pair_fresh(
        &view[LEFT], &view[RIGHT]);
    jump_kin_rates_usable = (uint8_t)(jump_kin_geometry_valid &&
        jump_kin_rate_pair_usable(&view[LEFT], &view[RIGHT]));
    jump_kin_rate_age_ms = fmaxf(view[LEFT].valid_rate_age_ms,
                                 view[RIGHT].valid_rate_age_ms);
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        jump_kin_length_rate[side] = jump_kin_rates_usable ?
            view[side].l0_rate : 0.0f;
        jump_kin_theta_rate[side] = jump_kin_rates_usable ?
            view[side].theta_rate : 0.0f;
    }
    chassis_safety_module_set_jump_kinematic_rates(
        jump_kin_length_rate[LEFT], jump_kin_length_rate[RIGHT],
        jump_kin_theta_rate[LEFT], jump_kin_theta_rate[RIGHT],
        jump_kin_rates_usable, jump_kin_rates_fresh, jump_kin_rate_age_ms);
#else
    jump_kin_geometry_valid = (uint8_t)(
        isfinite(leg[LEFT]->theta) && isfinite(leg[RIGHT]->theta) &&
        isfinite(leg[LEFT]->l0) && isfinite(leg[RIGHT]->l0));
    jump_kin_rates_fresh = jump_kin_geometry_valid;
    jump_kin_rates_usable = jump_kin_geometry_valid;
    jump_kin_rate_age_ms = 0.0f;
    jump_kin_length_rate[LEFT] = jump_length_rate(LEFT);
    jump_kin_length_rate[RIGHT] = jump_length_rate(RIGHT);
    jump_kin_theta_rate[LEFT] = leg[LEFT]->d_theta_lpf;
    jump_kin_theta_rate[RIGHT] = leg[RIGHT]->d_theta_lpf;
    chassis_safety_module_set_jump_kinematic_rates(
        jump_length_rate(LEFT), jump_length_rate(RIGHT),
        leg[LEFT]->d_theta_lpf, leg[RIGHT]->d_theta_lpf,
        jump_kin_rates_usable, jump_kin_rates_fresh, jump_kin_rate_age_ms);
#endif
}

static void jump_hip_feedback_snapshot_capture(
    jump_hip_feedback_snapshot_t *snapshot)
{
    static const uint8_t motor_order[4] = {3U, 0U, 2U, 1U};
    if (snapshot == NULL) {
        return;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    for (uint8_t i = 0U; i < 4U; ++i) {
        const uint8_t index = motor_order[i];
        if (dm_motor[index] != NULL) {
            snapshot->rx_count[i] = dm_motor[index]->measure.rx_count;
            snapshot->last_rx_ms[i] = dm_motor[index]->measure.last_rx_ms;
            snapshot->angle_abs[i] = dm_motor[index]->measure.angle_abs;
        } else {
            snapshot->rx_count[i] = 0U;
            snapshot->last_rx_ms[i] = 0U;
            snapshot->angle_abs[i] = NAN;
        }
    }
    __set_PRIMASK(primask);
}

static uint8_t jump_hip_feedback_snapshot_fresh(
    const jump_hip_feedback_snapshot_t *snapshot, uint32_t now_tick,
    uint32_t *max_age_ms)
{
    uint32_t age_ms[4] = {0U, 0U, 0U, 0U};
    if (max_age_ms != NULL) {
        *max_age_ms = UINT32_MAX;
    }
    if (snapshot == NULL ||
        !jump_landing_feedback_snapshot_fresh(
            now_tick, snapshot->rx_count, snapshot->last_rx_ms,
            JUMP_LANDING_RATE_FEEDBACK_MAX_AGE_MS,
            JUMP_LANDING_RATE_SNAPSHOT_MAX_SKEW_MS, age_ms)) {
        return 0U;
    }
    if (!(isfinite(snapshot->angle_abs[0]) &&
          isfinite(snapshot->angle_abs[1]) &&
          isfinite(snapshot->angle_abs[2]) &&
          isfinite(snapshot->angle_abs[3]) &&
          isfinite(leg[LEFT]->theta) &&
          isfinite(leg[RIGHT]->theta) &&
          isfinite(leg[LEFT]->l0) &&
          isfinite(leg[RIGHT]->l0))) {
        return 0U;
    }
    if (max_age_ms != NULL) {
        *max_age_ms = age_ms[0];
        for (uint8_t i = 1U; i < 4U; ++i) {
            if (age_ms[i] > *max_age_ms) {
                *max_age_ms = age_ms[i];
            }
        }
    }
    return 1U;
}

static void jump_landing_length_rate_update(
    float now_ms, uint32_t now_tick,
    const jump_hip_feedback_snapshot_t *snapshot,
    uint8_t feedback_fresh)
{
    jump_position_snapshot_fresh = feedback_fresh;
    chassis_safety_module_set_jump_position_feedback_fresh(feedback_fresh);

    const uint8_t jump_scope = (uint8_t)(
        jump_control_active() ||
        chassis_safety_module_landing_buffer_jump_origin() ||
        chassis_safety_module_landing_postbuffer_active());
    if (!jump_scope) {
        memset(jump_landing_length_rate_state, 0,
               sizeof(jump_landing_length_rate_state));
        jump_landing_length_rate[LEFT] = 0.0f;
        jump_landing_length_rate[RIGHT] = 0.0f;
        jump_landing_length_rate_age_ms = INFINITY;
        jump_landing_length_rate_valid = 0U;
        chassis_safety_module_set_jump_landing_length_rates(
            0.0f, 0.0f, 0U);
        return;
    }

    if (!feedback_fresh) {
        memset(jump_landing_length_rate_state, 0,
               sizeof(jump_landing_length_rate_state));
        jump_landing_length_rate_valid = 0U;
        jump_landing_length_rate[LEFT] = 0.0f;
        jump_landing_length_rate[RIGHT] = 0.0f;
        jump_landing_length_rate_age_ms = INFINITY;
        chassis_safety_module_set_jump_landing_length_rates(
            0.0f, 0.0f, 0U);
        return;
    }

    static const uint8_t pair_index[CHASSIS_SIDE_COUNT][2] = {
        {0U, 1U}, {2U, 3U}};
    uint8_t pair_fresh[CHASSIS_SIDE_COUNT] = {0U, 0U};
    jump_landing_rate_view_t view[CHASSIS_SIDE_COUNT];
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; ++side) {
        const uint8_t first = pair_index[side][0];
        const uint8_t second = pair_index[side][1];
        const uint32_t pair_counts[2] = {
            snapshot->rx_count[first], snapshot->rx_count[second]};
        const uint32_t pair_times[2] = {
            snapshot->last_rx_ms[first], snapshot->last_rx_ms[second]};
        uint32_t pair_ages[2] = {0U, 0U};
        pair_fresh[side] = jump_landing_feedback_pair_fresh(
            now_tick, pair_counts, pair_times, pair_ages);
        jump_landing_rate_state_expire(
            &jump_landing_length_rate_state[side], now_ms);
        if (pair_fresh[side]) {
            const float pair_time_ms = now_ms - 0.5f *
                ((float)pair_ages[0] + (float)pair_ages[1]);
            (void)jump_landing_rate_state_update(
                &jump_landing_length_rate_state[side], pair_counts, 1U,
                leg[side]->l0, pair_time_ms);
        }
        view[side] = jump_landing_rate_state_view(
            &jump_landing_length_rate_state[side], now_ms);
    }

    jump_landing_length_rate_age_ms = fmaxf(
        view[LEFT].valid_rate_age_ms, view[RIGHT].valid_rate_age_ms);
    jump_landing_length_rate_valid = (uint8_t)(
        feedback_fresh &&
        view[LEFT].usable && view[RIGHT].usable &&
        jump_landing_length_rate_age_ms <=
            JUMP_LANDING_RATE_MAX_VALID_AGE_MS);
    jump_landing_length_rate[LEFT] = jump_landing_length_rate_valid ?
        view[LEFT].rate : 0.0f;
    jump_landing_length_rate[RIGHT] = jump_landing_length_rate_valid ?
        view[RIGHT].rate : 0.0f;
    chassis_safety_module_set_jump_landing_length_rates(
        jump_landing_length_rate[LEFT], jump_landing_length_rate[RIGHT],
        jump_landing_length_rate_valid);
}

static uint8_t jump_air_wheel_admitted(void)
{
    const uint8_t normal_mode =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL;
    const uint8_t normal_stand =
        chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK;
    const uint8_t safe_state =
        chassis_fdb_data.stand_state != CHASSIS_IS_DANGER;
    const uint8_t active_jump =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        jump_control_active() && safe_state;
    const uint8_t guard_airborne = jump_reaction_wheel_guard_admitted(
        JUMP_AIR_REACTION_WHEEL_ENABLE,
        chassis_safety_module_landing_airborne_guard_active(),
        chassis_safety_module_off_ground(), normal_mode, normal_stand,
        safe_state);
    return jump_reaction_wheel_admitted(
        JUMP_AIR_REACTION_WHEEL_ENABLE,
        active_jump || guard_airborne,
        jump_state == JUMP_STATE_FLIGHT || guard_airborne,
        chassis_safety_module_off_ground(),
        chassis_safety_module_jump_air_profile_active(),
        jump_state == JUMP_STATE_THRUST,
        jump_thrust_active,
        chassis_safety_module_landing_buffer_active());
}

/* The LQR state buffer is not a reliable airborne pitch-rate source: during
 * the fast jump path it can retain the last grounded sample while the IMU
 * attitude is already moving.  The INS task publishes the Euler-rate paired
 * with the published pitch angle; use it while the sample is fresh and keep
 * the LQR value only as a bounded compatibility fallback. */
static float jump_body_pitch_rate_feedback(void)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint32_t age_ms = now_ms - ins.publish_tick_ms;
    const float direct_rate = ins.pitch_rate_rad_s;
    if (isfinite(direct_rate) && fabsf(direct_rate) <= 20.0f &&
        age_ms <= 30U) {
        return direct_rate;
    }

    const float gyro_rate = -ins.gyro[0];
    if (isfinite(gyro_rate) && fabsf(gyro_rate) <= 20.0f) {
        return gyro_rate;
    }

    const float lqr_rate = -LQRXObsBuf[LEFT][5];
    return isfinite(lqr_rate) ? lqr_rate : 0.0f;
}

static uint8_t jump_air_wheel_output_admitted(void)
{
#ifdef M3508_SET_ZERO
    return 0U;
#else
    return jump_air_wheel_admitted();
#endif
}

static void jump_state_enter(jump_state_e state)
{
    jump_state = state;
    jump_state_enter_ms = HAL_GetTick();
    jump_thrust_active = 0U;
    if (state == JUMP_STATE_PRELOAD ||
        state == JUMP_STATE_ABORT_SETTLE) {
        jump_stroke_release_reason = JUMP_STROKE_RELEASE_NONE;
        jump_stroke_release_elapsed_origin_s = 0.0f;
    }
    if (state == JUMP_STATE_PRELOAD) {
        jump_landing_theta_ref = 0.0f;
        jump_landing_owner_theta_ref_valid = 0U;
        jump_landing_balance_reset(&jump_landing_balance_state);
        jump_landing_ground_balance_gain = 0.0f;
        jump_ground_recovery_reset(&jump_ground_recovery_state);
        jump_ground_recovery_gain = 0.0f;
        jump_ground_recovery_scope_active = 0U;
        jump_ground_wheel_cache_valid = 0U;
    }
    chassis_safety_module_set_jump_legs_unloaded(0U);
    jump_landing_ground_timer = 0.0f;
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
    leg_stuck_compensation_phase_begin();
    leg_stuck_force[LEFT] = 0.0f;
    leg_stuck_force[RIGHT] = 0.0f;
    if (state == JUMP_STATE_THRUST) {
        /* Capture the actual preloaded geometry.  Starting every jump with a
         * hard theta=0 target made original jumps cross vertical with large
         * residual rate, while moving jumps pitched the chassis forward. */
        jump_thrust_theta_ref = 0.5f *
            (leg[LEFT]->theta + leg[RIGHT]->theta);
    }
    if (state == JUMP_STATE_LANDING) {
        /* Hold the leg-rod angle captured at touchdown instead of letting the
         * ground length loop fold the rods through the phi0 safety limit. */
        if (!jump_landing_owner_theta_ref_valid) {
            jump_landing_theta_ref = 0.5f *
                (leg[LEFT]->theta + leg[RIGHT]->theta);
            jump_landing_owner_theta_ref_valid =
                (uint8_t)isfinite(jump_landing_theta_ref);
        }
        jump_landing_theta_hold_tp = 0.0f;
        /* Item 1: remember the length we arrived with so the landing confirm can
         * require a real compression from THIS touchdown. */
        jump_landing_entry_length = 0.5f *
            (leg[LEFT]->l0 + leg[RIGHT]->l0);
        if (!isfinite(jump_landing_entry_length)) {
            jump_landing_entry_length = JUMP_LANDING_COMPRESS_LENGTH_MAX;
        }
    }
    jump_thrust_theta_ref_rate = 0.0f;
}

/* 2026-09-23 item 3: thrust force fade.  1.0 up to FADE_START, then linear to
 * 0 at the mechanical limit, so the extension can never slam the leg into its
 * stop with full launch force still applied.  Evaluated from the CURRENT length
 * every cycle, so it is a pure function of geometry and cannot wind up. */
static float jump_thrust_force_scale(float average_length)
{
    const float span = JUMP_THRUST_TARGET_LENGTH - JUMP_THRUST_FORCE_FADE_START;
    float scale = (JUMP_THRUST_TARGET_LENGTH - average_length) / span;

    LIMIT_MIN_MAX(scale, 0.0f, 1.0f);
    return scale;
}

static void jump_control_finish(jump_fault_e fault)
{
    const uint8_t preserve_landing_owner =
        chassis_safety_module_jump_landing_owner_active() &&
        !chassis_safety_module_off_ground();
    chassis_safety_module_jump_air_profile_cancel();
    chassis_safety_module_set_jump_legs_unloaded(0U);
    jump_fault = fault;
    if (fault != JUMP_FAULT_NONE) {
        jump_ground_recovery_cancel(&jump_ground_recovery_state);
        jump_ground_recovery_gain = 0.0f;
        jump_ground_recovery_scope_active = 0U;
        jump_ground_wheel_common_cache = 0.0f;
        jump_ground_wheel_cache_valid = 0U;
    }
    jump_force_boost = 0.0f;
    jump_length_sync_force = 0.0f;
    jump_thrust_pitch_tp = 0.0f;
    jump_thrust_theta_hold_tp = 0.0f;
    jump_thrust_theta_ref = 0.0f;
    jump_thrust_theta_ref_rate = 0.0f;
    jump_landing_tp_handoff_timer = 0.0f;
    if (!preserve_landing_owner) {
        jump_landing_theta_ref = 0.0f;
        jump_landing_owner_theta_ref_valid = 0U;
        jump_landing_balance_reset(&jump_landing_balance_state);
        jump_landing_ground_balance_gain = 0.0f;
    }
    jump_landing_theta_hold_tp = 0.0f;
    jump_air_wheel_torque = 0.0f;
    jump_thrust_active = 0U;
    jump_stroke_release_reason = JUMP_STROKE_RELEASE_NONE;
    jump_stroke_release_elapsed_origin_s = 0.0f;
    jump_length_target = LEN_LEN_LOW;
    jump_state = JUMP_STATE_IDLE;
    jump_state_enter_ms = HAL_GetTick();
    if (fault != JUMP_FAULT_NONE) {
        jump_landing_speed_state_reset();
        jump_motion_vx_ref_state.cancelled =
            chassis_safety_module_off_ground();
    }
    jump_landing_ground_timer = 0.0f;
    if (!preserve_landing_owner) {
        jump_landing_return_ramp_active = 0U;
        jump_landing_lqr_return_gain = 1.0f;
        jump_landing_wheel_torque = 0.0f;
        jump_landing_wheel_fade.active = 0U;
        jump_landing_wheel_fade_duration_s =
            JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
        jump_landing_buffer_owner_previous = 0U;
        memset(jump_landing_wheel_last_owner_command_valid, 0,
               sizeof(jump_landing_wheel_last_owner_command_valid));
        jump_landing_wheel_buffer_return_gain = 0.0f;
    }
    if (!chassis_safety_module_off_ground()) {
        chassis_safety_module_set_wheel_shut(0U);
    }
    if (chassis_fdb_data.stand_state != CHASSIS_IS_DANGER) {
        chassis_fdb_data.stand_state = CHASSIS_IS_JUMPOK;
    }
    if (!preserve_landing_owner) {
        pid_clear(L_length_pid);
        pid_clear(R_length_pid);
    }
}

static void jump_control_fail_to_recovery(jump_fault_e fault)
{
    chassis_safety_module_jump_air_profile_cancel();
    chassis_safety_module_set_jump_legs_unloaded(0U);
    jump_fault = fault;
    jump_landing_speed_state_reset();
    jump_motion_vx_ref_state.cancelled = chassis_safety_module_off_ground();
    jump_force_boost = 0.0f;
    jump_length_sync_force = 0.0f;
    jump_thrust_pitch_tp = 0.0f;
    jump_thrust_theta_hold_tp = 0.0f;
    jump_thrust_theta_ref = 0.0f;
    jump_thrust_theta_ref_rate = 0.0f;
    jump_landing_tp_handoff_timer = 0.0f;
    jump_landing_theta_ref = 0.0f;
    jump_landing_owner_theta_ref_valid = 0U;
    jump_landing_balance_reset(&jump_landing_balance_state);
    jump_landing_ground_balance_gain = 0.0f;
    jump_ground_recovery_cancel(&jump_ground_recovery_state);
    jump_ground_recovery_gain = 0.0f;
    jump_ground_recovery_scope_active = 0U;
    jump_ground_wheel_cache_valid = 0U;
    jump_landing_theta_hold_tp = 0.0f;
    jump_air_wheel_torque = 0.0f;
    jump_thrust_active = 0U;
    jump_stroke_release_reason = JUMP_STROKE_RELEASE_NONE;
    jump_stroke_release_elapsed_origin_s = 0.0f;
    jump_length_target = LEN_LEN_LOW;
    jump_state = JUMP_STATE_IDLE;
    jump_state_enter_ms = HAL_GetTick();
    jump_landing_ground_timer = 0.0f;
    jump_landing_return_ramp_active = 0U;
    jump_landing_lqr_return_gain = 1.0f;
    jump_landing_wheel_torque = 0.0f;
    jump_landing_wheel_fade.active = 0U;
    jump_landing_wheel_fade_duration_s =
        JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
    jump_landing_buffer_owner_previous = 0U;
    memset(jump_landing_wheel_last_owner_command_valid, 0,
           sizeof(jump_landing_wheel_last_owner_command_valid));
    jump_landing_wheel_buffer_return_gain = 0.0f;
    /* Keep the wheel disabled only while contact is genuinely absent.  The
     * following command cycle enters RECOVERY, which owns the fallen pose. */
    chassis_safety_module_set_wheel_shut(
        chassis_safety_module_off_ground());
    chassis_fdb_data.stand_state = CHASSIS_IS_DANGER;
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
}

static void jump_control_reset(void)
{
    chassis_safety_module_jump_air_profile_cancel();
    chassis_safety_module_set_jump_legs_unloaded(0U);
    jump_state = JUMP_STATE_IDLE;
    jump_fault = JUMP_FAULT_NONE;
    jump_landing_speed_state_reset();
    jump_motion_vx_ref_state.cancelled = chassis_safety_module_off_ground();
    jump_state_enter_ms = HAL_GetTick();
    jump_takeoff_seen = 0U;
    jump_landing_seen = 0U;
    jump_preload_ready = 0U;
    jump_thrust_active = 0U;
    jump_stroke_release_reason = JUMP_STROKE_RELEASE_NONE;
    jump_stroke_release_elapsed_origin_s = 0.0f;
    jump_request_seen = 0U;
    jump_length_target = LEN_LEN_LOW;
    jump_force_boost = 0.0f;
    jump_length_sync_force = 0.0f;
    jump_thrust_pitch_tp = 0.0f;
    jump_thrust_pitch_rate_lpf = 0.0f;
    jump_thrust_theta_hold_tp = 0.0f;
    jump_thrust_theta_ref = 0.0f;
    jump_thrust_theta_ref_rate = 0.0f;
    jump_launch_vx = 0.0f;
    jump_landing_tp_handoff[LEFT] = 0.0f;
    jump_landing_tp_handoff[RIGHT] = 0.0f;
    jump_landing_tp_handoff_timer = 0.0f;
    jump_landing_theta_ref = 0.0f;
    jump_landing_owner_theta_ref_valid = 0U;
    jump_landing_balance_reset(&jump_landing_balance_state);
    jump_landing_ground_balance_gain = 0.0f;
    jump_ground_recovery_reset(&jump_ground_recovery_state);
    jump_ground_recovery_gain = 0.0f;
    jump_ground_recovery_scope_active = 0U;
    jump_ground_wheel_common_cache = 0.0f;
    jump_ground_wheel_cache_valid = 0U;
    jump_landing_theta_hold_tp = 0.0f;
    jump_air_wheel_torque = 0.0f;
    jump_vertical_speed_estimate = 0.0f;
    jump_landing_ground_timer = 0.0f;
    jump_landing_return_ramp_active = 0U;
    jump_landing_lqr_return_gain = 1.0f;
    jump_landing_wheel_torque = 0.0f;
    jump_landing_wheel_buffer_return_gain = 0.0f;
    jump_landing_wheel_fade.active = 0U;
    jump_landing_wheel_fade_duration_s =
        JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
    jump_landing_buffer_owner_previous = 0U;
    memset(jump_landing_wheel_last_owner_command_valid, 0,
           sizeof(jump_landing_wheel_last_owner_command_valid));
}

static uint8_t jump_precheck_ok(void)
{
    const uint8_t stand_ok =
        chassis_fdb_data.stand_state == CHASSIS_IS_STAND ||
        chassis_fdb_data.stand_state == CHASSIS_IS_JUMPOK;
    const uint8_t source_mode_ok =
        chassis_cmd.last_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.last_mode == CHASSIS_OPEN_LOOP ||
        (chassis_cmd.last_mode == CHASSIS_JUMP &&
         chassis_cmd.jump_request);
    const float average_length =
        0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    /* 2026-09-23: read the live gyro instead of `dbg_yaw_rate_fdb`.  That cached
     * value is only refreshed by Chassis_Vx_Detect()/update_yaw(), and
     * chassis_task does not call update_yaw() in CHASSIS_JUMP, so a request that
     * was latched and then retried here would have been gated on the yaw rate
     * captured before the mode changed.  `dbg_yaw_rate_fdb` is exactly
     * YAW_GYRO_SIGN * ins.gyro[2], so this is the same quantity, only current. */
    const float yaw_rate_fdb = YAW_GYRO_SIGN * ins.gyro[2];

    return stand_ok && source_mode_ok &&
           (!JUMP_KINEMATIC_RATE_OBSERVER_ENABLE ||
            jump_kin_rates_fresh) &&
           !chassis_safety_module_off_ground() &&
           !chassis_safety_module_landing_buffer_active() &&
           fabsf(ins.pitch) <= JUMP_PRECHECK_PITCH_MAX_DEG &&
           fabsf(ins.roll) <= JUMP_PRECHECK_ROLL_MAX_DEG &&
           fabsf(chassis_vx_filter) <= JUMP_PRECHECK_VX_MAX &&
           isfinite(yaw_rate_fdb) &&
           fabsf(yaw_rate_fdb) <= JUMP_PRECHECK_YAW_RATE_MAX &&
           isfinite(average_length) &&
           average_length >= ROLL_LENGTH_TARGET_MIN &&
           average_length <= ROLL_LENGTH_TARGET_MAX;
}

static void jump_control_try_start(void)
{
    const uint32_t now = HAL_GetTick();

    if (jump_last_accept_valid &&
        (uint32_t)(now - jump_last_accept_ms) < JUMP_COOLDOWN_MS) {
        jump_fault = JUMP_FAULT_COOLDOWN;
        return;
    }

    if (!jump_precheck_ok()) {
        jump_fault = JUMP_FAULT_PRECHECK;
        return;
    }

    jump_fault = JUMP_FAULT_NONE;
    jump_landing_speed_state_reset();
    jump_motion_vx_ref_state.cancelled = 0U;
    jump_kf_airborne_hold_latched = 0U;
    if (isfinite(chassis_vx_filter)) {
        jump_ground_vx_snapshot = chassis_vx_filter;
        jump_ground_vx_snapshot_valid = 1U;
    }
    jump_last_accept_ms = now;
    jump_last_accept_valid = 1U;
    jump_sequence_start_ms = now;   /* item 2: global failsafe reference */
    jump_air_theta_ref_valid = 0U;  /* re-capture the takeoff leg angle */
    jump_takeoff_seen = 0U;
    jump_landing_seen = 0U;
    jump_preload_ready = 0U;
    jump_vertical_speed_estimate = 0.0f;
    jump_launch_vx = chassis_vx_filter;
    if (!isfinite(jump_launch_vx)) {
        jump_launch_vx = 0.0f;
    }
    jump_thrust_pitch_rate_lpf = -LQRXObsBuf[LEFT][5];
    if (!isfinite(jump_thrust_pitch_rate_lpf)) {
        jump_thrust_pitch_rate_lpf = 0.0f;
    }
    jump_landing_tp_handoff_timer = 0.0f;
    jump_landing_ground_timer = 0.0f;
    jump_landing_return_ramp_active = 0U;
    jump_landing_lqr_return_gain = 1.0f;
    jump_landing_wheel_torque = 0.0f;
    jump_landing_wheel_buffer_return_gain = 0.0f;
    lqr_virtual_x_l = 0.0f;
    lqr_virtual_x_r = 0.0f;
    LQRXRefBuf[LEFT][2] = 0.0f;
    LQRXRefBuf[RIGHT][2] = 0.0f;
    LQRXRefBuf[LEFT][3] = 0.0f;
    LQRXRefBuf[RIGHT][3] = 0.0f;
    yaw_target = ins.yaw_total_angle * DEGREE_2_RAD;
    support_force_ff_reset();
    chassis_safety_module_reset();
    /* A new attempt must not inherit the previous abort's danger verdict. */
    chassis_safety_module_clear_danger_latch();
    jump_state_enter(JUMP_STATE_PRELOAD);
    chassis_fdb_data.stand_state = CHASSIS_IS_JUMPING;
    chassis_safety_module_set_wheel_shut(0U);
}

static void jump_control_prepare_cycle(uint8_t *stroke_release_invalid)
{
    if (stroke_release_invalid != NULL) {
        *stroke_release_invalid = 0U;
    }
    jump_request_seen = chassis_cmd.jump_request ? 1U : 0U;
    if (jump_state == JUMP_STATE_IDLE &&
        chassis_cmd.ctrl_mode == CHASSIS_JUMP &&
        chassis_cmd.jump_request) {
        jump_control_try_start();
    }

    if (!jump_control_active()) {
        chassis_safety_module_set_jump_legs_unloaded(0U);
        jump_force_boost = 0.0f;
        return;
    }

    if (chassis_cmd.ctrl_mode != CHASSIS_JUMP ||
        chassis_fdb_data.stand_state == CHASSIS_IS_DANGER) {
        chassis_safety_module_jump_air_profile_cancel();
        chassis_safety_module_set_jump_legs_unloaded(0U);
        jump_fault = JUMP_FAULT_SAFETY;
        jump_state = JUMP_STATE_IDLE;
        jump_landing_speed_state_reset();
        jump_motion_vx_ref_state.cancelled =
            chassis_safety_module_off_ground();
        jump_ground_recovery_cancel(&jump_ground_recovery_state);
        jump_ground_recovery_gain = 0.0f;
        jump_ground_recovery_scope_active = 0U;
        jump_ground_wheel_common_cache = 0.0f;
        jump_ground_wheel_cache_valid = 0U;
        jump_force_boost = 0.0f;
        jump_thrust_active = 0U;
        jump_stroke_release_reason = JUMP_STROKE_RELEASE_NONE;
        jump_stroke_release_elapsed_origin_s = 0.0f;
        return;
    }

    chassis_fdb_data.stand_state = CHASSIS_IS_JUMPING;
    /* 2026-09-23 item 2: global jump failsafe.  Checked before any state work,
     * so an unresolved airborne sequence cannot hold the chassis in JUMP past
     * the total budget; a grounded landing owner may finish its bounded exit. */
    if (jump_landing_total_timeout_expired(
            (uint32_t)(HAL_GetTick() - jump_sequence_start_ms),
            (uint32_t)(JUMP_TOTAL_TIMEOUT * 1000.0f),
            chassis_safety_module_jump_landing_owner_active(),
            chassis_safety_module_off_ground())) {
        jump_control_fail_to_recovery(JUMP_FAULT_LANDING_TIMEOUT);
        return;
    }
    /* Grounded preload/thrust still need normal wheel LQR to keep Pitch.  The
     * wheel becomes a strict zero-output actuator only after off-ground is
     * actually confirmed. */
    if (jump_state == JUMP_STATE_PRELOAD) {
        chassis_safety_module_set_wheel_shut(0U);
    } else {
        chassis_safety_module_set_wheel_shut(
            chassis_safety_module_off_ground());
    }
    chassis_cmd.leg_level = LEG_LOW;
    /* Keep the entry longitudinal command for a running jump.  The wheel LQR
     * owns it while grounded; in FLIGHT only the bounded reaction-wheel pitch
     * channel is allowed.  Yaw remains held because an airborne turn has no
     * safe wheel reaction. */
    chassis_cmd.vy = 0.0f;
    chassis_cmd.vy_set = 0.0f;
    chassis_cmd.vw_relative = 0.0f;
    chassis_cmd.vw_relative_set = 0.0f;

    float dt = jump_cycle_dt_s;
    if (!(dt > 0.0002f) || dt > 0.020f) {
        dt = 0.001f;
    }
    jump_vertical_speed_estimate += ins.motion_accel_b[2] * dt;
    LIMIT_MIN_MAX(jump_vertical_speed_estimate, -5.0f, 5.0f);
    /* 2026-09-23: airborne leg-angle reference.  Capture the real takeoff angle
     * the first time the timed tuck starts, then drive it to
     * JUMP_AIR_THETA_REF_RAD at a bounded rate.  The airborne joint controller
     * (chassis_safety_module_apply_airborne_control) holds theta against this
     * reference; it used to be hard-coded to 0.0, which turned the only
     * remaining attitude actuator into a spring pulling the rods vertical. */
    if (jump_state == JUMP_STATE_THRUST && !jump_air_theta_ref_valid) {
        float takeoff_theta = 0.5f *
            (leg[LEFT]->theta + leg[RIGHT]->theta);
        if (!isfinite(takeoff_theta)) {
            takeoff_theta = JUMP_AIR_THETA_REF_RAD;
        }
        jump_air_theta_ref = takeoff_theta;
        jump_air_theta_ref_valid = 1U;
    }
    if (jump_air_theta_ref_valid) {
        jump_air_theta_ref = move_towards_f32(
            jump_air_theta_ref, JUMP_AIR_THETA_REF_RAD,
            JUMP_AIR_THETA_REF_SLEW_RATE * dt);
        chassis_safety_module_set_jump_air_theta_ref(jump_air_theta_ref);
    }
    /* motion_accel_b is gravity-removed, so its integral remains the vertical
     * speed diagnostic used by the safety profile. Deployment itself now waits
     * for the actual compact dwell or the 0.25 s hard deadline; the fixed
     * 0.22 s touch/impact inhibit remains independent. */
    chassis_safety_module_jump_air_profile_set_descent(
        jump_vertical_speed_estimate < JUMP_AIR_DEPLOY_DESCENT_VZ);

    /* 2026-09-23 (jump18/jump19.csv): EARLY leg-angle guard.  The chassis safety
     * module only flags phi0 once the rod is within 20 deg of horizontal, and
     * that check needs 80 ms of confirmation -- by which time the rod has
     * already crossed vertical and theta has WRAPPED (theta = pi/2 - phi0 +
     * pitch).  jump19 measured both rods running phi0 120 -> 174 -> -165 deg
     * while theta wrapped from -0.62 to +5.29 rad: the legs physically rotated a
     * full turn, exactly what the operator saw.  theta is small and unambiguous
     * well before phi0 gets anywhere near its limit, so abort on it directly and
     * stop the jump while the rod is still on the safe side of vertical. */
    {
        const float theta_L = leg[LEFT]->theta;
        const float theta_R = leg[RIGHT]->theta;
        /* Once the safety module has confirmed a jump-origin touchdown, the
         * impact buffer owns the leg pose.  The rods can swing past the
         * airborne 0.85 rad guard while the buffer absorbs the landing; the
         * control task runs this function before update_ground() publishes the
         * next edge, so applying the guard here would abort and reset the
         * freshly opened buffer on the first rebound.  Keep the guard active
         * through PRELOAD/THRUST/FLIGHT and until that owner is established. */
        const uint8_t landing_owner =
            jump_state == JUMP_STATE_LANDING ||
            chassis_safety_module_jump_landing_owner_active();
        if (!landing_owner &&
            (fabsf(theta_L) > JUMP_ABORT_THETA_MAX_RAD ||
             fabsf(theta_R) > JUMP_ABORT_THETA_MAX_RAD)) {
            jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
            return;
        }
    }

    if (fabsf(ins.pitch) > JUMP_ABORT_PITCH_MAX_DEG ||
        fabsf(ins.roll) > JUMP_ABORT_ROLL_MAX_DEG) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        return;
    }

    const float elapsed = jump_state_elapsed_s();
    switch (jump_state) {
    case JUMP_STATE_PRELOAD: {
        const float average_length =
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        const float average_length_rate = 0.5f *
            (fabsf(leg[LEFT]->d_l0_lpf) +
             fabsf(leg[RIGHT]->d_l0_lpf));
        jump_length_target = JUMP_PRELOAD_LENGTH;
        jump_force_boost = 0.0f;
        jump_preload_ready =
            average_length <= JUMP_PRELOAD_READY_LENGTH &&
            average_length_rate <= JUMP_PRELOAD_READY_RATE;
        if (jump_preload_ready && elapsed >= JUMP_PRELOAD_MIN_TIME) {
            jump_state_enter(JUMP_STATE_THRUST);
            jump_length_target = JUMP_THRUST_TARGET_LENGTH;
        } else if (elapsed >= JUMP_PRELOAD_MAX_TIME) {
            jump_control_finish(JUMP_FAULT_PRELOAD_TIMEOUT);
        }
        break;
    }
    case JUMP_STATE_THRUST: {
        /* Extend toward the mechanical limit with the existing length target
         * and travel fade.  Once average length first reaches the target, latch
         * the existing pre-tuck release path so a later compression cannot
         * re-enable powered extension while contact confirmation is delayed.
         * The 0.32 s timer remains the maximum powered-window and fallback
         * release; it does not replace the earlier stroke-completion edge. */
        const float avg_thrust_length = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        const jump_stroke_release_result_t stroke_release =
            jump_stroke_release_step(
                1U,
                (uint8_t)(JUMP_STROKE_COMPLETION_RELEASE_ENABLE != 0),
                avg_thrust_length,
                elapsed,
                JUMP_THRUST_TARGET_LENGTH,
                JUMP_THRUST_BOOST_TIME,
                jump_stroke_release_reason,
                jump_stroke_release_elapsed_origin_s);
        if (!stroke_release.valid) {
            if (stroke_release_invalid != NULL) {
                *stroke_release_invalid = 1U;
            }
            jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
            return;
        }
        jump_stroke_release_reason = stroke_release.reason;
        jump_stroke_release_elapsed_origin_s =
            stroke_release.release_elapsed_origin_s;
        jump_thrust_active = stroke_release.powered;
        if (jump_thrust_active) {
            jump_length_target = JUMP_THRUST_TARGET_LENGTH;
        } else {
            /* A completed extension is a latched stroke edge.  Do not restart
             * the thrust if the leg compresses while contact confirmation is
             * still delayed.  The existing pre-tuck controller owns the same
             * release path for both travel and timer causes. */
            chassis_safety_module_jump_air_profile_prestart(
                stroke_release.profile_elapsed_s);
            jump_length_target = 0.5f *
                (chassis_safety_module_air_length_target(
                     CHASSIS_SIDE_LEFT) +
                 chassis_safety_module_air_length_target(
                     CHASSIS_SIDE_RIGHT));
        }
        /* Full launch force until 0.26 m, then linear to zero at the target. */
        const float force_scale =
            jump_thrust_force_scale(avg_thrust_length);
        jump_force_boost = jump_thrust_active ?
            JUMP_THRUST_BOOST_FORCE * force_scale : 0.0f;
        /* "Extended" now means the legs have finished their travel rather than
         * having passed an arbitrary 0.305 m waypoint. */
        const uint8_t thrust_extended =
            (avg_thrust_length >= (JUMP_THRUST_TARGET_LENGTH - 0.03f));
        const uint8_t thrust_boost_finished =
            (elapsed >= (JUMP_THRUST_BOOST_TIME + 0.05f));
        const uint8_t deterministic_takeoff =
            thrust_boost_finished && thrust_extended;
        const uint8_t takeoff_allowed =
            (elapsed >= (JUMP_THRUST_BOOST_TIME - 0.04f)) || thrust_extended;

        if (takeoff_allowed &&
            (chassis_safety_module_off_ground() || deterministic_takeoff)) {
            if (!chassis_safety_module_off_ground()) {
                chassis_safety_module_force_airborne();
            }
            jump_takeoff_seen = 1U;
            jump_state_enter(JUMP_STATE_FLIGHT);
            jump_length_target = LEN_LEN_LOW;
            jump_force_boost = 0.0f;
        } else if (elapsed >= JUMP_THRUST_MAX_TIME ||
                   (thrust_boost_finished && !thrust_extended)) {
            jump_fault = JUMP_FAULT_NO_TAKEOFF;
            jump_state_enter(JUMP_STATE_ABORT_SETTLE);
            chassis_safety_module_jump_air_profile_cancel();
            chassis_safety_module_set_wheel_shut(0U);
            /* Capture the actual extension for a bumpless failed-jump exit.
             * The legs commonly coast beyond the 0.285 m thrust reference;
             * returning to that stale reference immediately created a large
             * negative axial-force spike. */
            jump_length_target = 0.5f *
                (leg[LEFT]->l0 + leg[RIGHT]->l0);
            LIMIT_MIN_MAX(jump_length_target,
                          LEN_LEN_LOW,
                          LANDING_LENGTH_REF_MAX);
            jump_force_boost = 0.0f;
        }
        break;
        }

    case JUMP_STATE_FLIGHT: {
        /* The nominal target stays LOW so the existing landing module returns
         * there after impact.  Its independent airborne deployment reference
         * extends the legs toward 0.27 m while contact is absent. */
        jump_length_target = LEN_LEN_LOW;
        jump_force_boost = 0.0f;
        const uint8_t flight_landing_allowed = (elapsed >= 0.18f);
        if (flight_landing_allowed && !chassis_safety_module_off_ground() && jump_takeoff_seen) {
            jump_landing_seen = 1U;
            jump_vertical_speed_estimate = 0.0f;
            jump_state_enter(JUMP_STATE_LANDING);
        } else if (elapsed >= JUMP_FLIGHT_TIMEOUT) {
            /* A small test jump cannot remain airborne this long.  Persistent
             * off-ground means the chassis has fallen or contact detection is
             * invalid, so hand control to RECOVERY instead of latching JUMP. */
            jump_control_fail_to_recovery(JUMP_FAULT_FLIGHT_TIMEOUT);
        }
        break;
    }
    case JUMP_STATE_LANDING: {
        jump_length_target = LEN_LEN_LOW;
        jump_force_boost = 0.0f;

        const uint8_t posture_settled =
            fabsf(ins.pitch) <= JUMP_LANDING_PITCH_MAX_DEG &&
            fabsf(ins.roll) <= JUMP_LANDING_ROLL_MAX_DEG;
        const uint8_t legs_settled =
            fabsf(leg[LEFT]->l0 - leg[RIGHT]->l0) <=
                JUMP_LANDING_LENGTH_DIFF_MAX;
        /* 2026-09-23 item 1: real leg compression is the landing evidence.
         * Require both legs to have compressed from the length captured at
         * touchdown (or to be shorter than the compressed threshold outright).
         * This is what the reference chassis gates its soft-landing phase on,
         * and unlike the filtered contact edge it cannot be fooled by the
         * estimator bias that made jump13 toggle the off-ground flag 0/1
         * through the whole capture window. */
        const float landing_average_length =
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        const uint8_t legs_compressed =
            (landing_average_length <= JUMP_LANDING_COMPRESS_LENGTH_MAX) ||
            (landing_average_length <=
             (jump_landing_entry_length - JUMP_LANDING_COMPRESS_MIN));
        const uint8_t stable_ground =
            !chassis_safety_module_off_ground() &&
            posture_settled && legs_settled && legs_compressed;

        if (stable_ground) {
            jump_landing_ground_timer += dt;
            if (jump_landing_ground_timer >
                JUMP_LANDING_GROUND_CONFIRM_TIME) {
                jump_landing_ground_timer =
                    JUMP_LANDING_GROUND_CONFIRM_TIME;
            }
        } else {
            /* Every rebound or unsafe posture starts a fresh confirmation.
             * Do not gate release on noisy length/Pitch derivatives: during
             * the measured landing oscillation those signals repeatedly
             * crossed their thresholds and kept operator commands locked for
             * the full timeout.  The independent landing buffer remains
             * active after JUMP releases and still owns the axial handoff. */
            jump_landing_ground_timer = 0.0f;
        }

        const uint8_t timeout_upright_ground =
            !chassis_safety_module_off_ground() &&
            fabsf(ins.pitch) <= JUMP_PRECHECK_PITCH_MAX_DEG &&
            fabsf(ins.roll) <= JUMP_PRECHECK_ROLL_MAX_DEG &&
            fabsf(leg[LEFT]->l0 - leg[RIGHT]->l0) <=
                JUMP_LANDING_TIMEOUT_LENGTH_DIFF_MAX;

        if (jump_landing_ground_timer >=
            JUMP_LANDING_GROUND_CONFIRM_TIME) {
            jump_control_finish(JUMP_FAULT_NONE);
        } else if (elapsed >= JUMP_LANDING_TIMEOUT) {
            /* The landing buffer may deliberately remain active beyond the
             * jump-state timeout while suppressing rebound.  An upright,
             * grounded chassis must return to normal control instead of
             * starting a multi-second Recovery solely because its low-leg
             * derivative has not crossed the strict settle gate. */
            if (timeout_upright_ground) {
                jump_control_finish(JUMP_FAULT_LANDING_TIMEOUT);
            } else {
                jump_control_fail_to_recovery(
                    JUMP_FAULT_LANDING_TIMEOUT);
            }
        }
        break;
    }
    case JUMP_STATE_ABORT_SETTLE:
        /* A late off-ground decision is still possible after the thrust
         * window.  Keep both wheels hard-off long enough for this deliberately
         * small hop to return before handing control back. */
        /* Ground detection can trail the physical takeoff by several filter
         * windows.  Hold the thrust length briefly with zero boost so a late
         * takeoff edge does not see a one-cycle retract command before FLIGHT. */
        if (elapsed >= JUMP_ABORT_HOLD_TIME) {
            jump_length_target = move_towards_f32(
                jump_length_target, LEN_LEN_LOW,
                JUMP_ABORT_RETRACT_RATE * dt);
        }
        jump_force_boost = 0.0f;
        if (chassis_safety_module_off_ground()) {
            jump_fault = JUMP_FAULT_NONE;
            jump_takeoff_seen = 1U;
            jump_state_enter(JUMP_STATE_FLIGHT);
        } else if (elapsed >= JUMP_ABORT_SETTLE_TIME) {
            jump_control_finish(JUMP_FAULT_NO_TAKEOFF);
        }
        break;
    case JUMP_STATE_IDLE:
    default:
        jump_control_finish(JUMP_FAULT_SAFETY);
        break;
    }

    /* Update the estimator flag only after this cycle has evaluated the thrust
     * state and possibly started the pre-tuck profile.  Requiring the profile
     * prevents the zero-initialized thrust flag on entry from looking like a
     * release; doing this before update_ground() makes the handoff same-cycle. */
    const uint8_t jump_release_active =
        jump_state == JUMP_STATE_THRUST &&
        !jump_thrust_active &&
        chassis_safety_module_jump_air_profile_active();
    chassis_safety_module_set_jump_legs_unloaded(jump_release_active);

    if (jump_control_active()) {
        leg[LEFT]->length_ref = jump_length_target;
        leg[RIGHT]->length_ref = jump_length_target;
        chassis_cmd.leg_leng_change = LENGTH_STAY;
    }
}

static void jump_control_observe_ground(void)
{
    if (!jump_control_active()) {
        return;
    }

    const float elapsed = jump_state_elapsed_s();
    const float avg_thrust_length = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    const uint8_t thrust_extended =
        (avg_thrust_length >= (JUMP_THRUST_TARGET_LENGTH - 0.03f));
    const uint8_t takeoff_allowed =
        (jump_state != JUMP_STATE_THRUST) ||
        (elapsed >= (JUMP_THRUST_BOOST_TIME - 0.04f)) ||
        thrust_extended;

    if (chassis_safety_module_takeoff_edge() &&
        !jump_landing_seen &&
        takeoff_allowed &&
        (jump_state == JUMP_STATE_THRUST ||
         jump_state == JUMP_STATE_ABORT_SETTLE)) {
        /* The contact estimator is filtered and can report takeoff a few
         * cycles after the thrust timer.  Once a real edge arrives, do not
         * retain the transient NO_TAKEOFF diagnosis through flight/landing. */
        if (jump_fault == JUMP_FAULT_NO_TAKEOFF) {
            jump_fault = JUMP_FAULT_NONE;
        }
        jump_takeoff_seen = 1U;
        jump_state_enter(JUMP_STATE_FLIGHT);
        jump_length_target = LEN_LEN_LOW;
        jump_force_boost = 0.0f;
    }
    const uint8_t flight_landing_allowed =
        (jump_state != JUMP_STATE_FLIGHT) ||
        (elapsed >= 0.18f);

    if (chassis_safety_module_landing_edge() &&
        jump_takeoff_seen &&
        !jump_landing_seen &&
        flight_landing_allowed &&
        jump_state == JUMP_STATE_FLIGHT) {
        jump_landing_seen = 1U;
        jump_vertical_speed_estimate = 0.0f;
        jump_state_enter(JUMP_STATE_LANDING);
        jump_length_target = LEN_LEN_LOW;
        jump_force_boost = 0.0f;
    }
    /* 2026-10-07：**不要覆盖 BUMP 的 wheel_shut 决定**。
     * 本函数每拍运行(chassis_control.c:5470)，原来无条件把 wheel_shut 写成
     * off_ground —— 地面时写 0，于是 bump 在 bump_control_prepare_cycle() 里
     * 刚设的 1 被立刻冲掉。bump11 实测 I5 在爬台阶/ABORT 期间反复 1->0->1->0，
     * 操作手看到的现象就是「碰撞后轮子没关掉」。
     * 现在改成「离地 或 bump 要求关断」的或运算，两边诉求都不丢。 */
    chassis_safety_module_set_wheel_shut(
        chassis_safety_module_off_ground() ||
        (bump_control_active() && bump_wheel_shut_requested()));
}

/*
 * @brief Validated simulation airborne reaction-wheel allocation.
 *
 * The normal LQR wheel row is deliberately zeroed while airborne.  The
 * simulation jump, however, uses the two wheel rotors as an internal pitch
 * reaction axis while the hip common axis points both rods toward the landing
 * surface.  Keep this as a separate, bounded channel so it cannot become a
 * ground-drive command or leak into the landing buffer.
 */
static void jump_air_wheel_control_update(void)
{
    jump_air_wheel_torque = 0.0f;

#if JUMP_AIR_REACTION_WHEEL_ENABLE
    /* 2026-09-23 (jump23.csv): compute and LATCH the wheel channel's own inputs
     * BEFORE any early return.  The channel returns early whenever it is not
     * "air active", and jump23 showed exactly that happening through the
     * grounded pitch collapse -- the wheel read ZERO while Pitch fell from
     * +2.4 to -6.6 deg, and then pinned at +-8191 only after takeoff.  With the
     * inputs latched unconditionally the log answers whether the zero was a
     * reference/gating problem or a genuine zero demand. */
    const float pitch = ins.pitch * DEGREE_2_RAD;
    const float pitch_rate = jump_body_pitch_rate_feedback();
    const uint8_t compact_dbg =
        chassis_safety_module_jump_air_compact_phase();
    float pitch_reference_dbg =
        JUMP_AIR_TUCK_PITCH_BIAS_RAD * (compact_dbg ? 1.0f : 0.0f) +
        JUMP_AIR_DEPLOY_PITCH_BIAS_RAD * (compact_dbg ? 0.0f : 1.0f) +
        JUMP_AIR_MOVING_PITCH_GAIN_RAD * jump_launch_vx;
    LIMIT_MIN_MAX(pitch_reference_dbg,
                  -15.0f * DEGREE_2_RAD, 15.0f * DEGREE_2_RAD);
    jump_air_wheel_pitch_ref_rad = pitch_reference_dbg;
    jump_air_wheel_pitch_err_rad = pitch - pitch_reference_dbg;
    jump_air_wheel_pitch_rate = pitch_rate;

    const uint8_t reaction_admitted = jump_air_wheel_admitted();
    if (!reaction_admitted) {
        return;
    }

    const float air_elapsed = chassis_safety_module_airborne_elapsed();
    const uint8_t compact =
        chassis_safety_module_jump_air_compact_phase();
    const float theta_weight = compact ? 0.8f : 1.5f;
    float pitch_reference =
        JUMP_AIR_TUCK_PITCH_BIAS_RAD * (compact ? 1.0f : 0.0f) +
        JUMP_AIR_DEPLOY_PITCH_BIAS_RAD * (compact ? 0.0f : 1.0f) +
        JUMP_AIR_MOVING_PITCH_GAIN_RAD * jump_launch_vx;
    LIMIT_MIN_MAX(pitch_reference,
                  -15.0f * DEGREE_2_RAD, 15.0f * DEGREE_2_RAD);
    const float theta_common = 0.5f *
        (leg[LEFT]->theta + leg[RIGHT]->theta);
    const float theta_rate_common = 0.5f *
        (leg[LEFT]->d_theta_lpf + leg[RIGHT]->d_theta_lpf);
    float theta_hold =
        30.0f * theta_common + 4.0f * theta_rate_common;
    LIMIT_MIN_MAX(theta_hold, -22.0f, 22.0f);

    float wheel_feedback =
        JUMP_AIR_WHEEL_PITCH_KP * (pitch - pitch_reference) +
        JUMP_AIR_WHEEL_PITCH_KD * pitch_rate +
        JUMP_AIR_WHEEL_THETA_FF * theta_weight * theta_hold;
    LIMIT_MIN_MAX(wheel_feedback,
                  -JUMP_AIR_WHEEL_TORQUE_LIMIT,
                   JUMP_AIR_WHEEL_TORQUE_LIMIT);
    if (!isfinite(wheel_feedback)) {
        return;
    }

    /* Keep bounded correction available until the robot settles or flight
     * authority ends. The late cap limits sustained wheel current; measured
     * pitch error/rate smoothly fade the command in the stable deadband. */
    const float air_authority =
        jump_reaction_wheel_air_authority_scale(
            reaction_admitted,
            chassis_safety_module_off_ground(),
            chassis_safety_module_jump_landing_owner_active(),
            chassis_fdb_data.stand_state == CHASSIS_IS_DANGER,
            air_elapsed, pitch - pitch_reference, pitch_rate);
    jump_air_wheel_torque = wheel_feedback * air_authority;
#endif
}

/*
 * @brief Landing-buffer body-Pitch support and wheel return handoff.
 *
 * Before confirmed contact use the bounded body-Pitch damper. Confirmed ground
 * blends toward raw ground LQR with the same cached beta as the jump-origin
 * hip owner, independently of the axial COMPRESS/HOLD/RETURN phases. After the
 * buffer releases, the existing handoff fade starts from the last actual owner
 * output. The air-wheel schedule may not finish before early contact and does
 * not remove stored rotor momentum.
 */
static void jump_landing_wheel_control_update(void)
{
    const uint8_t jump_mode =
        chassis_cmd.ctrl_mode == CHASSIS_JUMP;
    const uint8_t owner_active =
        chassis_safety_module_jump_landing_owner_active();
    const uint8_t buffer_active = owner_active &&
        chassis_safety_module_landing_buffer_active();
    const uint8_t off_ground = chassis_safety_module_off_ground();

    jump_landing_wheel_fade_started_this_cycle = 0U;

    if (off_ground && jump_air_wheel_output_admitted()) {
        /* Keep the landing handoff paused through an admitted rebound.  The
         * final motor callback owns airborne torque; this function only keeps
         * the grounded fade clock from expiring without contact. */
        jump_landing_wheel_torque = 0.0f;
        return;
    }

    if (buffer_active && off_ground) {
        /* Preserve ownership over an unadmitted airborne interval. */
        jump_landing_wheel_torque = 0.0f;
        return;
    }

    if (buffer_active) {
        jump_landing_wheel_fade.active = 0U;
        jump_landing_wheel_fade_duration_s =
            JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
        jump_landing_buffer_owner_previous = 1U;
        jump_landing_return_ramp_active = 1U;
        jump_landing_lqr_return_gain = 0.0f;
#if JUMP_LANDING_WHEEL_ENABLE
        const float pitch = ins.pitch * DEGREE_2_RAD;
        const float pitch_rate = jump_body_pitch_rate_feedback();
        const float pitch_reference =
            JUMP_AIR_MOVING_PITCH_GAIN_RAD * jump_launch_vx;
        float damper_target =
            JUMP_LANDING_WHEEL_PITCH_KP * (pitch - pitch_reference) +
            JUMP_LANDING_WHEEL_PITCH_KD * pitch_rate;
        LIMIT_MIN_MAX(damper_target,
                      -JUMP_LANDING_WHEEL_TORQUE_LIMIT,
                       JUMP_LANDING_WHEEL_TORQUE_LIMIT);
        const float max_step =
            JUMP_LANDING_WHEEL_SLEW_RATE * jump_cycle_dt_s;
        jump_landing_wheel_torque = move_towards_f32(
            jump_landing_wheel_torque, damper_target, max_step);
#endif
        return;
    }

    if (jump_landing_buffer_owner_previous) {
        float start_left;
        float start_right;
        start_left = jump_landing_wheel_last_owner_command_valid[LEFT] ?
            jump_landing_wheel_last_owner_command[LEFT] :
            -jump_landing_wheel_torque * M3508_TOR_TO_CUR;
        start_right = jump_landing_wheel_last_owner_command_valid[RIGHT] ?
            jump_landing_wheel_last_owner_command[RIGHT] :
            jump_landing_wheel_torque * M3508_TOR_TO_CUR;
        jump_landing_wheel_fade_begin(&jump_landing_wheel_fade,
                                      start_left, start_right);
        jump_landing_wheel_fade_duration_s =
            jump_landing_wheel_handoff_duration(
                chassis_safety_module_landing_fast_exit_active());
        jump_landing_wheel_fade_started_this_cycle = 1U;
        jump_landing_buffer_owner_previous = 0U;
    }

    if (jump_landing_wheel_fade.active) {
        jump_landing_return_ramp_active = 1U;
        if (!jump_landing_wheel_fade_started_this_cycle) {
            jump_landing_wheel_fade_advance(
                &jump_landing_wheel_fade, jump_cycle_dt_s,
                jump_landing_wheel_fade_duration_s);
        }
        /* The output helper owns the linear blend.  Keep the legacy JUMP
         * source raw so it cannot apply the same gain a second time. */
        jump_landing_lqr_return_gain = 1.0f;
        jump_landing_wheel_torque = 0.0f;
        return;
    }

    if (!jump_mode) {
        jump_landing_return_ramp_active = 0U;
        jump_landing_lqr_return_gain = 1.0f;
        jump_landing_wheel_torque = 0.0f;
        return;
    }

    /* While the shared gate admits the reaction channel, prepare the return
     * ramp for touchdown. Landing-buffer ownership takes priority in the gate. */
    if (jump_air_wheel_admitted()) {
        jump_landing_wheel_torque = 0.0f;
        jump_landing_return_ramp_active = 1U;
        jump_landing_lqr_return_gain = 0.0f;
        return;
    }

    /* Buffer released: fade the standing LQR wheel row back in so the hot LQR
     * row does not step from the damper's near-zero command at handoff. */
    jump_landing_wheel_torque = 0.0f;
    if (jump_landing_return_ramp_active) {
        jump_landing_lqr_return_gain +=
            jump_cycle_dt_s / JUMP_LANDING_WHEEL_RELEASE_TIME;
        if (jump_landing_lqr_return_gain >= 1.0f) {
            jump_landing_lqr_return_gain = 1.0f;
            jump_landing_return_ramp_active = 0U;
        }
    }
}

static void leg_stuck_compensation_update(float left_target,
                                          float right_target)
{
    const float target[CHASSIS_SIDE_COUNT] = {left_target, right_target};
    const float now_ms = dwt_get_time_ms();
    float dt = (now_ms - leg_stuck_last_ms) * 0.001f;

    leg_stuck_last_ms = now_ms;
    if (dt <= 0.0f || dt > 0.020f) {
        dt = 0.001f;
    }

    if (leg_stuck_post_ramp_timer > 0.0f) {
        leg_stuck_post_ramp_timer -= dt;
        if (leg_stuck_post_ramp_timer < 0.0f) {
            leg_stuck_post_ramp_timer = 0.0f;
        }
    }

    const uint8_t reference_ramping =
        chassis_cmd.leg_leng_change == LENGTH_CHANGE;
    /* 参考斜坡完成后只保留 1 s 的有限到位窗口，避免落后腿永久卡住，
     * 同时禁止恢复过去无限重复的破静摩擦 burst。 */
    const uint8_t enabled = normal_leg_transition_mode() &&
                            (reference_ramping ||
                             leg_stuck_post_ramp_timer > 0.0f) &&
                            !chassis_safety_module_off_ground();
    const float force_max = reference_ramping ?
                            LEG_STUCK_FORCE_MAX :
                            LEG_STUCK_POST_RAMP_FORCE_MAX;

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        const float error = target[side] - leg[side]->l0;
        const float stuck_speed = leg_stuck_speed_measurement(side);
        const int8_t error_sign = (error > 0.0f) ? 1 :
                                  ((error < 0.0f) ? -1 : 0);
        const uint8_t stuck = enabled &&
                              fabsf(error) > LEG_STUCK_ERROR_THRESHOLD &&
                              fabsf(stuck_speed) < LEG_STUCK_SPEED_THRESHOLD;
        float force_target = 0.0f;

        if (!stuck || error_sign != leg_stuck_error_sign[side]) {
            leg_stuck_timer[side] = 0.0f;
            leg_stuck_error_sign[side] = error_sign;
            leg_stuck_burst_active[side] = 0U;
        } else if (!leg_stuck_burst_active[side] &&
                   !leg_stuck_burst_used[side]) {
            leg_stuck_timer[side] += dt;
        }

        if (stuck && !leg_stuck_burst_used[side] &&
            leg_stuck_timer[side] >= LEG_STUCK_DETECT_TIME) {
            leg_stuck_burst_active[side] = 1U;
            leg_stuck_burst_used[side] = 1U;
        }

        if (leg_stuck_burst_active[side]) {
            force_target = (float)error_sign * force_max;
        }

        const float rate = (force_target == 0.0f) ?
                           LEG_STUCK_FORCE_DECAY_RATE :
                           LEG_STUCK_FORCE_RAMP_RATE;
        leg_stuck_force[side] = move_towards_f32(
            leg_stuck_force[side], force_target, rate * dt);
    }
}

static uint8_t high_leg_geometry_active(void)
{
    const float average_length = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    return chassis_cmd.leg_level == LEG_HIG || average_length >= 0.245f;
}

static uint8_t mid_leg_geometry_active(void)
{
    const float average_length = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    return chassis_cmd.leg_level == LEG_MID || average_length >= 0.170f;
}

static float chassis_vx_limit_calculate(void)
{
    if (high_leg_geometry_active()) {
        return CHASSIS_V_MAX_HIGH;
    }
    if (mid_leg_geometry_active()) {
        return CHASSIS_V_MAX_MID;
    }
    return CHASSIS_V_MAX_LOW;
}

static float chassis_yaw_rate_limit_calculate(void)
{
    if (high_leg_geometry_active()) {
        return YAW_RATE_MAX_HIGH_RADPS;
    }
    if (mid_leg_geometry_active()) {
        return YAW_RATE_MAX_MID_RADPS;
    }
    return YAW_RATE_MAX_LOW_RADPS;
}

static float chassis_follow_yaw_rate_limit_calculate(void)
{
    if (high_leg_geometry_active()) {
        return YAW_FOLLOW_RATE_MAX_HIGH;
    }
    if (mid_leg_geometry_active()) {
        return YAW_FOLLOW_RATE_MAX_MID;
    }
    return YAW_FOLLOW_RATE_MAX_LOW;
}

static float chassis_spin_yaw_rate_for_level(leg_level_e level)
{
    if (level == LEG_HIG) {
        return YAW_SPIN_RATE_MAX_HIGH_RADPS;
    }
    if (level == LEG_MID) {
        return YAW_SPIN_RATE_MAX_MID_RADPS;
    }
    return YAW_SPIN_RATE_MAX_LOW_RADPS;
}

static void chassis_spin_profile_update(uint8_t spin_mode, float dt)
{
    if (!spin_mode) {
        spin_entry_leg_level = LEG_LOW;
        spin_entry_yaw_rate_limit = YAW_SPIN_RATE_MAX_LOW_RADPS;
        spin_entry_elapsed_s = 0.0f;
        spin_entry_latched = 0U;
        return;
    }

    if (!spin_entry_latched) {
        spin_entry_leg_level = chassis_cmd.leg_level <= LEG_HIG ?
            chassis_cmd.leg_level : LEG_LOW;
        spin_entry_yaw_rate_limit = chassis_spin_yaw_rate_for_level(
            spin_entry_leg_level);
        spin_entry_elapsed_s = 0.0f;
        spin_entry_latched = 1U;
    } else {
        spin_entry_elapsed_s += dt;
    }
}

static float chassis_spin_yaw_rate_limit_calculate(void)
{
    float dt = chassis_dt;
    float target_limit;
    const float average_length =
        0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    const float reference_length =
        0.5f * (leg[LEFT]->length_ref + leg[RIGHT]->length_ref);
    const float average_length_rate =
        0.5f * (leg[LEFT]->d_l0_lpf + leg[RIGHT]->d_l0_lpf);
    if (!(dt > 0.0002f) || dt > 0.02f) {
        dt = 0.001f;
    }
    /* The entry profile owns the Spin speed for the whole session.  Do not
     * derive this from current geometry: changing the left switch during Spin
     * must only move the legs, never alter the operator's angular speed. */
    if (!spin_entry_latched) {
        chassis_spin_profile_update(1U, dt);
    }
    target_limit = spin_entry_yaw_rate_limit;

    /* Keep transition state for diagnostics and roll integral hold, while
     * leaving the locked yaw-rate limit untouched. */
    if (chassis_cmd.leg_leng_change == LENGTH_CHANGE) {
        spin_leg_transition_active = 1U;
        spin_leg_transition_settle_timer = 0.0f;
    }

    if (spin_leg_transition_active) {
        const uint8_t physical_settled =
            chassis_cmd.leg_leng_change == LENGTH_STAY &&
            fabsf(reference_length - average_length) <= LEN_JUDGE_REGION &&
            fabsf(average_length_rate) <= LQR_K_TRANSITION_L0_RATE_MAX;
        if (physical_settled) {
            spin_leg_transition_settle_timer += dt;
            if (spin_leg_transition_settle_timer >=
                LQR_K_TRANSITION_SETTLE_TIME) {
                spin_leg_transition_active = 0U;
            }
        } else {
            spin_leg_transition_settle_timer = 0.0f;
        }

    }

    if (!spin_leg_transition_active) {
        spin_leg_transition_settle_timer = 0.0f;
    }
    spin_leg_transition_rate_limit = target_limit;
    return target_limit;
}

static float lateral_inertial_ff_gain_calculate(void)
{
    if (high_leg_geometry_active()) {
        return LATERAL_INERTIAL_FF_GAIN_HIGH;
    }
    if (mid_leg_geometry_active()) {
        return LATERAL_INERTIAL_FF_GAIN_MID;
    }
    return LATERAL_INERTIAL_FF_GAIN_LOW;
}

static float leg_split_limit_calculate(void)
{
    if (high_leg_geometry_active()) {
        return LEG_SPLIT_LIMIT_HIGH;
    }
    if (mid_leg_geometry_active()) {
        return LEG_SPLIT_LIMIT_MID;
    }
    return LEG_SPLIT_LIMIT_LOW;
}

static void roll_static_trim_update(void)
{
    float dt = chassis_dt;
    float applied_target;
    const uint8_t spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    const float trim_ki = spin_mode ?
        ROLL_STATIC_TRIM_KI : ROLL_STATIC_TRIM_NORMAL_KI;
    const float trim_limit = spin_mode ?
        ROLL_STATIC_TRIM_LIMIT : ROLL_STATIC_TRIM_NORMAL_LIMIT;
    const float trim_apply_limit = spin_mode ?
        ROLL_STATIC_TRIM_APPLY_LIMIT :
        ROLL_STATIC_TRIM_NORMAL_APPLY_LIMIT;
    const float trim_settle_time = spin_mode ?
        ROLL_STATIC_TRIM_SETTLE_TIME :
        ROLL_STATIC_TRIM_NORMAL_SETTLE_TIME;
    const float trim_vx_fdb_db = spin_mode ?
        ROLL_STATIC_TRIM_VX_FDB_DB :
        ROLL_STATIC_TRIM_NORMAL_VX_FDB_DB;
    const float trim_yaw_rate_db = spin_mode ?
        ROLL_STATIC_TRIM_YAW_RATE_DB :
        ROLL_STATIC_TRIM_NORMAL_YAW_RATE_DB;
    const float trim_roll_rate_db = spin_mode ?
        ROLL_STATIC_TRIM_ROLL_RATE_DB :
        ROLL_STATIC_TRIM_NORMAL_ROLL_RATE_DB;
    if (dt < 0.0002f || dt > 0.010f) {
        dt = 0.001f;
    }

    if (!normal_leg_transition_mode() ||
        chassis_safety_module_off_ground() ||
        chassis_safety_module_landing_buffer_active()) {
        roll_static_trim_force = 0.0f;
        roll_static_trim_applied = 0.0f;
        roll_static_trim_timer = 0.0f;
        return;
    }

    const uint8_t settled =
        chassis_cmd.leg_leng_change == LENGTH_STAY &&
        fabsf(chassis_cmd.vx_set) <= ROLL_STATIC_TRIM_VX_CMD_DB &&
        fabsf(chassis_vx_filter) <= trim_vx_fdb_db &&
        fabsf(chassis_cmd.vw_relative_set) <= YAW_RC_DEADBAND &&
        fabsf(dbg_yaw_rate_fdb) <= trim_yaw_rate_db &&
        fabsf(ins.gyro[1]) <= trim_roll_rate_db;

    if (!settled) {
        roll_static_trim_timer = 0.0f;
    } else {
        roll_static_trim_timer += dt;
        if (roll_static_trim_timer >= trim_settle_time &&
            fabsf(ins.roll) > ROLL_STATIC_TRIM_DEADBAND_DEG) {
            roll_static_trim_force -= trim_ki * ins.roll * dt;
            LIMIT_MIN_MAX(roll_static_trim_force,
                          -trim_limit,
                           trim_limit);
        }
    }

    /* Preserve the learned correction for diagnosis, but always reserve
     * differential force for the roll PD.  Slewing the applied copy prevents
     * a support-force step when the learned value changes. */
    applied_target = roll_static_trim_force;
    LIMIT_MIN_MAX(applied_target,
                  -trim_apply_limit,
                   trim_apply_limit);
    roll_static_trim_applied = move_towards_f32(
        roll_static_trim_applied,
        applied_target,
        ROLL_STATIC_TRIM_APPLY_SLEW_RATE * dt);
}

static float roll_spin_schedule(float l0_average,
                                float low_value,
                                float mid_value,
                                float high_value)
{
    float progress;
    if (l0_average <= ROLL_SPIN_SCHEDULE_L0_MID) {
        progress = (l0_average - ROLL_SPIN_SCHEDULE_L0_LOW) /
            (ROLL_SPIN_SCHEDULE_L0_MID - ROLL_SPIN_SCHEDULE_L0_LOW);
        LIMIT_MIN_MAX(progress, 0.0f, 1.0f);
        return low_value + progress * (mid_value - low_value);
    }

    progress = (l0_average - ROLL_SPIN_SCHEDULE_L0_MID) /
        (ROLL_SPIN_SCHEDULE_L0_HIGH - ROLL_SPIN_SCHEDULE_L0_MID);
    LIMIT_MIN_MAX(progress, 0.0f, 1.0f);
    return mid_value + progress * (high_value - mid_value);
}

static float roll_spin_compensation_update(float base_target,
                                           float l0_average)
{
    float dt = chassis_dt;
    float rate_ratio = 0.0f;
    float target_offset = 0.0f;
    if (!(dt > 0.0002f) || dt > 0.010f) {
        dt = 0.001f;
    }

    const uint8_t spin_available =
        chassis_cmd.ctrl_mode == CHASSIS_SPIN &&
        !chassis_safety_module_off_ground() &&
        !chassis_safety_module_landing_buffer_active() &&
        !chassis_safety_module_wheel_shut();

    if (spin_available && dbg_yaw_rate_limit > 0.10f) {
        rate_ratio = dbg_yaw_rate_ref / dbg_yaw_rate_limit;
        LIMIT_MIN_MAX(rate_ratio, -1.0f, 1.0f);
        const float target_amplitude = roll_spin_schedule(
            l0_average,
            ROLL_SPIN_TARGET_LOW_DEG,
            ROLL_SPIN_TARGET_MID_DEG,
            ROLL_SPIN_TARGET_HIGH_DEG);
        target_offset = target_amplitude * rate_ratio * fabsf(rate_ratio);
    }

    const float target_rate =
        fabsf(target_offset) > fabsf(roll_spin_target_applied) ?
        ROLL_SPIN_TARGET_ENGAGE_RATE :
        ROLL_SPIN_TARGET_RELEASE_RATE;
    roll_spin_target_applied = move_towards_f32(
        roll_spin_target_applied, target_offset, target_rate * dt);

    const uint8_t integral_active =
        spin_available &&
        !spin_leg_transition_active &&
        roll_spin_integral_settle_timer >=
            ROLL_SPIN_INTEGRAL_SETTLE_TIME &&
        fabsf(rate_ratio) >= ROLL_SPIN_INTEGRAL_RATE_RATIO &&
        fabsf(ins.gyro[1]) <= ROLL_SPIN_INTEGRAL_ROLL_RATE_MAX;

    if (!spin_available ||
        spin_leg_transition_active ||
        fabsf(ins.gyro[1]) > ROLL_SPIN_INTEGRAL_ROLL_RATE_MAX) {
        roll_spin_integral_settle_timer = 0.0f;
    } else if (roll_spin_integral_settle_timer <
               ROLL_SPIN_INTEGRAL_SETTLE_TIME) {
        roll_spin_integral_settle_timer += dt;
    }

    uint8_t profile_index = (uint8_t)chassis_cmd.leg_level;
    if (profile_index > (uint8_t)LEG_HIG) {
        profile_index = (uint8_t)LEG_LOW;
    }

    if (integral_active) {
        float roll_for_integral = ins.roll;
        if (fabsf(roll_for_integral) <=
            ROLL_SPIN_INTEGRAL_DEADBAND_DEG) {
            roll_for_integral = 0.0f;
        }
        const float integral_ki = roll_spin_schedule(
            l0_average,
            ROLL_SPIN_INTEGRAL_KI_LOW,
            ROLL_SPIN_INTEGRAL_KI_MID,
            ROLL_SPIN_INTEGRAL_KI_HIGH);
        float integral_limit = ROLL_SPIN_INTEGRAL_LIMIT_LOW;
        if (profile_index == (uint8_t)LEG_MID) {
            integral_limit = ROLL_SPIN_INTEGRAL_LIMIT_MID;
        } else if (profile_index == (uint8_t)LEG_HIG) {
            integral_limit = ROLL_SPIN_INTEGRAL_LIMIT_HIGH;
        }
        roll_spin_integral_force[profile_index] -=
            integral_ki * roll_for_integral * dt;
        LIMIT_MIN_MAX(roll_spin_integral_force[profile_index],
                      -integral_limit, integral_limit);
    }

    float integral_target = 0.0f;
    if (spin_available) {
        /* Do not blend two learned leg-level profiles while the mechanism is
         * moving.  Hold the already-applied force, then hand over to the
         * destination profile only after the leg reference has settled. */
        integral_target = spin_leg_transition_active ?
            roll_spin_integral_applied :
            roll_spin_schedule(l0_average,
                               roll_spin_integral_force[LEG_LOW],
                               roll_spin_integral_force[LEG_MID],
                               roll_spin_integral_force[LEG_HIG]);
    }
    const float integral_rate = spin_available ?
        ROLL_SPIN_INTEGRAL_APPLY_RATE :
        ROLL_SPIN_INTEGRAL_RELEASE_RATE;
    roll_spin_integral_applied = move_towards_f32(
        roll_spin_integral_applied, integral_target, integral_rate * dt);

    return base_target + roll_spin_target_applied;
}

static void roll_geometry_comp_update(void)
{
    float dt = chassis_dt;
    if (!(dt > 0.0002f) || dt > 0.010f) {
        dt = 0.001f;
    }

#if !ROLL_GEOMETRY_COMP_ENABLE
    roll_length_offset_target = 0.0f;
    roll_length_offset = 0.0f;
    roll_geometry_settle_timer = 0.0f;
    dbg_roll_slope_tan = 0.0f;
    return;
#else
    const uint8_t reset =
        !normal_leg_transition_mode() ||
        chassis_safety_module_off_ground() ||
        chassis_safety_module_landing_buffer_active() ||
        chassis_safety_module_wheel_shut() ||
        chassis_cmd.leg_leng_change == LENGTH_CHANGE;
    if (reset) {
        roll_length_offset_target = 0.0f;
        roll_length_offset = 0.0f;
        roll_geometry_settle_timer = 0.0f;
        dbg_roll_slope_tan = 0.0f;
        return;
    }

    const uint8_t settled =
        fabsf(LQRXRefBuf[LEFT][3]) <= ROLL_GEOMETRY_VX_REF_DB &&
        fabsf(chassis_vx_filter) <= ROLL_GEOMETRY_VX_FDB_DB &&
        fabsf(dbg_yaw_rate_ref) <= ROLL_GEOMETRY_YAW_RATE_DB &&
        fabsf(dbg_yaw_rate_fdb) <= ROLL_GEOMETRY_YAW_RATE_DB &&
        fabsf(ins.gyro[1]) <= ROLL_GEOMETRY_ROLL_RATE_DB;
    if (!settled) {
        /* Hold the last terrain estimate during motion.  This is the key
         * separation from dynamic centrifugal compensation below. */
        roll_geometry_settle_timer = 0.0f;
        return;
    }

    roll_geometry_settle_timer += dt;
    if (roll_geometry_settle_timer >= ROLL_GEOMETRY_SETTLE_TIME) {
        const float roll_rad = ins.roll * DEGREE_2_RAD;
        const float sin_roll = arm_sin_f32(roll_rad);
        const float cos_roll = arm_cos_f32(roll_rad);
        const float right_minus_left = leg[RIGHT]->l0 - leg[LEFT]->l0;
        const float numerator = right_minus_left * cos_roll +
                                2.0f * Rl * sin_roll;
        const float denominator = -right_minus_left * sin_roll +
                                  2.0f * Rl * cos_roll;
        if (fabsf(denominator) >= ROLL_GEOMETRY_DENOM_MIN) {
            dbg_roll_slope_tan = numerator / denominator;
            LIMIT_MIN_MAX(dbg_roll_slope_tan,
                          -ROLL_GEOMETRY_SLOPE_TAN_LIMIT,
                           ROLL_GEOMETRY_SLOPE_TAN_LIMIT);
            /* Half of DeltaL0d is applied in opposite directions. */
            roll_length_offset_target =
                ROLL_GEOMETRY_COMP_GAIN * Rl * dbg_roll_slope_tan;
            LIMIT_MIN_MAX(roll_length_offset_target,
                          -ROLL_GEOMETRY_OFFSET_LIMIT,
                           ROLL_GEOMETRY_OFFSET_LIMIT);
        }
    }

    roll_length_offset = move_towards_f32(
        roll_length_offset, roll_length_offset_target,
        ROLL_GEOMETRY_OFFSET_SLEW_RATE * dt);
#endif
}

static void lqr_dphi_notch_reset(float value)
{
    lqr_dphi_notch.x1 = value;
    lqr_dphi_notch.x2 = value;
    lqr_dphi_notch.y1 = value;
    lqr_dphi_notch.y2 = value;
    lqr_dphi_notch.output = value;
    lqr_dphi_notch.initialized = 0U;
    dbg_lqr_dphi_raw = value;
    dbg_lqr_dphi_filtered = value;
}

static float lqr_dphi_notch_update(float input, float dt, uint8_t enable)
{
    dbg_lqr_dphi_raw = input;

    if (isnan(input) || isinf(input)) {
        lqr_dphi_notch_reset(0.0f);
        return 0.0f;
    }
    if (!enable) {
        lqr_dphi_notch_reset(input);
        return input;
    }

    if (!lqr_dphi_notch.initialized) {
        lqr_dphi_notch.x1 = input;
        lqr_dphi_notch.x2 = input;
        lqr_dphi_notch.y1 = input;
        lqr_dphi_notch.y2 = input;
        lqr_dphi_notch.output = input;
        lqr_dphi_notch.initialized = 1U;
        dbg_lqr_dphi_filtered = input;
        return input;
    }

    /* update_LQR_obs may run twice on a takeoff/landing edge. Do not advance
     * the biquad twice with a nearly-zero timestamp interval. */
    if (dt < 0.0002f) {
        return lqr_dphi_notch.output;
    }
    if (dt > 0.010f) {
        dt = chassis_dt;
        if (dt < 0.0002f || dt > 0.010f) {
            dt = 0.001f;
        }
    }

    const float k = tanf(LQR_DPHI_NOTCH_PI *
                         LQR_DPHI_NOTCH_F0_HZ * dt);
    const float k2 = k * k;
    const float norm = 1.0f / (1.0f + k / LQR_DPHI_NOTCH_Q + k2);
    const float b0 = (1.0f + k2) * norm;
    const float b1 = 2.0f * (k2 - 1.0f) * norm;
    const float b2 = b0;
    const float a1 = b1;
    const float a2 = (1.0f - k / LQR_DPHI_NOTCH_Q + k2) * norm;
    const float notch_output = b0 * input +
                               b1 * lqr_dphi_notch.x1 +
                               b2 * lqr_dphi_notch.x2 -
                               a1 * lqr_dphi_notch.y1 -
                               a2 * lqr_dphi_notch.y2;

    lqr_dphi_notch.x2 = lqr_dphi_notch.x1;
    lqr_dphi_notch.x1 = input;
    lqr_dphi_notch.y2 = lqr_dphi_notch.y1;
    lqr_dphi_notch.y1 = notch_output;
    lqr_dphi_notch.output = LQR_DPHI_NOTCH_RAW_BLEND * input +
        (1.0f - LQR_DPHI_NOTCH_RAW_BLEND) * notch_output;

    if (isnan(lqr_dphi_notch.output) || isinf(lqr_dphi_notch.output)) {
        lqr_dphi_notch_reset(input);
        return input;
    }

    dbg_lqr_dphi_filtered = lqr_dphi_notch.output;
    return lqr_dphi_notch.output;
}

static void lqr_k_transition_update(const float target[2][6],
                                    leg_level_e target_level,
                                    float dt,
                                    uint8_t transition_enable)
{
    if (!transition_enable) {
        memcpy(lqr_k_transition_current, target,
               sizeof(lqr_k_transition_current));
        lqr_k_transition_level = target_level;
        lqr_k_transition_progress = 1.0f;
        lqr_k_transition_initialized = 0U;
        lqr_k_transition_settle_timer = 0.0f;
        lqr_k_transition_tail_released = 1U;
        dbg_lqr_k_tail_waiting = 0U;
        return;
    }

    if (!lqr_k_transition_initialized) {
        memcpy(lqr_k_transition_current, target,
               sizeof(lqr_k_transition_current));
        lqr_k_transition_level = target_level;
        lqr_k_transition_progress = 1.0f;
        lqr_k_transition_initialized = 1U;
        lqr_k_transition_settle_timer = 0.0f;
        lqr_k_transition_tail_released = 1U;
        dbg_lqr_k_tail_waiting = 0U;
        return;
    }

    if (target_level != lqr_k_transition_level) {
        memcpy(lqr_k_transition_from, lqr_k_transition_current,
               sizeof(lqr_k_transition_from));
        lqr_k_transition_level = target_level;
        lqr_k_transition_progress = 0.0f;
        lqr_k_transition_settle_timer = 0.0f;
        lqr_k_transition_tail_released = 0U;
        dbg_lqr_k_tail_waiting = 0U;
    }

    if (lqr_k_transition_progress < 1.0f) {
        float safe_dt = dt;
        if (!(safe_dt > 0.0f) || safe_dt > 0.02f) {
            safe_dt = chassis_dt;
        }
        if (!lqr_k_transition_tail_released) {
            const float l0_ref_average = 0.5f *
                (leg[LEFT]->length_ref + leg[RIGHT]->length_ref);
            const float l0_average = 0.5f *
                (leg[LEFT]->l0 + leg[RIGHT]->l0);
            const float l0_rate_average = 0.5f *
                (kin_rate_observer[LEFT].l0_rate +
                 kin_rate_observer[RIGHT].l0_rate);
            const uint8_t observer_ready =
                kin_rate_observer[LEFT].sample_count >= KIN_RATE_WINDOW_SIZE &&
                kin_rate_observer[RIGHT].sample_count >= KIN_RATE_WINDOW_SIZE;
            const uint8_t physical_settled = observer_ready &&
                chassis_cmd.leg_leng_change == LENGTH_STAY &&
                fabsf(l0_ref_average - l0_average) <=
                    LQR_K_TRANSITION_L0_ERROR_MAX &&
                fabsf(l0_rate_average) <= LQR_K_TRANSITION_L0_RATE_MAX;

            if (physical_settled) {
                lqr_k_transition_settle_timer += safe_dt;
                if (lqr_k_transition_settle_timer >=
                    LQR_K_TRANSITION_SETTLE_TIME) {
                    lqr_k_transition_settle_timer =
                        LQR_K_TRANSITION_SETTLE_TIME;
                    lqr_k_transition_tail_released = 1U;
                }
            } else {
                lqr_k_transition_settle_timer = 0.0f;
            }
        }

        float next_progress = lqr_k_transition_progress +
                              safe_dt / LQR_K_TRANSITION_TIME;
        if (!lqr_k_transition_tail_released &&
            next_progress > LQR_K_TRANSITION_TAIL_START) {
            next_progress = LQR_K_TRANSITION_TAIL_START;
            dbg_lqr_k_tail_waiting = 1U;
        } else {
            dbg_lqr_k_tail_waiting = 0U;
        }
        lqr_k_transition_progress = next_progress;
        if (lqr_k_transition_progress > 1.0f) {
            lqr_k_transition_progress = 1.0f;
        }

        /* Smoothstep 让 K 的起止斜率都为零，避免过渡端点再次产生冲击。 */
        float blend = lqr_k_transition_progress * lqr_k_transition_progress *
                      (3.0f - 2.0f * lqr_k_transition_progress);
        for (uint8_t row = 0U; row < 2U; row++) {
            for (uint8_t col = 0U; col < 6U; col++) {
                lqr_k_transition_current[row][col] =
                    lqr_k_transition_from[row][col] + blend *
                    (target[row][col] - lqr_k_transition_from[row][col]);
            }
        }
    } else {
        memcpy(lqr_k_transition_current, target,
               sizeof(lqr_k_transition_current));
        dbg_lqr_k_tail_waiting = 0U;
    }
}





static void update_LQR_obs() {

    static float dt, start;
    if(start != 0) // 避免第一次计算错误
        dt = (dwt_get_time_ms() - start) / 1000.0f;
    else
        dt = 0.003f;
    start = dwt_get_time_ms();

    /* RECOVERY+handoff 后强制按低腿长参数 */
    leg_level_e lqr_leg_level = chassis_cmd.leg_level;
    if (recovery_after_handoff()) {
        lqr_leg_level = LEG_LOW;
    }

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
       !recovery_after_handoff()){/*起立时的lqr*/
            /* recovery: Recovery 起立阶段 */
            /* Q = diag([ 100 10 1 1 1 1 ]), R = diag([ 2.75 1.25 ]) */
            /* max_fit_error=0.0169212442, max_fitted_rho=0.999420252 */
            a11[0] = 34.517057f, a11[1] = -27.493421f, a11[2] = -0.193668f, a11[3] = -15.963853f;
            a12[0] = 4.949758f, a12[1] = -5.262704f, a12[2] = 0.173420f, a12[3] = -2.220506f;
            a13[0] = 0.151830f, a13[1] = -0.100853f, a13[2] = -0.009755f, a13[3] = -0.569612f;
            a14[0] = 0.028359f, a14[1] = -0.050858f, a14[2] = -0.007957f, a14[3] = -1.402015f;
            a15[0] = 16.702990f, a15[1] = -12.907912f, a15[2] = -0.046049f, a15[3] = 2.450340f;
            a16[0] = 3.463598f, a16[1] = -2.671067f, a16[2] = -0.009502f, a16[3] = 0.579837f;
            a21[0] = 8.589041f, a21[1] = -7.901305f, a21[2] = 0.688983f, a21[3] = 6.940582f;
            a22[0] = 4.565121f, a22[1] = -3.823865f, a22[2] = 0.078012f, a22[3] = 0.732427f;
            a23[0] = 0.489365f, a23[1] = -0.455238f, a23[2] = 0.047637f, a23[3] = -0.073603f;
            a24[0] = 1.013283f, a24[1] = -0.967445f, a24[2] = 0.114522f, a24[3] = -0.252594f;
            a25[0] = -6.693568f, a25[1] = 4.741360f, a25[2] = 0.176823f, a25[3] = 9.489923f;
            a26[0] = -1.403940f, a26[1] = 0.990409f, a26[2] = 0.038001f, a26[3] = 2.066121f;
            
            
            }
        else{
        if(lqr_leg_level == LEG_LOW){/*虽然参数是根据腿长拟合的,但是在不同腿长下还是采用不同的参数*/
            /* low: 低腿长正常控制 */
            /* Q = diag([ 200 70 1 2 2000 50 ]), R = diag([ 2.6 1.5 ]) */
            /* max_fit_error=0.0251941384, max_fitted_rho=0.999472229 */
            a11[0] = 44.916491f, a11[1] = -35.029826f, a11[2] = -0.513056f, a11[3] = -19.580009f;
            a12[0] = 9.437026f, a12[1] = -8.422520f, a12[2] = -0.013035f, a12[3] = -4.818016f;
            a13[0] = 0.595311f, a13[1] = -0.423460f, a13[2] = -0.023375f, a13[3] = -0.528522f;
            a14[0] = 1.071533f, a14[1] = -0.777102f, a14[2] = -0.051666f, a14[3] = -1.640241f;
            a15[0] = 51.616394f, a15[1] = -39.448117f, a15[2] = -0.298388f, a15[3] = 8.137242f;
            a16[0] = 8.735194f, a16[1] = -6.686198f, a16[2] = -0.045734f, a16[3] = 1.468985f;
            a21[0] = 10.686609f, a21[1] = -9.829563f, a21[2] = 0.846726f, a21[3] = 7.607636f;
            a22[0] = 8.038049f, a22[1] = -6.525741f, a22[2] = 0.168430f, a22[3] = 1.483438f;
            a23[0] = 0.657245f, a23[1] = -0.556778f, a23[2] = 0.028162f, a23[3] = 0.101608f;
            a24[0] = 2.198227f, a24[1] = -1.799988f, a24[2] = 0.064107f, a24[3] = 0.325382f;
            a25[0] = -16.337782f, a25[1] = 10.735298f, a25[2] = 0.878556f, a25[3] = 39.530768f;
            a26[0] = -2.982729f, a26[1] = 1.999001f, a26[2] = 0.141819f, a26[3] = 6.638457f;
       
        }

        if(lqr_leg_level == LEG_MID){
            /* mid: 中腿长正常控制 */
            /* Q = diag([ 200 70 1 1 1700 30 ]), R = diag([ 2.75 1.6 ]), Ts=0.001 s */
            /* max_fit_error=0.0226819363, max_fitted_rho=0.999518904 */
            a11[0] = 43.142026f, a11[1] = -33.525025f, a11[2] = -0.505906f, a11[3] = -19.051889f;
            a12[0] = 8.915141f, a12[1] = -7.894468f, a12[2] = -0.024500f, a12[3] = -4.673016f;
            a13[0] = 0.562547f, a13[1] = -0.400972f, a13[2] = -0.021748f, a13[3] = -0.516287f;
            a14[0] = 0.839415f, a14[1] = -0.608516f, a14[2] = -0.043039f, a14[3] = -1.507059f;
            a15[0] = 46.589231f, a15[1] = -35.674413f, a15[2] = -0.263574f, a15[3] = 7.487381f;
            a16[0] = 7.210225f, a16[1] = -5.537124f, a16[2] = -0.034589f, a16[3] = 1.242157f;
            a21[0] = 9.296017f, a21[1] = -8.700764f, a21[2] = 0.800062f, a21[3] = 7.554579f;
            a22[0] = 7.709849f, a22[1] = -6.260654f, a22[2] = 0.157116f, a22[3] = 1.469995f;
            a23[0] = 0.602449f, a23[1] = -0.515614f, a23[2] = 0.027895f, a23[3] = 0.096545f;
            a24[0] = 1.931830f, a24[1] = -1.588498f, a24[2] = 0.058392f, a24[3] = 0.293536f;
            a25[0] = -15.062438f, a25[1] = 9.980861f, a25[2] = 0.775197f, a25[3] = 35.935301f;
            a26[0] = -2.608729f, a26[1] = 1.781326f, a26[2] = 0.109786f, a26[3] = 5.458628f;

        }

        if(lqr_leg_level == LEG_HIG){
            /* high: 限制中高频机体反馈，降低结构模态激励 */
            /* Q = diag([ 160 100 1 1 1600 50 ]), R = diag([ 3 1.9 ]), Ts=0.001 s */
            /* max_fit_error=0.0213190161, max_fitted_rho=0.999510516 */
            a11[0] = 44.647427f, a11[1] = -34.737582f, a11[2] = -0.484587f, a11[3] = -18.690827f;
            a12[0] = 9.974339f, a12[1] = -8.596687f, a12[2] = -0.072661f, a12[3] = -5.176848f;
            a13[0] = 0.579421f, a13[1] = -0.416486f, a13[2] = -0.021068f, a13[3] = -0.486448f;
            a14[0] = 0.987806f, a14[1] = -0.719397f, a14[2] = -0.045875f, a14[3] = -1.439274f;
            a15[0] = 42.835882f, a15[1] = -32.811041f, a15[2] = -0.243302f, a15[3] = 6.855959f;
            a16[0] = 7.837019f, a16[1] = -6.008229f, a16[2] = -0.041388f, a16[3] = 1.332882f;
            a21[0] = 6.103569f, a21[1] = -6.237916f, a21[2] = 0.804483f, a21[3] = 7.055401f;
            a22[0] = 7.907414f, a22[1] = -6.456258f, a22[2] = 0.179584f, a22[3] = 1.500737f;
            a23[0] = 0.486144f, a23[1] = -0.423551f, a23[2] = 0.026684f, a23[3] = 0.076553f;
            a24[0] = 1.604203f, a24[1] = -1.339543f, a24[2] = 0.060009f, a24[3] = 0.239426f;
            a25[0] = -13.204702f, a25[1] = 8.817961f, a25[2] = 0.650105f, a25[3] = 32.608164f;
            a26[0] = -2.545948f, a26[1] = 1.721292f, a26[2] = 0.114952f, a26[3] = 5.960182f;
        }

    }


    LQRXObsBuf[LEFT][1] = leg[LEFT]->d_theta_lpf ;

    chassis_vx_l = chassis_kf_l.FilteredValue[0];/*观测用*/
    chassis_vx_r = chassis_kf_r.FilteredValue[0];
    chassis_vx_filter = 0.5f * (chassis_kf_l.FilteredValue[0] + chassis_kf_r.FilteredValue[0]);

    LQRXObsBuf[LEFT][3] = chassis_vx_filter;


    LQRXObsBuf[LEFT][4] = -ins.pitch * DEGREE_2_RAD ;
    /* ins.pitch is published from QEKF Roll. After the INS sign mapping,
     * d(-ins.pitch)/dt corresponds to +ins.gyro[0], already in rad/s.
     * The notch is deliberately limited to normal LQR modes so recovery and
     * all non-LQR users continue to see the raw gyro. */
    const uint8_t normal_lqr_notch_enable =
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN ||
        chassis_cmd.ctrl_mode == CHASSIS_AUTO;
    const float lqr_dphi = lqr_dphi_notch_update(
        ins.gyro[0], dt, normal_lqr_notch_enable);
    LQRXObsBuf[LEFT][5] = lqr_dphi;


    LQRXObsBuf[RIGHT][1] = leg[RIGHT]->d_theta_lpf ;

    LQRXObsBuf[RIGHT][3] = chassis_vx_filter;


    LQRXObsBuf[RIGHT][4] = -ins.pitch * DEGREE_2_RAD ;
    LQRXObsBuf[RIGHT][5] = lqr_dphi;

    const uint8_t position_mode_allowed =
        recovery_after_handoff() ||
        chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN ||
        chassis_cmd.ctrl_mode == CHASSIS_AUTO;
    const uint8_t position_stop_requested =
        fabsf(chassis_cmd.vx_set) < LQR_POSITION_CMD_DEADBAND;
    const uint8_t position_runtime_allowed =
        (!THETA_LENGTH_CALIBRATION_ENABLE) &&
        position_mode_allowed && position_stop_requested &&
        fabsf(ins.pitch) < 4.0f &&
        !chassis_safety_module_off_ground() &&
        !bump_action_active() &&
        !jump_motion_vx_ref_state.active;

    if (!position_runtime_allowed) {
        lqr_position_state = LQR_POSITION_DRIVE;
        lqr_position_capture_timer = 0.0f;
        lqr_position_hold_x = 0.0f;
        lqr_position_brake_offset = 0.0f;
        lqr_virtual_x_l = 0.0f;
        lqr_virtual_x_r = 0.0f;
    } else {
        if (lqr_position_state == LQR_POSITION_DRIVE) {
            lqr_position_state = LQR_POSITION_BRAKE;
            lqr_position_capture_timer = 0.0f;
            lqr_position_hold_x = 0.0f;
            lqr_position_brake_offset = 0.0f;
        }

        if (lqr_position_state == LQR_POSITION_BRAKE) {
            /* Preserve position braking, then discard the braking history once
             * the chassis has actually stopped. */
            lqr_virtual_x_l += chassis_vx_filter * dt * X_RATIO;
            lqr_virtual_x_r += chassis_vx_filter * dt * X_RATIO;
            LIMIT_MIN_MAX(lqr_virtual_x_l,
                          -LQR_POSITION_LIMIT, LQR_POSITION_LIMIT);
            LIMIT_MIN_MAX(lqr_virtual_x_r,
                          -LQR_POSITION_LIMIT, LQR_POSITION_LIMIT);

            if (fabsf(chassis_vx_filter) < LQR_POSITION_CAPTURE_VX_MAX) {
                lqr_position_capture_timer += dt;
                if (lqr_position_capture_timer >= LQR_POSITION_CAPTURE_TIME) {
                    lqr_position_state = LQR_POSITION_HOLD;
                    lqr_position_capture_timer = LQR_POSITION_CAPTURE_TIME;
                    lqr_position_brake_offset = 0.5f *
                        (lqr_virtual_x_l + lqr_virtual_x_r);
                    lqr_position_hold_x = 0.0f;
                }
            } else {
                lqr_position_capture_timer = 0.0f;
            }
        } else {
            /* Fade the braking history while integrating displacement from the
             * newly captured hold point. This avoids a control-output step. */
            lqr_position_brake_offset = move_towards_f32(
                lqr_position_brake_offset, 0.0f,
                LQR_POSITION_BRAKE_RELEASE_RATE * dt);
            lqr_position_hold_x += chassis_vx_filter * dt * X_RATIO;
            LIMIT_MIN_MAX(lqr_position_hold_x,
                          -LQR_POSITION_LIMIT, LQR_POSITION_LIMIT);
            lqr_virtual_x_l = lqr_position_hold_x +
                              lqr_position_brake_offset;
            LIMIT_MIN_MAX(lqr_virtual_x_l,
                          -LQR_POSITION_LIMIT, LQR_POSITION_LIMIT);
            lqr_virtual_x_r = lqr_virtual_x_l;
        }
    }

    LQRXObsBuf[LEFT][2] = lqr_virtual_x_l;
    LQRXObsBuf[RIGHT][2] = lqr_virtual_x_r;

    /* ==================== 几何补偿 + 按腿长 theta 微补偿 ==================== */
    float safe_l0_L = (leg[LEFT]->l0 > 0.05f) ? leg[LEFT]->l0 : 0.05f;
    float safe_l0_R = (leg[RIGHT]->l0 > 0.05f) ? leg[RIGHT]->l0 : 0.05f;

    /* --- 当前腿长 index：RECOVERY+handoff 强制低腿长 --- */
    leg_level_e theta_comp_level = chassis_cmd.leg_level;
    if (recovery_after_handoff()) {
        theta_comp_level = LEG_LOW;
    }

    uint8_t level_idx = 0;
    switch (theta_comp_level) {
        case LEG_LOW: level_idx = 0; break;
        case LEG_MID: level_idx = 1; break;
        case LEG_HIG: level_idx = 2; break;
        default: level_idx = 0; break;
    }

    float theta_cal_target_ref;
    switch (theta_comp_level) {
        case LEG_LOW:
            theta_cal_target_ref = THETA_COMP_CAL_TARGET_LOW;
            break;
        case LEG_MID:
            theta_cal_target_ref = THETA_COMP_CAL_TARGET_MID;
            break;
        case LEG_HIG:
            theta_cal_target_ref = THETA_COMP_CAL_TARGET_HIG;
            break;
        default:
            theta_cal_target_ref = THETA_COMP_CAL_TARGET_LOW;
            break;
    }
    float theta_cal_length_ref = 0.5f *
        (leg[LEFT]->length_ref + leg[RIGHT]->length_ref);
    uint8_t theta_cal_ref_ok =
        (!THETA_LENGTH_CALIBRATION_ENABLE) ||
        fabsf(theta_cal_length_ref - theta_cal_target_ref) <
            THETA_CAL_REF_TOLERANCE;
    float theta_cal_actual_length = 0.5f *
        (leg[LEFT]->l0 + leg[RIGHT]->l0);
    uint8_t theta_cal_actual_length_ok =
        (!THETA_LENGTH_CALIBRATION_ENABLE) ||
        fabsf(theta_cal_actual_length - theta_cal_target_ref) <
            THETA_CAL_ACTUAL_LENGTH_ERROR_MAX;

    if (THETA_LENGTH_CALIBRATION_ENABLE &&
        theta_comp_level != theta_cal_last_level) {
        theta_cal_last_level = theta_comp_level;
        theta_cal_vx_lpf = chassis_vx_filter;
        theta_cal_vx_slow_lpf = chassis_vx_filter;
        theta_cal_vx_trend = 0.0f;
        theta_cal_leg_settle_timer = 0.0f;
        theta_cal_zero_hold_timer = 0.0f;
        theta_cal_latch_bad_timer = 0.0f;
    }

    /* Leg switching freezes learning only. The fitted base compensation is
     * already continuous in actual length and must never be withdrawn here. */
    if (chassis_cmd.leg_level != last_leg_level_for_comp) {
        last_leg_level_for_comp = chassis_cmd.leg_level;
        leg_switch_freeze_timer = THETA_LEVEL_SWITCH_FREEZE_TIME;
        theta_comp_learn_settle_timer = 0.0f;
    }

    if (leg_switch_freeze_timer > 0.0f) {
        leg_switch_freeze_timer -= dt;
        if (leg_switch_freeze_timer < 0.0f) {
            leg_switch_freeze_timer = 0.0f;
        }
    }

    /* --- 应用/学习使能判断 --- */
    uint8_t recovery_stand = recovery_after_handoff();

    uint8_t mode_can_apply =
        recovery_stand ||
        (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
         chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
         chassis_cmd.ctrl_mode == CHASSIS_SPIN ||
         chassis_cmd.ctrl_mode == CHASSIS_AUTO);

    uint8_t mode_can_learn =
        (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
         chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL);

    uint8_t cmd_stop   = (fabsf(chassis_cmd.vx_set) < 20.0f);
    uint8_t posture_ok = (fabsf(ins.pitch) < 5.0f) &&
                         (fabsf(LQRXObsBuf[LEFT][5]) <
                          THETA_LEVEL_COMP_PITCH_RATE_MAX) &&
                         (fabsf(leg[LEFT]->d_theta_lpf) <
                          THETA_LEVEL_COMP_DTHETA_MAX) &&
                         (fabsf(leg[RIGHT]->d_theta_lpf) <
                          THETA_LEVEL_COMP_DTHETA_MAX);
    uint8_t vx_ok      = (fabsf(chassis_vx_filter) < THETA_LEVEL_COMP_VX_ENABLE_MAX);
    uint8_t x_ok       = (fabsf(0.5f *
                                (lqr_virtual_x_l + lqr_virtual_x_r)) <
                          THETA_LEVEL_COMP_X_ENABLE_MAX);
    uint8_t leg_stable  = (chassis_cmd.leg_leng_change == LENGTH_STAY) &&
                           (leg_switch_freeze_timer <= 0.0f);

    if (THETA_LENGTH_CALIBRATION_ENABLE && theta_cal_ref_ok &&
        theta_cal_actual_length_ok &&
        chassis_cmd.leg_leng_change == LENGTH_STAY) {
        theta_cal_leg_settle_timer += dt;
        if (theta_cal_leg_settle_timer > THETA_CAL_LEG_SETTLE_TIME) {
            theta_cal_leg_settle_timer = THETA_CAL_LEG_SETTLE_TIME;
        }
    } else {
        theta_cal_leg_settle_timer = 0.0f;
    }
    uint8_t theta_cal_ready =
        (!THETA_LENGTH_CALIBRATION_ENABLE) ||
        theta_cal_leg_settle_timer >= THETA_CAL_LEG_SETTLE_TIME;

    if (THETA_LENGTH_CALIBRATION_ENABLE) {
        float cal_alpha = dt / (THETA_CAL_VX_LPF_RC + dt);
        float trend_alpha = dt / (THETA_CAL_VX_TREND_RC + dt);
        LIMIT_MIN_MAX(cal_alpha, 0.0f, 1.0f);
        LIMIT_MIN_MAX(trend_alpha, 0.0f, 1.0f);
        theta_cal_vx_lpf +=
            cal_alpha * (chassis_vx_filter - theta_cal_vx_lpf);
        theta_cal_vx_slow_lpf +=
            trend_alpha * (theta_cal_vx_lpf - theta_cal_vx_slow_lpf);
        theta_cal_vx_trend =
            (theta_cal_vx_lpf - theta_cal_vx_slow_lpf) /
            THETA_CAL_VX_TREND_RC;
    }

    /* 正常控制中保持补偿连续；姿态条件只冻结慢积分学习，不能移动 LQR 零点。 */
    uint8_t comp_apply_enable = mode_can_apply;
    uint8_t learn_position_ok =
        THETA_LENGTH_CALIBRATION_ENABLE ||
        (lqr_position_state == LQR_POSITION_HOLD);
    uint8_t learn_common_ok = mode_can_apply && mode_can_learn &&
                              posture_ok && cmd_stop && leg_stable &&
                              learn_position_ok && theta_cal_ref_ok &&
                              theta_cal_ready &&
                              (!chassis_safety_module_off_ground());

    /* A short zero crossing can still satisfy the initial latch. Keep
     * validating a latched value while the driver command remains zero; a
     * sustained renewed drift releases only the active slot for more learning. */
    if (THETA_LENGTH_CALIBRATION_ENABLE &&
        theta_cal_value_latched[level_idx] && learn_common_ok) {
        if (fabsf(theta_cal_vx_lpf) > THETA_CAL_LATCH_RELEASE_VX) {
            theta_cal_latch_bad_timer += dt;
            if (theta_cal_latch_bad_timer >= THETA_CAL_LATCH_RELEASE_TIME) {
                theta_cal_value_latched[level_idx] = 0U;
                theta_cal_latch_bad_timer = 0.0f;
                theta_cal_zero_hold_timer = 0.0f;
            }
        } else {
            theta_cal_latch_bad_timer = 0.0f;
        }
    } else {
        theta_cal_latch_bad_timer = 0.0f;
    }

    uint8_t comp_static_learn_candidate;
#if THETA_LENGTH_CALIBRATION_ENABLE
    comp_static_learn_candidate =
        learn_common_ok && !theta_cal_value_latched[level_idx] &&
        fabsf(chassis_vx_filter) < THETA_CAL_VX_ENABLE_MAX;
#else
    comp_static_learn_candidate = learn_common_ok && vx_ok && x_ok;
#endif
    uint8_t low_drift_learn_candidate =
        (!THETA_LENGTH_CALIBRATION_ENABLE) &&
        learn_common_ok && level_idx == 0U &&
        !theta_low_drift_latched &&
        fabsf(chassis_vx_filter) < THETA_LOW_DRIFT_VX_ENABLE_MAX;
    uint8_t comp_learn_candidate =
        comp_static_learn_candidate || low_drift_learn_candidate;

    if (comp_learn_candidate) {
        theta_comp_learn_settle_timer += dt;
        if (theta_comp_learn_settle_timer > THETA_LEVEL_COMP_SETTLE_TIME) {
            theta_comp_learn_settle_timer = THETA_LEVEL_COMP_SETTLE_TIME;
        }
    } else {
        theta_comp_learn_settle_timer = 0.0f;
    }
    uint8_t comp_learn_enable =
        comp_learn_candidate &&
        theta_comp_learn_settle_timer >= THETA_LEVEL_COMP_SETTLE_TIME;

    /* --- apply_gain 斜坡：只控制补偿是否渐进应用，不清 theta_level_comp --- */
    if (recovery_stand) {
        /* Recovery already owns an 80 ms handoff ramp. Keeping a second slow
         * ramp here would continue moving the theta zero after wheel takeover. */
        theta_level_apply_gain = 1.0f;
    } else if (comp_apply_enable) {
        theta_level_apply_gain += THETA_LEVEL_APPLY_RAMP_RATE * dt;
    } else {
        theta_level_apply_gain -= THETA_LEVEL_APPLY_RAMP_RATE * dt;
    }
    LIMIT_MIN_MAX(theta_level_apply_gain, 0.0f, 1.0f);

    /* Keep the LOW-only fallback out of actual-length interpolation. During a
     * leg-length ramp its gain is held, then changed after the ramp so the LQR
     * zero and K matrix do not move at the same time. */
    if (!THETA_LENGTH_CALIBRATION_ENABLE &&
        chassis_cmd.leg_leng_change == LENGTH_STAY) {
        float low_drift_gain_target = (level_idx == 0U) ? 1.0f : 0.0f;
        theta_low_drift_apply_gain = move_towards_f32(
            theta_low_drift_apply_gain, low_drift_gain_target,
            THETA_LOW_DRIFT_APPLY_RAMP_RATE * dt);
    }
    LIMIT_MIN_MAX(theta_low_drift_apply_gain, 0.0f, 1.0f);

    /* --- 慢速学习：标定模式直接更新三点样本；多项式模式仅更新
     * 各档位的小范围积分残差。 --- */
    uint8_t low_drift_learning =
        comp_learn_enable && low_drift_learn_candidate &&
        !comp_static_learn_candidate;
    if (comp_learn_enable) {
        float vx_for_learning = THETA_LENGTH_CALIBRATION_ENABLE ?
                                theta_cal_vx_lpf : chassis_vx_filter;
        if (THETA_LENGTH_CALIBRATION_ENABLE) {
            /* Lead the slow integrator with the filtered speed trend so it
             * backs off before a zero crossing instead of hunting around it. */
            vx_for_learning += THETA_CAL_TREND_DAMP_TIME *
                               theta_cal_vx_trend;
        }
        float vx_err = 0.0f - vx_for_learning;
        float learn_ki = THETA_LENGTH_CALIBRATION_ENABLE ?
                         THETA_CAL_KI : THETA_LEVEL_COMP_KI;
        if (THETA_LENGTH_CALIBRATION_ENABLE) {
            LIMIT_MIN_MAX(vx_err,
                          -THETA_CAL_VX_ERROR_LIMIT,
                           THETA_CAL_VX_ERROR_LIMIT);
        }
        if (low_drift_learning) {
            LIMIT_MIN_MAX(vx_err,
                          -THETA_LOW_DRIFT_VX_ERROR_LIMIT,
                           THETA_LOW_DRIFT_VX_ERROR_LIMIT);
            learn_ki = THETA_LOW_DRIFT_KI;
        }
        float learn_deadband = THETA_LENGTH_CALIBRATION_ENABLE ?
                               THETA_CAL_LEARN_VX_DEADBAND :
                               THETA_LEVEL_COMP_VX_DEADBAND;
        if (fabsf(vx_err) < learn_deadband) {
            vx_err = 0.0f;
        }

        if (low_drift_learning) {
            theta_low_drift_trim += learn_ki * vx_err * dt;
        } else {
            theta_level_comp[level_idx] += learn_ki * vx_err * dt;
        }
#if THETA_LENGTH_CALIBRATION_ENABLE
        LIMIT_MIN_MAX(theta_level_comp[level_idx],
                      -THETA_COMP_CAL_LEARN_LIMIT,
                       THETA_COMP_CAL_LEARN_LIMIT);
#else
        if (low_drift_learning) {
            LIMIT_MIN_MAX(theta_low_drift_trim,
                          -THETA_LOW_DRIFT_TRIM_LIMIT,
                           THETA_LOW_DRIFT_TRIM_LIMIT);
        }
        LIMIT_MIN_MAX(theta_level_comp[level_idx],
                      -THETA_COMP_POLY_TRIM_LIMIT,
                       THETA_COMP_POLY_TRIM_LIMIT);
#endif
    }

    /* A slow zero crossing can remain inside the speed band for several
     * seconds. Require both low speed and a flat speed trend before latching. */
    if (THETA_LENGTH_CALIBRATION_ENABLE && comp_learn_enable) {
        if (fabsf(theta_cal_vx_lpf) < THETA_CAL_ZERO_VX_BAND &&
            fabsf(theta_cal_vx_trend) < THETA_CAL_ZERO_VX_TREND_MAX) {
            theta_cal_zero_hold_timer += dt;
            if (theta_cal_zero_hold_timer >= THETA_CAL_ZERO_HOLD_TIME) {
                theta_cal_zero_hold_timer = THETA_CAL_ZERO_HOLD_TIME;
                theta_cal_value_latched[level_idx] = 1U;
            }
        } else {
            theta_cal_zero_hold_timer = 0.0f;
        }
    } else if (THETA_LENGTH_CALIBRATION_ENABLE &&
               !theta_cal_value_latched[level_idx]) {
        theta_cal_zero_hold_timer = 0.0f;
    }

    uint8_t low_drift_converge_candidate =
        comp_learn_enable && low_drift_learn_candidate;
    if (low_drift_converge_candidate) {
        if (fabsf(chassis_vx_filter) < THETA_LOW_DRIFT_CONVERGE_VX_MAX) {
            theta_low_drift_converge_timer += dt;
            if (theta_low_drift_converge_timer >=
                THETA_LOW_DRIFT_CONVERGE_TIME) {
                theta_low_drift_converge_timer =
                    THETA_LOW_DRIFT_CONVERGE_TIME;
                theta_low_drift_latched = 1U;
            }
        } else {
            theta_low_drift_converge_timer = 0.0f;
        }
    } else if (!theta_low_drift_latched) {
        theta_low_drift_converge_timer = 0.0f;
    }

    /* --- NaN/Inf 安全保护 --- */
    if (isnan(theta_level_comp[0]) || isinf(theta_level_comp[0]) ||
        isnan(theta_level_comp[1]) || isinf(theta_level_comp[1]) ||
        isnan(theta_level_comp[2]) || isinf(theta_level_comp[2]) ||
        isnan(theta_low_drift_trim) || isinf(theta_low_drift_trim) ||
        isnan(theta_low_drift_apply_gain) ||
        isinf(theta_low_drift_apply_gain)) {
        theta_level_comp_hard_clear();
    }

    /* --- 补偿量换算：几何项使用左右实际腿长，多项式项使用实际平均腿长。 --- */
    float l0_average = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    float theta_poly_base = 0.0f;
    float theta_strict_trim = 0.0f;
    float theta_comp_trim;
    float theta_comp_active;
    float theta_geo_comp_L = asinf(DX_OFFSET / safe_l0_L);
    float theta_geo_comp_R = asinf(DX_OFFSET / safe_l0_R);
#if THETA_LENGTH_CALIBRATION_ENABLE
    theta_comp_trim = theta_level_comp[level_idx];
    theta_comp_active = theta_comp_trim;
#else
    theta_strict_trim = theta_comp_trim_interp(l0_average);
    theta_comp_trim = theta_strict_trim +
        theta_low_drift_apply_gain * theta_low_drift_trim;
    theta_poly_base = theta_comp_poly_eval(l0_average);
    theta_comp_active = theta_poly_base + theta_comp_trim;
#endif
    float theta_level_cur = theta_level_apply_gain * theta_comp_active;

    /* Recovery owns the transition edge and soft-start gain. */
    chassis_recovery_module_update_handoff_lqr();

    /* handoff_comp_gain：Recovery handoff 时渐进加载补偿，正常模式 = 1 */
    float handoff_comp_gain = recovery_after_handoff() ?
        chassis_recovery_module_handoff_gain() : 1.0f;
    float theta_total_comp_L = handoff_comp_gain * (theta_geo_comp_L + theta_level_cur);
    float theta_total_comp_R = handoff_comp_gain * (theta_geo_comp_R + theta_level_cur);

    /* The baseline LQR target is level in the compensated frame. The landing
     * owner may replace theta below; clearing every cycle prevents a stale
     * capture reference from leaking across any mode transition. */
    LQRXRefBuf[LEFT][0] = 0.0f;
    LQRXRefBuf[RIGHT][0] = 0.0f;
    LQRXRefBuf[LEFT][1] = 0.0f;
    LQRXRefBuf[RIGHT][1] = 0.0f;

    uint8_t airborne_allowed = chassis_safety_module_airborne_allowed();

    if (chassis_safety_module_off_ground() && airborne_allowed)
    {
        LQRXObsBuf[LEFT][0] = leg[LEFT]->theta;
        LQRXObsBuf[RIGHT][0] = leg[RIGHT]->theta;
        LQRXObsBuf[LEFT][1] = leg[LEFT]->d_theta_lpf;
        LQRXObsBuf[RIGHT][1] = leg[RIGHT]->d_theta_lpf;
        LQRXRefBuf[LEFT][0] = 0.0f;
        LQRXRefBuf[RIGHT][0] = 0.0f;
        LQRXRefBuf[LEFT][1] = 0.0f;
        LQRXRefBuf[RIGHT][1] = 0.0f;
    }
    else if ((chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
              !recovery_after_handoff()) ||
             chassis_cmd.ctrl_mode == CHASSIS_INIT ||
             chassis_cmd.ctrl_mode == CHASSIS_RELAX)
    {
        LQRXObsBuf[LEFT][0] = leg[LEFT]->theta;
        LQRXObsBuf[RIGHT][0] = leg[RIGHT]->theta;
    }
    else
    {
        LQRXObsBuf[LEFT][0] = leg[LEFT]->theta - theta_total_comp_L;
        LQRXObsBuf[RIGHT][0] = leg[RIGHT]->theta - theta_total_comp_R;
    }

    /* Jump-origin contact recovery alone gets the timestamp kinematic rate.
     * Blend it with the legacy VMC LPF through the same scope gain used by the
     * hip and wheel allocation; ordinary standing and airborne paths retain
     * their existing observations. */
#if JUMP_LOCAL_RATE_FEEDBACK_ENABLE && JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    if (jump_ground_recovery_scope_active &&
        !chassis_safety_module_off_ground() &&
        jump_kin_rates_fresh && jump_kin_rates_usable &&
        isfinite(jump_kin_theta_rate[LEFT]) &&
        isfinite(jump_kin_theta_rate[RIGHT])) {
        const float rate_gain = jump_ground_recovery_gain;
        LQRXObsBuf[LEFT][1] += rate_gain *
            (jump_kin_theta_rate[LEFT] - LQRXObsBuf[LEFT][1]);
        LQRXObsBuf[RIGHT][1] += rate_gain *
            (jump_kin_theta_rate[RIGHT] - LQRXObsBuf[RIGHT][1]);
    }
#endif

    /* --- 标定与调试变量 --- */
    uint16_t theta_cal_flags = (uint16_t)level_idx;
#if THETA_LENGTH_CALIBRATION_ENABLE
    theta_cal_flags |= 0x0004u;
#endif
    if (comp_apply_enable) {
        theta_cal_flags |= 0x0008u;
    }
    if (comp_learn_enable) {
        theta_cal_flags |= 0x0010u;
    }
    if (fabsf(chassis_vx_filter) < THETA_LEVEL_COMP_VX_DEADBAND) {
        theta_cal_flags |= 0x0020u;
    }
    if (leg_stable) {
        theta_cal_flags |= 0x0040u;
    }
    if (posture_ok) {
        theta_cal_flags |= 0x0080u;
    }
    if (!chassis_safety_module_off_ground()) {
        theta_cal_flags |= 0x0100u;
    }
    if (lqr_position_state == LQR_POSITION_HOLD) {
        theta_cal_flags |= 0x0200u;
    }
    if (comp_learn_candidate) {
        theta_cal_flags |= 0x0400u;
    }
    if (low_drift_learn_candidate) {
        theta_cal_flags |= 0x0800u;
    }
    if (theta_low_drift_latched) {
        theta_cal_flags |= 0x1000u;
    }
    if (theta_low_drift_apply_gain > 0.001f) {
        theta_cal_flags |= 0x2000u;
    }
    if (theta_cal_value_latched[level_idx]) {
        theta_cal_flags |= 0x4000u;
    }

    dbg_theta_level_comp_cur = theta_comp_active;
    dbg_theta_level_comp_low = theta_level_comp[0];
    dbg_theta_level_comp_mid = theta_level_comp[1];
    dbg_theta_level_comp_hig = theta_level_comp[2];
    dbg_theta_level_apply_gain = theta_level_apply_gain;
    dbg_theta_level_apply_enable = comp_apply_enable;
    dbg_theta_level_learn_enable = comp_learn_enable;
    dbg_theta_vx_filter = chassis_vx_filter;
    dbg_theta_total_comp_L = theta_total_comp_L;
    dbg_theta_total_comp_R = theta_total_comp_R;
    dbg_theta_l0_average = l0_average;
    dbg_theta_poly_base = theta_poly_base;
    dbg_theta_active_comp = theta_comp_active;
    dbg_theta_active_trim = theta_comp_trim;
    dbg_theta_cal_flags = (float)theta_cal_flags;
    dbg_theta_strict_trim = theta_strict_trim;

    /* Diagnostic-only displacement accumulated while the driver requests stop. */
    if (mode_can_learn && cmd_stop && leg_stable) {
        dbg_theta_drift_x += chassis_vx_filter * dt;
        LIMIT_MIN_MAX(dbg_theta_drift_x, -2.0f, 2.0f);
    } else {
        dbg_theta_drift_x = 0.0f;
    }
    /* ========================================================================= */


    LQRXRefBuf[RIGHT][2] = 0;/*期望禁止时位移为0*/
    LQRXRefBuf[LEFT][2] = 0;

    dbg_chassis_vx_limit = chassis_vx_limit_calculate();
    float vx_command = (chassis_cmd.vx_set / RC_DBUS_MAX_VALUE) *
                       dbg_chassis_vx_limit;
    LIMIT_MIN_MAX(vx_command, -dbg_chassis_vx_limit, dbg_chassis_vx_limit);
    const uint8_t jump_motion_owned = (uint8_t)(
        jump_state != JUMP_STATE_IDLE ||
        chassis_safety_module_landing_buffer_jump_origin() ||
        chassis_safety_module_landing_postbuffer_active() ||
        chassis_safety_module_landing_airborne_guard_active());
    LQRXRefBuf[LEFT][3] = jump_landing_vx_ref_apply(
        &jump_motion_vx_ref_state, vx_command, dt,
        JUMP_LANDING_VX_REF_ACCEL_MPS2, jump_motion_owned,
        chassis_safety_module_off_ground(), 0.04f);
    LQRXRefBuf[RIGHT][3] = LQRXRefBuf[LEFT][3];


    leg[LEFT]->l0_average = 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0); /*TODO 在左右腿长不一致的情况下的k是否合理有待讨论*/
    leg[RIGHT]->l0_average = leg[LEFT]->l0_average;
    leg[LEFT]->l0_pow3 = leg[LEFT]->l0_average * leg[LEFT]->l0_average * leg[LEFT]->l0_average;
    leg[LEFT]->l0_pow2 = leg[LEFT]->l0_average * leg[LEFT]->l0_average ;

    float lqr_k_target[2][6] = {
        {
            a11[0] * leg[LEFT]->l0_pow3 + a11[1] * leg[LEFT]->l0_pow2 + a11[2] * leg[LEFT]->l0_average + a11[3],
            a12[0] * leg[LEFT]->l0_pow3 + a12[1] * leg[LEFT]->l0_pow2 + a12[2] * leg[LEFT]->l0_average + a12[3],
            a13[0] * leg[LEFT]->l0_pow3 + a13[1] * leg[LEFT]->l0_pow2 + a13[2] * leg[LEFT]->l0_average + a13[3],
            a14[0] * leg[LEFT]->l0_pow3 + a14[1] * leg[LEFT]->l0_pow2 + a14[2] * leg[LEFT]->l0_average + a14[3],
            a15[0] * leg[LEFT]->l0_pow3 + a15[1] * leg[LEFT]->l0_pow2 + a15[2] * leg[LEFT]->l0_average + a15[3],
            a16[0] * leg[LEFT]->l0_pow3 + a16[1] * leg[LEFT]->l0_pow2 + a16[2] * leg[LEFT]->l0_average + a16[3],
        },
        {
            a21[0] * leg[LEFT]->l0_pow3 + a21[1] * leg[LEFT]->l0_pow2 + a21[2] * leg[LEFT]->l0_average + a21[3],
            a22[0] * leg[LEFT]->l0_pow3 + a22[1] * leg[LEFT]->l0_pow2 + a22[2] * leg[LEFT]->l0_average + a22[3],
            a23[0] * leg[LEFT]->l0_pow3 + a23[1] * leg[LEFT]->l0_pow2 + a23[2] * leg[LEFT]->l0_average + a23[3],
            a24[0] * leg[LEFT]->l0_pow3 + a24[1] * leg[LEFT]->l0_pow2 + a24[2] * leg[LEFT]->l0_average + a24[3],
            a25[0] * leg[LEFT]->l0_pow3 + a25[1] * leg[LEFT]->l0_pow2 + a25[2] * leg[LEFT]->l0_average + a25[3],
            a26[0] * leg[LEFT]->l0_pow3 + a26[1] * leg[LEFT]->l0_pow2 + a26[2] * leg[LEFT]->l0_average + a26[3],
        },
    };

    /* Recovery 使用独立矩阵；正常站立完整使用求解得到的 dphi 增益。 */
    if (!(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
          !recovery_after_handoff())) {
        lqr_k_target[0][5] *= LQR_NORMAL_DPHI_GAIN;
        lqr_k_target[1][5] *= LQR_NORMAL_DPHI_GAIN;
    }
    lqr_k_transition_update(lqr_k_target, lqr_leg_level, dt,
                            normal_leg_transition_mode());

    if(chassis_safety_module_off_ground() && airborne_allowed){

        chassis_safety_module_set_wheel_shut(1U);

        LQRXRefBuf[RIGHT][2] = 0;
        LQRXRefBuf[LEFT][2] = 0;

        if (!jump_motion_vx_ref_state.active) {
            LQRXRefBuf[LEFT][3]  = 0;
            LQRXRefBuf[RIGHT][3] = 0;
        }

        yaw_target = ins.yaw_total_angle * DEGREE_2_RAD;

        MatLQR_K[0][0] = 0.0f;
        MatLQR_K[0][1] = 0.0f;
        MatLQR_K[0][2] = 0.0f;
        MatLQR_K[0][3] = 0.0f;
        MatLQR_K[0][4] = 0.0f;
        MatLQR_K[0][5] = 0.0f;
        MatLQR_K[1][0] = lqr_k_target[1][0];
        MatLQR_K[1][1] = lqr_k_target[1][1];
        MatLQR_K[1][2] = 0.0f;
        MatLQR_K[1][3] = 0.0f;
        /* Keep the normal LQR wheel row off in flight; the bounded reaction
         * channel below provides the separate jump pitch authority. */
        MatLQR_K[1][4] = jump_control_active() ?
            JUMP_AIR_PITCH_K_GAIN * lqr_k_target[1][4] : 0.0f;
        MatLQR_K[1][5] = jump_control_active() ?
            JUMP_AIR_PITCH_D_GAIN * lqr_k_target[1][5] : 0.0f;

    }else{/*TODO 在腿长不切换是每次都进行运算浪费资源 */
        /*更新LQR反馈矩阵K*/
        chassis_safety_module_set_wheel_shut(0U);

        memcpy(MatLQR_K, lqr_k_transition_current, sizeof(MatLQR_K));

    }
}



#ifdef BSP_CHASSIS_LEG_MODE
static void leg_calc()
{
    jump_cycle_dt_update();
    jump_hip_feedback_snapshot_t hip_feedback = {0};
    jump_hip_feedback_snapshot_capture(&hip_feedback);

    /* A jump or jump-origin landing owner must never run another VMC cycle
     * from frozen hip geometry.  Check the snapshot before phi/vmc calculation
     * so a stale CAN frame cannot produce one-sided extension torque. */
    const uint32_t now_tick = HAL_GetTick();
    uint32_t hip_feedback_max_age_ms = UINT32_MAX;
    const uint8_t feedback_fresh =
        jump_hip_feedback_snapshot_fresh(
            &hip_feedback, now_tick, &hip_feedback_max_age_ms);
    const uint8_t jump_landing_owner_scope =
        chassis_safety_module_jump_landing_owner_active();
    if ((jump_control_active() || jump_landing_owner_scope) &&
        !feedback_fresh) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }

    // 左腿解算
    /*Warning: 若是电机没有按照规定安装,需要调整phi1和phi4的计算,这会很大程度影响到后续的vmc解算*/
    leg[LEFT]->phi_calc_L(leg[LEFT], hip_feedback.angle_abs[0],
                          hip_feedback.angle_abs[1]);
    leg[LEFT]->vmc_calc(leg[LEFT],&ins,chassis_dt);
    // 右腿解算
    leg[RIGHT]->phi_calc_R(leg[RIGHT], hip_feedback.angle_abs[2],
                           hip_feedback.angle_abs[3]);
    leg[RIGHT]->vmc_calc(leg[RIGHT],&ins,chassis_dt);

    /* BUMP 入口门控。914 没有 611 的 chassis_feedback_update()，用 914 自己的
     * 髋反馈新鲜度判据（页 52 的 hips_fresh）加 IMU 角速度有限性等价替代。
     * 本次移植中只作为 BUMP 状态机的入口门控，不承担其他职责。 */
    const float now_ms = dwt_get_time_ms();
    const float position_length[CHASSIS_SIDE_COUNT] = {
        leg[LEFT]->l0, leg[RIGHT]->l0
    };
    const float position_theta[CHASSIS_SIDE_COUNT] = {
        leg[LEFT]->theta, leg[RIGHT]->theta
    };
    chassis_safety_module_set_position_feedback(
        position_length, position_theta, hip_feedback.rx_count,
        feedback_fresh,
        feedback_fresh ? (float)hip_feedback_max_age_ms : -1.0f);
    control_feedback_valid = (uint8_t)(
        feedback_fresh && isfinite(ins.pitch_rate_rad_s));
    bump_ins_feedback_fresh = lateral_inertial_ff_sample_fresh(
        now_tick, ins.publish_tick_ms, ins.publish_count,
        LATERAL_FF_INS_MAX_AGE_MS);
    bump_phi0_rate_update(now_ms, now_tick, &hip_feedback,
                          feedback_fresh);

    /* Keep this observer in the same geometry frame as both VMC solutions.
     * Its rates affect LQR only during the latched jump-ground recovery scope;
     * legacy leg LPFs and ordinary standing observations remain unchanged. */
    jump_kinematic_rate_update(now_ms);
    jump_landing_length_rate_update(
        now_ms, now_tick, &hip_feedback, feedback_fresh);
    /* Clear stale scope before recovery/idle early returns, and fail closed if
     * a previously latched scope has lost its required observer pair. This
     * zero-time pass never consumes the per-cycle hold/fade clock. */
    if (!jump_ground_recovery_update(0.0f, 0U)) {
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }
    if ((jump_control_active() || jump_landing_owner_scope) &&
        (!jump_kin_geometry_valid ||
         (jump_control_active() && !jump_kin_rates_usable) ||
         (jump_landing_owner_scope &&
          JUMP_LOCAL_RATE_FEEDBACK_ENABLE &&
          JUMP_KINEMATIC_RATE_OBSERVER_ENABLE &&
          !jump_kin_rates_usable))) {
        jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }

    if (!chassis_safety_module_jump_landing_owner_active()) {
        /* Keep idle/init/abort/recovery early returns from preserving a stale
         * touchdown clock when they bypass the normal LQR-cycle update. */
        jump_landing_balance_reset(&jump_landing_balance_state);
        jump_landing_ground_balance_gain = 0.0f;
    }

    chassis_safety_module_landing_fast_exit_cancel_if_unsafe();
    if (chassis_recovery_module_apply_retract_control()) {
        return;
    }

    /* BUMP 只在自身状态机内推进；不会启动 jump_state 或跳跃空中 profile。
     * 611 的 jump_control_prepare_cycle() 无参，914 带 stroke_release 指针，
     * 这里只在其之前插入 BUMP，不改动 914 的函数签名。 */
    bump_control_prepare_cycle();
    bump_length_reference_update();
    uint8_t stroke_release_invalid = 0U;
    jump_control_prepare_cycle(&stroke_release_invalid);
    if (stroke_release_invalid) {
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }
    if (chassis_safety_module_landing_fast_exit_ready(
            leg[LEFT]->l0, leg[RIGHT]->l0)) {
        /* Run the normal jump guards first. Then finish while the landing
         * owner is still active so theta/wheel history survives buffer reset. */
        if (jump_control_active()) {
            jump_control_finish(JUMP_FAULT_NONE);
        }
        if (!chassis_safety_module_landing_fast_exit_begin()) {
            jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
            memset(vmc_out_l, 0, sizeof(vmc_out_l));
            memset(vmc_out_r, 0, sizeof(vmc_out_r));
            return;
        }
    }
    if (!jump_landing_balance_update(jump_cycle_dt_s, 1U)) {
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }
    if (!jump_ground_recovery_update(jump_cycle_dt_s, 1U)) {
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }
    update_LQR_obs();
    if (!jump_landing_balance_apply_lqr_refs()) {
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }
    LQR_Cal();

    /* ===== 地面标准控制：无条件每周期执行（FN 检测信号必须活着） ===== */
    chassis_theta_sync_gain_update();
    pid_calculate(theta_pid, leg[LEFT]->theta - leg[RIGHT]->theta, 0);

    float length_target[CHASSIS_SIDE_COUNT] = {
        leg[LEFT]->length_ref, leg[RIGHT]->length_ref
    };
    const uint8_t bump_length_owner = bump_length_target_owner_active();
    if (!bump_length_owner) {
        chassis_safety_module_landing_length_override(
            leg[LEFT]->length_ref, leg[RIGHT]->length_ref,
            &length_target[LEFT], &length_target[RIGHT]);
    }
    float selected_length_target[CHASSIS_SIDE_COUNT];
    bump_length_target_select(bump_length_owner, length_target,
                              selected_length_target);
    bump_length_target_effective[LEFT] =
        selected_length_target[LEFT];
    bump_length_target_effective[RIGHT] =
        selected_length_target[RIGHT];
    Leg_FN_Calculation(0, selected_length_target[LEFT],
                       selected_length_target[RIGHT]);

    /* 离地时只冻结积分，P/D 照算——检测信号活着，积分不污染 */
    chassis_safety_module_freeze_integrators_if_airborne();
    chassis_safety_module_update_ground();

    const uint8_t takeoff_edge = chassis_safety_module_takeoff_edge();
    const uint8_t landing_edge = chassis_safety_module_landing_edge();
    if (takeoff_edge || landing_edge) {
        if (landing_edge) {
            if (!jump_landing_balance_update(0.0f, 0U)) {
                memset(vmc_out_l, 0, sizeof(vmc_out_l));
                memset(vmc_out_r, 0, sizeof(vmc_out_r));
                return;
            }
        }
        if (!jump_ground_recovery_update(0.0f, 0U)) {
            memset(vmc_out_l, 0, sizeof(vmc_out_l));
            memset(vmc_out_r, 0, sizeof(vmc_out_r));
            return;
        }
        update_LQR_obs();
        if (!jump_landing_balance_apply_lqr_refs()) {
            memset(vmc_out_l, 0, sizeof(vmc_out_l));
            memset(vmc_out_r, 0, sizeof(vmc_out_r));
            return;
        }
        LQR_Cal();
    }
    jump_control_observe_ground();

    if (landing_edge && jump_control_active()) {
        jump_landing_tp_handoff[LEFT] = leg[LEFT]->Tp;
        jump_landing_tp_handoff[RIGHT] = leg[RIGHT]->Tp;
        if (!isfinite(jump_landing_tp_handoff[LEFT])) {
            jump_landing_tp_handoff[LEFT] = 0.0f;
        }
        if (!isfinite(jump_landing_tp_handoff[RIGHT])) {
            jump_landing_tp_handoff[RIGHT] = 0.0f;
        }
        jump_landing_tp_handoff_timer = JUMP_LANDING_TP_HANDOFF_TIME;
    }

    const uint8_t bump_pose_owner_now =
        chassis_safety_module_bump_pose_owner_active();
    const uint8_t safety_airborne_override = !bump_pose_owner_now &&
        chassis_safety_module_apply_airborne_control();
    if (!safety_airborne_override) {
        dbg_theta_sync_p = -theta_pid->Output;
        if (jump_state == JUMP_STATE_THRUST) {
            dbg_theta_sync_p = JUMP_THRUST_THETA_DIFF_KP *
                (leg[LEFT]->theta - leg[RIGHT]->theta);
            LIMIT_MIN_MAX(dbg_theta_sync_p,
                          -JUMP_THRUST_THETA_HOLD_LIMIT,
                           JUMP_THRUST_THETA_HOLD_LIMIT);
        }
        uint8_t theta_diff_observer_invalid = 0U;
        const float theta_sync_d =
            chassis_theta_sync_damping_calculate(
                &theta_diff_observer_invalid);
        if (theta_diff_observer_invalid) {
            jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
            memset(vmc_out_l, 0, sizeof(vmc_out_l));
            memset(vmc_out_r, 0, sizeof(vmc_out_r));
            return;
        }
        const float theta_sync_guard =
            chassis_theta_sync_guard_calculate();
        leg[LEFT]->Tp = -LQROutBuf[LEFT][1] +
                        dbg_theta_sync_p + theta_sync_d + theta_sync_guard;
        leg[RIGHT]->Tp = -LQROutBuf[RIGHT][1] -
                         dbg_theta_sync_p - theta_sync_d - theta_sync_guard;

        /* Settle Pitch while ground reaction is still available.  The common
         * hip axis also determines the absolute direction of both leg rods:
         * jump30 had zero release force, yet theta_common crossed from +0.16
         * to -0.14/-0.24 rad before takeoff and the chassis still left with a
         * rearward rate.  Blend from body-Pitch assistance to an absolute
         * theta=0 hold as the rods leave their small vertical window. */
        if (jump_state == JUMP_STATE_THRUST) {
            const float body_pitch_rad = ins.pitch * DEGREE_2_RAD;
            const float body_pitch_rate = -LQRXObsBuf[LEFT][5];
            float rate_filter_alpha = jump_cycle_dt_s /
                (JUMP_THRUST_PITCH_RATE_FILTER_TAU + jump_cycle_dt_s);
            LIMIT_MIN_MAX(rate_filter_alpha, 0.0f, 1.0f);
            jump_thrust_pitch_rate_lpf += rate_filter_alpha *
                (body_pitch_rate - jump_thrust_pitch_rate_lpf);
            const float average_length =
                0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
            float rate_target_gain =
                (average_length - JUMP_THRUST_PITCH_RATE_START_L0) /
                (JUMP_THRUST_PITCH_RATE_FULL_L0 -
                 JUMP_THRUST_PITCH_RATE_START_L0);
            LIMIT_MIN_MAX(rate_target_gain, 0.0f, 1.0f);
            float body_pitch_rate_target =
                JUMP_THRUST_PITCH_RATE_TARGET_ZERO +
                JUMP_THRUST_PITCH_RATE_VX_GAIN * jump_launch_vx;
            LIMIT_MIN_MAX(body_pitch_rate_target,
                          JUMP_THRUST_PITCH_RATE_TARGET_MIN,
                          JUMP_THRUST_PITCH_RATE_TARGET_MAX);
            body_pitch_rate_target *= rate_target_gain;
            float pitch_tp_request =
                -JUMP_THRUST_PITCH_KP * body_pitch_rad -
                 JUMP_THRUST_PITCH_KD *
                    (jump_thrust_pitch_rate_lpf -
                     body_pitch_rate_target);
            LIMIT_MIN_MAX(pitch_tp_request,
                          -JUMP_THRUST_PITCH_TP_LIMIT,
                           JUMP_THRUST_PITCH_TP_LIMIT);
            /* The measured 25-50 Hz Pitch-rate oscillation in jump26 made a
             * raw D command alternate by several Nm every control sample.
             * Keep the useful takeoff bias, but bound its torque derivative. */
            const float pitch_tp_step =
                JUMP_THRUST_PITCH_TP_SLEW_RATE * jump_cycle_dt_s;
            jump_thrust_pitch_tp = move_towards_f32(
                jump_thrust_pitch_tp, pitch_tp_request, pitch_tp_step);
            const float theta_common = 0.5f *
                (leg[LEFT]->theta + leg[RIGHT]->theta);
            const float theta_rate_common = 0.5f *
                (jump_theta_rate_or_zero(
                     LEFT, leg[LEFT]->d_theta_lpf, 1U) +
                 jump_theta_rate_or_zero(
                     RIGHT, leg[RIGHT]->d_theta_lpf, 1U));
            /* Keep the captured rod direction throughout the explosive
             * extension.  Once the timed axial tuck starts, drive the common
             * reference toward the airborne hold angle so the legs are already
             * at the attitude the flight controller will hold.
             *
             * 2026-09-23: this used to slew to theta = 0.0 ("compact legs point
             * down").  All three reference implementations instead hold a
             * NON-zero leg angle in flight (climber v3 holds theta = 0;
             * chassis.7z holds 0.25 rad with joint gain ~80; 2026Climber closes
             * the swing angle with kp 60).  Slewing to 0 removed the body's only
             * reaction surface, and jump14 then measured Pitch falling 31 deg
             * with the reaction wheel pinned at +-8191. */
            if (chassis_safety_module_jump_air_profile_active()) {
                jump_thrust_theta_ref = move_towards_f32(
                    jump_thrust_theta_ref, JUMP_AIR_THETA_REF_RAD,
                    JUMP_PRETUCK_THETA_REF_RATE * jump_cycle_dt_s);
            }
            jump_thrust_theta_ref_rate = 0.0f;
            /* SIGN -- SETTLED BY THE 1:1 SIMULATION (2026-09-23): POSITIVE gain.
             * The 2x2 experiment in /home/kaiser/robomaster/rl-lab (the model
             * this controller was ported from) shows theta -80/-12 produces
             * max|theta| = 5.5 rad and pitch > 150 deg -- exactly the real
             * robot's "legs spin a full turn" failure -- while +80/+12 gives a
             * clean jump with max|theta| = 0.12 rad.  See the longer note in
             * chassis_safety.c.
             *
             * ERROR CLAMP is kept: theta wraps at +-pi, so bounding the demand
             * to the linear band is cheap insurance (the sim's max error is
             * 0.118 rad, so it never bites in normal operation). */
            const float thrust_theta_err =
                theta_common - jump_thrust_theta_ref;
            float thrust_theta_err_clamped = thrust_theta_err;
            LIMIT_MIN_MAX(thrust_theta_err_clamped,
                          -JUMP_THETA_ERR_CLAMP, JUMP_THETA_ERR_CLAMP);
            jump_thrust_theta_hold_tp =
                JUMP_THRUST_THETA_HOLD_KP * thrust_theta_err_clamped +
                JUMP_THRUST_THETA_HOLD_KD *
                    (theta_rate_common - jump_thrust_theta_ref_rate);
            LIMIT_MIN_MAX(jump_thrust_theta_hold_tp,
                          -JUMP_THRUST_THETA_HOLD_LIMIT,
                           JUMP_THRUST_THETA_HOLD_LIMIT);

            const float theta_tracking_error =
                theta_common - jump_thrust_theta_ref;
            float pitch_authority =
                (JUMP_THRUST_THETA_YIELD_END -
                 fabsf(theta_tracking_error)) /
                (JUMP_THRUST_THETA_YIELD_END -
                 JUMP_THRUST_THETA_YIELD_START);
            LIMIT_MIN_MAX(pitch_authority, 0.0f, 1.0f);
            /* With a captured theta reference, small tracking error leaves
             * Pitch/LQR authority available throughout grounded extension.
             * It is removed only after takeoff, where the common hip axis is
             * reserved for vertical-leg hold. */
            const float lqr_common = 0.5f *
                (leg[LEFT]->Tp + leg[RIGHT]->Tp);
            const float lqr_differential = 0.5f *
                (leg[LEFT]->Tp - leg[RIGHT]->Tp);
            const float common_tp =
                pitch_authority *
                    (JUMP_THRUST_THETA_LQR_GAIN * lqr_common +
                     jump_thrust_pitch_tp) +
                jump_thrust_theta_hold_tp;
            leg[LEFT]->Tp = common_tp + lqr_differential;
            leg[RIGHT]->Tp = common_tp - lqr_differential;
        } else {
            jump_thrust_pitch_tp = 0.0f;
            jump_thrust_pitch_rate_lpf = -LQRXObsBuf[LEFT][5];
            jump_thrust_theta_hold_tp = 0.0f;
        }

        if (jump_landing_tp_handoff_timer > 0.0f &&
            !chassis_safety_module_jump_landing_owner_active() &&
            !jump_landing_postbuffer_normal_base_active()) {
            float handoff_gain = jump_landing_tp_handoff_timer /
                JUMP_LANDING_TP_HANDOFF_TIME;
            LIMIT_MIN_MAX(handoff_gain, 0.0f, 1.0f);
            leg[LEFT]->Tp = handoff_gain *
                jump_landing_tp_handoff[LEFT] +
                (1.0f - handoff_gain) * leg[LEFT]->Tp;
            leg[RIGHT]->Tp = handoff_gain *
                jump_landing_tp_handoff[RIGHT] +
                (1.0f - handoff_gain) * leg[RIGHT]->Tp;
            jump_landing_tp_handoff_timer -= jump_cycle_dt_s;
            if (jump_landing_tp_handoff_timer < 0.0f) {
                jump_landing_tp_handoff_timer = 0.0f;
            }
        }

        /* Landing common-theta hold: the ground length loop alone can fold
         * both rods through the phi0 limit.  Hold the angle captured at
         * touchdown with a bounded common hip torque.
         * SIGN (2026-09-23): POSITIVE, matching the airborne and thrust holds.
         * The 1:1 simulation in rl-lab shows the theta loop needs positive gain;
         * the negated variants there reproduce the real robot's leg wrap. */
        if (jump_state == JUMP_STATE_LANDING &&
            !chassis_safety_module_jump_landing_owner_active() &&
            !jump_landing_postbuffer_normal_base_active()) {
            const float theta_common = 0.5f *
                (leg[LEFT]->theta + leg[RIGHT]->theta);
            const float theta_rate_common = 0.5f *
                (jump_theta_rate_or_zero(
                     LEFT, leg[LEFT]->d_theta_lpf, 1U) +
                 jump_theta_rate_or_zero(
                     RIGHT, leg[RIGHT]->d_theta_lpf, 1U));
            float hold_tp =
                JUMP_LANDING_THETA_HOLD_KP *
                    (theta_common - jump_landing_theta_ref) +
                JUMP_LANDING_THETA_HOLD_KD * theta_rate_common;
            LIMIT_MIN_MAX(hold_tp,
                          -JUMP_LANDING_THETA_HOLD_LIMIT,
                           JUMP_LANDING_THETA_HOLD_LIMIT);
            leg[LEFT]->Tp += hold_tp;
            leg[RIGHT]->Tp += hold_tp;
            jump_landing_theta_hold_tp = hold_tp;

            /* Differential hold: the common hold above cannot stop one rod
             * from folding while the other stays put.  Add a bounded
             * left-minus-right PD term.
             *
             * Sign check against the 1:1 simulation (2026-09-23): the sim does
             *   dif = 0.5*kp*(th_l - th_r) + 0.5*kd*(dth_l - dth_r)
             *   l = common + dif ;  r = common - dif
             * Here diff_tp is NEGATED and then applied as (l -= , r += ), i.e.
             *   l += (Kp*theta_diff + Kd*rate_diff) ;  r -= (same)
             * which is the same convention.  The double negation is correct --
             * do not "simplify" it. */
            const float theta_diff =
                leg[LEFT]->theta - leg[RIGHT]->theta;
            const float theta_rate_diff =
                jump_theta_rate_or_zero(
                    LEFT, leg[LEFT]->d_theta_lpf, 1U) -
                jump_theta_rate_or_zero(
                    RIGHT, leg[RIGHT]->d_theta_lpf, 1U);
            float diff_tp =
                -(JUMP_LANDING_THETA_DIFF_KP * theta_diff +
                  JUMP_LANDING_THETA_DIFF_KD * theta_rate_diff);
            LIMIT_MIN_MAX(diff_tp,
                          -JUMP_LANDING_THETA_DIFF_LIMIT,
                           JUMP_LANDING_THETA_DIFF_LIMIT);
            leg[LEFT]->Tp -= diff_tp;
            leg[RIGHT]->Tp += diff_tp;
        } else {
            jump_landing_theta_hold_tp = 0.0f;
        }
    } else {
        jump_thrust_pitch_tp = 0.0f;
        jump_thrust_theta_hold_tp = 0.0f;
        jump_landing_theta_hold_tp = 0.0f;
        dbg_theta_sync_p = 0.0f;
        dbg_theta_sync_d = 0.0f;
        dbg_theta_rate_diff = 0.0f;
        dbg_theta_sync_guard = 0.0f;
    }
    if (chassis_safety_module_bump_pose_owner_active()) {
        if (bump_state == BUMP_STATE_SWING_OVER_BODY ||
            bump_state == BUMP_STATE_RETRACT ||
            bump_state == BUMP_STATE_SWING_SUPPORT) {
            leg[LEFT]->Tp = bump_hip_tp[LEFT];
            leg[RIGHT]->Tp = bump_hip_tp[RIGHT];
        } else if (bump_state == BUMP_STATE_STAND) {
            for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
                leg[side]->Tp = bump_hip_tp[side] + bump_stand_handoff_gain *
                    (leg[side]->Tp - bump_hip_tp[side]);
            }
        }
    }
    jump_air_wheel_control_update();
    bump_wheel_handoff_update();
    jump_landing_wheel_control_update();
    chassis_safety_module_landing_fast_exit_finish_cycle();
    if (!chassis_safety_module_bump_pose_owner_active()) {
        chassis_recovery_module_apply_handoff_blend();
    }

    float owner_gain = 1.0f;
    float wheel_buffer_return_gain = 0.0f;
    jump_landing_base_tp[LEFT] = leg[LEFT]->Tp;
    jump_landing_base_tp[RIGHT] = leg[RIGHT]->Tp;
    if (!chassis_safety_module_bump_pose_owner_active() &&
        chassis_safety_module_jump_landing_owner_active() &&
        !chassis_safety_module_off_ground()) {
        jump_landing_tp_handoff_timer = 0.0f;
        if (!jump_landing_owner_theta_ref_valid) {
            const float first_owner_ref = 0.5f *
                (leg[LEFT]->theta + leg[RIGHT]->theta);
            if (!isfinite(first_owner_ref)) {
                jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
                memset(vmc_out_l, 0, sizeof(vmc_out_l));
                memset(vmc_out_r, 0, sizeof(vmc_out_r));
                return;
            }
            jump_landing_theta_ref = first_owner_ref;
            jump_landing_owner_theta_ref_valid = 1U;
        }
        uint8_t left_valid = 0U;
        uint8_t right_valid = 0U;
        const jump_local_rate_selection_t left_rate =
            jump_theta_rate_select(LEFT, leg[LEFT]->d_theta_lpf, 1U);
        const jump_local_rate_selection_t right_rate =
            jump_theta_rate_select(RIGHT, leg[RIGHT]->d_theta_lpf, 1U);
        const float capture_left = jump_landing_capture_tp(
            leg[LEFT]->theta, jump_landing_theta_ref, left_rate.rate,
            left_rate.valid, JUMP_LANDING_THETA_HOLD_KP,
            JUMP_LANDING_THETA_HOLD_KD, JUMP_LANDING_THETA_HOLD_LIMIT,
            &left_valid);
        const float capture_right = jump_landing_capture_tp(
            leg[RIGHT]->theta, jump_landing_theta_ref, right_rate.rate,
            right_rate.valid, JUMP_LANDING_THETA_HOLD_KP,
            JUMP_LANDING_THETA_HOLD_KD, JUMP_LANDING_THETA_HOLD_LIMIT,
            &right_valid);
        if (!left_valid || !right_valid) {
            jump_control_fail_to_recovery(JUMP_FAULT_SAFETY);
            memset(vmc_out_l, 0, sizeof(vmc_out_l));
            memset(vmc_out_r, 0, sizeof(vmc_out_r));
            return;
        }
        owner_gain = jump_landing_ground_balance_gain;
        LIMIT_MIN_MAX(owner_gain, 0.0f, 1.0f);
        leg[LEFT]->Tp = jump_landing_capture_return_blend(
            capture_left, jump_landing_base_tp[LEFT],
            owner_gain);
        leg[RIGHT]->Tp = jump_landing_capture_return_blend(
            capture_right, jump_landing_base_tp[RIGHT],
            owner_gain);
        /* Axial impact absorption keeps its own buffer. Balance needs its
         * paired hip/wheel LQR as soon as contact is confirmed; waiting for
         * RETURN left the pitch-only damper accelerating a moving touchdown. */
        const float landing_length_diff = fabsf(
            leg[LEFT]->l0 - leg[RIGHT]->l0);
        const uint8_t landing_wheel_return_stable =
            chassis_safety_module_landing_gain() > 0.0f &&
            isfinite(ins.pitch) && isfinite(ins.roll) &&
            fabsf(ins.pitch) <= JUMP_LANDING_OWNER_PITCH_STABLE_DEG &&
            fabsf(ins.roll) <= JUMP_LANDING_OWNER_ROLL_STABLE_DEG &&
            isfinite(chassis_vx_filter) &&
            isfinite(landing_length_diff) &&
            landing_length_diff <= JUMP_LANDING_OWNER_LENGTH_DIFF_MAX;
        wheel_buffer_return_gain = landing_wheel_return_stable ?
            owner_gain : 0.0f;
    }
    jump_landing_owner_gain = owner_gain;
    jump_landing_wheel_buffer_return_gain = wheel_buffer_return_gain;
    /* Publish one complete-cycle allocation scalar for both asynchronous
     * wheel callbacks. A failing/airborne transition invalidates it. */
    if (!jump_ground_wheel_cache_publish()) {
        memset(vmc_out_l, 0, sizeof(vmc_out_l));
        memset(vmc_out_r, 0, sizeof(vmc_out_r));
        return;
    }

    /* 最终 VMC 力矩映射 */
    leg[LEFT]->vmc_cal_T(leg[LEFT], vmc_out_l);
    leg[RIGHT]->vmc_cal_T(leg[RIGHT], vmc_out_r);
}

static void Leg_FN_Calculation(float ROLL_TARGET, float L_TARGET_L, float L_TARGET_R){/*交23年平步开源中对于支持力的计算*/
    const float support_mass = CHASSIS_SUPPORT_MASS; /* reverted 2026-09-22: the 13.592626 kg ruled feedforward stepped +/-7.5 N at jump entry/exit; standing feedforward back to firmware mass pending hardware validation (VMC_SUPPORT_MASS kept for provenance) */
    const uint8_t bump_pose_owner =
        chassis_safety_module_bump_pose_owner_active();
    const uint8_t landing_buffer_active = !bump_pose_owner &&
        chassis_safety_module_landing_buffer_active();
    const uint8_t landing_postbuffer_active =
        !bump_pose_owner && chassis_safety_module_landing_postbuffer_active();

    /* 云台使整车质量变为裸车的约 2.09 倍。低腿长保留已验证增益，中高腿
     * 根据实际腿长连续恢复整车所需的 P/D，避免档位切换时产生控制量阶跃。 */
    const float roll_l0_average =
        0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    const uint8_t roll_spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    const float roll_gain_l0_start = roll_spin_mode ?
        ROLL_GAIN_SCHEDULE_L0_START : ROLL_NORMAL_GAIN_L0_START;
    const float roll_gain_l0_end = roll_spin_mode ?
        ROLL_GAIN_SCHEDULE_L0_END : ROLL_NORMAL_GAIN_L0_END;
    const float roll_gain_target = roll_spin_mode ?
        ROLL_WHOLE_BODY_GAIN_SCALE : ROLL_NORMAL_GAIN_SCALE;
    const uint8_t roll_high_yaw = !roll_spin_mode &&
        roll_high_yaw_profile_active();
    const float roll_base_kp = roll_spin_mode ? roll_Kp :
        (roll_high_yaw ? ROLL_HIGH_YAW_KP : ROLL_NORMAL_KP);
    const float roll_base_kd = roll_spin_mode ? roll_Kd :
        (roll_high_yaw ? ROLL_HIGH_YAW_KD : ROLL_NORMAL_KD);
    const float roll_output_limit = roll_spin_mode ? (float)roll_MaxVal :
        ROLL_NORMAL_MAX_FORCE;
    roll_static_trim_update();
    float roll_pd_headroom = 0.0f;
    if (!roll_spin_mode && isfinite(roll_static_trim_applied)) {
        roll_pd_headroom = fabsf(roll_static_trim_applied);
        LIMIT_MIN_MAX(roll_pd_headroom, 0.0f,
                      ROLL_STATIC_TRIM_NORMAL_APPLY_LIMIT);
    }
    float roll_gain_progress =
        (roll_l0_average - roll_gain_l0_start) /
        (roll_gain_l0_end - roll_gain_l0_start);
    LIMIT_MIN_MAX(roll_gain_progress, 0.0f, 1.0f);
    float roll_gain_scale = 1.0f + roll_gain_progress *
        (roll_gain_target - 1.0f);
    if (roll_spin_mode) {
        float spin_extended_progress =
            (roll_l0_average - ROLL_SPIN_SCHEDULE_L0_LOW) /
            (ROLL_SPIN_SCHEDULE_L0_MID - ROLL_SPIN_SCHEDULE_L0_LOW);
        LIMIT_MIN_MAX(spin_extended_progress, 0.0f, 1.0f);
        roll_gain_scale *= 1.0f + spin_extended_progress *
            (ROLL_SPIN_EXTENDED_GAIN_SCALE - 1.0f);
    }
    roll_pid->Kp = roll_base_kp * roll_gain_scale;
    roll_pid->Kd = roll_base_kd * roll_gain_scale;
    roll_pid->MaxOut = roll_output_limit + roll_pd_headroom;
    const float roll_target = roll_spin_compensation_update(
        ROLL_TARGET, roll_l0_average);
    dbg_roll_pd_output = pid_calculate(roll_pid, ins.roll, roll_target);
    dbg_roll_pre_limit_force = dbg_roll_pd_output +
        roll_static_trim_applied + roll_spin_integral_applied;
    dbg_roll_pre_split_guard = dbg_roll_pre_limit_force;
    LIMIT_MIN_MAX(dbg_roll_pre_split_guard,
                  -roll_output_limit, roll_output_limit);
    F_roll = dbg_roll_pre_split_guard;

    /* Keep symmetric length targets and use the Roll controller directly.
     * A large split remains a diagnostic flag, not a second controller. */
    const float signed_split = leg[LEFT]->l0 - leg[RIGHT]->l0;
    const float actual_split = fabsf(signed_split);
    const float split_limit = leg_split_limit_calculate();
    const uint8_t split_protection_enabled =
        normal_leg_transition_mode() &&
        !chassis_safety_module_off_ground() &&
        !chassis_safety_module_landing_buffer_active();

    if (!split_protection_enabled) {
        leg_split_target_guard_active = 0U;
    } else if (!leg_split_target_guard_active &&
               actual_split > split_limit) {
        leg_split_target_guard_active = 1U;
    } else if (leg_split_target_guard_active &&
               actual_split < split_limit - LEG_SPLIT_RELEASE_HYSTERESIS) {
        leg_split_target_guard_active = 0U;
    }

    roll_geometry_comp_update();
    roll_length_target_l = L_TARGET_L - roll_length_offset;
    roll_length_target_r = L_TARGET_R + roll_length_offset;

    /* 防止差动目标越出五连杆正常工作范围。 */
    const float length_target_min = landing_buffer_active ?
                                    LANDING_LENGTH_REF_MIN :
                                    ROLL_LENGTH_TARGET_MIN;
    const float length_target_max =
                                    (landing_buffer_active ||
                                     bump_control_active() ||
                                     chassis_cmd.bump_mode != 0U) ?
                                    LANDING_LENGTH_REF_MAX :
                                    ROLL_LENGTH_TARGET_MAX;
    LIMIT_MIN_MAX(roll_length_target_l,
                  length_target_min, length_target_max);
    LIMIT_MIN_MAX(roll_length_target_r,
                  length_target_min, length_target_max);

    if (landing_buffer_active || landing_postbuffer_active ||
        jump_control_active()) {
        leg_stuck_compensation_phase_begin();
        leg_stuck_force[LEFT] = 0.0f;
        leg_stuck_force[RIGHT] = 0.0f;
        leg_stuck_post_ramp_timer = 0.0f;
    } else {
        leg_stuck_compensation_update(roll_length_target_l,
                                      roll_length_target_r);
    }

    const float length_error_l = roll_length_target_l - leg[LEFT]->l0;
    const float length_error_r = roll_length_target_r - leg[RIGHT]->l0;
    if (landing_buffer_active || landing_postbuffer_active ||
        chassis_cmd.leg_leng_change == LENGTH_CHANGE ||
        length_error_l * L_length_pid->Iout < 0.0f) {
        L_length_pid->Iout = 0.0f;
        L_length_pid->ITerm = 0.0f;
    }
    if (landing_buffer_active || landing_postbuffer_active ||
        chassis_cmd.leg_leng_change == LENGTH_CHANGE ||
        length_error_r * R_length_pid->Iout < 0.0f) {
        R_length_pid->Iout = 0.0f;
        R_length_pid->ITerm = 0.0f;
    }

    float normal_kp_l = l_length_Kp;
    float normal_kp_r = r_length_Kp;
    float normal_kd_l = l_length_Kd;
    float normal_kd_r = r_length_Kd;
    const uint8_t spin_length_profile =
        roll_spin_mode && !chassis_safety_module_landing_buffer_active();
    if (spin_length_profile) {
        float extended_length_gain_progress =
            (roll_l0_average - ROLL_GAIN_SCHEDULE_L0_START) /
            (ROLL_GAIN_SCHEDULE_L0_END - ROLL_GAIN_SCHEDULE_L0_START);
        LIMIT_MIN_MAX(extended_length_gain_progress, 0.0f, 1.0f);
        normal_kp_l += extended_length_gain_progress *
            (LENGTH_PID_SPIN_EXTENDED_KP - l_length_Kp);
        normal_kp_r += extended_length_gain_progress *
            (LENGTH_PID_SPIN_EXTENDED_KP - r_length_Kp);
        normal_kd_l += extended_length_gain_progress *
            (LENGTH_PID_SPIN_EXTENDED_KD - l_length_Kd);
        normal_kd_r += extended_length_gain_progress *
            (LENGTH_PID_SPIN_EXTENDED_KD - r_length_Kd);
} else if (chassis_cmd.leg_leng_change != LENGTH_CHANGE &&
               !high_leg_geometry_active() && mid_leg_geometry_active()) {
        normal_kd_l = LENGTH_PID_NORMAL_MID_KD;
        normal_kd_r = LENGTH_PID_NORMAL_MID_KD;
    }
    /* v20 (jump103): the released landing buffer hands its length-PID tuning to
     * the post-buffer window, otherwise the loop drops from Kp 650 / Kd 45-110
     * to Kp 450 / Kd 18 about 60 ms after touchdown, right as the leg starts
     * ringing at ~10 Hz (20-40 mm peak-to-peak for ~0.8 s, measured). */
    const uint8_t landing_tuning_active =
        landing_buffer_active || landing_postbuffer_active;
    const uint8_t jump_origin_buffer =
        chassis_safety_module_landing_buffer_jump_origin() ||
        landing_postbuffer_active;
    jump_landing_pid_prepare(L_length_pid, &jump_landing_pid_state[LEFT],
        jump_origin_buffer, leg[LEFT]->l0, roll_length_target_l);
    jump_landing_pid_prepare(R_length_pid, &jump_landing_pid_state[RIGHT],
        jump_origin_buffer, leg[RIGHT]->l0, roll_length_target_r);
    if (chassis_safety_module_landing_fast_exit_take_pid_seed()) {
        /* The buffer ref and landing gains end now. Seed once against the
         * ordinary target to avoid a derivative kick on this handoff sample. */
        jump_landing_pid_seed(L_length_pid, leg[LEFT]->l0,
                              roll_length_target_l);
        jump_landing_pid_seed(R_length_pid, leg[RIGHT]->l0,
                              roll_length_target_r);
    }
    if (landing_tuning_active) {
        /* Buffer live: the RETURN-phase blend owns the handoff.  Buffer already
         * released: blend out with the post-buffer reference fade so the gains
         * return to the standing values without a step. */
        const float return_gain = landing_buffer_active ?
            chassis_safety_module_landing_pid_return_gain() :
            1.0f - chassis_safety_module_landing_postbuffer_gain();
        const float landing_kd =
            (chassis_cmd.leg_level == LEG_LOW) ?
            (jump_origin_buffer ? JUMP_LANDING_LENGTH_KD_LOW :
                                  LANDING_LENGTH_KD_LOW) :
            LANDING_LENGTH_KD;
        float landing_kd_l = landing_kd;
        float landing_kd_r = landing_kd;
        if (chassis_cmd.leg_level == LEG_LOW) {
            const float low_leg_normal_kd = jump_origin_buffer ?
                JUMP_LANDING_LENGTH_KD_LOW : LANDING_LENGTH_KD_LOW;
            landing_kd_l = jump_landing_pid_select_kd(
                jump_origin_buffer, 1U, low_leg_normal_kd,
                LANDING_LENGTH_KD_LOW_REBOUND, leg[LEFT]->d_l0_lpf);
            landing_kd_r = jump_landing_pid_select_kd(
                jump_origin_buffer, 1U, low_leg_normal_kd,
                LANDING_LENGTH_KD_LOW_REBOUND, leg[RIGHT]->d_l0_lpf);
        }
        const float landing_output_limit =
            (chassis_cmd.leg_level == LEG_LOW) ?
            LANDING_LENGTH_OUTPUT_LIMIT_LOW :
            LANDING_LENGTH_OUTPUT_LIMIT;
        L_length_pid->Kp = LANDING_LENGTH_KP + return_gain *
            (normal_kp_l - LANDING_LENGTH_KP);
        R_length_pid->Kp = LANDING_LENGTH_KP + return_gain *
            (normal_kp_r - LANDING_LENGTH_KP);
        L_length_pid->Ki = LANDING_LENGTH_KI;
        R_length_pid->Ki = LANDING_LENGTH_KI;
        L_length_pid->Kd = landing_kd_l + return_gain *
            (normal_kd_l - landing_kd_l);
        R_length_pid->Kd = landing_kd_r + return_gain *
            (normal_kd_r - landing_kd_r);
        L_length_pid->MaxOut = landing_output_limit;
        R_length_pid->MaxOut = landing_output_limit;
    } else if (bump_action_active()) {
        /* BUMP 使用独立的腿长 PID 档位；不进入 jump_state 或 LQR 档位索引。 */
        L_length_pid->Kp = BUMP_LENGTH_KP;
        R_length_pid->Kp = BUMP_LENGTH_KP;
        L_length_pid->Ki = 0.0f;
        R_length_pid->Ki = 0.0f;
        L_length_pid->Kd = BUMP_LENGTH_KD;
        R_length_pid->Kd = BUMP_LENGTH_KD;
        const float bump_length_output_limit =
            bump_state == BUMP_STATE_RETRACT ?
            BUMP_RETRACT_SUSTAIN_FORCE_LIMIT : BUMP_LENGTH_OUTPUT_LIMIT;
        L_length_pid->MaxOut = bump_length_output_limit;
        R_length_pid->MaxOut = bump_length_output_limit;
    } else if (jump_control_active()) {
        /* 2026-09-23 item 3: the THRUST state now spans both the powered
         * extension (jump_thrust_active) and the tuck release, so select the
         * PID by the impulse flag rather than by the state alone. */
        const uint8_t thrust_phase = jump_state == JUMP_STATE_THRUST;
        const uint8_t release_phase = thrust_phase && !jump_thrust_active;
        L_length_pid->Kp = release_phase ? JUMP_LENGTH_KP_RELEASE :
            (thrust_phase ? JUMP_LENGTH_KP_THRUST :
                            JUMP_LENGTH_KP_PRELOAD);
        R_length_pid->Kp = L_length_pid->Kp;
        L_length_pid->Ki = 0.0f;
        R_length_pid->Ki = 0.0f;
        L_length_pid->Kd = release_phase ? JUMP_LENGTH_KD_RELEASE :
            (thrust_phase ? JUMP_LENGTH_KD_THRUST :
                            JUMP_LENGTH_KD_PRELOAD);
        R_length_pid->Kd = L_length_pid->Kd;
        L_length_pid->MaxOut = release_phase ? JUMP_LENGTH_LIMIT_RELEASE :
            (thrust_phase ? JUMP_LENGTH_LIMIT_THRUST :
                            JUMP_LENGTH_LIMIT_PRELOAD);
        R_length_pid->MaxOut = L_length_pid->MaxOut;
    } else {
        L_length_pid->Kp = normal_kp_l;
        R_length_pid->Kp = normal_kp_r;
        L_length_pid->Ki = l_length_Ki;
        R_length_pid->Ki = r_length_Ki;
        L_length_pid->Kd = normal_kd_l;
        R_length_pid->Kd = normal_kd_r;
        L_length_pid->MaxOut = l_length_MaxVal;
        R_length_pid->MaxOut = r_length_MaxVal;
    }

    if (jump_origin_buffer) {
        F_l_L = jump_landing_pid_apply_external_rate(
            L_length_pid, &jump_landing_pid_state[LEFT],
            leg[LEFT]->l0, roll_length_target_l, L_length_pid->Kd,
            jump_landing_length_rate[LEFT],
            jump_landing_length_rate_valid) + leg_stuck_force[LEFT];
        F_l_R = jump_landing_pid_apply_external_rate(
            R_length_pid, &jump_landing_pid_state[RIGHT],
            leg[RIGHT]->l0, roll_length_target_r, R_length_pid->Kd,
            jump_landing_length_rate[RIGHT],
            jump_landing_length_rate_valid) + leg_stuck_force[RIGHT];
    } else {
        F_l_L = pid_calculate(L_length_pid, leg[LEFT]->l0,
                              roll_length_target_l) +
                leg_stuck_force[LEFT];
        F_l_R = pid_calculate(R_length_pid, leg[RIGHT]->l0,
                              roll_length_target_r) +
                leg_stuck_force[RIGHT];
    }

    /* Jump takeoff needs a dedicated differential channel.  Two independent
     * length PIDs already react to a split, but the measured right leg leads
     * the left by 12-21 mm during the first part of thrust.  Add a small,
     * explicitly limited cross-coupled correction: a longer/faster right leg
     * increases left force and removes the same amount from the right. */
    jump_length_sync_force = 0.0f;
    if (jump_state == JUMP_STATE_THRUST &&
        !chassis_safety_module_off_ground()) {
        const float length_diff = leg[RIGHT]->l0 - leg[LEFT]->l0;
        const float length_rate_diff =
            jump_length_rate(RIGHT) - jump_length_rate(LEFT);
        jump_length_sync_force =
            JUMP_LENGTH_SYNC_KP * length_diff +
            JUMP_LENGTH_SYNC_KD * length_rate_diff;
        LIMIT_MIN_MAX(jump_length_sync_force,
                      -JUMP_LENGTH_SYNC_LIMIT,
                       JUMP_LENGTH_SYNC_LIMIT);
        F_l_L += jump_length_sync_force;
        F_l_R -= jump_length_sync_force;
    } else if (jump_state == JUMP_STATE_LANDING &&
               landing_buffer_active &&
               !chassis_safety_module_off_ground()) {
        /* Touchdown data show the right leg compressing roughly 2-4 times
         * faster than the left.  A velocity-dominant differential channel
         * brakes the fast side and asks the slow side to absorb more, while
         * keeping the common landing support unchanged. */
        const float length_diff = leg[RIGHT]->l0 - leg[LEFT]->l0;
        const float length_rate_diff =
            jump_landing_length_rate_valid ?
            jump_landing_length_rate[RIGHT] -
                jump_landing_length_rate[LEFT] : 0.0f;
        /* Fade the differential impact damper with the same bumpless handoff
         * used by the landing length PID.  Keep the P+D law direct so the
         * measured rate term cannot be left behind after it changes sign. */
        jump_length_sync_force = jump_landing_sync_force_calculate(
            length_diff, length_rate_diff,
            JUMP_LANDING_SYNC_KP, JUMP_LANDING_SYNC_KD,
            1.0f - chassis_safety_module_landing_pid_return_gain(),
            JUMP_LANDING_SYNC_LIMIT);
        F_l_L += jump_length_sync_force;
        F_l_R -= jump_length_sync_force;
    }
    if (landing_buffer_active) {
        const float landing_output_limit =
            (chassis_cmd.leg_level == LEG_LOW) ?
            LANDING_LENGTH_OUTPUT_LIMIT_LOW :
            LANDING_LENGTH_OUTPUT_LIMIT;
        LIMIT_MIN_MAX(F_l_L, -landing_output_limit,
                      landing_output_limit);
        LIMIT_MIN_MAX(F_l_R, -landing_output_limit,
                      landing_output_limit);
    }

    /* Preserve the proven static support baseline.  Dynamic feedforward is
     * added below only as a bounded correction, so recovery handoff cannot
     * create an axial-force step. */
    F_bl_gravity = 0.5f * support_mass * g;

    leg[LEFT]->F_Spring_to_F_Vertical(leg[LEFT]);
    leg[RIGHT]->F_Spring_to_F_Vertical(leg[RIGHT]);

    const float legacy_support_l = F_bl_gravity - leg[LEFT]->F_Vertical;
    const float legacy_support_r = F_bl_gravity - leg[RIGHT]->F_Vertical;
    float gravity_projected_l = F_bl_gravity * arm_cos_f32(leg[LEFT]->theta);
    float gravity_projected_r = F_bl_gravity * arm_cos_f32(leg[RIGHT]->theta);
    if (gravity_projected_l < 0.0f) gravity_projected_l = 0.0f;
    if (gravity_projected_r < 0.0f) gravity_projected_r = 0.0f;

    const float support_ff_raw_l =
        gravity_projected_l - leg[LEFT]->F_Vertical;
    const float support_ff_raw_r =
        gravity_projected_r - leg[RIGHT]->F_Vertical;
    const uint8_t support_ff_active =
        normal_leg_transition_mode() &&
        chassis_cmd.leg_leng_change != LENGTH_CHANGE &&
        !bump_control_active() &&
        !chassis_safety_module_off_ground() &&
        !landing_buffer_active &&
        !landing_postbuffer_active &&
        !chassis_safety_module_wheel_shut();
    const float support_ff_l = support_force_ff_update(
        LEFT, support_ff_raw_l, legacy_support_l,
        chassis_dt, support_ff_active);
    const float support_ff_r = support_force_ff_update(
        RIGHT, support_ff_raw_r, legacy_support_r,
        chassis_dt, support_ff_active);

    /* Predict lateral acceleration from commanded translation and yaw.  Spin
     * has zero translational command, so its small wheel-speed common mode must
     * not be interpreted as circular motion: Page 50 showed that this estimate
     * generated feedforward in the same direction as the measured roll bias.
     * Roll feedback remains active during Spin.  h/(2*Rl) converts body moment
     * to per-side force. */
    const float lateral_dt = jump_cycle_dt_s;
    const float lateral_coast_dt =
        jump_cycle_dt_first_sample ? lateral_dt :
        jump_cycle_raw_dt_ms * 0.001f;
    const uint8_t lateral_ff_active =
        normal_leg_transition_mode() &&
        chassis_cmd.leg_leng_change != LENGTH_CHANGE &&
        !bump_control_active() &&
        !chassis_safety_module_off_ground() &&
        !landing_buffer_active &&
        !landing_postbuffer_active &&
        !chassis_safety_module_wheel_shut();
    if (!lateral_ff_active) {
        dbg_lateral_ff_target = 0.0f;
        dbg_lateral_feedback_mode_allowed = 0U;
        dbg_lateral_observer_ready = 0U;
        dbg_lateral_feedback_fresh = 0U;
        F_bl_intertial = 0.0f;
        lateral_inertial_ff_reset(&lateral_ff_state);
    } else {
        const uint8_t spin_mode =
            chassis_cmd.ctrl_mode == CHASSIS_SPIN;
        const float lateral_vx = spin_mode ?
            0.0f : LQRXRefBuf[LEFT][3];
        const float lateral_yaw_rate = dbg_yaw_rate_ref;
        const float lateral_accel_ref = lateral_vx * lateral_yaw_rate;
        const float actual_yaw_rate = YAW_GYRO_SIGN * ins.gyro[2];
        dbg_lateral_observer_ready =
            kin_rate_observer[LEFT].sample_count >= KIN_RATE_WINDOW_SIZE &&
            kin_rate_observer[RIGHT].sample_count >= KIN_RATE_WINDOW_SIZE &&
            isfinite(kin_rate_observer[LEFT].theta_rate) &&
            isfinite(kin_rate_observer[RIGHT].theta_rate) &&
            isfinite(kin_rate_observer[LEFT].l0_rate) &&
            isfinite(kin_rate_observer[RIGHT].l0_rate);
        dbg_lateral_feedback_mode_allowed =
            (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
             chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL) &&
            !bump_control_active() && chassis_cmd.bump_mode == 0U &&
            !jump_control_active() && !jump_motion_vx_ref_state.active &&
            !jump_kf_airborne_hold_latched && !jump_kf_reacquire_active &&
            !chassis_safety_module_landing_airborne_guard_active() &&
            chassis_fdb_data.stand_state != CHASSIS_IS_DANGER;
        dbg_lateral_feedback_fresh = lateral_ff_feedback_fresh();
        const uint8_t lateral_feedback_valid =
            dbg_lateral_feedback_mode_allowed && dbg_lateral_observer_ready &&
            dbg_lateral_feedback_fresh && isfinite(chassis_vx_filter) &&
            isfinite(actual_yaw_rate);
        const float lateral_accel_command = lateral_inertial_ff_step(
            &lateral_ff_state, lateral_accel_ref, lateral_vx,
            lateral_yaw_rate, chassis_vx_filter, actual_yaw_rate,
            lateral_coast_dt, lateral_ff_active, lateral_feedback_valid);
        const float body_height =
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        if (!isfinite(body_height) || body_height <= 0.0f ||
            !isfinite(support_mass) || !isfinite(Rl) || Rl <= 0.0f ||
            !isfinite(lateral_accel_command)) {
            dbg_lateral_ff_target = 0.0f;
        } else if (fabsf(lateral_accel_command) <
                   LATERAL_INERTIAL_FF_ACCEL_DB) {
            dbg_lateral_ff_target = 0.0f;
        } else {
            const float lateral_ff_gain =
                lateral_inertial_ff_gain_calculate();
            dbg_lateral_ff_target = lateral_ff_gain * support_mass *
                lateral_accel_command * body_height / (2.0f * Rl);
            if (!isfinite(dbg_lateral_ff_target)) {
                dbg_lateral_ff_target = 0.0f;
            } else {
                LIMIT_MIN_MAX(dbg_lateral_ff_target,
                              -LATERAL_INERTIAL_FF_LIMIT,
                               LATERAL_INERTIAL_FF_LIMIT);
            }
        }
        F_bl_intertial = move_towards_f32(
            F_bl_intertial, dbg_lateral_ff_target,
            LATERAL_INERTIAL_FF_SLEW_RATE * lateral_dt);
    }

    /* Positive vx*yaw produced positive corrective F_roll in the verified
     * vehicle traces, so the commanded feedforward uses the same sign. */
    const uint8_t roll_feedback_priority_normal_ground =
        roll_feedback_priority_scope_active();
    F_lateral = roll_feedback_priority_lateral_force(
        F_roll, F_bl_intertial, roll_feedback_priority_normal_ground);


    const float ordinary_support_l =
        F_lateral + F_l_L + legacy_support_l + support_ff_l;
    const float ordinary_support_r =
        -F_lateral + F_l_R + legacy_support_r + support_ff_r;
    if (bump_pose_owner) {
        const float spring_cancel_l = bump_spring_cancel_force(
            leg[LEFT]->F_Vertical, BUMP_SPRING_COMP_LIMIT);
        const float spring_cancel_r = bump_spring_cancel_force(
            leg[RIGHT]->F_Vertical, BUMP_SPRING_COMP_LIMIT);
        const float stand_force_gain = bump_state == BUMP_STATE_STAND ?
            bump_stand_support_gain : 0.0f;
        float bump_support_l = F_l_L + spring_cancel_l;
        float bump_support_r = F_l_R + spring_cancel_r;
        const uint8_t stand_lower_hold =
            bump_stand_lower_hold_pending(
                (uint8_t)(bump_state == BUMP_STATE_STAND),
                chassis_safety_module_off_ground(),
                bump_stand_handoff.support_started,
                bump_stand_support_gain);
        if (bump_state == BUMP_STATE_RETRACT ||
            bump_state == BUMP_STATE_SWING_SUPPORT ||
            stand_lower_hold) {
            const float pulse_l = bump_retract_breakaway_active[LEFT] ?
                bump_retract_breakaway_force(
                    bump_retract_breakaway_elapsed[LEFT], leg[LEFT]->l0) :
                    0.0f;
            const float pulse_r = bump_retract_breakaway_active[RIGHT] ?
                bump_retract_breakaway_force(
                    bump_retract_breakaway_elapsed[RIGHT], leg[RIGHT]->l0) :
                    0.0f;
            if (pulse_l > 0.0f) {
                if (bump_retract_breakaway_handoff[LEFT]) {
                    LIMIT_MIN_MAX(bump_support_l,
                                  -BUMP_LENGTH_OUTPUT_LIMIT,
                                  BUMP_RETRACT_EXTEND_LIMIT);
                    bump_support_l = bump_retract_breakaway_handoff_force(
                        -pulse_l, bump_support_l,
                        bump_retract_breakaway_handoff_elapsed[LEFT],
                        BUMP_RETRACT_BREAKAWAY_HANDOFF_TIME);
                } else {
                    LIMIT_MIN_MAX(bump_support_l,
                                  -pulse_l, BUMP_RETRACT_EXTEND_LIMIT);
                    if (bump_support_l > -pulse_l) {
                        bump_support_l = -pulse_l;
                    }
                }
            }
            if (pulse_r > 0.0f) {
                if (bump_retract_breakaway_handoff[RIGHT]) {
                    LIMIT_MIN_MAX(bump_support_r,
                                  -BUMP_LENGTH_OUTPUT_LIMIT,
                                  BUMP_RETRACT_EXTEND_LIMIT);
                    bump_support_r = bump_retract_breakaway_handoff_force(
                        -pulse_r, bump_support_r,
                        bump_retract_breakaway_handoff_elapsed[RIGHT],
                        BUMP_RETRACT_BREAKAWAY_HANDOFF_TIME);
                } else {
                    LIMIT_MIN_MAX(bump_support_r,
                                  -pulse_r, BUMP_RETRACT_EXTEND_LIMIT);
                    if (bump_support_r > -pulse_r) {
                        bump_support_r = -pulse_r;
                    }
                }
            }
            bump_support_l = bump_retract_lower_latched_hold_force(
                bump_retract_lower_latched[LEFT],
                bump_retract_one_way_force(
                    leg[LEFT]->l0, bump_support_l,
                    BUMP_RETRACT_MIN_STABLE_LENGTH),
                BUMP_RETRACT_LOWER_HOLD_FORCE);
            bump_support_r = bump_retract_lower_latched_hold_force(
                bump_retract_lower_latched[RIGHT],
                bump_retract_one_way_force(
                    leg[RIGHT]->l0, bump_support_r,
                    BUMP_RETRACT_MIN_STABLE_LENGTH),
                BUMP_RETRACT_LOWER_HOLD_FORCE);
            bump_support_l = bump_retract_sustain_force_floor(
                leg[LEFT]->l0, bump_support_l,
                BUMP_RETRACT_MIN_STABLE_LENGTH,
                BUMP_RETRACT_LOWER_HOLD_FORCE);
            bump_support_r = bump_retract_sustain_force_floor(
                leg[RIGHT]->l0, bump_support_r,
                BUMP_RETRACT_MIN_STABLE_LENGTH,
                BUMP_RETRACT_LOWER_HOLD_FORCE);
            bump_support_l = bump_force_with_torque_reserve(
                leg[LEFT], bump_support_l, bump_hip_tp[LEFT],
                bump_retract_breakaway_active[LEFT] ?
                    BUMP_RETRACT_BREAKAWAY_TORQUE_RESERVE :
                    BUMP_RETRACT_TORQUE_RESERVE);
            bump_support_r = bump_force_with_torque_reserve(
                leg[RIGHT], bump_support_r, bump_hip_tp[RIGHT],
                bump_retract_breakaway_active[RIGHT] ?
                    BUMP_RETRACT_BREAKAWAY_TORQUE_RESERVE :
                    BUMP_RETRACT_TORQUE_RESERVE);
            /* The allocator temporarily clears Tp while evaluating the force
             * Jacobian; restore the requested hip command for the final VMC. */
            leg[LEFT]->Tp = bump_hip_tp[LEFT];
            leg[RIGHT]->Tp = bump_hip_tp[RIGHT];
            /* Keep diagnostics aligned with the actual BUMP force after the
             * dead-point pulse and torque allocation. */
            F_l_L = bump_support_l;
            F_l_R = bump_support_r;
        }
        leg[LEFT]->support_force = bump_support_force_blend(
            bump_support_l, ordinary_support_l, stand_force_gain);
        leg[RIGHT]->support_force = bump_support_force_blend(
            bump_support_r, ordinary_support_r, stand_force_gain);
        /* Apply the safety hold after the ordinary-force blend.  At STAND
         * entry support_gain is deliberately zero and bump9 still reports
         * off_ground, so allowing the blended command through here would
         * step from the -110 N lower-stop hold to an outward command. */
        const uint8_t stand_off_ground =
            chassis_safety_module_off_ground();
        leg[LEFT]->support_force = bump_stand_lower_hold_force(
            (uint8_t)(bump_state == BUMP_STATE_STAND),
            bump_retract_lower_latched[LEFT], stand_off_ground,
            bump_stand_support_gain, bump_stand_extend_gain,
            leg[LEFT]->support_force,
            BUMP_RETRACT_LOWER_HOLD_FORCE);
        leg[RIGHT]->support_force = bump_stand_lower_hold_force(
            (uint8_t)(bump_state == BUMP_STATE_STAND),
            bump_retract_lower_latched[RIGHT], stand_off_ground,
            bump_stand_support_gain, bump_stand_extend_gain,
            leg[RIGHT]->support_force,
            BUMP_RETRACT_LOWER_HOLD_FORCE);
        leg[LEFT]->support_force = bump_stand_short_leg_force_limit(
            (uint8_t)(bump_state == BUMP_STATE_STAND),
            bump_retract_lower_latched[LEFT], leg[LEFT]->l0,
            leg[LEFT]->support_force, BUMP_STAND_SHORT_LEG_RELEASE_M,
            BUMP_STAND_SHORT_LEG_FORCE_LIMIT);
        leg[RIGHT]->support_force = bump_stand_short_leg_force_limit(
            (uint8_t)(bump_state == BUMP_STATE_STAND),
            bump_retract_lower_latched[RIGHT], leg[RIGHT]->l0,
            leg[RIGHT]->support_force, BUMP_STAND_SHORT_LEG_RELEASE_M,
            BUMP_STAND_SHORT_LEG_FORCE_LIMIT);
    } else {
        leg[LEFT]->support_force = ordinary_support_l;
        leg[RIGHT]->support_force = ordinary_support_r;
    }
    LIMIT_MIN_MAX(leg[LEFT]->support_force, -FORCE_LIMIT, FORCE_LIMIT);
    LIMIT_MIN_MAX(leg[RIGHT]->support_force, -FORCE_LIMIT, FORCE_LIMIT);

    /* A short, common axial pulse supplies takeoff energy and contains no
     * wheel torque.  Near full extension the length PID starts braking before
     * contact disappears, so keep a bounded minimum support through the
     * active impulse window instead of letting the launch force collapse.
     * 2026-09-23 item 3: both the boost and that floor now fade with travel
     * (see jump_thrust_force_scale), so the extension cannot arrive at the
     * mechanical stop with the full launch force still applied. */
    if (jump_control_active() && jump_force_boost > 0.0f) {
        const float average_length =
            0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
        const float thrust_scale = jump_thrust_force_scale(average_length);
        const float support_floor =
            JUMP_THRUST_SUPPORT_FLOOR * thrust_scale;

        leg[LEFT]->support_force += jump_force_boost;
        leg[RIGHT]->support_force += jump_force_boost;
        if (leg[LEFT]->support_force < support_floor) {
            leg[LEFT]->support_force = support_floor;
        }
        if (leg[RIGHT]->support_force < support_floor) {
            leg[RIGHT]->support_force = support_floor;
        }
        LIMIT_MIN_MAX(leg[LEFT]->support_force,
                      -FORCE_LIMIT, FORCE_LIMIT);
        LIMIT_MIN_MAX(leg[RIGHT]->support_force,
                      -FORCE_LIMIT, FORCE_LIMIT);
    }

    /* 2026-09-23 item 4: bounded Roll differential PD for the whole jump.
     * It only moves force between the two legs, so it cannot change the total
     * support or the jump height; it just stops one side from lagging during
     * the launch and the landing impact.  The normal-mode Roll path is disabled
     * inside jump (see roll_static_trim / F_roll scheduling), so this is the
     * only Roll channel active there.
     * The launch momentum term uses measured local leg rates, which become
     * zero when the shared observer is stale. */
    jump_roll_diff_force = 0.0f;
    if (jump_control_active() && !chassis_safety_module_landing_buffer_active()) {
        float roll_error = ins.roll;
        if (fabsf(roll_error) < JUMP_ROLL_DIFF_DEADBAND_DEG) {
            roll_error = 0.0f;
        } else {
            roll_error -= (roll_error > 0.0f) ?
                JUMP_ROLL_DIFF_DEADBAND_DEG : -JUMP_ROLL_DIFF_DEADBAND_DEG;
        }
        /* INS publishes roll=-QEKF Pitch and gyro Y without a sign change.
         * Near upright, roll_dot=-gyro[1]; gyro[0] belongs to pitch. */
        const float roll_rate_rad_s = -ins.gyro[1] +
            (jump_state == JUMP_STATE_THRUST ?
                JUMP_ROLL_MOMENTUM_RATE_GAIN *
                    (jump_length_rate(RIGHT) -
                     JUMP_ROLL_LEFT_INERTIA_RATIO * jump_length_rate(LEFT)) :
                0.0f);
        jump_roll_diff_force =
            JUMP_ROLL_DIFF_KP * roll_error +
            JUMP_ROLL_DIFF_KD * roll_rate_rad_s;
        LIMIT_MIN_MAX(jump_roll_diff_force,
                      -JUMP_ROLL_DIFF_LIMIT, JUMP_ROLL_DIFF_LIMIT);
        leg[LEFT]->support_force -= jump_roll_diff_force;
        leg[RIGHT]->support_force += jump_roll_diff_force;
        LIMIT_MIN_MAX(leg[LEFT]->support_force,
                      -FORCE_LIMIT, FORCE_LIMIT);
        LIMIT_MIN_MAX(leg[RIGHT]->support_force,
                      -FORCE_LIMIT, FORCE_LIMIT);
    }

    /* The thrust pulse owns grounded extension.  As soon as it ends, the
     * prestarted air controller below replaces these temporary zero commands
     * in the same cycle; keeping the raw ground-estimator input unloaded also
     * lets its filtered takeoff decision converge instead of feeding back the
     * deliberate retract force as contact. */
    if (jump_state == JUMP_STATE_THRUST &&
        !chassis_safety_module_off_ground()) {
        if (jump_thrust_active) {
            if (leg[LEFT]->support_force < 0.0f) {
                leg[LEFT]->support_force = 0.0f;
            }
            if (leg[RIGHT]->support_force < 0.0f) {
                leg[RIGHT]->support_force = 0.0f;
            }
        } else {
            leg[LEFT]->support_force = 0.0f;
            leg[RIGHT]->support_force = 0.0f;
        }
    }

    /* Blend the final normal support candidate from the last landing-buffer
     * command. This is the virtual support command (gas compensation already
     * accounted for), so it must not be gas-compensated a second time. */
    if (!bump_pose_owner) {
        chassis_safety_module_landing_fast_exit_apply_support(
            jump_cycle_dt_s, &leg[LEFT]->support_force,
            &leg[RIGHT]->support_force);
    }

    /* 保存 raw 支持力（在任何离地覆盖之前），供 Ground_Detect 和 Page6 使用 */
    chassis_safety_module_capture_raw_support(
        leg[LEFT]->support_force, leg[RIGHT]->support_force);

    /* 落地电机缓冲力：确认接地后的对称共模制动，直接叠加到轴向支持力上。
     * 必须在 raw support 抓取之后（不能让地检看到它）、VMC 力矩映射之前。
     * 气弹簧只能吸收一部分冲击，两个独立的腿长环还会在触地瞬间给出反向
     * 命令；这一项只推不拉、左右同值，因此不会引入差模/roll 力矩。 */
    if (!bump_pose_owner) {
        chassis_safety_module_landing_cushion_apply(
            jump_cycle_dt_s, &leg[LEFT]->support_force,
            &leg[RIGHT]->support_force);
    }
}


#endif /* BSP_CHASSIS_LEG_MODE */
static int16_t set_l,set_r;/*观测用*/

static float jump_ground_wheel_common_select(float legacy_common)
{
    if (jump_ground_recovery_scope_active &&
        jump_landing_wheel_handoff_mode_allowed() &&
        !chassis_safety_module_off_ground() &&
        jump_ground_wheel_cache_valid) {
        return jump_ground_wheel_common_cache;
    }
    return legacy_common;
}

static float yaw_lqr_local_residual_calculate(float lqr_left,
                                              float lqr_right)
{
    const float raw = 0.5f * (lqr_right - lqr_left);
    float applied = 0.0f;

#if YAW_LQR_LOCAL_RESIDUAL_ENABLE
    const float l0_average =
        0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0);
    float progress =
        (l0_average - YAW_LQR_LOCAL_RESIDUAL_L0_START) /
        (YAW_LQR_LOCAL_RESIDUAL_L0_END -
         YAW_LQR_LOCAL_RESIDUAL_L0_START);
    LIMIT_MIN_MAX(progress, 0.0f, 1.0f);

    const float gain = progress * progress * (3.0f - 2.0f * progress);
    applied = raw * gain;
    LIMIT_MIN_MAX(applied,
                  -YAW_LQR_LOCAL_RESIDUAL_LIMIT,
                   YAW_LQR_LOCAL_RESIDUAL_LIMIT);
#endif

    dbg_lqr_local_residual_raw = raw;
    dbg_lqr_local_residual_applied = applied;
    return applied;
}

static uint8_t yaw_priority_should_override_lqr(void)
{
    float lqr_left = LQROutBuf[LEFT][0];
    float lqr_right = LQROutBuf[RIGHT][0];
    LIMIT_MIN_MAX(lqr_left, -M3508_TOR_MAX, M3508_TOR_MAX);
    LIMIT_MIN_MAX(lqr_right, -M3508_TOR_MAX, M3508_TOR_MAX);
    const float lqr_common = jump_ground_wheel_common_select(
        0.5f * (lqr_left + lqr_right));
    const float vx_fdb_deadband =
        chassis_cmd.ctrl_mode == CHASSIS_SPIN ?
            YAW_PRIORITY_SPIN_VX_FDB_DB : YAW_PRIORITY_VX_FDB_DB;

    /* 原地转向时防止静摩擦和微小平衡量吃掉一侧 yaw；
     * 一旦存在直行意图或较大的 LQR 共模，必须保留正常叠加关系。 */
    return YAW_PRIORITY_ENABLE &&
           (yaw_ctrl_state == YAW_RATE_MODE) &&
           (fabsf(dbg_yaw_rate_ref) > YAW_RATE_REF_DEADBAND) &&
           (fabsf(chassis_cmd.vx) <= YAW_PRIORITY_VX_CMD_DB) &&
           (fabsf(chassis_cmd.vx_set) <= YAW_PRIORITY_VX_CMD_DB) &&
           (fabsf(chassis_vx_filter) <= vx_fdb_deadband) &&
           (fabsf(lqr_common) <= YAW_PRIORITY_LQR_COMMON_MAX);
}

static float yaw_priority_mix(float lqr_cmd, float yaw_cmd, uint8_t enable)
{
    if (!enable || fabsf(yaw_cmd) < 1e-4f) {
        return lqr_cmd + yaw_cmd;
    }

    float yaw_sign = (yaw_cmd >= 0.0f) ? 1.0f : -1.0f;
    float lqr_use = lqr_cmd;

    /* LQR 和 yaw 反向时，限制 LQR 抵消量 */
    if (lqr_cmd * yaw_cmd < 0.0f) {
        float max_counter = fabsf(yaw_cmd) * YAW_LQR_COUNTER_RATIO_MAX;
        LIMIT_MIN_MAX(lqr_use, -max_counter, max_counter);
    }

    float net = lqr_use + yaw_cmd;

    /* 保证沿 yaw 方向有最小净输出 */
    if (net * yaw_sign < YAW_WHEEL_NET_MIN) {
        net = yaw_sign * YAW_WHEEL_NET_MIN;
    }

    return net;
}

static int16_t jump_landing_wheel_handoff_output(int16_t normal_current,
                                                  chassis_side_e side,
                                                  uint8_t reaction_admitted)
{
    const uint8_t owner_active =
        chassis_safety_module_jump_landing_owner_active();
    const uint8_t buffer_owner_active = owner_active &&
        chassis_safety_module_landing_buffer_active();
    const uint8_t mode_allowed =
        jump_landing_wheel_handoff_mode_allowed();
    const uint8_t off_ground = chassis_safety_module_off_ground();
    uint8_t output_source = JUMP_LANDING_WHEEL_SOURCE_NORMAL;
    float used_gain = 1.0f;
    float normal_command = (float)normal_current;
    float output;
    int16_t output_current;
    const uint8_t buffer_return_blend_enabled =
        buffer_owner_active && mode_allowed && !off_ground &&
        chassis_safety_module_landing_buffer_jump_origin() &&
        /* Recheck contact in the asynchronous callback: a completed-cycle
         * beta must never survive loss of ground/contact ownership. */
        chassis_safety_module_landing_gain() > 0.0f;

    if (off_ground && reaction_admitted) {
        /* The landing owner may pre-arm before contact, but it cannot suppress
         * the bounded reaction-wheel command while the chassis is airborne. */
        jump_landing_wheel_last_used_gain = 1.0f;
        dbg_jump_handoff_wheel_source =
            JUMP_LANDING_WHEEL_SOURCE_REACTION;
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_REACTION;
        return normal_current;
    }

    /* During a jump buffer, the selector has already replaced normal_current
     * with the landing damper. Rebuild the raw grounded LQR wheel command from
     * the jointly clamped pair so the shared beta is applied exactly once. */
    if (buffer_return_blend_enabled &&
        chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        float lqr_left = LQROutBuf[LEFT][0];
        float lqr_right = LQROutBuf[RIGHT][0];
        LIMIT_MIN_MAX(lqr_left, -M3508_TOR_MAX, M3508_TOR_MAX);
        LIMIT_MIN_MAX(lqr_right, -M3508_TOR_MAX, M3508_TOR_MAX);
        const float lqr_common = jump_ground_wheel_common_select(
            0.5f * (lqr_left + lqr_right));
        normal_command = jump_wheel_output_command(
            JUMP_WHEEL_OUTPUT_GROUND_LQR,
            (uint8_t)(side == RIGHT), 0.0f, 0.0f, lqr_common, 1.0f) *
            M3508_TOR_TO_CUR;
    }
    if (buffer_return_blend_enabled) {
        const float owner_current_limit = M3508_TOR_MAX * M3508_TOR_TO_CUR;
        normal_command = jump_landing_owner_current_clamp(
            normal_command, owner_current_limit);
    }
    output = normal_command;

    if (buffer_owner_active || jump_landing_wheel_fade.active) {
        const float damper_current = jump_wheel_output_command(
            JUMP_WHEEL_OUTPUT_LANDING_DAMPER,
            (uint8_t)(side == RIGHT), 0.0f,
            jump_landing_wheel_torque, 0.0f, 0.0f) * M3508_TOR_TO_CUR;
        output = jump_landing_wheel_handoff_apply(
            &jump_landing_wheel_fade, (uint8_t)(side == RIGHT),
            normal_command, damper_current, buffer_owner_active,
            mode_allowed, off_ground, buffer_return_blend_enabled,
            jump_landing_wheel_buffer_return_gain,
            jump_landing_wheel_fade_duration_s,
            &used_gain, &output_source);
        if (buffer_return_blend_enabled) {
            const float owner_current_limit =
                M3508_TOR_MAX * M3508_TOR_TO_CUR;
            output = jump_landing_owner_current_clamp(
                output, owner_current_limit);
        }
        jump_landing_wheel_last_used_gain = used_gain;
        if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
            if (output_source == JUMP_LANDING_WHEEL_SOURCE_DAMPER ||
                output_source == JUMP_LANDING_WHEEL_SOURCE_FADE) {
                dbg_jump_wheel_output_source =
                    JUMP_WHEEL_OUTPUT_LANDING_DAMPER;
            } else if (output_source ==
                       JUMP_LANDING_WHEEL_SOURCE_RETURN_BLEND) {
                dbg_jump_wheel_output_source =
                    JUMP_WHEEL_OUTPUT_RETURN_BLEND;
            } else if (output_source ==
                       JUMP_LANDING_WHEEL_SOURCE_NORMAL) {
                dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_GROUND_LQR;
            } else {
                dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
            }
        }
        dbg_jump_handoff_wheel_source = mode_allowed ? output_source :
            JUMP_LANDING_WHEEL_SOURCE_ZERO;
        if ((output_source == JUMP_LANDING_WHEEL_SOURCE_DAMPER ||
             output_source == JUMP_LANDING_WHEEL_SOURCE_RETURN_BLEND) &&
            mode_allowed && !off_ground) {
            output_current = (int16_t)output;
            jump_landing_wheel_last_owner_command[side] =
                (float)output_current;
            jump_landing_wheel_last_owner_command_valid[side] = 1U;
        }
    } else {
        jump_landing_wheel_last_used_gain = 1.0f;
        if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
            dbg_jump_handoff_wheel_source = dbg_jump_wheel_output_source;
        } else if (chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
                   chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
                   chassis_cmd.ctrl_mode == CHASSIS_SPIN) {
            dbg_jump_handoff_wheel_source =
                JUMP_LANDING_WHEEL_SOURCE_NORMAL;
        } else {
            dbg_jump_handoff_wheel_source =
                JUMP_LANDING_WHEEL_SOURCE_ZERO;
        }
    }

    if (!owner_active && !mode_allowed) {
        jump_landing_wheel_fade.active = 0U;
        jump_landing_wheel_fade_duration_s =
            JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
        jump_landing_buffer_owner_previous = 0U;
        memset(jump_landing_wheel_last_owner_command_valid, 0,
               sizeof(jump_landing_wheel_last_owner_command_valid));
        jump_landing_wheel_torque = 0.0f;
    }
    output_current = (int16_t)output;
    return output_current;
}

static int16_t bump_torque_to_current(float torque)
{
    if (!isfinite(torque)) {
        return 0;
    }
    float current = torque * M3508_TOR_TO_CUR;
    if (!isfinite(current)) {
        return 0;
    }
    if (current > 32767.0f) current = 32767.0f;
    if (current < -32768.0f) current = -32768.0f;
    return (int16_t)current;
}

static int16_t bump_wheel_output_finalize(int16_t ordinary_current,
                                          chassis_side_e side,
                                          uint8_t pose_owner)
{
    const uint8_t bump_scope = bump_control_active() ||
        bump_wheel_handoff_active || pose_owner;
    if (!bump_scope) {
        return ordinary_current;
    }
#ifdef M3508_SET_ZERO
    return 0;
#endif
    const uint8_t normal_mode = chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
        chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    if (!normal_mode || chassis_fdb_data.stand_state == CHASSIS_IS_DANGER) {
        return 0;
    }
    if (bump_state == BUMP_STATE_ABORT) {
        return 0;
    }
    if (!pose_owner) {
        if (bump_control_active() && bump_wheel_handoff_active) {
            const float normal_common = 0.5f *
                (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]);
            if (!isfinite(normal_common)) {
                return 0;
            }
            const float logical_torque =
                bump_wheel_handoff_mix(normal_common);
            const float command = jump_wheel_output_command(
                JUMP_WHEEL_OUTPUT_LANDING_DAMPER,
                (uint8_t)(side == RIGHT), 0.0f, logical_torque,
                0.0f, 1.0f);
            return bump_torque_to_current(command);
        }
        return ordinary_current;
    }
    if (bump_state != BUMP_STATE_STAND) {
        return 0;
    }

    float ordinary_torque = 0.5f *
        (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]);
    if (!isfinite(ordinary_torque)) {
        return 0;
    }
    LIMIT_MIN_MAX(ordinary_torque, -M3508_TOR_MAX, M3508_TOR_MAX);
    float gain = bump_stand_handoff_gain;
    LIMIT_MIN_MAX(gain, 0.0f, 1.0f);
    const float command = jump_wheel_output_command(
        JUMP_WHEEL_OUTPUT_GROUND_LQR, (uint8_t)(side == RIGHT),
        0.0f, 0.0f, ordinary_torque, gain);
    return bump_torque_to_current(command);
}

/*当输入为正是,转动方向为 顺时针 时针
 * 计算输入应该为负
 * */
static float jump_ground_yaw_mix(
    float common, jump_wheel_output_source_t source)
{
    /* Never steer with a wheel that has already become a pitch reaction
     * wheel or a landing damper.  Both mirrored callbacks use identical yaw
     * torque/headroom, preserving the existing logical pitch common mode. */
    if (chassis_cmd.ctrl_mode != CHASSIS_JUMP ||
        source != JUMP_WHEEL_OUTPUT_GROUND_LQR ||
        (jump_state != JUMP_STATE_PRELOAD && jump_state != JUMP_STATE_THRUST) ||
        chassis_safety_module_off_ground() ||
        chassis_safety_module_wheel_shut() ||
        chassis_safety_module_landing_buffer_active() ||
        !control_feedback_valid || !jump_kin_rates_fresh ||
        !jump_kin_rates_usable ||
        !lateral_inertial_ff_sample_fresh(
            HAL_GetTick(), ins.publish_tick_ms, ins.publish_count,
            LATERAL_FF_INS_MAX_AGE_MS) ||
        !isfinite(common) || !isfinite(yaw_target) ||
        !isfinite(ins.yaw_total_angle) || !isfinite(ins.gyro[2])) {
        return common;
    }
    float horizontal_rate[CHASSIS_SIDE_COUNT];
    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        if (leg[side] == NULL || !isfinite(leg[side]->theta) ||
            !isfinite(leg[side]->l0) || leg[side]->l0 <= 0.0f ||
            !isfinite(jump_kin_theta_rate[side]) ||
            !isfinite(jump_kin_length_rate[side])) {
            return common;
        }
        horizontal_rate[side] =
            sinf(leg[side]->theta) * jump_length_rate(side) +
            leg[side]->l0 * cosf(leg[side]->theta) *
                jump_theta_rate_or_zero(side, 0.0f, 1U);
    }
    const float launch_rate = YAW_GYRO_SIGN * ins.gyro[2] +
        (jump_state == JUMP_STATE_THRUST ?
            JUMP_GROUND_YAW_MOMENTUM_GAIN *
                (horizontal_rate[RIGHT] -
                 JUMP_GROUND_YAW_LEFT_INERTIA_RATIO * horizontal_rate[LEFT]) :
            0.0f);
    float yaw_torque = JUMP_GROUND_YAW_KP *
        chassis_angle_delta(yaw_target, ins.yaw_total_angle * DEGREE_2_RAD) -
        JUMP_GROUND_YAW_KD * launch_rate;
    if (!isfinite(yaw_torque)) {
        return common;
    }
    const float headroom = fminf(JUMP_GROUND_YAW_TORQUE_LIMIT,
        fmaxf(0.0f, M3508_TOR_MAX - fabsf(common)));
    LIMIT_MIN_MAX(yaw_torque, -headroom, headroom);
    return common - yaw_torque;
}

static int16_t M3508_control_l(dji_motor_measure_t  measure){
    static int16_t set;
    dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
    const uint8_t reaction_admitted = jump_air_wheel_output_admitted();
    /* Keep the solver's raw pair immutable. Legacy selectors still operate on
     * per-side limited copies; jump recovery reads the completed-cycle common
     * candidate cache instead. */
    float lqr_wheel_left = LQROutBuf[LEFT][0];
    float lqr_wheel_right = LQROutBuf[RIGHT][0];
    LIMIT_MIN_MAX(lqr_wheel_left, -M3508_TOR_MAX, M3508_TOR_MAX);
    LIMIT_MIN_MAX(lqr_wheel_right, -M3508_TOR_MAX, M3508_TOR_MAX);

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
       (chassis_fdb_data.stand_state == CHASSIS_LEG_BACK_IS_OK ||
        chassis_fdb_data.stand_state == CHASSIS_IS_STAND)){
        /* Recovery handoff soft start. */
        set = (int16_t)(-(lqr_wheel_left) *
            chassis_recovery_module_handoff_gain() * M3508_TOR_TO_CUR);
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
       chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK &&
       chassis_fdb_data.stand_state != CHASSIS_IS_STAND){
        set = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
       chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
       chassis_cmd.ctrl_mode == CHASSIS_SPIN){
        if(!chassis_safety_module_wheel_shut()){
            float yaw_eff = yaw_ctrl_output;

            /* 起转踢一脚 */
#if YAW_START_KICK_ENABLE
            static float kick_timer_l = 0;
            static uint8_t last_rc_kick_l = 0;
            uint8_t rc_now =
                (fabsf(dbg_yaw_rate_ref) > YAW_RATE_REF_DEADBAND);
            if (rc_now && !last_rc_kick_l) kick_timer_l = YAW_START_KICK_TIME;
            last_rc_kick_l = rc_now;
            if (kick_timer_l > 0) {
                kick_timer_l -= 0.005f;
                if (fabsf(dbg_yaw_rate_fdb) < YAW_START_RATE_LOW_TH)
                    yaw_eff *= YAW_START_KICK_GAIN;
            }
#endif

            float lqr_local_l = 0.0f;
#if YAW_LQR_COMMON_MIX_ENABLE
            float lqr_T_common_l = jump_ground_wheel_common_select(0.5f * (lqr_wheel_left + lqr_wheel_right));
            lqr_local_l =
                yaw_lqr_local_residual_calculate(lqr_wheel_left, lqr_wheel_right);
            float lqr_cmd_l = -lqr_T_common_l + lqr_local_l;
#else
            float lqr_cmd_l = -lqr_wheel_left;
#endif
            float yaw_mix   = -yaw_eff;
            float yaw_cmd_l = YAW_MIX_L_SIGN * yaw_mix;

#if YAW_MIX_TEST_ENABLE
            lqr_cmd_l = 0.0f;
            float test_yaw = 0.0f;
            if (fabsf(chassis_cmd.vw_relative_set) > YAW_RC_DEADBAND)
                test_yaw = YAW_RATE_SIGN * chassis_cmd.vw_relative_set / RC_DBUS_MAX_VALUE * 0.10f;
            yaw_cmd_l = YAW_MIX_L_SIGN * (-test_yaw);
#endif

            uint8_t yaw_prio = yaw_priority_should_override_lqr();

            float net_cmd_l;
            if (jump_ground_recovery_scope_active &&
                jump_landing_wheel_handoff_mode_allowed() &&
                !chassis_safety_module_off_ground() &&
                jump_ground_wheel_cache_valid) {
                float extra = yaw_priority_mix(lqr_local_l, yaw_cmd_l,
                                               yaw_prio);
                const float headroom = fmaxf(0.0f,
                    M3508_TOR_MAX - fabsf(jump_ground_wheel_common_cache));
                LIMIT_MIN_MAX(extra, -headroom, headroom);
                net_cmd_l = -jump_ground_wheel_common_cache + extra;
            } else {
                net_cmd_l = yaw_priority_mix(lqr_cmd_l, yaw_cmd_l, yaw_prio);
            }
            set = (int16_t)(net_cmd_l * M3508_TOR_TO_CUR);

            dbg_lqr_cmd_l = lqr_cmd_l;
            dbg_yaw_cmd_l = yaw_cmd_l;
            dbg_net_cmd_l = net_cmd_l;

            /* Page 5: yaw full chain sign debug */
            dbg_yaw_eff_l = yaw_eff;
            dbg_yaw_mix_l = yaw_mix;
            dbg_yaw_prio_l = (float)yaw_prio;
            dbg_wheel_speed_raw_l = measure.speed_aps;
            dbg_wheel_speed_phys_l = -(measure.speed_aps / M3508_READUCTION_RATIO_L * DEGREE_2_RAD);
        }else{
            set = (int16_t)(-(lqr_wheel_left)* M3508_TOR_TO_CUR);
            dbg_lqr_cmd_l = -lqr_wheel_left;
            dbg_yaw_cmd_l = 0;
        }
    }
    if (bump_control_active()) {
        /* 2026-10-07 (操作手要求)：BUMP 期间 yaw 与前进直行的权限**与普通高腿长
         * 模式完全一致** —— 地面阶段不再改写轮端指令，直接沿用上面普通路径算好的
         * set（已含 LQR 共模 + yaw 差模 + 前进指令）。与右轮成对。
         * 只有离地/落地缓冲/轮端交接用阻尼力矩，ABORT 归零。 */
        const uint8_t bump_handoff =
            chassis_safety_module_off_ground() ||
            chassis_safety_module_landing_buffer_active() ||
            bump_wheel_handoff_active;
        if (bump_state == BUMP_STATE_ABORT) {
            set = 0;
            dbg_lqr_cmd_l = 0.0f;
            dbg_yaw_cmd_l = 0.0f;
            dbg_net_cmd_l = 0.0f;
        } else if (bump_handoff) {
            const float normal_torque = -0.5f *
                (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]);
            const float bump_torque = bump_wheel_handoff_mix(normal_torque);
            set = (int16_t)(bump_torque * M3508_TOR_TO_CUR);
            dbg_lqr_cmd_l = bump_torque;
            dbg_yaw_cmd_l = 0.0f;
            dbg_net_cmd_l = dbg_lqr_cmd_l;
        }
        /* else: 地面正常阶段 —— 保持普通轮端指令，不动 set */
    } else if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        const uint8_t off_ground =
            chassis_safety_module_off_ground();
        const uint8_t flight_air_phase =
            (jump_state == JUMP_STATE_FLIGHT) &&
            (off_ground ||
             chassis_safety_module_jump_air_profile_active());
        const uint8_t landing_buffer_active =
            chassis_safety_module_landing_buffer_active();
        const jump_wheel_output_source_t output_source =
            jump_wheel_output_source(
                reaction_admitted, flight_air_phase, off_ground,
                landing_buffer_active,
                chassis_safety_module_wheel_shut());
        dbg_jump_wheel_output_source = (uint8_t)output_source;
        const float lqr_common = jump_ground_wheel_common_select(
            0.5f * (lqr_wheel_left + lqr_wheel_right));
        const float logical_command = jump_wheel_output_command(
            output_source, 0U, jump_air_wheel_torque,
            jump_landing_wheel_torque, lqr_common,
            jump_landing_wheel_fade.active ? 1.0f :
                jump_landing_lqr_return_gain);
        const float mixed_command = jump_ground_yaw_mix(logical_command, output_source);
        set = (int16_t)(mixed_command * M3508_TOR_TO_CUR);
        dbg_lqr_cmd_l = logical_command;
        dbg_yaw_cmd_l = mixed_command - logical_command;
        dbg_net_cmd_l = mixed_command;
    } else if (reaction_admitted &&
               chassis_safety_module_off_ground()) {
        const float logical_command = jump_wheel_output_command(
            JUMP_WHEEL_OUTPUT_REACTION, 0U, jump_air_wheel_torque,
            0.0f, 0.0f, 1.0f);
        set = (int16_t)(logical_command * M3508_TOR_TO_CUR);
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_REACTION;
        dbg_lqr_cmd_l = logical_command;
        dbg_yaw_cmd_l = 0.0f;
        dbg_net_cmd_l = logical_command;
    }
#ifdef M3508_SET_ZERO
    set = 0;
    dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
#endif
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX){
        set = 0;
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
    }
    /* The landing-buffer damper and air reaction wheel are deliberate bounded
     * commands, so they must not be zeroed by the contact-confirmed gate. */
    const uint8_t landing_damper_authority =
        jump_reaction_wheel_landing_damper_authority(
            reaction_admitted, chassis_cmd.ctrl_mode == CHASSIS_JUMP,
            chassis_safety_module_off_ground(),
            chassis_safety_module_jump_air_profile_active(),
            chassis_safety_module_jump_landing_owner_active());
    if (!landing_damper_authority) {
        set = (int16_t)((float)set *
            chassis_safety_module_landing_gain());
    }
#ifndef M3508_SET_ZERO
    set = jump_landing_wheel_handoff_output(set, LEFT,
                                             reaction_admitted);
#else
    dbg_jump_handoff_wheel_source = JUMP_LANDING_WHEEL_SOURCE_ZERO;
    jump_landing_wheel_last_used_gain = 1.0f;
#endif
    /* Generic airborne paths keep the wheel hard-off; active jump flight has
     * its separate bounded reaction-wheel channel above. */
    if (jump_reaction_wheel_final_zero_required(
            chassis_safety_module_off_ground(),
            chassis_safety_module_airborne_allowed(),
            reaction_admitted)) {
        set = 0;
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
        dbg_jump_handoff_wheel_source =
            JUMP_LANDING_WHEEL_SOURCE_ZERO;
    }
    set = bump_wheel_output_finalize(
        set, LEFT, chassis_safety_module_bump_pose_owner_active());
    set_l = set;
    dbg_wheel_set_l = (float)set;
    return set;
}
/*当输入为正是,转动方向为 顺时针 时针
 * 计算输入应该为正
 * */
static int16_t M3508_control_r(dji_motor_measure_t measure){
    static int16_t set;
    dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
    const uint8_t reaction_admitted = jump_air_wheel_output_admitted();
    /* Keep the solver's raw pair immutable. Legacy selectors still operate on
     * per-side limited copies; jump recovery reads the completed-cycle common
     * candidate cache instead. */
    float lqr_wheel_left = LQROutBuf[LEFT][0];
    float lqr_wheel_right = LQROutBuf[RIGHT][0];
    LIMIT_MIN_MAX(lqr_wheel_left, -M3508_TOR_MAX, M3508_TOR_MAX);
    LIMIT_MIN_MAX(lqr_wheel_right, -M3508_TOR_MAX, M3508_TOR_MAX);

    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
       (chassis_fdb_data.stand_state == CHASSIS_LEG_BACK_IS_OK ||
        chassis_fdb_data.stand_state == CHASSIS_IS_STAND)){
        /* Recovery handoff soft start. */
        set = (int16_t)((lqr_wheel_right) *
            chassis_recovery_module_handoff_gain() * M3508_TOR_TO_CUR);
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_RECOVERY &&
       chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK &&
       chassis_fdb_data.stand_state != CHASSIS_IS_STAND){
        set = 0;
    }
    if(chassis_cmd.ctrl_mode == CHASSIS_OPEN_LOOP ||
       chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL ||
       chassis_cmd.ctrl_mode == CHASSIS_SPIN){
        if(!chassis_safety_module_wheel_shut()){
            float yaw_eff = yaw_ctrl_output;

            /* 起转踢一脚 */
#if YAW_START_KICK_ENABLE
            static float kick_timer_r = 0;
            static uint8_t last_rc_kick_r = 0;
            uint8_t rc_now =
                (fabsf(dbg_yaw_rate_ref) > YAW_RATE_REF_DEADBAND);
            if (rc_now && !last_rc_kick_r) kick_timer_r = YAW_START_KICK_TIME;
            last_rc_kick_r = rc_now;
            if (kick_timer_r > 0) {
                kick_timer_r -= 0.005f;
                if (fabsf(dbg_yaw_rate_fdb) < YAW_START_RATE_LOW_TH)
                    yaw_eff *= YAW_START_KICK_GAIN;
            }
#endif

            float lqr_local_r = 0.0f;
#if YAW_LQR_COMMON_MIX_ENABLE
            float lqr_T_common_r = jump_ground_wheel_common_select(0.5f * (lqr_wheel_left + lqr_wheel_right));
            lqr_local_r =
                yaw_lqr_local_residual_calculate(lqr_wheel_left, lqr_wheel_right);
            float lqr_cmd_r =  lqr_T_common_r + lqr_local_r;
#else
            float lqr_cmd_r =  lqr_wheel_right;
#endif
            float yaw_mix   = -yaw_eff;
            float yaw_cmd_r = YAW_MIX_R_SIGN * yaw_mix;

#if YAW_MIX_TEST_ENABLE
            lqr_cmd_r = 0.0f;
            float test_yaw = 0.0f;
            if (fabsf(chassis_cmd.vw_relative_set) > YAW_RC_DEADBAND)
                test_yaw = YAW_RATE_SIGN * chassis_cmd.vw_relative_set / RC_DBUS_MAX_VALUE * 0.10f;
            yaw_cmd_r = YAW_MIX_R_SIGN * (-test_yaw);
#endif

            uint8_t yaw_prio = yaw_priority_should_override_lqr();

            float net_cmd_r;
            if (jump_ground_recovery_scope_active &&
                jump_landing_wheel_handoff_mode_allowed() &&
                !chassis_safety_module_off_ground() &&
                jump_ground_wheel_cache_valid) {
                float extra = yaw_priority_mix(lqr_local_r, yaw_cmd_r,
                                               yaw_prio);
                const float headroom = fmaxf(0.0f,
                    M3508_TOR_MAX - fabsf(jump_ground_wheel_common_cache));
                LIMIT_MIN_MAX(extra, -headroom, headroom);
                net_cmd_r = jump_ground_wheel_common_cache + extra;
            } else {
                net_cmd_r = yaw_priority_mix(lqr_cmd_r, yaw_cmd_r, yaw_prio);
            }
            set = (int16_t)(net_cmd_r * M3508_TOR_TO_CUR);

            dbg_lqr_cmd_r = lqr_cmd_r;
            dbg_yaw_cmd_r = yaw_cmd_r;
            dbg_net_cmd_r = net_cmd_r;

            /* Page 5: yaw full chain sign debug */
            dbg_yaw_eff_r = yaw_eff;
            dbg_yaw_mix_r = yaw_mix;
            dbg_yaw_prio_r = (float)yaw_prio;
            dbg_wheel_speed_raw_r = measure.speed_aps;
            dbg_wheel_speed_phys_r = +(measure.speed_aps / M3508_READUCTION_RATIO_R * DEGREE_2_RAD);
        }else{
            set = (int16_t)((lqr_wheel_right) * M3508_TOR_TO_CUR);
            dbg_lqr_cmd_r = lqr_wheel_right;
            dbg_yaw_cmd_r = 0;
        }
    }
    if (bump_control_active()) {
        /* 2026-10-07 (操作手要求)：BUMP 期间 yaw 与前进直行的权限**与普通高腿长
         * 模式完全一致** —— 地面阶段不再改写轮端指令，直接沿用上面普通路径算好的
         * set（已含 LQR 共模 + yaw 差模 + 前进指令）。与左轮成对。
         * 只有离地/落地缓冲/轮端交接用阻尼力矩，ABORT 归零。 */
        const uint8_t bump_handoff =
            chassis_safety_module_off_ground() ||
            chassis_safety_module_landing_buffer_active() ||
            bump_wheel_handoff_active;
        if (bump_state == BUMP_STATE_ABORT) {
            set = 0;
            dbg_lqr_cmd_r = 0.0f;
            dbg_yaw_cmd_r = 0.0f;
            dbg_net_cmd_r = 0.0f;
        } else if (bump_handoff) {
            const float normal_torque = 0.5f *
                (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]);
            const float bump_torque = bump_wheel_handoff_mix(normal_torque);
            set = (int16_t)(bump_torque * M3508_TOR_TO_CUR);
            dbg_lqr_cmd_r = bump_torque;
            dbg_yaw_cmd_r = 0.0f;
            dbg_net_cmd_r = dbg_lqr_cmd_r;
        }
        /* else: 地面正常阶段 —— 保持普通轮端指令，不动 set */
    } else if (chassis_cmd.ctrl_mode == CHASSIS_JUMP) {
        const uint8_t off_ground =
            chassis_safety_module_off_ground();
        const uint8_t flight_air_phase =
            (jump_state == JUMP_STATE_FLIGHT) &&
            (off_ground ||
             chassis_safety_module_jump_air_profile_active());
        const uint8_t landing_buffer_active =
            chassis_safety_module_landing_buffer_active();
        const jump_wheel_output_source_t output_source =
            jump_wheel_output_source(
                reaction_admitted, flight_air_phase, off_ground,
                landing_buffer_active,
                chassis_safety_module_wheel_shut());
        dbg_jump_wheel_output_source = (uint8_t)output_source;
        const float lqr_common = jump_ground_wheel_common_select(
            0.5f * (lqr_wheel_left + lqr_wheel_right));
        const float logical_command = jump_wheel_output_command(
            output_source, 1U, jump_air_wheel_torque,
            jump_landing_wheel_torque, lqr_common,
            jump_landing_wheel_fade.active ? 1.0f :
                jump_landing_lqr_return_gain);
        const float mixed_command = jump_ground_yaw_mix(logical_command, output_source);
        set = (int16_t)(mixed_command * M3508_TOR_TO_CUR);
        dbg_lqr_cmd_r = logical_command;
        dbg_yaw_cmd_r = mixed_command - logical_command;
        dbg_net_cmd_r = mixed_command;
    } else if (reaction_admitted &&
               chassis_safety_module_off_ground()) {
        const float logical_command = jump_wheel_output_command(
            JUMP_WHEEL_OUTPUT_REACTION, 1U, jump_air_wheel_torque,
            0.0f, 0.0f, 1.0f);
        set = (int16_t)(logical_command * M3508_TOR_TO_CUR);
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_REACTION;
        dbg_lqr_cmd_r = logical_command;
        dbg_yaw_cmd_r = 0.0f;
        dbg_net_cmd_r = logical_command;
    }
#ifdef M3508_SET_ZERO
    set = 0;
    dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
#endif
    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX){
        set = 0;
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
    }
    /* The landing-buffer damper and air reaction wheel are deliberate bounded
     * commands, so they must not be zeroed by the contact-confirmed gate. */
    const uint8_t landing_damper_authority =
        jump_reaction_wheel_landing_damper_authority(
            reaction_admitted, chassis_cmd.ctrl_mode == CHASSIS_JUMP,
            chassis_safety_module_off_ground(),
            chassis_safety_module_jump_air_profile_active(),
            chassis_safety_module_jump_landing_owner_active());
    if (!landing_damper_authority) {
        set = (int16_t)((float)set *
            chassis_safety_module_landing_gain());
    }
#ifndef M3508_SET_ZERO
    set = jump_landing_wheel_handoff_output(set, RIGHT,
                                             reaction_admitted);
#else
    dbg_jump_handoff_wheel_source = JUMP_LANDING_WHEEL_SOURCE_ZERO;
    jump_landing_wheel_last_used_gain = 1.0f;
#endif
    /* Generic airborne paths keep the wheel hard-off; active jump flight has
     * its separate bounded reaction-wheel channel above. */
    if (jump_reaction_wheel_final_zero_required(
            chassis_safety_module_off_ground(),
            chassis_safety_module_airborne_allowed(),
            reaction_admitted)) {
        set = 0;
        dbg_jump_wheel_output_source = JUMP_WHEEL_OUTPUT_ZERO;
        dbg_jump_handoff_wheel_source =
            JUMP_LANDING_WHEEL_SOURCE_ZERO;
    }
    set = bump_wheel_output_finalize(
        set, RIGHT, chassis_safety_module_bump_pose_owner_active());
    set_r = set;
    dbg_wheel_set_r = (float)set;

    return set;
}


static void Leg_Is_Ok(){

    if((leg[LEFT]->l0 > (leg[LEFT]->length_ref - LEN_JUDGE_REGION)) && (leg[LEFT]->l0 < (leg[LEFT]->length_ref + LEN_JUDGE_REGION))){
        Leg_Left_len_JudgeOK = 1;
    }else{
        Leg_Left_len_JudgeOK = 0;
    }

    if((leg[RIGHT]->l0 > (leg[RIGHT]->length_ref - LEN_JUDGE_REGION)) && (leg[RIGHT]->l0 < (leg[RIGHT]->length_ref + LEN_JUDGE_REGION))){
        Leg_right_len_JudgeOK = 1;
    }else{
        Leg_right_len_JudgeOK = 0;
    }

    if((ins.roll > (0.0f -ROLL_JUDGE_REGION)) && (ins.roll < (0.0f +ROLL_JUDGE_REGION))){
        Body_roll_JudgeOK = 1;

    }else{
        Body_roll_JudgeOK = 0;
    }
    if((Leg_Left_len_JudgeOK == 1 || Leg_right_len_JudgeOK == 1) && Body_roll_JudgeOK == 1){
        chassis_cmd.leg_leng_change = LENGTH_STAY;
    }

}

/*
 * @brief 在非平衡状态下,把length_pid相关的积分清空,防止长时间累加开腿时失控
 */
/* 2026-10-07: BUMP 状态机的完整复位。process_clear() 与「进入 RECOVERY」两处
 * 都需要它 —— 后者是因为 RECOVERY 模式下 leg_calc() 不再运行，
 * bump_control_prepare_cycle() 不会被调用，状态机会永久冻结（见 bump6.csv：
 * 状态停在 RETRACT，腿已收到 0.114 却再也退不出来）。 */
void chassis_control_module_bump_reset(void)
{
    bump_state = BUMP_STATE_IDLE;
    bump_state_elapsed_s_accum = 0.0f;
    bump_request_consumed = 0U;
    bump_abort_reason = BUMP_ABORT_NONE;
    memset(bump_contact_latched, 0, sizeof(bump_contact_latched));
    memset(bump_contact_timer, 0, sizeof(bump_contact_timer));
    memset(bump_contact_last_rx_count, 0,
           sizeof(bump_contact_last_rx_count));
    memset(bump_hip_tp, 0, sizeof(bump_hip_tp));
    memset(bump_phi0_ref, 0, sizeof(bump_phi0_ref));
    memset(bump_phi0_hold, 0, sizeof(bump_phi0_hold));
    memset(bump_phi0_ref_rate, 0, sizeof(bump_phi0_ref_rate));
    memset(bump_angle_stable_timer, 0, sizeof(bump_angle_stable_timer));
    memset(bump_length_stable_timer, 0, sizeof(bump_length_stable_timer));
    memset(bump_phi0_rate_state, 0, sizeof(bump_phi0_rate_state));
    memset(bump_retract_breakaway_active, 0,
           sizeof(bump_retract_breakaway_active));
    memset(bump_retract_breakaway_elapsed, 0,
           sizeof(bump_retract_breakaway_elapsed));
    memset(bump_retract_breakaway_handoff, 0,
           sizeof(bump_retract_breakaway_handoff));
    memset(bump_retract_breakaway_handoff_elapsed, 0,
           sizeof(bump_retract_breakaway_handoff_elapsed));
    memset(bump_retract_lower_latched, 0,
           sizeof(bump_retract_lower_latched));
    bump_target_length = LEN_LEN_LOW;
    bump_length_ref_initialized = 0U;
    bump_length_ref[LEFT] = LEN_LEN_LOW;
    bump_length_ref[RIGHT] = LEN_LEN_LOW;
    bump_done_length_target[LEFT] = BUMP_APPROACH_LENGTH;
    bump_done_length_target[RIGHT] = BUMP_APPROACH_LENGTH;
    bump_airborne_lockout_seen = 0U;
    bump_retract_angle_reacquire = 0U;
    bump_stand_angle_reacquire = 0U;
    bump_buffer_seen = 0U;
    bump_ins_feedback_fresh = 0U;
    bump_wheel_handoff_active = 0U;
    bump_wheel_handoff_gain = 1.0f;
    bump_wheel_handoff_torque = 0.0f;
    bump_stand_support_gain = 1.0f;
    bump_stand_handoff_gain = 1.0f;
    bump_stand_extend_gain = 1.0f;
    memset(&bump_stand_handoff, 0, sizeof(bump_stand_handoff));
    chassis_safety_module_set_bump_active(0U);
    chassis_safety_module_set_bump_action_active(0U);
}

uint8_t chassis_control_module_bump_torque_budget_active(void)
{
    return (uint8_t)(bump_action_active() || bump_wheel_handoff_active ||
        (bump_buffer_seen &&
         (chassis_safety_module_landing_buffer_active() ||
          chassis_safety_module_bump_airborne_lockout())));
}

uint8_t chassis_control_module_bump_low_lqr_active(void)
{
    return (uint8_t)(bump_state == BUMP_STATE_STAND &&
                     chassis_cmd.bump_mode != 0U);
}

void chassis_control_module_process_clear(void){
    /* 2026-10-07 (bump2.csv): BUMP 状态机必须在这里复位。
     *
     * 移植时漏了这一段，后果很严重：bump 一旦卡在 ABORT，其状态会**扛过
     * 失能/使能**（process_clear 只在 CHASSIS_RELAX / CHASSIS_INIT 调用），
     * 于是重使能后仍然一直收腿，形成"收腿→松开"死循环。bump2 实测：
     * bump_state 卡在 ABORT(8) 达 46 秒以上，l0 在 0.355↔0.113 之间穿越
     * 0.20 m 共 10 次，且失能重使能无效。
     *
     * 下面与 code_611 的 process_clear 逐条对齐。 */
    chassis_control_module_bump_reset();

    const uint8_t interrupted_airborne =
        jump_landing_kf_interrupted_airborne(
            chassis_safety_module_off_ground(),
            jump_kf_airborne_hold_latched,
            (uint8_t)(jump_state == JUMP_STATE_FLIGHT));
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
    pid_clear(roll_pid);
    pid_clear(theta_pid);
    pid_clear(yaw_pid);
    chassis_safety_module_reset();
    leg_stuck_compensation_reset();
    support_force_ff_reset();
    roll_length_offset = 0.0f;
    roll_length_offset_target = 0.0f;
    roll_geometry_settle_timer = 0.0f;
    dbg_roll_slope_tan = 0.0f;
    roll_static_trim_force = 0.0f;
    roll_static_trim_applied = 0.0f;
    roll_static_trim_timer = 0.0f;
    dbg_roll_pd_output = 0.0f;
    dbg_roll_pre_split_guard = 0.0f;
    dbg_roll_pre_limit_force = 0.0f;
    roll_spin_target_applied = 0.0f;
    memset(roll_spin_integral_force, 0,
           sizeof(roll_spin_integral_force));
    roll_spin_integral_applied = 0.0f;
    roll_spin_integral_settle_timer = 0.0f;
    spin_leg_transition_rate_limit = 0.0f;
    spin_leg_transition_settle_timer = 0.0f;
    spin_leg_transition_active = 0U;
    spin_entry_leg_level = LEG_LOW;
    spin_entry_yaw_rate_limit = YAW_SPIN_RATE_MAX_LOW_RADPS;
    spin_entry_elapsed_s = 0.0f;
    spin_entry_latched = 0U;
    spin_leg_slew_rate_applied = LEG_LENGTH_REF_SLEW_RATE;
    F_lateral = 0.0f;
    F_bl_intertial = 0.0f;
    dbg_lateral_ff_target = 0.0f;
    lateral_inertial_ff_reset(&lateral_ff_state);
    dbg_lateral_feedback_mode_allowed = 0U;
    dbg_lateral_observer_ready = 0U;
    dbg_lateral_feedback_fresh = 0U;
    leg_split_target_guard_active = 0U;
    lqr_k_transition_initialized = 0U;
    lqr_k_transition_progress = 1.0f;
    lqr_k_transition_settle_timer = 0.0f;
    lqr_k_transition_tail_released = 1U;
    dbg_lqr_k_tail_waiting = 0U;
    lqr_dphi_notch_reset(0.0f);

    /* yaw 状态机复位 */
    yaw_ctrl_output = 0.0f;
    yaw_rate_iout = 0.0f;
    yaw_ctrl_state = YAW_DAMP_MODE;
    yaw_zero_timer = 0.0f;
    yaw_blocked = 0;
    yaw_block_on_cnt = 0;
    yaw_block_off_cnt = 0;
    block_release_holdoff_timer = 0.0f;
    last_rc_turning = 0;
    yaw_release_coast_timer = 0.0f;
    yaw_rate_ref_applied = 0.0f;

    LQRXRefBuf[LEFT][3] = 0;
    LQRXRefBuf[RIGHT][3] = 0;

    LQRXObsBuf[LEFT][3] = 0;
    LQRXObsBuf[RIGHT][3] = 0;

    yaw_target = ins.yaw_total_angle * DEGREE_2_RAD;
    /* Page 5 debug 变量复位 */
    dbg_yaw_raw_rc = 0.0f;
    dbg_yaw_raw_rc_set = 0.0f;
    dbg_yaw_vx_set = 0.0f;
    dbg_yaw_lqr_common = 0.0f;
    dbg_yaw_now_rad = 0.0f;
    dbg_yaw_gyro_raw_radps = 0.0f;
    dbg_yaw_gyro_signed_radps = 0.0f;
    dbg_yaw_rate_err = 0.0f;
    dbg_yaw_rate_desired = 0.0f;
    dbg_follow_yaw_relative = 0.0f;
    dbg_follow_yaw_error = 0.0f;
    dbg_yaw_rc_turning = 0.0f;
    dbg_yaw_invalid_flag = 0.0f;
    dbg_yaw_block_condition = 0.0f;
    dbg_yaw_unblock_condition = 0.0f;
    dbg_yaw_vx_l = 0.0f;
    dbg_yaw_vx_r = 0.0f;
    dbg_yaw_vx_delta_full = 0.0f;
    dbg_chassis_vx_limit = CHASSIS_V_MAX_LOW;
    dbg_yaw_rate_limit = YAW_RATE_MAX_LOW_RADPS;
    dbg_yaw_eff_l = 0.0f;
    dbg_yaw_eff_r = 0.0f;
    dbg_yaw_mix_l = 0.0f;
    dbg_yaw_mix_r = 0.0f;
    dbg_yaw_prio_l = 0.0f;
    dbg_yaw_prio_r = 0.0f;
    dbg_wheel_speed_raw_l = 0.0f;
    dbg_wheel_speed_raw_r = 0.0f;
    dbg_wheel_speed_phys_l = 0.0f;
    dbg_wheel_speed_phys_r = 0.0f;
    dbg_wheel_diff_phys = 0.0f;
    jump_control_reset();
    /* The safety reset above may clear its filtered off-ground flag. Preserve
     * the decision made from the pre-reset snapshot so a canceled airborne
     * jump cannot re-arm its old velocity reference on the next cycle. */
    jump_motion_vx_ref_state.cancelled = interrupted_airborne;
    if (interrupted_airborne) {
        jump_kf_airborne_hold_latched = 1U;
    }

}


static void Chassis_Vx_Detect(void)
{
    /* --- dt 计算 --- */
    static float _yaw_dt_start = 0;
    float dt;
    if (_yaw_dt_start != 0)
        dt = (dwt_get_time_ms() - _yaw_dt_start) / 1000.0f;
    else
        dt = 0.005f;
    _yaw_dt_start = dwt_get_time_ms();
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.020f) dt = 0.020f;

    const float yaw_now = ins.yaw_total_angle * DEGREE_2_RAD;
    const float yaw_rate_fdb = YAW_GYRO_SIGN * ins.gyro[2];
    const float rc_yaw = chassis_cmd.vw_relative_set;
    const uint8_t follow_mode =
        chassis_cmd.ctrl_mode == CHASSIS_FOLLOW_GIMBAL;
    const uint8_t spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    chassis_spin_profile_update(spin_mode, dt);
    if (!spin_mode) {
        spin_leg_transition_rate_limit = 0.0f;
        spin_leg_transition_settle_timer = 0.0f;
        spin_leg_transition_active = 0U;
    }
    const float manual_yaw_rate_limit = chassis_yaw_rate_limit_calculate();
    const float operator_yaw_rate_limit = spin_mode ?
        chassis_spin_yaw_rate_limit_calculate() :
        manual_yaw_rate_limit;
    const float yaw_rate_limit = follow_mode ?
        chassis_follow_yaw_rate_limit_calculate() :
        operator_yaw_rate_limit;
    dbg_yaw_rate_limit = yaw_rate_limit;

    float follow_yaw_relative = 0.0f;
    float follow_yaw_error = 0.0f;
    float yaw_rate_desired = 0.0f;

    if (follow_mode) {
        follow_yaw_relative = chassis_cmd.vw_fllow_gimbal_set;
        if (!isfinite(follow_yaw_relative)) {
            follow_yaw_relative = 0.0f;
        }
        follow_yaw_relative = chassis_angle_delta(
            follow_yaw_relative, 0.0f);
        follow_yaw_error = follow_yaw_relative;
        if (fabsf(follow_yaw_error) <= YAW_FOLLOW_ANGLE_DEADBAND) {
            follow_yaw_error = 0.0f;
        }
        LIMIT_MIN_MAX(follow_yaw_error,
                      -YAW_FOLLOW_ANGLE_LIMIT,
                       YAW_FOLLOW_ANGLE_LIMIT);
        /* Page 50 showed that positive chassis yaw increases the published
         * DM4310 relative angle.  Follow must therefore command the opposite
         * chassis direction to close the mechanical angle error. */
        yaw_rate_desired = YAW_FOLLOW_ANGLE_SIGN *
            YAW_FOLLOW_ANGLE_KP * follow_yaw_error;
        LIMIT_MIN_MAX(yaw_rate_desired,
                      -yaw_rate_limit, yaw_rate_limit);
    } else if (fabsf(rc_yaw) > YAW_RC_DEADBAND) {
        yaw_rate_desired =
            YAW_RATE_SIGN * (rc_yaw / RC_DBUS_MAX_VALUE) *
            operator_yaw_rate_limit;
        LIMIT_MIN_MAX(yaw_rate_desired,
                      -operator_yaw_rate_limit,
                       operator_yaw_rate_limit);
    }

    if (follow_mode || spin_mode) {
        const uint8_t decelerating =
            yaw_rate_ref_applied * yaw_rate_desired <= 0.0f ||
            fabsf(yaw_rate_desired) < fabsf(yaw_rate_ref_applied);
        const float slew_rate = decelerating ?
            YAW_OPERATOR_RATE_DECEL : YAW_OPERATOR_RATE_ACCEL;
        yaw_rate_ref_applied = move_towards_f32(
            yaw_rate_ref_applied, yaw_rate_desired, slew_rate * dt);
    } else {
        yaw_rate_ref_applied = yaw_rate_desired;
    }

    const uint8_t rc_turning =
        fabsf(yaw_rate_ref_applied) > YAW_RATE_REF_DEADBAND ||
        (follow_mode && follow_yaw_error != 0.0f);

    dbg_yaw_rate_desired = yaw_rate_desired;
    dbg_follow_yaw_relative = follow_yaw_relative;
    dbg_follow_yaw_error = follow_yaw_error;

    Vx_Delta = fabsf(chassis_kf_l.FilteredValue[0] -
                     chassis_kf_r.FilteredValue[0]);

    /* Discard every yaw state accumulated while airborne. */
    if (chassis_safety_module_landing_edge()) {
        yaw_target = yaw_now;
        yaw_ctrl_output = 0.0f;
        yaw_rate_iout = 0.0f;
        pid_clear(yaw_pid);
        yaw_ctrl_state = YAW_DAMP_MODE;
        yaw_zero_timer = 0.0f;
        yaw_blocked = 0;
        yaw_block_on_cnt = 0;
        yaw_block_off_cnt = 0;
        block_release_holdoff_timer = 0.0f;
        yaw_release_coast_timer = 0.0f;
        yaw_rate_ref_applied = 0.0f;
        last_rc_turning = 0U;
        chassis_safety_module_start_landing_yaw_hold();
    }

    if (chassis_safety_module_step_landing_yaw_hold(dt)) {
        yaw_target = yaw_now;
        yaw_ctrl_output = 0.0f;
        yaw_rate_iout = 0.0f;
        pid_clear(yaw_pid);
        yaw_ctrl_state = YAW_DAMP_MODE;
        yaw_zero_timer = 0.0f;
        yaw_rate_ref_applied = 0.0f;
        last_rc_turning = 0U;

        dbg_yaw_state = (uint8_t)yaw_ctrl_state;
        dbg_yaw_blocked = yaw_blocked;
        dbg_yaw_rate_ref = 0.0f;
        dbg_yaw_rate_fdb = yaw_rate_fdb;
        dbg_yaw_ctrl_output = 0.0f;
        dbg_yaw_ctrl_raw = 0.0f;
        dbg_yaw_target = yaw_target;
        dbg_yaw_zero_timer = 0.0f;
        dbg_vx_delta = Vx_Delta;
        dbg_yaw_release_coast_timer = 0.0f;
        dbg_last_rc_turning = 0U;
        return;
    }

    /* --- 卡住检测：必须同时满足"有yaw输出 + yaw_rate很小 + Vx_Delta大" --- */
    uint8_t yaw_cmd_large =
        (fabsf(yaw_ctrl_output) > YAW_BLOCK_CMD_MIN);

    uint8_t yaw_not_responding =
        (fabsf(yaw_rate_fdb) < YAW_BLOCK_RATE_LOW_TH);

    uint8_t block_condition =
        rc_turning &&
        yaw_cmd_large &&
        yaw_not_responding &&
        (Vx_Delta > VX_DELTA_MAX);

    uint8_t unblock_condition =
        (!rc_turning) ||
        (fabsf(yaw_rate_fdb) > YAW_BLOCK_RATE_LOW_TH * 2.0f) ||
        (Vx_Delta < VX_DELTA_MAX * YAW_BLOCK_HYST_RATIO);

    if (block_condition) {
        yaw_block_on_cnt++;
        yaw_block_off_cnt = 0;
        if (yaw_block_on_cnt >= YAW_BLOCK_ON_CNT) {
            yaw_blocked = 1;
            yaw_block_on_cnt = YAW_BLOCK_ON_CNT;
        }
    } else if (unblock_condition) {
        yaw_block_off_cnt++;
        yaw_block_on_cnt = 0;
        if (yaw_block_off_cnt >= YAW_BLOCK_OFF_CNT) {
            if (yaw_blocked) {
                block_release_holdoff_timer = YAW_BLOCK_RELEASE_HOLDOFF;
            }
            yaw_blocked = 0;
            yaw_block_off_cnt = YAW_BLOCK_OFF_CNT;
        }
    }

    /* holdoff 倒计时 */
    if (block_release_holdoff_timer > 0.0f) {
        block_release_holdoff_timer -= dt;
        if (block_release_holdoff_timer < 0.0f)
            block_release_holdoff_timer = 0.0f;
    }

    /* --- 无效状态：直接安全退出 --- */
    uint8_t yaw_invalid =
        chassis_safety_module_off_ground() ||
        chassis_safety_module_wheel_shut() ||
        (chassis_cmd.ctrl_mode == CHASSIS_RELAX) ||
        (chassis_cmd.ctrl_mode == CHASSIS_INIT) ||
        (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY);

    if (yaw_invalid) {
        yaw_target = yaw_now;
        yaw_ctrl_output = 0;
        yaw_rate_iout = 0.0f;
        pid_clear(yaw_pid);
        yaw_ctrl_state = YAW_DAMP_MODE;
        yaw_zero_timer = 0;
        block_release_holdoff_timer = 0;
        yaw_release_coast_timer = 0;
        yaw_rate_ref_applied = 0.0f;
        last_rc_turning = 0;
        /* 调试 */
        dbg_yaw_state = (uint8_t)yaw_ctrl_state;
        dbg_yaw_blocked = yaw_blocked;
        dbg_yaw_rate_ref = 0;
        dbg_yaw_rate_fdb = yaw_rate_fdb;
        dbg_yaw_ctrl_output = 0;
        dbg_yaw_ctrl_raw = 0;
        dbg_yaw_target = yaw_target;
        dbg_yaw_zero_timer = 0;
        dbg_vx_delta = Vx_Delta;
        dbg_yaw_release_coast_timer = 0;
        dbg_last_rc_turning = 0;
        return;
    }

    /* --- 核心状态机 --- */
    float yaw_ctrl_raw = 0;
    float yaw_rate_ref = 0;

    if (rc_turning) {
        /* ===== YAW_RATE_MODE：拨杆非零，前馈+反馈角速度控制 ===== */
        if (!last_rc_turning) {
            /* 新一次转向不能沿用松杆阻尼/锁角阶段的残余输出。 */
            yaw_ctrl_output = 0.0f;
            yaw_blocked = 0;
            yaw_block_on_cnt = 0;
            yaw_block_off_cnt = 0;
            block_release_holdoff_timer = 0.0f;
        }
        yaw_ctrl_state = YAW_RATE_MODE;
        if (!follow_mode) {
            yaw_target = yaw_now;
        }
        pid_clear(yaw_pid);
        yaw_zero_timer = 0;
        yaw_release_coast_timer = 0;

        yaw_rate_ref = yaw_rate_ref_applied;

        float yaw_cmd_sign;
        if (fabsf(yaw_rate_ref) > YAW_RATE_REF_DEADBAND) {
            yaw_cmd_sign = (yaw_rate_ref >= 0.0f) ? 1.0f : -1.0f;
        } else {
            yaw_cmd_sign = (yaw_rate_desired >= 0.0f) ? 1.0f : -1.0f;
        }
        float rate_err = yaw_rate_ref - yaw_rate_fdb;
        const uint8_t static_ff_active =
            !follow_mode || fabsf(yaw_rate_ref) >= YAW_FOLLOW_FF_RATE_MIN;
        const uint8_t follow_active_brake =
            follow_mode &&
            rate_err * yaw_cmd_sign < -YAW_FOLLOW_ACTIVE_BRAKE_MARGIN;

#if YAW_RATE_FB_ENABLE
        /* Page21 confirmed that measured current follows the requested wheel
         * torque, while a direction-dependent steady load remains.  A small
         * symmetric integral term rejects that load without a direction gain.
         * Reset on every new turn/sign reversal so no stored torque is carried
         * into the opposite direction. */
        if (!static_ff_active || !last_rc_turning || follow_active_brake ||
            yaw_rate_ref * dbg_yaw_rate_ref < 0.0f) {
            yaw_rate_iout = 0.0f;
        }
        if (static_ff_active) {
            yaw_rate_iout += YAW_RATE_FB_KI * rate_err * dt;
            LIMIT_MIN_MAX(yaw_rate_iout,
                          -YAW_RATE_I_LIMIT, YAW_RATE_I_LIMIT);
            if (yaw_rate_iout * yaw_cmd_sign < 0.0f) {
                yaw_rate_iout = 0.0f;
            }
        }
        yaw_ctrl_raw = (static_ff_active ?
                        yaw_cmd_sign * YAW_RATE_FF : 0.0f) +
                       YAW_RATE_FB_KP * rate_err + yaw_rate_iout;
#else
        yaw_rate_iout = 0.0f;
        yaw_ctrl_raw = static_ff_active ?
            yaw_cmd_sign * YAW_RATE_FF : 0.0f;
#endif

        /* 拨杆非零时，不允许反馈把输出反向刹死；actual 超速时允许反向阻尼。 */
        uint8_t yaw_overspeed =
            (fabsf(yaw_rate_fdb) > (yaw_rate_limit + YAW_RATE_OVERSPEED_MARGIN)) &&
            (yaw_rate_fdb * yaw_cmd_sign > 0.0f);
        if (yaw_overspeed) {
            yaw_rate_iout = 0.0f;
            yaw_ctrl_raw = YAW_RATE_OVERSPEED_DAMP_KP * rate_err;
        } else if (static_ff_active && !follow_active_brake &&
                   yaw_ctrl_raw * yaw_cmd_sign < YAW_RATE_CMD_MIN) {
            yaw_ctrl_raw = yaw_cmd_sign * YAW_RATE_CMD_MIN;
        }

        LIMIT_MIN_MAX(yaw_ctrl_raw, -yaw_MaxVal, yaw_MaxVal);

        /* 卡住时限制最大输出，但不缩放 */
        if (yaw_blocked) {
            yaw_rate_iout = 0.0f;
            LIMIT_MIN_MAX(yaw_ctrl_raw, -YAW_BLOCK_OUTPUT_LIMIT, YAW_BLOCK_OUTPUT_LIMIT);
        }

    } else {
        /* ===== 拨杆回零 ===== */
        yaw_rate_ref = 0;
        yaw_rate_iout = 0.0f;

        if (follow_mode) {
            /* Mechanical Follow has no accumulated chassis yaw target.  Near
             * center it only damps residual rotation; moving the gimbal out of
             * the deadband re-enters the shared rate controller above. */
            yaw_ctrl_state = YAW_DAMP_MODE;
            yaw_release_coast_timer = 0.0f;
            yaw_zero_timer = 0.0f;
            pid_clear(yaw_pid);
            yaw_ctrl_raw = YAW_DAMP_KP * (0.0f - yaw_rate_fdb);
        } else {

        /* 检测松杆瞬间，启动滑行计时器 */
        if (last_rc_turning) {
            yaw_release_coast_timer = YAW_RELEASE_COAST_TIME;
            yaw_zero_timer = 0;
            yaw_target = yaw_now;
            pid_clear(yaw_pid);
        }

        if (yaw_release_coast_timer > 0.0f) {
            /* ===== YAW_COAST_MODE：松杆滑行，弱阻尼 ===== */
            yaw_release_coast_timer -= dt;
            if (yaw_release_coast_timer < 0.0f) {
                yaw_release_coast_timer = 0.0f;
            }

            yaw_ctrl_state = YAW_COAST_MODE;
            yaw_target = yaw_now;
            pid_clear(yaw_pid);
            yaw_zero_timer = 0;

            yaw_ctrl_raw = YAW_COAST_DAMP_KP * (0.0f - yaw_rate_fdb);

        } else if (yaw_ctrl_state == YAW_LOCK_MODE) {
            /* ===== YAW_LOCK_MODE：目标已冻结，用角度-角速度 PD 保持 ===== */
            float yaw_err = yaw_target - yaw_now;
            if (fabsf(yaw_err) < YAW_LOCK_CAPTURE_DEADBAND) {
                yaw_err = 0.0f;
            }

            yaw_ctrl_raw = YAW_LOCK_KP * yaw_err -
                           YAW_LOCK_KD * yaw_rate_fdb;
            LIMIT_MIN_MAX(yaw_ctrl_raw,
                          -YAW_LOCK_OUTPUT_MAX,
                           YAW_LOCK_OUTPUT_MAX);

        } else {
            /* 尚未锁角：目标持续跟随当前角度，先把旋转和残余刹车消掉。 */
            uint8_t need_damp =
                yaw_blocked ||
                (block_release_holdoff_timer > 0.0f) ||
                (fabsf(yaw_rate_fdb) > YAW_LOCK_RATE_TH);

            yaw_ctrl_state = YAW_DAMP_MODE;
            yaw_target = yaw_now;
            pid_clear(yaw_pid);
            yaw_ctrl_raw = YAW_DAMP_KP * (0.0f - yaw_rate_fdb);

            if (need_damp ||
                fabsf(yaw_ctrl_output) > YAW_LOCK_OUTPUT_SETTLE_TH) {
                yaw_zero_timer = 0.0f;
            } else {
                yaw_zero_timer += dt;
            }

            if (yaw_zero_timer >= YAW_LOCK_ZERO_TIME) {
                /* 此时输出已接近零，捕获停止后的姿态，避免残余阻尼造成回拉。 */
                yaw_zero_timer = YAW_LOCK_ZERO_TIME;
                yaw_target = yaw_now;
                yaw_ctrl_state = YAW_LOCK_MODE;
                yaw_ctrl_raw = 0.0f;
                yaw_ctrl_output = 0.0f;
                pid_clear(yaw_pid);
            }
        }
        }
    }

    last_rc_turning = rc_turning;

    /* --- 输出斜率限制 + 限幅 --- */
    float delta = yaw_ctrl_raw - yaw_ctrl_output;
    float max_step = YAW_OUTPUT_SLEW_RATE * dt;
    if (delta > max_step)  delta = max_step;
    if (delta < -max_step) delta = -max_step;
    yaw_ctrl_output += delta;

    LIMIT_MIN_MAX(yaw_ctrl_output, -yaw_MaxVal, yaw_MaxVal);

    /* NaN/Inf 保护 */
    if (isnan(yaw_ctrl_output) || isinf(yaw_ctrl_output)) {
        yaw_ctrl_output = 0;
        yaw_rate_iout = 0.0f;
        pid_clear(yaw_pid);
        yaw_target = yaw_now;
    }

    /* --- 调试变量 --- */
    dbg_yaw_state       = (uint8_t)yaw_ctrl_state;
    dbg_yaw_blocked     = yaw_blocked;
    dbg_yaw_rate_ref    = yaw_rate_ref;
    dbg_yaw_rate_fdb    = yaw_rate_fdb;
    dbg_yaw_ctrl_output = yaw_ctrl_output;
    dbg_yaw_ctrl_raw    = yaw_ctrl_raw;
    dbg_yaw_rate_iout   = yaw_rate_iout;
    dbg_yaw_target      = follow_mode ? 0.0f : yaw_target;
    dbg_yaw_zero_timer  = yaw_zero_timer;
    dbg_vx_delta        = Vx_Delta;
    dbg_yaw_release_coast_timer = yaw_release_coast_timer;
    dbg_last_rc_turning = last_rc_turning;

    /* --- Page 5: yaw full chain sign debug 变量 --- */
    dbg_yaw_raw_rc        = chassis_cmd.vw_relative;
    dbg_yaw_raw_rc_set    = chassis_cmd.vw_relative_set;
    dbg_yaw_vx_set        = chassis_cmd.vx_set;
    dbg_yaw_lqr_common    = 0.5f * (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]);
    dbg_yaw_now_rad       = yaw_now;
    dbg_yaw_gyro_raw_radps  = ins.gyro[2];                  // 已是 rad/s，不乘 YAW_GYRO_SIGN
    dbg_yaw_gyro_signed_radps = yaw_rate_fdb;               // 等于 YAW_GYRO_SIGN * gyro
    dbg_yaw_rate_err      = yaw_rate_ref - yaw_rate_fdb;
    dbg_yaw_rc_turning    = (float)rc_turning;
    dbg_yaw_vx_l          = chassis_kf_l.FilteredValue[0];
    dbg_yaw_vx_r          = chassis_kf_r.FilteredValue[0];
    dbg_yaw_vx_delta_full = fabsf(chassis_kf_l.FilteredValue[0] - chassis_kf_r.FilteredValue[0]);
    dbg_yaw_block_condition   = (float)block_condition;
    dbg_yaw_unblock_condition = (float)unblock_condition;
    dbg_yaw_invalid_flag  = (float)yaw_invalid;
}

/*
 * @brief 检查底盘姿态,认为溃散后失能
 * */


/* Subscription and publication. */

/**
 * @brief chassis 线程中所有订阅者初始化
 */
void chassis_control_module_bind(const chassis_control_context_t *context)
{
    control_context = context;
    if (context == NULL) {
        return;
    }

    for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
        m3508_motor[side] = context->wheel_motor[side];
    }
    for (uint8_t motor = 0U; motor < 4U; motor++) {
        dm_motor[motor] = context->dm_motor[motor];
    }
}

void chassis_control_module_init(leg_config_t *leg_config)
{
    if (control_context == NULL || leg_config == NULL) {
        return;
    }

    leg[LEFT] = leg_register(leg_config);
    leg[RIGHT] = leg_register(leg_config);

    pid_config_t left_length_pid_config = INIT_PID_CONFIG(
        l_length_Kp, l_length_Ki, l_length_Kd, l_length_InteVal,
        l_length_MaxVal,
        (PID_Integral_Limit | PID_ChangingIntegrationRate |
         PID_DerivativeFilter | PID_OutputFilter));
    left_length_pid_config.CoefA = length_Integral_CoefA;
    left_length_pid_config.CoefB = length_Integral_CoefB;
    left_length_pid_config.Derivative_LPF_RC = length_Derivative_LPF_RC;
    L_length_pid = pid_register(&left_length_pid_config);

    pid_config_t right_length_pid_config = INIT_PID_CONFIG(
        r_length_Kp, r_length_Ki, r_length_Kd, r_length_InteVal,
        r_length_MaxVal,
        (PID_Integral_Limit | PID_ChangingIntegrationRate |
         PID_DerivativeFilter | PID_OutputFilter));
    right_length_pid_config.CoefA = length_Integral_CoefA;
    right_length_pid_config.CoefB = length_Integral_CoefB;
    right_length_pid_config.Derivative_LPF_RC = length_Derivative_LPF_RC;
    R_length_pid = pid_register(&right_length_pid_config);
    leg_stuck_compensation_reset();

    pid_config_t theta_pid_config = INIT_PID_CONFIG(
        theta_Kp, theta_Ki, theta_Kd, theta_InteVal, theta_MaxVal,
        PID_Integral_Limit | PID_OutputFilter);
    theta_pid = pid_register(&theta_pid_config);

    pid_config_t yaw_pid_config = INIT_PID_CONFIG(
        yaw_Kp, yaw_Ki, yaw_Kd, yaw_InteVal, yaw_MaxVal,
        PID_Integral_Limit | PID_OutputFilter);
    yaw_pid = pid_register(&yaw_pid_config);

    pid_config_t roll_pid_config = INIT_PID_CONFIG(
        roll_Kp, roll_Ki, roll_Kd, roll_InteVal, roll_MaxVal,
        PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter);
    roll_pid_config.Derivative_LPF_RC = roll_Derivative_LPF_RC;
    roll_pid = pid_register(&roll_pid_config);

    pid_config_t theta_comp_pid_config = INIT_PID_CONFIG(
        theta_comp_Kp, theta_comp_Ki, theta_comp_Kd,
        theta_comp_InteVal, theta_comp_MaxVal,
        PID_Integral_Limit | PID_OutputFilter);
    theta_comp_pid = pid_register(&theta_comp_pid_config);

    chassis_kf_init();
    jump_control_reset();
}

leg_obj_t *chassis_control_module_leg(chassis_side_e side)
{
    return side < CHASSIS_SIDE_COUNT ? leg[side] : NULL;
}

pid_obj_t *chassis_control_module_theta_pid(void)
{
    return theta_pid;
}

pid_obj_t *chassis_control_module_roll_pid(void)
{
    return roll_pid;
}

pid_obj_t *chassis_control_module_length_pid(chassis_side_e side)
{
    return side == LEFT ? L_length_pid : (side == RIGHT ? R_length_pid : NULL);
}

float *chassis_control_module_vmc_output(chassis_side_e side)
{
    return side == LEFT ? vmc_out_l : (side == RIGHT ? vmc_out_r : NULL);
}

float (*chassis_control_module_lqr_output(void))[2]
{
    return LQROutBuf;
}

float (*chassis_control_module_lqr_x_obs(void))[6]
{
    return LQRXObsBuf;
}

float *chassis_control_module_lqr_virtual_x(chassis_side_e side)
{
    return side == LEFT ? &lqr_virtual_x_l :
           (side == RIGHT ? &lqr_virtual_x_r : NULL);
}

float *chassis_control_module_yaw_target(void)
{
    return &yaw_target;
}

float *chassis_control_module_wheel_speed_phys(chassis_side_e side)
{
    return side == LEFT ? &dbg_wheel_speed_phys_l :
           (side == RIGHT ? &dbg_wheel_speed_phys_r : NULL);
}

void chassis_control_module_get_hip_shake_debug(
    chassis_hip_shake_debug_t *snapshot)
{
    if (snapshot == NULL || leg[LEFT] == NULL || leg[RIGHT] == NULL) {
        return;
    }

    snapshot->leg_length_l = leg[LEFT]->l0;
    snapshot->leg_length_r = leg[RIGHT]->l0;
    snapshot->theta_raw_diff = leg[LEFT]->theta - leg[RIGHT]->theta;
    snapshot->theta_lqr_diff = LQRXObsBuf[LEFT][0] - LQRXObsBuf[RIGHT][0];
    snapshot->theta_rate_l = kin_rate_observer[LEFT].theta_rate;
    snapshot->theta_rate_r = kin_rate_observer[RIGHT].theta_rate;
    snapshot->theta_rate_diff = dbg_theta_rate_diff;
    snapshot->theta_sync_p = dbg_theta_sync_p;
    snapshot->theta_sync_d = dbg_theta_sync_d;
    snapshot->theta_sync_guard = dbg_theta_sync_guard;
    snapshot->lqr_tp_l = -LQROutBuf[LEFT][1];
    snapshot->lqr_tp_r = -LQROutBuf[RIGHT][1];
    snapshot->final_tp_l = leg[LEFT]->Tp;
    snapshot->final_tp_r = leg[RIGHT]->Tp;
    snapshot->support_force_l = leg[LEFT]->support_force;
    snapshot->support_force_r = leg[RIGHT]->support_force;
    snapshot->length_force_l = F_l_L;
    snapshot->length_force_r = F_l_R;
    snapshot->roll_force = F_roll;
    snapshot->roll_p = roll_pid->Pout;
    snapshot->roll_d = roll_pid->Dout;
}

uint8_t chassis_control_module_jump_peak_torque_active(void)
{
    const uint8_t thrust_peak =
        jump_state == JUMP_STATE_THRUST &&
        !chassis_safety_module_off_ground();
    const uint8_t air_peak =
        jump_state == JUMP_STATE_FLIGHT &&
        chassis_safety_module_off_ground();
    return thrust_peak || air_peak;
}

uint8_t chassis_control_module_jump_fault(void)
{
    return (uint8_t)jump_fault;
}

uint8_t chassis_control_module_jump_state(void)
{
    return (uint8_t)jump_state;
}

uint8_t chassis_control_module_jump_fast_poll_active(void)
{
    if (control_context == NULL ||
        (chassis_cmd.ctrl_mode != CHASSIS_JUMP &&
         chassis_cmd.ctrl_mode != CHASSIS_OPEN_LOOP &&
         chassis_cmd.ctrl_mode != CHASSIS_FOLLOW_GIMBAL)) {
        return 0U;
    }
    return (uint8_t)(jump_state != JUMP_STATE_IDLE ||
        chassis_safety_module_landing_buffer_jump_origin() ||
        chassis_safety_module_landing_postbuffer_active() ||
        chassis_safety_module_landing_airborne_guard_active() ||
        jump_ground_recovery_scope_active);
}

float chassis_control_module_jump_elapsed_s(void)
{
    return jump_state_elapsed_s();
}

float chassis_control_module_jump_cycle_raw_dt_ms(void)
{
    return jump_cycle_raw_dt_ms;
}

float chassis_control_module_jump_leg_length_rate(chassis_side_e side)
{
    if (side >= CHASSIS_SIDE_COUNT) {
        return 0.0f;
    }
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    return jump_kin_rates_usable ? jump_kin_length_rate[side] : 0.0f;
#else
    return isfinite(leg[side]->d_l0_lpf) ? leg[side]->d_l0_lpf : 0.0f;
#endif
}

float chassis_control_module_jump_length_target(void)
{
    return jump_length_target;
}

uint8_t chassis_control_module_jump_thrust_active(void)
{
    return jump_thrust_active;
}

uint8_t chassis_control_module_jump_stroke_release_reason(void)
{
    return jump_control_active() ?
        (uint8_t)jump_stroke_release_reason :
        (uint8_t)JUMP_STROKE_RELEASE_NONE;
}

float chassis_control_module_jump_roll_diff_force(void)
{
    return jump_roll_diff_force;
}

/* Page-56 wheel-channel diagnostics: inputs (reference / error / rate) and the
 * final logical torque.  All three are latched every control cycle. */
float chassis_control_module_jump_air_wheel_torque(void)
{
    return jump_air_wheel_torque;
}

float chassis_control_module_jump_air_wheel_pitch_ref_rad(void)
{
    return jump_air_wheel_pitch_ref_rad;
}

float chassis_control_module_jump_air_wheel_pitch_err_rad(void)
{
    return jump_air_wheel_pitch_err_rad;
}

float chassis_control_module_jump_air_wheel_pitch_rate(void)
{
    return jump_air_wheel_pitch_rate;
}

uint8_t chassis_control_module_jump_wheel_output_source(void)
{
    return dbg_jump_wheel_output_source;
}

void chassis_control_module_set_leg_lengths(float left, float right)
{
    static float last_update_ms = 0.0f;
    float now_ms = dwt_get_time_ms();

    if (leg[LEFT] == NULL || leg[RIGHT] == NULL) {
        return;
    }

    if (!normal_leg_transition_mode()) {
        spin_leg_slew_rate_applied = LEG_LENGTH_REF_SLEW_RATE;
        leg[LEFT]->length_ref = left;
        leg[RIGHT]->length_ref = right;
        chassis_cmd.leg_leng_change = LENGTH_STAY;
        leg_stuck_post_ramp_timer = 0.0f;
        leg_stuck_compensation_phase_begin();
        leg_length_reference_ramping_prev = 0U;
        last_update_ms = now_ms;
        return;
    }

    float safe_dt = (now_ms - last_update_ms) / 1000.0f;
    if (!(safe_dt > 0.0f) || safe_dt > 0.02f) {
        safe_dt = 0.001f;
    }
    last_update_ms = now_ms;
    const uint8_t spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
    spin_leg_slew_rate_applied = spin_mode ?
        LEG_LENGTH_REF_SLEW_RATE_SPIN : LEG_LENGTH_REF_SLEW_RATE;
    float max_step = spin_leg_slew_rate_applied * safe_dt;

    leg[LEFT]->length_ref = move_towards_f32(
        leg[LEFT]->length_ref, left, max_step);
    leg[RIGHT]->length_ref = move_towards_f32(
        leg[RIGHT]->length_ref, right, max_step);

    const uint8_t was_ramping = leg_length_reference_ramping_prev;
    const uint8_t reference_ramping =
        fabsf(left - leg[LEFT]->length_ref) > LEG_LENGTH_REF_EPSILON ||
        fabsf(right - leg[RIGHT]->length_ref) > LEG_LENGTH_REF_EPSILON;

    if (!was_ramping && reference_ramping) {
        leg_stuck_post_ramp_timer = 0.0f;
        leg_stuck_compensation_phase_begin();
    } else if (was_ramping && !reference_ramping) {
        leg_stuck_post_ramp_timer = LEG_STUCK_POST_RAMP_TIME;
        leg_stuck_compensation_phase_begin();
    }
    leg_length_reference_ramping_prev = reference_ramping;
    chassis_cmd.leg_leng_change = reference_ramping ?
        LENGTH_CHANGE : LENGTH_STAY;
}

void chassis_control_module_set_yaw_target(float target)
{
    yaw_target = target;
}

void chassis_control_module_clear_recovery_runtime(void)
{
    pid_clear(roll_pid);
    pid_clear(L_length_pid);
    pid_clear(R_length_pid);
    pid_clear(theta_pid);
    leg_stuck_compensation_reset();
    support_force_ff_reset();
    roll_length_offset = 0.0f;
    roll_length_offset_target = 0.0f;
    roll_geometry_settle_timer = 0.0f;
    dbg_roll_slope_tan = 0.0f;
    roll_static_trim_force = 0.0f;
    roll_static_trim_applied = 0.0f;
    roll_static_trim_timer = 0.0f;
    dbg_roll_pd_output = 0.0f;
    dbg_roll_pre_split_guard = 0.0f;
    dbg_roll_pre_limit_force = 0.0f;
    roll_spin_target_applied = 0.0f;
    memset(roll_spin_integral_force, 0,
           sizeof(roll_spin_integral_force));
    roll_spin_integral_applied = 0.0f;
    roll_spin_integral_settle_timer = 0.0f;
    spin_leg_transition_rate_limit = 0.0f;
    spin_leg_transition_settle_timer = 0.0f;
    spin_leg_transition_active = 0U;
    spin_entry_leg_level = LEG_LOW;
    spin_entry_yaw_rate_limit = YAW_SPIN_RATE_MAX_LOW_RADPS;
    spin_entry_elapsed_s = 0.0f;
    spin_entry_latched = 0U;
    spin_leg_slew_rate_applied = LEG_LENGTH_REF_SLEW_RATE;
    F_lateral = 0.0f;
    F_bl_intertial = 0.0f;
    dbg_lateral_ff_target = 0.0f;
    lateral_inertial_ff_reset(&lateral_ff_state);
    dbg_lateral_feedback_mode_allowed = 0U;
    dbg_lateral_observer_ready = 0U;
    dbg_lateral_feedback_fresh = 0U;
    leg_split_target_guard_active = 0U;
    lqr_virtual_x_l = 0.0f;
    lqr_virtual_x_r = 0.0f;
    lqr_position_capture_timer = 0.0f;
    lqr_position_hold_x = 0.0f;
    lqr_position_brake_offset = 0.0f;
    lqr_position_state = LQR_POSITION_DRIVE;
    theta_comp_learn_settle_timer = 0.0f;
    lqr_k_transition_settle_timer = 0.0f;
    lqr_k_transition_tail_released = 1U;
    dbg_lqr_k_tail_waiting = 0U;
    lqr_dphi_notch_reset(0.0f);
    LQRXObsBuf[LEFT][2] = 0.0f;
    LQRXObsBuf[RIGHT][2] = 0.0f;
}

void chassis_control_module_update_yaw(void)
{
    Chassis_Vx_Detect();
}

void chassis_control_module_kf_update(void)
{
    chassis_kf_update();
}

void chassis_control_module_step(void)
{
#ifdef BSP_CHASSIS_LEG_MODE
    leg_calc();
#endif
}

void chassis_control_module_capture_wheel_feedback(chassis_side_e side,
                                                   float speed_aps)
{
    if (side == LEFT) {
        dbg_m3508_spd_l = speed_aps;
    } else if (side == RIGHT) {
        dbg_m3508_spd_r = speed_aps;
    }
}

int16_t chassis_control_module_wheel_current(chassis_side_e side,
                                             dji_motor_measure_t measure)
{
    if (control_context == NULL) {
        return 0;
    }
    return side == LEFT ? M3508_control_l(measure) :
           (side == RIGHT ? M3508_control_r(measure) : 0);
}

uint8_t chassis_control_module_fill_debug_page(uint8_t page)
{
    if (page == 53U) {
        /* Page 53: Spin + variable leg-length diagnostics, I0-I31:
         * I0=time; I1=mode; I2=Spin latch; I3=entry leg level;
         * I4=current leg level; I5=latched yaw-rate limit; I6=active limit;
         * I7=desired yaw rate; I8=applied yaw-rate reference;
         * I9=measured yaw rate; I10=yaw-rate error; I11=length state;
         * I12/I13=mean target/actual leg length; I14/I15=left/right target;
         * I16/I17=left/right actual length; I18=mean measured length rate;
         * I19=applied length slew rate; I20=leg transition active;
         * I21=transition settle timer; I22=Spin elapsed time;
         * I23=raw yaw command; I24=yaw controller output;
         * I25=transition rate diagnostic; I26=entry rate duplicate for
         * easy VOFA overlay; I27=Spin mode; I28=sloped yaw command;
         * I29=raw yaw controller output; I30=mean length error;
         * I31=status bits: 1=Spin, 2=latched, 4=length transition,
         * 8=reference ramping. */
        const float target_length = 0.5f *
            (leg[LEFT]->length_ref + leg[RIGHT]->length_ref);
        const float actual_length = 0.5f *
            (leg[LEFT]->l0 + leg[RIGHT]->l0);
        const float actual_length_rate = 0.5f *
            (leg[LEFT]->d_l0_lpf + leg[RIGHT]->d_l0_lpf);
        const uint8_t spin_mode = chassis_cmd.ctrl_mode == CHASSIS_SPIN;
        const uint8_t status = (uint8_t)(
            (spin_mode ? 1U : 0U) |
            (spin_entry_latched ? 2U : 0U) |
            (spin_leg_transition_active ? 4U : 0U) |
            (chassis_cmd.leg_leng_change == LENGTH_CHANGE ? 8U : 0U));
        chassis_debug_set(0U, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1U, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2U, (float)spin_entry_latched);
        chassis_debug_set(3U, (float)spin_entry_leg_level);
        chassis_debug_set(4U, (float)chassis_cmd.leg_level);
        chassis_debug_set(5U, spin_entry_yaw_rate_limit);
        chassis_debug_set(6U, dbg_yaw_rate_limit);
        chassis_debug_set(7U, dbg_yaw_rate_desired);
        chassis_debug_set(8U, dbg_yaw_rate_ref);
        chassis_debug_set(9U, dbg_yaw_rate_fdb);
        chassis_debug_set(10U, dbg_yaw_rate_ref - dbg_yaw_rate_fdb);
        chassis_debug_set(11U, (float)chassis_cmd.leg_leng_change);
        chassis_debug_set(12U, target_length);
        chassis_debug_set(13U, actual_length);
        chassis_debug_set(14U, leg[LEFT]->length_ref);
        chassis_debug_set(15U, leg[RIGHT]->length_ref);
        chassis_debug_set(16U, leg[LEFT]->l0);
        chassis_debug_set(17U, leg[RIGHT]->l0);
        chassis_debug_set(18U, actual_length_rate);
        chassis_debug_set(19U, spin_leg_slew_rate_applied);
        chassis_debug_set(20U, (float)spin_leg_transition_active);
        chassis_debug_set(21U, spin_leg_transition_settle_timer);
        chassis_debug_set(22U, spin_entry_elapsed_s);
        chassis_debug_set(23U, chassis_cmd.vw_relative);
        chassis_debug_set(24U, yaw_ctrl_output);
        chassis_debug_set(25U, spin_leg_transition_rate_limit);
        chassis_debug_set(26U, spin_entry_yaw_rate_limit);
        chassis_debug_set(27U, (float)spin_mode);
        chassis_debug_set(28U, chassis_cmd.vw_relative_set);
        chassis_debug_set(29U, dbg_yaw_ctrl_raw);
        chassis_debug_set(30U, target_length - actual_length);
        chassis_debug_set(31U, (float)status);
        return 1U;
    }

    if (page == 64U) {
        chassis_debug_set(22U, dbg_wheel_set_l);
        chassis_debug_set(23U, dbg_wheel_set_r);
        return 1U;
    }

    if (page == 54U) {
        /* BUMP 专用诊断页，避免覆盖跳跃页 52 的通道定义。
         *
         * 移植自 code_611。注意：这一页在 611 里是**死代码** —— 611 的调度链是
         *   chassis_debug_module_fill_runtime_page() || ... || 本函数
         * 而 fill_runtime_page() 先认领了 54 页并返回 1，短路链直接结束，
         * 所以 611 的 BUMP 页从未被执行过。914 没有 fill_runtime_page()，
         * 因此本页在 914 里会真正生效。
         *
         * I0=time s; I1=mode; I2=state; I3=abort reason; I4=request;
         * I5=control active; I6=pose owner; I7=true-air lockout;
         * I8/I9=left/right l0; I10/I11=effective left/right length targets;
         * I12/I13=left/right phi0; I14/I15=left/right phi0 reference;
         * I16/I17=left/right measured phi0 rate; I18/I19=final left/right hip Tp;
         * I20/I21=left/right axial support; I22/I23=left/right wheel torque;
         * I24=gravity support restore gain; I25=hip/wheel handoff gain;
         * I26/I27=pitch/roll; I28=state elapsed seconds; I29=ground contact;
         * I30=landing-buffer phase; I31=status bits (wheel shut, offground,
         * pose owner, true-air lockout, contact latches, hip/INS freshness,
         * left/right phi0-rate validity, stand-stage latches, and RETRACT
         * lower-length latches in bits 13/14). */
        chassis_debug_set(0U, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1U, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2U, (float)bump_state);
        chassis_debug_set(3U, (float)bump_abort_reason);
        chassis_debug_set(4U, (float)chassis_cmd.bump_mode);
        chassis_debug_set(5U, (float)bump_control_active());
        chassis_debug_set(6U,
            (float)chassis_safety_module_bump_pose_owner_active());
        chassis_debug_set(7U,
            (float)chassis_safety_module_bump_airborne_lockout());
        chassis_debug_set(8U, leg[LEFT]->l0);
        chassis_debug_set(9U, leg[RIGHT]->l0);
        chassis_debug_set(10U, bump_length_target_effective[LEFT]);
        chassis_debug_set(11U, bump_length_target_effective[RIGHT]);
        chassis_debug_set(12U, leg[LEFT]->phi0);
        chassis_debug_set(13U, leg[RIGHT]->phi0);
        chassis_debug_set(14U, bump_phi0_ref[LEFT]);
        chassis_debug_set(15U, bump_phi0_ref[RIGHT]);
        chassis_debug_set(16U, bump_phi0_rate_state[LEFT].rate);
        chassis_debug_set(17U, bump_phi0_rate_state[RIGHT].rate);
        chassis_debug_set(18U, leg[LEFT]->Tp);
        chassis_debug_set(19U, leg[RIGHT]->Tp);
        chassis_debug_set(20U, leg[LEFT]->support_force);
        chassis_debug_set(21U, leg[RIGHT]->support_force);
        chassis_debug_set(22U, (float)set_l / M3508_TOR_TO_CUR);
        chassis_debug_set(23U, (float)set_r / M3508_TOR_TO_CUR);
        chassis_debug_set(24U, bump_stand_support_gain);
        chassis_debug_set(25U, bump_stand_handoff_gain);
        chassis_debug_set(26U, ins.pitch);
        chassis_debug_set(27U, ins.roll);
        chassis_debug_set(28U, bump_state_elapsed_s());
        chassis_debug_set(29U,
            (float)chassis_safety_module_ground_contact_confirmed());
        chassis_debug_set(30U,
            (float)chassis_safety_module_landing_buffer_phase());
        uint16_t status = 0U;
        if (chassis_safety_module_wheel_shut()) status |= 1U << 0;
        if (chassis_safety_module_off_ground()) status |= 1U << 1;
        if (chassis_safety_module_bump_pose_owner_active()) status |= 1U << 2;
        if (chassis_safety_module_bump_airborne_lockout()) status |= 1U << 3;
        if (bump_contact_latched[LEFT]) status |= 1U << 4;
        if (bump_contact_latched[RIGHT]) status |= 1U << 5;
        if (control_feedback_valid) status |= 1U << 6;
        if (bump_ins_feedback_fresh) status |= 1U << 7;
        if (bump_phi0_rate_state[LEFT].valid) status |= 1U << 8;
        if (bump_phi0_rate_state[RIGHT].valid) status |= 1U << 9;
        if (bump_stand_handoff.handoff_started) status |= 1U << 10;
        if (bump_stand_handoff.extension_started) status |= 1U << 11;
        if (bump_stand_handoff.support_started) status |= 1U << 12;
        if (bump_retract_lower_latched[LEFT]) status |= 1U << 13;
        if (bump_retract_lower_latched[RIGHT]) status |= 1U << 14;
        chassis_debug_set(31U, (float)status);
        return 1U;
    }

    if (page == 52U) {
        /* Jump diagnostic, I0-I31:
         * I0-I3=time/mode/stand/jump state;
         * I4=fault + 10*request-gate + 100*request + 1000*thrust-active
         *    + 2000*timed-air-profile-active;
         * I5=state time;
         * I6/I7=effective left/right length target;
         * I8-I11=L/R length and rate;
         * I12/I13=L/R virtual hip Tp;
         * I14=body roll (deg);
         * I15=mean left/right leg theta (rad);
         * I16=four hip DM feedback fresh flag (1=all fresh <=30 ms);
         * I17=off-ground; I18=landing phase (0/1/2/3);
         * I19=forward speed captured when this jump started;
         * I20/I21=pitch/pitch-rate; I22/I23=L/R final axial force.
         * Chassis task overwrites I24-I27 with LF/LB/RF/RB final torque;
         * I28/I29=left/right filtered support used by ground detection;
         * I30/I31=left/right leg theta. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_fdb_data.stand_state);
        chassis_debug_set(3, (float)jump_state);
        chassis_debug_set(4, (float)jump_fault +
                             10.0f * (float)chassis_cmd.jump_request_gate +
                             100.0f * (float)jump_request_seen +
                             1000.0f * (float)jump_thrust_active +
                             2000.0f * (float)
                                 chassis_safety_module_jump_air_profile_active());
        chassis_debug_set(5, jump_state_elapsed_s());
        chassis_debug_set(6,
            (chassis_safety_module_off_ground() ||
             chassis_safety_module_jump_air_profile_active() ||
             chassis_safety_module_landing_buffer_active()) ?
            chassis_safety_module_air_length_target(CHASSIS_SIDE_LEFT) :
            jump_length_target);
        chassis_debug_set(7,
            (chassis_safety_module_off_ground() ||
             chassis_safety_module_jump_air_profile_active() ||
             chassis_safety_module_landing_buffer_active()) ?
            chassis_safety_module_air_length_target(CHASSIS_SIDE_RIGHT) :
            jump_length_target);
        chassis_debug_set(8, leg[LEFT]->l0);
        chassis_debug_set(9, leg[RIGHT]->l0);
        chassis_debug_set(10, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(11, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(12, leg[LEFT]->Tp);
        chassis_debug_set(13, leg[RIGHT]->Tp);
        chassis_debug_set(14, ins.roll);
        /* Mean theta is more important than its already visible derivative:
         * it distinguishes a genuine tuck from both rods swinging together. */
        chassis_debug_set(15,
            0.5f * (leg[LEFT]->theta + leg[RIGHT]->theta));
        {
            const uint32_t now_ms = HAL_GetTick();
            uint8_t hips_fresh = 1U;
            for (uint8_t motor = 0U; motor < 4U; ++motor) {
                if (dm_motor[motor] == NULL ||
                    dm_motor[motor]->measure.rx_count == 0U ||
                    (uint32_t)(now_ms - dm_motor[motor]->measure.last_rx_ms) >
                        30U) {
                    hips_fresh = 0U;
                    break;
                }
            }
            chassis_debug_set(16, (float)hips_fresh);
        }
        chassis_debug_set(17,
                          (float)chassis_safety_module_off_ground());
        chassis_debug_set(18,
                          (float)chassis_safety_module_landing_buffer_phase());
        chassis_debug_set(19, jump_launch_vx);
        chassis_debug_set(20, ins.pitch);
        chassis_debug_set(21, -LQRXObsBuf[LEFT][5]);
        chassis_debug_set(22, leg[LEFT]->support_force);
        chassis_debug_set(23, leg[RIGHT]->support_force);
        for (uint8_t channel = 24U; channel < 32U; channel++) {
            chassis_debug_set(channel, 0.0f);
        }
        return 1U;
    }

    if (page == 2U) {
        float vx_target = 0.5f * (LQRXRefBuf[LEFT][3] + LQRXRefBuf[RIGHT][3]);
        float theta_diff = leg[LEFT]->theta - leg[RIGHT]->theta;

        chassis_debug_set(0, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(1, (float)chassis_fdb_data.stand_state);
        chassis_debug_set(2, vx_target);
        chassis_debug_set(3, chassis_vx_filter);
        chassis_debug_set(4, vx_target - chassis_vx_filter);
        chassis_debug_set(5, leg[LEFT]->length_ref);
        chassis_debug_set(6, leg[LEFT]->l0);
        chassis_debug_set(7, L_length_pid->Output);
        chassis_debug_set(8, leg[RIGHT]->length_ref);
        chassis_debug_set(9, leg[RIGHT]->l0);
        chassis_debug_set(10, leg[RIGHT]->length_ref - leg[RIGHT]->l0);
        chassis_debug_set(11, R_length_pid->Output);
        chassis_debug_set(12, theta_diff);
        chassis_debug_set(13, theta_pid->Output);
        chassis_debug_set(14, yaw_target);
        chassis_debug_set(15, yaw_ctrl_output);
        return 1U;
    }

    if (page == 3U) {
        /* Mechanical Follow + Spin handoff, I0-I15:
         * time; mode+10*leg; relative yaw; bounded follow error;
         * desired/applied/measured yaw rate; rate error; output/raw/integral;
         * leg split; roll; yaw state; blocked flag; raw Spin command. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, dbg_follow_yaw_relative);
        chassis_debug_set(3, dbg_follow_yaw_error);
        chassis_debug_set(4, dbg_yaw_rate_desired);
        chassis_debug_set(5, dbg_yaw_rate_ref);
        chassis_debug_set(6, dbg_yaw_rate_fdb);
        chassis_debug_set(7, dbg_yaw_rate_ref - dbg_yaw_rate_fdb);
        chassis_debug_set(8, dbg_yaw_ctrl_output);
        chassis_debug_set(9, dbg_yaw_ctrl_raw);
        chassis_debug_set(10, dbg_yaw_rate_iout);
        chassis_debug_set(11, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(12, ins.roll);
        chassis_debug_set(13, (float)dbg_yaw_state);
        chassis_debug_set(14, (float)dbg_yaw_blocked);
        chassis_debug_set(15, chassis_cmd.vw_relative_set);
        return 1U;
    }

    if (page == 50U) {
        /* Follow Gimbal complete 32-channel chain:
         * I0=time; I1/I2/I3=chassis mode/stand state/leg level;
         * Follow: I4=DM4310 mechanical relative yaw, I5=bounded follow error;
         * Spin: I4=average actual leg length, I5=body pitch;
         * I6=yaw-rate limit; I7/I8/I9=desired/applied/measured yaw rate;
         * I10=rate error; I11/I12/I13=raw/output/integral yaw command;
         * Follow I14/I15=yaw state/blocked flag;
         * Spin I14/I15=roll target offset/integral force;
         * I16/I17=vx command/feedback;
         * I18=left-right velocity-observer mismatch;
         * I19/I20=left/right final wheel command;
         * I21/I22=left/right physical wheel speed;
         * I23/I24=left/right measured wheel torque;
         * I25/I26=left/right yaw differential command;
         * I27/I28=leg-length/swing-angle split; I29=roll; I30=F_roll;
         * I31=safety bits: 1=off-ground, 2=wheel-shut,
         * 4=yaw-priority mixing, 8=leg-split guard,
         * 16=Spin leg-transition interlock (reference or physical settling). */
        uint8_t follow_safety_flags = 0U;
        if (chassis_safety_module_off_ground()) {
            follow_safety_flags |= 1U;
        }
        if (chassis_safety_module_wheel_shut()) {
            follow_safety_flags |= 2U;
        }
        if (yaw_priority_should_override_lqr()) {
            follow_safety_flags |= 4U;
        }
        if (leg_split_target_guard_active) {
            follow_safety_flags |= 8U;
        }
        if (spin_leg_transition_active) {
            follow_safety_flags |= 16U;
        }

        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_fdb_data.stand_state);
        chassis_debug_set(3, (float)chassis_cmd.leg_level);
        const uint8_t page50_spin =
            chassis_cmd.ctrl_mode == CHASSIS_SPIN;
        chassis_debug_set(4, page50_spin ?
                             0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0) :
                             chassis_cmd.vw_fllow_gimbal_set);
        chassis_debug_set(5, page50_spin ? ins.pitch :
                             dbg_follow_yaw_error);
        chassis_debug_set(6, dbg_yaw_rate_limit);
        chassis_debug_set(7, dbg_yaw_rate_desired);
        chassis_debug_set(8, dbg_yaw_rate_ref);
        chassis_debug_set(9, dbg_yaw_rate_fdb);
        chassis_debug_set(10, dbg_yaw_rate_ref - dbg_yaw_rate_fdb);
        chassis_debug_set(11, dbg_yaw_ctrl_raw);
        chassis_debug_set(12, dbg_yaw_ctrl_output);
        chassis_debug_set(13, dbg_yaw_rate_iout);
        chassis_debug_set(14, page50_spin ? roll_spin_target_applied :
                             (float)dbg_yaw_state);
        chassis_debug_set(15, page50_spin ? roll_spin_integral_applied :
                             (float)dbg_yaw_blocked);
        chassis_debug_set(16, chassis_cmd.vx_set);
        chassis_debug_set(17, chassis_vx_filter);
        chassis_debug_set(18, Vx_Delta);
        chassis_debug_set(19, dbg_net_cmd_l);
        chassis_debug_set(20, dbg_net_cmd_r);
        chassis_debug_set(21, dbg_wheel_speed_phys_l);
        chassis_debug_set(22, dbg_wheel_speed_phys_r);
        chassis_debug_set(23,
            m3508_motor[LEFT] != NULL ?
                (float)m3508_motor[LEFT]->measure.real_current /
                    M3508_TOR_TO_CUR : 0.0f);
        chassis_debug_set(24,
            m3508_motor[RIGHT] != NULL ?
                (float)m3508_motor[RIGHT]->measure.real_current /
                    M3508_TOR_TO_CUR : 0.0f);
        chassis_debug_set(25, dbg_yaw_cmd_l);
        chassis_debug_set(26, dbg_yaw_cmd_r);
        chassis_debug_set(27, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(28, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(29, ins.roll);
        chassis_debug_set(30, F_roll);
        chassis_debug_set(31, (float)follow_safety_flags);
        return 1U;
    }

    if (page == 4U) {
        chassis_debug_set(0, leg[LEFT]->length_ref);
        chassis_debug_set(1, leg[LEFT]->l0);
        chassis_debug_set(2, leg[LEFT]->length_ref - leg[LEFT]->l0);
        chassis_debug_set(3, L_length_pid->Output);
        chassis_debug_set(4, leg[RIGHT]->length_ref);
        chassis_debug_set(5, leg[RIGHT]->l0);
        chassis_debug_set(6, leg[RIGHT]->length_ref - leg[RIGHT]->l0);
        chassis_debug_set(7, R_length_pid->Output);
        chassis_debug_set(8, LQROutBuf[LEFT][0]);
        chassis_debug_set(9, LQROutBuf[RIGHT][0]);
        chassis_debug_set(10, dbg_yaw_cmd_l);
        chassis_debug_set(11, dbg_yaw_cmd_r);
        chassis_debug_set(12, dbg_net_cmd_l);
        chassis_debug_set(13, dbg_net_cmd_r);
        chassis_debug_set(14, dbg_m3508_spd_l);
        chassis_debug_set(15, dbg_m3508_spd_r);
        return 1U;
    }

    if (page == 5U) {
        /* I0: 0~3=状态机；+10=卡滞，+20=离地，+40=轮端关闭，+80=yaw 优先混控。 */
        chassis_debug_set(0, (float)dbg_yaw_state +
                             (dbg_yaw_blocked ? 10.0f : 0.0f) +
                             (chassis_safety_module_off_ground() ? 20.0f : 0.0f) +
                             (chassis_safety_module_wheel_shut() ? 40.0f : 0.0f) +
                             (yaw_priority_should_override_lqr() ? 80.0f : 0.0f));
        chassis_debug_set(1, dbg_yaw_ctrl_output);
        chassis_debug_set(2, dbg_yaw_vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, dbg_yaw_rate_ref);
        chassis_debug_set(5, dbg_yaw_gyro_signed_radps);
        chassis_debug_set(6, dbg_yaw_target);
        chassis_debug_set(7, dbg_yaw_now_rad);
        chassis_debug_set(8, dbg_yaw_target - dbg_yaw_now_rad);
        chassis_debug_set(9, dbg_yaw_lqr_common);
        chassis_debug_set(10, dbg_yaw_cmd_l);
        chassis_debug_set(11, dbg_net_cmd_l);
        chassis_debug_set(12, dbg_net_cmd_r);
        chassis_debug_set(13, dbg_wheel_speed_phys_l);
        chassis_debug_set(14, dbg_wheel_speed_phys_r);
        chassis_debug_set(15, dbg_yaw_zero_timer);
        return 1U;
    }

    if (page == 14U) {
        /* Roll 腿长目标与防劈叉约束，严格使用 I0-I15：
         * I0=t，I1=模式，I2=档位，I3=vx，I4=yaw rate；
         * I5=目标保护状态，I6=roll 偏置目标，I7=实际应用偏置；
         * I8=average ref, I9-I10=left/right l0, I11-I12=total length force;
         * I13=roll, I14-I15=left/right effective length target. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_cmd.leg_level);
        chassis_debug_set(3, chassis_vx_filter);
        chassis_debug_set(4, YAW_GYRO_SIGN * ins.gyro[2]);
        chassis_debug_set(5, (float)leg_split_target_guard_active);
        chassis_debug_set(6, roll_length_offset_target);
        chassis_debug_set(7, roll_length_offset);
        chassis_debug_set(8, 0.5f * (leg[LEFT]->length_ref +
                                     leg[RIGHT]->length_ref));
        chassis_debug_set(9, leg[LEFT]->l0);
        chassis_debug_set(10, leg[RIGHT]->l0);
        chassis_debug_set(11, F_l_L);
        chassis_debug_set(12, F_l_R);
        chassis_debug_set(13, ins.roll);
        chassis_debug_set(14, roll_length_target_l);
        chassis_debug_set(15, roll_length_target_r);
        return 1U;
    }

    if (page == 15U) {
        /* 横滚 PID：I0=t，I1~I6=模式/双腿长度，I7=实际转向指令，
         * I8~I14=roll PID，I15=左右腿长度环输出差。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_fdb_data.stand_state);
        chassis_debug_set(3, leg[LEFT]->length_ref);
        chassis_debug_set(4, leg[LEFT]->l0);
        chassis_debug_set(5, leg[RIGHT]->length_ref);
        chassis_debug_set(6, leg[RIGHT]->l0);
        chassis_debug_set(7, chassis_cmd.vw_relative_set);
        chassis_debug_set(8, roll_pid->Measure);
        chassis_debug_set(9, roll_pid->Err);
        chassis_debug_set(10, ins.gyro[0]);
        chassis_debug_set(11, roll_pid->Pout);
        chassis_debug_set(12, roll_pid->Iout);
        chassis_debug_set(13, roll_pid->Dout);
        chassis_debug_set(14, roll_pid->Output);
        chassis_debug_set(15, F_l_L - F_l_R);
        return 1U;
    }

    if (page == 16U) {
        /* 高腿长纵向 LQR，严格使用 I0-I15：
         * I0=t，I1=模式，I2=腿长档位，I3/I4=目标/实际平均腿长；
         * I5/I6=目标/实际速度，I7/I8=pitch角/角速度；
         * I9~I12=轮/髋的 pitch 与 pitch-rate K；
         * I13/I14=轮端/髋关节 LQR 共模输出，I15=yaw rate。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_cmd.leg_level);
        chassis_debug_set(3, 0.5f * (leg[LEFT]->length_ref +
                                     leg[RIGHT]->length_ref));
        chassis_debug_set(4, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(5, LQRXRefBuf[LEFT][3]);
        chassis_debug_set(6, chassis_vx_filter);
        chassis_debug_set(7, ins.pitch);
        chassis_debug_set(8, LQRXObsBuf[LEFT][5]);
        chassis_debug_set(9, MatLQR_K[0][4]);
        chassis_debug_set(10, MatLQR_K[0][5]);
        chassis_debug_set(11, MatLQR_K[1][4]);
        chassis_debug_set(12, MatLQR_K[1][5]);
        chassis_debug_set(13, 0.5f * (LQROutBuf[LEFT][0] +
                                      LQROutBuf[RIGHT][0]));
        chassis_debug_set(14, 0.5f * (LQROutBuf[LEFT][1] +
                                      LQROutBuf[RIGHT][1]));
        chassis_debug_set(15, YAW_GYRO_SIGN * ins.gyro[2]);
        return 1U;
    }

    if (page == 17U) {
        /* 高腿长轴向/VMC诊断，严格使用 I0-I15：
         * I0=t，I1/I2=模式/档位，I3=平均目标腿长；
         * I4-I7=左右腿长及速度，I8-I9=左右腿长环总输出；
         * I10-I11=左右静摩擦补偿，I12-I15=四个VMC请求力矩。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_cmd.leg_level);
        chassis_debug_set(3, 0.5f * (leg[LEFT]->length_ref + leg[RIGHT]->length_ref));
        chassis_debug_set(4, leg[LEFT]->l0);
        chassis_debug_set(5, leg[RIGHT]->l0);
        chassis_debug_set(6, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(7, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(8, F_l_L);
        chassis_debug_set(9, F_l_R);
        chassis_debug_set(10, leg_stuck_force[LEFT]);
        chassis_debug_set(11, leg_stuck_force[RIGHT]);
        chassis_debug_set(12, vmc_out_l[FRONT]);
        chassis_debug_set(13, vmc_out_l[BACK]);
        chassis_debug_set(14, vmc_out_r[FRONT]);
        chassis_debug_set(15, vmc_out_r[BACK]);
        return 1U;
    }

    if (page == 19U) {
        /* 左腿长度振荡与目标保护诊断，严格使用 I0-I15：
         * I0=t，I1/I2=模式/档位，I3/I4=左右有效目标；
         * I5/I6=左右实际腿长，I7/I8=左右滤波腿长速度；
         * I9/I10=左右腿长环输出，I11=左右实际腿长差；
         * I12=目标保护状态，I13=实际应用的 roll 腿长偏置；
         * I14/I15=左腿前/后髋关节 VMC 请求力矩。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_cmd.leg_level);
        chassis_debug_set(3, roll_length_target_l);
        chassis_debug_set(4, roll_length_target_r);
        chassis_debug_set(5, leg[LEFT]->l0);
        chassis_debug_set(6, leg[RIGHT]->l0);
        chassis_debug_set(7, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(8, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(9, F_l_L);
        chassis_debug_set(10, F_l_R);
        chassis_debug_set(11, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(12, (float)leg_split_target_guard_active);
        chassis_debug_set(13, roll_length_offset);
        chassis_debug_set(14, vmc_out_l[FRONT]);
        chassis_debug_set(15, vmc_out_l[BACK]);
        return 1U;
    }

    if (page == 20U) {
        /* 低腿长纵向失稳诊断，严格使用 I0-I15：
         * I0=t，I1/I2=模式/档位，I3/I4=pitch/pitch-rate，I5=roll；
         * I6/I7=平均有效目标/实际腿长，I8/I9=左右实际腿长；
         * I10=平均腿长速度，I11/I12=轮端/髋关节 LQR 共模输出；
         * I13=腿长环共模输出，I14=静摩擦补偿共模输出；
         * I15=直接 roll 差动力输出。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode);
        chassis_debug_set(2, (float)chassis_cmd.leg_level);
        chassis_debug_set(3, ins.pitch);
        chassis_debug_set(4, LQRXObsBuf[LEFT][5]);
        chassis_debug_set(5, ins.roll);
        chassis_debug_set(6, 0.5f * (roll_length_target_l +
                                     roll_length_target_r));
        chassis_debug_set(7, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(8, leg[LEFT]->l0);
        chassis_debug_set(9, leg[RIGHT]->l0);
        chassis_debug_set(10, 0.5f * (leg[LEFT]->d_l0_lpf +
                                      leg[RIGHT]->d_l0_lpf));
        chassis_debug_set(11, 0.5f * (LQROutBuf[LEFT][0] +
                                      LQROutBuf[RIGHT][0]));
        chassis_debug_set(12, 0.5f * (LQROutBuf[LEFT][1] +
                                      LQROutBuf[RIGHT][1]));
        chassis_debug_set(13, 0.5f * (F_l_L + F_l_R));
        chassis_debug_set(14, 0.5f * (leg_stuck_force[LEFT] +
                                      leg_stuck_force[RIGHT]));
        chassis_debug_set(15, F_roll);
        return 1U;
    }

    if (page == 21U) {
        /* 中高腿转向、roll 与劈叉联合诊断，严格使用 I0-I15：
         * I0=t，I1=模式+10*腿长档位，I2=原始 yaw 遥杆；
         * I3/I4=yaw-rate 目标/反馈，I5=yaw 输出；
         * I6=Yaw 卡滞 + 2*腿差保护；
         * I7=轮端 LQR 共模，I8/I9=左右轮最终净指令；
         * I10/I11=左右物理轮速，I12=roll，I13=F_roll；
         * I14=左右实际腿长差，I15=pitch-rate。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        /* Symmetric yaw plant diagnosis:
         * I0=time; I1=mode+10*level+100*ramping+1000*K-tail-waiting;
         * I2/I3=yaw-rate ref/fdb; I4=yaw controller output;
         * I5=yaw-rate integral; I6/I7=left/right final wheel command;
         * I8/I9=left/right measured current in torque units;
         * I10/I11=left/right physical wheel speed; I12/I13=roll/rate;
         * I14=left-right leg length; I15=F_roll. */
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f * (float)dbg_lqr_k_tail_waiting);
        chassis_debug_set(2, dbg_yaw_rate_ref);
        chassis_debug_set(3, dbg_yaw_rate_fdb);
        chassis_debug_set(4, dbg_yaw_ctrl_output);
        chassis_debug_set(5, dbg_yaw_rate_iout);
        chassis_debug_set(6, dbg_net_cmd_l);
        chassis_debug_set(7, dbg_net_cmd_r);
        chassis_debug_set(8, (float)m3508_motor[LEFT]->measure.real_current /
                             M3508_TOR_TO_CUR);
        chassis_debug_set(9, (float)m3508_motor[RIGHT]->measure.real_current /
                             M3508_TOR_TO_CUR);
        chassis_debug_set(10, dbg_wheel_speed_phys_l);
        chassis_debug_set(11, dbg_wheel_speed_phys_r);
        chassis_debug_set(12, ins.roll);
        chassis_debug_set(13, ins.gyro[0]);
        chassis_debug_set(14, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(15, F_roll);
        return 1U;
    }

    if (page == 22U) {
        /* 左腿摆角发散首发诊断，严格使用 I0-I15：
         * I0=t，I1=模式+10*腿长档位，I2/I3=前进/转向原始指令；
         * I4/I5=roll/F_roll；I6/I7=左右摆角 theta；
         * I8/I9=左右摆角速度；I10/I11=左右实际腿长；
         * I12/I13=左右腿长环轴向力；I14/I15=左右最终髋关节 Tp。 */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.roll);
        chassis_debug_set(5, F_roll);
        chassis_debug_set(6, leg[LEFT]->theta);
        chassis_debug_set(7, leg[RIGHT]->theta);
        chassis_debug_set(8, leg[LEFT]->d_theta_lpf);
        chassis_debug_set(9, leg[RIGHT]->d_theta_lpf);
        chassis_debug_set(10, leg[LEFT]->l0);
        chassis_debug_set(11, leg[RIGHT]->l0);
        chassis_debug_set(12, F_l_L);
        chassis_debug_set(13, F_l_R);
        chassis_debug_set(14, leg[LEFT]->Tp);
        chassis_debug_set(15, leg[RIGHT]->Tp);
        return 1U;
    }

    if (page == 23U) {
        float theta_rate_residual_l = leg[LEFT]->d_theta +
                                      leg[LEFT]->d_phi0 + ins.gyro[0];
        float theta_rate_residual_r = leg[RIGHT]->d_theta +
                                      leg[RIGHT]->d_phi0 + ins.gyro[0];
        float wheel_surface_l =
            -(m3508_motor[LEFT]->measure.speed_aps /
              M3508_READUCTION_RATIO_L * DEGREE_2_RAD) * WHEEL_RADIUS;
        float wheel_surface_r =
            (m3508_motor[RIGHT]->measure.speed_aps /
             M3508_READUCTION_RATIO_R * DEGREE_2_RAD) * WHEEL_RADIUS;

        /* Observer timebase and kinematic consistency, exactly I0-I15. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_kf_dt_s * 1000.0f);
        chassis_debug_set(3, ins.gyro[0]);
        chassis_debug_set(4, leg[LEFT]->d_phi0);
        chassis_debug_set(5, leg[RIGHT]->d_phi0);
        chassis_debug_set(6, leg[LEFT]->d_theta);
        chassis_debug_set(7, leg[RIGHT]->d_theta);
        chassis_debug_set(8, theta_rate_residual_l);
        chassis_debug_set(9, theta_rate_residual_r);
        chassis_debug_set(10, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(11, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(12, 0.5f * (wheel_surface_l + wheel_surface_r));
        chassis_debug_set(13, 0.5f * (wheel_to_ground_l + wheel_to_ground_r));
        chassis_debug_set(14, chassis_vx_filter);
        chassis_debug_set(15, ins.motion_accel_b[1]);
        return 1U;
    }

    if (page == 24U) {
        /* DM feedback speed versus angle-difference leg velocity, I0-I15. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_kf_dt_s * 1000.0f);
        chassis_debug_set(3, ins.gyro[0]);
        chassis_debug_set(4, leg[LEFT]->phi1);
        chassis_debug_set(5, leg[LEFT]->phi4);
        chassis_debug_set(6, leg[RIGHT]->phi1);
        chassis_debug_set(7, leg[RIGHT]->phi4);
        chassis_debug_set(8, -dm_motor[3]->measure.speed_rads);
        chassis_debug_set(9, -dm_motor[0]->measure.speed_rads);
        chassis_debug_set(10, dm_motor[2]->measure.speed_rads);
        chassis_debug_set(11, dm_motor[1]->measure.speed_rads);
        chassis_debug_set(12, leg[LEFT]->d_phi0);
        chassis_debug_set(13, leg[RIGHT]->d_phi0);
        chassis_debug_set(14, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(15, leg[RIGHT]->d_l0_lpf);
        return 1U;
    }

    if (page == 25U) {
        dm_motor_object_t *page_motor[4] = {
            dm_motor[0], dm_motor[3], dm_motor[1], dm_motor[2]
        };

        /* Four complete DM frames packed as 16 unsigned 16-bit words. */
        for (uint8_t motor_idx = 0U; motor_idx < 4U; motor_idx++) {
            for (uint8_t word_idx = 0U; word_idx < 4U; word_idx++) {
                uint8_t byte_idx = (uint8_t)(word_idx * 2U);
                uint16_t word =
                    ((uint16_t)page_motor[motor_idx]->measure.raw_data[byte_idx] << 8U) |
                    page_motor[motor_idx]->measure.raw_data[byte_idx + 1U];
                chassis_debug_set((uint8_t)(motor_idx * 4U + word_idx),
                                  (float)word);
            }
        }
        return 1U;
    }

    if (page == 26U) {
        float theta_rate_l = kin_rate_observer[LEFT].theta_rate;
        float theta_rate_r = kin_rate_observer[RIGHT].theta_rate;
        float l0_rate_l = kin_rate_observer[LEFT].l0_rate;
        float l0_rate_r = kin_rate_observer[RIGHT].l0_rate;

        /* Legacy versus timestamped five-sample observer, I0-I15. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_kf_dt_s * 1000.0f);
        chassis_debug_set(3, ins.gyro[0]);
        chassis_debug_set(4, leg[LEFT]->d_theta_lpf);
        chassis_debug_set(5, theta_rate_l);
        chassis_debug_set(6, leg[RIGHT]->d_theta_lpf);
        chassis_debug_set(7, theta_rate_r);
        chassis_debug_set(8, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(9, l0_rate_l);
        chassis_debug_set(10, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(11, l0_rate_r);
        chassis_debug_set(12, 0.5f * (wheel_to_ground_legacy_l +
                                      wheel_to_ground_legacy_r));
        chassis_debug_set(13, 0.5f * (wheel_to_ground_candidate_l +
                                      wheel_to_ground_candidate_r));
        chassis_debug_set(14, chassis_vx_filter);
        chassis_debug_set(15, wheel_to_ground_candidate_l -
                              wheel_to_ground_candidate_r);
        return 1U;
    }

    if (page == 27U) {
        /* Normal-mode stability after the velocity-observer handoff, I0-I15. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, LQRXObsBuf[LEFT][5]);
        chassis_debug_set(6, ins.roll);
        chassis_debug_set(7, dbg_yaw_rate_fdb);
        chassis_debug_set(8, chassis_vx_filter);
        chassis_debug_set(9, wheel_to_ground_candidate_l -
                             wheel_to_ground_candidate_r);
        chassis_debug_set(10, leg[LEFT]->theta);
        chassis_debug_set(11, leg[RIGHT]->theta);
        chassis_debug_set(12, leg[LEFT]->d_theta_lpf);
        chassis_debug_set(13, leg[RIGHT]->d_theta_lpf);
        chassis_debug_set(14, leg[LEFT]->Tp);
        chassis_debug_set(15, leg[RIGHT]->Tp);
        return 1U;
    }

    if (page == 28U) {
        /* Differential swing damping validation, exactly I0-I15.
         * I0=time, I1=mode+10*leg level, I2/I3=vx/yaw commands;
         * I4/I5=pitch/roll, I6=theta L-R, I7/I8=candidate theta rates;
         * I9=rate L-R, I10/I11=sync P/D, I12/I13=base LQR hip commands;
         * I14/I15=final left/right hip commands. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, ins.roll);
        chassis_debug_set(6, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(7, kin_rate_observer[LEFT].theta_rate);
        chassis_debug_set(8, kin_rate_observer[RIGHT].theta_rate);
        chassis_debug_set(9, dbg_theta_rate_diff);
        chassis_debug_set(10, dbg_theta_sync_p);
        chassis_debug_set(11, dbg_theta_sync_d);
        chassis_debug_set(12, -LQROutBuf[LEFT][1]);
        chassis_debug_set(13, -LQROutBuf[RIGHT][1]);
        chassis_debug_set(14, leg[LEFT]->Tp);
        chassis_debug_set(15, leg[RIGHT]->Tp);
        return 1U;
    }

    if (page == 29U) {
        /* High-leg roll, split and wheel-mix diagnosis, exactly I0-I15.
         * I0=time, I1=mode+10*leg level, I2/I3=vx/yaw commands;
         * I4/I5=pitch/roll, I6=applied roll force, I7=average leg length;
         * I8=left-right leg length, I9=left-right theta, I10=length-force diff;
         * I11=estimated vx, I12/I13=unmixed local wheel LQR commands;
         * I14/I15=final mixed left/right wheel commands. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, ins.roll);
        chassis_debug_set(6, F_roll);
        chassis_debug_set(7, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(8, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(9, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(10, F_l_L - F_l_R);
        chassis_debug_set(11, chassis_vx_filter);
        chassis_debug_set(12, -LQROutBuf[LEFT][0]);
        chassis_debug_set(13, LQROutBuf[RIGHT][0]);
        chassis_debug_set(14, dbg_net_cmd_l);
        chassis_debug_set(15, dbg_net_cmd_r);
        return 1U;
    }

    if (page == 30U) {
        /* Whole-body roll gain scheduling validation, exactly I0-I15.
         * I0=time, I1=mode+10*leg level, I2/I3=vx/yaw commands;
         * I4/I5=pitch/roll, I6=average actual leg length, I7=left-right length;
         * I8=left-right theta, I9/I10=active roll Kp/Kd, I11=roll force;
         * I12=length-force difference, I13=estimated vx;
         * I14/I15=unmixed local left/right wheel LQR commands. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, ins.roll);
        chassis_debug_set(6, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(7, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(8, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(9, roll_pid->Kp);
        chassis_debug_set(10, roll_pid->Kd);
        chassis_debug_set(11, F_roll);
        chassis_debug_set(12, F_l_L - F_l_R);
        chassis_debug_set(13, chassis_vx_filter);
        chassis_debug_set(14, -LQROutBuf[LEFT][0]);
        chassis_debug_set(15, LQROutBuf[RIGHT][0]);
        return 1U;
    }

    if (page == 31U) {
        /* Yaw stall and local LQR residual validation, exactly I0-I15.
         * I0=time, I1=mode+10*leg level, I2/I3=vx/yaw commands;
         * I4/I5=pitch/roll, I6=length difference, I7=theta difference;
         * I8/I9=yaw-rate reference/feedback, I10=yaw output;
         * I11=state+10*blocked+80*priority, I12/I13=raw/applied residual;
         * I14/I15=final left/right wheel commands. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, ins.roll);
        chassis_debug_set(6, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(7, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(8, dbg_yaw_rate_ref);
        chassis_debug_set(9, dbg_yaw_rate_fdb);
        chassis_debug_set(10, dbg_yaw_ctrl_output);
        chassis_debug_set(11, (float)dbg_yaw_state +
                              (dbg_yaw_blocked ? 10.0f : 0.0f) +
                              (yaw_priority_should_override_lqr() ? 80.0f : 0.0f));
        chassis_debug_set(12, dbg_lqr_local_residual_raw);
        chassis_debug_set(13, dbg_lqr_local_residual_applied);
        chassis_debug_set(14, dbg_net_cmd_l);
        chassis_debug_set(15, dbg_net_cmd_r);
        return 1U;
    }

    if (page == 32U) {
        /* Hip differential stabilization and yaw symmetry, exactly I0-I15.
         * I0=time, I1=mode+10*leg level, I2/I3=vx/yaw commands;
         * I4/I5=pitch/roll, I6=average length, I7=left-right length;
         * I8=left-right theta, I9=left-right candidate theta rate;
         * I10=active theta Kp, I11/I12=theta P/D torque;
         * I13/I14=yaw-rate reference/feedback, I15=yaw output. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, ins.roll);
        chassis_debug_set(6, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(7, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(8, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(9, dbg_theta_rate_diff);
        chassis_debug_set(10, dbg_theta_sync_kp);
        chassis_debug_set(11, dbg_theta_sync_p);
        chassis_debug_set(12, dbg_theta_sync_d);
        chassis_debug_set(13, dbg_yaw_rate_ref);
        chassis_debug_set(14, dbg_yaw_rate_fdb);
        chassis_debug_set(15, dbg_yaw_ctrl_output);
        return 1U;
    }

    if (page == 33U) {
        /* Roll trim, high-leg split guard and mid-leg shake, exactly I0-I15.
         * I0=time, I1=mode+10*leg level, I2/I3=vx/yaw commands;
         * I4/I5=pitch/pitch-rate, I6=roll, I7=average actual length;
         * I8=left-right length, I9=average length-loop force;
         * I10/I11=wheel/hip LQR common output, I12=stuck-force average;
         * I13=left-right theta, I14=nonlinear theta guard, I15=roll trim. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, LQRXObsBuf[LEFT][5]);
        chassis_debug_set(6, ins.roll);
        chassis_debug_set(7, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(8, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(9, 0.5f * (F_l_L + F_l_R));
        chassis_debug_set(10, 0.5f * (LQROutBuf[LEFT][0] +
                                      LQROutBuf[RIGHT][0]));
        chassis_debug_set(11, 0.5f * (LQROutBuf[LEFT][1] +
                                      LQROutBuf[RIGHT][1]));
        chassis_debug_set(12, 0.5f * (leg_stuck_force[LEFT] +
                                      leg_stuck_force[RIGHT]));
        chassis_debug_set(13, leg[LEFT]->theta - leg[RIGHT]->theta);
        chassis_debug_set(14, dbg_theta_sync_guard);
        chassis_debug_set(15, roll_static_trim_force);
        return 1U;
    }

    if (page == 34U) {
        /* Length switching and structural-mode validation, exactly I0-I15.
         * I0=time, I1=mode+10*level+100*ramping+1000*K-tail-waiting;
         * I2/I3=vx/yaw commands; I4/I5=pitch/raw pitch-rate;
         * I6/I7=average target/actual length;
         * I8=left-right length, I9=average length-loop force;
         * I10/I11=left/right length-loop force, I12=notched LQR pitch-rate;
         * I13/I14=wheel/hip LQR common output, I15=average candidate l0 rate. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f * (float)dbg_lqr_k_tail_waiting);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_yaw_raw_rc_set);
        chassis_debug_set(4, ins.pitch);
        chassis_debug_set(5, dbg_lqr_dphi_raw);
        chassis_debug_set(6, 0.5f * (leg[LEFT]->length_ref +
                                     leg[RIGHT]->length_ref));
        chassis_debug_set(7, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(8, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(9, 0.5f * (F_l_L + F_l_R));
        chassis_debug_set(10, F_l_L);
        chassis_debug_set(11, F_l_R);
        chassis_debug_set(12, dbg_lqr_dphi_filtered);
        chassis_debug_set(13, 0.5f * (LQROutBuf[LEFT][0] +
                                      LQROutBuf[RIGHT][0]));
        chassis_debug_set(14, 0.5f * (LQROutBuf[LEFT][1] +
                                      LQROutBuf[RIGHT][1]));
        chassis_debug_set(15, 0.5f * (kin_rate_observer[LEFT].l0_rate +
                                      kin_rate_observer[RIGHT].l0_rate));
        return 1U;
    }

    if (page == 35U) {
        /* Theta compensation and zero-command drift, exactly I0-I15.
         * I0=time, I1=mode+10*level, I2=vx command, I3=filtered vx;
         * I4=diagnostic drift distance, I5/I6=pitch/pitch-rate;
         * I7=actual average length, I8=raw average theta;
         * I9=average theta presented to LQR after compensation;
         * I10=polynomial base, I11=active level trim, I12=base+trim;
         * I13=application gain, I14=average final total compensation;
         * I15=calibration/apply/learn condition flags. */
        const float raw_theta_average =
            0.5f * (leg[LEFT]->theta + leg[RIGHT]->theta);
        const float lqr_theta_average =
            0.5f * (LQRXObsBuf[LEFT][0] + LQRXObsBuf[RIGHT][0]);

        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_theta_vx_filter);
        chassis_debug_set(4, dbg_theta_drift_x);
        chassis_debug_set(5, ins.pitch);
        chassis_debug_set(6, LQRXObsBuf[LEFT][5]);
        chassis_debug_set(7, dbg_theta_l0_average);
        chassis_debug_set(8, raw_theta_average);
        chassis_debug_set(9, lqr_theta_average);
        chassis_debug_set(10, dbg_theta_poly_base);
        chassis_debug_set(11, dbg_theta_active_trim);
        chassis_debug_set(12, dbg_theta_active_comp);
        chassis_debug_set(13, dbg_theta_level_apply_gain);
        chassis_debug_set(14, raw_theta_average - lqr_theta_average);
        chassis_debug_set(15, dbg_theta_cal_flags);
        return 1U;
    }

    if (page == 36U) {
        /* Stop-state and theta-learning verification, exactly I0-I15.
         * I0=time, I1=mode+10*level, I2=vx command, I3=filtered vx;
         * I4=virtual position, I5=0 DRIVE/1 BRAKE/2 HOLD;
         * I6=position capture timer, I7=theta learning settle timer;
         * I8=actual average length, I9=polynomial base;
         * I10=actual-length interpolated trim, I11=base+trim;
         * I12=application gain, I13=final theta compensation;
         * I14=common LQR wheel output, I15=condition flags. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_theta_vx_filter);
        chassis_debug_set(4, 0.5f *
                             (lqr_virtual_x_l + lqr_virtual_x_r));
        chassis_debug_set(5, (float)lqr_position_state);
        chassis_debug_set(6, lqr_position_capture_timer);
        chassis_debug_set(7, theta_comp_learn_settle_timer);
        chassis_debug_set(8, dbg_theta_l0_average);
        chassis_debug_set(9, dbg_theta_poly_base);
        chassis_debug_set(10, dbg_theta_active_trim);
        chassis_debug_set(11, dbg_theta_active_comp);
        chassis_debug_set(12, dbg_theta_level_apply_gain);
        chassis_debug_set(13, dbg_theta_total_comp_L);
        chassis_debug_set(14, 0.5f *
                              (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]));
        chassis_debug_set(15, dbg_theta_cal_flags);
        return 1U;
    }

    if (page == 37U) {
        /* LOW fallback isolation and leg-switch verification, I0-I15 only.
         * I0=time, I1=mode+10*level, I2=vx command, I3=filtered vx;
         * I4=actual average length, I5=leg-length ramp state;
         * I6=learning freeze timer, I7=LOW convergence timer;
         * I8=polynomial base, I9=strict interpolated trim;
         * I10=stored LOW fallback, I11=LOW fallback gain;
         * I12=total active compensation, I13=final left compensation;
         * I14=common LQR wheel output, I15=condition flags. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, dbg_theta_vx_filter);
        chassis_debug_set(4, dbg_theta_l0_average);
        chassis_debug_set(5, (float)chassis_cmd.leg_leng_change);
        chassis_debug_set(6, leg_switch_freeze_timer);
        chassis_debug_set(7, theta_low_drift_converge_timer);
        chassis_debug_set(8, dbg_theta_poly_base);
        chassis_debug_set(9, dbg_theta_strict_trim);
        chassis_debug_set(10, theta_low_drift_trim);
        chassis_debug_set(11, theta_low_drift_apply_gain);
        chassis_debug_set(12, dbg_theta_active_comp);
        chassis_debug_set(13, dbg_theta_total_comp_L);
        chassis_debug_set(14, 0.5f *
                              (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]));
        chassis_debug_set(15, dbg_theta_cal_flags);
        return 1U;
    }

    if (page == 38U) {
        /* Leg-stuck trigger and PID verification, exactly I0-I15.
         * I0=time; I1=mode+10*level+100*ramping+1000*K-tail-waiting;
         * I2=average target; I3/I4=left/right actual length;
         * I5/I6=left/right legacy single-cycle length rate;
         * I7/I8=left/right timestamped observer length rate;
         * I9/I10=left/right final length-loop force;
         * I11/I12=left/right integral output;
         * I13/I14=left/right stuck compensation force;
         * I15=post-ramp compensation-window timer. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f * (float)dbg_lqr_k_tail_waiting);
        chassis_debug_set(2, 0.5f *
                             (leg[LEFT]->length_ref +
                              leg[RIGHT]->length_ref));
        chassis_debug_set(3, leg[LEFT]->l0);
        chassis_debug_set(4, leg[RIGHT]->l0);
        chassis_debug_set(5, leg[LEFT]->d_l0_lpf);
        chassis_debug_set(6, leg[RIGHT]->d_l0_lpf);
        chassis_debug_set(7, kin_rate_observer[LEFT].l0_rate);
        chassis_debug_set(8, kin_rate_observer[RIGHT].l0_rate);
        chassis_debug_set(9, F_l_L);
        chassis_debug_set(10, F_l_R);
        chassis_debug_set(11, L_length_pid->Iout);
        chassis_debug_set(12, R_length_pid->Iout);
        chassis_debug_set(13, leg_stuck_force[LEFT]);
        chassis_debug_set(14, leg_stuck_force[RIGHT]);
        chassis_debug_set(15, leg_stuck_post_ramp_timer);
        return 1U;
    }

    if (page == 39U) {
        /* Agility step: speed/yaw/roll/leg verification, I0-I15.
         * I0=time; I1=mode+10*level+100*ramping+1000*K-tail-waiting;
         * I2=raw vx set; I3=limited vx reference; I4=active vx limit;
         * I5=raw yaw set; I6/I7=yaw-rate reference/feedback;
         * I8=yaw output; I9=active yaw-rate limit; I10=roll; I11=F_roll.
         * Normal: I12/I13=left/right length, I14=average stuck force,
         * I15=post-ramp timer. Recovery is intercepted by the recovery module
         * and reports its state plus reference/actual swing rates instead. */
        const uint8_t recovery_debug =
            chassis_cmd.ctrl_mode == CHASSIS_RECOVERY;
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f * (float)dbg_lqr_k_tail_waiting);
        chassis_debug_set(2, chassis_cmd.vx_set);
        chassis_debug_set(3, LQRXRefBuf[LEFT][3]);
        chassis_debug_set(4, dbg_chassis_vx_limit);
        chassis_debug_set(5, dbg_yaw_raw_rc_set);
        chassis_debug_set(6, dbg_yaw_rate_ref);
        chassis_debug_set(7, dbg_yaw_rate_fdb);
        chassis_debug_set(8, dbg_yaw_ctrl_output);
        chassis_debug_set(9, dbg_yaw_rate_limit);
        chassis_debug_set(10, ins.roll);
        chassis_debug_set(11, F_roll);
        chassis_debug_set(12, recovery_debug ?
                              leg[LEFT]->phi0 : leg[LEFT]->l0);
        chassis_debug_set(13, recovery_debug ?
                              leg[RIGHT]->phi0 : leg[RIGHT]->l0);
        chassis_debug_set(14, recovery_debug ?
                              leg[LEFT]->d_phi0 :
                              0.5f * (leg_stuck_force[LEFT] +
                                      leg_stuck_force[RIGHT]));
        chassis_debug_set(15, recovery_debug ?
                              leg[RIGHT]->d_phi0 :
                              leg_stuck_post_ramp_timer);
        return 1U;
    }

    if (page == 40U) {
        /* OPEN_LOOP yaw-to-roll shake validation, exactly I0-I15:
         * I0=time; I1=mode+10*level+100*ramping+1000*K-tail-waiting;
         * I2=limited vx reference; I3/I4=yaw-rate reference/feedback;
         * I5=yaw output; I6/I7=roll/roll rate; I8/I9=roll Kp/Kd;
         * I10=roll PD output; I11=stored trim; I12=applied trim;
         * I13=final F_roll; I14=average leg length; I15=left-right length. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f * (float)dbg_lqr_k_tail_waiting);
        chassis_debug_set(2, LQRXRefBuf[LEFT][3]);
        chassis_debug_set(3, dbg_yaw_rate_ref);
        chassis_debug_set(4, dbg_yaw_rate_fdb);
        chassis_debug_set(5, dbg_yaw_ctrl_output);
        chassis_debug_set(6, ins.roll);
        chassis_debug_set(7, ins.gyro[0]);
        chassis_debug_set(8, roll_pid->Kp);
        chassis_debug_set(9, roll_pid->Kd);
        chassis_debug_set(10, dbg_roll_pd_output);
        chassis_debug_set(11, roll_static_trim_force);
        chassis_debug_set(12, roll_static_trim_applied);
        chassis_debug_set(13, F_roll);
        chassis_debug_set(14, 0.5f * (leg[LEFT]->l0 + leg[RIGHT]->l0));
        chassis_debug_set(15, leg[LEFT]->l0 - leg[RIGHT]->l0);
        return 1U;
    }

    if (page == 41U) {
        /* Directional yaw, Roll split guard and left-leg resonance check:
         * I0=time; I1=mode+10*level+100*ramping+1000*K-tail-waiting;
         * I2=vx reference; I3/I4=yaw-rate reference/feedback;
         * I5=yaw controller output; I6=reserved (1.0);
         * I7=roll; I8=raw F_roll; I9=reserved (1.0);
         * I10=final F_roll; I11/I12=left/right length;
         * I13/I14=left/right final length-loop force;
         * I15=timestamped left length rate. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f * (float)dbg_lqr_k_tail_waiting);
        chassis_debug_set(2, LQRXRefBuf[LEFT][3]);
        chassis_debug_set(3, dbg_yaw_rate_ref);
        chassis_debug_set(4, dbg_yaw_rate_fdb);
        chassis_debug_set(5, dbg_yaw_ctrl_output);
        chassis_debug_set(6, 1.0f);
        chassis_debug_set(7, ins.roll);
        chassis_debug_set(8, dbg_roll_pre_split_guard);
        chassis_debug_set(9, 1.0f);
        chassis_debug_set(10, F_roll);
        chassis_debug_set(11, leg[LEFT]->l0);
        chassis_debug_set(12, leg[RIGHT]->l0);
        chassis_debug_set(13, F_l_L);
        chassis_debug_set(14, F_l_R);
        chassis_debug_set(15, kin_rate_observer[LEFT].l0_rate);
        return 1U;
    }

    if (page == 42U) {
        /* Dynamic support-force feedforward verification, exactly I0-I15:
         * I0=time; I1=mode+10*level+100*ramping+1000*off-ground;
         * I2/I3=vx reference/feedback; I4/I5=left/right raw force ref;
         * I6/I7=left/right filtered force ref; I8/I9=left/right ref dot;
         * I10/I11=left/right ref ddot; I12/I13=applied correction;
         * I14/I15=left/right final support force. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f *
                                 (float)chassis_safety_module_off_ground());
        chassis_debug_set(2, LQRXRefBuf[LEFT][3]);
        chassis_debug_set(3, chassis_vx_filter);
        chassis_debug_set(4, dbg_support_ff_raw_ref[LEFT]);
        chassis_debug_set(5, dbg_support_ff_raw_ref[RIGHT]);
        chassis_debug_set(6, support_force_ff_state[LEFT].ref_lpf);
        chassis_debug_set(7, support_force_ff_state[RIGHT].ref_lpf);
        chassis_debug_set(8, support_force_ff_state[LEFT].ref_dot);
        chassis_debug_set(9, support_force_ff_state[RIGHT].ref_dot);
        chassis_debug_set(10, support_force_ff_state[LEFT].ref_ddot);
        chassis_debug_set(11, support_force_ff_state[RIGHT].ref_ddot);
        chassis_debug_set(12, dbg_support_ff_correction[LEFT]);
        chassis_debug_set(13, dbg_support_ff_correction[RIGHT]);
        chassis_debug_set(14, leg[LEFT]->support_force);
        chassis_debug_set(15, leg[RIGHT]->support_force);
        return 1U;
    }

    if (page == 43U) {
        /* Geometric Roll and centrifugal feedforward verification, I0-I31:
         * I0=time; I1=mode+10*level+100*ramping+1000*off-ground;
         * I2/I3=vx ref/fdb; I4/I5=yaw-rate ref/fdb;
         * I6=roll; I7=legacy ins.gyro[0] value (preserved, not Roll rate);
         * I8=estimated tan(slope);
         * I9/I10=target/applied half leg-length difference;
         * I11=actual left-right length; I12=Roll feedback force;
         * I13=centrifugal target; I14=raw applied centrifugal force; I15=final
         * combined force after normal-ground Roll feedback-priority gating;
         * I16=Roll rate; I17/I18/I19=reference/actual/selected lateral accel;
         * I20=coast elapsed seconds; I21 low 6 bits=coast active+
         * 2*started+4*locked+8*mode-allowed+16*observer-ready+
         * 32*samples-fresh; I21=low6+64*first lock reason
         * (0=none,1=dt,2=timeout,3=feedback-invalid,4=out-of-range,
         * 5=actual-reverse);
         * I22=final measured-feedback validity; I23=captured reference accel peak;
         * I24=Roll PID output; I25/I26=learned/applied static trim;
         * I27=pre-limit Roll software force; I28=INS sample age ms;
         * I29=Roll PID MaxOut; I30/I31=max wheel/hip sample age in ms.
         * I12 remains the final clamped Roll force. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level +
                             100.0f * (float)chassis_cmd.leg_leng_change +
                             1000.0f *
                                 (float)chassis_safety_module_off_ground());
        chassis_debug_set(2, LQRXRefBuf[LEFT][3]);
        chassis_debug_set(3, chassis_vx_filter);
        chassis_debug_set(4, dbg_yaw_rate_ref);
        chassis_debug_set(5, dbg_yaw_rate_fdb);
        chassis_debug_set(6, ins.roll);
        chassis_debug_set(7, ins.gyro[0]);
        chassis_debug_set(8, dbg_roll_slope_tan);
        chassis_debug_set(9, roll_length_offset_target);
        chassis_debug_set(10, roll_length_offset);
        chassis_debug_set(11, leg[LEFT]->l0 - leg[RIGHT]->l0);
        chassis_debug_set(12, F_roll);
        chassis_debug_set(13, dbg_lateral_ff_target);
        chassis_debug_set(14, F_bl_intertial);
        chassis_debug_set(15, F_lateral);
        {
            const uint32_t now_ms = HAL_GetTick();
            float ins_age_ms = -1.0f;
            float max_wheel_age_ms = -1.0f;
            float max_hip_age_ms = -1.0f;
            if (control_context != NULL) {
                const struct ins_msg *ins_sample = &ins;
                if (ins_sample != NULL) {
                    ins_age_ms = lateral_inertial_ff_sample_age_ms(
                        now_ms, ins_sample->publish_tick_ms,
                        ins_sample->publish_count);
                }
            }
            uint8_t wheel_ages_valid = control_context != NULL &&
                control_context->wheel_feedback != NULL;
            uint8_t hip_ages_valid = 1U;
            for (uint8_t side = LEFT; side < CHASSIS_SIDE_COUNT; side++) {
                if (!wheel_ages_valid ||
                    control_context->wheel_feedback[side].rx_count == 0U) {
                    wheel_ages_valid = 0U;
                    break;
                }
                const float age_ms = lateral_inertial_ff_sample_age_ms(
                    now_ms,
                    control_context->wheel_feedback[side].last_rx_ms,
                    control_context->wheel_feedback[side].rx_count);
                if (!isfinite(age_ms) || age_ms < 0.0f) {
                    wheel_ages_valid = 0U;
                    break;
                }
                if (age_ms > max_wheel_age_ms) {
                    max_wheel_age_ms = age_ms;
                }
            }
            for (uint8_t motor = 0U; motor < 4U; motor++) {
                if (dm_motor[motor] == NULL ||
                    dm_motor[motor]->measure.rx_count == 0U) {
                    hip_ages_valid = 0U;
                    break;
                }
                const float age_ms = lateral_inertial_ff_sample_age_ms(
                    now_ms, dm_motor[motor]->measure.last_rx_ms,
                    dm_motor[motor]->measure.rx_count);
                if (!isfinite(age_ms) || age_ms < 0.0f) {
                    hip_ages_valid = 0U;
                    break;
                }
                if (age_ms > max_hip_age_ms) {
                    max_hip_age_ms = age_ms;
                }
            }
            chassis_debug_set(16, -ins.gyro[1]);
            chassis_debug_set(17, lateral_ff_state.reference_accel);
            chassis_debug_set(18, lateral_ff_state.actual_accel);
            chassis_debug_set(19, lateral_ff_state.selected_accel);
            chassis_debug_set(20, lateral_ff_state.coast_elapsed_s);
            chassis_debug_set(21,
                64.0f * (float)lateral_ff_state.coast_lock_reason +
                (float)lateral_ff_state.coast_active +
                2.0f * (float)lateral_ff_state.coast_started +
                4.0f * (float)lateral_ff_state.coast_locked +
                8.0f * (float)dbg_lateral_feedback_mode_allowed +
                16.0f * (float)dbg_lateral_observer_ready +
                32.0f * (float)dbg_lateral_feedback_fresh);
            chassis_debug_set(22, (float)lateral_ff_state.feedback_valid);
            chassis_debug_set(23, lateral_ff_state.ref_accel_peak);
            chassis_debug_set(24, dbg_roll_pd_output);
            chassis_debug_set(25, roll_static_trim_force);
            chassis_debug_set(26, roll_static_trim_applied);
            chassis_debug_set(27, dbg_roll_pre_limit_force);
            chassis_debug_set(28, ins_age_ms);
            chassis_debug_set(29, roll_pid->MaxOut);
            chassis_debug_set(30,
                wheel_ages_valid ? max_wheel_age_ms : -1.0f);
            chassis_debug_set(31,
                hip_ages_valid ? max_hip_age_ms : -1.0f);
        }
        return 1U;
    }

    if (page == 12U) {
        /* Theta calibration, exactly I0-I15.
         * I0=time; I1=average target length; I2=average measured length;
         * I3/I4=left/right measured length; I5=active compensation;
         * I6/I7=left/right length PID output; I8/I9=left/right integral output;
         * I10=zero-speed hold timer; I11=calibration vx LPF;
         * I12=calibration vx trend; I13=filtered vx;
         * I14=learning enable; I15=flags. */
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, 0.5f * (leg[LEFT]->length_ref + leg[RIGHT]->length_ref));
        chassis_debug_set(2, dbg_theta_l0_average);
        chassis_debug_set(3, leg[LEFT]->l0);
        chassis_debug_set(4, leg[RIGHT]->l0);
        chassis_debug_set(5, dbg_theta_active_comp);
        chassis_debug_set(6, L_length_pid->Output);
        chassis_debug_set(7, R_length_pid->Output);
        chassis_debug_set(8, L_length_pid->Iout);
        chassis_debug_set(9, R_length_pid->Iout);
        chassis_debug_set(10, theta_cal_zero_hold_timer);
        chassis_debug_set(11, theta_cal_vx_lpf);
        chassis_debug_set(12, theta_cal_vx_trend);
        chassis_debug_set(13, dbg_theta_vx_filter);
        chassis_debug_set(14, (float)dbg_theta_level_learn_enable);
        chassis_debug_set(15, dbg_theta_cal_flags);
        return 1U;
    }

    return 0U;
}

uint8_t chassis_control_module_fill_jump_rate_debug_page(uint16_t status)
{
    chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
    chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                         10.0f * (float)chassis_fdb_data.stand_state +
                         100.0f * (float)jump_state);
    chassis_debug_set(2, (float)(jump_fault +
                         10U * chassis_safety_module_danger_reason()));
    chassis_debug_set(3, (float)status);
    chassis_debug_set(4, jump_cycle_raw_dt_ms);

    uint16_t observer_status = 0U;
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    if (jump_kin_rates_usable) observer_status |= 1U << 0;
    if (jump_kin_rates_fresh) observer_status |= 1U << 1;
#endif
    if (JUMP_LOCAL_RATE_FEEDBACK_ENABLE &&
        JUMP_KINEMATIC_RATE_OBSERVER_ENABLE) {
        observer_status |= 1U << 2;
    }
    if (chassis_safety_module_landing_buffer_jump_origin()) {
        observer_status |= 1U << 3;
    }
    chassis_debug_set(5, (float)observer_status);
    chassis_debug_set(6, leg[LEFT] != NULL ? leg[LEFT]->l0 : 0.0f);
    chassis_debug_set(7, leg[RIGHT] != NULL ? leg[RIGHT]->l0 : 0.0f);
    chassis_debug_set(8, leg[LEFT] != NULL ? leg[LEFT]->theta : 0.0f);
    chassis_debug_set(9, leg[RIGHT] != NULL ? leg[RIGHT]->theta : 0.0f);
    chassis_debug_set(10,
        jump_kin_rates_usable ? jump_kin_theta_rate[LEFT] : 0.0f);
    chassis_debug_set(11,
        jump_kin_rates_usable ? jump_kin_theta_rate[RIGHT] : 0.0f);
    chassis_debug_set(12, leg[LEFT] != NULL ? leg[LEFT]->d_theta_lpf : 0.0f);
    chassis_debug_set(13, leg[RIGHT] != NULL ? leg[RIGHT]->d_theta_lpf : 0.0f);
    chassis_debug_set(14,
        jump_kin_rates_usable ? jump_kin_length_rate[LEFT] : 0.0f);
    chassis_debug_set(15,
        jump_kin_rates_usable ? jump_kin_length_rate[RIGHT] : 0.0f);
    chassis_debug_set(16, roll_length_target_l);
    chassis_debug_set(17, roll_length_target_r);
    chassis_debug_set(18, L_length_pid != NULL ? L_length_pid->Pout : 0.0f);
    chassis_debug_set(19, R_length_pid != NULL ? R_length_pid->Pout : 0.0f);
    chassis_debug_set(20, L_length_pid != NULL ? L_length_pid->Dout : 0.0f);
    chassis_debug_set(21, R_length_pid != NULL ? R_length_pid->Dout : 0.0f);
    chassis_debug_set(22, L_length_pid != NULL ? L_length_pid->Output : 0.0f);
    chassis_debug_set(23, R_length_pid != NULL ? R_length_pid->Output : 0.0f);
    chassis_debug_set(24, F_roll);
    chassis_debug_set(25, jump_length_sync_force);
    chassis_debug_set(26, leg[LEFT] != NULL ? leg[LEFT]->Tp : 0.0f);
    chassis_debug_set(27, leg[RIGHT] != NULL ? leg[RIGHT]->Tp : 0.0f);
    chassis_debug_set(28,
        leg[LEFT] != NULL ? leg[LEFT]->support_force : 0.0f);
    chassis_debug_set(29,
        leg[RIGHT] != NULL ? leg[RIGHT]->support_force : 0.0f);
    chassis_debug_set(30, ins.roll);
    chassis_debug_set(31, ins.pitch);
    return 1U;
}

uint8_t chassis_control_module_fill_jump_handoff_debug_page(uint16_t status)
{
    uint16_t observer_status = 0U;
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    if (jump_kin_rates_usable) observer_status |= 1U << 0;
    if (jump_kin_rates_fresh) observer_status |= 1U << 1;
#endif
    if (JUMP_LOCAL_RATE_FEEDBACK_ENABLE &&
        JUMP_KINEMATIC_RATE_OBSERVER_ENABLE) observer_status |= 1U << 2;
    if (chassis_safety_module_landing_buffer_jump_origin()) {
        observer_status |= 1U << 3;
    }
    observer_status |= (uint16_t)(dbg_jump_handoff_wheel_source & 0x07U) << 4;

    chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
    chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                         10.0f * (float)chassis_fdb_data.stand_state +
                         100.0f * (float)jump_state);
    chassis_debug_set(2, (float)(jump_fault +
                         10U * chassis_safety_module_danger_reason()));
    chassis_debug_set(3, (float)status);
    chassis_debug_set(4, jump_cycle_raw_dt_ms);
    chassis_debug_set(5, (float)observer_status);
    chassis_debug_set(6, leg[LEFT] != NULL ? leg[LEFT]->l0 : 0.0f);
    chassis_debug_set(7, leg[RIGHT] != NULL ? leg[RIGHT]->l0 : 0.0f);
    chassis_debug_set(8, leg[LEFT] != NULL ? leg[LEFT]->theta : 0.0f);
    chassis_debug_set(9, leg[RIGHT] != NULL ? leg[RIGHT]->theta : 0.0f);
    chassis_debug_set(10,
        jump_kin_rates_usable ? jump_kin_theta_rate[LEFT] : 0.0f);
    chassis_debug_set(11,
        jump_kin_rates_usable ? jump_kin_theta_rate[RIGHT] : 0.0f);
    chassis_debug_set(12,
        -(float)dbg_wheel_set_l / M3508_TOR_TO_CUR);
    chassis_debug_set(13,
        +(float)dbg_wheel_set_r / M3508_TOR_TO_CUR);
    chassis_debug_set(14,
        jump_kin_rates_usable ? jump_kin_length_rate[LEFT] : 0.0f);
    chassis_debug_set(15,
        jump_kin_rates_usable ? jump_kin_length_rate[RIGHT] : 0.0f);
    chassis_debug_set(16, roll_length_target_l);
    chassis_debug_set(17, roll_length_target_r);
    chassis_debug_set(18,
        m3508_motor[LEFT] != NULL ?
        -(m3508_motor[LEFT]->measure.speed_aps /
          M3508_READUCTION_RATIO_L * DEGREE_2_RAD) : 0.0f);
    chassis_debug_set(19,
        m3508_motor[RIGHT] != NULL ?
        +(m3508_motor[RIGHT]->measure.speed_aps /
          M3508_READUCTION_RATIO_R * DEGREE_2_RAD) : 0.0f);
    chassis_debug_set(20, chassis_vx_filter);
    chassis_debug_set(21,
        0.5f * (LQRXRefBuf[LEFT][3] + LQRXRefBuf[RIGHT][3]));
    chassis_debug_set(22, jump_landing_base_tp[LEFT]);
    chassis_debug_set(23, jump_landing_base_tp[RIGHT]);
    chassis_debug_set(24, jump_landing_owner_gain);
    /* Updated by the most recent async wheel callback; Page59 may lag the
     * control task by one sample because the callbacks run independently. */
    chassis_debug_set(25, jump_landing_wheel_last_used_gain);
    chassis_debug_set(26, leg[LEFT] != NULL ? leg[LEFT]->Tp : 0.0f);
    chassis_debug_set(27, leg[RIGHT] != NULL ? leg[RIGHT]->Tp : 0.0f);
    chassis_debug_set(28,
        leg[LEFT] != NULL ? leg[LEFT]->support_force : 0.0f);
    chassis_debug_set(29,
        leg[RIGHT] != NULL ? leg[RIGHT]->support_force : 0.0f);
    chassis_debug_set(30, ins.roll);
    chassis_debug_set(31, ins.pitch);
    return 1U;
}

uint8_t chassis_control_module_fill_jump_capture_debug_page(uint16_t status)
{
    if (!chassis_control_module_fill_jump_handoff_debug_page(status)) {
        return 0U;
    }

    /* Page 60 preserves Page 59's frame and substitutes air-profile diagnostics.
     * I16/I17=safety air_length_target() snapshot (buffer refs while buffer is
     * active; otherwise internal air-profile refs, not guaranteed final PID
     * refs); I21=release-origin elapsed seconds; I22=body pitch rate rad/s;
     * I23=actual air-wheel pitch reference rad (zero before it is initialized). */
    chassis_debug_set(16,
        chassis_safety_module_air_length_target(CHASSIS_SIDE_LEFT));
    chassis_debug_set(17,
        chassis_safety_module_air_length_target(CHASSIS_SIDE_RIGHT));
    chassis_debug_set(21, chassis_safety_module_airborne_elapsed());
    chassis_debug_set(22, -LQRXObsBuf[LEFT][5]);
    chassis_debug_set(23, jump_air_wheel_pitch_ref_rad);
    return 1U;
}

uint8_t chassis_control_module_fill_jump_balance_debug_page(uint16_t status)
{
    if (!chassis_control_module_fill_jump_capture_debug_page(status)) {
        return 0U;
    }
    /* Page 61 keeps Page 60 channels except I4=clamped raw common LQR wheel
     * torque (Nm), I21=independent ground-balance elapsed seconds, and
     * I22/I23=unblended grounded base hip torques (L/R). I24/I25 remain the
     * hip beta and last asynchronously consumed wheel beta; source 6 denotes
     * their paired owner blend. */
    float common_wheel = 0.5f *
        (LQROutBuf[LEFT][0] + LQROutBuf[RIGHT][0]);
    LIMIT_MIN_MAX(common_wheel, -M3508_TOR_MAX, M3508_TOR_MAX);
    chassis_debug_set(4, common_wheel);
    chassis_debug_set(21, jump_landing_balance_state.ground_elapsed_s);
    chassis_debug_set(22, jump_landing_base_tp[LEFT]);
    chassis_debug_set(23, jump_landing_base_tp[RIGHT]);
    return 1U;
}

uint8_t chassis_control_module_fill_jump_ground_recovery_debug_page(
    uint16_t status)
{
    if (!chassis_control_module_fill_jump_balance_debug_page(status)) {
        return 0U;
    }
    /* Page62 v22 preserves Page61's status layout and most signals. I4 is the
     * cached allocated common wheel candidate; I10/11 are actual LQR theta-rate
     * observations; I14/15 use tuple-timestamped length rates in the landing
     * D window; I16/17 are effective air refs in flight/prestart and final
     * ground refs otherwise; I21 remains recovery gain. I22/I23 expose profile
     * elapsed time and the applied longitudinal LQR reference. */
    uint16_t observer_status = 0U;
#if JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
    if (jump_kin_rates_usable) observer_status |= 1U << 0;
    if (jump_kin_rates_fresh) observer_status |= 1U << 1;
#endif
    if (JUMP_LOCAL_RATE_FEEDBACK_ENABLE &&
        JUMP_KINEMATIC_RATE_OBSERVER_ENABLE) observer_status |= 1U << 2;
    if (chassis_safety_module_landing_buffer_jump_origin()) {
        observer_status |= 1U << 3;
    }
    observer_status |= (uint16_t)(dbg_jump_handoff_wheel_source & 0x07U) << 4;
    if (chassis_safety_module_jump_air_actual_compact()) {
        observer_status |= 1U << 7;
    }
    if (chassis_safety_module_jump_air_deploy_started()) {
        observer_status |= 1U << 8;
    }
    const uint8_t landing_rate_scope = (uint8_t)(
        chassis_safety_module_landing_buffer_jump_origin() ||
        chassis_safety_module_landing_postbuffer_active());
    if (landing_rate_scope && jump_landing_length_rate_valid) {
        observer_status |= 1U << 9;
    }
    if (jump_position_snapshot_fresh) observer_status |= 1U << 10;

    chassis_debug_set(4, jump_ground_wheel_common_cache);
    chassis_debug_set(5, (float)observer_status);
    chassis_debug_set(10, LQRXObsBuf[LEFT][1]);
    chassis_debug_set(11, LQRXObsBuf[RIGHT][1]);
    const uint8_t air_reference_active = (uint8_t)(
        !chassis_safety_module_landing_buffer_active() &&
        (chassis_safety_module_off_ground() ||
         chassis_safety_module_jump_air_profile_active()));
    chassis_debug_set(14, landing_rate_scope ?
        (jump_landing_length_rate_valid ?
            jump_landing_length_rate[LEFT] : 0.0f) :
        (jump_kin_rates_usable ? jump_kin_length_rate[LEFT] : 0.0f));
    chassis_debug_set(15, landing_rate_scope ?
        (jump_landing_length_rate_valid ?
            jump_landing_length_rate[RIGHT] : 0.0f) :
        (jump_kin_rates_usable ? jump_kin_length_rate[RIGHT] : 0.0f));
    chassis_debug_set(16, air_reference_active ?
        chassis_safety_module_air_length_target(CHASSIS_SIDE_LEFT) :
        roll_length_target_l);
    chassis_debug_set(17, air_reference_active ?
        chassis_safety_module_air_length_target(CHASSIS_SIDE_RIGHT) :
        roll_length_target_r);
    chassis_debug_set(21, jump_ground_recovery_gain);
    chassis_debug_set(22, chassis_safety_module_airborne_elapsed());
    chassis_debug_set(23,
        0.5f * (LQRXRefBuf[LEFT][3] + LQRXRefBuf[RIGHT][3]));
    return 1U;
}
