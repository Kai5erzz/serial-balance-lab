/*
* Change Logs:
* Date            Author          Notes
* 2023-09-24      ChuShicheng     first version
* 2025-07-25      SuperChen       second version
* 2025-11-01      SuperChen       third version (结合上交建模lqr)
* 2026-01-15      SuperChen       fourth version (尝试港大的仿真)
* 2026-01-20      SuperChen       fifth version (重回哈工程开源)
*/
#include "chassis_task.h"
#include "control/chassis_control.h"
#include "control/jump_stroke_release.h"
#include "control/jump_theta_diff_d_helpers.h"
#include "control/jump_torque_budget.h"
#include "debug/chassis_debug.h"
#include "recovery/chassis_recovery.h"
#include "safety/chassis_safety.h"
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"
#include "DM_motor.h"
#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1) && defined(SYSTEMVIEW_USE_UART) && (SYSTEMVIEW_USE_UART == 1))
#include "segger_sysview_uart.h"
#endif
extern float dm_obs[4];

/* Chassis task communication. */
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chassis_cmd;
// 发布
MCN_DECLARE(chassis_fdb);
static struct chassis_fdb_msg chassis_fdb_data;

static void chassis_sub_init(void);
static void chassis_pub_push(void);
static void chassis_sub_pull(void);
static uint8_t chassis_motor_fill_debug_page(uint8_t page);
static void jump_torque_budget_publish_vmc(void);

/* Latest software-limit event from each motor callback. Reads are not a
 * synchronized four-motor frame snapshot; bits identify the last callback
 * result observed for each physical joint. */
static volatile uint8_t dm_output_clip_latch[4];

/* --------------------------------- 电机控制相关 --------------------------------- */

#define LEFT    0
#define RIGHT   1
#define FRONT   0
#define BACK    1


#if THETA_LENGTH_CALIBRATION_ENABLE
/* First gimbal calibration group. The fit uses measured leg length; these
 * targets are deliberately close enough for the current length controller. */
#define LEN_LEN_LOW     THETA_COMP_CAL_TARGET_LOW
#define LEN_LEN_MID     THETA_COMP_CAL_TARGET_MID
#define LEN_LEN_HIG     THETA_COMP_CAL_TARGET_HIG
#else
#define LEN_LEN_LOW     0.13f // 单位：m
#define LEN_LEN_MID     0.21f // 单位：m
#define LEN_LEN_HIG     0.34f // 单位：m；操作手 LEG_HIG 档位高度（BUMP 另有自己的目标，见 BUMP_APPROACH_LENGTH=0.30f）
#endif
static float dm_v_set[2][2];
#define DM_V_SET  0.0f
static dm_motor_object_t *dm_motor[4];

/* 驱动电机 3508 实例 */
static dji_motor_object_t *m3508_motor[2];
/* 轮毂完整反馈帧快照，供 BUMP 接触判据与底盘反馈发布共用。 */
static dji_motor_measure_t wheel_feedback[2];

static chassis_control_context_t control_context;
static chassis_recovery_context_t recovery_context;
static chassis_safety_context_t safety_context;
static void dm_motor_init();
static int chassis_motor_init(void);
static void m3508_motor_init();

static void motor_relax();
/*电机使能*/
static void motor_enable();
/*设置电机零点*/
static void leg_init_get_zero();

void chassis_task_init(void)
{
    chassis_debug_module_init();
    chassis_sub_init();
    chassis_motor_init();
    control_context = (chassis_control_context_t){
        .dm_motor = {dm_motor[0], dm_motor[1], dm_motor[2], dm_motor[3]},
        .wheel_motor = {m3508_motor[LEFT], m3508_motor[RIGHT]},
        .wheel_feedback = wheel_feedback,
        .ins = &ins,
        .cmd = &chassis_cmd,
        .fdb = &chassis_fdb_data,
        .control_dt_s = 0.005f,
    };
    chassis_control_module_bind(&control_context);

    leg_config_t leg_config = {
        0.21f,
        0.250f,
        0.0f,
        300,
        0.0503f,
        0.20209f,
        0.214218f,
    };
    chassis_control_module_init(&leg_config);

    recovery_context = (chassis_recovery_context_t){
        .leg = {chassis_control_module_leg(CHASSIS_SIDE_LEFT),
                chassis_control_module_leg(CHASSIS_SIDE_RIGHT)},
        .ins = &ins,
        .cmd = &chassis_cmd,
        .fdb = &chassis_fdb_data,
        .theta_pid = chassis_control_module_theta_pid(),
        .roll_pid = chassis_control_module_roll_pid(),
        .length_pid = {chassis_control_module_length_pid(CHASSIS_SIDE_LEFT),
                       chassis_control_module_length_pid(CHASSIS_SIDE_RIGHT)},
        .vmc_out = {chassis_control_module_vmc_output(CHASSIS_SIDE_LEFT),
                    chassis_control_module_vmc_output(CHASSIS_SIDE_RIGHT)},
        .lqr_x_obs = chassis_control_module_lqr_x_obs(),
        .lqr_output = chassis_control_module_lqr_output(),
        .lqr_virtual_x = {
            chassis_control_module_lqr_virtual_x(CHASSIS_SIDE_LEFT),
            chassis_control_module_lqr_virtual_x(CHASSIS_SIDE_RIGHT)},
        .dm_obs = dm_obs,
        .control_dt_s = 0.005f,
        .low_leg_length = LEN_LEN_LOW,
    };
    chassis_recovery_module_bind(&recovery_context);
    safety_context = (chassis_safety_context_t){
        .leg = {chassis_control_module_leg(CHASSIS_SIDE_LEFT),
                chassis_control_module_leg(CHASSIS_SIDE_RIGHT)},
        .ins = &ins,
        .cmd = &chassis_cmd,
        .fdb = &chassis_fdb_data,
        .theta_pid = chassis_control_module_theta_pid(),
        .roll_pid = chassis_control_module_roll_pid(),
        .length_pid = {chassis_control_module_length_pid(CHASSIS_SIDE_LEFT),
                       chassis_control_module_length_pid(CHASSIS_SIDE_RIGHT)},
        .lqr_output = chassis_control_module_lqr_output(),
        .lqr_virtual_x = {
            chassis_control_module_lqr_virtual_x(CHASSIS_SIDE_LEFT),
            chassis_control_module_lqr_virtual_x(CHASSIS_SIDE_RIGHT)},
        .yaw_target = chassis_control_module_yaw_target(),
        .dm_obs = dm_obs,
        .dm_motor = {dm_motor[0], dm_motor[1], dm_motor[2], dm_motor[3]},
        .wheel_speed_phys = {
            chassis_control_module_wheel_speed_phys(CHASSIS_SIDE_LEFT),
            chassis_control_module_wheel_speed_phys(CHASSIS_SIDE_RIGHT)},
        .control_dt_s = 0.005f,
    };
    chassis_safety_module_bind(&safety_context);

}

/* 2026-09-23: keep the legacy global control_dt_s = 0.005 s for the
 * normal/recovery/safety chains; the previous global 1 ms fix made
 * VMC d_theta/d_l0 5x stronger and destabilized normal standing.
 * Jump-specific timers that really need the measured period use
 * jump_cycle_dt_s inside chassis_control.c. */
/**
 * @brief 底盘控制任务,以 1 khz 运行（平衡步兵需要1khz, chassis_task_entry）
 */
void chassis_control_task(void)
{
    /* 更新该线程所有的订阅者 */
    chassis_sub_pull();

    chassis_recovery_module_update_mode();

    // 切换腿长姿态
    switch (chassis_cmd.leg_level)
    {
        case LEG_LOW:
            chassis_control_module_set_leg_lengths(LEN_LEN_LOW, LEN_LEN_LOW);
            break;
        case LEG_MID:
            chassis_control_module_set_leg_lengths(LEN_LEN_MID, LEN_LEN_MID);
            break;
        case LEG_HIG:
            chassis_control_module_set_leg_lengths(LEN_LEN_HIG, LEN_LEN_HIG);
            break;
        default:
            chassis_control_module_set_leg_lengths(LEN_LEN_LOW, LEN_LEN_LOW);
            break;
    }
    /* RECOVERY + handoff/站稳 后强制低腿长 */
    if (chassis_recovery_module_after_handoff())
    {
        chassis_control_module_set_leg_lengths(LEN_LEN_LOW, LEN_LEN_LOW);
    }

    /* Once BUMP has finished RETRACT, STAND is a persistent LOW-mode latch.
     * Keep the command layer's temporary BUMP/HIGH selection from feeding the
     * normal reference generator; releasing the BUMP switch clears the latch
     * and restores the operator-selected LOW/MID level on the next cycle. */
    if (chassis_control_module_bump_low_lqr_active())
    {
        chassis_cmd.leg_level = LEG_LOW;
        chassis_control_module_set_leg_lengths(LEN_LEN_LOW, LEN_LEN_LOW);
    }

    switch (chassis_cmd.ctrl_mode)
    {
        case CHASSIS_RELAX:
            chassis_control_module_process_clear();
            chassis_recovery_module_reset();
            motor_relax();
            chassis_fdb_data.stand_state = CHASSIS_IS_RECOVERY;

            break;
        case CHASSIS_INIT:
            chassis_control_module_process_clear();
            chassis_recovery_module_reset();
            motor_enable();
#ifdef DM_8009_SET_ZERO_POSITION
            leg_init_get_zero();
#endif
            break;
        case CHASSIS_RECOVERY:/*不稳定的状态,介于init和relex状态的过渡状态*/
            /* 2026-10-07 (bump6.csv): 进入 RECOVERY 必须把 BUMP 状态机复位。
             *
             * RECOVERY 模式下 leg_calc() 不再运行，bump_control_prepare_cycle()
             * 也就不会被调用，BUMP 状态机会**永久冻结**在当时的阶段。
             * bump6 实测：t=81.364 进入 RETRACT，腿成功收到 0.114（已满足
             * avg_l0<=0.145 的退出条件），但模式在日志空洞期间切到 RECOVERY，
             * 状态机就停在 RETRACT 再也走不动 —— 既进不了 SWING_POSE，
             * 也退不回 IDLE，bump_active 一直保持 1，整车卡在收腿姿态。
             * 这里显式复位，让下次使能后从 IDLE 干净开始。 */
            chassis_control_module_bump_reset();
            /* 进入/退出 reset 已在 switch 前统一处理 */
            chassis_safety_module_reset();
            chassis_control_module_set_yaw_target(
                ins.yaw_total_angle * DEGREE_2_RAD);
            motor_enable();
            dm_v_set[LEFT][FRONT] = DM_V_SET;
            dm_v_set[LEFT][BACK] = DM_V_SET;
            dm_v_set[RIGHT][FRONT] = DM_V_SET;
            dm_v_set[RIGHT][BACK] = DM_V_SET;
            chassis_recovery_module_step();
        if(chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK &&
           chassis_fdb_data.stand_state != CHASSIS_IS_STAND){
            chassis_control_module_clear_recovery_runtime();


            /* stand_state 回退时重置 handoff gain */
            chassis_recovery_module_clear_handoff();
        }
            chassis_cmd.vx = 0.0f;
            chassis_cmd.vx_set = 0.0f;
            chassis_cmd.vy = 0.0f;
            chassis_cmd.vy_set = 0.0f;
            chassis_cmd.vw_relative = 0.0f;
            chassis_cmd.vw_relative_set = 0.0f;

            break;

        case CHASSIS_OPEN_LOOP:/*底盘开环控制,init成功,或是revovery成功,此状态应为稳定站立时*/
            motor_enable();
            chassis_control_module_update_yaw();
            chassis_safety_module_check_danger();
            /* Ground_Detect 已移至 switch 外部统一调用 */
            break;

        case CHASSIS_FOLLOW_GIMBAL:
            motor_enable();
            chassis_control_module_update_yaw();
            chassis_safety_module_check_danger();

            break;
        case CHASSIS_SPIN:
            motor_enable();
            chassis_control_module_update_yaw();
            chassis_safety_module_check_danger();

            break;
        case CHASSIS_JUMP:
            motor_enable();
            chassis_safety_module_check_danger();

            break;
        case CHASSIS_AUTO:
            motor_enable();
            chassis_safety_module_check_danger();
            break;
        default:
            motor_relax();
            break;
    }

    /* Ground_Detect + 边沿更新 + 落地动作 已移入 leg_calc() 统一处理 */

    /* Keep the final compact leg pose through the automatic RECOVERY ->
     * OPEN_LOOP transition, before the length PID is evaluated this cycle. */
    chassis_recovery_module_apply_length_hold();

    chassis_control_module_kf_update();
    chassis_control_module_step(); // 保证稳定的运算频率，不受模式影响
    jump_torque_budget_publish_vmc();
    /* The same TIM4 tick sends one completed-cycle command to each DM motor
     * during jump/landing. Normal driving retains its original round robin. */
    dm_motor_set_batch_poll(chassis_control_module_jump_fast_poll_active());
    chassis_recovery_module_capture_debug();

    if (chassis_debug_module_should_emit()) {
        uint8_t vofa_page = chassis_debug_module_page();

        chassis_emit_debug_page(vofa_page);
    }

    /* 轮毂反馈快照：BUMP 接触判据与 chassis_fdb 共用同一帧，避免异步读到不一致的采样。 */
    if (m3508_motor[LEFT] != NULL) {
        wheel_feedback[LEFT] = m3508_motor[LEFT]->measure;
    }
    if (m3508_motor[RIGHT] != NULL) {
        wheel_feedback[RIGHT] = m3508_motor[RIGHT]->measure;
    }

    chassis_control_module_capture_wheel_feedback(
        CHASSIS_SIDE_LEFT, wheel_feedback[LEFT].speed_aps);
    chassis_control_module_capture_wheel_feedback(
        CHASSIS_SIDE_RIGHT, wheel_feedback[RIGHT].speed_aps);

    chassis_fdb_data.M3508_l = wheel_feedback[LEFT];
    chassis_fdb_data.M3508_r = wheel_feedback[RIGHT];
    /* 更新发布该线程的msg */
    chassis_pub_push();

#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1) && defined(SYSTEMVIEW_USE_UART) && (SYSTEMVIEW_USE_UART == 1))
    SEGGER_SYSVIEW_UART_Poll();
#endif

    vTaskDelay(1);
}

/**
 * @brief 底盘初始化（注册底盘电机及其控制器初始化等）
 */
static int chassis_motor_init(void)
{
    dm_motor_init();
    m3508_motor_init();
    return 0;
}

/* --------------------------------- 底盘解算控制 --------------------------------- */
#ifdef BSP_CHASSIS_LEG_MODE

/**
 * @brief 轮腿底盘运动解算   _正方向
 *    |_____电池架_____|  |
 *      |_4     3_|
 *      |_1     2_|
 *     |___________|
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮力矩
 */

/*****************************************电机接口*************************************************************************/
/**
 * @brief 轮腿底盘运动解算   _正方向
 *    |_______________|  |
 *      |_4     3_|
 *      |_1     2_|
 *     |___________|
 * @param cmd cmd 底盘指令值，使用其中的速度
 * @param out_speed 底盘各轮力矩
 */


/* ----------------------------------------- 电机控制相关 ---------------------------------------------------------- */
static void leg_init_get_zero()
{
    /*设置电机零点*/
    for (uint8_t i = 0; i < 4; i++)
    {
        dm_motor[i]->set_mode(dm_motor[i], DM_CMD_ZERO_POSITION);
    }

}
static void motor_enable()
{
    dm_motor_enable_all();  // 所有电机进入 motor 模式
    for (uint8_t i = 0; i < 2; i++)
    {
        dji_motor_enable(m3508_motor[i]);
    }
}

static void motor_relax()
{
    dm_motor_disable_all();
    for (uint8_t i = 0; i < 2; i++)
    {
        dji_motor_relax(m3508_motor[i]);
    }
}

static float control_dt[4];
static float control_start[4];
static float dm_send_t[4];
static volatile float jump_torque_budget_snapshot[4];
static volatile uint8_t jump_torque_budget_snapshot_axis_clip[4];
static volatile uint8_t jump_torque_budget_jump_active;
static volatile uint8_t jump_torque_budget_exit_held_mask;
static volatile uint8_t jump_torque_budget_snapshot_scaled;
static volatile uint8_t jump_torque_budget_latest_residual_clip;
static int16_t m3508_send_current[2];
float dm_obs[4];
#define DM_RATIO 1.0f
#define DM_OUTPUT_LIMIT_NOMINAL 20.0f
#define DM_OUTPUT_LIMIT_JUMP_THRUST 35.0f /* 2026-09-22: cut 40->35, big jumps tripped battery OCP */
#define DM_OUTPUT_LIMIT_JUMP_AIR    35.0f /* 2026-09-22: cut 40->35, same battery-OCP reason */
#define DM_OUTPUT_LIMIT_JUMP_LANDING 36.0f

/* Page 47 captures the complete six-actuator chassis execution chain. */
/* Both the on-vehicle task and host replay use this exact page dispatch,
 * including motor packet/clip telemetry on pages 47 and 57--62. */
void chassis_emit_debug_page(uint8_t page)
{
    chassis_debug_begin();
    if (page == 64U) {
        (void)chassis_safety_module_fill_debug_page(page);
        (void)chassis_control_module_fill_debug_page(page);
    } else {
        (void)(chassis_motor_fill_debug_page(page) ||
               gimbal_fill_debug_page(page) ||
               chassis_recovery_module_fill_debug_page(page) ||
               chassis_safety_module_fill_debug_page(page) ||
               chassis_control_module_fill_debug_page(page));
    }
    chassis_debug_transmit(page);
}

static uint8_t chassis_motor_fill_debug_page(uint8_t page)
{
    if (page == 57U || page == 58U || page == 59U || page == 60U ||
        page == 61U || page == 62U) {
        /* Page 57 v5: preserve v4 bits; bits 14/15 diagnose aggregate jump
         * hip-command scaling and the latest asynchronous residual clamp. */
        leg_obj_t *leg_l = chassis_control_module_leg(CHASSIS_SIDE_LEFT);
        leg_obj_t *leg_r = chassis_control_module_leg(CHASSIS_SIDE_RIGHT);
        const float fn_l = chassis_safety_module_offground_filtered_force(
            CHASSIS_SIDE_LEFT);
        const float fn_r = chassis_safety_module_offground_filtered_force(
            CHASSIS_SIDE_RIGHT);
        uint16_t status = 0U;
        if (chassis_safety_module_off_ground()) status |= 1U;
        status |= (uint16_t)((chassis_safety_module_landing_buffer_phase() & 0x03U) << 1);
        if (chassis_safety_module_wheel_shut()) status |= (1U << 3);
        if (chassis_safety_module_jump_air_profile_active()) status |= (1U << 4);
        if (chassis_safety_module_takeoff_edge()) status |= (1U << 5);
        if (chassis_safety_module_landing_edge()) status |= (1U << 6);
        if (chassis_control_module_jump_thrust_active()) status |= (1U << 7);
        const uint8_t release_reason =
            chassis_control_module_jump_stroke_release_reason();
        status |= jump_stroke_release_status_bits(
            (jump_stroke_release_reason_t)release_reason);
        status = jump_page57_add_dm_clip_bits(
            status,
            dm_output_clip_latch[3], /* left front */
            dm_output_clip_latch[0], /* left back */
            dm_output_clip_latch[2], /* right front */
            dm_output_clip_latch[1]);/* right back */
        status |= jump_torque_budget_page57_bits(
            jump_torque_budget_snapshot_scaled,
            jump_torque_budget_latest_residual_clip);

        if (page == 58U) {
            return chassis_control_module_fill_jump_rate_debug_page(status);
        }
        if (page == 59U) {
            return chassis_control_module_fill_jump_handoff_debug_page(status);
        }
        if (page == 60U) {
            return chassis_control_module_fill_jump_capture_debug_page(status);
        }
        if (page == 61U) {
            return chassis_control_module_fill_jump_balance_debug_page(status);
        }
        if (page == 62U) {
            return chassis_control_module_fill_jump_ground_recovery_debug_page(status);
        }

        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_fdb_data.stand_state +
                             100.0f * (float)chassis_control_module_jump_state());
        chassis_debug_set(2, (float)(chassis_control_module_jump_fault() +
                             10U * chassis_safety_module_danger_reason()));
        chassis_debug_set(3, (float)status);
        /* Source: 0=zero, 1=ground LQR, 2=reaction wheel, 3=landing damper,
         * 4=jump-origin RETURN blend. Latched by the same callback as I24/I25. */
        chassis_debug_set(4,
            (float)chassis_control_module_jump_wheel_output_source());
        chassis_debug_set(5, chassis_control_module_jump_elapsed_s());
        chassis_debug_set(6, leg_l != NULL ? leg_l->l0 : 0.0f);
        chassis_debug_set(7, leg_r != NULL ? leg_r->l0 : 0.0f);
        chassis_debug_set(8,
            chassis_control_module_jump_leg_length_rate(CHASSIS_SIDE_LEFT));
        chassis_debug_set(9,
            chassis_control_module_jump_leg_length_rate(CHASSIS_SIDE_RIGHT));
        chassis_debug_set(10, leg_l != NULL ? leg_l->theta : 0.0f);
        chassis_debug_set(11, leg_r != NULL ? leg_r->theta : 0.0f);
        chassis_debug_set(12, leg_l != NULL ? leg_l->d_theta_lpf : 0.0f);
        chassis_debug_set(13, leg_r != NULL ? leg_r->d_theta_lpf : 0.0f);
        chassis_debug_set(14, ins.roll);
        chassis_debug_set(15, ins.pitch);
        chassis_debug_set(16, -ins.gyro[0]);
        chassis_debug_set(17, ins.motion_accel_b[2]);
        chassis_debug_set(18, leg_l != NULL ? leg_l->support_force : 0.0f);
        chassis_debug_set(19, leg_r != NULL ? leg_r->support_force : 0.0f);
        chassis_debug_set(20,
            chassis_safety_module_raw_support_candidate(CHASSIS_SIDE_LEFT));
        chassis_debug_set(21,
            chassis_safety_module_raw_support_candidate(CHASSIS_SIDE_RIGHT));
        chassis_debug_set(22, fn_l);
        chassis_debug_set(23, fn_r);
        chassis_debug_set(24, (float)m3508_send_current[LEFT]);
        chassis_debug_set(25, (float)m3508_send_current[RIGHT]);
        chassis_debug_set(26, leg_l != NULL ? leg_l->F_Vertical : 0.0f);
        chassis_debug_set(27, leg_r != NULL ? leg_r->F_Vertical : 0.0f);
        chassis_debug_set(28,
            (float)chassis_safety_module_jump_legs_unloaded());
        chassis_debug_set(29, 0.5f * (fn_l + fn_r));
        chassis_debug_set(30,
            chassis_control_module_jump_air_wheel_torque());
        /* I31 is raw measured task dt in ms (0=first sample invalid). Landing
         * phase is in I3 bits 1-2; sanitized jump dt still drives control. */
        chassis_debug_set(31,
            chassis_control_module_jump_cycle_raw_dt_ms());
        return 1U;
    }

    if (page == 56U) {
        /* Page 56: all-in-one 32-channel jump page.
         * I0 time; I1 mode+10*stand+100*jump; I2 jump_fault+10*danger;
         * I3 status bits: bit0 offground, bit1-2 landing phase, bit3 wheel shut,
         *    bit4 air profile, bit5 takeoff edge, bit6 landing edge, bit7 thrust;
         * I4 LATCHED danger reason (survives the abort; cleared at jump start);
         * I5 jump target; I6/I7 l0 L/R; I8/I9 dl0 L/R;
         * I10/I11 theta L/R; I12/I13 d_theta L/R; I14 roll deg; I15 pitch deg;
         * I16 pitch rate (-gyro[0]); I17 yaw rate gyro[2];
         * I18 airborne leg-angle reference; I19 common theta;
         * I20/I21 support L/R;
         * I22 reaction-wheel pitch ERROR (pitch - its own reference), rad;
         * I23 reaction-wheel pitch RATE input, rad/s;
         * I24/I25 wheel cmd L/R;
         * I26 gimbal pitch deg; I27 GM6020 current cmd; I28 gimbal health bits;
         * I29 jump Roll differential PD force N;
         * I30 reaction-wheel logical torque Nm (answers "was it really zero?");
         * I31 current jump state.
         * NOTE: I22/I23 replaced the two contact-force channels -- contact force
         * is still on page 52.  They exist because jump23 showed the wheel
         * reading ZERO while Pitch collapsed +2.4 -> -6.6 deg ON THE GROUND, and
         * the log had no way to separate a gating problem from a real zero
         * demand.  I29 is written after gimbal_fill_all_debug() on purpose. */
        const uint32_t now_ms = HAL_GetTick();
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_fdb_data.stand_state +
                             100.0f * (float)chassis_control_module_jump_state());
        chassis_debug_set(2, (float)(chassis_control_module_jump_fault() +
                             10U * chassis_safety_module_danger_reason()));

        uint16_t status = 0U;
        if (chassis_safety_module_off_ground()) status |= 1U;
        status |= (uint16_t)((chassis_safety_module_landing_buffer_phase() & 0x03U) << 1);
        if (chassis_safety_module_wheel_shut()) status |= (1U << 3);
        if (chassis_safety_module_jump_air_profile_active()) status |= (1U << 4);
        if (chassis_safety_module_takeoff_edge()) status |= (1U << 5);
        if (chassis_safety_module_landing_edge()) status |= (1U << 6);
        if (chassis_control_module_jump_thrust_active()) status |= (1U << 7);
        chassis_debug_set(3, (float)status);
        /* I4 is the latched danger verdict (survives the abort that clears
         * `danger_reason` in the same cycle); jump elapsed time moved to I29. */
        chassis_debug_set(4, (float)chassis_safety_module_danger_reason_latched());
        chassis_debug_set(5, chassis_control_module_jump_length_target());

        leg_obj_t *leg_l = chassis_control_module_leg(CHASSIS_SIDE_LEFT);
        leg_obj_t *leg_r = chassis_control_module_leg(CHASSIS_SIDE_RIGHT);
        if (leg_l != NULL && leg_r != NULL) {
            chassis_debug_set(6, leg_l->l0);
            chassis_debug_set(7, leg_r->l0);
            chassis_debug_set(8, leg_l->d_l0_lpf);
            chassis_debug_set(9, leg_r->d_l0_lpf);
            chassis_debug_set(10, leg_l->theta);
            chassis_debug_set(11, leg_r->theta);
            chassis_debug_set(12, leg_l->d_theta_lpf);
            chassis_debug_set(13, leg_r->d_theta_lpf);
            /* I18/I19: airborne leg-angle reference and common theta.
             * I29/I30: raw per-leg Tp (moved here when the wheel diagnostics
             * took the old Tp slots). */
            chassis_debug_set(18,
                chassis_safety_module_jump_air_theta_ref());
            chassis_debug_set(19, 0.5f * (leg_l->theta + leg_r->theta));
            chassis_debug_set(20, leg_l->support_force);
            chassis_debug_set(21, leg_r->support_force);
        }
        chassis_debug_set(14, ins.roll);
        chassis_debug_set(15, ins.pitch);
        chassis_debug_set(16, -ins.gyro[0]);
        chassis_debug_set(17, ins.gyro[2]);
        /* I22/I23: reaction-wheel channel INPUTS -- pitch error against its own
         * reference, and the pitch rate it feeds back.  These replaced the two
         * contact-force channels: jump23 showed the wheel reading ZERO while
         * Pitch collapsed from +2.4 to -6.6 deg on the ground, and without these
         * there is no way to separate a gating problem from a genuine zero
         * demand.  Contact force is still on page 52. */
        chassis_debug_set(22,
            chassis_control_module_jump_air_wheel_pitch_err_rad());
        chassis_debug_set(23,
            chassis_control_module_jump_air_wheel_pitch_rate());
        chassis_debug_set(24, (float)m3508_send_current[LEFT]);
        chassis_debug_set(25, (float)m3508_send_current[RIGHT]);

        /* gimbal_fill_all_debug(first) writes first+0..first+3, i.e. I26-I29.
         * It runs FIRST so that the I29 write below deliberately takes that
         * slot back: the gimbal's "max feedback age" is already covered by page
         * 55, while the Roll differential force is jump-critical.  Net map:
         * I26 gimbal pitch, I27 GM6020 current cmd, I28 gimbal health flags,
         * I29 roll diff force, I30 wheel logical torque, I31 jump state. */
        gimbal_fill_all_debug(26U);

        chassis_debug_set(29, chassis_control_module_jump_roll_diff_force());
        chassis_debug_set(30, chassis_control_module_jump_air_wheel_torque());
        chassis_debug_set(31, (float)chassis_control_module_jump_state());
        return 1U;
    }

    if (page == 55U) {
        /* Page 55: whole-machine communication health summary.
         * I0-I3   hip feedback age ms: LF/LB/RF/RB, -1 = never received
         * I4-I7   hip DM status code: 0=disabled, 1=enabled, 8=over-voltage
         * I8-I11  hip software mode: 1 = ctrl_mode DM_CMD_MOTOR_MODE
         * I12-I15 hip max temperature, degC
         * I16/I17 wheel feedback age ms: left/right
         * I18/I19 wheel feedback current
         * I20/I21 wheel temperature, degC
         * I22-I31 filled by gimbal page 55. */
        static const uint8_t health_motor_order[4] = {3U, 0U, 2U, 1U};
        const uint32_t now_ms = HAL_GetTick();

        for (uint8_t i = 0U; i < 4U; ++i) {
            const uint8_t motor_index = health_motor_order[i];
            dm_motor_object_t *motor = dm_motor[motor_index];
            const uint8_t enabled =
                (motor != NULL &&
                 motor->ctrl_mode == DM_CMD_MOTOR_MODE) ? 1U : 0U;
            const float age_ms =
                (motor != NULL && motor->measure.rx_count != 0U) ?
                (float)(now_ms - motor->measure.last_rx_ms) : -1.0f;
            const float max_temp = (motor != NULL) ?
                fmaxf(motor->measure.temperature_MOS,
                      motor->measure.temperature_Rotor) : -1.0f;
            chassis_debug_set((uint8_t)(0U + i), age_ms);
            chassis_debug_set((uint8_t)(4U + i),
                              motor != NULL ? (float)motor->measure.ERR : -1.0f);
            chassis_debug_set((uint8_t)(8U + i), (float)enabled);
            chassis_debug_set((uint8_t)(12U + i), max_temp);
        }

        chassis_debug_set(16U,
            (m3508_motor[LEFT] != NULL &&
             m3508_motor[LEFT]->measure.rx_count != 0U) ?
            (float)(now_ms - m3508_motor[LEFT]->measure.last_rx_ms) : -1.0f);
        chassis_debug_set(17U,
            (m3508_motor[RIGHT] != NULL &&
             m3508_motor[RIGHT]->measure.rx_count != 0U) ?
            (float)(now_ms - m3508_motor[RIGHT]->measure.last_rx_ms) : -1.0f);
        chassis_debug_set(18U, m3508_motor[LEFT] != NULL ?
            (float)m3508_motor[LEFT]->measure.real_current : 0.0f);
        chassis_debug_set(19U, m3508_motor[RIGHT] != NULL ?
            (float)m3508_motor[RIGHT]->measure.real_current : 0.0f);
        chassis_debug_set(20U, m3508_motor[LEFT] != NULL ?
            (float)m3508_motor[LEFT]->measure.temperature : 0.0f);
        chassis_debug_set(21U, m3508_motor[RIGHT] != NULL ?
            (float)m3508_motor[RIGHT]->measure.temperature : 0.0f);

        (void)gimbal_fill_debug_page(55U);
        return 1U;
    }

    if (page == 52U) {
        /* Keep one capture for both jump and a possible RECOVERY.  In
         * RECOVERY, reuse the Page 39 motion block in I0-I23; otherwise fill
         * the normal jump state/force block. */
        if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY) {
            if (!chassis_recovery_module_fill_debug_page(39U)) {
                return 0U;
            }
        } else if (!chassis_control_module_fill_debug_page(page)) {
            return 0U;
        }

        static const uint8_t physical_motor_order[4] = {3U, 0U, 2U, 1U};
        for (uint8_t physical_index = 0U; physical_index < 4U;
             physical_index++) {
            const uint8_t motor_index = physical_motor_order[physical_index];
            chassis_debug_set((uint8_t)(24U + physical_index),
                              dm_send_t[motor_index]);
        }
        /* Show the exact filtered support values used by the two-leg takeoff
         * and landing detector.  The old motor-speed channels were stale on
         * this page and could not explain delayed takeoff decisions. */
        chassis_debug_set(28,
            chassis_safety_module_offground_filtered_force(
                CHASSIS_SIDE_LEFT));
        chassis_debug_set(29,
            chassis_safety_module_offground_filtered_force(
                CHASSIS_SIDE_RIGHT));
        leg_obj_t *left_leg =
            chassis_control_module_leg(CHASSIS_SIDE_LEFT);
        leg_obj_t *right_leg =
            chassis_control_module_leg(CHASSIS_SIDE_RIGHT);
        chassis_debug_set(30, left_leg != NULL ? left_leg->theta : 0.0f);
        chassis_debug_set(31, right_leg != NULL ? right_leg->theta : 0.0f);
        /* Latch the reason even after the mode changes to RECOVERY:
         * I19 = jump_fault + 10 * danger_reason; 0 = no fault/no danger. */
        chassis_debug_set(19,
            (float)(chassis_control_module_jump_fault() +
                    10U * chassis_safety_module_danger_reason()));
        return 1U;
    }

    if (page == 51U) {
        chassis_hip_shake_debug_t hip = {0};
        chassis_control_module_get_hip_shake_debug(&hip);

        /* One-page left-front hip shake diagnosis, exactly I0-I31.
         * I0=time; I1=mode+10*leg level.
         * I2/I3=left/right actual length; I4=raw theta L-R;
         * I5=LQR-observed theta L-R after compensation.
         * I6/I7=left/right timestamped theta rate; I8=rate L-R.
         * I9/I10/I11=theta-sync P/D/nonlinear-guard virtual torque.
         * I12/I13=left/right base LQR Tp; I14/I15=final Tp.
         * I16/I17=left/right final support force.
         * I18/I19=LF(ID4)/LB(ID1) absolute motor angle; in Recovery these
         * become left/right leg phi0 so a stalled return-to-zero is visible.
         * I20-I23=final MIT torque command in LF(ID4), LB(ID1), RF(ID3),
         * RB(ID2) order. I24=valid LF reported speed.
         * I25/I26=left/right length-loop force; I27=F_roll.
         * I28/I29=LF/LB reported torque; normally I30/I31=Roll P/D output,
         * while in Recovery they are the Recovery sub-state/fault code. */
        const uint8_t recovery_debug =
            chassis_cmd.ctrl_mode == CHASSIS_RECOVERY;
        chassis_debug_set(0, dwt_get_time_ms() / 1000.0f);
        chassis_debug_set(1, (float)chassis_cmd.ctrl_mode +
                             10.0f * (float)chassis_cmd.leg_level);
        chassis_debug_set(2, hip.leg_length_l);
        chassis_debug_set(3, hip.leg_length_r);
        chassis_debug_set(4, hip.theta_raw_diff);
        chassis_debug_set(5, hip.theta_lqr_diff);
        chassis_debug_set(6, hip.theta_rate_l);
        chassis_debug_set(7, hip.theta_rate_r);
        chassis_debug_set(8, hip.theta_rate_diff);
        chassis_debug_set(9, hip.theta_sync_p);
        chassis_debug_set(10, hip.theta_sync_d);
        chassis_debug_set(11, hip.theta_sync_guard);
        chassis_debug_set(12, hip.lqr_tp_l);
        chassis_debug_set(13, hip.lqr_tp_r);
        chassis_debug_set(14, hip.final_tp_l);
        chassis_debug_set(15, hip.final_tp_r);
        chassis_debug_set(16, hip.support_force_l);
        chassis_debug_set(17, hip.support_force_r);
        chassis_debug_set(18, recovery_debug ? recovery_context.leg[LEFT]->phi0 :
                              (dm_motor[3] != NULL ?
                               dm_motor[3]->measure.angle_abs : 0.0f));
        chassis_debug_set(19, recovery_debug ? recovery_context.leg[RIGHT]->phi0 :
                              (dm_motor[0] != NULL ?
                               dm_motor[0]->measure.angle_abs : 0.0f));

        static const uint8_t physical_motor_order[4] = {3U, 0U, 2U, 1U};
        for (uint8_t physical_index = 0U; physical_index < 4U;
             physical_index++) {
            const uint8_t motor_index = physical_motor_order[physical_index];
            chassis_debug_set((uint8_t)(20U + physical_index),
                              dm_send_t[motor_index]);
        }
        chassis_debug_set(24, dm_motor[3] != NULL ?
                              dm_motor[3]->measure.speed_rads : 0.0f);
        chassis_debug_set(25, hip.length_force_l);
        chassis_debug_set(26, hip.length_force_r);
        chassis_debug_set(27, hip.roll_force);
        chassis_debug_set(28, dm_motor[3] != NULL ?
                              dm_motor[3]->measure.torque : 0.0f);
        chassis_debug_set(29, dm_motor[0] != NULL ?
                              dm_motor[0]->measure.torque : 0.0f);
        chassis_debug_set(30, recovery_debug ?
                              (float)chassis_recovery_module_debug_state() :
                              hip.roll_p);
        chassis_debug_set(31, recovery_debug ?
                              (float)chassis_recovery_module_debug_fault_reason() :
                              hip.roll_d);
        return 1U;
    }

    if (page != 47U) {
        return 0U;
    }

    const uint32_t now_ms = HAL_GetTick();

    /* DM groups are ordered by CAN ID 0x01, 0x02, 0x03, 0x04:
     * I0..I3   = final MIT torque command (N.m)
     * I4..I7   = reported angle (rad)
     * I8..I11  = reported speed (rad/s)
     * I12..I15 = reported torque (N.m)
     * I16..I19 = feedback age (ms, -1 means never received)
     * I20..I23 = DM error code. */
    for (uint8_t motor_index = 0U; motor_index < 4U; motor_index++) {
        dm_motor_object_t *motor = dm_motor[motor_index];
        chassis_debug_set(motor_index, dm_send_t[motor_index]);
        chassis_debug_set((uint8_t)(4U + motor_index),
                          motor != NULL ? motor->measure.angle : 0.0f);
        chassis_debug_set((uint8_t)(8U + motor_index),
                          motor != NULL ? motor->measure.speed_rads : 0.0f);
        chassis_debug_set((uint8_t)(12U + motor_index),
                          motor != NULL ? motor->measure.torque : 0.0f);
        chassis_debug_set((uint8_t)(16U + motor_index),
                          (motor != NULL && motor->measure.rx_count != 0U) ?
                          (float)(now_ms - motor->measure.last_rx_ms) : -1.0f);
        chassis_debug_set((uint8_t)(20U + motor_index),
                          motor != NULL ? (float)motor->measure.ERR : -1.0f);
    }

    /* Wheel groups are ordered LEFT, RIGHT:
     * I24..I25 = final current command (raw C620 units)
     * I26..I27 = reported speed (rpm)
     * I28..I29 = reported current (raw C620 units)
     * I30..I31 = feedback age (ms, -1 means never received). */
    for (uint8_t side = LEFT; side <= RIGHT; side++) {
        dji_motor_object_t *motor = m3508_motor[side];
        chassis_debug_set((uint8_t)(24U + side),
                          (float)m3508_send_current[side]);
        chassis_debug_set((uint8_t)(26U + side),
                          motor != NULL ? (float)motor->measure.speed_rpm : 0.0f);
        chassis_debug_set((uint8_t)(28U + side),
                          motor != NULL ? (float)motor->measure.real_current : 0.0f);
        chassis_debug_set((uint8_t)(30U + side),
                          (motor != NULL && motor->measure.rx_count != 0U) ?
                          (float)(now_ms - motor->measure.last_rx_ms) : -1.0f);
    }

    return 1U;
}

static inline float dm_output_limit_current(void)
{
    if ((chassis_cmd.ctrl_mode == CHASSIS_JUMP ||
         chassis_safety_module_jump_landing_owner_active()) &&
        chassis_safety_module_off_ground()) {
        return DM_OUTPUT_LIMIT_JUMP_AIR;
    }
    if (chassis_safety_module_jump_landing_owner_active()) {
        return DM_OUTPUT_LIMIT_JUMP_LANDING;
    }
    if (chassis_control_module_jump_peak_torque_active()) {
        return DM_OUTPUT_LIMIT_JUMP_THRUST;
    }
    return chassis_recovery_module_dm_output_limit(DM_OUTPUT_LIMIT_NOMINAL);
}

#define DM_OUTPUT_LIMIT dm_output_limit_current()

static uint8_t jump_torque_budget_guard_enabled(void)
{
    return (uint8_t)(chassis_cmd.ctrl_mode == CHASSIS_JUMP ||
                     chassis_control_module_bump_torque_budget_active() ||
                     jump_torque_budget_jump_active != 0U ||
                     jump_torque_budget_exit_held_mask != 0U);
}

static uint8_t jump_torque_budget_live_scope_active(void)
{
    return (uint8_t)(chassis_cmd.ctrl_mode == CHASSIS_JUMP ||
        chassis_safety_module_jump_landing_owner_active() ||
        chassis_control_module_bump_torque_budget_active());
}

static float jump_torque_budget_callback_request(uint8_t slot,
                                                chassis_side_e side,
                                                uint8_t leg_motor,
                                                float sign)
{
    if (jump_torque_budget_live_scope_active() &&
        jump_torque_budget_jump_active != 0U &&
        slot < JUMP_HIP_TORQUE_SLOT_COUNT) {
        return jump_torque_budget_snapshot[slot];
    }
    return sign * chassis_control_module_vmc_output(side)[leg_motor] *
           DM_RATIO;
}

static uint8_t jump_torque_budget_callback_seed_clip(uint8_t slot)
{
    if (jump_torque_budget_live_scope_active() &&
        jump_torque_budget_jump_active != 0U &&
        slot < JUMP_HIP_TORQUE_SLOT_COUNT) {
        return jump_torque_budget_snapshot_axis_clip[slot];
    }
    return 0U;
}

static void jump_torque_budget_callback_finish(uint8_t slot)
{
    if (!jump_torque_budget_guard_enabled()) {
        jump_torque_budget_latest_residual_clip = 0U;
        return;
    }

    float other_abs_sum = 0.0f;
    for (uint8_t i = 0U; i < JUMP_HIP_TORQUE_SLOT_COUNT; ++i) {
        if (i != slot) other_abs_sum += fabsf(dm_send_t[i]);
    }
    const jump_torque_budget_residual_t result =
        jump_torque_budget_apply_residual(dm_send_t[slot], other_abs_sum,
                                           JUMP_HIP_TORQUE_BUDGET_NM);
    dm_send_t[slot] = result.valid ? result.torque : 0.0f;
    jump_torque_budget_latest_residual_clip =
        (uint8_t)(!result.valid || result.residual_limited);

    if (!jump_torque_budget_live_scope_active() &&
        jump_torque_budget_exit_held_mask != 0U) {
        jump_torque_budget_exit_held_mask =
            jump_torque_budget_callback_exit_mask(
                jump_torque_budget_exit_held_mask, slot);
    }
}

/* Called once after the complete 1 kHz control/VMC step. A short PRIMASK
 * section makes the four volatile snapshot values visible as one publication
 * to the asynchronous timer callbacks. */
static void jump_torque_budget_publish_vmc(void)
{
    const uint8_t is_jump_active = (uint8_t)(
        chassis_cmd.ctrl_mode == CHASSIS_JUMP ||
        chassis_safety_module_jump_landing_owner_active() ||
        chassis_control_module_bump_torque_budget_active());
    float candidate[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t axis_clip[4] = {0U, 0U, 0U, 0U};
    uint8_t vector_scaled = 0U;

    if (is_jump_active) {
        const float *left = chassis_control_module_vmc_output(
            CHASSIS_SIDE_LEFT);
        const float *right = chassis_control_module_vmc_output(
            CHASSIS_SIDE_RIGHT);
        const float captured_limit = dm_output_limit_current();
        if (left != NULL && right != NULL) {
            const jump_torque_budget_vector_t prepared =
                jump_torque_budget_prepare_vmc(
                    left[FRONT], left[BACK], right[FRONT], right[BACK],
                    captured_limit, JUMP_HIP_TORQUE_BUDGET_NM);
            if (prepared.valid) {
                for (uint8_t i = 0U; i < JUMP_HIP_TORQUE_SLOT_COUNT; ++i) {
                    candidate[i] = prepared.torque[i];
                    axis_clip[i] = prepared.per_motor_clipped[i];
                }
                vector_scaled = prepared.uniform_scaled;
            }
        }
    }

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    jump_torque_budget_exit_held_mask =
        jump_torque_budget_publish_exit_mask(
            jump_torque_budget_exit_held_mask,
            jump_torque_budget_jump_active, is_jump_active);
    for (uint8_t i = 0U; i < JUMP_HIP_TORQUE_SLOT_COUNT; ++i) {
        jump_torque_budget_snapshot[i] = candidate[i];
        jump_torque_budget_snapshot_axis_clip[i] = axis_clip[i];
    }
    jump_torque_budget_snapshot_scaled = vector_scaled;
    jump_torque_budget_jump_active = is_jump_active;
    __DMB();
    if (primask == 0U) __enable_irq();
}


/*目前是以护栏较窄的一侧为正方向,从正方向向后看去,rigdm_front:id 2 ; rigdm_back:id 3;left_front:id 1;left_back:id 4*/
/*
 * slope_unwrap_to_target_f32 - unwrapped 单方向斜坡
 * current 沿单调方向逼近 target，不做 wrap
 */
static inline float slope_unwrap_to_target_f32(float current, float target, float step)
{
    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

/* 1 号电机
 * 输入值为正,逆时针
 * */
static dm_motor_para_t dm_control_1(dm_motor_measure_t measure)
{
    control_dt[0] = dwt_get_time_us() - control_start[0];
    control_start[0] = dwt_get_time_us();
    static dm_motor_para_t set;
    uint8_t clip_seen = jump_torque_budget_callback_seed_clip(0U);

    dm_send_t[0] = jump_torque_budget_callback_request(
        0U, CHASSIS_SIDE_LEFT, BACK, -1.0f);

    float limit_first = DM_OUTPUT_LIMIT;
    float torque_before_limit = dm_send_t[0];
    LIMIT_MIN_MAX(dm_send_t[0], -limit_first, limit_first);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[0],
                                 limit_first));


    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT){
        dm_send_t[0] = 0;
        set.kp = 0;
        set.kd = 0;
    }
    if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK && chassis_fdb_data.stand_state != CHASSIS_IS_STAND)){
        /* Before VMC recovery becomes active, keep this joint fully passive. */
        dm_send_t[0] = 0.0f;
        set.p = 0.0f;
        set.v = 0.0f;
        set.kp = 0.0f;
        set.kd = 0.0f;
        set.t = 0.0f;
    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[0] = 0;
        set.kp = 0;
        set.kd = 0;
#endif
    float limit_second = DM_OUTPUT_LIMIT;
    torque_before_limit = dm_send_t[0];
    LIMIT_MIN_MAX(dm_send_t[0], -limit_second, limit_second);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[0],
                                 limit_second));
    jump_torque_budget_callback_finish(0U);
    dm_output_clip_latch[0] =
        jump_dm_clip_finish(clip_seen, dm_send_t[0]);
    dm_obs[0] = dm_send_t[0];
    {
        set.t = dm_send_t[0]; // 正负没问题
    }
    return set;
}
/* 2 号电机
* 输入值为正,逆时针
 * */
/*应该顺时针转*/
static dm_motor_para_t dm_control_2(dm_motor_measure_t measure)
{
    control_dt[1] = dwt_get_time_us() - control_start[1];
    control_start[1] = dwt_get_time_us();
    static dm_motor_para_t set;
    uint8_t clip_seen = jump_torque_budget_callback_seed_clip(1U);


    dm_send_t[1] = jump_torque_budget_callback_request(
        1U, CHASSIS_SIDE_RIGHT, BACK, 1.0f);

    float limit_first = DM_OUTPUT_LIMIT;
    float torque_before_limit = dm_send_t[1];
    LIMIT_MIN_MAX(dm_send_t[1], -limit_first, limit_first);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[1],
                                 limit_first));


    if((chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT)){
        dm_send_t[1] = 0;
        set.kp = 0;
        set.kd = 0;
    }

    if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK && chassis_fdb_data.stand_state != CHASSIS_IS_STAND)){
        /* Before VMC recovery becomes active, keep this joint fully passive. */
        dm_send_t[1] = 0.0f;
        set.p = 0.0f;
        set.v = 0.0f;
        set.kp = 0.0f;
        set.kd = 0.0f;
        set.t = 0.0f;
    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[1] = 0;
        set.kp = 0;
        set.kd = 0;
#endif
    float limit_second = DM_OUTPUT_LIMIT;
    torque_before_limit = dm_send_t[1];
    LIMIT_MIN_MAX(dm_send_t[1], -limit_second, limit_second);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[1],
                                 limit_second));
    jump_torque_budget_callback_finish(1U);
    dm_output_clip_latch[1] =
        jump_dm_clip_finish(clip_seen, dm_send_t[1]);
    dm_obs[1] = dm_send_t[1];
    {
        set.t = dm_send_t[1]; // 正负没问题
    }
    return set;
}
/* 3 号电机
 * 输入值为正,逆时针
 * */
/*应该逆时针转*/
static dm_motor_para_t dm_control_3(dm_motor_measure_t measure)
{
    control_dt[2] = dwt_get_time_us() - control_start[2];
    control_start[2] = dwt_get_time_us();
    static dm_motor_para_t set;
    uint8_t clip_seen = jump_torque_budget_callback_seed_clip(2U);

    dm_send_t[2] = jump_torque_budget_callback_request(
        2U, CHASSIS_SIDE_RIGHT, FRONT, 1.0f);

    float limit_first = DM_OUTPUT_LIMIT;
    float torque_before_limit = dm_send_t[2];
    LIMIT_MIN_MAX(dm_send_t[2], -limit_first, limit_first);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[2],
                                 limit_first));


    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT){
        dm_send_t[2] = 0;
        set.kp = 0;
        set.kd = 0;
    }
    if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK && chassis_fdb_data.stand_state != CHASSIS_IS_STAND)){
        /* Before VMC recovery becomes active, keep this joint fully passive. */
        dm_send_t[2] = 0.0f;
        set.p = 0.0f;
        set.v = 0.0f;
        set.kp = 0.0f;
        set.kd = 0.0f;
        set.t = 0.0f;
    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
        dm_send_t[2] = 0;
        set.kp = 0;
        set.kd = 0;
#endif
    float limit_second = DM_OUTPUT_LIMIT;
    torque_before_limit = dm_send_t[2];
    LIMIT_MIN_MAX(dm_send_t[2], -limit_second, limit_second);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[2],
                                 limit_second));
    jump_torque_budget_callback_finish(2U);
    dm_output_clip_latch[2] =
        jump_dm_clip_finish(clip_seen, dm_send_t[2]);
    dm_obs[2] = dm_send_t[2];
    {
        set.t = dm_send_t[2]; // 正负没问题
    }
    return set;
}
/* 4 号电机
 * 输入为正,方向逆时针
 * */
static dm_motor_para_t dm_control_4(dm_motor_measure_t measure)
{
    control_dt[3] = dwt_get_time_us() - control_start[3];
    control_start[3] = dwt_get_time_us();
    static dm_motor_para_t set;
    uint8_t clip_seen = jump_torque_budget_callback_seed_clip(3U);


    dm_send_t[3] = jump_torque_budget_callback_request(
        3U, CHASSIS_SIDE_LEFT, FRONT, -1.0f);

    float limit_first = DM_OUTPUT_LIMIT;
    float torque_before_limit = dm_send_t[3];
    LIMIT_MIN_MAX(dm_send_t[3], -limit_first, limit_first);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[3],
                                 limit_first));

    if(chassis_cmd.ctrl_mode == CHASSIS_RELAX || chassis_cmd.ctrl_mode == CHASSIS_INIT){
        dm_send_t[3] = 0;
        set.kp = 0;
        set.kd = 0;
    }
    if (chassis_cmd.ctrl_mode == CHASSIS_RECOVERY && (chassis_fdb_data.stand_state != CHASSIS_IS_RECOVERY_READY && chassis_fdb_data.stand_state != CHASSIS_LEG_BACK_IS_OK && chassis_fdb_data.stand_state != CHASSIS_IS_STAND)){
        /* Before VMC recovery becomes active, keep this joint fully passive. */
        dm_send_t[3] = 0.0f;
        set.p = 0.0f;
        set.v = 0.0f;
        set.kp = 0.0f;
        set.kd = 0.0f;
        set.t = 0.0f;
    }
    else{/*其余情况则使用mit力矩控制*/
        set.p = 0;
        set.kp = 0;
        set.v = 0;
        set.kd = 0;
    }
#ifdef DM8009P_SET_ZERO
    dm_send_t[3] = 0;
    set.kp = 0;
    set.kd = 0;

#endif
    float limit_second = DM_OUTPUT_LIMIT;
    torque_before_limit = dm_send_t[3];
    LIMIT_MIN_MAX(dm_send_t[3], -limit_second, limit_second);
    clip_seen = jump_dm_clip_accumulate(clip_seen,
        jump_dm_clip_was_applied(torque_before_limit, dm_send_t[3],
                                 limit_second));
    jump_torque_budget_callback_finish(3U);
    dm_output_clip_latch[3] =
        jump_dm_clip_finish(clip_seen, dm_send_t[3]);
    dm_obs[3] = dm_send_t[3];
    {
        set.t = dm_send_t[3]; // 正负没问题
    }
    return set;
}
/* 底盘每个电机对应的控制函数 */
static void *dm_control[4] =
        {
                dm_control_1,
                dm_control_2,
                dm_control_3,
                dm_control_4,
        };

static void dm_motor_init()
{
    /*电机id等可参考平步实物进行修改*/
    motor_config_t dm_motor_config1 = {   //left_back
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_LEFT_BACK,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[0] = dm_motor_register(&dm_motor_config1, dm_control[0]);

    motor_config_t dm_motor_config4 = {     //left_front
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_LEFT_FRONT,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[3] = dm_motor_register(&dm_motor_config4, dm_control[3]);

    motor_config_t dm_motor_config2 = {     //right_back
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_RIGHT_BACK,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[1] = dm_motor_register(&dm_motor_config2, dm_control[1]);

    motor_config_t dm_motor_config3 = {     //roght_front
            .motor_type = DM8009P,
            .can_id = CAN_ID_CHASSIS_MOTOR,
            .tx_id = CHASSIS_JOINT_RIGHT_FRONT,
            .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
    };
    dm_motor[2] = dm_motor_register(&dm_motor_config3, dm_control[2]);


}


static int16_t M3508_control_l(dji_motor_measure_t measure)
{
    m3508_send_current[LEFT] =
        chassis_control_module_wheel_current(CHASSIS_SIDE_LEFT, measure);
    return m3508_send_current[LEFT];
}
/*当输入为正是,转动方向为 顺时针 时针
 * 计算输入应该为正
 * */
static int16_t M3508_control_r(dji_motor_measure_t measure)
{
    m3508_send_current[RIGHT] =
        chassis_control_module_wheel_current(CHASSIS_SIDE_RIGHT, measure);
    return m3508_send_current[RIGHT];
}

static void *M3508_control[2] =
        {
                M3508_control_l,
                M3508_control_r,
        };

static void m3508_motor_init()
{
    motor_config_t motor_config1 = {
            .motor_type = M3508,
            .can_id = CAN_ID_WHEEL_MOTOR,
            .rx_id = CHASSIS_WHEEL_LEFT_ID,
    };
    m3508_motor[LEFT] = dji_motor_register(&motor_config1, M3508_control[LEFT]);

    motor_config_t motor_config_2 = {
            .motor_type = M3508,
            .can_id = CAN_ID_WHEEL_MOTOR,
            .rx_id = CHASSIS_WHEEL_RIGHT_ID,
    };
    m3508_motor[RIGHT] = dji_motor_register(&motor_config_2, M3508_control[RIGHT]);
}


/********************************************************************************************************************/
/**
 * @brief 底盘跳跃处理
 * @note 进入跳跃模式时记录当前时间，并将jump输出力矩加上跳跃下压力矩值，持续0.2s,开始跳跃0，15s后加上跳跃回缩力矩值，持续0.4s,落地后跳跃结束
 */
// TODO: 结合离地检测进行优化（进阶玩法：空中轮子充当动量轮调整姿态
// TODO: 跳跃力矩和持续时间还需继续优化
// TODO: 跳跃前需要增加机体pitch轴范围判断，或起跳前通过改写k矩阵，使得机体pitch轴更快收敛，充分准备起跳，减少空中发撒
// TODO：目前通过多状态量判断跳跃是否结束，以及切入跳跃模式仅跳跃一次，需要进一步优化，可以结合cmd线程
#endif /* BSP_CHASSIS_LEG_MODE */


static void chassis_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
}

/**
 * @brief chassis 线程中所有发布者推送更新话题
 */
static void chassis_pub_push(void)
{
    mcn_publish(MCN_HUB(chassis_fdb), &chassis_fdb_data);
}

/**
 * @brief chassis 线程中所有订阅者获取更新话题
 */
static void chassis_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_cmd);
    }
}
