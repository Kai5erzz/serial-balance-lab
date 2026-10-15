/**
 * @file chassis_control.h
 * @brief Normal chassis control module interface.
 */
#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H

#include <stdint.h>

#include "chassis_private.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"
#include "DM_motor.h"

typedef struct
{
    dm_motor_object_t *dm_motor[4];
    dji_motor_object_t *wheel_motor[CHASSIS_SIDE_COUNT];
    /* BUMP 接触判据需要完整的轮毂反馈帧（电流/转速/时间戳），
     * 不只是 capture_wheel_feedback() 记录的调试速度。 */
    const dji_motor_measure_t *wheel_feedback;
    struct ins_msg *ins;
    struct chassis_cmd_msg *cmd;
    struct chassis_fdb_msg *fdb;
    float control_dt_s;
} chassis_control_context_t;

/* Snapshot used by the one-page left-front hip shake diagnostic.  Keep the
 * values at the virtual-leg layer so the chassis task can place them beside
 * the final four DM commands and feedback without exposing control internals. */
typedef struct
{
    float leg_length_l;
    float leg_length_r;
    float theta_raw_diff;
    float theta_lqr_diff;
    float theta_rate_l;
    float theta_rate_r;
    float theta_rate_diff;
    float theta_sync_p;
    float theta_sync_d;
    float theta_sync_guard;
    float lqr_tp_l;
    float lqr_tp_r;
    float final_tp_l;
    float final_tp_r;
    float support_force_l;
    float support_force_r;
    float length_force_l;
    float length_force_r;
    float roll_force;
    float roll_p;
    float roll_d;
} chassis_hip_shake_debug_t;

void chassis_control_module_bind(const chassis_control_context_t *context);
void chassis_control_module_init(leg_config_t *leg_config);

leg_obj_t *chassis_control_module_leg(chassis_side_e side);
pid_obj_t *chassis_control_module_theta_pid(void);
pid_obj_t *chassis_control_module_roll_pid(void);
pid_obj_t *chassis_control_module_length_pid(chassis_side_e side);
float *chassis_control_module_vmc_output(chassis_side_e side);
float (*chassis_control_module_lqr_output(void))[2];
float (*chassis_control_module_lqr_x_obs(void))[6];
float *chassis_control_module_lqr_virtual_x(chassis_side_e side);
float *chassis_control_module_yaw_target(void);
float *chassis_control_module_wheel_speed_phys(chassis_side_e side);
void chassis_control_module_get_hip_shake_debug(
    chassis_hip_shake_debug_t *snapshot);
uint8_t chassis_control_module_jump_peak_torque_active(void);
uint8_t chassis_control_module_bump_torque_budget_active(void);
uint8_t chassis_control_module_bump_low_lqr_active(void);
uint8_t chassis_control_module_jump_fault(void);
uint8_t chassis_control_module_jump_state(void);
uint8_t chassis_control_module_jump_fast_poll_active(void);
float chassis_control_module_jump_elapsed_s(void);
float chassis_control_module_jump_cycle_raw_dt_ms(void);
float chassis_control_module_jump_leg_length_rate(chassis_side_e side);
float chassis_control_module_jump_length_target(void);
uint8_t chassis_control_module_jump_thrust_active(void);
uint8_t chassis_control_module_jump_stroke_release_reason(void);
float chassis_control_module_jump_roll_diff_force(void);
float chassis_control_module_jump_air_wheel_torque(void);
float chassis_control_module_jump_air_wheel_pitch_ref_rad(void);
float chassis_control_module_jump_air_wheel_pitch_err_rad(void);
float chassis_control_module_jump_air_wheel_pitch_rate(void);
uint8_t chassis_control_module_jump_wheel_output_source(void);

void chassis_control_module_set_leg_lengths(float left, float right);
void chassis_control_module_set_yaw_target(float target);
void chassis_control_module_process_clear(void);
void chassis_control_module_bump_reset(void);
void chassis_control_module_clear_recovery_runtime(void);
void chassis_control_module_update_yaw(void);
void chassis_control_module_kf_update(void);
void chassis_control_module_step(void);
void chassis_control_module_capture_wheel_feedback(chassis_side_e side,
                                                   float speed_aps);
int16_t chassis_control_module_wheel_current(chassis_side_e side,
                                             dji_motor_measure_t measure);
uint8_t chassis_control_module_fill_debug_page(uint8_t page);
uint8_t chassis_control_module_fill_jump_rate_debug_page(uint16_t status);
uint8_t chassis_control_module_fill_jump_handoff_debug_page(uint16_t status);
uint8_t chassis_control_module_fill_jump_capture_debug_page(uint16_t status);
uint8_t chassis_control_module_fill_jump_balance_debug_page(uint16_t status);
uint8_t chassis_control_module_fill_jump_ground_recovery_debug_page(uint16_t status);

#endif /* CHASSIS_CONTROL_H */
