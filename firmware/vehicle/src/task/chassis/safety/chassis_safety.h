/**
 * @file chassis_safety.h
 * @brief Safety, off-ground detection, and airborne-control interface.
 */
#ifndef CHASSIS_SAFETY_H
#define CHASSIS_SAFETY_H

#include <stdint.h>
#include "chassis_private.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"

typedef struct
{
    leg_obj_t *leg[CHASSIS_SIDE_COUNT];
    struct ins_msg *ins;
    struct chassis_cmd_msg *cmd;
    struct chassis_fdb_msg *fdb;
    pid_obj_t *theta_pid;
    pid_obj_t *roll_pid;
    pid_obj_t *length_pid[CHASSIS_SIDE_COUNT];
    float (*lqr_output)[2];
    float *lqr_virtual_x[CHASSIS_SIDE_COUNT];
    float *yaw_target;
    const float *dm_obs;
    dm_motor_object_t *dm_motor[4];
    const float *wheel_speed_phys[CHASSIS_SIDE_COUNT];
    float control_dt_s;
} chassis_safety_context_t;

void chassis_safety_module_bind(const chassis_safety_context_t *context);
void chassis_safety_module_reset(void);
void chassis_safety_module_set_position_feedback(
    const float length[CHASSIS_SIDE_COUNT],
    const float theta[CHASSIS_SIDE_COUNT],
    const uint32_t rx_count[4], uint8_t feedback_fresh,
    float max_feedback_age_ms);
void chassis_safety_module_capture_raw_support(float left_force, float right_force);
void chassis_safety_module_freeze_integrators_if_airborne(void);
void chassis_safety_module_update_ground(void);
uint8_t chassis_safety_module_apply_airborne_control(void);
uint8_t chassis_safety_module_landing_fast_exit_ready(
    float left_length, float right_length);
uint8_t chassis_safety_module_landing_fast_exit_begin(void);
uint8_t chassis_safety_module_landing_fast_exit_active(void);
uint8_t chassis_safety_module_landing_fast_exit_take_pid_seed(void);
void chassis_safety_module_landing_fast_exit_cancel_if_unsafe(void);
void chassis_safety_module_landing_fast_exit_finish_cycle(void);
void chassis_safety_module_landing_fast_exit_apply_support(
    float dt_s, float *left_force, float *right_force);
void chassis_safety_module_landing_cushion_apply(
    float dt_s, float *left_force, float *right_force);
void chassis_safety_module_check_danger(void);

uint8_t chassis_safety_module_airborne_allowed(void);
uint8_t chassis_safety_module_off_ground(void);
uint8_t chassis_safety_module_ground_contact_confirmed(void);
uint8_t chassis_safety_module_wheel_shut(void);
void chassis_safety_module_set_wheel_shut(uint8_t wheel_shut);
/* BUMP 使用通用离地/落地观测器，但不改变 CHASSIS_JUMP 或 jump_state。 */
void chassis_safety_module_set_bump_active(uint8_t active);
void chassis_safety_module_set_bump_action_active(uint8_t active);
uint8_t chassis_safety_module_bump_airborne_lockout(void);
uint8_t chassis_safety_module_bump_pose_owner_active(void);
uint8_t chassis_safety_module_takeoff_edge(void);
uint8_t chassis_safety_module_landing_edge(void);
void chassis_safety_module_force_airborne(void);
float chassis_safety_module_landing_gain(void);
uint8_t chassis_safety_module_landing_buffer_active(void);
/* v20: the released landing buffer hands its length-PID tuning to this window,
 * so the 10 Hz post-touchdown leg ring stays damped. */
uint8_t chassis_safety_module_landing_postbuffer_active(void);
float chassis_safety_module_landing_postbuffer_gain(void);
uint8_t chassis_safety_module_landing_buffer_jump_origin(void);
uint8_t chassis_safety_module_jump_landing_owner_active(void);
/* v18: nonzero while a jump-origin landing's post-touchdown airborne guard is
 * armed, so an off-ground bounce keeps the jump attitude channels. */
uint8_t chassis_safety_module_landing_airborne_guard_active(void);
uint8_t chassis_safety_module_danger_reason(void);
uint8_t chassis_safety_module_danger_reason_latched(void);
void chassis_safety_module_clear_danger_latch(void);
uint8_t chassis_safety_module_landing_buffer_phase(void);
float chassis_safety_module_air_length_sync_force(void);
float chassis_safety_module_air_lqr_tp_common(void);
float chassis_safety_module_air_theta_hold_tp_common(void);
float chassis_safety_module_air_length_target(chassis_side_e side);
float chassis_safety_module_airborne_elapsed(void);
uint8_t chassis_safety_module_jump_air_profile_descent(void);
float chassis_safety_module_offground_filtered_force(chassis_side_e side);
float chassis_safety_module_raw_support_candidate(chassis_side_e side);
uint8_t chassis_safety_module_jump_legs_unloaded(void);
void chassis_safety_module_jump_air_profile_prestart(float elapsed_s);
void chassis_safety_module_jump_air_profile_cancel(void);
void chassis_safety_module_set_jump_legs_unloaded(uint8_t unloaded);
void chassis_safety_module_set_jump_kinematic_rates(
    float length_rate_left, float length_rate_right,
    float theta_rate_left, float theta_rate_right,
    uint8_t usable, uint8_t fresh, float valid_rate_age_ms);
void chassis_safety_module_set_jump_landing_length_rates(
    float length_rate_left, float length_rate_right, uint8_t valid);
void chassis_safety_module_set_jump_air_theta_ref(float theta_ref);
float chassis_safety_module_jump_air_theta_ref(void);
void chassis_safety_module_jump_air_profile_set_descent(uint8_t descending);
uint8_t chassis_safety_module_jump_air_profile_active(void);
uint8_t chassis_safety_module_jump_air_compact_phase(void);
void chassis_safety_module_set_jump_position_feedback_fresh(uint8_t fresh);
uint8_t chassis_safety_module_jump_air_actual_compact(void);
uint8_t chassis_safety_module_jump_air_deploy_started(void);
float chassis_safety_module_jump_air_actual_compact_hold(void);
float chassis_safety_module_landing_pid_return_gain(void);
void chassis_safety_module_landing_length_override(
    float nominal_left, float nominal_right,
    float *output_left, float *output_right);
void chassis_safety_module_start_landing_yaw_hold(void);
uint8_t chassis_safety_module_step_landing_yaw_hold(float dt);
uint8_t chassis_safety_module_fill_debug_page(uint8_t page);

#endif /* CHASSIS_SAFETY_H */
