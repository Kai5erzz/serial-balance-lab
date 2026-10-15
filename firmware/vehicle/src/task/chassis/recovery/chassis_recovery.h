/**
 * @file chassis_recovery.h
 * @brief Torque-controlled recovery module interface.
 */
#ifndef CHASSIS_RECOVERY_H
#define CHASSIS_RECOVERY_H

#include "chassis_private.h"
#include "rm_algorithm.h"
#include "rm_module.h"
#include "robot.h"

/* The task owns all shared chassis state. Recovery only receives the references
 * it needs, so it never reaches back into chassis_task.c through extern data. */
typedef struct
{
    leg_obj_t *leg[CHASSIS_SIDE_COUNT];
    struct ins_msg *ins;
    struct chassis_cmd_msg *cmd;
    struct chassis_fdb_msg *fdb;
    pid_obj_t *theta_pid;
    pid_obj_t *roll_pid;
    pid_obj_t *length_pid[CHASSIS_SIDE_COUNT];
    float *vmc_out[CHASSIS_SIDE_COUNT];
    float (*lqr_x_obs)[6];
    float (*lqr_output)[2];
    float *lqr_virtual_x[CHASSIS_SIDE_COUNT];
    const float *dm_obs;
    float control_dt_s;
    float low_leg_length;
} chassis_recovery_context_t;

void chassis_recovery_module_bind(const chassis_recovery_context_t *context);
void chassis_recovery_module_reset(void);
void chassis_recovery_module_clear_handoff(void);
void chassis_recovery_module_update_mode(void);
void chassis_recovery_module_step(void);
void chassis_recovery_module_apply_length_hold(void);
uint8_t chassis_recovery_module_apply_retract_control(void);
void chassis_recovery_module_apply_handoff_blend(void);
void chassis_recovery_module_capture_debug(void);
void chassis_recovery_module_update_handoff_lqr(void);

uint8_t chassis_recovery_module_has_stood_up(void);
uint8_t chassis_recovery_module_after_handoff(void);
float chassis_recovery_module_handoff_gain(void);
float chassis_recovery_module_dm_output_limit(float nominal_limit);
uint8_t chassis_recovery_module_debug_state(void);
uint8_t chassis_recovery_module_debug_fault_reason(void);
uint8_t chassis_recovery_module_fill_debug_page(uint8_t page);

#endif /* CHASSIS_RECOVERY_H */
