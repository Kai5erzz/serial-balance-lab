/**
 * @file chassis_private.h
 * @brief Chassis-internal module boundary definitions.
 *
 * This header is intentionally private to src/task/chassis.  It is the single
 * future home for data exchanged by control, recovery, safety, and debug.
 * Hardware lifecycle, MCN, and motor output stay in chassis_task.c.
 */
#ifndef CHASSIS_PRIVATE_H
#define CHASSIS_PRIVATE_H

#include <stdint.h>

#include "chassis_task.h"

typedef enum
{
    CHASSIS_SIDE_LEFT = 0,
    CHASSIS_SIDE_RIGHT,
    CHASSIS_SIDE_COUNT,
} chassis_side_e;

/* Phase 1 only defines transfer shapes. Runtime state is not duplicated here
 * until its owning logic is moved out of chassis_task.c. */
typedef struct
{
    float control_dt_s;
    chassis_mode_e mode;
    chassis_stand_state_e stand_state;
} chassis_module_snapshot_t;

typedef struct
{
    float wheel_torque[CHASSIS_SIDE_COUNT];
    float leg_force[CHASSIS_SIDE_COUNT];
    float leg_torque[CHASSIS_SIDE_COUNT];
} chassis_control_target_t;

typedef struct
{
    uint8_t off_ground;
    uint8_t danger;
    uint8_t wheel_shut;
} chassis_safety_state_t;

#endif /* CHASSIS_PRIVATE_H */
