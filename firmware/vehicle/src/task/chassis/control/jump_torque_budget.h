#ifndef JUMP_TORQUE_BUDGET_H
#define JUMP_TORQUE_BUDGET_H

#include <math.h>
#include <stdint.h>

#define JUMP_HIP_TORQUE_BUDGET_NM 120.0f
#define JUMP_HIP_TORQUE_SLOT_COUNT 4U

typedef enum {
    JUMP_HIP_LB = 0U,
    JUMP_HIP_RB = 1U,
    JUMP_HIP_RF = 2U,
    JUMP_HIP_LF = 3U
} jump_hip_torque_slot_t;

typedef struct {
    float torque[JUMP_HIP_TORQUE_SLOT_COUNT];
    uint8_t per_motor_clipped[JUMP_HIP_TORQUE_SLOT_COUNT];
    float pre_scale_abs_sum;
    float scale;
    uint8_t valid;
    uint8_t uniform_scaled;
} jump_torque_budget_vector_t;

typedef struct {
    float torque;
    float available_abs;
    uint8_t valid;
    uint8_t residual_limited;
} jump_torque_budget_residual_t;

/* Convert the four VMC outputs to the physical DM command signs/order, apply
 * the already-selected per-motor limit, then uniformly scale the vector to
 * the aggregate L1 budget. Call with one captured limit for a coherent frame.
 */
static inline jump_torque_budget_vector_t jump_torque_budget_prepare_vmc(
    float left_front, float left_back, float right_front, float right_back,
    float per_motor_limit, float budget)
{
    jump_torque_budget_vector_t result = {
        .torque = {0.0f, 0.0f, 0.0f, 0.0f},
        .per_motor_clipped = {0U, 0U, 0U, 0U},
        .pre_scale_abs_sum = 0.0f,
        .scale = 1.0f,
        .valid = 0U,
        .uniform_scaled = 0U
    };
    if (!isfinite(left_front) || !isfinite(left_back) ||
        !isfinite(right_front) || !isfinite(right_back) ||
        !isfinite(per_motor_limit) || per_motor_limit <= 0.0f ||
        !isfinite(budget) || budget <= 0.0f) {
        return result;
    }

    result.torque[JUMP_HIP_LB] = -left_back;
    result.torque[JUMP_HIP_RB] = right_back;
    result.torque[JUMP_HIP_RF] = right_front;
    result.torque[JUMP_HIP_LF] = -left_front;
    for (uint8_t i = 0U; i < JUMP_HIP_TORQUE_SLOT_COUNT; ++i) {
        float *torque = &result.torque[i];
        if (*torque > per_motor_limit) {
            *torque = per_motor_limit;
            result.per_motor_clipped[i] = 1U;
        }
        if (*torque < -per_motor_limit) {
            *torque = -per_motor_limit;
            result.per_motor_clipped[i] = 1U;
        }
        result.pre_scale_abs_sum += fabsf(*torque);
    }
    if (!isfinite(result.pre_scale_abs_sum)) {
        result.torque[0] = 0.0f;
        result.torque[1] = 0.0f;
        result.torque[2] = 0.0f;
        result.torque[3] = 0.0f;
        result.pre_scale_abs_sum = 0.0f;
        return result;
    }

    result.valid = 1U;
    if (result.pre_scale_abs_sum > budget) {
        result.scale = budget / result.pre_scale_abs_sum;
        result.uniform_scaled = 1U;
        for (uint8_t i = 0U; i < JUMP_HIP_TORQUE_SLOT_COUNT; ++i) {
            result.torque[i] *= result.scale;
        }
    }
    return result;
}

/* Apply the asynchronous command-slot guard after all existing per-motor
 * clamps/zero overrides. `other_abs_sum` is the sum of the other three latest
 * callback software candidates, not a measurement of transmitted motor torque.
 */
static inline jump_torque_budget_residual_t jump_torque_budget_apply_residual(
    float candidate, float other_abs_sum, float budget)
{
    jump_torque_budget_residual_t result = {0.0f, 0.0f, 0U, 0U};
    if (!isfinite(candidate) || !isfinite(other_abs_sum) ||
        other_abs_sum < 0.0f || !isfinite(budget) || budget <= 0.0f) {
        return result;
    }
    result.valid = 1U;
    result.available_abs = other_abs_sum < budget ?
                           budget - other_abs_sum : 0.0f;
    result.torque = candidate;
    if (result.torque > result.available_abs) {
        result.torque = result.available_abs;
        result.residual_limited = 1U;
    } else if (result.torque < -result.available_abs) {
        result.torque = -result.available_abs;
        result.residual_limited = 1U;
    }
    return result;
}

/* After leaving jump mode, keep the asynchronous guard until each of the four
 * callback slots has received one non-jump update. */
static inline uint8_t jump_torque_budget_publish_exit_mask(
    uint8_t current_mask, uint8_t was_jump_active, uint8_t is_jump_active)
{
    if (is_jump_active) return 0U;
    return was_jump_active ? 0x0fU : current_mask;
}

static inline uint8_t jump_torque_budget_callback_exit_mask(
    uint8_t current_mask, uint8_t slot)
{
    return slot < JUMP_HIP_TORQUE_SLOT_COUNT ?
           (uint8_t)(current_mask & (uint8_t)~(1U << slot)) : current_mask;
}

static inline uint16_t jump_torque_budget_page57_bits(
    uint8_t vector_uniformly_scaled, uint8_t latest_callback_residual_limited)
{
    uint16_t bits = 0U;
    if (vector_uniformly_scaled) bits |= (uint16_t)(1U << 14);
    if (latest_callback_residual_limited) bits |= (uint16_t)(1U << 15);
    return bits;
}

#endif
