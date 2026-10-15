#ifndef CHASSIS_JUMP_LANDING_FAST_EXIT_H
#define CHASSIS_JUMP_LANDING_FAST_EXIT_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* 2026-10-04 (jump100): 0.25 -> 0.28 m and a two-cycle confirmation.  jump100
 * measured the old single-sample 0.25 m crossing firing in the middle of the
 * touchdown compression transient (LANDING lasted only 50-70 ms, with the rods
 * still moving at 3-9 rad/s), so the jump-specific landing controller was
 * released before it could act.  The motor cushion (jump_landing_cushion.h)
 * now brakes the compression, and this gate only releases once BOTH legs have
 * been below the (higher) 0.28 m threshold for two consecutive control cycles,
 * i.e. after the cushion has arrested the stroke rather than on its first
 * sample through it. */
#define JUMP_LANDING_FAST_EXIT_LENGTH_M 0.28f
#define JUMP_LANDING_FAST_EXIT_CONFIRM_CYCLES 2U
#define JUMP_LANDING_AXIAL_HANDOFF_S    0.040f
#define JUMP_LANDING_FAST_WHEEL_HANDOFF_S 0.040f
#define JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S 0.250f

static inline float jump_landing_average_abs_length_rate(
    uint8_t rate_valid, float left_rate, float right_rate)
{
    if (!rate_valid || !isfinite(left_rate) || !isfinite(right_rate)) {
        return INFINITY;
    }
    return 0.5f * (fabsf(left_rate) + fabsf(right_rate));
}

typedef struct {
    uint8_t hold_cycles;
} jump_landing_fast_exit_confirm_t;

static inline void jump_landing_fast_exit_confirm_reset(
    jump_landing_fast_exit_confirm_t *state)
{
    if (state == NULL) return;
    state->hold_cycles = 0U;
}

/* One call per control cycle with the instantaneous eligibility.  The caller
 * owns the counter so the condition has to hold for
 * JUMP_LANDING_FAST_EXIT_CONFIRM_CYCLES consecutive cycles. */
static inline uint8_t jump_landing_fast_exit_confirm_step(
    jump_landing_fast_exit_confirm_t *state, uint8_t eligible)
{
    if (state == NULL) return 0U;
    if (!eligible) {
        state->hold_cycles = 0U;
        return 0U;
    }
    if (state->hold_cycles < JUMP_LANDING_FAST_EXIT_CONFIRM_CYCLES) {
        state->hold_cycles = (uint8_t)(state->hold_cycles + 1U);
    }
    return (uint8_t)(state->hold_cycles >=
                     JUMP_LANDING_FAST_EXIT_CONFIRM_CYCLES);
}

static inline float jump_landing_wheel_handoff_duration(uint8_t fast_exit)
{
    return fast_exit ? JUMP_LANDING_FAST_WHEEL_HANDOFF_S :
                       JUMP_LANDING_NORMAL_WHEEL_HANDOFF_S;
}

typedef struct {
    float start_left;
    float start_right;
    float elapsed_s;
    uint8_t active;
    uint8_t first_output;
} jump_landing_axial_handoff_t;

static inline uint8_t jump_landing_fast_exit_eligible(
    uint8_t buffer_active, uint8_t buffer_hold, uint8_t buffer_settled,
    uint8_t jump_origin,
    uint8_t contact_confirmed,
    uint8_t off_ground, uint8_t safe_owner_mode,
    float left_length, float right_length)
{
    return (uint8_t)(buffer_active && buffer_hold && buffer_settled &&
        jump_origin &&
        contact_confirmed &&
        !off_ground && safe_owner_mode &&
        isfinite(left_length) && isfinite(right_length) &&
        left_length > 0.0f && right_length > 0.0f &&
        left_length < JUMP_LANDING_FAST_EXIT_LENGTH_M &&
        right_length < JUMP_LANDING_FAST_EXIT_LENGTH_M);
}

/* The short axial handoff is only safe after the landing buffer has actually
 * arrested the impact.  Keep the numeric part separate from the owner/mode
 * gate above so it can be exercised without the full chassis context. */
static inline uint8_t jump_landing_fast_exit_hold_settled(
    uint8_t buffer_hold, float phase_elapsed_s, float phase_min_s,
    float average_length_speed, float speed_limit,
    float left_length, float right_length,
    float left_reference, float right_reference, float length_error_limit)
{
    return (uint8_t)(buffer_hold &&
        isfinite(phase_elapsed_s) && isfinite(phase_min_s) &&
        phase_min_s >= 0.0f && phase_elapsed_s >= phase_min_s &&
        isfinite(average_length_speed) &&
        average_length_speed >= 0.0f &&
        isfinite(speed_limit) && speed_limit >= 0.0f &&
        average_length_speed <= speed_limit &&
        isfinite(left_length) && isfinite(right_length) &&
        isfinite(left_reference) && isfinite(right_reference) &&
        isfinite(length_error_limit) && length_error_limit >= 0.0f &&
        fabsf(left_length - left_reference) <= length_error_limit &&
        fabsf(right_length - right_reference) <= length_error_limit);
}

static inline uint8_t jump_landing_fast_exit_preserve_ground_detection(
    uint8_t handoff_active, uint8_t jump_mode, uint8_t jumpok_state)
{
    return (uint8_t)(handoff_active && jump_mode && jumpok_state);
}

/* A jump-origin impact estimate is generated from unloaded airborne leg
 * forces, so it cannot open the landing buffer before the ground edge.  Keep
 * the ordinary/precontact path available for non-jump modes. */
static inline uint8_t jump_landing_impact_admitted(
    uint8_t jump_origin, uint8_t off_ground)
{
    return (uint8_t)(!(jump_origin && off_ground));
}

static inline uint8_t jump_landing_axial_handoff_begin(
    jump_landing_axial_handoff_t *state, float last_left, float last_right)
{
    if (state == NULL) {
        return 0U;
    }
    if (!isfinite(last_left) || !isfinite(last_right)) {
        state->start_left = 0.0f;
        state->start_right = 0.0f;
        state->elapsed_s = 0.0f;
        state->active = 0U;
        state->first_output = 0U;
        return 0U;
    }
    state->start_left = last_left;
    state->start_right = last_right;
    state->elapsed_s = 0.0f;
    state->active = 1U;
    state->first_output = 1U;
    return 1U;
}

static inline void jump_landing_axial_handoff_cancel(
    jump_landing_axial_handoff_t *state)
{
    if (state == NULL) return;
    state->start_left = 0.0f;
    state->start_right = 0.0f;
    state->elapsed_s = 0.0f;
    state->active = 0U;
    state->first_output = 0U;
}

/* Preserve the captured command exactly on the first normal-control sample;
 * then linearly hand it to the normal candidate over 40 ms. */
static inline void jump_landing_axial_handoff_apply(
    jump_landing_axial_handoff_t *state, float dt_s, uint8_t safe_ground,
    float normal_left, float normal_right,
    float *output_left, float *output_right)
{
    if (output_left != NULL) *output_left = normal_left;
    if (output_right != NULL) *output_right = normal_right;
    if (state == NULL || !state->active) return;
    if (!safe_ground || !isfinite(normal_left) || !isfinite(normal_right)) {
        jump_landing_axial_handoff_cancel(state);
        return;
    }

    if (state->first_output) {
        state->first_output = 0U;
    } else if (isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.050f) {
        state->elapsed_s += dt_s;
    }

    float alpha = state->elapsed_s / JUMP_LANDING_AXIAL_HANDOFF_S;
    if (state->elapsed_s >= JUMP_LANDING_AXIAL_HANDOFF_S - 1e-6f) {
        alpha = 1.0f;
    } else if (alpha > 1.0f) {
        alpha = 1.0f;
    }
    if (output_left != NULL) {
        *output_left = state->start_left +
            alpha * (normal_left - state->start_left);
    }
    if (output_right != NULL) {
        *output_right = state->start_right +
            alpha * (normal_right - state->start_right);
    }
}

static inline void jump_landing_axial_handoff_finish_cycle(
    jump_landing_axial_handoff_t *state, uint8_t safe_ground)
{
    if (state == NULL || !state->active) return;
    if (!safe_ground) {
        jump_landing_axial_handoff_cancel(state);
    } else if (state->elapsed_s >=
               JUMP_LANDING_AXIAL_HANDOFF_S - 1e-6f) {
        jump_landing_axial_handoff_cancel(state);
    }
}

#endif /* CHASSIS_JUMP_LANDING_FAST_EXIT_H */
