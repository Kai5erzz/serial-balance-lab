#ifndef CHASSIS_JUMP_LANDING_OWNERSHIP_H
#define CHASSIS_JUMP_LANDING_OWNERSHIP_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    float start_left;
    float start_right;
    float elapsed_s;
    uint8_t active;
} jump_landing_wheel_fade_t;

typedef struct {
    float ground_elapsed_s;
    float lost_contact_s;
    float captured_theta;
    uint8_t capture_valid;
} jump_landing_balance_state_t;

typedef struct {
    float gain;
    float captured_theta;
    uint8_t captured_this_step;
    uint8_t invalid_capture;
} jump_landing_balance_result_t;

/* Begin posture transfer on the first confirmed-ground sample; complete the
 * capture-to-ground blend over 40 ms with no additional quiet hold. */
#define JUMP_LANDING_BALANCE_CAPTURE_DELAY_S 0.000f
#define JUMP_LANDING_BALANCE_RAMP_S           0.040f
#define JUMP_LANDING_BALANCE_LOSS_RESET_S    0.080f
#define JUMP_LANDING_BALANCE_DT_MAX_S        0.050f

enum {
    JUMP_LANDING_WHEEL_SOURCE_ZERO = 0U,
    JUMP_LANDING_WHEEL_SOURCE_REACTION = 2U,
    JUMP_LANDING_WHEEL_SOURCE_DAMPER = 3U,
    JUMP_LANDING_WHEEL_SOURCE_NORMAL = 4U,
    JUMP_LANDING_WHEEL_SOURCE_FADE = 5U,
    JUMP_LANDING_WHEEL_SOURCE_RETURN_BLEND = 6U
};

static inline uint8_t jump_landing_jump_stand_allowed(
    uint8_t jump_mode, uint8_t jumping_state, uint8_t jumpok_state)
{
    return (uint8_t)(jump_mode && (jumping_state || jumpok_state));
}

static inline uint8_t jump_landing_owner_mode_allowed(
    uint8_t buffer_active, uint8_t jump_origin, uint8_t safe_state,
    uint8_t normal_mode_allowed, uint8_t jump_mode_allowed)
{
    return (uint8_t)(buffer_active && jump_origin && safe_state &&
                     (normal_mode_allowed || jump_mode_allowed));
}

static inline uint8_t jump_landing_total_timeout_expired(
    uint32_t elapsed_ms, uint32_t timeout_ms, uint8_t owner_active,
    uint8_t off_ground)
{
    return (uint8_t)(elapsed_ms >= timeout_ms &&
                     !(owner_active && !off_ground));
}

static inline uint8_t jump_landing_pose_owner_active(
    uint8_t landing_owner_active, uint8_t postbuffer_active,
    uint8_t buffer_active, uint8_t buffer_jump_origin)
{
    return (uint8_t)(landing_owner_active || postbuffer_active ||
                     (buffer_active && buffer_jump_origin));
}

static inline void jump_landing_balance_reset(
    jump_landing_balance_state_t *state)
{
    if (state == NULL) return;
    state->ground_elapsed_s = 0.0f;
    state->lost_contact_s = 0.0f;
    state->captured_theta = 0.0f;
    state->capture_valid = 0U;
}

static inline float jump_landing_balance_theta_ref(float captured_raw_theta,
                                                    float theta_total_comp,
                                                    float gain)
{
    if (!isfinite(captured_raw_theta) || !isfinite(theta_total_comp) ||
        !isfinite(gain)) return NAN;
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.0f) gain = 1.0f;
    return (1.0f - gain) * (captured_raw_theta - theta_total_comp);
}

/* Progress posture authority only on confirmed grounded owner cycles. A short
 * rebound freezes the ramp; a longer loss invalidates the old touchdown pose. */
static inline jump_landing_balance_result_t jump_landing_balance_step(
    jump_landing_balance_state_t *state, float dt_s,
    uint8_t owner_active, uint8_t contact_confirmed, uint8_t off_ground,
    float theta_common)
{
    jump_landing_balance_result_t result = {0.0f, 0.0f, 0U, 0U};
    uint8_t dt_valid;

    if (state == NULL) return result;
    dt_valid = (uint8_t)(isfinite(dt_s) && dt_s > 0.0f &&
                         dt_s <= JUMP_LANDING_BALANCE_DT_MAX_S);
    if (!owner_active) {
        jump_landing_balance_reset(state);
        return result;
    }

    if (contact_confirmed && !off_ground) {
        state->lost_contact_s = 0.0f;
        if (!state->capture_valid) {
            if (!isfinite(theta_common)) {
                result.invalid_capture = 1U;
                return result;
            }
            state->captured_theta = theta_common;
            state->capture_valid = 1U;
            state->ground_elapsed_s = 0.0f;
            result.captured_this_step = 1U;
        } else if (dt_valid) {
            state->ground_elapsed_s += dt_s;
        }
    } else if (state->capture_valid && dt_valid) {
        state->lost_contact_s += dt_s;
        if (state->lost_contact_s > JUMP_LANDING_BALANCE_LOSS_RESET_S) {
            jump_landing_balance_reset(state);
        }
    }

    if (!state->capture_valid) return result;
    result.captured_theta = state->captured_theta;
    const float ramp_elapsed = state->ground_elapsed_s -
        JUMP_LANDING_BALANCE_CAPTURE_DELAY_S;
    if (ramp_elapsed > 0.0f) {
        result.gain = ramp_elapsed / JUMP_LANDING_BALANCE_RAMP_S;
        if (result.gain > 1.0f) result.gain = 1.0f;
    }
    return result;
}

/* Used by RETURN exit logic and its off-ground branch: elapsed time is
 * continuous stable ground time, not wall time across a rebound. */
static inline float jump_landing_exit_stable_elapsed_step(
    float elapsed_s, float dt_s, uint8_t stable_ground)
{
    if (!isfinite(elapsed_s) || elapsed_s < 0.0f) elapsed_s = 0.0f;
    if (!stable_ground) return 0.0f;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.050f) {
        return elapsed_s;
    }
    return elapsed_s + dt_s;
}

static inline float jump_landing_capture_tp(float theta, float theta_ref,
                                             float theta_rate,
                                             uint8_t rate_valid,
                                             float kp, float kd,
                                             float limit,
                                             uint8_t *valid)
{
    float output;
    if (valid != NULL) *valid = 0U;
    if (!rate_valid || !isfinite(theta) || !isfinite(theta_ref) ||
        !isfinite(theta_rate) || !isfinite(kp) || !isfinite(kd) ||
        !isfinite(limit) || limit < 0.0f) {
        return 0.0f;
    }
    output = kp * (theta - theta_ref) + kd * theta_rate;
    if (!isfinite(output)) return 0.0f;
    if (output > limit) output = limit;
    if (output < -limit) output = -limit;
    if (valid != NULL) *valid = 1U;
    return output;
}

static inline float jump_landing_capture_return_blend(float capture_tp,
                                                       float base_tp,
                                                       float return_gain)
{
    if (!isfinite(capture_tp) || !isfinite(base_tp)) return 0.0f;
    if (!isfinite(return_gain)) return base_tp;
    if (return_gain < 0.0f) return_gain = 0.0f;
    if (return_gain > 1.0f) return_gain = 1.0f;
    return (1.0f - return_gain) * capture_tp + return_gain * base_tp;
}

static inline float jump_landing_owner_current_clamp(float current,
                                                      float limit)
{
    if (!isfinite(current) || !isfinite(limit) || limit < 0.0f) return 0.0f;
    if (current > limit) return limit;
    if (current < -limit) return -limit;
    return current;
}

static inline void jump_landing_wheel_fade_begin(
    jump_landing_wheel_fade_t *state, float previous_left,
    float previous_right)
{
    if (state == NULL) return;
    state->start_left = isfinite(previous_left) ? previous_left : 0.0f;
    state->start_right = isfinite(previous_right) ? previous_right : 0.0f;
    state->elapsed_s = 0.0f;
    state->active = 1U;
}

/* Apply the landing wheel owner once, after the caller forms its normal
 * command. Jump-origin RETURN may blend that command with the landing damper
 * using the same beta as the hip owner. Unsupported modes cancel a stale fade
 * and pass their existing command through; the caller's RELAX/off-ground hard
 * gates still run after this helper. */
static inline float jump_landing_wheel_handoff_apply(
    jump_landing_wheel_fade_t *state, uint8_t right_side,
    float normal_command, float damper_command, uint8_t buffer_owner_active,
    uint8_t mode_allowed, uint8_t off_ground,
    uint8_t buffer_return_blend_enabled, float buffer_return_gain,
    float duration_s, float *used_gain, uint8_t *source)
{
    float alpha;
    if (used_gain != NULL) *used_gain = 1.0f;
    if (source != NULL) *source = JUMP_LANDING_WHEEL_SOURCE_NORMAL;
    if (!isfinite(normal_command)) normal_command = 0.0f;
    if (!isfinite(damper_command)) damper_command = 0.0f;
    if (buffer_owner_active || (state != NULL && state->active)) {
        if (off_ground) {
            if (state != NULL) state->active = 0U;
            if (used_gain != NULL) *used_gain = 0.0f;
            if (source != NULL) *source = JUMP_LANDING_WHEEL_SOURCE_ZERO;
            return 0.0f;
        }
        if (!mode_allowed) {
            if (state != NULL) state->active = 0U;
            return normal_command;
        }
        if (buffer_owner_active) {
            if (state != NULL) state->active = 0U;
            if (buffer_return_blend_enabled) {
                float beta = isfinite(buffer_return_gain) ?
                    buffer_return_gain : 0.0f;
                if (beta < 0.0f) beta = 0.0f;
                if (beta > 1.0f) beta = 1.0f;
                if (used_gain != NULL) *used_gain = beta;
                if (beta > 0.0f) {
                    if (source != NULL) {
                        *source = JUMP_LANDING_WHEEL_SOURCE_RETURN_BLEND;
                    }
                    return damper_command +
                        beta * (normal_command - damper_command);
                }
            }
            if (used_gain != NULL) *used_gain = 0.0f;
            if (source != NULL) *source = JUMP_LANDING_WHEEL_SOURCE_DAMPER;
            return damper_command;
        }
        if (state != NULL && state->active && isfinite(duration_s) &&
            duration_s > 0.0f && isfinite(state->elapsed_s)) {
            alpha = state->elapsed_s / duration_s;
            if (alpha < 0.0f) alpha = 0.0f;
            if (alpha > 1.0f) alpha = 1.0f;
            if (used_gain != NULL) *used_gain = alpha;
            if (source != NULL) {
                *source = alpha < 1.0f ?
                    JUMP_LANDING_WHEEL_SOURCE_FADE :
                    JUMP_LANDING_WHEEL_SOURCE_NORMAL;
            }
            const float start = right_side ?
                state->start_right : state->start_left;
            return start + alpha * (normal_command - start);
        }
    }
    return normal_command;
}

static inline void jump_landing_wheel_fade_advance(
    jump_landing_wheel_fade_t *state, float dt_s, float duration_s)
{
    if (state == NULL || !state->active || !isfinite(dt_s) || dt_s < 0.0f ||
        !isfinite(duration_s) || duration_s <= 0.0f) return;
    state->elapsed_s += dt_s;
    if (state->elapsed_s >= duration_s) {
        state->elapsed_s = duration_s;
        state->active = 0U;
    }
}

#endif /* CHASSIS_JUMP_LANDING_OWNERSHIP_H */
