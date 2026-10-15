#ifndef CHASSIS_JUMP_GROUND_RECOVERY_H
#define CHASSIS_JUMP_GROUND_RECOVERY_H

#include <math.h>
#include <stdint.h>

#define JUMP_GROUND_RECOVERY_HOLD_S 0.500f
#define JUMP_GROUND_RECOVERY_FADE_S 0.200f

typedef struct {
    float elapsed_after_buffer_s;
    uint8_t latched;
    uint8_t consumed;
} jump_ground_recovery_t;

typedef struct {
    float gain;
    uint8_t entered;
    uint8_t invalid_observer;
} jump_ground_recovery_result_t;

static inline void jump_ground_recovery_reset(jump_ground_recovery_t *s)
{
    if (s == 0) return;
    s->elapsed_after_buffer_s = 0.0f;
    s->latched = 0U;
    s->consumed = 0U;
}

static inline void jump_ground_recovery_cancel(jump_ground_recovery_t *s)
{
    if (s == 0) return;
    s->elapsed_after_buffer_s = 0.0f;
    s->latched = 0U;
    s->consumed = 1U;
}

/* Lock once at confirmed jump-origin contact. The jump remains in this
 * recovery scope after the landing buffer ends; invalid observer data is a
 * hard failure while the scope is active, never an implicit zero rate. */
static inline jump_ground_recovery_result_t jump_ground_recovery_step(
    jump_ground_recovery_t *s, float dt_s, uint8_t advance,
    uint8_t jump_origin_contact, uint8_t buffer_active,
    uint8_t grounded, uint8_t safe_mode, uint8_t observer_valid,
    float theta_rate_left, float theta_rate_right)
{
    jump_ground_recovery_result_t r = {0.0f, 0U, 0U};
    if (s == 0) { r.invalid_observer = 1U; return r; }

    if (!grounded || !safe_mode) {
        if (s->latched) jump_ground_recovery_cancel(s);
        return r;
    }

    if (!s->latched && !s->consumed && jump_origin_contact) {
        if (!observer_valid || !isfinite(theta_rate_left) ||
            !isfinite(theta_rate_right)) {
            r.invalid_observer = 1U;
            jump_ground_recovery_cancel(s);
            return r;
        }
        s->latched = 1U;
        s->elapsed_after_buffer_s = 0.0f;
        r.entered = 1U;
    }

    if (!s->latched) return r;
    if (!observer_valid || !isfinite(theta_rate_left) ||
        !isfinite(theta_rate_right)) {
        r.invalid_observer = 1U;
        jump_ground_recovery_cancel(s);
        return r;
    }

    if (buffer_active) {
        s->elapsed_after_buffer_s = 0.0f;
        r.gain = 1.0f;
        return r;
    }
    if (advance && isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.050f) {
        s->elapsed_after_buffer_s += dt_s;
    }
    if (s->elapsed_after_buffer_s <= JUMP_GROUND_RECOVERY_HOLD_S) {
        r.gain = 1.0f;
    } else {
        float fade = (s->elapsed_after_buffer_s -
                      JUMP_GROUND_RECOVERY_HOLD_S) /
                     JUMP_GROUND_RECOVERY_FADE_S;
        if (fade >= 1.0f) {
            s->consumed = 1U;
            s->latched = 0U;
            s->elapsed_after_buffer_s = 0.0f;
            r.gain = 0.0f;
        } else {
            r.gain = 1.0f - fade;
        }
    }
    return r;
}

static inline float jump_ground_recovery_clamp(float x, float limit)
{
    if (!isfinite(x) || !isfinite(limit) || limit <= 0.0f) return 0.0f;
    if (x > limit) return limit;
    if (x < -limit) return -limit;
    return x;
}

/* Preserve legacy allocation outside recovery scope. In scope, first form
 * the raw common mode and clamp it as one actuator request; interpolate from
 * the old independently-clamped allocation for a bumpless recovery fade. */
static inline float jump_ground_recovery_common(float raw_left,
    float raw_right, float torque_limit, float recovery_gain)
{
    if (!isfinite(raw_left) || !isfinite(raw_right) ||
        !isfinite(torque_limit) || torque_limit <= 0.0f ||
        !isfinite(recovery_gain)) return 0.0f;
    if (recovery_gain < 0.0f) recovery_gain = 0.0f;
    if (recovery_gain > 1.0f) recovery_gain = 1.0f;
    const float legacy = 0.5f *
        (jump_ground_recovery_clamp(raw_left, torque_limit) +
         jump_ground_recovery_clamp(raw_right, torque_limit));
    const float paired = jump_ground_recovery_clamp(
        0.5f * (raw_left + raw_right), torque_limit);
    return legacy + recovery_gain * (paired - legacy);
}

#endif
