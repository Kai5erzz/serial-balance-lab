#ifndef CHASSIS_JUMP_LANDING_SPEED_H
#define CHASSIS_JUMP_LANDING_SPEED_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define JUMP_LANDING_VX_REACQUIRE_TIME_S 0.100f
#define JUMP_LANDING_VX_REF_ACCEL_MPS2   2.5f

typedef struct {
    float value;
    uint8_t active;
    uint8_t contact_confirmed;
    uint8_t contact_rebase_pending;
    uint8_t cancelled;
} jump_landing_vx_ref_state_t;

typedef void (*jump_landing_kf_update_fn)(void *context);

static inline uint8_t jump_landing_kf_update_if_allowed(
    uint8_t hold, jump_landing_kf_update_fn update, void *context)
{
    if (hold || update == NULL) return 0U;
    update(context);
    return 1U;
}

static inline uint8_t jump_landing_kf_interrupted_airborne(
    uint8_t off_ground, uint8_t airborne_hold_latched, uint8_t jump_flight)
{
    return (uint8_t)(off_ground || airborne_hold_latched || jump_flight);
}

static inline float jump_landing_vx_reacquire_blend(
    float ground_vx, float wheel_vx, float progress)
{
    if (!isfinite(ground_vx)) return 0.0f;
    if (!isfinite(wheel_vx)) return ground_vx;
    if (!isfinite(progress)) progress = 0.0f;
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    return ground_vx + progress * (wheel_vx - ground_vx);
}

static inline float jump_landing_vx_ref_step(
    float current, float command, float dt_s, float accel_mps2)
{
    if (!isfinite(current)) current = 0.0f;
    if (!isfinite(command)) command = current;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.020f ||
        !isfinite(accel_mps2) || accel_mps2 <= 0.0f) {
        return current;
    }
    const float max_step = accel_mps2 * dt_s;
    const float delta = command - current;
    if (delta > max_step) return current + max_step;
    if (delta < -max_step) return current - max_step;
    return command;
}

static inline uint8_t jump_landing_vx_ref_can_step(
    uint8_t contact_confirmed, uint8_t off_ground)
{
    return (uint8_t)(contact_confirmed && !off_ground);
}

static inline uint8_t jump_landing_vx_ref_can_start(
    uint8_t jump_owned_scope, uint8_t cancelled)
{
    return (uint8_t)(jump_owned_scope && !cancelled);
}

static inline uint8_t jump_landing_kf_should_hold(
    uint8_t jump_flight, uint8_t jump_active, uint8_t off_ground,
    uint8_t profile_unloaded, uint8_t landing_origin_scope,
    uint8_t airborne_hold_latched, uint8_t danger, uint8_t guard_airborne,
    uint8_t trusted_contact)
{
    return (uint8_t)(jump_flight || off_ground ||
        (jump_active && (off_ground || profile_unloaded)) ||
        (landing_origin_scope && off_ground) ||
        (airborne_hold_latched &&
         (off_ground || danger || !trusted_contact)) ||
        (guard_airborne && off_ground));
}

static inline uint8_t jump_landing_kf_contact_confirmed(
    uint8_t off_ground, uint8_t landing_origin_scope,
    uint8_t landing_buffer_contact, uint8_t airborne_hold_latched,
    uint8_t jump_idle, uint8_t trusted_contact, uint8_t danger)
{
    return (uint8_t)(!off_ground &&
        ((landing_origin_scope && landing_buffer_contact) ||
         (airborne_hold_latched && jump_idle && trusted_contact && !danger)));
}

static inline float jump_landing_vx_contact_seed(
    float launch_ground_vx, float previous_contact_vx,
    uint8_t has_previous_contact)
{
    const float seed = has_previous_contact ?
        previous_contact_vx : launch_ground_vx;
    return isfinite(seed) ? seed : 0.0f;
}

static inline void jump_landing_vx_ref_cancel(
    jump_landing_vx_ref_state_t *state, uint8_t cancelled)
{
    if (state == NULL) return;
    state->value = 0.0f;
    state->active = 0U;
    state->contact_confirmed = 0U;
    state->contact_rebase_pending = 0U;
    state->cancelled = cancelled ? 1U : 0U;
}

static inline uint8_t jump_landing_vx_ref_start(
    jump_landing_vx_ref_state_t *state, uint8_t jump_owned_scope,
    float initial_ground_vx)
{
    if (state == NULL || state->active ||
        !jump_landing_vx_ref_can_start(jump_owned_scope, state->cancelled)) {
        return 0U;
    }
    state->value = isfinite(initial_ground_vx) ? initial_ground_vx : 0.0f;
    state->active = 1U;
    state->contact_confirmed = 0U;
    state->contact_rebase_pending = 0U;
    return 1U;
}

static inline void jump_landing_vx_ref_mark_airborne(
    jump_landing_vx_ref_state_t *state)
{
    if (state != NULL) state->contact_confirmed = 0U;
}

static inline void jump_landing_vx_ref_confirm_contact(
    jump_landing_vx_ref_state_t *state)
{
    if (state != NULL && state->active && !state->cancelled) {
        state->contact_confirmed = 1U;
        state->contact_rebase_pending = 1U;
    }
}

static inline float jump_landing_vx_ref_apply(
    jump_landing_vx_ref_state_t *state, float command, float dt_s,
    float accel_mps2, uint8_t jump_owned_scope, uint8_t off_ground,
    float release_tolerance)
{
    if (state == NULL) return isfinite(command) ? command : 0.0f;
    if (!isfinite(command)) command = state->value;
    if (state->active && state->contact_confirmed &&
        state->contact_rebase_pending) {
        state->value = command;
        state->contact_rebase_pending = 0U;
    }
    if (state->active && state->contact_confirmed && !off_ground) {
        state->value = jump_landing_vx_ref_step(
            state->value, command, dt_s, accel_mps2);
    }
    if (state->active && state->contact_confirmed && !off_ground &&
        !jump_owned_scope && isfinite(release_tolerance) &&
        release_tolerance >= 0.0f &&
        fabsf(state->value - command) <= release_tolerance) {
        state->active = 0U;
        state->contact_confirmed = 0U;
    }
    return state->active ? state->value : command;
}

#endif /* CHASSIS_JUMP_LANDING_SPEED_H */
