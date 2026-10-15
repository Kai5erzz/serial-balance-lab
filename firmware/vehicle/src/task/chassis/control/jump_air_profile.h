#ifndef JUMP_AIR_PROFILE_H
#define JUMP_AIR_PROFILE_H

#include <stddef.h>
#include <stdint.h>
#include <math.h>

typedef struct {
    float actual_compact_hold_s;
    uint8_t actual_compact;
    uint8_t deploy_latched;
} jump_air_profile_state_t;

/* Reset only when a new flight profile starts or contact cancels the profile. */
static inline void jump_air_profile_reset(jump_air_profile_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->actual_compact_hold_s = 0.0f;
    state->actual_compact = 0U;
    state->deploy_latched = 0U;
}

/* A delayed takeoff edge inherits a release-prestarted profile. Other starts
 * initialize a clean compact dwell and deployment latch. */
static inline void jump_air_profile_begin(
    jump_air_profile_state_t *state, uint8_t preserve_prestarted)
{
    if (!preserve_prestarted) {
        jump_air_profile_reset(state);
    }
}

static inline uint8_t jump_air_profile_compact_phase_active(
    const jump_air_profile_state_t *state, uint8_t active_jump,
    uint8_t contact_inhibit)
{
    return (uint8_t)(state != NULL && active_jump && !contact_inhibit &&
                     !state->deploy_latched);
}

/* The compact dwell counts only consecutive samples where both measured legs
 * are compact and both references have reached the tuck target. The first
 * qualifying sample starts the dwell at zero; a broken/invalid sample resets
 * it. Deployment is latched after the dwell or at the hard elapsed-time cap. */
static inline uint8_t jump_air_profile_actual_compact_phase(
    jump_air_profile_state_t *state, float elapsed_s, float dt_s,
    uint8_t measurements_fresh,
    float actual_left, float actual_right,
    float ref_left, float ref_right,
    float actual_compact_max, float tuck_ref, float ref_tolerance,
    float actual_hold_s, float hard_max_elapsed_s)
{
    if (state == NULL) {
        return 0U;
    }

    const uint8_t valid = (uint8_t)(
        isfinite(elapsed_s) && elapsed_s >= 0.0f &&
        isfinite(dt_s) && dt_s >= 0.0f && dt_s <= 0.05f &&
        measurements_fresh &&
        isfinite(actual_left) && actual_left > 0.05f &&
        isfinite(actual_right) && actual_right > 0.05f &&
        isfinite(ref_left) && ref_left > 0.05f &&
        isfinite(ref_right) && ref_right > 0.05f &&
        isfinite(actual_compact_max) && actual_compact_max > 0.05f &&
        actual_compact_max <= 0.50f &&
        isfinite(tuck_ref) && tuck_ref > 0.05f &&
        tuck_ref <= actual_compact_max &&
        isfinite(ref_tolerance) && ref_tolerance >= 0.0f &&
        ref_tolerance <= 0.01f &&
        isfinite(actual_hold_s) && actual_hold_s > 0.0f &&
        isfinite(hard_max_elapsed_s) && hard_max_elapsed_s > 0.0f &&
        hard_max_elapsed_s <= 1.0f &&
        actual_hold_s <= hard_max_elapsed_s);
    const uint8_t compact_now = (uint8_t)(valid &&
        actual_left <= actual_compact_max &&
        actual_right <= actual_compact_max &&
        fabsf(ref_left - tuck_ref) <= ref_tolerance &&
        fabsf(ref_right - tuck_ref) <= ref_tolerance);

    if (state->deploy_latched) {
        state->actual_compact = compact_now;
        return 0U;
    }

    if (compact_now) {
        if (state->actual_compact) {
            if (dt_s > 0.0f) {
                state->actual_compact_hold_s += dt_s;
            }
        } else {
            state->actual_compact_hold_s = 0.0f;
        }
    } else {
        state->actual_compact_hold_s = 0.0f;
    }
    state->actual_compact = compact_now;

    if ((compact_now &&
         state->actual_compact_hold_s >= actual_hold_s - 0.000001f) ||
        (isfinite(elapsed_s) && elapsed_s >= 0.0f &&
         isfinite(hard_max_elapsed_s) && hard_max_elapsed_s > 0.0f &&
         elapsed_s >= hard_max_elapsed_s)) {
        state->deploy_latched = 1U;
    }
    return (uint8_t)!state->deploy_latched;
}

/* Legacy time-based compact estimate retained for existing diagnostics. The
 * safety length profile uses the stateful actual-compact dwell above; descent
 * no longer shortens that measured dwell. */
static inline uint8_t jump_air_profile_compact_phase(
    float elapsed_s, uint8_t descent,
    float tuck_time_s, float tuck_hold_time_s,
    float tuck_min_hold_time_s)
{
    if (tuck_min_hold_time_s < 0.0f) {
        tuck_min_hold_time_s = 0.0f;
    }
    if (tuck_min_hold_time_s > tuck_hold_time_s) {
        tuck_min_hold_time_s = tuck_hold_time_s;
    }
    const uint8_t deploy_started =
        elapsed_s >= (tuck_time_s + tuck_hold_time_s) ||
        (descent &&
         elapsed_s >= (tuck_time_s + tuck_min_hold_time_s));
    return (uint8_t)!deploy_started;
}

#endif /* JUMP_AIR_PROFILE_H */
