#ifndef BUMP_CONTROL_HELPERS_H
#define BUMP_CONTROL_HELPERS_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

typedef enum {
    BUMP_STATE_IDLE = 0,
    BUMP_STATE_APPROACH,
    BUMP_STATE_CONTACT,
    BUMP_STATE_SWING_OVER_BODY,
    BUMP_STATE_RETRACT,
    BUMP_STATE_SWING_SUPPORT,
    BUMP_STATE_STAND,
    BUMP_STATE_DONE,
    BUMP_STATE_ABORT,
} bump_state_e;

typedef struct {
    uint8_t active;
    uint8_t taper_active;
    uint8_t handoff_active;
    float elapsed_s;
    float handoff_elapsed_s;
} bump_retract_breakaway_t;

static inline float bump_retract_breakaway_force_magnitude(
    float elapsed_s, float ramp_s, float peak_force)
{
    if (!isfinite(elapsed_s) || elapsed_s <= 0.0f ||
        !isfinite(ramp_s) || ramp_s <= 0.0f ||
        !isfinite(peak_force)) {
        return 0.0f;
    }
    if (elapsed_s >= ramp_s) return peak_force;
    float progress = elapsed_s / ramp_s;
    progress = progress * progress * (3.0f - 2.0f * progress);
    return peak_force * progress;
}

static inline float bump_retract_breakaway_taper_force(
    float length_m, float release_length_m, float release_band_m,
    float peak_force, float sustain_force)
{
    if (!isfinite(length_m) || !isfinite(release_length_m) ||
        !isfinite(release_band_m) || release_band_m <= 0.0f ||
        !isfinite(peak_force) || !isfinite(sustain_force)) {
        return 0.0f;
    }
    float progress = (release_length_m - length_m) / release_band_m;
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    progress = progress * progress * (3.0f - 2.0f * progress);
    return peak_force + progress * (sustain_force - peak_force);
}

static inline void bump_retract_breakaway_step(
    bump_retract_breakaway_t *state, uint8_t in_retract, uint8_t lockout,
    float dt_s, float length_m, float release_length_m,
    float release_band_m, float min_time_s, float handoff_time_s)
{
    if (state == NULL) return;
    if (!in_retract) {
        *state = (bump_retract_breakaway_t){0};
        return;
    }
    if (lockout || !isfinite(dt_s) || dt_s <= 0.0f) return;
    if (!state->active) return;

    if (!state->handoff_active) {
        state->elapsed_s += dt_s;
        if (!isfinite(state->elapsed_s)) state->elapsed_s = 0.0f;
        if (isfinite(length_m) && isfinite(release_length_m) &&
            length_m <= release_length_m) {
            state->taper_active = 1U;
        }
        const float release_threshold = release_length_m - release_band_m;
        if (isfinite(length_m) && isfinite(release_threshold) &&
            isfinite(min_time_s) &&
            state->elapsed_s >= min_time_s &&
            length_m <= release_threshold) {
            state->handoff_active = 1U;
            state->handoff_elapsed_s = 0.0f;
        }
        return;
    }

    state->handoff_elapsed_s += dt_s;
    if (isfinite(handoff_time_s) && handoff_time_s > 0.0f &&
        state->handoff_elapsed_s >= handoff_time_s) {
        state->active = 0U;
        state->handoff_active = 0U;
        state->handoff_elapsed_s = handoff_time_s;
    }
}

static inline float bump_retract_breakaway_handoff_force(
    float breakaway_force, float length_pid_force,
    float elapsed_s, float handoff_time_s)
{
    if (!isfinite(breakaway_force) || !isfinite(length_pid_force) ||
        !isfinite(elapsed_s) || !isfinite(handoff_time_s) ||
        handoff_time_s <= 0.0f) {
        return 0.0f;
    }
    float progress = elapsed_s / handoff_time_s;
    if (progress < 0.0f) progress = 0.0f;
    if (progress > 1.0f) progress = 1.0f;
    progress = progress * progress * (3.0f - 2.0f * progress);
    return breakaway_force + progress *
           (length_pid_force - breakaway_force);
}

static inline float bump_retract_reference_capture(
    float reference_m, float actual_m, float minimum_m)
{
    if (!isfinite(reference_m) || !isfinite(actual_m) ||
        !isfinite(minimum_m)) {
        return minimum_m;
    }
    float captured = reference_m < actual_m ? reference_m : actual_m;
    return captured < minimum_m ? minimum_m : captured;
}

static inline float bump_retract_one_way_force(
    float length_m, float force, float minimum_m)
{
    if (!isfinite(length_m) || !isfinite(force) || !isfinite(minimum_m)) {
        return 0.0f;
    }
    if (length_m <= minimum_m || force >= 0.0f) {
        return 0.0f;
    }
    return force;
}

static inline uint8_t bump_retract_lower_latch_update(
    uint8_t *latched, float length_m, float latch_length_m)
{
    if (latched == NULL) return 0U;
    if (!*latched && isfinite(length_m) && isfinite(latch_length_m) &&
        length_m <= latch_length_m) {
        *latched = 1U;
    }
    return *latched;
}

static inline float bump_retract_lower_latched_force(
    uint8_t lower_latched, float force)
{
    if (lower_latched || !isfinite(force)) return 0.0f;
    return force;
}

/* Once the measured leg crosses the lower latch, keep a small one-way
 * compressive force.  Clearing the force entirely lets the gas spring reopen
 * the leg before SWING_SUPPORT takes over.  Preserve any stronger negative
 * command already present, but never create an outward command at the latch. */
static inline float bump_retract_lower_latched_hold_force(
    uint8_t lower_latched, float force, float hold_force)
{
    if (!isfinite(force)) return 0.0f;
    if (!lower_latched) return force;
    if (!isfinite(hold_force) || hold_force <= 0.0f) return 0.0f;
    return force < -hold_force ? force : -hold_force;
}

/* RETRACT must keep driving through the spring dead point even while a leg
 * has not reached the lower latch yet.  This floor is one-way and is removed
 * at the physical lower bound. */
static inline float bump_retract_sustain_force_floor(
    float length_m, float force, float minimum_m, float floor_force)
{
    if (!isfinite(length_m) || !isfinite(force) ||
        !isfinite(minimum_m) || !isfinite(floor_force) || floor_force <= 0.0f) {
        return 0.0f;
    }
    if (length_m <= minimum_m) return 0.0f;
    return force < -floor_force ? force : -floor_force;
}

static inline bump_state_e bump_sequence_transition(
    bump_state_e state, uint8_t progress, uint8_t timed_out)
{
    switch (state) {
    case BUMP_STATE_APPROACH:
        return progress ? BUMP_STATE_CONTACT : state;
    case BUMP_STATE_CONTACT:
        return progress ? BUMP_STATE_SWING_OVER_BODY : state;
    case BUMP_STATE_SWING_OVER_BODY:
        if (progress) return BUMP_STATE_RETRACT;
        return timed_out ? BUMP_STATE_ABORT : state;
    case BUMP_STATE_RETRACT:
        /* The short-leg pose only prepares the chassis for the support
         * swing.  STAND owns the normal LOW LQR handoff after that swing
         * reaches its support angle. */
        if (progress) return BUMP_STATE_SWING_SUPPORT;
        return timed_out ? BUMP_STATE_ABORT : state;
    case BUMP_STATE_SWING_SUPPORT:
        if (progress) return BUMP_STATE_STAND;
        /* A slow swing on the step must hand off to the support sequence;
         * aborting here leaves the chassis coasting without leg support. */
        return timed_out ? BUMP_STATE_STAND : state;
    case BUMP_STATE_STAND:
        if (progress) return BUMP_STATE_DONE;
        return timed_out ? BUMP_STATE_ABORT : state;
    case BUMP_STATE_DONE:
        return progress ? BUMP_STATE_IDLE : state;
    case BUMP_STATE_ABORT:
        return timed_out ? BUMP_STATE_IDLE : state;
    case BUMP_STATE_IDLE:
    default:
        return state;
    }
}

static inline uint8_t bump_request_release_can_cancel(bump_state_e state)
{
    return (uint8_t)(state == BUMP_STATE_IDLE ||
                     state == BUMP_STATE_APPROACH ||
                     state == BUMP_STATE_DONE ||
                     state == BUMP_STATE_STAND);
}

static inline uint8_t bump_safety_takeover_allowed(uint8_t action_active,
                                                   uint8_t true_air_lockout)
{
    return (uint8_t)(!action_active || true_air_lockout);
}

static inline uint8_t bump_precontact_buffer_stale(uint8_t action_active,
                                                   uint8_t buffer_active,
                                                   uint8_t contact_confirmed)
{
    return (uint8_t)(action_active && buffer_active && !contact_confirmed);
}

static inline float bump_wrap_pi(float value)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;
    if (!isfinite(value)) return 0.0f;
    value = fmodf(value + pi, two_pi);
    if (value < 0.0f) value += two_pi;
    return value - pi;
}

static inline float bump_angle_error(float target, float actual)
{
    return bump_wrap_pi(target - actual);
}

static inline float bump_move_reference(float reference, float target,
                                        float rate, float dt_s)
{
    if (!isfinite(reference) || !isfinite(target) ||
        !isfinite(rate) || !isfinite(dt_s) || rate < 0.0f || dt_s <= 0.0f) {
        return reference;
    }
    const float step = rate * dt_s;
    const float error = bump_angle_error(target, reference);
    if (fabsf(error) <= step) return target;
    return reference + copysignf(step, error);
}

static inline uint8_t bump_angle_gate(float target, float actual,
                                      float tolerance)
{
    return (uint8_t)(isfinite(target) && isfinite(actual) &&
                     isfinite(tolerance) && tolerance >= 0.0f &&
                     fabsf(bump_angle_error(target, actual)) <= tolerance);
}

static inline uint8_t bump_pair_angle_gate(const float target[2],
                                           const float actual[2],
                                           float tolerance)
{
    return target != NULL && actual != NULL &&
           bump_angle_gate(target[0], actual[0], tolerance) &&
           bump_angle_gate(target[1], actual[1], tolerance);
}

static inline uint8_t bump_pair_length_gate(const float actual[2],
                                            const float target[2],
                                            float tolerance)
{
    return (uint8_t)(actual != NULL && target != NULL &&
                     isfinite(tolerance) && tolerance >= 0.0f &&
                     isfinite(actual[0]) && isfinite(actual[1]) &&
                     isfinite(target[0]) && isfinite(target[1]) &&
                     fabsf(actual[0] - target[0]) <= tolerance &&
                     fabsf(actual[1] - target[1]) <= tolerance);
}

/* Keep the approach speed evidence after the robot brakes against a step. */
static inline uint8_t bump_approach_speed_seen_step(
    uint8_t seen, uint8_t feedback_fresh, float vx_mps, float min_vx_mps)
{
    if (seen) return 1U;
    return (uint8_t)(feedback_fresh && isfinite(vx_mps) &&
                     isfinite(min_vx_mps) &&
                     vx_mps >= min_vx_mps);
}

static inline uint8_t bump_contact_gate(
    uint8_t approach_speed_seen, float wheel_speed_radps,
    float wheel_torque_nm, float theta_rad, float torque_threshold_nm,
    float theta_min_rad, float theta_max_rad, float wheel_speed_max_radps)
{
    return (uint8_t)(approach_speed_seen && isfinite(wheel_speed_radps) &&
                     isfinite(wheel_torque_nm) && isfinite(theta_rad) &&
                     isfinite(torque_threshold_nm) &&
                     isfinite(theta_min_rad) && isfinite(theta_max_rad) &&
                     isfinite(wheel_speed_max_radps) &&
                     wheel_speed_radps <= wheel_speed_max_radps &&
                     wheel_torque_nm >= torque_threshold_nm &&
                     fabsf(theta_rad) >= theta_min_rad &&
                     fabsf(theta_rad) <= theta_max_rad);
}

/* Prefer a two-sided contact, then accept one complete side after the
 * bounded pairing window.  This keeps a single-side impact from waiting
 * forever while retaining the existing 35 ms per-side debounce. */
static inline uint8_t bump_contact_pair_confirmed(
    uint8_t left_latched, uint8_t right_latched,
    float pair_elapsed_ms, float pair_window_ms)
{
    if (left_latched && right_latched) return 1U;
    return (uint8_t)((left_latched || right_latched) &&
                     isfinite(pair_elapsed_ms) &&
                     isfinite(pair_window_ms) && pair_window_ms > 0.0f &&
                     pair_elapsed_ms >= pair_window_ms);
}

/* A leg unload is not a whole-body flight verdict.  The caller supplies the
 * debounced off-ground detector and the filtered raw accelerometer norm. */
static inline uint8_t bump_true_airborne_step(uint8_t off_ground,
                                              float accel_norm_mps2,
                                              float low_threshold_mps2,
                                              float dt_s,
                                              float required_s,
                                              float *low_elapsed_s,
                                              uint8_t clear_latched,
                                              uint8_t *latched)
{
    if (low_elapsed_s == NULL || latched == NULL ||
        !isfinite(dt_s) || dt_s <= 0.0f || !isfinite(required_s) ||
        required_s <= 0.0f || !isfinite(low_threshold_mps2)) {
        return (latched != NULL) ? *latched : 0U;
    }
    if (!off_ground) {
        *low_elapsed_s = 0.0f;
        if (clear_latched) *latched = 0U;
        return *latched;
    }
    if (isfinite(accel_norm_mps2) && accel_norm_mps2 < low_threshold_mps2) {
        *low_elapsed_s += dt_s;
        if (*low_elapsed_s >= required_s) *latched = 1U;
    } else {
        *low_elapsed_s = 0.0f;
    }
    return *latched;
}

static inline uint8_t bump_true_airborne_step_rising(
    uint8_t off_ground, float accel_norm_mps2, float low_threshold_mps2,
    float dt_s, float required_s, float *low_elapsed_s,
    uint8_t clear_latched, uint8_t *latched)
{
    const uint8_t was_latched = latched != NULL ? *latched : 0U;
    (void)bump_true_airborne_step(
        off_ground, accel_norm_mps2, low_threshold_mps2, dt_s, required_s,
        low_elapsed_s, clear_latched, latched);
    return (uint8_t)(!was_latched && latched != NULL && *latched);
}

static inline float bump_finish_dwell_update(float elapsed_ms,
                                             uint8_t eligible,
                                             float dt_s,
                                             float required_ms)
{
    if (!eligible || !isfinite(dt_s) || dt_s <= 0.0f ||
        !isfinite(required_ms) || required_ms <= 0.0f) {
        return 0.0f;
    }
    if (!isfinite(elapsed_ms) || elapsed_ms < 0.0f) {
        elapsed_ms = 0.0f;
    }
    elapsed_ms += dt_s * 1000.0f;
    return elapsed_ms < required_ms ? elapsed_ms : required_ms;
}

static inline float bump_spring_cancel_force(float spring_force,
                                             float max_force)
{
    if (!isfinite(spring_force) || !isfinite(max_force) || max_force < 0.0f) {
        return 0.0f;
    }
    float cancel_force = -spring_force;
    if (cancel_force > max_force) cancel_force = max_force;
    if (cancel_force < -max_force) cancel_force = -max_force;
    return cancel_force;
}

static inline float bump_support_force_blend(float bump_force,
                                             float ordinary_force,
                                             float gain)
{
    if (!isfinite(bump_force) || !isfinite(ordinary_force) ||
        !isfinite(gain)) {
        return 0.0f;
    }
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.0f) gain = 1.0f;
    const float force = bump_force + gain * (ordinary_force - bump_force);
    return isfinite(force) ? force : 0.0f;
}

/* Keep the lower-stop compression command while STAND is still airborne or
 * before its support handoff has started.  Once both conditions are safe, the
 * caller can use support_gain to blend this command out smoothly. */
static inline uint8_t bump_stand_lower_hold_pending(
    uint8_t stand_state, uint8_t off_ground, uint8_t support_started,
    float support_gain)
{
    if (!stand_state) return 0U;
    if (off_ground || !support_started) return 1U;
    return (uint8_t)(!isfinite(support_gain) || support_gain < 0.99f);
}

/* Apply the lower-stop hold to the final STAND support command.  The hold is
 * one-way: a stronger negative command is preserved, while an outward or
 * weak command is clamped to the remaining compression floor.  The support
 * and extension gains both release the floor, because the off-ground
 * detector can remain asserted until the leg has already started extending. */
static inline float bump_stand_lower_hold_force(
    uint8_t stand_state, uint8_t lower_latched, uint8_t off_ground,
    float support_gain, float extend_gain, float force, float hold_force)
{
    if (!stand_state || !lower_latched || !isfinite(force) ||
        !isfinite(hold_force) || hold_force <= 0.0f) {
        return force;
    }
    float release_gain = isfinite(support_gain) ? support_gain : 0.0f;
    if (isfinite(extend_gain) && extend_gain > release_gain) {
        release_gain = extend_gain;
    }
    if (release_gain < 0.0f) release_gain = 0.0f;
    if (release_gain > 1.0f) release_gain = 1.0f;
    /* If the detector is still airborne and neither handoff gain is valid,
     * keep the full hold.  A valid extension gain is intentionally allowed
     * to release it so STAND can initiate a controlled leg extension. */
    if (off_ground && !isfinite(extend_gain) && !isfinite(support_gain)) {
        release_gain = 0.0f;
    }
    const float remaining_hold = hold_force * (1.0f - release_gain);
    if (remaining_hold <= 0.0f || force <= -remaining_hold) {
        return force;
    }
    return -remaining_hold;
}

/* While a lower-latched leg is still short, a positive support command can
 * launch the chassis before the step has taken the load.  Keep a bounded
 * positive ceiling until the leg has opened into the support geometry.  The
 * production limit is deliberately high enough to extend from the lower
 * stop, while the release length removes it before normal support control. */
static inline float bump_stand_short_leg_force_limit(
    uint8_t stand_state, uint8_t lower_latched, float length_m,
    float force, float release_length_m, float positive_limit)
{
    if (!stand_state || !lower_latched || !isfinite(length_m) ||
        !isfinite(force) || !isfinite(release_length_m) ||
        !isfinite(positive_limit) || release_length_m <= 0.0f ||
        positive_limit < 0.0f || length_m >= release_length_m ||
        force <= positive_limit) {
        return force;
    }
    return positive_limit;
}

static inline float bump_stand_length_target(float hold_length,
                                             float normal_target,
                                             float extend_gain)
{
    return bump_support_force_blend(hold_length, normal_target, extend_gain);
}

#endif
