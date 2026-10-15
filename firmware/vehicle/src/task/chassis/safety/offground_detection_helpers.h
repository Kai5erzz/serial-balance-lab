#ifndef OFFGROUND_DETECTION_HELPERS_H
#define OFFGROUND_DETECTION_HELPERS_H

#include <math.h>
#include <stdint.h>
#include <string.h>

#define OFFGROUND_KIN_RATE_WINDOW_SIZE       5U
#define OFFGROUND_KIN_RATE_MIN_SAMPLE_US   200U
#define OFFGROUND_KIN_RATE_MAX_SAMPLE_US 12000U
#define OFFGROUND_KIN_RATE_MIN_WINDOW_US   500U
#define OFFGROUND_KIN_RATE_MAX_WINDOW_US 50000U
#define OFFGROUND_KIN_RATE_MAX_AGE_US    12000U
#define OFFGROUND_KIN_RATE_FILTER_TAU_S   0.004f

#ifndef OFFGROUND_CONFIRM_DROPOUT_MS
#define OFFGROUND_CONFIRM_DROPOUT_MS       3.0f
#endif
#ifndef OFFGROUND_CONFIRM_WINDOW_MS
#define OFFGROUND_CONFIRM_WINDOW_MS       30.0f
#endif
#ifndef OFFGROUND_CONFIRM_MAX_SAMPLE_DT_MS
#define OFFGROUND_CONFIRM_MAX_SAMPLE_DT_MS 3.0f
#endif
#ifndef OFFGROUND_POST_LANDING_REARM_WINDOW_MS
#define OFFGROUND_POST_LANDING_REARM_WINDOW_MS 30.0f
#endif
#ifndef OFFGROUND_POST_LANDING_WEAK_CONFIRM_MS
#define OFFGROUND_POST_LANDING_WEAK_CONFIRM_MS 12.0f
#endif

/*
 * Normal (non-jump, non-BUMP) airborne legs occasionally leave the takeoff
 * geometry with one leg much longer than the other.  Keep this small controller
 * independent from the jump profile: it only produces a differential force
 * after a large split is observed and slews that force at the control rate.
 */
#ifndef OFFGROUND_NORMAL_SPLIT_ENABLE_M
#define OFFGROUND_NORMAL_SPLIT_ENABLE_M       0.050f
#endif
#ifndef OFFGROUND_NORMAL_SPLIT_RELEASE_M
#define OFFGROUND_NORMAL_SPLIT_RELEASE_M      0.035f
#endif
#ifndef OFFGROUND_NORMAL_SPLIT_KP
#define OFFGROUND_NORMAL_SPLIT_KP             520.0f
#endif
#ifndef OFFGROUND_NORMAL_SPLIT_KD
#define OFFGROUND_NORMAL_SPLIT_KD              14.0f
#endif
#ifndef OFFGROUND_NORMAL_SPLIT_FORCE_LIMIT
#define OFFGROUND_NORMAL_SPLIT_FORCE_LIMIT     32.0f
#endif
#ifndef OFFGROUND_NORMAL_SPLIT_FORCE_SLEW
#define OFFGROUND_NORMAL_SPLIT_FORCE_SLEW    900.0f
#endif

typedef struct {
    float common_reference;
    float differential_force;
    uint8_t reference_valid;
    uint8_t active;
} offground_normal_split_guard_t;

static inline void offground_normal_split_guard_reset(
    offground_normal_split_guard_t *guard)
{
    if (guard != NULL) {
        memset(guard, 0, sizeof(*guard));
    }
}

static inline void offground_normal_split_guard_begin(
    offground_normal_split_guard_t *guard, float left_length,
    float right_length)
{
    if (guard == NULL) {
        return;
    }
    const float common = 0.5f * (left_length + right_length);
    guard->common_reference = isfinite(common) ? common : 0.0f;
    guard->reference_valid = isfinite(common) ? 1U : 0U;
    guard->active = 0U;
    guard->differential_force = 0.0f;
}

static inline float offground_normal_split_guard_update(
    offground_normal_split_guard_t *guard, float left_length,
    float right_length, float left_rate, float right_rate, float dt_s)
{
    if (guard == NULL || !isfinite(left_length) || !isfinite(right_length) ||
        !isfinite(left_rate) || !isfinite(right_rate) ||
        !isfinite(dt_s) || dt_s <= 0.0f) {
        return 0.0f;
    }

    if (!guard->reference_valid) {
        offground_normal_split_guard_begin(
            guard, left_length, right_length);
    }

    const float split = right_length - left_length;
    const float split_abs = fabsf(split);
    if (!guard->active && split_abs > OFFGROUND_NORMAL_SPLIT_ENABLE_M) {
        guard->active = 1U;
    } else if (guard->active && split_abs <
               OFFGROUND_NORMAL_SPLIT_RELEASE_M) {
        guard->active = 0U;
    }

    float target = 0.0f;
    if (guard->active) {
        /* The common reference makes the correction explicitly differential:
         * left/right errors are equal and opposite around takeoff geometry. */
        const float left_error = guard->common_reference - left_length;
        const float right_error = guard->common_reference - right_length;
        target = 0.5f * (OFFGROUND_NORMAL_SPLIT_KP *
                         (left_error - right_error) +
                         OFFGROUND_NORMAL_SPLIT_KD *
                         (right_rate - left_rate));
        target = fminf(fmaxf(target, -OFFGROUND_NORMAL_SPLIT_FORCE_LIMIT),
                       OFFGROUND_NORMAL_SPLIT_FORCE_LIMIT);
    }

    const float max_step = OFFGROUND_NORMAL_SPLIT_FORCE_SLEW * dt_s;
    if (guard->differential_force < target - max_step) {
        guard->differential_force += max_step;
    } else if (guard->differential_force > target + max_step) {
        guard->differential_force -= max_step;
    } else {
        guard->differential_force = target;
    }
    guard->differential_force = fminf(
        fmaxf(guard->differential_force,
              -OFFGROUND_NORMAL_SPLIT_FORCE_LIMIT),
        OFFGROUND_NORMAL_SPLIT_FORCE_LIMIT);
    return guard->differential_force;
}

typedef struct {
    uint64_t time_us[OFFGROUND_KIN_RATE_WINDOW_SIZE];
    float length[2][OFFGROUND_KIN_RATE_WINDOW_SIZE];
    float theta_unwrapped[2][OFFGROUND_KIN_RATE_WINDOW_SIZE];
    float latest_length[2];
    float latest_theta[2];
    float length_rate[2];
    float theta_rate[2];
    float length_accel[2];
    float theta_accel[2];
    uint64_t latest_position_us;
    uint64_t latest_rate_us;
    uint32_t consumed_rx_count[4];
    uint8_t write_index;
    uint8_t sample_count;
    uint8_t rx_baseline_valid;
    uint8_t rate_filter_initialized;
} offground_kin_rate_observer_t;

typedef struct {
    float length_rate[2];
    float theta_rate[2];
    float length_accel[2];
    float theta_accel[2];
    float position_age_ms;
    float rate_age_ms;
    float position_sample_dt_ms;
    float rate_update_dt_ms;
    uint8_t rx_advanced_mask;
    uint8_t new_position_sample;
    uint8_t new_rate_sample;
    uint8_t valid;
} offground_kin_rate_view_t;

static inline uint8_t offground_legacy_scope_required(
    uint8_t jump_mode, uint8_t jump_origin_pending,
    uint8_t jump_origin_buffer, uint8_t jump_origin_postbuffer,
    uint8_t jump_origin_guard)
{
    return (uint8_t)(jump_mode || jump_origin_pending ||
                     jump_origin_buffer || jump_origin_postbuffer ||
                     jump_origin_guard);
}

static inline uint8_t offground_source_scope_transition(
    uint8_t *initialized, uint8_t *previous_legacy_scope,
    uint8_t required_legacy_scope)
{
    if (initialized == NULL || previous_legacy_scope == NULL) {
        return 0U;
    }
    required_legacy_scope = required_legacy_scope ? 1U : 0U;
    if (!*initialized || *previous_legacy_scope != required_legacy_scope) {
        *initialized = 1U;
        *previous_legacy_scope = required_legacy_scope;
        return 1U;
    }
    return 0U;
}

static inline float offground_kin_wrap_delta(float delta)
{
    const float two_pi = 6.28318530717958647692f;
    return isfinite(delta) ? remainderf(delta, two_pi) : NAN;
}

static inline void offground_kin_observer_clear_window(
    offground_kin_rate_observer_t *observer)
{
    observer->write_index = 0U;
    observer->sample_count = 0U;
    observer->rate_filter_initialized = 0U;
    observer->latest_position_us = 0U;
    observer->latest_rate_us = 0U;
    observer->rx_baseline_valid = 0U;
    memset(observer->time_us, 0, sizeof(observer->time_us));
    memset(observer->length_rate, 0, sizeof(observer->length_rate));
    memset(observer->theta_rate, 0, sizeof(observer->theta_rate));
    memset(observer->length_accel, 0, sizeof(observer->length_accel));
    memset(observer->theta_accel, 0, sizeof(observer->theta_accel));
}

static inline void offground_kin_observer_reset(
    offground_kin_rate_observer_t *observer)
{
    if (observer != NULL) {
        memset(observer, 0, sizeof(*observer));
    }
}

static inline void offground_kin_observer_seed(
    offground_kin_rate_observer_t *observer,
    const float length[2], const float theta[2],
    const uint32_t rx_count[4], uint64_t now_us)
{
    observer->write_index = 1U;
    observer->sample_count = 1U;
    observer->time_us[0] = now_us;
    observer->latest_position_us = now_us;
    for (uint8_t side = 0U; side < 2U; ++side) {
        observer->length[side][0] = length[side];
        observer->theta_unwrapped[side][0] = theta[side];
        observer->latest_length[side] = length[side];
        observer->latest_theta[side] = theta[side];
    }
    for (uint8_t i = 0U; i < 4U; ++i) {
        observer->consumed_rx_count[i] = rx_count[i];
    }
    observer->rx_baseline_valid = 1U;
}

static inline offground_kin_rate_view_t offground_kin_observer_view(
    const offground_kin_rate_observer_t *observer,
    uint64_t now_us, uint8_t rx_advanced_mask,
    uint8_t new_position_sample, uint8_t new_rate_sample,
    float position_sample_dt_ms, float rate_update_dt_ms)
{
    offground_kin_rate_view_t view = {0};
    view.position_age_ms = INFINITY;
    view.rate_age_ms = INFINITY;
    view.position_sample_dt_ms = position_sample_dt_ms;
    view.rate_update_dt_ms = rate_update_dt_ms;
    view.rx_advanced_mask = rx_advanced_mask;
    view.new_position_sample = new_position_sample;
    view.new_rate_sample = new_rate_sample;
    if (observer == NULL || !observer->rate_filter_initialized ||
        now_us < observer->latest_rate_us ||
        now_us - observer->latest_rate_us > OFFGROUND_KIN_RATE_MAX_AGE_US) {
        return view;
    }

    for (uint8_t side = 0U; side < 2U; ++side) {
        if (!isfinite(observer->length_rate[side]) ||
            !isfinite(observer->theta_rate[side]) ||
            !isfinite(observer->length_accel[side]) ||
            !isfinite(observer->theta_accel[side])) {
            return view;
        }
        view.length_rate[side] = observer->length_rate[side];
        view.theta_rate[side] = observer->theta_rate[side];
        view.length_accel[side] = observer->length_accel[side];
        view.theta_accel[side] = observer->theta_accel[side];
    }
    view.position_age_ms =
        (float)(now_us - observer->latest_position_us) * 0.001f;
    view.rate_age_ms =
        (float)(now_us - observer->latest_rate_us) * 0.001f;
    view.valid = 1U;
    return view;
}

static inline offground_kin_rate_view_t offground_kin_observer_update(
    offground_kin_rate_observer_t *observer,
    const float length[2], const float theta[2],
    const uint32_t rx_count[4], uint8_t feedback_fresh,
    uint64_t now_us)
{
    offground_kin_rate_view_t view = {0};
    uint8_t rx_mask = 0U;
    uint8_t all_rx_advanced = 0U;
    float position_dt_ms = 0.0f;
    float rate_dt_ms = 0.0f;

    if (observer == NULL || length == NULL || theta == NULL ||
        rx_count == NULL) {
        return view;
    }

    if (!feedback_fresh || !isfinite(length[0]) || !isfinite(length[1]) ||
        !isfinite(theta[0]) || !isfinite(theta[1])) {
        offground_kin_observer_clear_window(observer);
        return view;
    }

    if (observer->rx_baseline_valid &&
        (now_us < observer->latest_position_us ||
         now_us < observer->latest_rate_us)) {
        offground_kin_observer_clear_window(observer);
        offground_kin_observer_seed(observer, length, theta, rx_count, now_us);
        return view;
    }

    if (!observer->rx_baseline_valid) {
        offground_kin_observer_seed(observer, length, theta, rx_count, now_us);
        return view;
    }

    all_rx_advanced = 1U;
    for (uint8_t i = 0U; i < 4U; ++i) {
        if (rx_count[i] != observer->consumed_rx_count[i]) {
            rx_mask |= (uint8_t)(1U << i);
        } else {
            all_rx_advanced = 0U;
        }
    }

    if (!all_rx_advanced) {
        return offground_kin_observer_view(
            observer, now_us, rx_mask, 0U, 0U, 0.0f, 0.0f);
    }

    if (observer->sample_count == 0U ||
        now_us <= observer->latest_position_us) {
        offground_kin_observer_clear_window(observer);
        offground_kin_observer_seed(observer, length, theta, rx_count, now_us);
        return view;
    }

    const uint64_t sample_dt_us = now_us - observer->latest_position_us;
    position_dt_ms = (float)sample_dt_us * 0.001f;
    for (uint8_t i = 0U; i < 4U; ++i) {
        observer->consumed_rx_count[i] = rx_count[i];
    }
    if (sample_dt_us < OFFGROUND_KIN_RATE_MIN_SAMPLE_US ||
        sample_dt_us > OFFGROUND_KIN_RATE_MAX_SAMPLE_US) {
        offground_kin_observer_clear_window(observer);
        offground_kin_observer_seed(observer, length, theta, rx_count, now_us);
        return view;
    }

    const uint8_t write = observer->write_index;
    uint64_t oldest_time_us = now_us;
    float theta_unwrapped[2];
    for (uint8_t side = 0U; side < 2U; ++side) {
        theta_unwrapped[side] = observer->latest_theta[side] +
            offground_kin_wrap_delta(theta[side] -
                                     observer->latest_theta[side]);
        if (!isfinite(theta_unwrapped[side])) {
            offground_kin_observer_clear_window(observer);
            offground_kin_observer_seed(observer, length, theta, rx_count,
                                        now_us);
            return view;
        }
        observer->length[side][write] = length[side];
        observer->theta_unwrapped[side][write] = theta_unwrapped[side];
    }
    observer->time_us[write] = now_us;
    observer->write_index =
        (uint8_t)((write + 1U) % OFFGROUND_KIN_RATE_WINDOW_SIZE);
    if (observer->sample_count < OFFGROUND_KIN_RATE_WINDOW_SIZE) {
        observer->sample_count++;
    }
    observer->latest_position_us = now_us;
    for (uint8_t side = 0U; side < 2U; ++side) {
        observer->latest_length[side] = length[side];
        observer->latest_theta[side] = theta_unwrapped[side];
    }

    if (observer->sample_count < OFFGROUND_KIN_RATE_WINDOW_SIZE) {
        return view;
    }

    const uint8_t oldest = observer->write_index;
    oldest_time_us = observer->time_us[oldest];
    if (now_us <= oldest_time_us) {
        offground_kin_observer_clear_window(observer);
        offground_kin_observer_seed(observer, length, theta, rx_count, now_us);
        return view;
    }
    const uint64_t window_us = now_us - oldest_time_us;
    if (window_us < OFFGROUND_KIN_RATE_MIN_WINDOW_US ||
        window_us > OFFGROUND_KIN_RATE_MAX_WINDOW_US) {
        offground_kin_observer_clear_window(observer);
        offground_kin_observer_seed(observer, length, theta, rx_count, now_us);
        return view;
    }

    const float window_s = (float)window_us * 0.000001f;
    const float sample_dt_s = (float)sample_dt_us * 0.000001f;
    const float alpha = OFFGROUND_KIN_RATE_FILTER_TAU_S /
        (OFFGROUND_KIN_RATE_FILTER_TAU_S + sample_dt_s);
    float length_rate_next[2];
    float theta_rate_next[2];
    float length_accel_next[2];
    float theta_accel_next[2];
    for (uint8_t side = 0U; side < 2U; ++side) {
        const float length_rate_raw =
            (observer->length[side][write] -
             observer->length[side][oldest]) / window_s;
        const float theta_rate_raw =
            (observer->theta_unwrapped[side][write] -
             observer->theta_unwrapped[side][oldest]) / window_s;
        if (!isfinite(length_rate_raw) || !isfinite(theta_rate_raw)) {
            offground_kin_observer_clear_window(observer);
            offground_kin_observer_seed(observer, length, theta, rx_count,
                                        now_us);
            return view;
        }

        if (!observer->rate_filter_initialized) {
            length_rate_next[side] = length_rate_raw;
            theta_rate_next[side] = theta_rate_raw;
            length_accel_next[side] = 0.0f;
            theta_accel_next[side] = 0.0f;
        } else {
            const float previous_length_rate = observer->length_rate[side];
            const float previous_theta_rate = observer->theta_rate[side];
            const float length_rate = alpha * previous_length_rate +
                (1.0f - alpha) * length_rate_raw;
            const float theta_rate = alpha * previous_theta_rate +
                (1.0f - alpha) * theta_rate_raw;
            const float length_accel_raw =
                (length_rate - previous_length_rate) / sample_dt_s;
            const float theta_accel_raw =
                (theta_rate - previous_theta_rate) / sample_dt_s;
            const float length_accel = alpha *
                observer->length_accel[side] +
                (1.0f - alpha) * length_accel_raw;
            const float theta_accel = alpha *
                observer->theta_accel[side] +
                (1.0f - alpha) * theta_accel_raw;
            length_rate_next[side] = length_rate;
            theta_rate_next[side] = theta_rate;
            length_accel_next[side] = length_accel;
            theta_accel_next[side] = theta_accel;
        }
        if (!isfinite(length_rate_next[side]) ||
            !isfinite(theta_rate_next[side]) ||
            !isfinite(length_accel_next[side]) ||
            !isfinite(theta_accel_next[side])) {
            offground_kin_observer_clear_window(observer);
            offground_kin_observer_seed(observer, length, theta, rx_count,
                                        now_us);
            return view;
        }
    }
    for (uint8_t side = 0U; side < 2U; ++side) {
        observer->length_rate[side] = length_rate_next[side];
        observer->theta_rate[side] = theta_rate_next[side];
        observer->length_accel[side] = length_accel_next[side];
        observer->theta_accel[side] = theta_accel_next[side];
    }
    observer->rate_filter_initialized = 1U;
    const uint64_t previous_rate_us = observer->latest_rate_us;
    observer->latest_rate_us = now_us;
    if (previous_rate_us > 0U && now_us > previous_rate_us) {
        rate_dt_ms = (float)(now_us - previous_rate_us) * 0.001f;
    } else {
        rate_dt_ms = position_dt_ms;
    }
    return offground_kin_observer_view(
        observer, now_us, rx_mask, 1U, 1U, position_dt_ms, rate_dt_ms);
}

typedef struct {
    float takeoff_progress_ms;
    float takeoff_weak_progress_ms;
    float takeoff_strong_progress_ms;
    float landing_progress_ms;
    float landing_candidate_ms;
    float landing_dropout_ms;
    float post_landing_ms;
    uint8_t takeoff_candidate_kind;
    uint8_t landing_candidate_active;
    uint8_t post_landing_active;
} offground_confirmation_state_t;

static inline void offground_confirmation_reset(
    offground_confirmation_state_t *state)
{
    if (state != NULL) {
        memset(state, 0, sizeof(*state));
    }
}

static inline void offground_confirmation_landing(
    offground_confirmation_state_t *state)
{
    if (state == NULL) return;
    state->takeoff_progress_ms = 0.0f;
    state->takeoff_weak_progress_ms = 0.0f;
    state->takeoff_strong_progress_ms = 0.0f;
    state->takeoff_candidate_kind = 0U;
    state->landing_progress_ms = 0.0f;
    state->landing_candidate_ms = 0.0f;
    state->landing_dropout_ms = 0.0f;
    state->landing_candidate_active = 0U;
    state->post_landing_ms = OFFGROUND_POST_LANDING_REARM_WINDOW_MS;
    state->post_landing_active = 1U;
}

static inline void offground_confirmation_advance_post_landing(
    offground_confirmation_state_t *state, float dt_ms)
{
    if (state == NULL || !state->post_landing_active ||
        !isfinite(dt_ms) || dt_ms <= 0.0f) {
        return;
    }
    state->post_landing_ms -= dt_ms;
    if (state->post_landing_ms <= 0.0f) {
        state->post_landing_ms = 0.0f;
        state->post_landing_active = 0U;
        state->takeoff_progress_ms = 0.0f;
        state->takeoff_weak_progress_ms = 0.0f;
        state->takeoff_strong_progress_ms = 0.0f;
        state->takeoff_candidate_kind = 0U;
    }
}

static inline void offground_confirmation_step_takeoff(
    offground_confirmation_state_t *state,
    uint8_t valid, uint8_t candidate, uint8_t strong_candidate,
    float dt_ms, float base_ms, float weak_post_landing_ms,
    uint8_t *confirmed)
{
    uint8_t post_landing;
    if (confirmed != NULL) *confirmed = 0U;
    if (state == NULL || confirmed == NULL || !isfinite(dt_ms) ||
        dt_ms <= 0.0f || !isfinite(base_ms) || base_ms <= 0.0f) {
        return;
    }

    post_landing = state->post_landing_active;

    if (dt_ms > OFFGROUND_CONFIRM_MAX_SAMPLE_DT_MS) {
        state->takeoff_progress_ms = 0.0f;
        state->takeoff_weak_progress_ms = 0.0f;
        state->takeoff_strong_progress_ms = 0.0f;
        state->takeoff_candidate_kind = 0U;
        return;
    }

    if (!valid || !candidate) {
        state->takeoff_progress_ms = 0.0f;
        state->takeoff_weak_progress_ms = 0.0f;
        state->takeoff_strong_progress_ms = 0.0f;
        state->takeoff_candidate_kind = 0U;
        return;
    }

    if (post_landing) {
        if (state->post_landing_ms <= 0.0f) {
            state->post_landing_active = 0U;
            post_landing = 0U;
        }
    }

    const uint8_t candidate_kind = post_landing ?
        (strong_candidate ? 2U : 1U) : 3U;
    if (state->takeoff_candidate_kind != candidate_kind) {
        state->takeoff_candidate_kind = candidate_kind;
        state->takeoff_progress_ms = 0.0f;
        state->takeoff_weak_progress_ms = 0.0f;
        state->takeoff_strong_progress_ms = 0.0f;
        return;
    }

    if (candidate_kind == 2U) {
        state->takeoff_strong_progress_ms += dt_ms;
        state->takeoff_progress_ms = state->takeoff_strong_progress_ms;
        if (state->takeoff_strong_progress_ms >= base_ms) {
            *confirmed = 1U;
        }
    } else if (candidate_kind == 1U) {
        const float required_ms =
            (base_ms > weak_post_landing_ms) ?
            base_ms : weak_post_landing_ms;
        state->takeoff_weak_progress_ms += dt_ms;
        state->takeoff_progress_ms = state->takeoff_weak_progress_ms;
        if (state->takeoff_weak_progress_ms >= required_ms) {
            *confirmed = 1U;
        }
    } else {
        state->takeoff_progress_ms += dt_ms;
        if (state->takeoff_progress_ms >= base_ms) {
            *confirmed = 1U;
        }
    }

    if (*confirmed) {
        state->takeoff_progress_ms = 0.0f;
        state->takeoff_weak_progress_ms = 0.0f;
        state->takeoff_strong_progress_ms = 0.0f;
        state->takeoff_candidate_kind = 0U;
    }
}

static inline void offground_confirmation_step_landing(
    offground_confirmation_state_t *state,
    uint8_t valid, uint8_t entry_condition, uint8_t hold_condition,
    float dt_ms, float required_ms,
    uint8_t *confirmed)
{
    if (confirmed != NULL) *confirmed = 0U;
    if (state == NULL || confirmed == NULL || !isfinite(dt_ms) ||
        dt_ms <= 0.0f || !isfinite(required_ms) || required_ms <= 0.0f) {
        return;
    }

    if (dt_ms > OFFGROUND_CONFIRM_MAX_SAMPLE_DT_MS) {
        state->landing_progress_ms = 0.0f;
        state->landing_candidate_ms = 0.0f;
        state->landing_dropout_ms = 0.0f;
        state->landing_candidate_active = 0U;
        return;
    }

    if (!state->landing_candidate_active) {
        if (valid && entry_condition) {
            state->landing_candidate_active = 1U;
            state->landing_candidate_ms = 0.0f;
            state->landing_progress_ms = 0.0f;
            state->landing_dropout_ms = 0.0f;
        }
        return;
    }

    if (state->landing_candidate_ms + dt_ms >
        OFFGROUND_CONFIRM_WINDOW_MS) {
        state->landing_progress_ms = 0.0f;
        state->landing_candidate_ms = 0.0f;
        state->landing_dropout_ms = 0.0f;
        state->landing_candidate_active = 0U;
        if (valid && entry_condition) {
            state->landing_candidate_active = 1U;
        }
        return;
    }

    state->landing_candidate_ms += dt_ms;
    if (valid && hold_condition) {
        state->landing_dropout_ms = 0.0f;
        state->landing_progress_ms += dt_ms;
        if (state->landing_progress_ms >= required_ms) {
            *confirmed = 1U;
            state->landing_progress_ms = 0.0f;
            state->landing_candidate_ms = 0.0f;
            state->landing_dropout_ms = 0.0f;
            state->landing_candidate_active = 0U;
            return;
        }
    } else {
        state->landing_dropout_ms += dt_ms;
    }

    if (state->landing_candidate_active &&
        (state->landing_dropout_ms > OFFGROUND_CONFIRM_DROPOUT_MS ||
         state->landing_candidate_ms > OFFGROUND_CONFIRM_WINDOW_MS)) {
        state->landing_progress_ms = 0.0f;
        state->landing_candidate_ms = 0.0f;
        state->landing_dropout_ms = 0.0f;
        state->landing_candidate_active = 0U;
    }
}

#endif
