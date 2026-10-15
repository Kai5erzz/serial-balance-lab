#ifndef JUMP_KINEMATIC_RATE_OBSERVER_H
#define JUMP_KINEMATIC_RATE_OBSERVER_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define JUMP_KIN_RATE_WINDOW_SIZE       5U
#define JUMP_KIN_RATE_LPF_TAU_S         0.004f
#define JUMP_KIN_RATE_MIN_SAMPLE_DT_MS  0.2f
#define JUMP_KIN_RATE_MAX_SAMPLE_DT_MS 20.0f
#define JUMP_KIN_RATE_MIN_WINDOW_MS     0.5f
#define JUMP_KIN_RATE_MAX_WINDOW_MS    50.0f
#define JUMP_KIN_RATE_FRESH_MAX_AGE_MS  2.5f
#define JUMP_KIN_RATE_HOLD_MAX_AGE_MS  10.0f

#ifndef JUMP_KINEMATIC_RATE_OBSERVER_ENABLE
#define JUMP_KINEMATIC_RATE_OBSERVER_ENABLE 1
#endif

typedef enum {
    JUMP_KIN_RATE_UPDATE_INVALID_GEOMETRY = -1,
    JUMP_KIN_RATE_UPDATE_RESEEDED = 0,
    JUMP_KIN_RATE_UPDATE_DUPLICATE = 1,
    JUMP_KIN_RATE_UPDATE_WARMING = 2,
    JUMP_KIN_RATE_UPDATE_READY = 3,
} jump_kin_rate_update_result_t;

typedef struct {
    float time_ms[JUMP_KIN_RATE_WINDOW_SIZE];
    float theta_unwrapped[JUMP_KIN_RATE_WINDOW_SIZE];
    float l0[JUMP_KIN_RATE_WINDOW_SIZE];
    float latest_theta;
    float latest_unwrapped_theta;
    float latest_l0;
    float latest_sample_ms;
    float last_valid_ms;
    float theta_rate;
    float l0_rate;
    uint8_t write_index;
    uint8_t sample_count;
    uint8_t ready;
    uint8_t rate_filter_initialized;
    uint8_t last_valid;
} jump_kin_rate_observer_t;

typedef struct {
    float theta_rate;
    float l0_rate;
    float sample_age_ms;
    float valid_rate_age_ms;
    uint8_t fresh;
    uint8_t usable;
} jump_kin_rate_view_t;

static inline uint8_t jump_kin_rate_pair_fresh(
    const jump_kin_rate_view_t *left,
    const jump_kin_rate_view_t *right)
{
    return (uint8_t)(left != NULL && right != NULL &&
                     left->fresh && right->fresh);
}

static inline uint8_t jump_kin_rate_pair_usable(
    const jump_kin_rate_view_t *left,
    const jump_kin_rate_view_t *right)
{
    return (uint8_t)(left != NULL && right != NULL &&
                     left->usable && right->usable);
}

static inline float jump_kin_rate_wrap_delta(float delta)
{
    const float pi = 3.14159265358979323846f;
    const float two_pi = 6.28318530717958647692f;
    while (delta > pi) {
        delta -= two_pi;
    }
    while (delta < -pi) {
        delta += two_pi;
    }
    return delta;
}

static inline void jump_kin_rate_observer_clear_window(
    jump_kin_rate_observer_t *observer)
{
    observer->write_index = 0U;
    observer->sample_count = 0U;
    observer->ready = 0U;
    observer->rate_filter_initialized = 0U;
}

static inline void jump_kin_rate_observer_seed(
    jump_kin_rate_observer_t *observer,
    float theta,
    float l0,
    float now_ms)
{
    jump_kin_rate_observer_clear_window(observer);
    observer->time_ms[0] = now_ms;
    observer->theta_unwrapped[0] = theta;
    observer->l0[0] = l0;
    observer->latest_theta = theta;
    observer->latest_unwrapped_theta = theta;
    observer->latest_l0 = l0;
    observer->latest_sample_ms = now_ms;
    observer->write_index = 1U;
    observer->sample_count = 1U;
}

static inline jump_kin_rate_update_result_t jump_kin_rate_observer_update(
    jump_kin_rate_observer_t *observer,
    float theta,
    float l0,
    float now_ms)
{
    if (observer == NULL || !isfinite(theta) || !isfinite(l0) ||
        !isfinite(now_ms)) {
        if (observer != NULL) {
            jump_kin_rate_observer_clear_window(observer);
        }
        return JUMP_KIN_RATE_UPDATE_INVALID_GEOMETRY;
    }

    if (observer->sample_count == 0U) {
        jump_kin_rate_observer_seed(observer, theta, l0, now_ms);
        return JUMP_KIN_RATE_UPDATE_RESEEDED;
    }

    const float sample_dt_ms = now_ms - observer->latest_sample_ms;
    if (sample_dt_ms == 0.0f) {
        /* A repeated DWT timestamp is not another kinematic measurement. */
        return JUMP_KIN_RATE_UPDATE_DUPLICATE;
    }
    if (!isfinite(sample_dt_ms) || sample_dt_ms < 0.0f ||
        sample_dt_ms < JUMP_KIN_RATE_MIN_SAMPLE_DT_MS ||
        sample_dt_ms > JUMP_KIN_RATE_MAX_SAMPLE_DT_MS) {
        jump_kin_rate_observer_seed(observer, theta, l0, now_ms);
        return JUMP_KIN_RATE_UPDATE_RESEEDED;
    }

    const float unwrapped_theta = observer->latest_unwrapped_theta +
        jump_kin_rate_wrap_delta(theta - observer->latest_theta);

    observer->time_ms[observer->write_index] = now_ms;
    observer->theta_unwrapped[observer->write_index] = unwrapped_theta;
    observer->l0[observer->write_index] = l0;
    observer->write_index = (uint8_t)(
        (observer->write_index + 1U) % JUMP_KIN_RATE_WINDOW_SIZE);
    if (observer->sample_count < JUMP_KIN_RATE_WINDOW_SIZE) {
        observer->sample_count++;
    }
    observer->latest_theta = theta;
    observer->latest_unwrapped_theta = unwrapped_theta;
    observer->latest_l0 = l0;
    observer->latest_sample_ms = now_ms;

    if (observer->sample_count < JUMP_KIN_RATE_WINDOW_SIZE) {
        observer->ready = 0U;
        return JUMP_KIN_RATE_UPDATE_WARMING;
    }

    const float window_ms = now_ms - observer->time_ms[observer->write_index];
    if (!isfinite(window_ms) ||
        window_ms < JUMP_KIN_RATE_MIN_WINDOW_MS ||
        window_ms > JUMP_KIN_RATE_MAX_WINDOW_MS) {
        jump_kin_rate_observer_seed(observer, theta, l0, now_ms);
        return JUMP_KIN_RATE_UPDATE_RESEEDED;
    }

    const float window_s = window_ms * 0.001f;
    const float theta_rate_raw =
        (unwrapped_theta - observer->theta_unwrapped[observer->write_index]) /
        window_s;
    const float l0_rate_raw =
        (l0 - observer->l0[observer->write_index]) / window_s;
    if (!isfinite(theta_rate_raw) || !isfinite(l0_rate_raw)) {
        jump_kin_rate_observer_seed(observer, theta, l0, now_ms);
        return JUMP_KIN_RATE_UPDATE_RESEEDED;
    }

    if (!observer->rate_filter_initialized) {
        /* Do not start with a zero-rate transient once the window is ready. */
        observer->theta_rate = theta_rate_raw;
        observer->l0_rate = l0_rate_raw;
        observer->rate_filter_initialized = 1U;
        observer->last_valid = 1U;
    } else {
        const float dt_s = sample_dt_ms * 0.001f;
        const float alpha = JUMP_KIN_RATE_LPF_TAU_S /
            (JUMP_KIN_RATE_LPF_TAU_S + dt_s);
        observer->theta_rate = alpha * observer->theta_rate +
            (1.0f - alpha) * theta_rate_raw;
        observer->l0_rate = alpha * observer->l0_rate +
            (1.0f - alpha) * l0_rate_raw;
    }
    observer->last_valid_ms = now_ms;
    observer->ready = 1U;
    return JUMP_KIN_RATE_UPDATE_READY;
}

static inline jump_kin_rate_view_t jump_kin_rate_observer_view(
    const jump_kin_rate_observer_t *observer,
    float now_ms)
{
    jump_kin_rate_view_t view = {0};
    view.sample_age_ms = INFINITY;
    view.valid_rate_age_ms = INFINITY;
    if (observer == NULL || !isfinite(now_ms)) {
        return view;
    }

    if (observer->sample_count > 0U) {
        view.sample_age_ms = now_ms - observer->latest_sample_ms;
        if (!isfinite(view.sample_age_ms) || view.sample_age_ms < 0.0f) {
            view.sample_age_ms = INFINITY;
        }
    }
    if (observer->last_valid) {
        view.valid_rate_age_ms = now_ms - observer->last_valid_ms;
        if (!isfinite(view.valid_rate_age_ms) ||
            view.valid_rate_age_ms < 0.0f) {
            view.valid_rate_age_ms = INFINITY;
        }
        view.theta_rate = observer->theta_rate;
        view.l0_rate = observer->l0_rate;
        view.usable = (uint8_t)(
            isfinite(view.theta_rate) && isfinite(view.l0_rate) &&
            view.valid_rate_age_ms <= JUMP_KIN_RATE_HOLD_MAX_AGE_MS);
        view.fresh = (uint8_t)(observer->ready &&
            view.sample_age_ms <= JUMP_KIN_RATE_FRESH_MAX_AGE_MS &&
            view.valid_rate_age_ms <= JUMP_KIN_RATE_FRESH_MAX_AGE_MS);
    }
    return view;
}

#endif /* JUMP_KINEMATIC_RATE_OBSERVER_H */
