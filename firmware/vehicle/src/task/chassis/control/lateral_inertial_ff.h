#ifndef LATERAL_INERTIAL_FF_H
#define LATERAL_INERTIAL_FF_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

#define LATERAL_FF_COAST_MAX_S          0.500f
#define LATERAL_FF_COAST_FADE_S         0.100f
#define LATERAL_FF_COAST_GAP_ON         0.20f
#define LATERAL_FF_COAST_GAP_OFF        0.05f
#define LATERAL_FF_REF_DEADBAND         0.20f
#define LATERAL_FF_REF_REARM_DEADBAND   0.10f
#define LATERAL_FF_VX_MIN_MPS           0.12f
#define LATERAL_FF_VX_MAX_MPS           3.50f
#define LATERAL_FF_YAW_MIN_RADPS        0.15f
#define LATERAL_FF_YAW_MAX_RADPS       12.00f
#define LATERAL_FF_REF_DELTA_EPS        0.00001f
#define LATERAL_FF_WHEEL_MAX_AGE_MS        20U
#define LATERAL_FF_HIP_MAX_AGE_MS          30U
#define LATERAL_FF_INS_MAX_AGE_MS          10U

typedef enum {
    LATERAL_FF_LOCK_REASON_NONE = 0U,
    LATERAL_FF_LOCK_REASON_DT = 1U,
    LATERAL_FF_LOCK_REASON_TIMEOUT = 2U,
    LATERAL_FF_LOCK_REASON_FEEDBACK_INVALID = 3U,
    LATERAL_FF_LOCK_REASON_OUT_OF_RANGE = 4U,
    LATERAL_FF_LOCK_REASON_ACTUAL_REVERSE = 5U
} lateral_inertial_ff_lock_reason_t;

static inline uint8_t lateral_inertial_ff_sample_fresh(
    uint32_t now_ms, uint32_t last_rx_ms, uint32_t rx_count,
    uint32_t max_age_ms)
{
    if (rx_count == 0U) {
        return 0U;
    }
    const uint32_t age_ms = now_ms - last_rx_ms;
    return (uint8_t)(age_ms <= max_age_ms || age_ms == UINT32_MAX);
}

static inline float lateral_inertial_ff_sample_age_ms(
    uint32_t now_ms, uint32_t last_rx_ms, uint32_t rx_count)
{
    if (rx_count == 0U) {
        return -1.0f;
    }
    const uint32_t age_ms = now_ms - last_rx_ms;
    return age_ms == UINT32_MAX ? 0.0f : (float)age_ms;
}

typedef struct {
    float coast_elapsed_s;
    float last_ref_accel;
    float ref_accel_peak;
    float last_ref_vx;
    float last_ref_yaw_rate;
    float reference_accel;
    float actual_accel;
    float selected_accel;
    int8_t ref_vx_sign;
    int8_t ref_yaw_sign;
    uint8_t ref_active;
    uint8_t ref_below_deadband;
    uint8_t ref_decelerating;
    uint8_t coast_started;
    uint8_t coast_active;
    uint8_t coast_locked;
    lateral_inertial_ff_lock_reason_t coast_lock_reason;
    uint8_t feedback_valid;
} lateral_inertial_ff_state_t;

static inline void lateral_inertial_ff_lock(
    lateral_inertial_ff_state_t *state,
    lateral_inertial_ff_lock_reason_t reason)
{
    if (state == NULL) {
        return;
    }
    if (!state->coast_locked) {
        state->coast_locked = 1U;
        state->coast_lock_reason = reason;
    }
    state->coast_active = 0U;
}

static inline void lateral_inertial_ff_reset(
    lateral_inertial_ff_state_t *state)
{
    if (state == NULL) {
        return;
    }
    state->coast_elapsed_s = 0.0f;
    state->last_ref_accel = 0.0f;
    state->ref_accel_peak = 0.0f;
    state->last_ref_vx = 0.0f;
    state->last_ref_yaw_rate = 0.0f;
    state->reference_accel = 0.0f;
    state->actual_accel = 0.0f;
    state->selected_accel = 0.0f;
    state->ref_vx_sign = 0;
    state->ref_yaw_sign = 0;
    state->ref_active = 0U;
    state->ref_below_deadband = 0U;
    state->ref_decelerating = 0U;
    state->coast_started = 0U;
    state->coast_active = 0U;
    state->coast_locked = 0U;
    state->coast_lock_reason = LATERAL_FF_LOCK_REASON_NONE;
    state->feedback_valid = 0U;
}

static inline int8_t lateral_inertial_ff_sign(float value)
{
    return value > 0.0f ? 1 : (value < 0.0f ? -1 : 0);
}

static inline uint8_t lateral_inertial_ff_ref_reversed(
    const lateral_inertial_ff_state_t *state,
    int8_t vx_sign, int8_t yaw_sign)
{
    return (uint8_t)(state->ref_active &&
        ((vx_sign != 0 && state->ref_vx_sign != 0 &&
          vx_sign != state->ref_vx_sign) ||
         (yaw_sign != 0 && state->ref_yaw_sign != 0 &&
          yaw_sign != state->ref_yaw_sign)));
}

/* Commanded acceleration remains authoritative; same-direction measured
 * motion only fills a positive reference-to-actual deficit during deceleration. */
static inline float lateral_inertial_ff_step(
    lateral_inertial_ff_state_t *state,
    float ref_accel,
    float ref_vx,
    float ref_yaw_rate,
    float actual_vx,
    float actual_yaw_rate,
    float dt_s,
    uint8_t allowed,
    uint8_t feedback_fresh)
{
    if (state == NULL) {
        return 0.0f;
    }
    if (!allowed || !isfinite(ref_accel) || !isfinite(ref_vx) ||
        !isfinite(ref_yaw_rate)) {
        lateral_inertial_ff_reset(state);
        return 0.0f;
    }

    state->reference_accel = ref_accel;
    state->selected_accel = ref_accel;
    state->feedback_valid = (uint8_t)(feedback_fresh &&
        isfinite(actual_vx) && isfinite(actual_yaw_rate));
    state->actual_accel =
        isfinite(actual_vx) && isfinite(actual_yaw_rate) ?
        actual_vx * actual_yaw_rate : 0.0f;
    if (!isfinite(state->actual_accel)) {
        state->actual_accel = 0.0f;
        state->feedback_valid = 0U;
    }

    const int8_t ref_vx_sign = lateral_inertial_ff_sign(ref_vx);
    const int8_t ref_yaw_sign = lateral_inertial_ff_sign(ref_yaw_rate);
    if (lateral_inertial_ff_ref_reversed(
            state, ref_vx_sign, ref_yaw_sign)) {
        lateral_inertial_ff_reset(state);
        state->reference_accel = ref_accel;
        state->selected_accel = ref_accel;
    }

    const float ref_abs = fabsf(ref_accel);
    const uint8_t ref_active_now =
        ref_abs >= LATERAL_FF_REF_DEADBAND &&
        ref_vx_sign != 0 && ref_yaw_sign != 0;
    uint8_t accelerating = 0U;
    uint8_t decelerating = 0U;

    if (ref_active_now) {
        if (!state->ref_active) {
            state->ref_active = 1U;
            state->ref_accel_peak = ref_abs;
            state->last_ref_accel = ref_accel;
            state->ref_vx_sign = ref_vx_sign;
            state->ref_yaw_sign = ref_yaw_sign;
        } else if (state->coast_locked &&
                   state->ref_below_deadband) {
            state->coast_elapsed_s = 0.0f;
            state->coast_started = 0U;
            state->coast_active = 0U;
            state->coast_locked = 0U;
            state->coast_lock_reason = LATERAL_FF_LOCK_REASON_NONE;
            state->ref_decelerating = 0U;
            state->ref_accel_peak = ref_abs;
            state->last_ref_accel = ref_accel;
            state->ref_vx_sign = ref_vx_sign;
            state->ref_yaw_sign = ref_yaw_sign;
            accelerating = 1U;
        } else {
            const float previous_abs = fabsf(state->last_ref_accel);
            accelerating = (uint8_t)(state->ref_below_deadband ||
                ref_abs > previous_abs + LATERAL_FF_REF_DELTA_EPS);
            decelerating = (uint8_t)(!state->ref_below_deadband &&
                ref_abs < previous_abs - LATERAL_FF_REF_DELTA_EPS);
            if (accelerating) {
                state->ref_decelerating = 0U;
            } else if (decelerating) {
                state->ref_decelerating = 1U;
            }
            if (ref_abs > state->ref_accel_peak) {
                state->ref_accel_peak = ref_abs;
            }
            state->last_ref_accel = ref_accel;
            state->ref_vx_sign = ref_vx_sign;
            state->ref_yaw_sign = ref_yaw_sign;
        }
        state->last_ref_vx = ref_vx;
        state->last_ref_yaw_rate = ref_yaw_rate;
        state->ref_below_deadband = 0U;
    } else if (state->ref_active) {
        decelerating = !state->ref_below_deadband &&
            ref_abs < fabsf(state->last_ref_accel) -
                LATERAL_FF_REF_DELTA_EPS;
        if (decelerating) {
            state->ref_decelerating = 1U;
        }
        state->last_ref_accel = ref_accel;
        if (ref_abs <= LATERAL_FF_REF_REARM_DEADBAND) {
            state->ref_below_deadband = 1U;
        }
    }

    if (!(isfinite(dt_s) && dt_s > 0.0f && dt_s <= 0.020f)) {
        state->feedback_valid = 0U;
        state->coast_active = 0U;
        if (state->ref_active) {
            lateral_inertial_ff_lock(
                state, LATERAL_FF_LOCK_REASON_DT);
        }
        return ref_accel;
    }

    if (state->coast_started && !state->coast_locked) {
        state->coast_elapsed_s += dt_s;
        if (state->coast_elapsed_s >= LATERAL_FF_COAST_MAX_S) {
            state->coast_elapsed_s = LATERAL_FF_COAST_MAX_S;
            state->coast_active = 0U;
            lateral_inertial_ff_lock(
                state, LATERAL_FF_LOCK_REASON_TIMEOUT);
        }
    }

    if (ref_abs < LATERAL_FF_REF_DEADBAND && state->ref_active &&
        !state->coast_locked && !state->coast_started &&
        state->feedback_valid && fabsf(actual_vx) < LATERAL_FF_VX_MIN_MPS &&
        fabsf(actual_yaw_rate) < LATERAL_FF_YAW_MIN_RADPS) {
        lateral_inertial_ff_reset(state);
        return 0.0f;
    }

    const uint8_t actual_in_range = state->feedback_valid &&
        fabsf(actual_vx) >= LATERAL_FF_VX_MIN_MPS &&
        fabsf(actual_vx) <= LATERAL_FF_VX_MAX_MPS &&
        fabsf(actual_yaw_rate) >= LATERAL_FF_YAW_MIN_RADPS &&
        fabsf(actual_yaw_rate) <= LATERAL_FF_YAW_MAX_RADPS;
    const uint8_t actual_same_direction = actual_in_range &&
        lateral_inertial_ff_sign(actual_vx) == state->ref_vx_sign &&
        lateral_inertial_ff_sign(actual_yaw_rate) == state->ref_yaw_sign;
    const uint8_t actual_component_reversed = actual_in_range &&
        ((state->ref_vx_sign != 0 &&
          lateral_inertial_ff_sign(actual_vx) != state->ref_vx_sign) ||
         (state->ref_yaw_sign != 0 &&
         lateral_inertial_ff_sign(actual_yaw_rate) != state->ref_yaw_sign));

    if (state->coast_started && !state->coast_locked &&
        (!state->feedback_valid || !actual_in_range)) {
        lateral_inertial_ff_lock(state,
            state->feedback_valid ?
                LATERAL_FF_LOCK_REASON_OUT_OF_RANGE :
                LATERAL_FF_LOCK_REASON_FEEDBACK_INVALID);
    }

    if (state->coast_started && actual_component_reversed &&
        !state->coast_locked) {
        lateral_inertial_ff_lock(
            state, LATERAL_FF_LOCK_REASON_ACTUAL_REVERSE);
    }

    if (state->coast_started && !state->coast_locked && accelerating) {
        state->coast_active = 0U;
    } else if (!state->coast_locked && actual_same_direction &&
               state->actual_accel != 0.0f &&
               fabsf(state->actual_accel) - ref_abs >=
                   (state->coast_started ?
                       LATERAL_FF_COAST_GAP_OFF :
                       LATERAL_FF_COAST_GAP_ON) &&
               state->ref_decelerating) {
        if (!state->coast_started) {
            state->coast_started = 1U;
            state->coast_elapsed_s = 0.0f;
        }
        state->coast_active = 1U;
    } else {
        state->coast_active = 0U;
    }

    if (!state->coast_active || state->coast_locked ||
        !state->feedback_valid || !actual_same_direction) {
        return ref_accel;
    }

    float selected_abs = fmaxf(ref_abs, fabsf(state->actual_accel));
    if (selected_abs > state->ref_accel_peak) {
        selected_abs = state->ref_accel_peak;
    }

    float coast_gain = 1.0f;
    if (state->coast_elapsed_s >
        LATERAL_FF_COAST_MAX_S - LATERAL_FF_COAST_FADE_S) {
        coast_gain = (LATERAL_FF_COAST_MAX_S - state->coast_elapsed_s) /
                     LATERAL_FF_COAST_FADE_S;
    }
    if (coast_gain < 0.0f) {
        coast_gain = 0.0f;
    }

    const float selected_sign = ref_abs > 0.0f ?
        copysignf(1.0f, ref_accel) : copysignf(1.0f, state->actual_accel);
    state->selected_accel = ref_accel + coast_gain *
        (selected_sign * selected_abs - ref_accel);
    return state->selected_accel;
}

#endif /* LATERAL_INERTIAL_FF_H */
