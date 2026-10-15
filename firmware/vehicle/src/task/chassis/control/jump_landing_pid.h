/* Narrow landing-PID policy for a landing buffer captured during CHASSIS_JUMP. */
#ifndef CHASSIS_JUMP_LANDING_PID_H
#define CHASSIS_JUMP_LANDING_PID_H

#include <math.h>
#include <float.h>
#include <stddef.h>
#include <stdint.h>
#include "../../../algorithm/PID/PID.h"

#define JUMP_LANDING_RATE_FEEDBACK_MAX_AGE_MS 12U
#define JUMP_LANDING_RATE_SNAPSHOT_MAX_SKEW_MS 10U
#define JUMP_LANDING_RATE_PAIR_MAX_SKEW_MS 4U
#define JUMP_LANDING_RATE_MAX_VALID_AGE_MS 12.0f
#define JUMP_LANDING_RATE_FILTER_TAU_S 0.004f
/* Jump batch polling supplies a distinct hip pair every 2 ms. HAL timestamps
 * have 1 ms resolution; rejecting those pairs would disable landing damping
 * and keep the RETURN stability gate closed indefinitely. */
#define JUMP_LANDING_RATE_MIN_SAMPLE_DT_MS 1.0f
#define JUMP_LANDING_RATE_MAX_SAMPLE_DT_MS 25.0f

static inline uint8_t jump_landing_feedback_age_ms(
    uint32_t now_ms, uint32_t sample_ms, uint32_t *age_ms)
{
    if (age_ms == NULL) {
        return 0U;
    }
    const uint32_t elapsed = now_ms - sample_ms;
    if (elapsed <= 0x7fffffffU) {
        *age_ms = elapsed;
        return 1U;
    }
    const uint32_t future = sample_ms - now_ms;
    if (future <= 1U) {
        *age_ms = 0U;
        return 1U;
    }
    return 0U;
}

static inline uint8_t jump_landing_feedback_snapshot_fresh(
    uint32_t now_ms, const uint32_t rx_count[4],
    const uint32_t last_rx_ms[4], uint32_t max_age_ms,
    uint32_t max_skew_ms, uint32_t age_ms[4])
{
    if (rx_count == NULL || last_rx_ms == NULL) {
        return 0U;
    }
    uint32_t oldest_age = 0U;
    uint32_t newest_age = UINT32_MAX;
    for (uint8_t i = 0U; i < 4U; ++i) {
        uint32_t age = 0U;
        if (rx_count[i] == 0U ||
            !jump_landing_feedback_age_ms(now_ms, last_rx_ms[i], &age) ||
            age > max_age_ms) {
            return 0U;
        }
        if (age_ms != NULL) {
            age_ms[i] = age;
        }
        if (age > oldest_age) oldest_age = age;
        if (age < newest_age) newest_age = age;
    }
    return (uint8_t)(oldest_age - newest_age <= max_skew_ms);
}

static inline uint8_t jump_landing_feedback_pair_fresh(
    uint32_t now_ms, const uint32_t rx_count[2],
    const uint32_t last_rx_ms[2], uint32_t age_ms[2])
{
    if (rx_count == NULL || last_rx_ms == NULL) {
        return 0U;
    }
    uint32_t age[2] = {0U, 0U};
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (rx_count[i] == 0U ||
            !jump_landing_feedback_age_ms(
                now_ms, last_rx_ms[i], &age[i]) ||
            age[i] > JUMP_LANDING_RATE_FEEDBACK_MAX_AGE_MS) {
            return 0U;
        }
    }
    const uint32_t skew = age[0] > age[1] ?
        age[0] - age[1] : age[1] - age[0];
    if (age_ms != NULL) {
        age_ms[0] = age[0];
        age_ms[1] = age[1];
    }
    return (uint8_t)(skew <= JUMP_LANDING_RATE_PAIR_MAX_SKEW_MS);
}

typedef struct {
    uint32_t accepted_rx_count[2];
    uint8_t initialized;
} jump_landing_rate_gate_t;

/* Each leg has its own adjacent two-motor polling pair. */
static inline uint8_t jump_landing_rate_gate_accept(
    jump_landing_rate_gate_t *gate, const uint32_t rx_count[2],
    const uint8_t feedback_fresh)
{
    if (gate == NULL || rx_count == NULL || !feedback_fresh) {
        return 0U;
    }
    uint8_t advanced = 1U;
    for (uint8_t i = 0U; i < 2U; ++i) {
        if (rx_count[i] == 0U ||
            (gate->initialized && rx_count[i] == gate->accepted_rx_count[i])) {
            advanced = 0U;
        }
    }
    if (!advanced) {
        return 0U;
    }
    for (uint8_t i = 0U; i < 2U; ++i) {
        gate->accepted_rx_count[i] = rx_count[i];
    }
    gate->initialized = 1U;
    return 1U;
}

typedef struct {
    jump_landing_rate_gate_t gate;
    float previous_length;
    float previous_time_ms;
    float rate;
    float last_valid_time_ms;
    uint8_t sample_initialized;
    uint8_t rate_valid;
} jump_landing_rate_state_t;

typedef struct {
    float rate;
    float valid_rate_age_ms;
    uint8_t usable;
} jump_landing_rate_view_t;

static inline void jump_landing_rate_state_reset(
    jump_landing_rate_state_t *state)
{
    if (state != NULL) {
        state->gate.accepted_rx_count[0] = 0U;
        state->gate.accepted_rx_count[1] = 0U;
        state->gate.initialized = 0U;
        state->previous_length = 0.0f;
        state->previous_time_ms = 0.0f;
        state->rate = 0.0f;
        state->last_valid_time_ms = 0.0f;
        state->sample_initialized = 0U;
        state->rate_valid = 0U;
    }
}

static inline void jump_landing_rate_state_expire(
    jump_landing_rate_state_t *state, float now_ms)
{
    if (state == NULL || !state->rate_valid) {
        return;
    }
    const float age_ms = now_ms - state->last_valid_time_ms;
    if (!isfinite(age_ms) || age_ms < 0.0f ||
        age_ms > JUMP_LANDING_RATE_MAX_VALID_AGE_MS) {
        jump_landing_rate_state_reset(state);
    }
}

/* Only a distinct fresh pair enters the two-sample derivative. Its timestamp
 * is the midpoint of the two HAL-tick feedback frames, mapped by the caller to
 * the same DWT-millisecond origin as now_ms. */
static inline uint8_t jump_landing_rate_state_update(
    jump_landing_rate_state_t *state, const uint32_t rx_count[2],
    uint8_t feedback_fresh, float length, float sample_time_ms)
{
    if (state == NULL || rx_count == NULL || !feedback_fresh ||
        !isfinite(length) || !isfinite(sample_time_ms) ||
        !jump_landing_rate_gate_accept(
            &state->gate, rx_count, feedback_fresh)) {
        return 0U;
    }
    if (!state->sample_initialized) {
        state->previous_length = length;
        state->previous_time_ms = sample_time_ms;
        state->sample_initialized = 1U;
        state->rate_valid = 0U;
        return 0U;
    }

    const float sample_dt_ms = sample_time_ms - state->previous_time_ms;
    if (!isfinite(sample_dt_ms) ||
        sample_dt_ms < JUMP_LANDING_RATE_MIN_SAMPLE_DT_MS ||
        sample_dt_ms > JUMP_LANDING_RATE_MAX_SAMPLE_DT_MS) {
        state->previous_length = length;
        state->previous_time_ms = sample_time_ms;
        state->rate = 0.0f;
        state->rate_valid = 0U;
        return 0U;
    }

    const float raw_rate = (length - state->previous_length) /
                           (sample_dt_ms * 0.001f);
    if (!isfinite(raw_rate)) {
        jump_landing_rate_state_reset(state);
        return 0U;
    }
    if (!state->rate_valid) {
        state->rate = raw_rate;
    } else {
        const float dt_s = sample_dt_ms * 0.001f;
        const float alpha = dt_s /
            (JUMP_LANDING_RATE_FILTER_TAU_S + dt_s);
        state->rate += alpha * (raw_rate - state->rate);
    }
    state->previous_length = length;
    state->previous_time_ms = sample_time_ms;
    state->last_valid_time_ms = sample_time_ms;
    state->rate_valid = 1U;
    return 1U;
}

static inline jump_landing_rate_view_t jump_landing_rate_state_view(
    const jump_landing_rate_state_t *state, float now_ms)
{
    jump_landing_rate_view_t view = {0};
    view.valid_rate_age_ms = INFINITY;
    if (state == NULL || !state->rate_valid || !isfinite(now_ms)) {
        return view;
    }
    view.valid_rate_age_ms = now_ms - state->last_valid_time_ms;
    if (!isfinite(view.valid_rate_age_ms) ||
        view.valid_rate_age_ms < 0.0f) {
        view.valid_rate_age_ms = INFINITY;
        return view;
    }
    view.rate = state->rate;
    view.usable = (uint8_t)(
        view.valid_rate_age_ms <= JUMP_LANDING_RATE_MAX_VALID_AGE_MS);
    return view;
}

static inline float jump_landing_external_d_force(
    float kd, float length_rate, uint8_t rate_valid)
{
    if (!rate_valid || !isfinite(kd) || kd < 0.0f ||
        !isfinite(length_rate)) {
        return 0.0f;
    }
    return -kd * length_rate;
}

static inline float jump_landing_sync_force_calculate(
    float length_diff, float length_rate_diff, float kp, float kd,
    float active_gain, float force_limit)
{
    if (!isfinite(length_diff) || !isfinite(length_rate_diff) ||
        !isfinite(kp) || !isfinite(kd) || !isfinite(active_gain) ||
        !isfinite(force_limit) || force_limit < 0.0f) {
        return 0.0f;
    }
    if (active_gain < 0.0f) active_gain = 0.0f;
    if (active_gain > 1.0f) active_gain = 1.0f;
    float force = active_gain *
        (kp * length_diff + kd * length_rate_diff);
    if (!isfinite(force)) return 0.0f;
    if (force > force_limit) force = force_limit;
    if (force < -force_limit) force = -force_limit;
    return force;
}

typedef struct {
    uint8_t active;
    uint8_t saved_dom_bit;
    float external_d_filtered;
    float external_output_history;
    uint8_t external_output_history_valid;
} jump_landing_pid_state_t;

/* Run the existing PID P/I calculation with adjacent-call D disabled, then
 * add timestamped length-rate D before the configured filters and one common
 * output clamp. On invalid feedback, clear D history at once so no stale
 * damping force survives the lost measurement. */
static inline float jump_landing_pid_apply_external_rate(
    pid_obj_t *pid, jump_landing_pid_state_t *state,
    float measurement, float reference, float kd,
    float length_rate, uint8_t rate_valid)
{
    if (pid == NULL || state == NULL) {
        return 0.0f;
    }
    if (!isfinite(measurement) || !isfinite(reference)) {
        state->external_d_filtered = 0.0f;
        state->external_output_history = 0.0f;
        state->external_output_history_valid = 1U;
        pid->Dout = 0.0f;
        pid->Last_Dout = 0.0f;
        pid->Output = 0.0f;
        pid->Last_Output = 0.0f;
        return 0.0f;
    }
    const pid_improvement_e original_improve = pid->Improve;
    const float original_kd = pid->Kd;
    const float original_max_out = pid->MaxOut;
    if (!state->external_output_history_valid) {
        state->external_output_history = isfinite(pid->Last_Output) ?
            pid->Last_Output : 0.0f;
        state->external_output_history_valid = 1U;
    }

    /* Run the existing P/I implementation with adjacent-call D and both
     * filters bypassed. Otherwise stale Last_Dout can leak into raw_pi before
     * the timestamped D is added. The final sum receives each configured
     * filter once and one common output clamp. */
    pid->Kd = 0.0f;
    pid->MaxOut = FLT_MAX;
    pid->Improve = (pid_improvement_e)(
        (unsigned)original_improve &
        ~((unsigned)PID_OutputFilter | (unsigned)PID_DerivativeFilter));
    (void)pid_calculate(pid, measurement, reference);
    const float raw_pi = fabsf(pid->Err) <= pid->DeadBand ? 0.0f :
        pid->Pout + pid->Iout;
    pid->Kd = original_kd;
    pid->MaxOut = original_max_out;
    pid->Improve = original_improve;

    const uint8_t d_valid = (uint8_t)(
        rate_valid && isfinite(kd) && kd >= 0.0f &&
        isfinite(length_rate) && isfinite(raw_pi));
    float dt = pid->dt;
    if (!isfinite(dt) || dt < 0.0002f || dt > 0.020f) {
        dt = 0.001f;
    }
    if (!d_valid) {
        state->external_d_filtered = 0.0f;
        pid->Dout = 0.0f;
        pid->Last_Dout = 0.0f;
    } else {
        const float raw_d = -kd * length_rate;
        const float derivative_rc = pid->Derivative_LPF_RC;
        if ((pid->Improve & PID_DerivativeFilter) != 0 &&
            isfinite(derivative_rc) && derivative_rc > 0.0f) {
            const float denominator = derivative_rc + dt;
            state->external_d_filtered =
                raw_d * dt / denominator +
                state->external_d_filtered * derivative_rc / denominator;
        } else {
            state->external_d_filtered = raw_d;
        }
        pid->Dout = state->external_d_filtered;
        pid->Last_Dout = pid->Dout;
    }

    float output = raw_pi + state->external_d_filtered;
    const float output_rc = pid->Output_LPF_RC;
    if (d_valid && (pid->Improve & PID_OutputFilter) != 0 &&
        isfinite(output_rc) && output_rc > 0.0f) {
        const float denominator = output_rc + dt;
        output = output * dt / denominator +
            state->external_output_history * output_rc / denominator;
    }
    if (isfinite(pid->MaxOut) && pid->MaxOut >= 0.0f) {
        if (output > pid->MaxOut) output = pid->MaxOut;
        if (output < -pid->MaxOut) output = -pid->MaxOut;
    }
    if (!isfinite(output)) output = 0.0f;
    if (!d_valid) {
        /* A stale observer must not leave its previous D contribution in the
         * output-filter state. P/I remains live; only external D fails closed. */
        state->external_output_history = raw_pi;
    }
    pid->Output = output;
    pid->Last_Output = output;
    state->external_output_history = output;
    return output;
}

static inline float jump_landing_pid_select_kd(
    uint8_t jump_origin_buffer, uint8_t low_leg, float normal_kd,
    float rebound_kd, float length_rate)
{
    if (jump_origin_buffer && low_leg) {
        return normal_kd;
    }
    return length_rate > 0.0f ? rebound_kd : normal_kd;
}

static inline void jump_landing_pid_seed(pid_obj_t *pid,
                                         float measurement,
                                         float reference)
{
    if (!isfinite(measurement) || !isfinite(reference) ||
        measurement <= 0.0f) {
        return;
    }
    pid->Measure = measurement;
    pid->Last_Measure = measurement;
    pid->Err = reference - measurement;
    pid->Last_Err = pid->Err;
    pid->Dout = 0.0f;
    pid->Last_Dout = 0.0f;
}

/* Keep each PID's preexisting DoM setting intact outside the jump-origin
 * buffer. Re-seed whenever production pid_clear() has zeroed both measures. */
static inline void jump_landing_pid_prepare(
    pid_obj_t *pid, jump_landing_pid_state_t *state,
    uint8_t jump_origin_buffer, float measurement, float reference)
{
    if (pid == NULL || state == NULL) {
        return;
    }
    const unsigned dom_bit = (unsigned)PID_Derivative_On_Measurement;
    if (jump_origin_buffer) {
        if (!state->active) {
            state->saved_dom_bit =
                (uint8_t)(((unsigned)pid->Improve & dom_bit) != 0U);
            state->active = 1U;
            state->external_d_filtered = 0.0f;
            state->external_output_history = isfinite(pid->Last_Output) ?
                pid->Last_Output : 0.0f;
            state->external_output_history_valid = 1U;
            jump_landing_pid_seed(pid, measurement, reference);
        } else if (isfinite(measurement) && measurement > 0.0f &&
                   pid->Measure == 0.0f && pid->Last_Measure == 0.0f) {
            jump_landing_pid_seed(pid, measurement, reference);
            state->external_d_filtered = 0.0f;
            state->external_output_history = isfinite(pid->Last_Output) ?
                pid->Last_Output : 0.0f;
            state->external_output_history_valid = 1U;
        }
        pid->Improve = (pid_improvement_e)((unsigned)pid->Improve | dom_bit);
        return;
    }

    if (state->active) {
        if (state->saved_dom_bit) {
            pid->Improve = (pid_improvement_e)((unsigned)pid->Improve | dom_bit);
        } else {
            pid->Improve = (pid_improvement_e)((unsigned)pid->Improve & ~dom_bit);
        }
        state->active = 0U;
        state->external_d_filtered = 0.0f;
        state->external_output_history_valid = 0U;
        /* The ordinary PID resumes adjacent-sample D with the same filtered
         * rate and output history, preserving the final postbuffer handoff.
         * Reset only error history so a reference change cannot kick D. */
        if (isfinite(pid->Measure) && isfinite(reference)) {
            pid->Err = reference - pid->Measure;
            pid->Last_Err = pid->Err;
        }
    }
}

#endif /* CHASSIS_JUMP_LANDING_PID_H */
