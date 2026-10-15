#ifndef JUMP_STROKE_RELEASE_H
#define JUMP_STROKE_RELEASE_H

#include <math.h>
#include <stdint.h>

#ifndef JUMP_STROKE_COMPLETION_RELEASE_ENABLE
#define JUMP_STROKE_COMPLETION_RELEASE_ENABLE 1
#endif

typedef enum {
    JUMP_STROKE_RELEASE_NONE = 0,
    JUMP_STROKE_RELEASE_TRAVEL = 1,
    JUMP_STROKE_RELEASE_TIMER = 2,
} jump_stroke_release_reason_t;

typedef struct {
    uint8_t valid;
    uint8_t powered;
    jump_stroke_release_reason_t reason;
    float release_elapsed_origin_s;
    float profile_elapsed_s;
} jump_stroke_release_result_t;

/* Pure per-cycle policy used directly by the firmware and host tests.  The
 * caller owns `reason` and `release_elapsed_origin_s`, preserving them across
 * FLIGHT/LANDING for diagnostics.  The feature-off path reproduces the old
 * timer release and anchors its profile to the configured timer edge even if
 * this function is first called late. */
static inline jump_stroke_release_result_t jump_stroke_release_step(
    uint8_t thrust_phase,
    uint8_t feature_enabled,
    float average_length_m,
    float elapsed_s,
    float target_length_m,
    float timer_s,
    jump_stroke_release_reason_t previous_reason,
    float previous_origin_s)
{
    jump_stroke_release_result_t result = {
        0U, 0U, previous_reason, previous_origin_s, 0.0f
    };

    if (!isfinite(average_length_m) || !isfinite(elapsed_s) ||
        !isfinite(target_length_m) || !isfinite(timer_s) ||
        !isfinite(previous_origin_s) || elapsed_s < 0.0f ||
        average_length_m <= 0.0f || target_length_m <= 0.0f || timer_s <= 0.0f ||
        previous_reason < JUMP_STROKE_RELEASE_NONE ||
        previous_reason > JUMP_STROKE_RELEASE_TIMER ||
        (previous_reason != JUMP_STROKE_RELEASE_NONE &&
         (previous_origin_s < 0.0f || previous_origin_s > elapsed_s))) {
        return result;
    }
    result.valid = 1U;

    if (!thrust_phase) {
        return result;
    }

    if (result.reason == JUMP_STROKE_RELEASE_NONE) {
        if (elapsed_s >= timer_s) {
            result.reason = JUMP_STROKE_RELEASE_TIMER;
            result.release_elapsed_origin_s = timer_s;
        } else if (feature_enabled &&
            average_length_m >= target_length_m) {
            result.reason = JUMP_STROKE_RELEASE_TRAVEL;
            result.release_elapsed_origin_s = elapsed_s;
        }
    }

    result.powered =
        (uint8_t)(result.reason == JUMP_STROKE_RELEASE_NONE &&
                  elapsed_s < timer_s);
    if (result.reason != JUMP_STROKE_RELEASE_NONE) {
        result.profile_elapsed_s =
            elapsed_s - result.release_elapsed_origin_s;
        if (result.profile_elapsed_s < 0.0f) {
            result.profile_elapsed_s = 0.0f;
        }
    }
    return result;
}

static inline uint16_t jump_stroke_release_status_bits(
    jump_stroke_release_reason_t reason)
{
    if (reason == JUMP_STROKE_RELEASE_TRAVEL) {
        return (uint16_t)(1U << 12);
    }
    if (reason == JUMP_STROKE_RELEASE_TIMER) {
        return (uint16_t)(1U << 13);
    }
    return 0U;
}

#endif
