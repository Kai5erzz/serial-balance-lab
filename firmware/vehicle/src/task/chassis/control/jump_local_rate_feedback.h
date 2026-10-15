#ifndef CHASSIS_JUMP_LOCAL_RATE_FEEDBACK_H
#define CHASSIS_JUMP_LOCAL_RATE_FEEDBACK_H

#include <math.h>
#include <stdint.h>

typedef struct {
    float rate;
    uint8_t valid;
    uint8_t observer_selected;
} jump_local_rate_selection_t;

static inline uint8_t jump_local_rate_snapshot_usable(
    uint8_t usable, float age_ms, float maximum_age_ms)
{
    return (uint8_t)(usable && isfinite(age_ms) && age_ms >= 0.0f &&
                     isfinite(maximum_age_ms) && maximum_age_ms >= 0.0f &&
                     age_ms <= maximum_age_ms);
}

/* Jump-only consumers may opt into timestamp-observer rates.  An eligible
 * consumer never silently treats a missing/stale observer as legacy data. */
static inline jump_local_rate_selection_t jump_local_rate_select(
    uint8_t feature_enabled,
    uint8_t observer_compiled,
    uint8_t jump_scope,
    uint8_t observer_usable,
    float observer_rate,
    float legacy_rate)
{
    jump_local_rate_selection_t result = {0.0f, 0U, 0U};
    if (feature_enabled && observer_compiled && jump_scope) {
        result.observer_selected = 1U;
        if (observer_usable && isfinite(observer_rate)) {
            result.rate = observer_rate;
            result.valid = 1U;
        }
        return result;
    }
    if (isfinite(legacy_rate)) {
        result.rate = legacy_rate;
        result.valid = 1U;
    }
    return result;
}

static inline float jump_local_rate_pair_average_abs(
    jump_local_rate_selection_t left,
    jump_local_rate_selection_t right)
{
    if (!left.valid || !right.valid || !isfinite(left.rate) ||
        !isfinite(right.rate)) {
        return INFINITY;
    }
    return 0.5f * (fabsf(left.rate) + fabsf(right.rate));
}

#endif /* CHASSIS_JUMP_LOCAL_RATE_FEEDBACK_H */
