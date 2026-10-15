#ifndef JUMP_THETA_DIFF_D_HELPERS_H
#define JUMP_THETA_DIFF_D_HELPERS_H

#include <math.h>
#include <stdint.h>

#ifndef JUMP_THRUST_THETA_DIFF_D_RATE_OBSERVER_ENABLE
#define JUMP_THRUST_THETA_DIFF_D_RATE_OBSERVER_ENABLE 1
#endif

typedef struct {
    float rate_diff;
    uint8_t observer_selected;
    uint8_t observer_valid;
} jump_theta_diff_d_selection_t;

/* Kept pure so the production source selector can be host-tested directly. */
static inline jump_theta_diff_d_selection_t
jump_theta_diff_d_select_rate_diff(
    uint8_t feature_enabled,
    uint8_t jump_mode,
    uint8_t thrust_state,
    uint8_t thrust_active,
    uint8_t off_ground,
    uint8_t observer_usable,
    float legacy_rate_diff,
    float observer_theta_rate_left,
    float observer_theta_rate_right)
{
    jump_theta_diff_d_selection_t result = {legacy_rate_diff, 0U, 0U};
    if (feature_enabled && jump_mode && thrust_state && thrust_active &&
        !off_ground) {
        /* An eligible powered-thrust consumer must never relabel legacy data
         * as a valid timestamped rate when the observer is bad or expired. */
        result.observer_selected = 1U;
        result.rate_diff = 0.0f;
        if (observer_usable && isfinite(observer_theta_rate_left) &&
            isfinite(observer_theta_rate_right)) {
            const float diff = observer_theta_rate_left -
                               observer_theta_rate_right;
            if (isfinite(diff)) {
                result.rate_diff = diff;
                result.observer_valid = 1U;
            }
        }
    }
    return result;
}

/* `before` and `after` are captured around the real LIMIT_MIN_MAX call. */
static inline uint8_t jump_dm_clip_was_applied(float before,
                                               float after,
                                               float limit)
{
    if (!isfinite(before) || !isfinite(after) || !isfinite(limit) ||
        limit < 0.0f || before == after) {
        return 0U;
    }
    return (uint8_t)(after == limit || after == -limit);
}

static inline uint8_t jump_dm_clip_accumulate(uint8_t prior,
                                               uint8_t clipped)
{
    return (uint8_t)(prior || clipped);
}

static inline uint8_t jump_dm_clip_finish(uint8_t clipped,
                                           float final_torque)
{
    return final_torque == 0.0f ? 0U : (uint8_t)(clipped != 0U);
}

/* Physical order in Page 57 bits 8..11: LF, LB, RF, RB. */
static inline uint16_t jump_page57_add_dm_clip_bits(uint16_t status,
                                                     uint8_t lf,
                                                     uint8_t lb,
                                                     uint8_t rf,
                                                     uint8_t rb)
{
    if (lf) status |= (uint16_t)(1U << 8);
    if (lb) status |= (uint16_t)(1U << 9);
    if (rf) status |= (uint16_t)(1U << 10);
    if (rb) status |= (uint16_t)(1U << 11);
    return status;
}

#endif /* JUMP_THETA_DIFF_D_HELPERS_H */
