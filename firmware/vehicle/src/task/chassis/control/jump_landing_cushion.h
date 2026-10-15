#ifndef CHASSIS_JUMP_LANDING_CUSHION_H
#define CHASSIS_JUMP_LANDING_CUSHION_H

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* Motor landing cushion (2026-10-04, jump100).
 *
 * The measured jump touchdown arrives with the legs at 0.35-0.38 m (the gas
 * spring has already flung them to the mechanical stop) and the compression
 * that follows is absorbed mostly by that spring plus two INDEPENDENT length
 * loops.  jump100 showed those loops commanding opposite signs on the two legs
 * at first contact (+156/-27 N, -114/+115 N), i.e. the pair produced a roll
 * moment instead of a cushion, and the rods were already at 3-9 rad/s while
 * the jump-specific landing controller had only 50-70 ms of authority.
 *
 * 2026-10-04 (jump102): v18 made this a ONE-SHOT per landing to stop the
 * jump101 bounce pumping.  That was wrong in the other direction: the one-shot
 * burned on the first, shallow touch (the leg briefly extended, which closed
 * it) and the second, much deeper collapse 60-120 ms later had NO cushion at
 * all -- measured support force -83 N while the leg was still compressing at
 * 1.4 m/s, i.e. the robot free-fell onto the fold stop.  The pumping is instead
 * prevented by the FAST RELEASE below (30000 N/s: the force is gone ~5 ms after
 * the compression ends, so it can never push into a rebound); replaying the
 * measured rates gives 0.00 N.s of rebound impulse while the delivered
 * compression impulse rises 2-5x versus the one-shot version.
 *
 * This policy adds an explicit, symmetric braking force on the axial channel:
 *
 *   F_cushion = kd * max(0, -v_common - deadband)     (>= 0, one value)
 *
 * applied to BOTH legs, so it can never create a left/right split or a body
 * roll moment, and it does negative work against the compression rate instead
 * of relying on the spring.  The force only ever pushes (never pulls), so it
 * cannot inject extension energy into a rebound. */

typedef struct {
    float applied_force;
    float armed_elapsed_s;
    uint8_t armed;
} jump_landing_cushion_t;

static inline void jump_landing_cushion_reset(jump_landing_cushion_t *state)
{
    if (state == NULL) return;
    state->applied_force = 0.0f;
    state->armed_elapsed_s = 0.0f;
    state->armed = 0U;
}

/* `rate_*` are signed leg-length rates in m/s, negative while compressing. */
static inline float jump_landing_cushion_target(
    float rate_left, float rate_right, float kd, float force_limit,
    float rate_deadband)
{
    if (!isfinite(rate_left) || !isfinite(rate_right) ||
        !isfinite(kd) || kd <= 0.0f ||
        !isfinite(force_limit) || force_limit <= 0.0f ||
        !isfinite(rate_deadband) || rate_deadband < 0.0f) {
        return 0.0f;
    }
    const float rate_common = 0.5f * (rate_left + rate_right);
    if (!isfinite(rate_common) || rate_common > -rate_deadband) {
        return 0.0f;
    }
    float force = kd * (-rate_common - rate_deadband);
    if (!isfinite(force) || force < 0.0f) {
        return 0.0f;
    }
    if (force > force_limit) {
        force = force_limit;
    }
    return force;
}

/* Slew the applied force so that arming, disarming or a mode transition can
 * never step the axial command.  The 1 kHz control loop gives about 15 ms of
 * rise and 70 ms of fall with the configured rates. */
static inline float jump_landing_cushion_slew(
    float applied, float target, float dt_s, float rise_rate, float fall_rate)
{
    if (!isfinite(applied) || applied < 0.0f) applied = 0.0f;
    if (!isfinite(target) || target < 0.0f) target = 0.0f;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.050f) return applied;
    const float rate = (target > applied) ? rise_rate : fall_rate;
    if (!isfinite(rate) || rate <= 0.0f) return target;
    const float max_step = rate * dt_s;
    const float delta = target - applied;
    if (delta > max_step) return applied + max_step;
    if (delta < -max_step) return applied - max_step;
    return target;
}

/* Arm once at confirmed contact and then serve a bounded window.  The window is
 * deliberately independent of the landing buffer, because the fast-exit
 * handoff resets that buffer while the legs are still compressing. */
static inline uint8_t jump_landing_cushion_window_step(
    jump_landing_cushion_t *state, float dt_s, uint8_t arm_request,
    uint8_t keep_alive, float window_s)
{
    if (state == NULL) return 0U;
    if (!isfinite(dt_s) || dt_s <= 0.0f || dt_s > 0.050f) {
        dt_s = 0.0f;
    }
    if (!isfinite(window_s) || window_s <= 0.0f) {
        jump_landing_cushion_reset(state);
        return 0U;
    }
    if (arm_request && !state->armed) {
        state->armed = 1U;
        state->armed_elapsed_s = 0.0f;
    }
    if (!state->armed) {
        return 0U;
    }
    if (!keep_alive) {
        jump_landing_cushion_reset(state);
        return 0U;
    }
    state->armed_elapsed_s += dt_s;
    if (state->armed_elapsed_s >= window_s) {
        state->armed = 0U;
        return 0U;
    }
    return 1U;
}

#endif /* CHASSIS_JUMP_LANDING_CUSHION_H */
