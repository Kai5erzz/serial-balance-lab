#ifndef JUMP_REACTION_WHEEL_H
#define JUMP_REACTION_WHEEL_H

#include <math.h>
#include <stdint.h>

typedef enum {
    JUMP_WHEEL_OUTPUT_ZERO = 0,
    JUMP_WHEEL_OUTPUT_GROUND_LQR,
    JUMP_WHEEL_OUTPUT_REACTION,
    JUMP_WHEEL_OUTPUT_LANDING_DAMPER,
    JUMP_WHEEL_OUTPUT_RETURN_BLEND,
} jump_wheel_output_source_t;

/* Keep useful pitch authority past the nominal simulated tuck interval, then
 * reduce sustained demand. A settled attitude gets zero output; if attitude or
 * pitch rate drifts back out of the deadband, correction resumes. */
#define JUMP_AIR_WHEEL_TAPER_START_S       0.20f
#define JUMP_AIR_WHEEL_TAPER_END_S         0.40f
#define JUMP_AIR_WHEEL_TAPER_FLOOR         0.50f
#define JUMP_AIR_WHEEL_LATE_LIMIT_START_S  0.80f
#define JUMP_AIR_WHEEL_LATE_AUTHORITY      0.25f
#define JUMP_AIR_WHEEL_SETTLED_ERROR_RAD   0.06981317f
#define JUMP_AIR_WHEEL_SETTLED_RATE_RAD_S  0.20f
#define JUMP_AIR_WHEEL_RECOVER_ERROR_RAD   0.26179939f
#define JUMP_AIR_WHEEL_RECOVER_RATE_RAD_S  0.80f

/* Pure airborne command policy, applied after the existing motor torque clip.
 * The authority falls with elapsed time but never disappears merely because a
 * short fixed profile expired. It reaches zero only in the measured settled
 * deadband or when flight authority is no longer valid. */
static inline float jump_reaction_wheel_air_authority_scale(
    uint8_t admitted, uint8_t off_ground, uint8_t landing_owner,
    uint8_t hard_abort, float air_elapsed_s,
    float pitch_error_rad, float pitch_rate_rad_s)
{
    if (!admitted || !off_ground || landing_owner || hard_abort ||
        !isfinite(air_elapsed_s) || air_elapsed_s < 0.0f ||
        !isfinite(pitch_error_rad) || !isfinite(pitch_rate_rad_s)) {
        return 0.0f;
    }

    float time_scale = 1.0f;
    if (air_elapsed_s > JUMP_AIR_WHEEL_TAPER_START_S) {
        const float taper_span = JUMP_AIR_WHEEL_TAPER_END_S -
                                 JUMP_AIR_WHEEL_TAPER_START_S;
        float progress = (air_elapsed_s - JUMP_AIR_WHEEL_TAPER_START_S) /
                         taper_span;
        if (progress > 1.0f) {
            progress = 1.0f;
        }
        time_scale = 1.0f - progress *
                     (1.0f - JUMP_AIR_WHEEL_TAPER_FLOOR);
    }
    if (air_elapsed_s >= JUMP_AIR_WHEEL_LATE_LIMIT_START_S &&
        time_scale > JUMP_AIR_WHEEL_LATE_AUTHORITY) {
        time_scale = JUMP_AIR_WHEEL_LATE_AUTHORITY;
    }

    float error_scale =
        (fabsf(pitch_error_rad) - JUMP_AIR_WHEEL_SETTLED_ERROR_RAD) /
        (JUMP_AIR_WHEEL_RECOVER_ERROR_RAD -
         JUMP_AIR_WHEEL_SETTLED_ERROR_RAD);
    float rate_scale =
        (fabsf(pitch_rate_rad_s) - JUMP_AIR_WHEEL_SETTLED_RATE_RAD_S) /
        (JUMP_AIR_WHEEL_RECOVER_RATE_RAD_S -
         JUMP_AIR_WHEEL_SETTLED_RATE_RAD_S);
    if (error_scale < 0.0f) {
        error_scale = 0.0f;
    } else if (error_scale > 1.0f) {
        error_scale = 1.0f;
    }
    if (rate_scale < 0.0f) {
        rate_scale = 0.0f;
    } else if (rate_scale > 1.0f) {
        rate_scale = 1.0f;
    }
    const float posture_scale =
        (error_scale > rate_scale) ? error_scale : rate_scale;
    return time_scale * posture_scale;
}

/* A GROUNDED landing buffer owns wheel output, and in THRUST the powered
 * extension keeps ground LQR; after release, either a started air profile or a
 * confirmed off-ground state admits the bounded reaction-wheel channel.
 *
 * 2026-10-06 (jump104, reverse ramp jump): the buffer must NOT deny the channel
 * while the robot is still airborne.  A one-wheel clip opened the pre-contact
 * landing buffer 110 ms before the flight state ended, which hard-zeroed the
 * wheels in mid-air; with the hips already saturated the measured Pitch then
 * ran -5 -> +39 deg in 100 ms and the robot landed on its back (RIGHT_PHI_HIGH).
 * While off-ground the reaction channel is the only pitch authority, so the
 * buffer only takes the wheel away once it is actually on the ground -- which
 * is also exactly when jump_wheel_output_source() selects the damper. */
static inline uint8_t jump_reaction_wheel_admitted(
    uint8_t feature_enabled,
    uint8_t jump_control_active,
    uint8_t flight_state,
    uint8_t off_ground,
    uint8_t air_profile_active,
    uint8_t thrust_state,
    uint8_t thrust_active,
    uint8_t landing_buffer_active)
{
    if (!feature_enabled || !jump_control_active ||
        (landing_buffer_active && !off_ground)) {
        return 0U;
    }

    return (uint8_t)((flight_state &&
                      (off_ground || air_profile_active)) ||
                     (thrust_state && !thrust_active &&
                      (air_profile_active || off_ground)));
}

static inline uint8_t jump_reaction_wheel_final_zero_required(
    uint8_t off_ground, uint8_t airborne_allowed,
    uint8_t reaction_admitted)
{
    return (uint8_t)(off_ground && airborne_allowed &&
                     !reaction_admitted);
}

/* A command admitted as reaction-wheel authority must bypass the contact
 * landing gain as well as the final airborne zero gate.  The CAN callbacks
 * call this shared predicate so both mirrored motor paths preserve exactly
 * the same bounded command when the gain is still zero during reacquisition. */
static inline uint8_t jump_reaction_wheel_landing_damper_authority(
    uint8_t reaction_admitted, uint8_t jump_mode, uint8_t off_ground,
    uint8_t air_profile_active, uint8_t owner_active)
{
    return (uint8_t)(reaction_admitted ||
        (jump_mode && (off_ground || air_profile_active)) ||
        owner_active);
}

static inline uint8_t jump_reaction_wheel_guard_admitted(
    uint8_t feature_enabled, uint8_t guard_active, uint8_t off_ground,
    uint8_t normal_mode, uint8_t normal_stand, uint8_t safe_state)
{
    return (uint8_t)(feature_enabled && guard_active && off_ground &&
                     normal_mode && normal_stand && safe_state);
}

/* Select the same source for both wheel motors. `flight_air_phase` preserves
 * the established hard-off behavior if the reaction-wheel feature is disabled
 * or a precontact landing buffer has already taken ownership. */
static inline jump_wheel_output_source_t jump_wheel_output_source(
    uint8_t reaction_admitted,
    uint8_t flight_air_phase,
    uint8_t off_ground,
    uint8_t landing_buffer_active,
    uint8_t wheel_shut)
{
    if (reaction_admitted) {
        return JUMP_WHEEL_OUTPUT_REACTION;
    }
    if (!off_ground && landing_buffer_active) {
        return JUMP_WHEEL_OUTPUT_LANDING_DAMPER;
    }
    if (flight_air_phase) {
        return JUMP_WHEEL_OUTPUT_ZERO;
    }
    if (!wheel_shut) {
        return JUMP_WHEEL_OUTPUT_GROUND_LQR;
    }
    return JUMP_WHEEL_OUTPUT_ZERO;
}

/* Convert logical torque to mirrored driver polarity. */
static inline float jump_wheel_output_command(
    jump_wheel_output_source_t source,
    uint8_t right_side,
    float reaction_torque,
    float landing_torque,
    float ground_lqr_torque,
    float ground_lqr_gain)
{
    float logical_torque = 0.0f;
    switch (source) {
    case JUMP_WHEEL_OUTPUT_REACTION:
        logical_torque = reaction_torque;
        break;
    case JUMP_WHEEL_OUTPUT_LANDING_DAMPER:
        logical_torque = landing_torque;
        break;
    case JUMP_WHEEL_OUTPUT_GROUND_LQR:
        logical_torque = ground_lqr_torque * ground_lqr_gain;
        break;
    case JUMP_WHEEL_OUTPUT_RETURN_BLEND:
        logical_torque = landing_torque + ground_lqr_gain *
            (ground_lqr_torque - landing_torque);
        break;
    case JUMP_WHEEL_OUTPUT_ZERO:
    default:
        return 0.0f;
    }
    return right_side ? logical_torque : -logical_torque;
}

#endif /* JUMP_REACTION_WHEEL_H */
