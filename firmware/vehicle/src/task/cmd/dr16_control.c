#include "dr16_control.h"

#include <math.h>
#include <stdlib.h>

#include "rm_algorithm.h"
#include "rm_config.h"

#define DR16_SWITCH_UP                    1U
#define DR16_SWITCH_MIDDLE                3U
#define DR16_SWITCH_DOWN                  2U

#define DR16_WHEEL_THRESHOLD              140
#define DR16_WHEEL_CONTINUOUS_MS          450U
#define DR16_FRICTION_WARMUP_MS           400U
#define DR16_SINGLE_FINISH_MS             300U
#define DR16_JAM_REVERSE_MS               300U
#define DR16_JAM_CURRENT_THRESHOLD        10000
#define DR16_JUMP_COOLDOWN_MS              3000U
#define DR16_JUMP_REQUEST_HOLD_MS          1000U

#define DR16_SPIN_RC_COMMAND_LOW         1200.0f
#define DR16_SPIN_RC_COMMAND_MID_HIGH    1200.0f
#define DR16_SPIN_DIRECTION              (-1.0f)
#define DR16_SPIN_RAMP_STEP               2.0f
#define DR16_CHANNEL_VALID_LIMIT           700
#define DR16_WHEEL_VALID_LIMIT             1000

/* Physical DR16 mapping:
 * right switch: UP=relax, MIDDLE=follow gimbal, DOWN=spin;
 * left switch: UP/MIDDLE/DOWN=high/mid/low leg;
 * wheel: physical downward/positive edge requests one jump.  Positive fire
 *        handling remains dormant while the jump request owns that edge.
 *        Friction-wheel
 *        toggling is temporarily displaced while jump tuning is active. */

typedef enum {
    DR16_WHEEL_NEGATIVE = -1,
    DR16_WHEEL_CENTER = 0,
    DR16_WHEEL_POSITIVE = 1,
} dr16_wheel_region_e;

static uint8_t friction_latched;
static uint8_t fire_edge_pending;
static int8_t wheel_region_last;
static uint32_t wheel_region_enter_ms;
static uint32_t friction_on_ms;
static uint32_t single_finish_ms;
static uint32_t jam_reverse_finish_ms;
static uint8_t jump_wheel_armed = 1U;
static uint32_t jump_cooldown_until_ms;
static uint8_t jump_request_pending;
static uint32_t jump_request_pending_until_ms;
static uint8_t bump_switch_last_up;

static uint8_t time_before(uint32_t now, uint32_t deadline)
{
    return (int32_t)(deadline - now) > 0;
}

static dr16_wheel_region_e wheel_region_get(int16_t wheel)
{
    if (wheel > DR16_WHEEL_THRESHOLD) {
        return DR16_WHEEL_POSITIVE;
    }
    if (wheel < -DR16_WHEEL_THRESHOLD) {
        return DR16_WHEEL_NEGATIVE;
    }
    return DR16_WHEEL_CENTER;
}

static leg_level_e leg_level_get(uint8_t left_switch)
{
    switch (left_switch) {
        case DR16_SWITCH_UP:
            return LEG_HIG;
        case DR16_SWITCH_MIDDLE:
            return LEG_MID;
        case DR16_SWITCH_DOWN:
        default:
            return LEG_LOW;
    }
}

static uint8_t chassis_is_operator_mode(chassis_mode_e mode)
{
    return mode == CHASSIS_OPEN_LOOP ||
           mode == CHASSIS_FOLLOW_GIMBAL ||
           mode == CHASSIS_SPIN;
}

static uint8_t jump_request_update(const Remote_Info_Typedef *remote,
                                   const struct chassis_fdb_msg *chassis_fdb,
                                   chassis_mode_e previous_mode,
                                   uint8_t master_relax,
                                   uint8_t *gate)
{
    const uint32_t now = HAL_GetTick();
    const dr16_wheel_region_e region =
        wheel_region_get(remote->rc.ch[4]);

    *gate = 0U;

    /* Keep a valid wheel edge alive across the command/chassis task handoff.
     * A one-cycle pulse could disappear before the chassis had published its
     * JUMPING acknowledgement, especially while the velocity command was
     * changing.  The feedback acknowledgement consumes the request and starts
     * the independent command-side cooldown. */
    if (jump_request_pending) {
        if (chassis_fdb->stand_state == CHASSIS_IS_JUMPING) {
            jump_request_pending = 0U;
            jump_cooldown_until_ms = now + DR16_JUMP_COOLDOWN_MS;
            *gate = 8U;
            return 0U;
        }
        if (master_relax) {
            jump_request_pending = 0U;
            *gate = 4U;
            return 0U;
        }
        if (time_before(now, jump_request_pending_until_ms)) {
            *gate = 1U;
            return 1U;
        }
        jump_request_pending = 0U;
        *gate = 9U;
        return 0U;
    }

    /* Returning through centre is mandatory before another request.  This is
     * independent of the 3 s cooldown, so a held or bouncing wheel cannot
     * queue a second jump. */
    if (region == DR16_WHEEL_CENTER) {
        jump_wheel_armed = 1U;
        return 0U;
    }
    if (region != DR16_WHEEL_POSITIVE) {
        *gate = 2U;
        return 0U;
    }
    if (!jump_wheel_armed) {
        *gate = 3U;
        return 0U;
    }

    jump_wheel_armed = 0U;
    if (master_relax) {
        *gate = 4U;
        return 0U;
    }
    if (time_before(now, jump_cooldown_until_ms)) {
        *gate = 5U;
        return 0U;
    }
    if (previous_mode != CHASSIS_FOLLOW_GIMBAL &&
        previous_mode != CHASSIS_OPEN_LOOP) {
        *gate = 6U;
        return 0U;
    }
    if (chassis_fdb->stand_state != CHASSIS_IS_STAND &&
        chassis_fdb->stand_state != CHASSIS_IS_JUMPOK) {
        *gate = 7U;
        return 0U;
    }

    jump_request_pending = 1U;
    jump_request_pending_until_ms = now + DR16_JUMP_REQUEST_HOLD_MS;
    *gate = 1U;
    return 1U;
}

static uint8_t dr16_input_is_valid(const Remote_Info_Typedef *remote)
{
    if (remote == NULL) {
        return 0U;
    }
    for (uint8_t i = 0U; i < 4U; i++) {
        if (abs((int)remote->rc.ch[i]) > DR16_CHANNEL_VALID_LIMIT) {
            return 0U;
        }
    }
    if (abs((int)remote->rc.ch[4]) > DR16_WHEEL_VALID_LIMIT) {
        return 0U;
    }
    return remote->rc.s[0] >= DR16_SWITCH_UP &&
           remote->rc.s[0] <= DR16_SWITCH_MIDDLE &&
           remote->rc.s[1] >= DR16_SWITCH_UP &&
           remote->rc.s[1] <= DR16_SWITCH_MIDDLE;
}

static void weapon_reset(const Remote_Info_Typedef *remote,
                         struct shoot_cmd_msg *shoot_cmd)
{
    friction_latched = 0U;
    fire_edge_pending = 0U;
    single_finish_ms = 0U;
    jam_reverse_finish_ms = 0U;
    wheel_region_last = (int8_t)wheel_region_get(remote->rc.ch[4]);
    wheel_region_enter_ms = HAL_GetTick();

    shoot_cmd->ctrl_mode = SHOOT_STOP;
    shoot_cmd->trigger_status = TRIGGER_OFF;
    shoot_cmd->shoot_trigger_freq = 0;
    shoot_cmd->friction_status = 0U;
}

static void weapon_update(const Remote_Info_Typedef *remote,
                          const struct shoot_fdb_msg *shoot_fdb,
                          uint8_t weapon_allowed,
                          struct shoot_cmd_msg *shoot_cmd)
{
    const uint32_t now = HAL_GetTick();
    const dr16_wheel_region_e region = wheel_region_get(remote->rc.ch[4]);

    if (!weapon_allowed) {
        weapon_reset(remote, shoot_cmd);
        return;
    }

    shoot_cmd->ctrl_mode = SHOOT_STOP;
    shoot_cmd->trigger_status = TRIGGER_OFF;
    shoot_cmd->shoot_trigger_freq = 0;

    if ((int8_t)region != wheel_region_last) {
        wheel_region_enter_ms = now;
        if (region == DR16_WHEEL_POSITIVE) {
            fire_edge_pending = 1U;
        } else if (region == DR16_WHEEL_CENTER) {
            fire_edge_pending = 0U;
        }
        wheel_region_last = (int8_t)region;
    }

    shoot_cmd->friction_status = friction_latched;

    if (shoot_fdb != NULL &&
        abs((int)shoot_fdb->trigger_motor_current) >=
            DR16_JAM_CURRENT_THRESHOLD) {
        jam_reverse_finish_ms = now + DR16_JAM_REVERSE_MS;
    }

    if (time_before(now, jam_reverse_finish_ms)) {
        shoot_cmd->ctrl_mode = SHOOT_REVERSE;
        shoot_cmd->friction_status = 0U;
        fire_edge_pending = 0U;
        single_finish_ms = 0U;
        return;
    }

    if (time_before(now, single_finish_ms)) {
        shoot_cmd->ctrl_mode = SHOOT_ONE;
    }

    const uint8_t friction_ready =
        friction_latched &&
        (uint32_t)(now - friction_on_ms) >= DR16_FRICTION_WARMUP_MS;

    if (region == DR16_WHEEL_POSITIVE && friction_ready) {
        const uint32_t held_ms = now - wheel_region_enter_ms;
        if (held_ms >= DR16_WHEEL_CONTINUOUS_MS) {
            shoot_cmd->ctrl_mode = SHOOT_COUNTINUE;
            shoot_cmd->shoot_trigger_freq = DBUS_TRIGGER_SPEED_H;
            fire_edge_pending = 0U;
            single_finish_ms = 0U;
        } else if (fire_edge_pending) {
            shoot_cmd->ctrl_mode = SHOOT_ONE;
            shoot_cmd->trigger_status = TRIGGER_ON;
            fire_edge_pending = 0U;
            single_finish_ms = now + DR16_SINGLE_FINISH_MS;
        }
    }
}

void dr16_control_update(const Remote_Info_Typedef *remote,
                         const struct chassis_fdb_msg *chassis_fdb,
                         const struct gimbal_fdb_msg *gimbal_fdb,
                         const struct shoot_fdb_msg *shoot_fdb,
                         struct chassis_cmd_msg *chassis_cmd,
                         struct gimbal_cmd_msg *gimbal_cmd,
                         struct shoot_cmd_msg *shoot_cmd)
{
    if (remote == NULL || chassis_fdb == NULL || gimbal_fdb == NULL ||
        chassis_cmd == NULL || gimbal_cmd == NULL || shoot_cmd == NULL) {
        return;
    }

    const chassis_mode_e previous_chassis_mode = chassis_cmd->ctrl_mode;
    const gimbal_mode_e previous_gimbal_mode = gimbal_cmd->ctrl_mode;
    const shoot_mode_e previous_shoot_mode = shoot_cmd->ctrl_mode;
    const uint8_t master_relax =
        remote->rc_lost || !dr16_input_is_valid(remote) ||
        remote->rc.s[0] == DR16_SWITCH_UP;
    chassis_cmd->jump_wheel_raw = remote->rc.ch[4];
    /* 恢复期间消费并禁止跳跃边沿，不能在起立完成后执行恢复中积压的请求。
     * 914 的 chassis_fdb_msg 没有 recovery_active 字段，用 611 判据中的
     * previous_chassis_mode == CHASSIS_RECOVERY 分支等价替代。 */
    const uint8_t bump_switch_up = remote->rc.s[1] == DR16_SWITCH_UP;
    const uint8_t jump_controls_blocked = master_relax ||
        bump_switch_up || chassis_cmd->bump_mode != 0U ||
        previous_chassis_mode == CHASSIS_RECOVERY;
    chassis_cmd->jump_request = jump_request_update(
        remote, chassis_fdb, previous_chassis_mode, jump_controls_blocked,
        &chassis_cmd->jump_request_gate);

    /* 左拨杆上位只在进入沿产生一次请求；保持上位不会重置 BUMP 状态机。 */
    if (!bump_switch_up) {
        bump_switch_last_up = 0U;
        chassis_cmd->bump_mode = 0U;
    } else if (!bump_switch_last_up && !master_relax &&
               previous_chassis_mode != CHASSIS_RECOVERY) {
        chassis_cmd->bump_mode = 1U;
        bump_switch_last_up = 1U;
    } else if (master_relax || previous_chassis_mode == CHASSIS_RECOVERY) {
        chassis_cmd->bump_mode = 0U;
    }

#if defined(GIMBAL_RELEX)
    /* A physically disabled gimbal can never complete INIT/BACK_IS_OK.
     * Keep it relaxed instead of trapping the chassis behind gimbal health. */
    gimbal_cmd->ctrl_mode = GIMBAL_RELAX;
#else
    if (master_relax) {
        gimbal_cmd->ctrl_mode = GIMBAL_RELAX;
    } else if (previous_gimbal_mode == GIMBAL_RELAX ||
               gimbal_fdb->back_mode != BACK_IS_OK) {
        gimbal_cmd->ctrl_mode = GIMBAL_INIT;
    } else {
        gimbal_cmd->ctrl_mode = GIMBAL_GYRO;
    }
#endif

#if defined(CHASSIS_FOLLOW_GIMBAL_CLOSE) || defined(GIMBAL_RELEX)
    const chassis_mode_e requested_chassis_mode =
        remote->rc.s[0] == DR16_SWITCH_DOWN ?
            CHASSIS_SPIN : CHASSIS_OPEN_LOOP;
#else
    const chassis_mode_e requested_chassis_mode =
        remote->rc.s[0] == DR16_SWITCH_DOWN ?
            CHASSIS_SPIN : CHASSIS_FOLLOW_GIMBAL;
#endif

    if (master_relax) {
        chassis_cmd->ctrl_mode = CHASSIS_RELAX;
    } else if (previous_chassis_mode == CHASSIS_RELAX ||
               previous_chassis_mode == CHASSIS_INIT) {
        chassis_cmd->ctrl_mode = CHASSIS_RECOVERY;
    } else if (chassis_cmd->jump_request ||
               chassis_fdb->stand_state == CHASSIS_IS_JUMPING) {
        chassis_cmd->ctrl_mode = CHASSIS_JUMP;
        chassis_cmd->bump_mode = 0U;
    } else if (chassis_fdb->stand_state == CHASSIS_IS_STAND ||
               chassis_fdb->stand_state == CHASSIS_IS_JUMPOK) {
#if defined(GIMBAL_RELEX)
        /* OPEN_LOOP and SPIN are fully chassis-owned in no-gimbal builds. */
        chassis_cmd->ctrl_mode = requested_chassis_mode;
#else
        chassis_cmd->ctrl_mode =
            requested_chassis_mode == CHASSIS_OPEN_LOOP ||
            gimbal_cmd->ctrl_mode == GIMBAL_GYRO ?
                requested_chassis_mode : CHASSIS_OPEN_LOOP;
#endif
    } else {
        chassis_cmd->ctrl_mode = CHASSIS_RECOVERY;
    }

    /* 2026-10-07: BUMP 期间底盘必须**跟随云台**，无条件强制 FOLLOW_GIMBAL。
     *
     * 原来的写法只在模式是 SPIN/AUTO 时才强制 FOLLOW_GIMBAL：
     *   if (bump_mode && (mode == SPIN || mode == AUTO)) mode = FOLLOW_GIMBAL;
     * 但上面的模式阶梯只有在**云台处于 GIMBAL_GYRO** 时才会给出 FOLLOW_GIMBAL，
     * 否则一律给 CHASSIS_OPEN_LOOP。于是当云台不在 GYRO 时模式已是 OPEN_LOOP，
     * 这个 if 不成立、什么都不做 —— bump 全程跑在 OPEN_LOOP 下，底盘不跟云台，
     * 操作手转云台时车身纹丝不动。
     *
     * 现在改成只要 bump_mode 有效就强制 FOLLOW_GIMBAL，让底盘 yaw 跟随云台。
     * 跳跃请求在此期间仍不缓存（由上面的 jump_request 门控负责）。 */
    if (chassis_cmd->bump_mode) {
        chassis_cmd->ctrl_mode = CHASSIS_FOLLOW_GIMBAL;
    }

#ifdef DM_8009_SET_ZERO_POSITION
    /* Zero-position procedure build: pin the chassis in CHASSIS_INIT so
     * chassis_task re-saves the DM zeros every cycle, same semantics as
     * the pre-modularization cmd_task ladder. s0 UP still forces RELAX. */
    if (!master_relax) {
        chassis_cmd->ctrl_mode = CHASSIS_INIT;
    }
#endif

    const leg_level_e requested_leg_level =
        leg_level_get(remote->rc.s[1]);

    float vx_input = 0.0f;
    float yaw_input = 0.0f;
    if (chassis_cmd->ctrl_mode == CHASSIS_OPEN_LOOP) {
        vx_input = (float)remote->rc.ch[1];
        yaw_input = (float)remote->rc.ch[2];
    } else if (chassis_cmd->ctrl_mode == CHASSIS_FOLLOW_GIMBAL) {
        vx_input = (float)remote->rc.ch[1];
    } else if (chassis_cmd->ctrl_mode == CHASSIS_SPIN) {
        /* Historical vehicle traces show that positive chassis yaw has the
         * smaller leg split and steering load.  Spin therefore has one fixed
         * direction and deliberately ignores both chassis translation sticks. */
        vx_input = 0.0f;
        const float spin_command =
            requested_leg_level == LEG_LOW ?
                DR16_SPIN_RC_COMMAND_LOW :
                DR16_SPIN_RC_COMMAND_MID_HIGH;
        yaw_input = DR16_SPIN_DIRECTION * spin_command;
    } else if (chassis_cmd->ctrl_mode == CHASSIS_JUMP) {
        /* Preserve the longitudinal command captured on the trigger cycle.
         * Steering is held at zero during jump, but a running jump must not
         * turn into an abrupt brake command during PRELOAD/THRUST/LANDING. */
        vx_input = chassis_cmd->vx_set;
    }

    chassis_cmd->vx = vx_input;
    slope_following(&chassis_cmd->vx, &chassis_cmd->vx_set, 10.0f);
    chassis_cmd->vy = 0.0f;
    chassis_cmd->vy_set = 0.0f;
    chassis_cmd->vw_relative = yaw_input;
    slope_following(&chassis_cmd->vw_relative,
                    &chassis_cmd->vw_relative_set,
                    chassis_cmd->ctrl_mode == CHASSIS_SPIN ?
                        DR16_SPIN_RAMP_STEP : 50.0f);

    if (gimbal_cmd->ctrl_mode == GIMBAL_GYRO) {
        gimbal_cmd->vw_relative_set = (float)remote->rc.ch[2];
        gimbal_cmd->pitch_relative_set = (float)remote->rc.ch[3];
    } else {
        gimbal_cmd->vw_relative_set = 0.0f;
        gimbal_cmd->pitch_relative_set = 0.0f;
    }

    if (chassis_cmd->ctrl_mode == CHASSIS_FOLLOW_GIMBAL) {
        /* Follow the DM4310 single-turn mechanical offset.  This deliberately
         * contains no accumulated IMU yaw, so Spin cannot leave a multi-turn
         * position debt when the operator returns to Follow Gimbal. */
        chassis_cmd->vw_fllow_gimbal_temp =
            gimbal_fdb->yaw_angle;
        chassis_cmd->vw_fllow_gimbal_set =
            chassis_cmd->vw_fllow_gimbal_temp;
    } else {
        chassis_cmd->vw_fllow_gimbal_temp = 0.0f;
        chassis_cmd->vw_fllow_gimbal_set = 0.0f;
    }

    chassis_cmd->leg_level =
        (chassis_is_operator_mode(chassis_cmd->ctrl_mode) ||
         chassis_cmd->ctrl_mode == CHASSIS_JUMP) ?
            requested_leg_level : LEG_LOW;

    if (chassis_cmd->bump_mode && chassis_cmd->ctrl_mode != CHASSIS_JUMP &&
        chassis_cmd->ctrl_mode != CHASSIS_RECOVERY &&
        chassis_cmd->ctrl_mode != CHASSIS_RELAX) {
        chassis_cmd->leg_level = LEG_LOW;
    }

    const uint8_t weapon_allowed =
        !master_relax &&
        gimbal_cmd->ctrl_mode == GIMBAL_GYRO &&
        chassis_is_operator_mode(chassis_cmd->ctrl_mode);
    weapon_update(remote, shoot_fdb, weapon_allowed, shoot_cmd);

    gimbal_cmd->last_mode = previous_gimbal_mode;
    chassis_cmd->last_mode = previous_chassis_mode;
    shoot_cmd->last_mode = previous_shoot_mode;
}
