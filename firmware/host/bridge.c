/* Hardware/scheduler boundary only. Controller, motor drivers and parameters
 * are compiled byte-for-byte from firmware/vehicle; no control law lives here. */
#include "bridge.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "chassis_task.h"
#include "control/chassis_control.h"
#include "recovery/chassis_recovery.h"
#include "safety/chassis_safety.h"
#include "debug/chassis_debug.h"
#include "gimbal_task.h"
#include "dr16_control.h"
#include "vofa_plus.h"
#include "ins_task.h"
#include <math.h>
#include <string.h>

FDCAN_HandleTypeDef hfdcan1, hfdcan2, hfdcan3;
UART_HandleTypeDef huart10;
TIM_HandleTypeDef htim3;
ImuDataTypeDef BMI088;
MCN_DEFINE(ins_topic, sizeof(struct ins_msg));
MCN_DEFINE(chassis_cmd, sizeof(struct chassis_cmd_msg));
MCN_DEFINE(chassis_fdb, sizeof(struct chassis_fdb_msg));
MCN_DEFINE(gimbal_cmd, sizeof(struct gimbal_cmd_msg));
MCN_DEFINE(gimbal_fdb, sizeof(struct gimbal_fdb_msg));
MCN_DEFINE(gimbal_ins_topic, sizeof(struct dm_imu_t));
MCN_DEFINE(transmission_fdb, sizeof(struct trans_fdb_msg));
static uint64_t clock_us;
static struct chassis_cmd_msg command;
static struct gimbal_cmd_msg gimbal_command;
static struct chassis_fdb_msg feedback;
static struct gimbal_fdb_msg gimbal_feedback;
static struct ins_msg ins_feedback;
static struct dm_imu_t yaw_feedback;
static struct shoot_cmd_msg shoot_command;
static struct shoot_fdb_msg shoot_feedback;
static lab_output_t held_output;
static float debug_values[64];
static uint64_t next_dm_us = LAB_DM_POLL_US;
static const lab_feedback_t *current_feedback;
static int ins_initialized;

uint32_t HAL_GetTick(void) { return (uint32_t)(clock_us / 1000); }
uint32_t osKernelSysTick(void) { return HAL_GetTick(); }
float dwt_get_time_ms(void) { return (float)clock_us * .001f; }
uint32_t dwt_get_time_us(void) { return (uint32_t)clock_us; }
float dwt_get_delta(uint32_t *last) {
    uint32_t now = (uint32_t)(clock_us * (LAB_CPU_HZ / 1000000));
    float dt = (float)(uint32_t)(now - *last) / (float)LAB_CPU_HZ;
    *last = now;
    return dt;
}
int Vofa_Transmit(Vofa_Instance_t *v, UART_HandleTypeDef *u) {
    (void)u; memcpy(debug_values, v->values, sizeof(debug_values)); return 0;
}
static float decode(unsigned n, float lo, float hi, unsigned bits) {
    return (float)n * (hi - lo) / (float)((1U << bits) - 1) + lo;
}
static unsigned encode(float x, float lo, float hi, unsigned bits) {
    x = fminf(hi, fmaxf(lo, x));
    return (unsigned)((x - lo) * (float)((1U << bits) - 1) / (hi - lo));
}
void BMI088_Read(ImuDataTypeDef *sample) {
    if (current_feedback) {
        memcpy(sample->gyro, current_feedback->gyro, sizeof(sample->gyro));
        memcpy(sample->accel, current_feedback->accel, sizeof(sample->accel));
    }
    sample->temperature = 40.0f;
}
static void imu_response(unsigned kind) {
    const lab_feedback_t *f = current_feedback;
    if (!f || !(f->feedback_mask & 256)) return;
    uint8_t packet[8] = {kind, 0};
    float values[3] = {0,0,0};
    float lo = ACCEL_CAN_MIN, hi = ACCEL_CAN_MAX;
    if (kind == GYRO_DATA) {
        values[2] = f->gimbal_yaw_rate; lo = GYRO_CAN_MIN; hi = GYRO_CAN_MAX;
    } else if (kind == EULER_DATA) {
        /* The requested scope fixes pitch at centre and simulates yaw only. */
        unsigned p = encode(0, PITCH_CAN_MIN, PITCH_CAN_MAX, 16);
        unsigned y = encode(remainderf(f->gimbal_yaw_deg, 360), YAW_CAN_MIN, YAW_CAN_MAX, 16);
        unsigned r = encode(0, ROLL_CAN_MIN, ROLL_CAN_MAX, 16);
        packet[2]=p; packet[3]=p>>8; packet[4]=y; packet[5]=y>>8; packet[6]=r; packet[7]=r>>8;
        dm_imu_rx_callback(DM_IMU_RX_ID, packet); return;
    }
    for (unsigned i=0; i<3; ++i) {
        unsigned n = encode(values[i], lo, hi, 16);
        packet[2+2*i] = n; packet[3+2*i] = n>>8;
    }
    dm_imu_rx_callback(DM_IMU_RX_ID, packet);
}
void CAN_send(FDCAN_HandleTypeDef *bus, uint32_t id, uint8_t *data) {
    if (bus == &hfdcan2 && id == DM_IMU_TX_ID && data[0] == 0xCC) {
        imu_response(data[1]); return;
    }
    if (id >= 1 && id <= 6 && id != 5) {
        static const int slot[7] = {-1, 0, 1, 2, 3, -1, 4};
        held_output.motor_tx_count[slot[id]]++;
        float *out = slot[id] == 4 ? held_output.yaw_command : held_output.hip_command[slot[id]];
        if (data[0] == 255 && data[1] == 255 && data[2] == 255 &&
            data[3] == 255 && data[4] == 255 && data[5] == 255 && data[6] == 255) {
            /* Mode packets carry no new MIT command. Disable clears the hold. */
            if (data[7] != DM_CMD_MOTOR_MODE) memset(out, 0, 5 * sizeof(float));
            return;
        }
        out[0] = decode((data[0] << 8) | data[1], DM_P_MIN, DM_P_MAX, 16);
        out[1] = decode((data[2] << 4) | (data[3] >> 4), DM_V_MIN, DM_V_MAX, 12);
        out[2] = decode(((data[3] & 15) << 8) | data[4], DM_KP_MIN, DM_KP_MAX, 12);
        out[3] = decode((data[5] << 4) | (data[6] >> 4), DM_KD_MIN, DM_KD_MAX, 12);
        out[4] = decode(((data[6] & 15) << 8) | data[7], DM_T_MIN, DM_T_MAX, 12);
    } else if (id == 0x200) {
        for (unsigned i = 0; i < 2; ++i)
            held_output.wheel_current[i] = (int16_t)((data[2*i] << 8) | data[2*i+1]);
    }
}
static void hip_receive(unsigned id, float p, float v, float t) {
    /* angle_abs is mod 2pi in the original driver. Encode in its wire domain. */
    p = fmodf(p + PI2, PI2);
    if (p > PI) p -= PI2;
    unsigned a = encode(p, DM_P_MIN, DM_P_MAX, 16);
    unsigned b = encode(v, DM_V_MIN, DM_V_MAX, 12);
    unsigned c = encode(t, DM_T_MIN, DM_T_MAX, 12);
    uint8_t packet[8] = {id, a >> 8, a, b >> 4, ((b & 15) << 4) | (c >> 8), c, 25, 25};
    dm_motor_rx_callback(CHASSIS_JOINT_AND_YAW_RX_ID, packet);
}
static void wheel_receive(unsigned side, float p, float v, float t) {
    const float ratio = side ? M3508_READUCTION_RATIO_R : M3508_READUCTION_RATIO_L;
    float rotor = fmodf(p * ratio, PI2); if (rotor < 0) rotor += PI2;
    unsigned a = (unsigned)(rotor / PI2 * 8192) & 8191;
    int16_t b = (int16_t)fmaxf(-32768, fminf(32767, v * ratio / RPM_2_RAD_PER_SEC));
    int16_t c = (int16_t)fmaxf(-32768, fminf(32767, t * M3508_TOR_TO_CUR));
    uint8_t packet[8] = {a >> 8, a, b >> 8, b, c >> 8, c, 25, 0};
    dji_motot_rx_callback(side ? CHASSIS_WHEEL_RIGHT_ID : CHASSIS_WHEEL_LEFT_ID, packet);
}
void lab_init(void) {
    mcn_init();
    McnHub_t hubs[] = {MCN_HUB(ins_topic), MCN_HUB(chassis_cmd), MCN_HUB(chassis_fdb),
        MCN_HUB(gimbal_cmd), MCN_HUB(gimbal_fdb), MCN_HUB(gimbal_ins_topic), MCN_HUB(transmission_fdb)};
    for (unsigned i = 0; i < sizeof(hubs)/sizeof(hubs[0]); ++i) mcn_advertise(hubs[i], NULL);
    chassis_task_init();
    gimbal_task_init();
    command.ctrl_mode = CHASSIS_RELAX;
    gimbal_command.ctrl_mode = GIMBAL_RELAX;
    mcn_publish(MCN_HUB(chassis_cmd), &command);
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_command);
}
void lab_debug(uint32_t page, float output[64]) {
    chassis_emit_debug_page((uint8_t)page);
    memcpy(output, debug_values, sizeof(debug_values));
}
void lab_step(const lab_feedback_t *f, const lab_command_t *c, lab_output_t *out) {
    current_feedback = f;
    clock_us = f->time_us;
    static const unsigned hip_ids[4] = {1,2,3,4};
    for (unsigned i=0; i<4; ++i) if (f->feedback_mask & (1U << i))
        hip_receive(hip_ids[i], f->hip_angle[i], f->hip_speed[i], f->hip_torque[i]);
    for (unsigned i=0; i<2; ++i) if (f->feedback_mask & (16U << i))
        wheel_receive(i, f->wheel_angle[i], f->wheel_speed[i], f->wheel_torque[i]);
    if (f->raw_imu && (f->feedback_mask & 64)) {
        if (!ins_initialized) {
            /* Begin after hardware startup; 100 seed reads see the initial
             * stationary sample. The simulated control clock starts here. */
            clock_us -= LAB_CONTROL_TICK_US;
            ins_task_init(); ins_initialized = 1;
            clock_us = f->time_us;
        }
        ins_control_task();
        mcn_copy_from_hub(MCN_HUB(ins_topic), &ins_feedback);
    } else if (f->feedback_mask & 64) {
        ins_feedback.publish_tick_ms = HAL_GetTick(); ins_feedback.publish_count++;
        memcpy(ins_feedback.gyro, f->gyro, sizeof(f->gyro));
        memcpy(ins_feedback.accel, f->accel, sizeof(f->accel));
        memcpy(ins_feedback.motion_accel_b, f->motion_accel, sizeof(f->motion_accel));
        ins_feedback.roll = f->roll_deg; ins_feedback.pitch = f->pitch_deg;
        ins_feedback.yaw = f->yaw_deg; ins_feedback.yaw_total_angle = f->yaw_deg;
        ins_feedback.pitch_rate_rad_s = f->pitch_rate;
        mcn_publish(MCN_HUB(ins_topic), &ins_feedback);
    }
    if (f->feedback_mask & 128)
        hip_receive(YAW_MOTOR_ID, f->yaw_motor_angle, f->yaw_motor_speed, f->yaw_motor_torque);
    if (!f->raw_imu && (f->feedback_mask & 256)) {
        yaw_feedback.yaw = f->gimbal_yaw_deg; yaw_feedback.yaw_total_angle = f->gimbal_yaw_deg;
        yaw_feedback.gyro[2] = f->gimbal_yaw_rate;
        yaw_feedback.rx_count++; yaw_feedback.last_rx_ms = HAL_GetTick();
        yaw_feedback.last_euler_rx_ms = yaw_feedback.last_gyro_rx_ms = HAL_GetTick();
        /* Pitch is held at centre: this model intentionally has only yaw. */
        mcn_publish(MCN_HUB(gimbal_ins_topic), &yaw_feedback);
    }
    /* Nominal equal-priority dispatch follows rm_task.h creation order:
     * INS, motor, chassis, cmd, gimbal. A newly received command reaches the
     * chassis next tick; DJI sends the previous chassis result. Actual IRQ
     * phase and RTOS jitter remain measurable hardware-boundary parameters. */
    dji_motor_control();
    chassis_control_task();
    mcn_copy_from_hub(MCN_HUB(chassis_fdb), &feedback);
    mcn_copy_from_hub(MCN_HUB(gimbal_fdb), &gimbal_feedback);
    if (c->use_remote) {
        Remote_Info_Typedef remote = {0};
        for (unsigned i=0; i<5; ++i) remote.rc.ch[i] = c->rc_channels[i];
        remote.rc.s[0] = c->rc_switches[0]; remote.rc.s[1] = c->rc_switches[1]; remote.rc_lost = c->rc_lost;
        dr16_control_update(&remote, &feedback, &gimbal_feedback, &shoot_feedback,
                            &command, &gimbal_command, &shoot_command);
    } else {
        command.last_mode = command.ctrl_mode; gimbal_command.last_mode = gimbal_command.ctrl_mode;
        command.ctrl_mode = c->chassis_mode; command.leg_level = c->leg_level;
        command.vx = command.vx_set = c->vx_raw;
        command.vw_relative = command.vw_relative_set = c->yaw_raw;
        command.jump_request = c->jump_request; command.bump_mode = c->bump_request;
        gimbal_command.ctrl_mode = c->gimbal_mode;
        gimbal_command.vw_relative_set = c->gimbal_yaw_raw; gimbal_command.yaw_auto_set = c->gimbal_auto_deg;
    }
    command.vw_fllow_gimbal_set = gimbal_feedback.yaw_angle;
    mcn_publish(MCN_HUB(chassis_cmd), &command);
    mcn_publish(MCN_HUB(gimbal_cmd), &gimbal_command);
    gimbal_control_task();
    /* Original DJI control task is 1 ms. Original TIM4 polls ONE DM every 2 ms,
     * in registration order LB, LF, RB, RF, yaw. Its held commands are retained. */
    while (next_dm_us <= clock_us) { dm_controll_all_poll(); next_dm_us += LAB_DM_POLL_US; }
    mcn_copy_from_hub(MCN_HUB(chassis_fdb), &feedback);
    mcn_copy_from_hub(MCN_HUB(gimbal_fdb), &gimbal_feedback);
    for (unsigned s=0; s<2; ++s) {
        leg_obj_t *leg = chassis_control_module_leg(s);
        held_output.length[s] = leg->l0; held_output.phi0[s] = leg->phi0;
        held_output.theta[s] = leg->theta; held_output.support[s] = leg->support_force;
        memcpy(held_output.vmc + 2*s, chassis_control_module_vmc_output(s), 2*sizeof(float));
    }
    memcpy(held_output.lqr, chassis_control_module_lqr_output(), sizeof(held_output.lqr));
    memcpy(held_output.obs, chassis_control_module_lqr_x_obs(), sizeof(held_output.obs));
    held_output.stand_state = feedback.stand_state;
    held_output.jump_state = chassis_control_module_jump_state(); held_output.jump_fault = chassis_control_module_jump_fault();
    held_output.recovery_state = chassis_recovery_module_debug_state();
    held_output.recovery_fault = chassis_recovery_module_debug_fault_reason();
    held_output.off_ground = chassis_safety_module_off_ground();
    held_output.landing_phase = chassis_safety_module_landing_buffer_phase();
    held_output.gimbal_ready = gimbal_feedback.back_mode;
    held_output.yaw_relative = gimbal_feedback.yaw_angle;
    held_output.yaw_target = gimbal_feedback.gimbal_yaw_refer;
    held_output.imu[0]=ins_feedback.roll; held_output.imu[1]=ins_feedback.pitch; held_output.imu[2]=ins_feedback.yaw_total_angle;
    memcpy(held_output.imu+3, ins_feedback.gyro, 3*sizeof(float));
    memcpy(held_output.imu+6, ins_feedback.motion_accel_b, 3*sizeof(float));
    lab_debug(54, debug_values);
    held_output.chassis_mode = (int32_t)debug_values[1];
    held_output.bump_state = (int32_t)debug_values[2];
    *out = held_output;
}
void lab_parameters(float output[12]) {
    output[0]=WHEEL_RADIUS; output[1]=M3508_TOR_TO_CUR; output[2]=M3508_TOR_MAX;
    output[3]=DM_P_MIN; output[4]=DM_P_MAX; output[5]=DM_V_MIN; output[6]=DM_V_MAX;
    output[7]=DM_T_MIN; output[8]=DM_T_MAX;
    output[9]=chassis_control_module_leg(0)->F_Spring;
    output[10]=chassis_control_module_leg(1)->F_Spring; output[11]=CHASSIS_SUPPORT_MASS;
}
uint32_t lab_abi_size(uint32_t index) {
    return index == 0 ? sizeof(lab_feedback_t) : index == 1 ? sizeof(lab_command_t) : sizeof(lab_output_t);
}
