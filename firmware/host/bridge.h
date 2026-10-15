#ifndef LAB_BRIDGE_H
#define LAB_BRIDGE_H
#include <stdint.h>
/* ABI uses fixed-width scalars. Firmware enums/objects never cross Python. */
typedef struct {
    uint64_t time_us;
    float hip_angle[4], hip_speed[4], hip_torque[4]; /* LB, RB, RF, LF; DM encoder rad */
    float wheel_angle[2], wheel_speed[2], wheel_torque[2]; /* output-shaft rad */
    float gyro[3], accel[3], motion_accel[3]; /* published chassis INS frame */
    float roll_deg, pitch_deg, yaw_deg, pitch_rate;
    float yaw_motor_angle, yaw_motor_speed, yaw_motor_torque;
    float gimbal_yaw_deg, gimbal_yaw_rate;
    uint32_t feedback_mask; /* bits 0..3 hips, 4..5 wheels, 6 INS, 7 yaw, 8 yaw IMU */
    int32_t raw_imu; /* 1: gyro/accel are corrected BMI088 samples; run original INS/EKF */
} lab_feedback_t;
typedef struct {
    int32_t use_remote;
    int32_t chassis_mode, leg_level, gimbal_mode;
    float vx_raw, yaw_raw, gimbal_yaw_raw, gimbal_auto_deg;
    int32_t jump_request, bump_request;
    int32_t rc_channels[5], rc_switches[2], rc_lost;
} lab_command_t;
typedef struct {
    float hip_command[4][5]; /* MIT p,v,kp,kd,t, after driver packet quantization */
    float wheel_current[2], yaw_command[5];
    float length[2], phi0[2], theta[2], support[2], vmc[4], lqr[4], obs[12];
    float yaw_relative, yaw_target;
    float imu[9]; /* published roll,pitch,yaw, gyro xyz, motion_accel xyz */
    uint32_t motor_tx_count[5]; /* LB,RB,RF,LF,yaw packets, including mode packets */
    int32_t chassis_mode, stand_state, jump_state, jump_fault;
    int32_t recovery_state, recovery_fault, off_ground, landing_phase, gimbal_ready, bump_state;
} lab_output_t;
void lab_init(void);
void lab_step(const lab_feedback_t *, const lab_command_t *, lab_output_t *);
void lab_debug(uint32_t page, float output[64]);
uint32_t lab_abi_size(uint32_t index);
void lab_parameters(float output[12]);
#endif
