//
// Created by 14685 on 2023/2/5.
//

#include "ins_task.h"
#include "cmsis_os.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "robot.h"
#include "sysview_debug.h"
#include <math.h>


#define X 0
#define Y 1
#define Z 2

/* ------------------------------- ipc 线程间通讯相关 ------------------------------ */
MCN_DECLARE(ins_topic);
static struct ins_msg ins_data;
static uint32_t ins_publish_count;

MCN_DECLARE(gimbal_ins_topic);
static struct dm_imu_t *gimbal_imu;
/* ----------------------------- IMU_TEMPRETURE ----------------------------- */
#define IMU_TARGET_TEMP           40            /* imu期望恒温温度 */


static uint32_t pulse = 0;
static pid_obj_t *imu_temp_pid;
#ifdef BSP_USING_IMU_HEAT
static pid_config_t imu_temp_config = INIT_PID_CONFIG(160,40,0.0001, 100, 500, PID_Integral_Limit);
#else
static pid_config_t imu_temp_config = INIT_PID_CONFIG(0,0,0, 0, 0, PID_Integral_Limit);
#endif

/* ----------------------------- IMU_TEMPRETURE ----------------------------- */

/* ---------------------------- Attitude_Solving ---------------------------- */
static ins_t ins;
static imu_param_t imu_param;
/* 611 用 ins_data.valid 作为 pitch_rate 的首拍门控；914 不引入该字段，
 * 改用本地首发标志，保证第一拍不产生虚假的大角速度。 */
static uint8_t ins_pitch_rate_primed;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

static void ins_init(void);
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3]);
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt);
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll);
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q);
static void InitQuaternion(float *init_q4);

/* ---------------------------- Attitude_Solving ---------------------------- */
static float ins_dt;
static float ins_start;
static uint32_t ins_dwt = 0;
static float dt = 0;
static uint32_t count = 0;
const float gravity[3] = {0, 0, 9.81f};
static uint32_t ins_wake_time;

void ins_task_init(){
    /* USER CODE BEGIN InsTask */
    imu_temp_pid = pid_register(&imu_temp_config);   /* 注册 PID 实例 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    BMI088_Read(&BMI088);
    ins_init();
    gimbal_imu = dm_imu_init(DM_IMU_TX_ID,DM_IMU_RX_ID,&hfdcan2);/*位于云台的dm_imu*/
    dt = dwt_get_delta(&ins_dwt);
    ins_start = dwt_get_time_ms();

    ins_wake_time = osKernelSysTick();
    LOGINFO("[freeRTOS] Ins Task Start\r\n");

}
void ins_control()
{

/* ------------------------------ 调试监测线程调度 ------------------------------ */
    ins_dt = dwt_get_time_ms() - ins_start;
    ins_start = dwt_get_time_ms();
    if(ins_dt > 1)LOGERROR("ERROR:[freeRTOS] Ins Task Delay\r\n");
/* ------------------------------ 调试监测线程调度 ------------------------------ */

    dt = dwt_get_delta(&ins_dwt);
    const uint32_t t_start_us = dwt_get_time_us();
    BMI088_Read(&BMI088);
    const uint32_t t_after_spi_us = dwt_get_time_us();
    ins.accel[0] = BMI088.accel[0];
    ins.accel[1] = BMI088.accel[1];
    ins.accel[2] = BMI088.accel[2];
    ins.gyro[0] = BMI088.gyro[0];
    ins.gyro[1] = BMI088.gyro[1];
    ins.gyro[2] = BMI088.gyro[2];
    IMU_Param_Correction(&imu_param, ins.gyro, ins.accel);
    // 核心函数,EKF更新四元数
    IMU_QuaternionEKF_Update(ins.gyro[X], ins.gyro[Y], ins.gyro[Z], ins.accel[X], ins.accel[Y], ins.accel[Z], dt);
    const uint32_t t_after_ekf_us = dwt_get_time_us();

    memcpy(ins.q, QEKF_INS.q, sizeof(QEKF_INS.q));

    // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
    BodyFrameToEarthFrame(xb, ins.xn, ins.q);
    BodyFrameToEarthFrame(yb, ins.yn, ins.q);
    BodyFrameToEarthFrame(zb, ins.zn, ins.q);

    // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, ins.q);
    for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
    {
        ins.motion_accel_b[i] = (ins.accel[i] - gravity_b[i]) * dt / (ins.accel_lpf + dt) + ins.motion_accel_b[i] * ins.accel_lpf / (ins.accel_lpf + dt);
    }
    BodyFrameToEarthFrame(ins.motion_accel_b, ins.motion_accel_n, ins.q); // 转换回导航系n

    ins.yaw = QEKF_INS.Yaw;
    ins.pitch = QEKF_INS.Pitch;
    ins.roll = QEKF_INS.Roll;
    ins.yaw_total_angle = QEKF_INS.YawTotalAngle;

    /* 已发布 pitch 的欧拉角速度，rad/s；底盘 BUMP 的落地轮端阻尼需要它。
     * 公式与 611 一致：ins.roll 是当前 EKF Roll，而 ins_data.pitch 也映射自
     * EKF Roll（见下方发布段），所以差值就是已发布 pitch 的增量。
     * 此处必须写在发布段之前，才能拿到上一拍的 ins_data.pitch。 */
    if (isfinite(dt) && dt > 0.0f && dt <= 0.020f && ins_pitch_rate_primed &&
        isfinite(ins.roll) && isfinite(ins_data.pitch)) {
        ins_data.pitch_rate_rad_s =
            remainderf(ins.roll - ins_data.pitch, 360.0f) * DEGREE_2_RAD / dt;
    } else {
        ins_data.pitch_rate_rad_s = 0.0f;
    }
    if (!isfinite(ins_data.pitch_rate_rad_s)) {
        ins_data.pitch_rate_rad_s = 0.0f;
    }
    ins_pitch_rate_primed = 1U;

    {/* publish msg */
        // FIXME:/* 根据陀螺仪安装情况进行调整 */
        // NOTE: yaw轴右为正，pitch轴上为正，roll轴顺时针为正
        ins_data.yaw = -ins.yaw;
        ins_data.roll = - ins.pitch;
        ins_data.yaw_total_angle = -ins.yaw_total_angle;
        ins_data.pitch =  ins.roll;
        ins_data.gyro[0] =-ins.gyro[0];
        ins_data.gyro[1] = ins.gyro[1];
        ins_data.gyro[2] =-ins.gyro[2];
        ins_data.accel[0] = ins.accel[0];
        ins_data.accel[1] = ins.accel[1];
        ins_data.accel[2] = ins.accel[2];
        ins_data.motion_accel_b[0] = ins.motion_accel_b[0];
        ins_data.motion_accel_b[1] = ins.motion_accel_b[1];
        ins_data.motion_accel_b[2] = ins.motion_accel_b[2];
        ins_data.publish_tick_ms = HAL_GetTick();
        ins_data.publish_count = ++ins_publish_count;
        mcn_publish(MCN_HUB(ins_topic), &ins_data);
        mcn_publish(MCN_HUB(gimbal_ins_topic), gimbal_imu);
    }
    // temperature control
    if ((count % 2) == 0)
    {
        pulse = pid_calculate(imu_temp_pid, BMI088.temperature, IMU_TARGET_TEMP);
        TIM_Set_PWM(&htim3, TIM_CHANNEL_4, pulse);
    }

    /* 125 Hz 分段计时: SystemView Data Plot, 定位 1kHz INS 任务耗时构成 */
    if ((count & 7U) == 0U)
    {
        const uint32_t t_end_us = dwt_get_time_us();
        /* IDs 8-10: keep clear of chassis debug plot IDs 0-5. */
        SYSVIEW_PLOT_FLOAT(8, "ins_spi_us", (float)(t_after_spi_us - t_start_us));
        SYSVIEW_PLOT_FLOAT(9, "ins_ekf_us", (float)(t_after_ekf_us - t_after_spi_us));
        SYSVIEW_PLOT_FLOAT(10, "ins_total_us", (float)(t_end_us - t_start_us));
    }

    count++;
    vTaskDelayUntil(&ins_wake_time, 1);
}
static uint8_t imu_flag=0;
void dm_imu_control(){
    if(imu_flag == 3)imu_flag=0;
    switch (imu_flag) {
        case 0:
            imu_request_accel();
            break;
        case 1:
            imu_request_gyro();
            break;
        case 2:
            imu_request_euler();
            break;
    }
    imu_flag++;

}
void ins_control_task(){
    ins_control();
    dm_imu_control();
}
/**
 * @brief 初始化 ins 解算系统
 *
 */
static void ins_init(void)
{
    imu_param.scale[X] = 1;
    imu_param.scale[Y] = 1;
    imu_param.scale[Z] = 1;
    imu_param.yaw = 0;
    imu_param.pitch = 0;
    imu_param.roll = 0;
    imu_param.flag = 1;

    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);

    // noise of accel is relatively big and of high freq,thus lpf is used
    ins.accel_lpf = 0.0085;
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_sum[3] = {0.0f, 0.0f, 0.0f};
    float acc_unit[3];
    float q_candidate[4];
    uint16_t valid_samples = 0U;

    /* A finite identity quaternion is the deterministic fallback for a
     * degenerate startup sample. Never let an invalid seed poison the EKF. */
    init_q4[0] = 1.0f;
    init_q4[1] = 0.0f;
    init_q4[2] = 0.0f;
    init_q4[3] = 0.0f;

    /* Average all valid startup samples. The former code overwrote each
     * sample and wrote accel Z outside the three-element array. */
    for (uint16_t i = 0U; i < 100U; ++i)
    {
        BMI088_Read(&BMI088);
        if (isfinite(BMI088.accel[0]) &&
            isfinite(BMI088.accel[1]) &&
            isfinite(BMI088.accel[2])) {
            acc_sum[0] += BMI088.accel[0];
            acc_sum[1] += BMI088.accel[1];
            acc_sum[2] += BMI088.accel[2];
            valid_samples++;
        }
        dwt_delay_s(0.001);
    }

    if (valid_samples == 0U) {
        return;
    }

    const float sample_scale = 1.0f / (float)valid_samples;
    acc_unit[0] = acc_sum[0] * sample_scale;
    acc_unit[1] = acc_sum[1] * sample_scale;
    acc_unit[2] = acc_sum[2] * sample_scale;

    const float acc_norm_sq = acc_unit[0] * acc_unit[0] +
                              acc_unit[1] * acc_unit[1] +
                              acc_unit[2] * acc_unit[2];
    if (!isfinite(acc_norm_sq) || acc_norm_sq <= 1.0e-6f) {
        return;
    }

    const float acc_inv_norm = 1.0f / sqrtf(acc_norm_sq);
    acc_unit[0] *= acc_inv_norm;
    acc_unit[1] *= acc_inv_norm;
    acc_unit[2] *= acc_inv_norm;

    /* Quaternion rotating measured gravity onto navigation +Z:
     * q = normalize([1 + dot(a,b), cross(a,b)]), b=(0,0,1). */
    float gravity_dot = acc_unit[2];
    if (gravity_dot > 1.0f) {
        gravity_dot = 1.0f;
    } else if (gravity_dot < -1.0f) {
        gravity_dot = -1.0f;
    }

    if (gravity_dot < -0.9999f) {
        /* At 180 degrees the axis is underdetermined. Select body X;
         * yaw is intentionally unobservable from gravity alone. */
        init_q4[0] = 0.0f;
        init_q4[1] = 1.0f;
        return;
    }

    q_candidate[0] = 1.0f + gravity_dot;
    q_candidate[1] = acc_unit[1];
    q_candidate[2] = -acc_unit[0];
    q_candidate[3] = 0.0f;

    const float q_norm_sq = q_candidate[0] * q_candidate[0] +
                            q_candidate[1] * q_candidate[1] +
                            q_candidate[2] * q_candidate[2] +
                            q_candidate[3] * q_candidate[3];
    if (!isfinite(q_norm_sq) || q_norm_sq <= 1.0e-8f) {
        return;
    }

    const float q_inv_norm = 1.0f / sqrtf(q_norm_sq);
    for (uint8_t i = 0U; i < 4U; ++i) {
        init_q4[i] = q_candidate[i] * q_inv_norm;
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
static void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
static void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(imu_param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->yaw - lastYawOffset) > 0.001f ||
        fabsf(param->pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; ++i)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->yaw;
    lastPitchOffset = param->pitch;
    lastRollOffset = param->roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief 四元数更新函数,即实现dq/dt=0.5Ωq
 *
 * @param q  四元数
 * @param gx
 * @param gy
 * @param gz
 * @param dt 距离上次调用的时间间隔
 */
static void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
static void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
static void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}
