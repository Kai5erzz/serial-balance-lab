
/*
* Change Logs:
* Date            Author          Notes
* 25-8-22         Gleam            1.0
* 25-8-22         SuperChen        2.0
*/
#include <math.h>
#include <stdlib.h>
#include "gimbal_task.h"
#include "rm_config.h"
#include "rm_module.h"
#include "rm_algorithm.h"
#include "../chassis/debug/chassis_debug.h"
#define GIM_PITCH_MOTOR_NUM 1
#define GIM_YAW_MOTOR_NUM 1
#define GIM_MOTOR_NUM 2
/* ----------------------------------------------- �̼߳�ͨѶ������� ---------------------------------------------------- */

// ����
MCN_DECLARE(ins_topic);
static McnNode_t ins_topic_node;
static struct ins_msg ins;
MCN_DECLARE(chassis_cmd);
static McnNode_t chassis_cmd_node;
static struct chassis_cmd_msg chassis_fdb_cmd;
MCN_DECLARE(gimbal_cmd);
static McnNode_t gimbal_cmd_node;
static struct gimbal_cmd_msg gim_cmd;
MCN_DECLARE(gimbal_ins_topic);
static McnNode_t gimbal_ins_node;
static struct dm_imu_t gim_ins;
MCN_DECLARE(transmission_fdb);
static McnNode_t trans_fdb_node;
static struct trans_fdb_msg trans_fdb;

// ����
MCN_DECLARE(gimbal_fdb);
struct gimbal_fdb_msg gimbal_fdb_data;

static void gimbal_pub_push(void);
static void gimbal_sub_init(void);
static void gimbal_sub_pull(void);

static dm_motor_para_t dm_yaw_control(dm_motor_measure_t measure);//dm�������
static int16_t motor_control_pitch(dji_motor_measure_t measure);//6020�������


/*
 * @brief ���ô��������
 * */
static void leg_init_get_zero();
/* ------------------------------------------------- ���������� ------------------------------------------------------ */


static float dm_yaw_send_t[GIM_YAW_MOTOR_NUM];
static float dm_yaw_obs[1];
static float yaw_recovery_position;//mit���ٶ�λ��ģʽʹ��,�ڵ�������ǰgimbal����ת����ǰ��������,��ֹ�Ȳ��ڶ�ʱײ����̨
static float yaw_recovery_velocity;
#define DM_GIMBAL_OUTPUT_LIMIT 3.5f
#define DM_YAW_RATIO 1.0f
#define DM_YAW_MIT_KP 0.06f
#define DM_YAW_MIT_KD 3.5f

static struct gimbal_controller_t{
    /* Mechanical centering controller. */
    pid_obj_t *pid_speed_init;
    pid_obj_t *pid_angle_init;
    /* ����imu���ݱջ�����Ҫ�����ֶ�ģʽ */
    pid_obj_t *pid_speed_imu;
    pid_obj_t *pid_angle_imu;
    /* ����imu���ݱջ�����Ҫ�����Զ�ģʽ */
    pid_obj_t *pid_speed_auto;
    pid_obj_t *pid_angle_auto;
}gim_controller[GIM_MOTOR_NUM];

motor_config_t gimbal_motor_config[GIM_MOTOR_NUM] = {
        {
                .motor_type = DM4310,
                .can_id = CAN_ID_CHASSIS_MOTOR,
                .tx_id = YAW_MOTOR_ID,
                .rx_id = CHASSIS_JOINT_AND_YAW_RX_ID,
                .controller = &gim_controller[YAW],
        },

        {
                .motor_type = GM6020,   //Ӣ��pitch�����˿�˽ṹ������3508���
                .can_id = CAN_ID_GIMBAL_MOTOR,
                .rx_id = PITCH_MOTOR_ID,   //���ID����
                .controller = &gim_controller[PITCH],
        }
};


/* ------------------------------------------------ ��̨������� ----------------------------------------------------- */
/* gyro���᣺[0]ΪX��[1]ΪY��[2]ΪZ */
#define X 0
#define Y 1
#define Z 2

/* Full-stick operator rates.  Targets are integrated with measured task dt so
 * scheduler jitter does not alter the manual sensitivity. */
#define GIMBAL_YAW_MANUAL_RATE_MAX_RADPS    5.3f
#define GIMBAL_PITCH_MANUAL_RATE_MAX_RADPS  1.6f
#define GIMBAL_YAW_TARGET_LEAD_LIMIT_RAD     1.10f
#define GIMBAL_STICK_DEADBAND_NORMALIZED     0.02f

static dji_motor_object_t *gim_motor_pitch[GIM_PITCH_MOTOR_NUM];  // ���̵��ʵ��
static dm_motor_object_t *gim_motor_yaw[GIM_YAW_MOTOR_NUM];  // ���̵��ʵ��
static float gim_motor_ref[GIM_MOTOR_NUM]; // �����������ֵ

/* Debug mirrors only: populated by the existing controllers after their
 * calculations and never fed back into control. */
static float dbg_pitch_rate_ref;
static float dbg_pitch_current_cmd;
static float dbg_pitch_speed_fdb;
static float dbg_pitch_speed_iout;
static float dbg_pitch_angle_iout;
static float dbg_pitch_gravity_ff;
static float dbg_yaw_rate_ref;
static float dbg_yaw_speed_fdb;
static float dbg_yaw_torque_cmd;
static float dbg_yaw_angle_fdb;   /* yaw 环内实际反馈角（与 dm_yaw_control 所用一致） */
static float dbg_yaw_speed_iout;  /* yaw 速度环积分项 */
static float dbg_yaw_angle_iout;  /* yaw 角度环积分项 */

/* pid_clear() only clears arithmetic state.  Refresh the DWT timestamp too,
 * otherwise the first active sample integrates the entire RELAX interval. */
static void gimbal_pid_reset(pid_obj_t *pid)
{
    if (pid == NULL) {
        return;
    }
    pid_clear(pid);
    (void)dwt_get_delta(&pid->DWT_CNT);
    pid->dt = 0.0f;
}

/* DM4310 single-turn mechanical yaw in the chassis-relative frame.  Follow
 * gimbal must use this wrapped angle, never either IMU's accumulated yaw. */
static float gimbal_relative_yaw_angle_get(void)
{
    if (gim_motor_yaw[0] == NULL) {
        return 0.0f;
    }

    const dm_motor_measure_t *measure = &gim_motor_yaw[0]->measure;
    if (measure->rx_count == 0U ||
        (uint32_t)(HAL_GetTick() - measure->last_rx_ms) > 50U) {
        return 0.0f;
    }
    float period = measure->yaw_angle_round;
    if (!isfinite(period) || period < 0.1f ||
        !isfinite(measure->yaw_angle)) {
        return 0.0f;
    }

    float motor_angle = fmodf(measure->yaw_angle, period);
    if (motor_angle > 0.5f * period) {
        motor_angle -= period;
    } else if (motor_angle < -0.5f * period) {
        motor_angle += period;
    }

    float relative_yaw =
        GIMBAL_YAW_DRIVE_SIGN * motor_angle / GEAR_RATIO -
        CENTER_YAW_ANGLE;
    while (relative_yaw > PI) {
        relative_yaw -= PI2;
    }
    while (relative_yaw < -PI) {
        relative_yaw += PI2;
    }
    return relative_yaw;
}

static void gimbal_motor_init();

/* ------------------------------------------------ ��̨�߳���� ----------------------------------------------------- */

void gimbal_task_init(void){
    gimbal_sub_init();
    gimbal_motor_init();

}

static float gimbal_start;
static float gimbal_dt;
static uint32_t init_start_time; // ��̨��ʼ�����п�ʼʱ�䣬���ⳤʱ����Ϊ��̬�����ڹ���ģʽ
static uint32_t init_dt; // ��̨��ʼ�����н���ʱ��
static uint32_t init_settle_start_time;
static uint8_t init_active;
/* USER CODE END Header_ChassisTask_Entry */
void gimbal_control()
{
    /* ------------------------------ ���Լ���̵߳��� ------------------------------ */
    gimbal_dt = dwt_get_time_ms() - gimbal_start;
    gimbal_start = dwt_get_time_ms();
    if(gimbal_dt > 1)LOGERROR("ERROR:[freeRTOS] Gimbal Task Delay\r\n");

    gimbal_fdb_data.yaw_angle = gimbal_relative_yaw_angle_get();

    for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
    {
        dji_motor_enable(gim_motor_pitch[i]);
    }
    for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
    {
        dm_motor_enable(gim_motor_yaw[i]);
    }

    /* Track entry in this task: command last_mode can persist for several ticks. */
    if (gim_cmd.ctrl_mode != GIMBAL_INIT) {
        init_active = 0U;
    }

    switch (gim_cmd.ctrl_mode)
    {
        case GIMBAL_RELAX:
            for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor_pitch[i]);
            }
            dm_motor_yaw_relax();
            gimbal_fdb_data.back_mode = BACK_STEP;


            break;
        case GIMBAL_INIT: {
            // TODO������б���㷨�����Կ��ƹ���ʱ��
            // TODO: ��������ֵת��Ϊ�Ƕ�ֵ
            // TODO: �Ż������߼���yaw��ѡȡ����ķ���

#ifdef YAW_MOTOR_SET_ZERO_POSITION
            leg_init_get_zero();
#else
            const uint32_t init_now_ms = HAL_GetTick();
            if (!init_active) {
                init_active = 1U;
                init_start_time = init_now_ms;
                init_dt = 0U;
                init_settle_start_time = 0U;
                gimbal_fdb_data.back_mode = BACK_STEP;
            } else {
                init_dt = init_now_ms - init_start_time;
            }

            gim_motor_ref[YAW] = CENTER_YAW_ANGLE;
            gim_motor_ref[PITCH] = CENTER_PITCH_ANGLE;/*������̨��imu��Ϣ*/

            //pitch�����˿�˽ṹ��ֻ�ܸ���imu���ݿ��ƹ���
            const float init_pitch_error =
                fabsf(gim_ins.pitch * DEGREE_2_RAD - CENTER_PITCH_ANGLE);
            const float init_yaw_error = fabsf(gimbal_fdb_data.yaw_angle);
            const float init_pitch_rate = fabsf(gim_ins.gyro[Y]);
            const float init_yaw_rate = fabsf(
                GIMBAL_YAW_DRIVE_SIGN *
                gim_motor_yaw[0]->measure.speed_rads / GEAR_RATIO);
            const uint8_t init_feedback_fresh =
                gim_motor_yaw[0]->measure.rx_count != 0U &&
                (uint32_t)(init_now_ms -
                    gim_motor_yaw[0]->measure.last_rx_ms) <= 50U;
            const uint8_t init_rate_ready =
                init_pitch_rate <= GIMBAL_INIT_PITCH_RATE_MAX_RADPS &&
                init_yaw_rate <= GIMBAL_INIT_YAW_RATE_MAX_RADPS;
            const uint8_t init_strict_ready =
                init_pitch_error <= GIMBAL_INIT_PITCH_ANGLE_STRICT_RAD &&
                init_yaw_error <= GIMBAL_INIT_YAW_ANGLE_STRICT_RAD;
            const uint8_t init_timeout_ready =
                init_dt > INIT_TIMEOUT &&
                init_pitch_error <= GIMBAL_INIT_PITCH_ANGLE_TIMEOUT_RAD &&
                init_yaw_error <= GIMBAL_INIT_YAW_ANGLE_TIMEOUT_RAD;

            if (init_feedback_fresh && init_rate_ready &&
                (init_strict_ready || init_timeout_ready)) {
                if (init_settle_start_time == 0U) {
                    init_settle_start_time = init_now_ms;
                }
            } else {
                init_settle_start_time = 0U;
            }

            /* Hand control back after 3 s even if centering keeps oscillating.
             * Both exits seed the operator targets from the measured pose. */
            if (init_dt >= GIMBAL_INIT_FORCE_TIMEOUT_MS ||
                (init_settle_start_time != 0U &&
                 (uint32_t)(init_now_ms - init_settle_start_time) >=
                     BACK_CENTER_TIME))
               // ����ʱ�����ڹ���ģʽ�������ʵ��ſ���������
            {
                gimbal_fdb_data.back_mode = BACK_IS_OK;
                gimbal_fdb_data.yaw_delta = gim_ins.yaw_total_angle * DEGREE_2_RAD - ins.yaw_total_angle * DEGREE_2_RAD/*����yaw_totoal_angle*/;
                gim_motor_ref[YAW] = gim_ins.yaw_total_angle * DEGREE_2_RAD ;/*TODO ע�⻡�Ȼ��ǽǶ�*/
                gimbal_fdb_data.fllow_gimbal_yaw_total_position =  ins.yaw_total_angle * DEGREE_2_RAD ;
                gim_motor_ref[PITCH] = gim_ins.pitch * DEGREE_2_RAD ;
                gimbal_fdb_data.yaw_offset_angle=gim_ins.yaw;
                gimbal_fdb_data.pit_offset_angle = gim_ins.pitch;

            }
#endif

            break;
        }
        case GIMBAL_GYRO:
        {
            float command_dt_s = gimbal_dt * 0.001f;
            float yaw_stick =
                gim_cmd.vw_relative_set / RC_DBUS_MAX_VALUE;
            float pitch_stick =
                gim_cmd.pitch_relative_set / RC_DBUS_MAX_VALUE;
            LIMIT_MIN_MAX(command_dt_s, 0.0005f, 0.005f);
            LIMIT_MIN_MAX(yaw_stick, -1.0f, 1.0f);
            LIMIT_MIN_MAX(pitch_stick, -1.0f, 1.0f);
            if (fabsf(yaw_stick) < GIMBAL_STICK_DEADBAND_NORMALIZED) {
                yaw_stick = 0.0f;
            }
            if (fabsf(pitch_stick) < GIMBAL_STICK_DEADBAND_NORMALIZED) {
                pitch_stick = 0.0f;
            }
            gim_motor_ref[YAW] +=
                (-yaw_stick) *
                GIMBAL_YAW_MANUAL_RATE_MAX_RADPS * command_dt_s;
            gim_motor_ref[PITCH] +=
                pitch_stick *
                GIMBAL_PITCH_MANUAL_RATE_MAX_RADPS * command_dt_s;

            /* Keep the virtual angle target close enough for the physical
             * gimbal to follow.  This prevents a hard stop, invalid RC frame,
             * or temporary torque limit from accumulating multi-turn debt. */
            if (gim_ins.last_euler_rx_ms != 0U &&
                (uint32_t)(HAL_GetTick() - gim_ins.last_euler_rx_ms) <= 50U) {
                const float yaw_fdb =
                    gim_ins.yaw_total_angle * DEGREE_2_RAD;
                float yaw_target_lead = gim_motor_ref[YAW] - yaw_fdb;
                LIMIT_MIN_MAX(yaw_target_lead,
                              -GIMBAL_YAW_TARGET_LEAD_LIMIT_RAD,
                              GIMBAL_YAW_TARGET_LEAD_LIMIT_RAD);
                gim_motor_ref[YAW] = yaw_fdb + yaw_target_lead;
            }
            LIMIT_MIN_MAX(gim_motor_ref[PITCH],PITCH_ANGLE_MIN * DEGREE_2_RAD,PITCH_ANGLE_MAX * DEGREE_2_RAD);
            gimbal_fdb_data.gimbal_yaw_refer = gim_motor_ref[YAW];
            gimbal_fdb_data.gimbal_pitch_refer = gim_motor_ref[PITCH];

            break;
        }

            // TODO: add auto mode
        case GIMBAL_AUTO:
            /*gim_motor_ref[YAW] = gim_cmd.yaw_auto;*/
            gim_motor_ref[YAW] = gim_cmd.yaw_auto_set * DEGREE_2_RAD;/*����*/
            gim_motor_ref[PITCH] = gim_cmd.pitch_auto_set * DEGREE_2_RAD;


            // �����������̨����ֵ�ĽǶȣ�ȡ��
            break;

        default:
            for (uint8_t i = 0; i < GIM_PITCH_MOTOR_NUM; i++)
            {
                dji_motor_relax(gim_motor_pitch[i]);
            }
            for (uint8_t i = 0; i < GIM_YAW_MOTOR_NUM; i++)
            {
                dm_motor_relax(gim_motor_yaw[i]);
            }
            break;
    }

    vTaskDelay(1);
}

void gimbal_control_task(){
    /* ���¸��߳����еĶ����� */
    gimbal_sub_pull();
    gimbal_control();
    /* ���·������̵߳�msg */
    gimbal_pub_push();

}

/* ------------------------------------------------ ��̨������� ----------------------------------------------------- */
/**
 * @brief ע����̨��������������ʼ��
 */
static void gimbal_motor_init()
{
/* ----------------------------------- yaw����pid ---------------------------------- */
/* ----------------------------------- yaw ---------------------------------- */
    pid_config_t yaw_speed_init_config = INIT_PID_CONFIG(YAW_KP_V_INIT, YAW_KI_V_INIT, YAW_KD_V_INIT, YAW_INTEGRAL_V_INIT, YAW_MAX_V_INIT,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_init_config = INIT_PID_CONFIG(YAW_KP_A_INIT, YAW_KI_A_INIT, YAW_KD_A_INIT, YAW_INTEGRAL_A_INIT, YAW_MAX_A_INIT,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_speed_imu_config = INIT_PID_CONFIG(YAW_KP_V_IMU, YAW_KI_V_IMU, YAW_KD_V_IMU, YAW_INTEGRAL_V_IMU, YAW_MAX_V_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_imu_config = INIT_PID_CONFIG(YAW_KP_A_IMU, YAW_KI_A_IMU, YAW_KD_A_IMU, YAW_INTEGRAL_A_IMU, YAW_MAX_A_IMU,
                                                        (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: ����ģʽ��������
    pid_config_t yaw_speed_auto_config = INIT_PID_CONFIG(YAW_KP_V_AUTO, YAW_KI_V_AUTO, YAW_KD_V_AUTO, YAW_INTEGRAL_V_AUTO, YAW_MAX_V_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t yaw_angle_auto_config = INIT_PID_CONFIG(YAW_KP_A_AUTO, YAW_KI_A_AUTO, YAW_KD_A_AUTO, YAW_INTEGRAL_A_AUTO, YAW_MAX_A_AUTO,
                                                         (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[YAW].pid_speed_init = pid_register(&yaw_speed_init_config);
    gim_controller[YAW].pid_angle_init = pid_register(&yaw_angle_init_config);
    gim_controller[YAW].pid_speed_imu = pid_register(&yaw_speed_imu_config);
    gim_controller[YAW].pid_angle_imu = pid_register(&yaw_angle_imu_config);
    gim_controller[YAW].pid_speed_auto = pid_register(&yaw_speed_auto_config);
    gim_controller[YAW].pid_angle_auto = pid_register(&yaw_angle_auto_config);

/* ----------------------------------- pitch����pid ---------------------------------- */
    pid_config_t pitch_speed_init_config = INIT_PID_CONFIG(PITCH_KP_V_INIT, PITCH_KI_V_INIT, PITCH_KD_V_INIT, PITCH_INTEGRAL_V_INIT, PITCH_MAX_V_INIT,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_init_config = INIT_PID_CONFIG(PITCH_KP_A_INIT, PITCH_KI_A_INIT, PITCH_KD_A_INIT, PITCH_INTEGRAL_A_INIT, PITCH_MAX_A_INIT,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_speed_imu_config = INIT_PID_CONFIG(PITCH_KP_V_IMU, PITCH_KI_V_IMU, PITCH_KD_V_IMU, PITCH_INTEGRAL_V_IMU, PITCH_MAX_V_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_imu_config = INIT_PID_CONFIG(PITCH_KP_A_IMU, PITCH_KI_A_IMU, PITCH_KD_A_IMU, PITCH_INTEGRAL_A_IMU, PITCH_MAX_A_IMU,
                                                          (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    // TODO: ����ģʽ��������
    pid_config_t pitch_speed_auto_config = INIT_PID_CONFIG(PITCH_KP_V_AUTO, PITCH_KI_V_AUTO, PITCH_KD_V_AUTO, PITCH_INTEGRAL_V_AUTO, PITCH_MAX_V_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));
    pid_config_t pitch_angle_auto_config = INIT_PID_CONFIG(PITCH_KP_A_AUTO, PITCH_KI_A_AUTO, PITCH_KD_A_AUTO, PITCH_INTEGRAL_A_AUTO, PITCH_MAX_A_AUTO,
                                                           (PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement));

    gim_controller[PITCH].pid_speed_init = pid_register(&pitch_speed_init_config);
    gim_controller[PITCH].pid_angle_init = pid_register(&pitch_angle_init_config);
    gim_controller[PITCH].pid_speed_imu = pid_register(&pitch_speed_imu_config);
    gim_controller[PITCH].pid_angle_imu = pid_register(&pitch_angle_imu_config);
    gim_controller[PITCH].pid_speed_auto = pid_register(&pitch_speed_auto_config);
    gim_controller[PITCH].pid_angle_auto = pid_register(&pitch_angle_auto_config);
/* ----------------------------------- yaw ---------------------------------- */

    gim_motor_yaw[0] = dm_motor_register(&gimbal_motor_config[YAW], dm_yaw_control);

/* ---------------------------------- pitch --------------------------------- */
    gim_motor_pitch[0] = dji_motor_register(&gimbal_motor_config[PITCH], motor_control_pitch);
}


/*
 * @brief pitch��pid����
 * */
static int16_t motor_control_pitch(dji_motor_measure_t measure){

    /* PID�ֲ�ָ�룬�л���ͬģʽ��PID������ */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // �ջ�������
    static float pid_out_angle;         // �ǶȻ����
    static int16_t pitch_pid_output;        // ���շ��͸����������


    if (gim_cmd.ctrl_mode == GIMBAL_RELAX) {
        if (pid_angle != NULL) {
            gimbal_pid_reset(pid_angle);
        }
        if (pid_speed != NULL) {
            gimbal_pid_reset(pid_speed);
        }
        dbg_pitch_rate_ref = 0.0f;
        dbg_pitch_current_cmd = 0.0f;
        dbg_pitch_speed_fdb = 0.0f;
        dbg_pitch_speed_iout = 0.0f;
        dbg_pitch_angle_iout = 0.0f;
        dbg_pitch_gravity_ff = 0.0f;
        return 0;
    }

    switch (gim_cmd.ctrl_mode)
    {
        /* ������̨ģʽ���л���Ӧ�Ŀ��������۲��� */
        case GIMBAL_INIT:// TODO: ��̨��ʼ��ģʽ����б���㷨�����Կ��ƹ���ʱ��
            pid_speed = gim_controller[PITCH].pid_speed_init;
            pid_angle = gim_controller[PITCH].pid_angle_init;
            break;
        case GIMBAL_GYRO:
            pid_speed = gim_controller[PITCH].pid_speed_imu;
            pid_angle = gim_controller[PITCH].pid_angle_imu;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[PITCH].pid_speed_auto;
            pid_angle = gim_controller[PITCH].pid_angle_auto;
            break;
        default:
            break;
    }
    /* DM-IMU-L1 gyro is already decoded in rad/s (range +/-34.88 rad/s).
     * The old conversion made this feedback 57.3 times too small. */
    get_speed = gim_ins.gyro[Y];
    get_angle = gim_ins.pitch * DEGREE_2_RAD;
    dbg_pitch_speed_fdb = get_speed;
    /* �л�ģʽ��Ҫ���  ��������ʷ״̬ */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        gimbal_pid_reset(pid_angle);
        gimbal_pid_reset(pid_speed);
    }


    if(gim_cmd.ctrl_mode == GIMBAL_INIT)  // �������ջ�
    {
        /*����pid��ʹ�ã��ǶȻ������ٶȻ�����*/
        /* ע�⸺�� */
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);  // ����������������imu�෴
        pitch_pid_output = (int16_t)pid_calculate(pid_speed, get_speed, pid_out_angle);     // ���ת����������imu�෴
    }
    else /* imu�ջ� */
    {
        /* ������̨�����Ƕ� */
        VAL_LIMIT(gim_motor_ref[PITCH],
                  PITCH_ANGLE_MIN * DEGREE_2_RAD,
                  PITCH_ANGLE_MAX * DEGREE_2_RAD);
        /* ע�⸺�� */
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[PITCH]);
        pitch_pid_output = (int16_t)pid_calculate(pid_speed, get_speed, pid_out_angle);      // ���ת����������imu�෴
    }

    /* 重力前馈：Pitch 轴重力矩随 cos(IMU pitch) 变化，直驱/低减速比云台必须补偿。
     * 叠加在速度环输出之后，与反馈环共用电流限幅；RELAX 分支已提前返回不会到这里。 */
    dbg_pitch_gravity_ff =
        PITCH_GRAVITY_FF * arm_cos_f32(gim_ins.pitch * DEGREE_2_RAD);
    pitch_pid_output =
        (int16_t)((float)pitch_pid_output + dbg_pitch_gravity_ff);
    const int16_t pitch_output_limit = (int16_t)(
        gim_cmd.ctrl_mode == GIMBAL_INIT ?
            PITCH_MAX_V_INIT : PITCH_MAX_V_IMU);
    VAL_LIMIT(pitch_pid_output, -pitch_output_limit, pitch_output_limit);
    if(gim_cmd.ctrl_mode == GIMBAL_RELAX)  // �������ջ�
    {
        gimbal_pid_reset(pid_angle);
        gimbal_pid_reset(pid_speed);
        pitch_pid_output = 0;     // ���ת����������imu�෴
    }

#if defined(GIMBAL_RELEX) || defined(GM6020_SET_ZERO)
    pitch_pid_output = 0;
#endif

    dbg_pitch_rate_ref = pid_out_angle;
    dbg_pitch_current_cmd = (float)pitch_pid_output;
    if (pid_angle != NULL) {
        dbg_pitch_angle_iout = pid_angle->Iout;
    }
    if (pid_speed != NULL) {
        dbg_pitch_speed_iout = pid_speed->Iout;
    }

    return pitch_pid_output;

}
/*
 * @brief ��̨yaw����
 * */
static dm_motor_para_t dm_yaw_control(dm_motor_measure_t measure)
{
    /* PID�ֲ�ָ�룬�л���ͬģʽ��PID������ */
    static pid_obj_t *pid_angle;
    static pid_obj_t *pid_speed;
    static float get_speed, get_angle;  // �ջ�������
    static float pid_out_angle;         // �ǶȻ����
    static float send_data;        // ���շ��͸����������

    static dm_motor_para_t set;

    if (gim_cmd.ctrl_mode == GIMBAL_RELAX) {
        if (pid_angle != NULL) {
            gimbal_pid_reset(pid_angle);
        }
        if (pid_speed != NULL) {
            gimbal_pid_reset(pid_speed);
        }
        set.p = 0.0f;
        set.v = 0.0f;
        set.kp = 0.0f;
        set.kd = 0.0f;
        set.t = 0.0f;
        dm_yaw_send_t[0] = 0.0f;
        dbg_yaw_rate_ref = 0.0f;
        dbg_yaw_speed_fdb = 0.0f;
        dbg_yaw_torque_cmd = 0.0f;
        dbg_yaw_angle_fdb = 0.0f;
        dbg_yaw_speed_iout = 0.0f;
        dbg_yaw_angle_iout = 0.0f;
        return set;
    }

    switch (gim_cmd.ctrl_mode)
    {
        // TODO: ��̨��ʼ��ģʽ����б���㷨�����Կ��ƹ���ʱ��
        case GIMBAL_INIT: {
            pid_speed = gim_controller[YAW].pid_speed_init;
            pid_angle = gim_controller[YAW].pid_angle_init;
            /*�۲�ֵ*/
            /*!!!ǰ������Ϊ��������Ϊyaw���������λ��*/
            if(gim_motor_yaw[0]->measure.yaw_angle >= gim_motor_yaw[0]->measure.yaw_angle_round / 2.0f){
                get_angle = (gim_motor_yaw[0]->measure.yaw_angle_round - gim_motor_yaw[0]->measure.yaw_angle);/*ȡ�ӻ�*/
            }
            if(gim_motor_yaw[0]->measure.yaw_angle <= - gim_motor_yaw[0]->measure.yaw_angle_round / 2.0f){
                get_angle = -(gim_motor_yaw[0]->measure.yaw_angle_round - gim_motor_yaw[0]->measure.yaw_angle);/*ȡ�ӻ�*/
            }
            else{
                get_angle = gim_motor_yaw[0]->measure.yaw_angle;/*yaw��۲�Ƕ�*/
            }
            break;
        }
        case GIMBAL_GYRO:
            pid_speed = gim_controller[YAW].pid_speed_imu;
            pid_angle = gim_controller[YAW].pid_angle_imu;
            get_angle = gim_ins.yaw_total_angle * DEGREE_2_RAD;
            break;
        case GIMBAL_AUTO:
            pid_speed = gim_controller[YAW].pid_speed_auto;
            pid_angle = gim_controller[YAW].pid_angle_auto;
            get_angle = gim_ins.yaw_total_angle * DEGREE_2_RAD;
            break;
        default:
            break;
    }

    if (gim_cmd.ctrl_mode == GIMBAL_INIT) {
        float yaw_motor_angle = measure.yaw_angle;
        if (yaw_motor_angle > 0.5f * measure.yaw_angle_round) {
            yaw_motor_angle -= measure.yaw_angle_round;
        } else if (yaw_motor_angle < -0.5f * measure.yaw_angle_round) {
            yaw_motor_angle += measure.yaw_angle_round;
        }
        get_angle = GIMBAL_YAW_DRIVE_SIGN * yaw_motor_angle / GEAR_RATIO;
        /* Mechanical homing is chassis-relative, so use motor-relative rate. */
        get_speed =
            GIMBAL_YAW_DRIVE_SIGN * measure.speed_rads / GEAR_RATIO;
    } else {
        /* GYRO/AUTO angle feedback is world-referenced IMU yaw.  Its inner
         * rate loop must use the matching inertial yaw rate. */
        get_speed = gim_ins.gyro[Z];
    }

    dbg_yaw_speed_fdb = get_speed;
    dbg_yaw_angle_fdb = get_angle;

    /* �л�ģʽ��Ҫ��տ�������ʷ״̬ */
    if(gim_cmd.ctrl_mode != gim_cmd.last_mode)
    {
        gimbal_pid_reset(pid_angle);
        gimbal_pid_reset(pid_speed);
    }

    if(gim_cmd.ctrl_mode == GIMBAL_INIT)  // �������ջ�
    {
        /* ע�⸺�� */
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);  // ����������������imu�෴
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);     // ���ת����������imu�෴
    }
    else /* imu�ջ� */
    {
        /* ע�⸺�� */
        pid_out_angle = pid_calculate(pid_angle, get_angle, gim_motor_ref[YAW]);
        send_data = pid_calculate(pid_speed, get_speed, pid_out_angle);      // ���ת����������imu�෴

    }

    dm_yaw_send_t[0] = send_data * DM_YAW_RATIO;/*TODO ����ʵ�����������*/

    dm_yaw_send_t[0] = GIMBAL_YAW_DRIVE_SIGN *
        send_data * DM_YAW_RATIO;
    LIMIT_MIN_MAX(dm_yaw_send_t[0], -DM_GIMBAL_OUTPUT_LIMIT, DM_GIMBAL_OUTPUT_LIMIT);
    dm_yaw_obs[0] = dm_yaw_send_t[0] ;

    set.p = 0;
    set.kp = 0;
    set.v = 0;
    set.kd = 0;
#if defined(GIMBAL_RELEX) || defined(DM4310_SET_ZERO)
        dm_yaw_send_t[0] = 0;
#endif
    if(gim_cmd.ctrl_mode == GIMBAL_RELAX)  // �������ջ�
    {
        gimbal_pid_reset(pid_angle);
        gimbal_pid_reset(pid_speed);
        dm_yaw_send_t[0] = 0;     // ���ת����������imu�෴
    }
    LIMIT_MIN_MAX(dm_yaw_send_t[0], -DM_GIMBAL_OUTPUT_LIMIT, DM_GIMBAL_OUTPUT_LIMIT);
    {
        set.t = dm_yaw_send_t[0]; // ����û����
    }
    dbg_yaw_rate_ref = pid_out_angle;
    dbg_yaw_torque_cmd = dm_yaw_send_t[0];
    if (pid_angle != NULL) {
        dbg_yaw_angle_iout = pid_angle->Iout;
    }
    if (pid_speed != NULL) {
        dbg_yaw_speed_iout = pid_speed->Iout;
    }
    return set;
}

#define GIMBAL_DEBUG_FRESH_MS 50U

static float gimbal_debug_age_ms(uint32_t now_ms, uint32_t last_rx_ms,
                                 uint32_t rx_count)
{
    if (rx_count == 0U) {
        return -1.0f;
    }
    return (float)(now_ms - last_rx_ms);
}

static uint8_t gimbal_debug_is_fresh(float age_ms)
{
    return age_ms >= 0.0f && age_ms <= (float)GIMBAL_DEBUG_FRESH_MS;
}

uint8_t gimbal_fill_debug_page(uint8_t page)
{
    if (page != 44U && page != 45U && page != 46U && page != 49U &&
        page != 55U) {
        return 0U;
    }

    const uint32_t now_ms = HAL_GetTick();
    const float time_s = dwt_get_time_ms() / 1000.0f;
    const uint8_t pitch_valid = (gim_motor_pitch[0] != NULL);
    const uint8_t yaw_valid = (gim_motor_yaw[0] != NULL);

    const uint32_t pitch_count = pitch_valid ?
        gim_motor_pitch[0]->measure.rx_count : 0U;
    const uint32_t yaw_count = yaw_valid ?
        gim_motor_yaw[0]->measure.rx_count : 0U;
    const float imu_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_rx_ms, gim_ins.rx_count);
    const float accel_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_accel_rx_ms,
        gim_ins.last_accel_rx_ms != 0U ? 1U : 0U);
    const float gyro_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_gyro_rx_ms,
        gim_ins.last_gyro_rx_ms != 0U ? 1U : 0U);
    const float euler_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_euler_rx_ms,
        gim_ins.last_euler_rx_ms != 0U ? 1U : 0U);
    const float pitch_age = pitch_valid ? gimbal_debug_age_ms(
        now_ms, gim_motor_pitch[0]->measure.last_rx_ms, pitch_count) : -1.0f;
    const float yaw_age = yaw_valid ? gimbal_debug_age_ms(
        now_ms, gim_motor_yaw[0]->measure.last_rx_ms, yaw_count) : -1.0f;

    static uint32_t rate_last_ms;
    static uint32_t imu_count_last;
    static uint32_t pitch_count_last;
    static uint32_t yaw_count_last;
    static float imu_rx_hz;
    static float pitch_rx_hz;
    static float yaw_rx_hz;
    const uint32_t rate_dt_ms = now_ms - rate_last_ms;
    if (rate_last_ms != 0U && rate_dt_ms > 0U) {
        const float rate_scale = 1000.0f / (float)rate_dt_ms;
        imu_rx_hz = (float)(gim_ins.rx_count - imu_count_last) * rate_scale;
        pitch_rx_hz = (float)(pitch_count - pitch_count_last) * rate_scale;
        yaw_rx_hz = (float)(yaw_count - yaw_count_last) * rate_scale;
    }
    rate_last_ms = now_ms;
    imu_count_last = gim_ins.rx_count;
    pitch_count_last = pitch_count;
    yaw_count_last = yaw_count;

    uint16_t health_flags = 0U;
    if (gimbal_debug_is_fresh(imu_age)) health_flags |= (1U << 0);
    if (gimbal_debug_is_fresh(accel_age)) health_flags |= (1U << 1);
    if (gimbal_debug_is_fresh(gyro_age)) health_flags |= (1U << 2);
    if (gimbal_debug_is_fresh(euler_age)) health_flags |= (1U << 3);
    if (gimbal_debug_is_fresh(pitch_age)) health_flags |= (1U << 4);
    if (gimbal_debug_is_fresh(yaw_age)) health_flags |= (1U << 5);
    if (yaw_valid && gimbal_debug_is_fresh(yaw_age) &&
        gim_motor_yaw[0]->measure.ERR <= 1U) {
        health_flags |= (1U << 6);
    }
    if ((health_flags & 0x7FU) == 0x7FU) health_flags |= (1U << 7);

    if (page == 55U) {
        /* Page 55 I22-I31: gimbal motors and gimbal IMU health.
         * I22/I23 pitch/yaw feedback age ms; I24/I25 pitch/yaw max temp;
         * I26 pitch feedback current; I27 yaw DM status (1=enabled);
         * I28 gimbal IMU age; I29/I30 gyro/euler age; I31 health bits. */
        float yaw_max_temp = -1.0f;
        if (yaw_valid) {
            yaw_max_temp = fmaxf(gim_motor_yaw[0]->measure.temperature_MOS,
                                 gim_motor_yaw[0]->measure.temperature_Rotor);
        }
        chassis_debug_set(22U, pitch_age);
        chassis_debug_set(23U, yaw_age);
        chassis_debug_set(24U, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.temperature : -1.0f);
        chassis_debug_set(25U, yaw_max_temp);
        chassis_debug_set(26U, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.real_current : 0.0f);
        chassis_debug_set(27U, yaw_valid ?
            (float)gim_motor_yaw[0]->measure.ERR : -1.0f);
        chassis_debug_set(28U, imu_age);
        chassis_debug_set(29U, gyro_age);
        chassis_debug_set(30U, euler_age);
        chassis_debug_set(31U, (float)health_flags);
        return 1U;
    }

    if (page == 44U) {
        const float dm_max_temp = yaw_valid ?
            ((gim_motor_yaw[0]->measure.temperature_MOS >
              gim_motor_yaw[0]->measure.temperature_Rotor) ?
             gim_motor_yaw[0]->measure.temperature_MOS :
             gim_motor_yaw[0]->measure.temperature_Rotor) : 0.0f;
        chassis_debug_set(0, time_s);
        chassis_debug_set(1, (float)gim_cmd.ctrl_mode);
        chassis_debug_set(2, (float)health_flags);
        chassis_debug_set(3, imu_age);
        chassis_debug_set(4, imu_rx_hz);
        chassis_debug_set(5, accel_age);
        chassis_debug_set(6, gyro_age);
        chassis_debug_set(7, euler_age);
        chassis_debug_set(8, pitch_age);
        chassis_debug_set(9, pitch_rx_hz);
        chassis_debug_set(10, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.temperature : 0.0f);
        chassis_debug_set(11, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.real_current : 0.0f);
        chassis_debug_set(12, yaw_age);
        chassis_debug_set(13, yaw_rx_hz);
        chassis_debug_set(14, yaw_valid ?
            (float)gim_motor_yaw[0]->measure.ERR : -1.0f);
        chassis_debug_set(15, dm_max_temp);
        return 1U;
    }

    if (page == 45U) {
        const float pitch_fdb = gim_ins.pitch * DEGREE_2_RAD;
        chassis_debug_set(0, time_s);
        chassis_debug_set(1, (float)gim_cmd.ctrl_mode);
        chassis_debug_set(2, (float)health_flags);
        chassis_debug_set(3, gim_motor_ref[PITCH]);
        chassis_debug_set(4, pitch_fdb);
        chassis_debug_set(5, gim_motor_ref[PITCH] - pitch_fdb);
        chassis_debug_set(6, gim_ins.gyro[Y]);
        chassis_debug_set(7, dbg_pitch_rate_ref);
        chassis_debug_set(8, dbg_pitch_current_cmd);
        chassis_debug_set(9, pitch_valid ?
            gim_motor_pitch[0]->measure.angle_single_round : 0.0f);
        chassis_debug_set(10, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.speed_rpm : 0.0f);
        chassis_debug_set(11, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.real_current : 0.0f);
        chassis_debug_set(12, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.temperature : 0.0f);
        chassis_debug_set(13, pitch_age);
        chassis_debug_set(14, pitch_rx_hz);
        chassis_debug_set(15, pitch_valid ?
            (float)gim_motor_pitch[0]->stop_flag : -1.0f);
        return 1U;
    }

    if (page == 49U) {
        /* Combined 32-channel tuning page:
         * 0..4 common, 5..17 pitch, 18..31 yaw. */
        const float pitch_fdb = gim_ins.pitch * DEGREE_2_RAD;
        const float yaw_imu_fdb =
            gim_ins.yaw_total_angle * DEGREE_2_RAD;
        const float yaw_motor_rate = yaw_valid ?
            GIMBAL_YAW_DRIVE_SIGN *
                gim_motor_yaw[0]->measure.speed_rads / GEAR_RATIO : 0.0f;

        chassis_debug_set(0, time_s);
        chassis_debug_set(1, (float)gim_cmd.ctrl_mode);
        chassis_debug_set(2, (float)health_flags);
        chassis_debug_set(3, gim_cmd.pitch_relative_set);
        chassis_debug_set(4, gim_cmd.vw_relative_set);

        chassis_debug_set(5, gim_motor_ref[PITCH]);
        chassis_debug_set(6, pitch_fdb);
        chassis_debug_set(7, gim_motor_ref[PITCH] - pitch_fdb);
        chassis_debug_set(8, dbg_pitch_rate_ref);
        chassis_debug_set(9, dbg_pitch_speed_fdb);
        chassis_debug_set(10, dbg_pitch_current_cmd);
        chassis_debug_set(11, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.real_current : 0.0f);
        chassis_debug_set(12, pitch_valid ?
            (float)gim_motor_pitch[0]->measure.speed_rpm : 0.0f);
        chassis_debug_set(13, dbg_pitch_speed_iout);
        chassis_debug_set(14, dbg_pitch_angle_iout);
        chassis_debug_set(15, dbg_pitch_gravity_ff);
        chassis_debug_set(16, pitch_valid ?
            gim_motor_pitch[0]->measure.angle_single_round : 0.0f);
        chassis_debug_set(17, pitch_age);

        chassis_debug_set(18, gim_motor_ref[YAW]);
        chassis_debug_set(19, dbg_yaw_angle_fdb);
        chassis_debug_set(20, gim_motor_ref[YAW] - dbg_yaw_angle_fdb);
        chassis_debug_set(21, dbg_yaw_rate_ref);
        chassis_debug_set(22, dbg_yaw_speed_fdb);
        chassis_debug_set(23, dbg_yaw_torque_cmd);
        chassis_debug_set(24, gimbal_fdb_data.yaw_angle);
        chassis_debug_set(25, yaw_motor_rate);
        chassis_debug_set(26, yaw_valid ?
            gim_motor_yaw[0]->measure.torque : 0.0f);
        chassis_debug_set(27, gim_ins.gyro[Z]);
        chassis_debug_set(28, yaw_imu_fdb);
        chassis_debug_set(29, dbg_yaw_speed_iout);
        chassis_debug_set(30, dbg_yaw_angle_iout);
        chassis_debug_set(31, yaw_age);
        return 1U;
    }

    const float yaw_fdb = gim_ins.yaw_total_angle * DEGREE_2_RAD;
    chassis_debug_set(0, time_s);
    chassis_debug_set(1, (float)gim_cmd.ctrl_mode);
    chassis_debug_set(2, gim_motor_ref[YAW]);
    chassis_debug_set(3, dbg_yaw_angle_fdb);
    chassis_debug_set(4, gim_motor_ref[YAW] - dbg_yaw_angle_fdb);
    chassis_debug_set(5, dbg_yaw_rate_ref);
    chassis_debug_set(6, dbg_yaw_speed_fdb);
    chassis_debug_set(7, dbg_yaw_torque_cmd);
    chassis_debug_set(8, gimbal_fdb_data.yaw_angle);
    chassis_debug_set(9, yaw_valid ?
        gim_motor_yaw[0]->measure.speed_rads : 0.0f);
    chassis_debug_set(10, yaw_valid ?
        gim_motor_yaw[0]->measure.torque : 0.0f);
    chassis_debug_set(11, gim_ins.gyro[Z]);
    chassis_debug_set(12, dbg_yaw_speed_iout);
    chassis_debug_set(13, dbg_yaw_angle_iout);
    chassis_debug_set(14, yaw_fdb);
    chassis_debug_set(15, yaw_age);
    return 1U;
}




void gimbal_fill_all_debug(uint8_t first_channel)
{
    const uint32_t now_ms = HAL_GetTick();
    const uint8_t pitch_valid = (gim_motor_pitch[0] != NULL);
    const uint8_t yaw_valid = (gim_motor_yaw[0] != NULL);
    const uint32_t pitch_count = pitch_valid ?
        gim_motor_pitch[0]->measure.rx_count : 0U;
    const uint32_t yaw_count = yaw_valid ?
        gim_motor_yaw[0]->measure.rx_count : 0U;
    const float imu_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_rx_ms, gim_ins.rx_count);
    const float gyro_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_gyro_rx_ms,
        gim_ins.last_gyro_rx_ms != 0U ? 1U : 0U);
    const float euler_age = gimbal_debug_age_ms(
        now_ms, gim_ins.last_euler_rx_ms,
        gim_ins.last_euler_rx_ms != 0U ? 1U : 0U);
    const float pitch_age = pitch_valid ? gimbal_debug_age_ms(
        now_ms, gim_motor_pitch[0]->measure.last_rx_ms, pitch_count) : -1.0f;
    const float yaw_age = yaw_valid ? gimbal_debug_age_ms(
        now_ms, gim_motor_yaw[0]->measure.last_rx_ms, yaw_count) : -1.0f;
    uint8_t health_flags = 0U;
    if (pitch_valid && gimbal_debug_is_fresh(pitch_age)) health_flags |= (1U << 0);
    if (yaw_valid && gimbal_debug_is_fresh(yaw_age) &&
        gim_motor_yaw[0]->measure.ERR <= 1U) health_flags |= (1U << 1);
    if (gimbal_debug_is_fresh(gyro_age)) health_flags |= (1U << 2);
    if (gimbal_debug_is_fresh(euler_age)) health_flags |= (1U << 3);
    if (gimbal_debug_is_fresh(imu_age)) health_flags |= (1U << 4);
    if ((health_flags & 0x1FU) == 0x1FU) health_flags |= (1U << 5);
    float max_age = fmaxf(imu_age, fmaxf(gyro_age, euler_age));
    max_age = fmaxf(max_age, fmaxf(pitch_age, yaw_age));
    chassis_debug_set((uint8_t)(first_channel + 0U), gim_ins.pitch);
    chassis_debug_set((uint8_t)(first_channel + 1U), dbg_pitch_current_cmd);
    chassis_debug_set((uint8_t)(first_channel + 2U), (float)health_flags);
    chassis_debug_set((uint8_t)(first_channel + 3U), max_age);
}

/*
 * @brief ���ô��������
 * */
static void leg_init_get_zero()
{
    gim_motor_yaw[0]->set_mode(gim_motor_yaw[0], DM_CMD_ZERO_POSITION);

}
/******************************************************��Ϣ����*************************************************************************/
/**
 * @brief cmd �߳������з��������͸��»���
 */
static void gimbal_pub_push(void)
{
    // data_content my_data = ;
    mcn_publish(MCN_HUB(gimbal_fdb), &gimbal_fdb_data);
}

/**
 * @brief cmd �߳������ж����߳�ʼ��
 */
static void gimbal_sub_init(void)
{
    ins_topic_node = mcn_subscribe(MCN_HUB(ins_topic), NULL, NULL);
    chassis_cmd_node = mcn_subscribe(MCN_HUB(chassis_cmd), NULL, NULL);
    gimbal_cmd_node = mcn_subscribe(MCN_HUB(gimbal_cmd), NULL, NULL);
    gimbal_ins_node = mcn_subscribe(MCN_HUB(gimbal_ins_topic), NULL, NULL);
    trans_fdb_node = mcn_subscribe(MCN_HUB(transmission_fdb), NULL, NULL) ;
}


/**
 * @brief cmd �߳������ж����߻�ȡ���»���
 */
static void gimbal_sub_pull(void)
{
    if (mcn_poll(ins_topic_node))
    {
        mcn_copy(MCN_HUB(ins_topic), ins_topic_node, &ins);
    }

    if (mcn_poll(chassis_cmd_node))
    {
        mcn_copy(MCN_HUB(chassis_cmd), chassis_cmd_node, &chassis_fdb_cmd);
    }

    if (mcn_poll(gimbal_cmd_node))
    {
        mcn_copy(MCN_HUB(gimbal_cmd), gimbal_cmd_node, &gim_cmd);
    }

    if (mcn_poll(gimbal_ins_node))
    {
        mcn_copy(MCN_HUB(gimbal_ins_topic), gimbal_ins_node, &gim_ins);
    }

    if (mcn_poll(trans_fdb_node))
    {
        mcn_copy(MCN_HUB(transmission_fdb), trans_fdb_node, &trans_fdb);
    }
}

