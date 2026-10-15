//
// Created by SuperChen on 2025/11/15.
//

#ifndef HNU_RM_DOWN_DM_MOTOR_H
#define HNU_RM_DOWN_DM_MOTOR_H

#include "rm_module.h"

#define Motar_mode 0	//设置模式为何种模式，为0为IMT模式，为1为位置速度模式，为2为速度模式

#define DM_P_MIN -PI2		//位置最小值
#define DM_P_MAX PI2		//位置最大值
#define DM_V_MIN -45			//速度最小值
#define DM_V_MAX 45			//速度最大值
#define DM_KP_MIN 0		//Kp最小值
#define DM_KP_MAX 3.0	//Kp最大值,目前根据上位机控制设置中读取的值设定,实际mit模式下位置和速度基本不起作用,主要为力矩T
#define DM_KD_MIN 0		//Kd最小值
#define DM_KD_MAX 5.0		//Kd最大值
#define DM_T_MIN -35			//转矩最小值
#define DM_T_MAX 35			//转矩最大值

#define DM_ANGLE_DELTA_THRESHOLD   12.0f  //实测数据电机套圈时角度的变化,此值观测得到,和上位机DM_P_MAX取值有关

#define DM_SPEED_BIAS 0.0f   //电机速度偏差 rad/s
/* 滤波系数设置为1的时候即关闭滤波 */
#define DM_CURRENT_SMOOTH_COEF 0.9f

#define DM_SPEED_BUFFER_SIZE  5
/*电机的控制模式*/
typedef enum
{
    MIT ,
    POSITION_SPEED,
    SPEED ,
    PVT,
}dm_set_mode_e;

/* dm电机模式,初始化时自动进入CMD_MOTOR_MODE*/
typedef enum
{
    DM_CMD_RESET_MODE = 0xFD,   // 停止
    DM_CMD_MOTOR_MODE = 0xFC,   // 使能,会响应指令
    DM_CMD_ZERO_POSITION = 0xFE // 将当前的位置设置为编码器零位
} dm_motor_mode_e;

/**
 * @brief 海泰电机控制参数，具体用法见说明书
 */
typedef struct dm_motor_para
{
    float p;     // 目标位置，单位为弧度(rad)
    float v;     // 目标速度，单位为 rad/s
    float kp;    // 为位置增益，单位为 N-m/rad
    float kd;    // 为速度增益，单位为 N-m*s/rad
    float t;     // 力矩，单位为 N-m
}dm_motor_para_t;

/**
 * @brief dm04 motor feedback
 */
typedef struct
{

    uint8_t id;
    uint8_t ERR;   //错误码

    float total_angle;        // 角度为多圈角度
    float circle_cnt;          //圈数
    float yaw_angle;          //有yaw轴电机算出的实际云台角度
    float yaw_angle_round;    //云台连接处总弧度
    float angle;              //角度范围在-2π到2π
    float angle_abs;          //绝对角度,每个位置固定
    float last_angle_abs;
    float angle_delta;
    float speed_rads;
    float torque;             //扭矩
    uint8_t raw_data[8];      //最近一次原始反馈报文，用于协议诊断
    float temperature_MOS;   /*!< Motor Temperature_MOS   */
    float temperature_Rotor; /*!< Motor Temperature_Rotor */
    float target;            // 目标值(输出轴扭矩矩/速度/角度(单位度))
    uint32_t rx_count;        /*!< Successfully decoded feedback frames. */
    uint32_t last_rx_ms;      /*!< HAL tick of the latest feedback frame. */
} dm_motor_measure_t;

/**
 * @brief dm intelligent motor typedef
 */
typedef struct dm_motor_object
{
    FDCAN_HandleTypeDef  *fdcan;                // 电机挂载CAN句柄
    dm_motor_measure_t measure;             // 电机测量值

    uint32_t tx_id;                         // 发送id(主发)
    uint32_t rx_id;                         // 接收id(主收)

    motor_type_e motor_type;                // 电机类型
    dm_set_mode_e   set_control_mode;               //电机控制模式:mit,position_speed,speed
    dm_motor_mode_e ctrl_mode;              // 电机当前模式
    dm_motor_mode_e to_mode;                // 电机将要切换的模式
    motor_working_type_e stop_flag;         // 启停标志
    osSemaphoreId turn_complete;            // 电机模式切换完成量

    /* 监控线程相关 */

    /* 电机控制相关 */
    void *controller;            // 电机控制器
    dm_motor_para_t (*control)(dm_motor_measure_t measure);   // 控制电机的接口 用户可以自定义,返回值为 dm_motor_para_t 类型控制参数
    void (*set_mode)(struct dm_motor_object *motor, dm_motor_mode_e cmd);    // 用户可以调用改方法设置电机模式
} dm_motor_object_t;

/**
 * @brief 调用此函数注册一个dm04电机
 *
 * @param config 电机初始化结构体,包含了电机控制设置,电机PID参数设置,电机类型以及电机挂载的CAN设置
 *
 * @return dm_motor_object_t*
 */
dm_motor_object_t *dm_motor_register(motor_config_t *config, void *control);

/**
 * @brief 所有电机退出 motor 模式
 */
void dm_motor_disable_all();
/*失能云台yaw的4310*/
void dm_motor_yaw_relax();
/**
 * @brief 所有电机进入 motor 模式
 *        初始化时不需要此函数,因为stop_flag的默认值为0
 */
void dm_motor_enable_all();

/**
 * @brief 电机设置启停模式
 */
void dm_motor_set_type(dm_motor_object_t *motor, motor_working_type_e type);

/**
 * @brief 电机反馈报文接收回调函数,该函数被can_rx_call调用
 *
 * @param id 接收到的报文的id
 * @param data 接收到的报文的数据
 */
int dm_motor_rx_callback(uint32_t id, uint8_t *data);

/**
 * @brief 为了避免总线堵塞,为每个电机创建一个发送任务(目前没使用该控制方案，通过定时器中断控制频率)
 * @param argument 传入的电机指针
 */
void dm_motor_task_init(void);

/**
 * @brief 用户按一定频率轮询控制dm，避免总线拥堵，目前与定时器中断中被调用
 */
void dm_controll_all_poll(void);
/* Publish jump/landing polling policy from the completed chassis cycle. */
void dm_motor_set_batch_poll(uint8_t enabled);

void dm_motor_relax(dm_motor_object_t *motor);
void dm_motor_enable(dm_motor_object_t *motor);

#endif //HNU_RM_DOWN_DM_MOTOR_H
