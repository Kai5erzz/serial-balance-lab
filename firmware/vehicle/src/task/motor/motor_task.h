/**
 * @file motor_task.h
 * @author neozng
 * @brief  对所有电机,舵机等控制任务的进一步封装,motor_control_task()将在操作系统中按一定频率调用
 * @version beta
 * @date 2022-11-01
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

/**
 * @brief 电机控制闭环任务初始化
 */
void motor_task_init(void);

/**
 * @brief 电机控制闭环任务,在RTOS中应该设定为1Khz运行
 *        舵机控制任务的频率设定为20Hz或更低
 */
void motor_control_task(void);

/**
 * @brief damiao电机接收回调
 */
//void CAN1_Motor_Call_Back(Struct_CAN_Rx_Buffer * Rx_Buffer);

#endif // !MOTOR_TASK_H

