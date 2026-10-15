
 /**
 * @file rm_task.h
 * @brief  注意该文件应只用于任务初始化,只能被robot.c包含
 * @date 2023-12-28
 */

 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-28      ChuShicheng     first version
 */
#ifndef _RM_TASK_H
#define _RM_TASK_H

#include "ins_task.h"
#include "motor_task.h"
#include "cmd_task.h"
#include "chassis_task.h"
#include "trans_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "bsp_log.h"
#include "drv_dwt.h"

/* ---------------------------------- 线程相关 ---------------------------------- */
osThreadId insTaskHandle;
osThreadId chassisTaskHandle;
osThreadId cmdTaskHandle;
osThreadId motorTaskHandle;
osThreadId transTaskHandle;
osThreadId refereeTaskHandle;

void ins_task_entry(void const *argument);
void motor_task_entry(void const *argument);
void chassis_task_entry(void const *argument);
void cmd_task_entry(void const *argument);
void trans_task_entry(void const *argument);
void gimbal_task_entry(void const *argument);
void shoot_task_entry(void const *argument);
//void referee_task_entry(void const *argument);

static float motor_dt;
static float chassis_dt;
static float cmd_dt;
static float trans_dt;
static float shoot_dt;
static float ins_dt;
static float gimbal_dt;
static char cmd_dt_str[BSP_LOG_FLOAT_STR_LEN];
static char motor_dt_str[BSP_LOG_FLOAT_STR_LEN];
static char chassis_dt_str[BSP_LOG_FLOAT_STR_LEN];
static char trans_dt_str[BSP_LOG_FLOAT_STR_LEN];
static char shoot_dt_str[BSP_LOG_FLOAT_STR_LEN];
static char gimbal_start_str[BSP_LOG_FLOAT_STR_LEN];
static char ins_dt_str[BSP_LOG_FLOAT_STR_LEN];
//static float referee_dt;

/**
 * @brief 初始化机器人任务,所有持续运行的任务都在这里初始化
 *
 */
void OS_task_init()
{
    osThreadDef(instask, ins_task_entry, osPriorityNormal, 0, 1024);
    insTaskHandle = osThreadCreate(osThread(instask), NULL); // 为姿态解算设置较高优先级,确保以1khz的频率执行

    osThreadDef(motortask, motor_task_entry, osPriorityNormal, 0, 2048);
    motorTaskHandle = osThreadCreate(osThread(motortask), NULL);

    osThreadDef(chassistask, chassis_task_entry, osPriorityNormal, 0, 2048);
    chassisTaskHandle = osThreadCreate(osThread(chassistask), NULL);

    osThreadDef(cmdtask, cmd_task_entry, osPriorityNormal, 0, 1024);
    cmdTaskHandle = osThreadCreate(osThread(cmdtask), NULL);

    osThreadDef(gimbaltask, gimbal_task_entry, osPriorityNormal, 0, 1024);
    cmdTaskHandle = osThreadCreate(osThread(gimbaltask), NULL);

    osThreadDef(transtask, trans_task_entry, osPriorityNormal, 0, 1024);
    transTaskHandle = osThreadCreate(osThread(transtask), NULL);

    osThreadDef(shoottask, shoot_task_entry, osPriorityNormal, 0, 1024);
    transTaskHandle = osThreadCreate(osThread(shoottask), NULL);

//    osThreadDef(refereetask, referee_task_entry, osPriorityNormal, 0, 1024);
//    refereeTaskHandle = osThreadCreate(osThread(refereetask), NULL);

}

__attribute__((noreturn)) void motor_task_entry(void const *argument)
{
    float motor_start = dwt_get_time_ms();
    LOGINFO("[freeRTOS] Motor Task Start\r\n");

    uint32_t motor_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        motor_dt = dwt_get_time_ms() - motor_start;
        motor_start = dwt_get_time_ms();
        if (motor_dt > 1.5) {
            Float2Str(motor_dt_str,motor_dt);
            LOGERROR("[freeRTOS] Motor Task is being DELAY! dt = %s\r\n", motor_dt_str);
        }

/* ------------------------------ 调试监测线程调度 ------------------------------ */

        motor_control_task();

        vTaskDelayUntil(&motor_wake_time, 1);
    }
}

__attribute__((noreturn)) void chassis_task_entry(void const *argument)
{
    float chassis_start = dwt_get_time_ms();
    LOGINFO("[freeRTOS] Chassis Task Start\r\n");

    uint32_t chassis_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        chassis_dt = dwt_get_time_ms() - chassis_start;
        chassis_start = dwt_get_time_ms();
        if (chassis_dt > 5.5) {
            Float2Str(chassis_dt_str,chassis_dt);
            LOGERROR("[freeRTOS] Chassis Task is being DELAY! dt = %s\r\n", chassis_dt_str);
        }

/* ------------------------------ 调试监测线程调度 ------------------------------ */

        chassis_control_task();

        vTaskDelayUntil(&chassis_wake_time, 1);  // 平衡步兵需要1khz
    }
}

 __attribute__((noreturn)) void cmd_task_entry(void const *argument)
 {
     float cmd_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] Cmd Task Start\r\n");
     uint32_t robot_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
         cmd_dt = dwt_get_time_ms() - cmd_start;
         cmd_start = dwt_get_time_ms();
         if (cmd_dt > 1.5) {
             Float2Str(cmd_dt_str,cmd_dt);
             LOGERROR("[freeRTOS] Cmd Task is being DELAY! dt = %s\r\n", cmd_dt_str);
         }

/* ------------------------------ 调试监测线程调度 ------------------------------ */

         cmd_control_task();

         vTaskDelayUntil(&robot_wake_time, 1);
     }
 }

 __attribute__((noreturn)) void trans_task_entry(void const *argument)
{
    float trans_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] Trans Task Start\r\n");
    uint32_t trans_wake_time = osKernelSysTick();
    for (;;)
    {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
        trans_dt = dwt_get_time_ms() - trans_start;
        trans_start = dwt_get_time_ms();
        if (trans_dt > 1.5) {
            Float2Str(trans_dt_str,trans_dt);
            LOGERROR("[freeRTOS] Trans Task is being DELAY! dt = %s\r\n", trans_dt_str);
        }

/* ------------------------------ 调试监测线程调度 ------------------------------ */

        trans_control_task();

        vTaskDelayUntil(&trans_wake_time, 1);
    }
}

 __attribute__((noreturn)) void gimbal_task_entry(void const *argument)
 {
     float gimbal_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] gimbal Task Start\r\n");
     uint32_t gimbal_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
         gimbal_dt = dwt_get_time_ms() - gimbal_start;
         gimbal_start = dwt_get_time_ms();
         if (gimbal_dt > 1.5) {
             Float2Str(gimbal_start_str,gimbal_dt);
             LOGERROR("[freeRTOS] Gimbal Task is being DELAY! dt = %s\r\n", gimbal_start_str);
         }

/* ------------------------------ 调试监测线程调度 ------------------------------ */

         gimbal_control_task();

         vTaskDelayUntil(&gimbal_wake_time, 1);
     }
 }

 __attribute__((noreturn)) void shoot_task_entry(void const *argument)
 {
     float shoot_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] shoot Task Start\r\n");
     uint32_t shoot_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
         shoot_dt = dwt_get_time_ms() - shoot_start;
         shoot_start = dwt_get_time_ms();
         if (shoot_dt > 1.5) {
             Float2Str(shoot_dt_str,shoot_dt);
             LOGERROR("[freeRTOS] shoot Task is being DELAY! dt = %s\r\n", shoot_dt_str);
         }

/* ------------------------------ 调试监测线程调度 ------------------------------ */

         shoot_control_task();

         vTaskDelayUntil(&shoot_wake_time, 1);
     }
 }

 __attribute__((noreturn)) void ins_task_entry(void const *argument)
 {
     float ins_start = dwt_get_time_ms();
     LOGINFO("[freeRTOS] ins Task Start\r\n");
     uint32_t ins_wake_time = osKernelSysTick();
     for (;;)
     {
/* ------------------------------ 调试监测线程调度 ------------------------------ */
         ins_dt = dwt_get_time_ms() - ins_start;
         ins_start = dwt_get_time_ms();
         if (ins_dt > 1.5) {
             Float2Str(ins_dt_str,ins_dt);
             LOGERROR("[freeRTOS] ins Task is being DELAY! dt = %s\r\n", ins_dt_str);
         }

/* ------------------------------ 调试监测线程调度 ------------------------------ */
         ins_control_task();

         vTaskDelayUntil(&ins_wake_time, 1);
     }
 }

//__attribute__((noreturn)) void referee_task_entry(void const *argument)
//{
//    /* USER CODE BEGIN RefereeTask */
//    static float referee_start;
//    static uint32_t referee_dwt = 0;
//    static float dt = 0;
//    static uint32_t count = 0;
//
//    // referee_UI_task_init();
//
//    dt = dwt_get_delta(&referee_dwt);
//    referee_start = dwt_get_time_ms();
//
//    uint32_t referee_wake_time = osKernelSysTick();
//    PrintLog("[freeRTOS] Ins Task Start\n");
//    /* Infinite loop */
//    for(;;)
//    {
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//        referee_dt = dwt_get_time_ms() - referee_start;
//        referee_start = dwt_get_time_ms();
///* ------------------------------ 调试监测线程调度 ------------------------------ */
//
//        dt = dwt_get_delta(&referee_dwt);
//
//
//        vTaskDelayUntil(&referee_wake_time, 1);
//    }
//}

#endif /* _RM_TASK_H */
