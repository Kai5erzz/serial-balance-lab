 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-08-28      ChuShicheng     first version
 */
#include "robot.h"
#include "rm_config.h"
#include "rm_module.h"
#include "rm_task.h"


/* 同一个主题名称不可被重复定义 */
MCN_DEFINE(chassis_cmd, sizeof(struct chassis_cmd_msg));
MCN_DEFINE(chassis_fdb, sizeof(struct chassis_fdb_msg));
MCN_DEFINE(ins_topic, sizeof(struct ins_msg));
MCN_DEFINE(gimbal_ins_topic, sizeof(struct dm_imu_t));
MCN_DEFINE(trans_fdb,sizeof(struct trans_fdb_msg));
MCN_DEFINE(gimbal_cmd,sizeof(struct gimbal_cmd_msg));
MCN_DEFINE(shoot_cmd,sizeof(struct shoot_cmd_msg));
MCN_DEFINE(gimbal_fdb,sizeof(struct gimbal_fdb_msg));
MCN_DEFINE(shoot_fdb,sizeof(struct shoot_fdb_msg));
MCN_DEFINE(transmission_fdb,sizeof(struct trans_fdb_msg));

static void mcn_topic_init(void);

void robot_init()
{  
    // 关闭中断,防止在初始化过程中发生中断
    // 请不要在初始化过程中使用中断和延时函数！
    // 若必须,则只允许使用 dwt 进行延时
    __disable_irq();


    OS_task_init(); // 创建基础任务

    mcn_topic_init(); // 话题注册初始化

    chassis_task_init();
    motor_task_init();
    cmd_task_init();
    gimbal_task_init();
    trans_task_init();
    shoot_task_init();
    ins_task_init();

    // 初始化完成,开启中断
    __enable_irq();
}

/**
 * @brief ipc uMCN 各话题注册
 * 
 */
static void mcn_topic_init(void)
{
    mcn_advertise(MCN_HUB(ins_topic), NULL);
    mcn_advertise(MCN_HUB(gimbal_ins_topic), NULL);
    mcn_advertise(MCN_HUB(chassis_cmd), NULL);
    mcn_advertise(MCN_HUB(chassis_fdb), NULL);
    mcn_advertise(MCN_HUB(trans_fdb), NULL);
    mcn_advertise(MCN_HUB(gimbal_cmd), NULL);
    mcn_advertise(MCN_HUB(shoot_cmd), NULL);
    mcn_advertise(MCN_HUB(gimbal_fdb), NULL);
    mcn_advertise(MCN_HUB(shoot_fdb), NULL);
    mcn_advertise(MCN_HUB(transmission_fdb), NULL);
}
