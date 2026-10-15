//
// Created by Gleam on 25-8-22.
//

#ifndef CTRBOARD_H7_ALL_SHOOT_TASK_H
#define CTRBOARD_H7_ALL_SHOOT_TASK_H

/**
  * @brief 单发和连发角度继承
  */
typedef enum
{
    SHOOT_ANGLE_CONTINUE=0,   //角度为连发状态
    SHOOT_ANGLE_SINGLE=1,  //角度为单发状态
} shoot_angle_inherit_e;

typedef enum {
    REGULAR=0,
    RISK=1,
    STOP=2
    //还可添加临界状态表示
}risk_level_e;

void shoot_task_init();
void shoot_control_task();

#endif //CTRBOARD_H7_ALL_SHOOT_TASK_H
