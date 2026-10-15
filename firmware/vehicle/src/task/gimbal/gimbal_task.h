//
// Created by Gleam on 25-8-22.
//

#ifndef CTRBOARD_H7_ALL_GIMBAL_TASK_H
#define CTRBOARD_H7_ALL_GIMBAL_TASK_H

#include <stdint.h>

#define GIMBAL_WX_MAX        270.0f
#define GIMBAL_YAW_TURN_RATIO  0.2f  //单位应该为°,有关调节遥控器转向敏感度的系数,自行在安全范围内调节大小
#define GIMBAL_PITCH_TURN_RATIO  0.2f

void gimbal_control_task();
void gimbal_task_init(void);
uint8_t gimbal_fill_debug_page(uint8_t page);
void gimbal_fill_all_debug(uint8_t first_channel);
#endif //CTRBOARD_H7_ALL_GIMBAL_TASK_H
