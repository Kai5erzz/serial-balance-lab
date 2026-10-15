//
// Created by turboDog on 2021/11/21.
//

#ifndef _RAMP_H
#define _RAMP_H

#include "rm_config.h"

#define RAMP_NUM_MAX 30      // 最大RAMP实例数

typedef struct ramp_obj
{
    int32_t count;  // 计数器
    int32_t scale;  // 控制数据变化斜率
    float   out;    // 斜坡控制计算输出
    void  (*reset)(struct ramp_obj* ramp, int32_t count,int32_t scale);  // 斜坡控制实例重置
    float (*calc)(struct ramp_obj* ramp);  // 斜坡控制实例计算
}ramp_obj_t;


/**
  * @brief     斜坡控制实例重置
  * @param[in] ramp: 斜坡实例指针
  * @param[in] scale: 控制数据变化斜率
  */
void ramp_reset(ramp_obj_t *ramp, int32_t count,int32_t scale);

/**
  * @brief     斜坡控制计算函数
  * @param[in] ramp: 斜坡实例指针
  * @retval    斜坡控制计算输出
  */
float ramp_calc(ramp_obj_t *ramp);


/**
 * @brief 初始化ramp实例,并返回ramp实例指针
 * @param config PID初始化设置
 */
ramp_obj_t *ramp_register(int32_t count, int32_t scale);
/*
 * @brief 斜坡函数
 * @param target目标值
 * @param set 斜坡过程值
 * @param acc 斜坡坡度
 *
 * */
void slope_following(const float *target,float *set,float acc);
/*
 * @brief 斜坡函数
 * @param target目标值
 * @param measure 测量值
 * @param set 斜坡过程值
 * @param acc 斜坡坡度
 * */
void slope_following_begin_end(const float *target,const float *measure,float *set,float acc);

#endif //_RAMP_H
