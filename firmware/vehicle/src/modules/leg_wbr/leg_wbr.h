//
// Created by SuperChen on 2025/11/9.
//

#ifndef HNU_RM_DOWN_LEG_WBR_H
#define HNU_RM_DOWN_LEG_WBR_H

#include "rm_module.h"

enum wbr_leg_state_e
{
    WBR_LEG_NORMAL	= 0,
    WBR_LEG_ERROR	= 1
};

typedef struct wbr_leg_obj
{
    /*单位m*/
    float L1u,L2u;  // L1u=L2u,与关节电机连接一侧杆长
    float L1d,L2d; // L1d=L2d,与轮毂电机连接一侧杆长
    float L1a,L2a; //电机间距,串腿则默认为0

    float phi1,phi2; //对应上交开源中的φ1和φ2

    /*极限值*/
    float phi1_min;
    float phi1_max;
    float phi2_min;
    float phi2_max;

    /*杆状态*/
    uint8_t wbr_leg_state;

    /*倒立摆长度*/
    float L;
    float L_average;
    float last_L;
    /*倒立摆长度速度*/
    float d_L;
    /*上一次倒立摆长度速度*/
    float last_d_L;
    /*倒立摆长度加速度*/
    float dd_L;

    /*倒立摆角度*/
    float q;
    float d_q;
    float last_q;

    /*腿部wbr逆解算变量*/
    float X1;
    float Y1;
    float X2;
    float Y2;
    float Xe;
    float Ye;
    float sigma1;

    /*雅可比元素*/
    float q1_L,q1_q,q2_L,q2_q;

    /*雅可比矩阵*/
    float a,b,c,d; //雅可比矩阵元素
    float detA;
    float J_inv[2][2];  //雅可比逆矩阵
    float J_inv_T[2][2];  //雅可比逆矩阵的转置矩阵

    /*wbr摆杆力矩Tp*/
    float WBR_Tbl;

    /*倒立摆期望长度*/
    float wbr_L_ref;

    /*theta用于之后计算lqr,为状态矩阵x中的状态量*/
    float wbr_theta;
    float  thetab;  //机体倾角,沿机体正方向前倾为正
    /*d_theta用于之后计算lqr,为状态矩阵x中的状态量*/
    float wbr_d_theta;
    float wbr_dd_theta;
    float last_d_wbr_theta;
    float last_wbr_theta;

    /*腿支持力*/
    float wbr_Fbl;


    /* 受到地面的支持力（用于离地检测） */
    float wbr_support_force;
    /* 接触地面状态量 */
    uint8_t wbr_touch_ground;
    /* 一段时间未接触地面，进入离地状态（离地飞行） */
    uint8_t wbr_fly_flag;

    /* wbr_calc 输入q1和q2解算出腿部的q以及L,以及他们的速度加速度*/
    void (*wbr_calc)(struct wbr_leg_obj *leg,struct ins_msg *ins,float dt);

    /* wbr_cal_T 输入支持力和摆杆力矩,解算出关节电机的输入扭矩 */
    void (*wbr_cal_T)(struct wbr_leg_obj *leg,float *WBR_T);

    /*解算腿部的phi1和phi2值,方便后续计算*/
    void (*phi_calc_L)(struct wbr_leg_obj *leg, float phi1_raw, float phi2_raw);
    /*解算腿部的phi1和phi2值,方便后续计算*/
    void (*phi_calc_R)(struct wbr_leg_obj *leg, float phi1_raw, float phi2_raw);
    /*获取q1和q2的角度值,由关节电机读取*/
    int8_t (*input_wbr_leg_angle)(struct wbr_leg_obj *leg, float q1, float q2);

}wbr_leg_obj_t;

typedef struct
{
    /*单位mm*/
    float L1u;  // L1u = L2u
    float L2u;
    float L1d; // L1d = L2d
    float L2d;
    float L1a; //电机间距
    float L2a;
    /* 后续可能需要加入更多参数 */

}wbr_leg_config_t;

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return wbr_leg_obj_t* wbr建模轮腿实例指针
 */
wbr_leg_obj_t * wbr_leg_register(wbr_leg_config_t *config);

#endif //HNU_RM_DOWN_LEG_WBR_H
