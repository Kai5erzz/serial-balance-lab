/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-11-01     ChuShicheng   first version
 */
#ifndef _DRV_LEG_H
#define _DRV_LEG_H

#include "rm_module.h"



enum leg_state_e
{
	LEG_NORMAL	= 0,
	LEG_ERROR	= 1
};

typedef struct leg_obj
{
	/*单位m*/
	uint8_t side;              /* 0=LEFT, 1=RIGHT，用于 IK 分支左右独立配置 */
	float l1,l4;  // l4=l1,与关节电机连接一侧杆长
	float l2,l3; // l3=l2,与轮毂电机连接一侧杆长
    float lBD;
	float motor_distance; //电机间距

	float phi1,phi4;
	float phi2,phi3;

    float phi1_z1,phi4_z4;/*phi1_inv和phi4_inv原始解,分为-b+-两种情况*/
    float phi1_inv,phi4_inv;/*更加l0和theta逆解出来phi1和phi4*/
    float motor_phi1_inv_position_refer,motor_phi4_inv_position_refer;/*vmc逆解算出的phi1和phi4继续逆解出电机实际要转到的输出轴位置(rad)*/
    float a,b,c;/*二阶方程求解公式常规参数*/
    /*倒立摆坐标逆解*/
    float XC_inv,YC_inv;

    /* VMC逆解调试字段 */
    float phi1_candidate[2];       /* 左侧二连杆两个候选解 */
    float phi4_candidate[2];       /* 右侧二连杆两个候选解 */
    float inv_cost;                /* 最优候选解相对当前实测角的连续性代价 */
    float inv_jump;                /* 最优候选解相对当前实测角的最大单关节跳变量 */
    int8_t inv_branch_phi1;        /* 当前选择的phi1候选分支编号 */
    int8_t inv_branch_phi4;        /* 当前选择的phi4候选分支编号 */
    uint8_t inv_valid;             /* 本次逆解是否有效 */
    int8_t inv_fail_reason;        /* IK失败原因: 0=ok, 1=no_candidate, 2=joint_limit, 3=jump_too_large, 4=geometry_invalid */
    uint32_t inv_calc_cnt;         /* vmc_calc_inv调用计数 */


	/*极限值*/
	float phi1_min; // PI/2
	float phi4_max; // PI/2
	/*杆状态*/
	int8_t leg_state;
	
	/*倒立摆长度*/
	float l0;
    float l0_average;
    float last_l0;

    /*用于计算LQR反馈矩阵K*/
    float l0_pow3;  //摆长三次方,下面同理
    float l0_pow2;

	/*倒立摆角度*/
	float phi0;
	/*倒立摆坐标*/
	float XC,YC;
	/*第二象限节点坐标*/
	float XB,YB;
	/*第二象限节点坐标*/
	float XD,YD;

    /*倒立摆期望长度*/
	float length_ref;
    /*倒立摆长度速度*/
    float d_l0;
    float d_l0_lpf;//一阶低通滤波值
    /*上一次倒立摆长度速度*/
    float last_dl0;

    /*倒立摆角度速度*/
    float d_phi0;
    float last_phi0;

    /*theta用于之后计算lqr,为状态矩阵x中的状态量*/
    float theta;
    /*d_theta用于之后计算lqr,为状态矩阵x中的状态量*/
    float d_theta;
    float d_theta_lpf;//一阶低通滤波值
    float last_d_theta;
    float last_theta;

    /*摆杆力矩*/
    float Tp;

    /*气弹簧分析部分*/
    float l_s; /*气弹簧长度*/
    float l_s2;/*气弹簧在l2上安装孔位到膝关节的距离*/
    float l6;/*气弹簧在l1处端点到膝关节的距离*/
    float theta_3;/*l1与l2在膝关节处的夹角*/
    float alpha_s;/*气弹簧在l1的安装孔和膝关节连线与l1的夹角,rad*/
    float F_Spring;/*气弹簧压力*/
    float F_Vertical;/*气弹簧映射到竖直方向的力*/

    /* 受到地面的支持力（用于离地检测） */
    float support_force;
    /* 接触地面状态量 */
    uint8_t touch_ground;
    /* 一段时间未接触地面，进入离地状态（离地飞行） */
    uint8_t fly_flag;

    /* vmc计算l0,d_lo,dd_l0,theta,d_theta */
    void (*vmc_calc)(struct leg_obj *leg,struct ins_msg *ins,float dt);
    /*vmc逆解算,通过目标phi0和l0解算出phi1和phi4*/
    void (*vmc_calc_inv)(struct leg_obj *leg,float phi0_refer,float l0_refer);
    /*通过vmc_inv解算phi1_inv和phi4_inv对应的原始电机输出轴角度值(rad)*/
    void (*phi1_phi4_calc_left_inv)(struct leg_obj *leg, float phi1_inv, float phi2_inv);
    /*通过vmc_inv解算phi1_inv和phi4_inv对应的原始电机输出轴角度值(rad)*/
    void (*phi1_phi4_calc_right_inv)(struct leg_obj *leg, float phi1_inv, float phi2_inv);

    /* FT = [PendulumForce PendulumTorque] */
	void (*vmc_cal_T)(struct leg_obj *leg,float *Tmotor);

    /*解算腿部的phi1和phi2值,方便后续计算*/
    void (*phi_calc_L)(struct leg_obj *leg, float phi1_raw, float phi4_raw);
    /*解算腿部的phi1和phi2值,方便后续计算*/
    void (*phi_calc_R)(struct leg_obj *leg, float phi1_raw, float phi4_raw);
    /*解算气弹簧通过虚功原理到竖直方向的支持力*/
    void (*F_Spring_to_F_Vertical)(struct leg_obj *leg);

	int8_t (*input_leg_angle)(struct leg_obj *leg, float phi4, float phi1);
}leg_obj_t;

typedef struct
{
	/*单位mm*/
	float l1;  // l4=l1
	float l2; // l3=l2
	float motor_distance; //电机间距
    float F_Spring;/*气弹簧压力*/
    float l_s2;/*气弹簧在l2上安装孔位到膝关节的距离*/
    float l6;/*气弹簧在l1处端点到膝关节的距离*/
    float alpha_s;/*气弹簧在l1的安装孔和膝关节连线与l1的夹角,rad*/
    /* 后续可能需要加入更多参数 */
}leg_config_t;

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return leg_obj_t* 轮腿实例指针
 */
leg_obj_t * leg_register(leg_config_t *config);

#endif //_DRV_LEG_H
