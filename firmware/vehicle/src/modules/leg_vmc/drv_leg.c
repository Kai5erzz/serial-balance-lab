/*
 * Copyright (c) 2006-2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author        Notes
 * 2023-11-01     ChuShicheng   first version
 */
#include "drv_leg.h"
#include "rm_algorithm.h"
#include <math.h>



#define LEFT 0
#define RIGHT 1
static uint8_t idx = 0; // register idx,是该文件的全局轮腿索引,在注册时使用
static leg_obj_t leg_obj[2];

#define VMC_INV_MAX_JUMP_RAD 0.35f  /* 实车防止逆解目标突跳, 约20° */
#define VMC_INV_EPS 1e-6f

/* 装配模式锁定：跳过 2x2 枚举和绝对角限位，左右腿独立指定 branch */
#define VMC_INV_LOCK_ASSEMBLY_MODE        1
#define VMC_INV_LOCK_LEFT_PHI1_BRANCH     0   /* 左腿 phi1: candidate[0] */
#define VMC_INV_LOCK_LEFT_PHI4_BRANCH     1   /* 左腿 phi4: candidate[1] */
#define VMC_INV_LOCK_RIGHT_PHI1_BRANCH    0   /* 右腿 phi1: candidate[0] */
#define VMC_INV_LOCK_RIGHT_PHI4_BRANCH    1   /* 右腿 phi4: candidate[1] */

uint8_t One_Older_LPF(float input, float* lpf_output ,float Klpf)//一阶低通滤波函数
{
    if(Klpf >= 1.0f)
        return 1;
    float X0 = input;
    *lpf_output= Klpf  * (*lpf_output) + (1 - Klpf) * X0;
    return 0;
}

static void vmc_calc(leg_obj_t *leg,struct ins_msg *ins,float dt)//右侧腿部计算vmc时使用
{
    leg->l4 = leg->l1;
    leg->l3 = leg->l2;
    /* xdb = xd-xb; ydb = yd-yb; */
    leg->XD= leg->motor_distance + leg->l4 * arm_cos_f32(leg->phi4);
    leg->XB = leg->l1 * arm_cos_f32(leg->phi1);
    leg->YD = leg->l4 * arm_sin_f32(leg->phi4);
    leg->YB = leg->l1 * arm_sin_f32(leg->phi1);

    leg->lBD = sqrtf((leg->XD - leg->XB)*(leg->XD - leg->XB) + (leg->YD - leg->YB)*(leg->YD - leg->YB));

    float A0 = 2 * leg->l2 * (leg->XD - leg->XB);
    float B0 = 2 * leg->l2 * (leg->YD - leg->YB);
    float C0 = leg->l2*leg->l2 + leg->lBD*leg->lBD - leg->l3*leg->l3;
    leg->phi2 = 2.0f * atan2f(B0 + sqrtf(A0*A0 + B0*B0 - C0*C0), (A0+C0));

    /*计算C坐标*/
    leg->XC = leg->XB + leg->l2 * arm_cos_f32(leg->phi2);
    leg->YC = leg->YB + leg->l2 * arm_sin_f32(leg->phi2);

    /*计算phi3*/
    leg->phi3 = atan2f((leg->YB - leg->YD) + leg->l2 * arm_sin_f32(leg->phi2), (leg->XB - leg->XD) + leg->l2 * arm_cos_f32(leg->phi2));

    /*输出摆长摆角,即c点的极坐标*/
    leg->phi0 = atan2f(leg->YC,(leg->XC - leg->motor_distance/2.0f));
    leg->l0 = sqrtf((leg->XC - leg->motor_distance/2.0f)*(leg->XC - leg->motor_distance/2.0f) + leg->YC*leg->YC);

    leg->d_phi0 = (leg->phi0 - leg->last_phi0)/dt;


    leg->theta = PI/2.0f - leg->phi0 - (-ins->pitch* DEGREE_2_RAD);
    leg->d_theta = (leg->theta - leg->last_theta) / dt;

    leg->d_theta = (leg->theta - leg->last_theta) / dt ;

    One_Older_LPF(leg->d_theta,&leg->d_theta_lpf,0.5f);//一阶低通滤波

    leg->last_phi0 = leg->phi0;
    leg->last_theta = leg->theta;

    /*l0的一阶和二阶导数,对应腿长变化的速度和加速度*/
    leg->d_l0 = (leg->l0 - leg->last_l0)/dt;

    One_Older_LPF(leg->d_l0,&leg->d_l0_lpf,0.5f);

    leg->last_l0 = leg->l0;
    leg->last_dl0 = leg->d_l0;
    leg->last_d_theta = leg->d_theta;
    leg->last_theta = leg->theta;


}

static float vmc_norm_0_2pi(float x)
{
    while (x < 0.0f) {
        x += PI2;
    }
    while (x >= PI2) {
        x -= PI2;
    }
    return x;
}

/* Wrap to [-π, π] for DM single-turn range */
static inline float vmc_wrap_pi(float x)
{
    while (x > PI) {
        x -= PI2;
    }
    while (x <= -PI) {
        x += PI2;
    }
    return x;
}

static inline float vmc_limit_f32(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

/*
 * Convert continuous reference to a single-turn DM command.
 * DM only accepts [-π, π], so never send multi-turn target directly.
 * The command is chosen on the shortest side relative to measured motor angle.
 */
static inline float vmc_cont_ref_to_dm_single_turn(float ref_cont,
                                                    float motor_meas_single,
                                                    float max_step)
{
    float ref_single = vmc_wrap_pi(ref_cont);
    float err = vmc_wrap_pi(ref_single - motor_meas_single);
    err = vmc_limit_f32(err, -max_step, max_step);
    return vmc_wrap_pi(motor_meas_single + err);
}

static float vmc_angle_diff(float a, float b)
{
    return atan2f(arm_sin_f32(a - b), arm_cos_f32(a - b));
}

static float vmc_clampf(float x, float min_v, float max_v)
{
    if (x < min_v) {
        return min_v;
    }
    if (x > max_v) {
        return max_v;
    }
    return x;
}

static uint8_t vmc_isfinite(float x)
{
    return (x == x) && (x < 3.4e38f) && (x > -3.4e38f);
}

/*
 * 二连杆 IK。
 * 输入目标点相对该侧电机原点的坐标 x,y。
 * 输出两个候选解 q[0], q[1]。
 */
static uint8_t vmc_solve_2link_ik(float x, float y, float l1, float l2, float q[2])
{
    float r2 = x * x + y * y;
    float r = sqrtf(r2);

    if (r < VMC_INV_EPS) {
        return 0;
    }

    if (r > l1 + l2 + VMC_INV_EPS) {
        return 0;
    }

    if (r < fabsf(l1 - l2) - VMC_INV_EPS) {
        return 0;
    }

    float alpha = atan2f(y, x);
    float cos_beta = (l1 * l1 + r2 - l2 * l2) / (2.0f * l1 * r);
    cos_beta = vmc_clampf(cos_beta, -1.0f, 1.0f);

    float beta = acosf(cos_beta);

    q[0] = vmc_norm_0_2pi(alpha + beta);
    q[1] = vmc_norm_0_2pi(alpha - beta);

    return 2;
}

static uint8_t vmc_inv_candidate_valid(leg_obj_t *leg, float phi1, float phi4)
{
    if (!vmc_isfinite(phi1) || !vmc_isfinite(phi4)) {
        return 0;
    }

    if (phi1 < leg->phi1_min) {
        return 0;
    }

    if (phi4 > leg->phi4_max) {
        return 0;
    }

    return 1;
}

static void vmc_inv_hold_current(leg_obj_t *leg)
{
    leg->phi1_inv = vmc_norm_0_2pi(leg->phi1);
    leg->phi4_inv = vmc_norm_0_2pi(leg->phi4);
    /* z1/z4 保持连续空间，不归一化到 [0,2π]，避免 jump reference 空间不一致 */
    leg->phi1_z1 = leg->phi1;
    leg->phi4_z4 = leg->phi4;
}

static void vmc_calc_inv(struct leg_obj *leg, float phi0_refer, float l0_refer)
{
    leg->inv_calc_cnt++;

    float phi1_candidate[2] = {0.0f, 0.0f};
    float phi4_candidate[2] = {0.0f, 0.0f};

    leg->inv_valid = 0;
    leg->inv_fail_reason = 0;
    leg->inv_branch_phi1 = -1;
    leg->inv_branch_phi4 = -1;
    leg->inv_cost = 999.0f;
    leg->inv_jump = 999.0f;

    leg->l4 = leg->l1;
    leg->l3 = leg->l2;

    /*
     * 目标 C 点。用 cos/sin 替代 tan，避免 phi0 接近 ±90° 时数值不稳定。
     * phi0 = atan2(YC, XC - motor_distance/2)
     * l0 = sqrt((XC - motor_distance/2)^2 + YC^2)
     */
    leg->XC_inv = leg->motor_distance / 2.0f + l0_refer * arm_cos_f32(phi0_refer);
    leg->YC_inv = l0_refer * arm_sin_f32(phi0_refer);

    /* 左侧二连杆：电机原点为 (0,0) */
    uint8_t n1 = vmc_solve_2link_ik(
        leg->XC_inv,
        leg->YC_inv,
        leg->l1,
        leg->l2,
        phi1_candidate
    );

    /* 右侧二连杆：电机原点为 (motor_distance,0) */
    uint8_t n4 = vmc_solve_2link_ik(
        leg->XC_inv - leg->motor_distance,
        leg->YC_inv,
        leg->l4,
        leg->l3,
        phi4_candidate
    );

    leg->phi1_candidate[0] = phi1_candidate[0];
    leg->phi1_candidate[1] = phi1_candidate[1];
    leg->phi4_candidate[0] = phi4_candidate[0];
    leg->phi4_candidate[1] = phi4_candidate[1];

    if (n1 == 0 || n4 == 0) {
        leg->inv_fail_reason = 1; /* no_candidate: IK无实数解 */
        vmc_inv_hold_current(leg);
        leg->leg_state = LEG_ERROR;
        return;
    }

#if VMC_INV_LOCK_ASSEMBLY_MODE
    /* ===== 装配模式锁定：左右腿独立指定 branch，绕过绝对角限位 ===== */
    {
        int bi, bj;
        if (leg->side == LEFT) {
            bi = VMC_INV_LOCK_LEFT_PHI1_BRANCH;
            bj = VMC_INV_LOCK_LEFT_PHI4_BRANCH;
        } else {
            bi = VMC_INV_LOCK_RIGHT_PHI1_BRANCH;
            bj = VMC_INV_LOCK_RIGHT_PHI4_BRANCH;
        }
        float p1 = phi1_candidate[bi];
        float p4 = phi4_candidate[bj];

        if (!vmc_isfinite(p1) || !vmc_isfinite(p4)) {
            leg->inv_valid = 0;
            leg->inv_fail_reason = 1;   /* no_candidate / geometry invalid */
            leg->inv_branch_phi1 = -1;
            leg->inv_branch_phi4 = -1;
            leg->inv_jump = 999.0f;
            vmc_inv_hold_current(leg);
            leg->leg_state = LEG_ERROR;
            return;
        }

        /*
         * Jump protection checks command continuity, not mechanical tracking lag.
         * With assembly branch locked and phi0_ref ramped, the IK solution should
         * change only by one ramp step per cycle. Therefore compare the new solution
         * to the previous IK command solution phi1_z1 / phi4_z4, not to measured
         * phi1 / phi4.
         */
        float d1 = vmc_angle_diff(p1, leg->phi1_z1);
        float d4 = vmc_angle_diff(p4, leg->phi4_z4);
        float best_jump = (fabsf(d1) > fabsf(d4)) ? fabsf(d1) : fabsf(d4);

        leg->inv_branch_phi1 = (int8_t)bi;
        leg->inv_branch_phi4 = (int8_t)bj;
        leg->inv_cost = d1 * d1 + d4 * d4;
        leg->inv_jump = best_jump;

        if (best_jump > VMC_INV_MAX_JUMP_RAD) {
            leg->inv_valid = 0;
            leg->inv_fail_reason = 3;   /* jump_too_large */
            vmc_inv_hold_current(leg);
            leg->leg_state = LEG_ERROR;
            return;
        }

        leg->phi1_z1 = p1;
        leg->phi4_z4 = p4;
        leg->phi1_inv = vmc_norm_0_2pi(p1);
        leg->phi4_inv = vmc_norm_0_2pi(p4);

        leg->inv_valid = 1;
        leg->inv_fail_reason = 0; /* ok */
        leg->leg_state = LEG_NORMAL;
        return;
    }
#else
    /* ===== 原始 2x2 枚举 + 绝对角限位 ===== */
    {
        float best_cost = 1e30f;
        float best_jump = 1e30f;
        float best_phi1 = leg->phi1_inv;
        float best_phi4 = leg->phi4_inv;
        int8_t best_i = -1;
        int8_t best_j = -1;
        uint8_t joint_limit_rejected = 0;

        for (uint8_t i = 0; i < n1; i++) {
            for (uint8_t j = 0; j < n4; j++) {
                float p1 = phi1_candidate[i];
                float p4 = phi4_candidate[j];

                if (!vmc_inv_candidate_valid(leg, p1, p4)) {
                    joint_limit_rejected = 1;
                    continue;
                }

                float d1 = vmc_angle_diff(p1, leg->phi1);
                float d4 = vmc_angle_diff(p4, leg->phi4);

                float jump = fabsf(d1);
                if (fabsf(d4) > jump) {
                    jump = fabsf(d4);
                }

                float cost = d1 * d1 + d4 * d4;

                if (cost < best_cost) {
                    best_cost = cost;
                    best_jump = jump;
                    best_phi1 = p1;
                    best_phi4 = p4;
                    best_i = (int8_t)i;
                    best_j = (int8_t)j;
                }
            }
        }

        if (best_i < 0 || best_j < 0) {
            leg->inv_fail_reason = joint_limit_rejected ? 2 : 1;
            vmc_inv_hold_current(leg);
            leg->leg_state = LEG_ERROR;
            return;
        }

        leg->inv_cost = best_cost;
        leg->inv_jump = best_jump;
        leg->inv_branch_phi1 = best_i;
        leg->inv_branch_phi4 = best_j;

        if (best_jump > VMC_INV_MAX_JUMP_RAD) {
            leg->inv_fail_reason = 3;
            vmc_inv_hold_current(leg);
            leg->leg_state = LEG_ERROR;
            leg->inv_valid = 0;
            return;
        }

        leg->phi1_z1 = best_phi1;
        leg->phi4_z4 = best_phi4;
        leg->phi1_inv = vmc_norm_0_2pi(best_phi1);
        leg->phi4_inv = vmc_norm_0_2pi(best_phi4);

        leg->inv_valid = 1;
        leg->inv_fail_reason = 0;
        leg->leg_state = LEG_NORMAL;
    }
#endif
}
/*
 * @brief 通过电机角度解算phi1和phi2值
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_raw:phi1对应的电机angle_abs;
 * @param phi4_raw:phi4对应的电机angle_abs
 * */
/*这两个函数在电机位置更换时重新写!!!*/
static void phi1_phi4_calc_left(struct leg_obj *leg, float phi1_raw, float phi4_raw){

    leg->phi1 = fmodf(PI2 - (phi1_raw - DM_ZERO_OFFSET_LF * DEGREE_2_RAD),PI2) ;/*推导加实测出来的*/
    leg->phi4 = fmodf(PI2 - (phi4_raw + DM_ZERO_OFFSET_LB * DEGREE_2_RAD),PI2) ;

}

/*
 * @brief 通过vmc_inv解算phi1_inv和phi2_inv对应的原始电机输出轴角度值(rad)
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_inv:phi1对应的电机对应的原始电机输出轴角度值(rad);
 * @param phi4_inv:phi4对应的电机对应的原始电机输出轴角度值(rad)
 * */
/*这两个函数在电机位置更换时重新写!!!*/
static void phi1_phi4_calc_left_inv(struct leg_obj *leg, float phi1_inv, float phi4_inv){

    /* Raw target (continuous, not wrapped) */
    float c1 = PI2 - phi1_inv + DM_ZERO_OFFSET_LF * DEGREE_2_RAD;
    float c4 = PI2 - phi4_inv - DM_ZERO_OFFSET_LB * DEGREE_2_RAD;

    /* Maintain continuous reference using shortest-path delta */
    leg->motor_phi1_inv_position_refer += vmc_angle_diff(c1, leg->motor_phi1_inv_position_refer);
    leg->motor_phi4_inv_position_refer += vmc_angle_diff(c4, leg->motor_phi4_inv_position_refer);

}

/*
 * @brief 通过电机角度解算phi1和phi2值
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_raw:phi1对应的电机angle_abs;
 * @param phi4_raw:phi4对应的电机angle_abs
 * */
static void phi1_phi4_calc_right(struct leg_obj *leg, float phi1_raw, float phi4_raw){

    leg->phi1 = fmodf(phi1_raw + DM_ZERO_OFFSET_RF * DEGREE_2_RAD,PI2);
    leg->phi4 = fmodf(phi4_raw - DM_ZERO_OFFSET_RB * DEGREE_2_RAD,PI2);


}

/*
 * @brief 通过vmc_inv解算phi1_inv和phi2_inv对应的原始电机输出轴角度值(rad)
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_inv:phi1对应的电机对应的原始电机输出轴角度值(rad);
 * @param phi4_inv:phi4对应的电机对应的原始电机输出轴角度值(rad)
 * */
static void phi1_phi4_calc_right_inv(struct leg_obj *leg, float phi1_inv, float phi4_inv){

    /* Raw target (continuous, not wrapped) */
    float c1 = phi1_inv - DM_ZERO_OFFSET_RF * DEGREE_2_RAD;
    float c4 = phi4_inv + DM_ZERO_OFFSET_RB * DEGREE_2_RAD;

    /* Maintain continuous reference using shortest-path delta */
    leg->motor_phi1_inv_position_refer += vmc_angle_diff(c1, leg->motor_phi1_inv_position_refer);
    leg->motor_phi4_inv_position_refer += vmc_angle_diff(c4, leg->motor_phi4_inv_position_refer);

}

/*
 * @brief 计算气弹簧在不同长度情况下,通过虚功原理得到的竖直方向上的支持
 * */
static void F_Spring_to_F_Vertical(struct leg_obj *leg){

    leg->theta_3 = acosf((leg->l1 * leg->l1 + leg->l2 * leg->l2 - leg->l0 * leg->l0 ) / (2 * leg->l1 * leg->l2));
    leg->l_s = sqrtf(leg->l6 * leg->l6 + leg->l_s2 * leg->l_s2 - 2 * leg->l6 * leg->l_s2 * arm_cos_f32(leg->theta_3 - leg->alpha_s));
    leg->F_Vertical = (leg->F_Spring * leg->l0 * leg->l6 * leg->l_s2 * arm_sin_f32(leg->theta_3 - leg->alpha_s)) / (leg->l_s * leg->l1 * leg->l2 * arm_sin_f32(leg->theta_3));

}

static int8_t input_leg_angle(leg_obj_t *leg, float phi4, float phi1)
{
    leg->phi4 = phi4;
    leg->phi1 = phi1;

    if(phi4>leg->phi4_max)
    {leg->leg_state = LEG_ERROR;}
    else if(phi1<leg->phi1_min)
    {leg->leg_state = LEG_ERROR;}
    else
    {leg->leg_state = LEG_NORMAL;}

    return (uint8_t)leg->leg_state;
}

/*Leg motors*/
/*FT = [F Tp]   Torque = [Motor3Torque(backmotor)  Motor2Torque(frontmotor)] */
static void vmc_calculation_Torque(leg_obj_t *leg,float *Tmotor)
{
    /*计算VMC*/
    volatile float q00,q01,q10,q11;
    /*中间变量*/
    volatile float sin32 = arm_sin_f32(leg->phi3 - leg->phi2);
    volatile float sin12 = arm_sin_f32(leg->phi1 - leg->phi2);
    volatile float sin34 = arm_sin_f32(leg->phi3 - leg->phi4);


    q00 = leg->l1 * arm_sin_f32(leg->phi0 - leg->phi3) * sin12 / sin32;
    q01 = leg->l1 * arm_cos_f32(leg->phi0 - leg->phi3) * sin12 / (leg->l0 *sin32);
    q10 = leg->l4 * arm_sin_f32(leg->phi0 - leg->phi2) * sin34 / sin32;
    q11 = leg->l4 * arm_cos_f32(leg->phi0 - leg->phi2) * sin34 / (leg->l0 *sin32);

    /*矩阵乘法,T1,T2,F,Tp*/
    Tmotor[0] = q00*leg->support_force + q01*leg->Tp;
    Tmotor[1] = q10*leg->support_force + q11*leg->Tp;
}

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return leg_obj_t* 轮腿实例指针
 */
leg_obj_t * leg_register(leg_config_t *config/* , void *control */)
{
    leg_obj[idx].side = idx;  /* 0=LEFT, 1=RIGHT */
    leg_obj[idx].l1 = config->l1;
    leg_obj[idx].l2 = config->l2;
    leg_obj[idx].motor_distance = config->motor_distance;
    leg_obj[idx].F_Spring = config->F_Spring;
    leg_obj[idx].l_s2 = config->l_s2;
    leg_obj[idx].l6 = config->l6;
    leg_obj[idx].alpha_s = config->alpha_s;
    leg_obj[idx].phi4_max = PI/2;
    leg_obj[idx].phi1_min = PI/2;
    leg_obj[idx].leg_state = LEG_ERROR;
    leg_obj[idx].phi0 = PI/2;
    leg_obj[idx].phi1_inv = 0.0f;
    leg_obj[idx].phi4_inv = 0.0f;
    leg_obj[idx].phi1_candidate[0] = 0.0f;
    leg_obj[idx].phi1_candidate[1] = 0.0f;
    leg_obj[idx].phi4_candidate[0] = 0.0f;
    leg_obj[idx].phi4_candidate[1] = 0.0f;
    leg_obj[idx].inv_cost = 0.0f;
    leg_obj[idx].inv_jump = 0.0f;
    leg_obj[idx].inv_branch_phi1 = -1;
    leg_obj[idx].inv_branch_phi4 = -1;
    leg_obj[idx].inv_valid = 0;
    leg_obj[idx].inv_fail_reason = 0;
    leg_obj[idx].inv_calc_cnt = 0;
    leg_obj[idx].vmc_calc = vmc_calc;
    leg_obj[idx].vmc_calc_inv = vmc_calc_inv;
    leg_obj[idx].phi1_phi4_calc_left_inv = phi1_phi4_calc_left_inv;
    leg_obj[idx].phi1_phi4_calc_right_inv = phi1_phi4_calc_right_inv;
    leg_obj[idx].vmc_cal_T = vmc_calculation_Torque;
    leg_obj[idx].phi_calc_L = phi1_phi4_calc_left ;
    leg_obj[idx].phi_calc_R = phi1_phi4_calc_right ;
    leg_obj[idx].F_Spring_to_F_Vertical = F_Spring_to_F_Vertical;
    leg_obj[idx].input_leg_angle = input_leg_angle;

    return &leg_obj[idx++];
}

