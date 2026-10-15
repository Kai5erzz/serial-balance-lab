//
// Created by SuperChen on 2025/11/9.
//

#include "leg_wbr.h"
#include "arm_math.h"


#define LEFT 0
#define RIGHT 1
static uint8_t idx = 0; // register idx,是该文件的全局轮腿索引,在注册时使用
static wbr_leg_obj_t wbr_obj[2];




static void wbr_calc(struct wbr_leg_obj *leg,struct ins_msg *ins,float dt){

    leg->thetab = ins->pitch * DEGREE_2_RAD;//机体倾角,沿机体正方向前倾为正,注意正负号,实测需要修正

    leg->X1 = leg->L1a - leg->L1u * arm_cos_f32(leg->phi1);
    leg->X2 = leg->L2a - leg->L2u * arm_cos_f32(leg->phi2);
    leg->Y1 = leg->L1u * arm_sin_f32(leg->phi1);
    leg->Y2 = leg->L2u * arm_sin_f32(leg->phi2);
    leg->sigma1 = sqrt(-(-leg->L1d*leg->L1d-2*leg->L1d*leg->L2d-leg->L2d*leg->L2d+leg->X1*leg->X1+2*leg->X1*leg->X2+leg->X2*leg->X2+leg->Y1*leg->Y1-2*leg->Y1*leg->Y2+leg->Y2*leg->Y2) * (-leg->L1d*leg->L1d+2*leg->L1d*leg->L2d-leg->L2d*leg->L2d+leg->X1*leg->X1+2*leg->X1*leg->X2+leg->X2*leg->X2+leg->Y1*leg->Y1-2*leg->Y1*leg->Y2+leg->Y2*leg->Y2));
    leg->Xe = (leg->Y1*leg->sigma1-leg->Y2*leg->sigma1+leg->L1d*leg->L1d*leg->X1+leg->L1d*leg->L1d*leg->X2-leg->L2d*leg->L2d*leg->X1-leg->L2d*leg->L2d*leg->X2+leg->X1*leg->X2*leg->X2-leg->X1*leg->X1*leg->X2-leg->X1*leg->Y1*leg->Y1-leg->X1*leg->Y2*leg->Y2+leg->X2*leg->Y1*leg->Y1+leg->X2*leg->Y2*leg->Y2-leg->X1*leg->X1*leg->X1+leg->X2*leg->X2*leg->X2+2*leg->X1*leg->Y1*leg->Y2-2*leg->X2*leg->Y1*leg->Y2) / (2*(leg->X1*leg->X1+2*leg->X1*leg->X2+leg->X2*leg->X2+leg->Y1*leg->Y1-2*leg->Y1*leg->Y2+leg->Y2*leg->Y2));
    leg->Ye = (leg->X1*leg->sigma1+leg->X2*leg->sigma1-leg->L1d*leg->L1d*leg->Y1+leg->L1d*leg->L1d*leg->Y2+leg->L2d*leg->L2d*leg->Y1-leg->L2d*leg->L2d*leg->Y2+leg->X1*leg->X1*leg->Y1+leg->X1*leg->X1*leg->Y2+leg->X2*leg->X2*leg->Y1+leg->X2*leg->X2*leg->Y2-leg->Y1*leg->Y2*leg->Y2-leg->Y1*leg->Y1*leg->Y2+leg->Y1*leg->Y1*leg->Y1+leg->Y2*leg->Y2*leg->Y2+2*leg->X1*leg->X2*leg->Y1+2*leg->X1*leg->X2*leg->Y2) / (2*(leg->X1*leg->X1+2*leg->X1*leg->X2+leg->X2*leg->X2+leg->Y1*leg->Y1-2*leg->Y1*leg->Y2+leg->Y2*leg->Y2));
    leg->L = sqrt(leg->Xe*leg->Xe + leg->Ye*leg->Ye);
    leg->q = atan2f(leg->Xe / leg->L,leg->Ye / leg->L);

    leg->d_L = (leg->L - leg->last_L)/dt;
    leg->dd_L = (leg->d_L - leg->last_d_L)/dt;
    leg->d_q = (leg->q - leg->last_q)/dt;
    leg->wbr_theta = leg->q + leg->thetab;
    leg->wbr_d_theta = (leg->wbr_theta - leg->last_wbr_theta)/dt;
    leg->wbr_dd_theta = (leg->wbr_d_theta - leg->last_d_wbr_theta)/dt;

    leg->last_L = leg->L;
    leg->last_d_L = leg->d_L;
    leg->last_q = leg->q;
    leg->last_wbr_theta = leg->wbr_theta;
    leg->last_d_wbr_theta = leg->wbr_d_theta;

}

static void wbr_calculation_Torque(struct wbr_leg_obj *leg,float *WBR_T){

    //雅可比矩阵元素计算
    leg->q1_L = ((leg->Xe+leg->X1)*arm_sin_f32(leg->q) + (leg->Ye-leg->Y1)*arm_cos_f32(leg->q))/(leg->L1u*(-(leg->Xe+leg->X1)*arm_sin_f32(leg->phi1) + (leg->Ye-leg->Y1)*arm_cos_f32(leg->phi1)));
    leg->q1_q = (leg->L*((leg->Xe+leg->X1)*arm_cos_f32(leg->q) - (leg->Ye-leg->Y1)*arm_sin_f32(leg->q)))/(leg->L1u*(-(leg->Xe+leg->X1)*arm_sin_f32(leg->phi1) + (leg->Ye-leg->Y1)*arm_cos_f32(leg->phi1)));
    leg->q2_L = ((leg->Xe-leg->X2)*arm_sin_f32(leg->q) + (leg->Ye-leg->Y2)*arm_cos_f32(leg->q))/(leg->L2u*((leg->Xe-leg->X2)*arm_sin_f32(leg->phi2) + (leg->Ye-leg->Y2)*arm_cos_f32(leg->phi2)));
    leg->q2_q = (leg->L*((leg->Xe-leg->X2)*arm_cos_f32(leg->q) - (leg->Ye-leg->Y2)*arm_sin_f32(leg->q)))/(leg->L2u*((leg->Xe-leg->X2)*arm_sin_f32(leg->phi2) + (leg->Ye-leg->Y2)*arm_cos_f32(leg->phi2)));

    //定义矩阵A的元素(对角线)

    leg->a = leg->q1_L;
    leg->b = leg->q1_q;
    leg->c = leg->q2_L;
    leg->d = leg->q2_q;

    //计算行列式

    leg->detA = leg->a*leg->d - leg->b*leg->c;

    if(leg->detA != 0){
        leg->J_inv[0][0] = leg->d / leg->detA;
        leg->J_inv[0][1] = -leg->b / leg->detA;
        leg->J_inv[1][0] = -leg->c / leg->detA;
        leg->J_inv[1][1] = leg->a / leg->detA;

        leg->J_inv_T[0][0] = leg->J_inv[0][0];
        leg->J_inv_T[0][1] = leg->J_inv[1][0];
        leg->J_inv_T[1][0] = leg->J_inv[0][1];
        leg->J_inv_T[1][1] = leg->J_inv[1][1];

    }

    WBR_T[0] = leg->J_inv_T[0][0] * leg->wbr_Fbl + leg->J_inv_T[0][1] * leg->WBR_Tbl ;
    WBR_T[1] = leg->J_inv_T[1][0] * leg->wbr_Fbl + leg->J_inv_T[1][1] * leg->WBR_Tbl;
}

static int8_t input_wbr_leg_angle(struct wbr_leg_obj *leg, float phi1, float phi2){

    leg->phi1 = phi1;
    leg->phi2 = phi2;

    if(phi1 < leg->phi1_max && phi1 > leg->phi1_min){
        if(phi2 < leg->phi2_max && phi2 > leg->phi2_min){
            leg->wbr_leg_state = WBR_LEG_NORMAL;
        }else{
            leg->wbr_leg_state = WBR_LEG_ERROR;
        }
    }else{
        leg->wbr_leg_state = WBR_LEG_ERROR;
    }


    return (uint8_t)leg->wbr_leg_state;
}

/*
 * @brief 通过电机角度解算phi1和phi2值
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_raw:phi1对应的电机angle_abs;
 * @param phi2_raw:phi2对应的电机angle_abs
 * */
/*这两个函数在电机位置更换时重新写!!!*/
static void phi1_phi2_calc_left(struct wbr_leg_obj *leg, float phi1_raw, float phi2_raw){

    leg->phi1 = fmod(PI2 - (phi1_raw - DM_ZERO_OFFSET_LF * DEGREE_2_RAD),PI2) ;
    leg->phi2 = fmod(PI - (PI2 - (phi2_raw + DM_ZERO_OFFSET_LB * DEGREE_2_RAD)),PI2) ;

}
/*
 * @brief 通过电机角度解算phi1和phi2值
 * @param struct wbr_leg_obj *leg:腿部实例;
 * @param phi1_raw:phi1对应的电机angle_abs;
 * @param phi2_raw:phi2对应的电机angle_abs
 * */
static void phi1_phi2_calc_right(struct wbr_leg_obj *leg, float phi1_raw, float phi2_raw){

    leg->phi1 = fmod((phi1_raw + DM_ZERO_OFFSET_RF * DEGREE_2_RAD),PI2);
    leg->phi2 = fmod(PI2 - (phi2_raw - DM_ZERO_OFFSET_RB * DEGREE_2_RAD) + PI,PI2);

}

/**
 * @brief 轮腿初始化,返回一个轮腿实例
 * @param config 轮腿配置
 * @return wbr_leg_obj_t* wbr建模轮腿实例指针
 */
wbr_leg_obj_t * wbr_leg_register(wbr_leg_config_t *config)
{
    wbr_obj[idx].L1u = config->L1u;
    wbr_obj[idx].L2u = config->L2u;
    wbr_obj[idx].L1d = config->L1d;
    wbr_obj[idx].L2d = config->L2d;
    wbr_obj[idx].L1a = config->L1a;
    wbr_obj[idx].L2a = config->L2a;
    wbr_obj[idx].phi1_max = PI;    //根据实际修改
    wbr_obj[idx].phi1_min = 0;
    wbr_obj[idx].phi2_max = PI;
    wbr_obj[idx].phi2_min = 0;
    wbr_obj[idx].wbr_leg_state = WBR_LEG_ERROR;
    wbr_obj[idx].q = 0;
    wbr_obj[idx].wbr_calc = wbr_calc;
    wbr_obj[idx].wbr_cal_T = wbr_calculation_Torque;
    wbr_obj[idx].phi_calc_L = phi1_phi2_calc_left ;
    wbr_obj[idx].phi_calc_R = phi1_phi2_calc_right ;
    wbr_obj[idx].input_wbr_leg_angle = input_wbr_leg_angle;

    return &wbr_obj[idx++];
}