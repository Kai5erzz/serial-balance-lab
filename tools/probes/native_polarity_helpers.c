/* Read-only diagnostic: run the original registered leg function pointers on
 * a local copy. No controller state or production control law is changed. */
#include "drv_leg.h"
#include "rm_config.h"
#include <math.h>

void audit_constants(float result[5])
{
    result[0] = M3508_READUCTION_RATIO_L;
    result[1] = M3508_READUCTION_RATIO_R;
    result[2] = RPM_2_RAD_PER_SEC;
    result[3] = GIMBAL_YAW_DRIVE_SIGN;
    result[4] = GEAR_RATIO;
}

void audit_vmc(const leg_obj_t *source, float front, float back, float pitch,
               float force, float tp, float result[11])
{
    leg_obj_t leg = *source;
    struct ins_msg ins = {0};
    ins.pitch = pitch;
    if (leg.side == 0) leg.phi_calc_L(&leg, front, back);
    else leg.phi_calc_R(&leg, front, back);
    leg.vmc_calc(&leg, &ins, 0.001f);
    result[0] = leg.l0;
    result[1] = leg.phi0;
    result[2] = leg.theta;
    result[3] = leg.phi1;
    result[4] = leg.phi4;
    leg.support_force = 1.0f;
    leg.Tp = 0.0f;
    float torque[2];
    leg.vmc_cal_T(&leg, torque);
    result[5] = torque[0];
    result[7] = torque[1];
    leg.support_force = 0.0f;
    leg.Tp = 1.0f;
    leg.vmc_cal_T(&leg, torque);
    result[6] = torque[0];
    result[8] = torque[1];
    leg.support_force = force;
    leg.Tp = tp;
    leg.vmc_cal_T(&leg, torque);
    result[9] = torque[0];
    result[10] = torque[1];
}
