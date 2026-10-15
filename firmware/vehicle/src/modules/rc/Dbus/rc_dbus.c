// Tip: 遥控器接收模块
#include "rc_dbus.h"
#include <string.h>
#include "stdlib.h"
#include "rm_algorithm.h"

uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM] __attribute__((section(".sram_dma")));

/**
 * @brief remote control structure variable
 */
Remote_Info_Typedef remote_ctrl={
        .online_cnt = 0xFAU,
        .rc_lost = true,
};

/*****************************键鼠相关**************************************/
int16_t delta_spd_vx =  KEY_ACCEL_VX_RATIO;
int16_t delta_spd_vy =  KEY_ACCEL_VY_RATIO;


kb_control_t key_board_ctrl;
/*******************************************************************/

/**
  * @brief  convert the remote control received message
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef  *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;

    /* Channel 0, 1, 2, 3 */
    remote_ctrl->rc.ch[0] = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->rc.ch[1] = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->rc.ch[2] = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->rc.ch[3] = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->rc.ch[4] = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4

    /* Switch left, right */
    remote_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
    remote_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;             //!< Switch right

    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis

    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press

    /* KeyBoard value */
    remote_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value

    remote_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    remote_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;

    /* reset the online count */
    remote_ctrl->online_cnt = 0xFAU;

    /* reset the lost flag */
    remote_ctrl->rc_lost = false;
}


/**
  * @brief  clear the remote control data while the device offline
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void Remote_Message_Moniter(Remote_Info_Typedef  *remote_ctrl)
{
    /* Juege the device status */
    if(remote_ctrl->online_cnt <= 0x32U)
    {
        /* clear the data */
        memset(remote_ctrl,0,sizeof(Remote_Info_Typedef));

        /* reset the online count */

        /* set the lost flag */
        remote_ctrl->rc_lost = true;

    }
    else if(remote_ctrl->online_cnt > 0)
    {
        /* online count decrements which reseted in received interrupt  */
        remote_ctrl->online_cnt--;
    }
}

/*****************************键鼠信息处理***************************************/
/**
  * @brief     鼠标按键状态机
  * @param[in] sta: 按键状态指针
  * @param[in] key: 按键键值
  */
static void key_fsm(kb_state_e *sta, uint8_t key)
{
    switch (*sta)
    {
        case KEY_RELEASE:
        {
            if (key)
                *sta = KEY_WAIT_EFFECTIVE;
            else
                *sta = KEY_RELEASE;
        }break;

        case KEY_WAIT_EFFECTIVE:
        {
            if (key)
                *sta = KEY_PRESS_ONCE;
            else
                *sta = KEY_RELEASE;
        }break;


        case KEY_PRESS_ONCE:
        {
            if (key)
            {
                *sta = KEY_PRESS_DOWN;
                if (sta == &key_board_ctrl.lk_sta)
                    key_board_ctrl.lk_cnt = 0;
                else
                    key_board_ctrl.rk_cnt = 0;
            }
            else
                *sta = KEY_RELEASE;
        }break;

        case KEY_PRESS_DOWN:
        {
            if (key)
            {
                if (sta == &key_board_ctrl.lk_sta)
                {
                    if (key_board_ctrl.lk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
                        *sta = KEY_PRESS_LONG;
                }
                else
                {
                    if (key_board_ctrl.rk_cnt++ > LONG_PRESS_TIME/GIMBAL_PERIOD)
                        *sta = KEY_PRESS_LONG;
                }
            }
            else
                *sta = KEY_RELEASE;
        }break;

        case KEY_PRESS_LONG:
        {
            if (!key)
            {
                *sta = KEY_RELEASE;
            }
        }break;

        default:
            break;

    }
}

/**
  * @brief     PC 处理键盘鼠标数据函数
  */
void PC_Handle_kb(void)
{
    if (remote_ctrl.key.set.SHIFT)
    {
        key_board_ctrl.move_mode = FAST_MODE;
        key_board_ctrl.max_spd = 3.5f;
    }
    else if (remote_ctrl.key.set.CTRL)
    {
        key_board_ctrl.move_mode = SLOW_MODE;
        key_board_ctrl.max_spd = 1.0f;
    }
    else
    {
        key_board_ctrl.move_mode = NORMAL_MODE;
        key_board_ctrl.max_spd = 2.0f;
    }

    //add ramp
    if (remote_ctrl.key.set.W)
        key_board_ctrl.vy += (float)delta_spd_vy;
    else if (remote_ctrl.key.set.S)
        key_board_ctrl.vy -= (float)delta_spd_vy;
    else
    {
        key_board_ctrl.vy =(float)key_board_ctrl.vy;
    }

    if (remote_ctrl.key.set.A)
        key_board_ctrl.vx -= (float)delta_spd_vx;
    else if (remote_ctrl.key.set.D)
        key_board_ctrl.vx += (float)delta_spd_vx;
    else
    {
        key_board_ctrl.vx = (float) key_board_ctrl.vx;
    }

    VAL_LIMIT(key_board_ctrl.vx, -key_board_ctrl.max_spd, key_board_ctrl.max_spd);
    VAL_LIMIT(key_board_ctrl.vy, -key_board_ctrl.max_spd, key_board_ctrl.max_spd);


    VAL_LIMIT(key_board_ctrl.vx, -MAX_CHASSIS_VX_SPEED, MAX_CHASSIS_VX_SPEED);
    VAL_LIMIT(key_board_ctrl.vy, -MAX_CHASSIS_VY_SPEED, MAX_CHASSIS_VY_SPEED);

    key_fsm(&key_board_ctrl.lk_sta, remote_ctrl.mouse.press_l);/*remote_ctrl.key.XX是实际解析出的键鼠信息,通过此函数判断按下状态(按下,未按下,长按,短按)*/
    key_fsm(&key_board_ctrl.rk_sta, remote_ctrl.mouse.press_r);
    key_fsm(&key_board_ctrl.W_sta, remote_ctrl.key.set.W);
    key_fsm(&key_board_ctrl.S_sta, remote_ctrl.key.set.S);
    key_fsm(&key_board_ctrl.A_sta, remote_ctrl.key.set.A);
    key_fsm(&key_board_ctrl.D_sta, remote_ctrl.key.set.D);
    key_fsm(&key_board_ctrl.SHIFT_sta, remote_ctrl.key.set.SHIFT);
    key_fsm(&key_board_ctrl.CTRL_sta, remote_ctrl.key.set.CTRL);
    key_fsm(&key_board_ctrl.Q_sta, remote_ctrl.key.set.Q);
    key_fsm(&key_board_ctrl.E_sta, remote_ctrl.key.set.E);
    key_fsm(&key_board_ctrl.R_sta, remote_ctrl.key.set.R);
    key_fsm(&key_board_ctrl.F_sta, remote_ctrl.key.set.F);
    key_fsm(&key_board_ctrl.G_sta, remote_ctrl.key.set.G);
    key_fsm(&key_board_ctrl.Z_sta, remote_ctrl.key.set.Z);
    key_fsm(&key_board_ctrl.C_sta, remote_ctrl.key.set.C);
    key_fsm(&key_board_ctrl.V_sta, remote_ctrl.key.set.V);
    key_fsm(&key_board_ctrl.B_sta, remote_ctrl.key.set.B);

}