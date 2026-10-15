#ifndef _RC_DBUS_H
#define _RC_DBUS_H

#include <stdbool.h>
#include "cmsis_os.h"

/* Exported defines -----------------------------------------------------------*/
/**
 * @brief Length of SBUS received data
 */
#define SBUS_RX_BUF_NUM		18u
/**
 * @brief offset of remote control channel data
 */
#define RC_CH_VALUE_OFFSET		1024U

/************************************按键控制相关定义*******************************************/

/* mouse button long press time */
#define LONG_PRESS_TIME  800   //ms

/**
  * @brief     底盘运动速度快慢模式
  */
typedef enum/*默认正常*/
{
    NORMAL_MODE = 0,    //正常模式
    FAST_MODE,          //快速模式
    SLOW_MODE,          //慢速模式
} kb_move_e;

/**
  * @brief     鼠标按键状态类型枚举
  */
typedef enum
{
    KEY_RELEASE = 0,    //没有按键按下
    KEY_WAIT_EFFECTIVE, //等待按键按下有效，防抖
    KEY_PRESS_ONCE,     //按键按下一次的状态
    KEY_PRESS_DOWN,     //按键已经被按下
    KEY_PRESS_LONG,     //按键长按状态
} kb_state_e;

/**
  * @brief     键盘鼠标数据结构体
  */
typedef struct
{
    /* 键盘模式使能标志 */
    uint8_t kb_enable;

    /* 鼠标键盘控制模式下的底盘移动速度目标值 */
    float vx;          //底盘前进后退目标速度
    float vy;          //底盘左右平移目标速度
    float vw;          //底盘旋转速度
    float max_spd;     //运动最大速度

    /* 左右按键状态 */
    kb_state_e lk_sta; //左侧按键状态
    kb_state_e rk_sta; //右侧按键状态
    uint16_t lk_cnt;
    uint16_t rk_cnt;

    /* 键盘按键状态 */
    /*前后左右运动*/
    kb_state_e W_sta;
    kb_state_e S_sta;
    kb_state_e A_sta;
    kb_state_e D_sta;

    kb_state_e SHIFT_sta;/*起跳*/
    kb_state_e CTRL_sta;/*启用超电*/
    kb_state_e Q_sta;/*关小陀螺*/
    kb_state_e E_sta;/*开小陀螺*/
    kb_state_e R_sta;
    kb_state_e F_sta;/*开摩擦轮*/
    kb_state_e G_sta;/*关摩擦轮*/
    kb_state_e Z_sta;/*低腿长*/
    kb_state_e X_sta;/*中间腿长*/
    kb_state_e C_sta;/*高腿长*/
    kb_state_e V_sta;/*切换连发和其他射击模式*/
    kb_state_e B_sta;/*ui显示初始化*/

    /* 运动模式，键盘控制底盘运动快慢 */
    kb_move_e move_mode;

} kb_control_t;

/* Exported types ------------------------------------------------------------*/

/**
 * @brief typedef structure that contains the information for the remote control.
 */
typedef  struct
{
    /**
     * @brief structure that contains the information for the lever/Switch.
     */
    struct
    {
        int16_t ch[5];
        uint8_t s[2];
    } rc;

    /**
     * @brief structure that contains the information for the mouse.
     */
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;


    /**
     * @brief structure that contains the information for the keyboard.
     */
    union/*union使用很巧妙的方法,v和后续结构体中的键位共享内存,v可以反应是否有键鼠的信息*/
    {
        uint16_t v;
        struct
        {
            uint16_t W:1;
            uint16_t S:1;
            uint16_t A:1;
            uint16_t D:1;
            uint16_t SHIFT:1;
            uint16_t CTRL:1;
            uint16_t Q:1;
            uint16_t E:1;
            uint16_t R:1;
            uint16_t F:1;
            uint16_t G:1;
            uint16_t Z:1;
            uint16_t X:1;
            uint16_t C:1;
            uint16_t V:1;
            uint16_t B:1;
        } set;
    } key;

    bool rc_lost;   /*!< lost flag */
    uint8_t online_cnt;   /*!< online count */
} Remote_Info_Typedef;

/* Exported variables ---------------------------------------------------------*/
/**
 * @brief remote control structure variable
 */
extern Remote_Info_Typedef remote_ctrl;
/*
 * @brief remote keyboard control structure variable
 * */
extern kb_control_t key_board_ctrl;
/**
 * @brief remote control usart RxDMA MultiBuffer
 */
extern uint8_t SBUS_MultiRx_Buf[2][SBUS_RX_BUF_NUM];
/* Exported functions prototypes ---------------------------------------------*/
/**
  * @brief  convert the remote control received message
  */
extern void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_Info_Typedef *remote_ctrl);
/**
  * @brief  clear the remote control data while the device offline
  */
extern void Remote_Message_Moniter(Remote_Info_Typedef *remote_ctrl);
/**
  * @brief     PC 处理键盘鼠标数据函数
  */
void PC_Handle_kb(void);

void BSP_USART_Init();



#endif /* _RC_DBUS_H */
