#ifndef _RC_SBUS_H
#define _RC_SBUS_H

#include "cmsis_os.h"

#define NOW 0
#define LAST 1

#define SBUS_HEAD 0X0F
#define SBUS_END 0X00
#define SBUS_RX_BUF_SIZE (25+12+4) //41

#define DBUS_RX_BUF_SIZE (25+12+4) //41


/**
  * @brief 遥控器拨杆值
  */
enum {
    RC_UP = 240,
    RC_MI = 0,
    RC_DN = 15,
};

typedef struct __attribute__((__packed__))
{
    uint16_t online;
    /* 摇杆最终值为：-784~783 */
    int16_t ch1;   //右侧左右
    int16_t ch2;   //左侧上下
    int16_t ch3;   //右侧上下
    int16_t ch4;   //左侧左右
    /* FS-i6x旋钮为线性，左右最终值为：-784~783 */
    int16_t ch5;   //左侧线性旋钮
    int16_t ch6;   //右侧线性旋钮
    /* 遥控器的拨杆数据，上(中)下最终值分别为：240、（0）、15 */
    uint8_t sw1;   //SWA，二档
    uint8_t sw2;   //SWB，二档
    uint8_t sw3;   //SWC，三档
    uint8_t sw4;   //SWD，二档
} sbus_data_t;

void sbus_data_init();
void sbus_data_unpack(uint8_t *data, uint16_t len);


#endif /* _RC_SBUS_H */
