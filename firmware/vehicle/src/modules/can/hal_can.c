 /*
 * Change Logs:
 * Date            Author          Notes
 * 2023-12-23      ChuShicheng     first version
 */
 /*
* Change Logs:
* Date            Author          Notes
* 2025-11-22      SuperChen     Second version
*/
#include "hal_can.h"
#include "rm_module.h"
#include "DM_motor.h"

extern FDCAN_HandleTypeDef hfdcan1;

extern FDCAN_HandleTypeDef hfdcan2;

extern FDCAN_HandleTypeDef hfdcan3;

static FDCAN_TxHeaderTypeDef  tx_message;


void CAN_send(FDCAN_HandleTypeDef *hfdcan, uint32_t send_id, uint8_t data[])
{

    tx_message.Identifier = send_id;
    tx_message.IdType = FDCAN_STANDARD_ID;
    tx_message.TxFrameType = FDCAN_DATA_FRAME;
    tx_message.DataLength = FDCAN_DLC_BYTES_8;
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE; //CAN发送错误指示
    tx_message.BitRateSwitch = FDCAN_BRS_OFF;//波特率切换关闭
    tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_message.FDFormat = FDCAN_CLASSIC_CAN;//经典can
    tx_message.TxEventFifoControl = FDCAN_NO_TX_EVENTS;//不储存发送事件
    tx_message.MessageMarker = 0;//消息标记0
    while (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0) // 等待邮箱空闲
    {
    }
    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_message, data);
}

/**
 * @brief 初始化并启动 CAN 服务
 *
 * @note 此函数会启动 CAN1 和 CAN2 和 CAN3 ,开启 FIFO0 & FIFO1 接收通知
 *
 */
void CAN_service_init(void)
{
    FDCAN_FilterTypeDef can_filter_st_1;
    can_filter_st_1.IdType = FDCAN_STANDARD_ID;  //过滤标准ID
    can_filter_st_1.FilterIndex = 0;//过滤器编号,用几路can就一次类推0,1,2
    can_filter_st_1.FilterType = FDCAN_FILTER_MASK;  //过滤器Mask模式
    can_filter_st_1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  //选择那个FIFO接收,根据Cubemx配置来
    can_filter_st_1.FilterID1 = 0x00000000;
    can_filter_st_1.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &can_filter_st_1);//将上述配置到CAN1
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);//开启can1的全局过滤器
    HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    HAL_FDCAN_Start(&hfdcan1);//使能CAN1

    FDCAN_FilterTypeDef can_filter_st_2;
    can_filter_st_2.IdType = FDCAN_STANDARD_ID;  //过滤标准ID
    can_filter_st_2.FilterIndex = 1;//过滤器编号,用几路can就一次类推0,1,2
    can_filter_st_2.FilterType = FDCAN_FILTER_MASK;  //过滤器Mask模式
    can_filter_st_2.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;  //选择那个FIFO接收,根据Cubemx配置来
    can_filter_st_2.FilterID1 = 0x00000000;
    can_filter_st_2.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &can_filter_st_2);//将上述配置到CAN2
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);//开启can2的全局过滤器
    HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE,0);
    HAL_FDCAN_Start(&hfdcan2);//使能CAN2

    FDCAN_FilterTypeDef can_filter_st_3;
    can_filter_st_3.IdType = FDCAN_STANDARD_ID;  //过滤标准ID
    can_filter_st_3.FilterIndex = 2;//过滤器编号,用几路can就一次类推0,1,2
    can_filter_st_3.FilterType = FDCAN_FILTER_MASK;  //过滤器Mask模式
    can_filter_st_3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;  //选择那个FIFO接收,根据Cubemx配置来
    can_filter_st_3.FilterID1 = 0x00000000;
    can_filter_st_3.FilterID2 = 0x00000000;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &can_filter_st_3);//将上述配置到CAN3
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3,FDCAN_REJECT,FDCAN_REJECT,FDCAN_FILTER_REMOTE,FDCAN_FILTER_REMOTE);//开启can3的全局过滤器
    HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
    HAL_FDCAN_Start(&hfdcan3);//使能CAN3
}

/*TODO 需要根据电机布局进行回调位置调整*/
 void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0)) // FIFO不为空,有可能在其他中断时有多帧数据进入
    {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data);
        if (hfdcan == &hfdcan1)
        {

#ifdef BSP_USING_LK_MOTOR
            if(lk_motot_rx_callback(rx_header.StdId, rx_data) == 0)
                return;
#endif
#ifdef BSP_USING_HT_MOTOR
            if(ht_motor_rx_callback(rx_header.StdId, rx_data) == 0)
                return;
#endif
#ifdef BSP_USING_DM_MOTOR
            if(dm_motor_rx_callback(rx_header.Identifier, rx_data) == 0)
                return;
#endif
        }
        if (hfdcan == &hfdcan3)
        {
#ifdef BSP_USING_DJI_MOTOR
            if(dji_motot_rx_callback(rx_header.Identifier, rx_data) == 0)
                return;
#endif

        }

    }

}

 void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
 {
     FDCAN_RxHeaderTypeDef rx_header;
     uint8_t rx_data[8];
     while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO1)) // FIFO不为空,有可能在其他中断时有多帧数据进入
     {
         HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &rx_header, rx_data);
         if (hfdcan == &hfdcan2)
         {
#ifdef BSP_USING_DJI_MOTOR
             if(dji_motot_rx_callback(rx_header.Identifier, rx_data) == 0)
                 return;
#endif
#ifdef BSP_USING_LK_MOTOR
             if(lk_motot_rx_callback(rx_header.StdId, rx_data) == 0)
                return;
#endif
#ifdef BSP_USING_HT_MOTOR
             if(ht_motor_rx_callback(rx_header.StdId, rx_data) == 0)
                return;
#endif
#ifdef BSP_USING_DM_IMU
             if(dm_imu_rx_callback(rx_header.Identifier, rx_data) == 0)
                 return;
#endif

#ifdef BSP_USING_DM_MOTOR
             if(dm_motor_rx_callback(rx_header.Identifier, rx_data) == 0)
                 return;
#endif

         }
     }
 }
