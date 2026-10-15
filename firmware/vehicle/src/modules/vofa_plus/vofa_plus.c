/**
 * @file    vofa_plus.c
 * @author  kaiser
 * @version V1.1.0
 * @date    2026-09-07
 * @brief   VOFA+ JustFloat 协议驱动实现 (支持 DMA 非阻塞零拷贝)
 *
 * 协议格式:
 *   [ch0 float][ch1 float]...[chN float][0x00 0x00 0x80 0x7F]
 *   帧尾为IEEE 754 NaN (+Inf), VOFA+据此识别帧边界
 */

#include "vofa_plus.h"
#include <string.h>

/** JustFloat协议固定帧尾 */
static const uint8_t VOFA_TAIL[4] = {0x00, 0x00, 0x80, 0x7F};

/**
 * @brief  初始化VOFA+实例
 * @param  vofa           实例指针
 * @param  channel_count  通道数 (上限VOFA_MAX_CHANNELS)
 * @note   清零数据缓冲并计算发送长度
 */
void Vofa_Init(Vofa_Instance_t *vofa, uint8_t channel_count)
{
    if (vofa == NULL) return;
    memset(vofa, 0, sizeof(Vofa_Instance_t));
    Vofa_SetChannelCount(vofa, channel_count);
}

void Vofa_SetChannelCount(Vofa_Instance_t *vofa, uint8_t channel_count)
{
    if (vofa == NULL) return;
    vofa->channel_count = (channel_count > VOFA_MAX_CHANNELS) ?
                          VOFA_MAX_CHANNELS : channel_count;
    vofa->tx_length = (uint16_t)(vofa->channel_count * sizeof(float) + sizeof(VOFA_TAIL));
}

/**
 * @brief  设置指定通道的浮点数据
 * @param  vofa           实例指针
 * @param  channel_index  通道索引 (0起始, 需 < channel_count)
 * @param  data           待发送的float值
 */
void Vofa_SetData(Vofa_Instance_t *vofa, uint8_t channel_index, float data)
{
    if (vofa == NULL || channel_index >= vofa->channel_count) return;
    vofa->packet.channels[channel_index] = data;
}

/**
 * @brief  发送JustFloat数据帧到VOFA+上位机
 * @param  vofa   实例指针
 * @param  huart  UART句柄 (由调用者传入)
 * @note   优先使用DMA非阻塞发送(若huart->hdmatx存在), 彻底避免阻塞1kHz控制任务
 */
HAL_StatusTypeDef Vofa_Transmit(Vofa_Instance_t *vofa,
                                UART_HandleTypeDef *huart)
{
    if (vofa == NULL || huart == NULL || huart->Init.BaudRate == 0U) {
        return HAL_ERROR;
    }

    /* 写入帧尾 */
    uint8_t *tail_ptr = (uint8_t *)(&vofa->packet.channels[vofa->channel_count]);
    tail_ptr[0] = VOFA_TAIL[0];
    tail_ptr[1] = VOFA_TAIL[1];
    tail_ptr[2] = VOFA_TAIL[2];
    tail_ptr[3] = VOFA_TAIL[3];

    /* 1. 若配置了DMA TX, 采用非阻塞DMA发送 */
    if (huart->hdmatx != NULL) {
        if (huart->gState != HAL_UART_STATE_READY) {
            /* 上一帧DMA传输仍在进行, 跳过此帧以防总线冲突, 零延迟返回 */
            return HAL_BUSY;
        }
        /* 复制到独立的DMA平铺缓冲区, 避免后续SetData操作冲刷正在发送的数据 */
        memcpy(vofa->dma_buf, &vofa->packet, vofa->tx_length);
        return HAL_UART_Transmit_DMA(huart, vofa->dma_buf, vofa->tx_length);
    }

    /* 2. 未配置DMA时的回退阻塞发送 */
    const uint32_t tx_bits = (uint32_t)vofa->tx_length * 10U;
    const uint32_t wire_time_ms =
        (tx_bits * 1000U + huart->Init.BaudRate - 1U) /
        huart->Init.BaudRate;
    const uint32_t timeout_ms = wire_time_ms + 8U;
    HAL_StatusTypeDef status = HAL_UART_Transmit(
        huart, (uint8_t *)&vofa->packet, vofa->tx_length, timeout_ms);

    if (status != HAL_OK && huart->gState != HAL_UART_STATE_READY) {
        (void)HAL_UART_AbortTransmit(huart);
    }
    return status;
}
