/**
 * @file    vofa_plus.h
 * @author  kaiser
 * @version V1.0.0
 * @date    2026-05-01
 * @brief   VOFA+ JustFloat 协议驱动
 *
 * 功能:
 *   - 基于JustFloat协议通过UART发送多通道浮点数据
 *   - 支持最多32个float通道
 *   - 二进制帧尾 0x00 0x00 0x80 0x7F, VOFA+上位机自动识别
 *   - 支持 DMA 非阻塞发送 (当 UART 配置有 hdmatx 时自动使用)
 */

#ifndef VOFA_PLUS_H
#define VOFA_PLUS_H

#include <stdint.h>
#include "usart.h"

#define VOFA_MAX_CHANNELS 32U

/**
 * @brief 数据包结构体
 * @note  tail[4]预留缓冲, 避免全通道写入帧尾时越界
 */
typedef struct {
    float    channels[VOFA_MAX_CHANNELS]; /**< 通道数据缓冲区 */
    uint8_t  tail[4];                     /**< 帧尾预留缓冲 */
} Vofa_Packet_t;

/**
 * @brief VOFA+实例结构体
 */
typedef struct {
    Vofa_Packet_t packet;                         /**< 数据包 */
    uint8_t       dma_buf[sizeof(Vofa_Packet_t)]; /**< DMA 发送专用平铺缓冲 */
    uint16_t      tx_length;                      /**< 发送长度 = channel_count * 4 + 4 */
    uint8_t       channel_count;                  /**< 实际使用的通道数 */
} Vofa_Instance_t;

/**
 * @brief  初始化VOFA+实例
 * @param  vofa           实例指针
 * @param  channel_count  通道数 (1 ~ VOFA_MAX_CHANNELS)
 */
void Vofa_Init(Vofa_Instance_t *vofa, uint8_t channel_count);

/** Update the actual channel count and frame length for the next frame. */
void Vofa_SetChannelCount(Vofa_Instance_t *vofa, uint8_t channel_count);

/**
 * @brief  设置指定通道的浮点数据
 * @param  vofa           实例指针
 * @param  channel_index  通道索引 (0起始)
 * @param  data           待发送的float数据
 */
void Vofa_SetData(Vofa_Instance_t *vofa, uint8_t channel_index, float data);

/**
 * @brief  发送数据帧到VOFA+上位机 (配置有DMA时优先DMA非阻塞发送)
 * @param  vofa   实例指针
 * @param  huart  UART句柄
 */
HAL_StatusTypeDef Vofa_Transmit(Vofa_Instance_t *vofa,
                                UART_HandleTypeDef *huart);

#endif /* VOFA_PLUS_H */
