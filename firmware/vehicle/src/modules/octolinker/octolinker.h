#ifndef OCTOLINKER_H
#define OCTOLINKER_H

#include <stdint.h>
#include <stddef.h>
#include "usart.h"

#define OCTOLINKER_VERSION          1U
#define OCTOLINKER_SOF0             0xA5U
#define OCTOLINKER_SOF1             0x5AU
#define OCTOLINKER_MAX_PAYLOAD      256U
#define OCTOLINKER_MAX_SHAPE_DIMS   4U
#define OCTOLINKER_TX_TIMEOUT_MS    2U

#define OCTOLINKER_TYPE_I32         1U
#define OCTOLINKER_TYPE_U32         2U
#define OCTOLINKER_TYPE_F32         3U
#define OCTOLINKER_TYPE_I16         4U
#define OCTOLINKER_TYPE_U16         5U
#define OCTOLINKER_TYPE_I8          6U
#define OCTOLINKER_TYPE_U8          7U
#define OCTOLINKER_TYPE_RAW         8U
#define OCTOLINKER_TYPE_F32_ARRAY   9U
#define OCTOLINKER_TYPE_U16_ARRAY   10U
#define OCTOLINKER_TYPE_U8_MATRIX   11U
#define OCTOLINKER_TYPE_BIT_MATRIX  12U

#define OCTOLINKER_TX_FRAME_MAX     (2U + 1U + 2U + 1U + 1U + OCTOLINKER_MAX_SHAPE_DIMS + 2U + OCTOLINKER_MAX_PAYLOAD + 1U)
#define OCTOLINKER_TX_QUEUE_DEPTH   96U

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t tx_queue[OCTOLINKER_TX_QUEUE_DEPTH][OCTOLINKER_TX_FRAME_MAX];
    uint16_t tx_queue_len[OCTOLINKER_TX_QUEUE_DEPTH];
    volatile uint8_t tx_head;
    volatile uint8_t tx_tail;
    volatile uint8_t tx_count;
    volatile uint8_t tx_active;
    uint8_t tx_scratch[OCTOLINKER_TX_FRAME_MAX];
} Octolinker_Instance_t;

void Octolinker_Init(Octolinker_Instance_t *ol, UART_HandleTypeDef *huart);

HAL_StatusTypeDef Octolinker_SendFrame(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    uint8_t value_type,
    const uint8_t *shape,
    uint8_t shape_len,
    const uint8_t *payload,
    uint16_t payload_len
);

HAL_StatusTypeDef Octolinker_SendF32(Octolinker_Instance_t *ol, uint16_t var_id, float value);
HAL_StatusTypeDef Octolinker_SendI32(Octolinker_Instance_t *ol, uint16_t var_id, int32_t value);
HAL_StatusTypeDef Octolinker_SendU32(Octolinker_Instance_t *ol, uint16_t var_id, uint32_t value);
HAL_StatusTypeDef Octolinker_SendI16(Octolinker_Instance_t *ol, uint16_t var_id, int16_t value);
HAL_StatusTypeDef Octolinker_SendU16(Octolinker_Instance_t *ol, uint16_t var_id, uint16_t value);
HAL_StatusTypeDef Octolinker_SendI8(Octolinker_Instance_t *ol, uint16_t var_id, int8_t value);
HAL_StatusTypeDef Octolinker_SendU8(Octolinker_Instance_t *ol, uint16_t var_id, uint8_t value);

HAL_StatusTypeDef Octolinker_SendF32Array(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    const float *values,
    uint8_t count
);

HAL_StatusTypeDef Octolinker_SendU16Array(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    const uint16_t *values,
    uint8_t count
);

HAL_StatusTypeDef Octolinker_SendU8Matrix(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    const uint8_t *values,
    uint8_t rows,
    uint8_t cols
);

HAL_StatusTypeDef Octolinker_SendRaw(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    const uint8_t *data,
    uint16_t len
);

HAL_StatusTypeDef Octolinker_SendBitMatrix(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    const uint8_t *data,
    uint8_t rows,
    uint8_t cols
);

HAL_StatusTypeDef Octolinker_PrintLine(Octolinker_Instance_t *ol, const char *line);
HAL_StatusTypeDef Octolinker_SendKV_F32(Octolinker_Instance_t *ol, const char *key, float value);
HAL_StatusTypeDef Octolinker_SendKV_I32(Octolinker_Instance_t *ol, const char *key, int32_t value);
HAL_StatusTypeDef Octolinker_SendKV_U32(Octolinker_Instance_t *ol, const char *key, uint32_t value);

#endif




