#include "octolinker.h"
#include <string.h>
#include <stdio.h>
#include "rm_config.h"
#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1) && defined(SYSTEMVIEW_USE_UART) && (SYSTEMVIEW_USE_UART == 1))
#include "segger_sysview_uart.h"
#endif

#define OCTOLINKER_MAX_INSTANCES 4U

static Octolinker_Instance_t *s_instances[OCTOLINKER_MAX_INSTANCES];

static uint8_t Octolinker_CalcCRC8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        crc ^= data[i];
    }
    return crc;
}

static uint32_t Octolinker_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void Octolinker_ExitCritical(uint32_t primask)
{
    if (primask == 0U) {
        __enable_irq();
    }
}

static void Octolinker_RegisterInstance(Octolinker_Instance_t *ol)
{
    if (ol == NULL || ol->huart == NULL) {
        return;
    }

    uint32_t primask = Octolinker_EnterCritical();
    for (uint8_t i = 0U; i < OCTOLINKER_MAX_INSTANCES; i++) {
        if (s_instances[i] == ol) {
            Octolinker_ExitCritical(primask);
            return;
        }
    }
    for (uint8_t i = 0U; i < OCTOLINKER_MAX_INSTANCES; i++) {
        if (s_instances[i] == NULL) {
            s_instances[i] = ol;
            break;
        }
    }
    Octolinker_ExitCritical(primask);
}

static Octolinker_Instance_t *Octolinker_FindByUart(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return NULL;
    }

    for (uint8_t i = 0U; i < OCTOLINKER_MAX_INSTANCES; i++) {
        if (s_instances[i] != NULL && s_instances[i]->huart == huart) {
            return s_instances[i];
        }
    }
    return NULL;
}

static HAL_StatusTypeDef Octolinker_StartCurrent(Octolinker_Instance_t *ol)
{
    HAL_StatusTypeDef status;

    if (ol == NULL || ol->huart == NULL) {
        return HAL_ERROR;
    }

    uint32_t primask = Octolinker_EnterCritical();
    if (ol->tx_active != 0U || ol->tx_count == 0U) {
        Octolinker_ExitCritical(primask);
        return HAL_OK;
    }
    if (ol->huart->gState != HAL_UART_STATE_READY) {
        Octolinker_ExitCritical(primask);
        return HAL_BUSY;
    }

    uint8_t head = ol->tx_head;
    uint8_t *buf = ol->tx_queue[head];
    uint16_t len = ol->tx_queue_len[head];
    ol->tx_active = 1U;
    Octolinker_ExitCritical(primask);

    if (ol->huart->hdmatx != NULL) {
        status = HAL_UART_Transmit_DMA(ol->huart, buf, len);
    } else {
        status = HAL_UART_Transmit_IT(ol->huart, buf, len);
    }

    if (status != HAL_OK) {
        primask = Octolinker_EnterCritical();
        ol->tx_active = 0U;
        Octolinker_ExitCritical(primask);
    }

    return status;
}

static HAL_StatusTypeDef Octolinker_QueueFrame(Octolinker_Instance_t *ol, const uint8_t *frame, uint16_t len)
{
    if (ol == NULL || frame == NULL || len == 0U || len > OCTOLINKER_TX_FRAME_MAX) {
        return HAL_ERROR;
    }

    uint32_t primask = Octolinker_EnterCritical();
    if (ol->tx_count >= OCTOLINKER_TX_QUEUE_DEPTH) {
        Octolinker_ExitCritical(primask);
        return HAL_BUSY;
    }

    uint8_t tail = ol->tx_tail;
    memcpy(ol->tx_queue[tail], frame, len);
    ol->tx_queue_len[tail] = len;
    ol->tx_tail = (uint8_t)((tail + 1U) % OCTOLINKER_TX_QUEUE_DEPTH);
    ol->tx_count++;
    Octolinker_ExitCritical(primask);

    return Octolinker_StartCurrent(ol);
}

static void Octolinker_TxDone(Octolinker_Instance_t *ol)
{
    if (ol == NULL) {
        return;
    }

    uint32_t primask = Octolinker_EnterCritical();
    if (ol->tx_active != 0U && ol->tx_count > 0U) {
        ol->tx_head = (uint8_t)((ol->tx_head + 1U) % OCTOLINKER_TX_QUEUE_DEPTH);
        ol->tx_count--;
    }
    ol->tx_active = 0U;
    Octolinker_ExitCritical(primask);

    (void)Octolinker_StartCurrent(ol);
}

static void Octolinker_TxAbort(Octolinker_Instance_t *ol)
{
    if (ol == NULL) {
        return;
    }

    uint32_t primask = Octolinker_EnterCritical();
    ol->tx_active = 0U;
    if (ol->tx_count > 0U) {
        ol->tx_head = (uint8_t)((ol->tx_head + 1U) % OCTOLINKER_TX_QUEUE_DEPTH);
        ol->tx_count--;
    }
    Octolinker_ExitCritical(primask);

    (void)Octolinker_StartCurrent(ol);
}

void Octolinker_Init(Octolinker_Instance_t *ol, UART_HandleTypeDef *huart)
{
    if (ol == NULL) {
        return;
    }

    memset(ol, 0, sizeof(*ol));
    ol->huart = huart;
    Octolinker_RegisterInstance(ol);
}

HAL_StatusTypeDef Octolinker_SendFrame(
    Octolinker_Instance_t *ol,
    uint16_t var_id,
    uint8_t value_type,
    const uint8_t *shape,
    uint8_t shape_len,
    const uint8_t *payload,
    uint16_t payload_len)
{
    uint8_t *frame;
    uint16_t pos = 0U;

    if (ol == NULL || ol->huart == NULL || payload == NULL) {
        return HAL_ERROR;
    }
    if (shape_len > OCTOLINKER_MAX_SHAPE_DIMS || payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return HAL_ERROR;
    }
    if (shape_len > 0U && shape == NULL) {
        return HAL_ERROR;
    }

    frame = ol->tx_scratch;
    frame[pos++] = OCTOLINKER_SOF0;
    frame[pos++] = OCTOLINKER_SOF1;
    frame[pos++] = OCTOLINKER_VERSION;
    frame[pos++] = (uint8_t)(var_id & 0xFFU);
    frame[pos++] = (uint8_t)((var_id >> 8U) & 0xFFU);
    frame[pos++] = value_type;
    frame[pos++] = shape_len;

    for (uint8_t i = 0U; i < shape_len; i++) {
        frame[pos++] = shape[i];
    }

    frame[pos++] = (uint8_t)(payload_len & 0xFFU);
    frame[pos++] = (uint8_t)((payload_len >> 8U) & 0xFFU);
    memcpy(&frame[pos], payload, payload_len);
    pos = (uint16_t)(pos + payload_len);
    frame[pos++] = Octolinker_CalcCRC8(&frame[2], (uint16_t)(pos - 2U));

    return Octolinker_QueueFrame(ol, frame, pos);
}

HAL_StatusTypeDef Octolinker_SendF32(Octolinker_Instance_t *ol, uint16_t var_id, float value)
{
    uint8_t payload[sizeof(float)];
    memcpy(payload, &value, sizeof(payload));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_F32, NULL, 0U, payload, sizeof(payload));
}

HAL_StatusTypeDef Octolinker_SendI32(Octolinker_Instance_t *ol, uint16_t var_id, int32_t value)
{
    uint8_t payload[sizeof(int32_t)];
    memcpy(payload, &value, sizeof(payload));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_I32, NULL, 0U, payload, sizeof(payload));
}

HAL_StatusTypeDef Octolinker_SendU32(Octolinker_Instance_t *ol, uint16_t var_id, uint32_t value)
{
    uint8_t payload[sizeof(uint32_t)];
    memcpy(payload, &value, sizeof(payload));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U32, NULL, 0U, payload, sizeof(payload));
}

HAL_StatusTypeDef Octolinker_SendI16(Octolinker_Instance_t *ol, uint16_t var_id, int16_t value)
{
    uint8_t payload[sizeof(int16_t)];
    memcpy(payload, &value, sizeof(payload));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_I16, NULL, 0U, payload, sizeof(payload));
}

HAL_StatusTypeDef Octolinker_SendU16(Octolinker_Instance_t *ol, uint16_t var_id, uint16_t value)
{
    uint8_t payload[sizeof(uint16_t)];
    memcpy(payload, &value, sizeof(payload));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U16, NULL, 0U, payload, sizeof(payload));
}

HAL_StatusTypeDef Octolinker_SendI8(Octolinker_Instance_t *ol, uint16_t var_id, int8_t value)
{
    uint8_t payload[sizeof(int8_t)];
    memcpy(payload, &value, sizeof(payload));
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_I8, NULL, 0U, payload, sizeof(payload));
}

HAL_StatusTypeDef Octolinker_SendU8(Octolinker_Instance_t *ol, uint16_t var_id, uint8_t value)
{
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U8, NULL, 0U, &value, sizeof(value));
}

HAL_StatusTypeDef Octolinker_SendF32Array(Octolinker_Instance_t *ol, uint16_t var_id, const float *values, uint8_t count)
{
    if (values == NULL || count == 0U) {
        return HAL_ERROR;
    }

    uint16_t payload_len = (uint16_t)count * (uint16_t)sizeof(float);
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return HAL_ERROR;
    }

    uint8_t shape[1] = {count};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_F32_ARRAY, shape, 1U, (const uint8_t *)values, payload_len);
}

HAL_StatusTypeDef Octolinker_SendU16Array(Octolinker_Instance_t *ol, uint16_t var_id, const uint16_t *values, uint8_t count)
{
    if (values == NULL || count == 0U) {
        return HAL_ERROR;
    }

    uint16_t payload_len = (uint16_t)count * (uint16_t)sizeof(uint16_t);
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return HAL_ERROR;
    }

    uint8_t shape[1] = {count};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U16_ARRAY, shape, 1U, (const uint8_t *)values, payload_len);
}

HAL_StatusTypeDef Octolinker_SendU8Matrix(Octolinker_Instance_t *ol, uint16_t var_id, const uint8_t *values, uint8_t rows, uint8_t cols)
{
    if (values == NULL || rows == 0U || cols == 0U) {
        return HAL_ERROR;
    }

    uint16_t payload_len = (uint16_t)rows * (uint16_t)cols;
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return HAL_ERROR;
    }

    uint8_t shape[2] = {rows, cols};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_U8_MATRIX, shape, 2U, values, payload_len);
}

HAL_StatusTypeDef Octolinker_SendRaw(Octolinker_Instance_t *ol, uint16_t var_id, const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0U || len > OCTOLINKER_MAX_PAYLOAD) {
        return HAL_ERROR;
    }

    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_RAW, NULL, 0U, data, len);
}

HAL_StatusTypeDef Octolinker_SendBitMatrix(Octolinker_Instance_t *ol, uint16_t var_id, const uint8_t *data, uint8_t rows, uint8_t cols)
{
    if (data == NULL || rows == 0U || cols == 0U) {
        return HAL_ERROR;
    }

    uint16_t total_bits = (uint16_t)rows * (uint16_t)cols;
    uint16_t payload_len = (uint16_t)((total_bits + 7U) / 8U);
    if (payload_len > OCTOLINKER_MAX_PAYLOAD) {
        return HAL_ERROR;
    }

    uint8_t shape[2] = {rows, cols};
    return Octolinker_SendFrame(ol, var_id, OCTOLINKER_TYPE_BIT_MATRIX, shape, 2U, data, payload_len);
}

HAL_StatusTypeDef Octolinker_PrintLine(Octolinker_Instance_t *ol, const char *line)
{
    char buf[128];
    size_t len;

    if (line == NULL) {
        return HAL_ERROR;
    }

    len = strlen(line);
    if (len > 0U && (line[len - 1U] == '\n' || line[len - 1U] == '\r')) {
        if (len >= sizeof(buf)) {
            len = sizeof(buf) - 1U;
        }
        memcpy(buf, line, len);
    } else {
        if (len >= sizeof(buf) - 2U) {
            len = sizeof(buf) - 3U;
        }
        memcpy(buf, line, len);
        buf[len++] = '\r';
        buf[len++] = '\n';
    }

    return Octolinker_SendRaw(ol, 0U, (const uint8_t *)buf, (uint16_t)len);
}

HAL_StatusTypeDef Octolinker_SendKV_F32(Octolinker_Instance_t *ol, const char *key, float value)
{
    char buf[64];
    int len;

    if (key == NULL) {
        return HAL_ERROR;
    }

    len = snprintf(buf, sizeof(buf), "%s=%.3f\r\n", key, (double)value);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        return HAL_ERROR;
    }

    return Octolinker_SendRaw(ol, 0U, (const uint8_t *)buf, (uint16_t)len);
}

HAL_StatusTypeDef Octolinker_SendKV_I32(Octolinker_Instance_t *ol, const char *key, int32_t value)
{
    char buf[48];
    int len;

    if (key == NULL) {
        return HAL_ERROR;
    }

    len = snprintf(buf, sizeof(buf), "%s=%ld\r\n", key, (long)value);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        return HAL_ERROR;
    }

    return Octolinker_SendRaw(ol, 0U, (const uint8_t *)buf, (uint16_t)len);
}

HAL_StatusTypeDef Octolinker_SendKV_U32(Octolinker_Instance_t *ol, const char *key, uint32_t value)
{
    char buf[48];
    int len;

    if (key == NULL) {
        return HAL_ERROR;
    }

    len = snprintf(buf, sizeof(buf), "%s=%lu\r\n", key, (unsigned long)value);
    if (len < 0 || (size_t)len >= sizeof(buf)) {
        return HAL_ERROR;
    }

    return Octolinker_SendRaw(ol, 0U, (const uint8_t *)buf, (uint16_t)len);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1) && defined(SYSTEMVIEW_USE_UART) && (SYSTEMVIEW_USE_UART == 1))
    if (huart == &huart10) {
        SEGGER_SYSVIEW_UART_TxCpltCallback();
        return;
    }
#endif
    Octolinker_TxDone(Octolinker_FindByUart(huart));
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1) && defined(SYSTEMVIEW_USE_UART) && (SYSTEMVIEW_USE_UART == 1))
    if (huart == &huart10) {
        SEGGER_SYSVIEW_UART_ErrorCallback();
        return;
    }
#endif
    Octolinker_TxAbort(Octolinker_FindByUart(huart));
}


