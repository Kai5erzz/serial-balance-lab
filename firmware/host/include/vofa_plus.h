#ifndef LAB_VOFA_H
#define LAB_VOFA_H
#include "main.h"
#define VOFA_MAX_CHANNELS 64
typedef struct { float values[64]; unsigned count; } Vofa_Instance_t;
static inline void Vofa_Init(Vofa_Instance_t *v, unsigned n) { v->count = n; }
static inline void Vofa_SetChannelCount(Vofa_Instance_t *v, unsigned n) { v->count = n; }
static inline void Vofa_SetData(Vofa_Instance_t *v, unsigned i, float x) { v->values[i] = x; }
int Vofa_Transmit(Vofa_Instance_t *v, UART_HandleTypeDef *u);
#endif
