#ifndef LAB_STM32_H
#define LAB_STM32_H
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdatomic.h>
typedef struct { int unused; } FDCAN_HandleTypeDef;
typedef struct { int unused; } UART_HandleTypeDef;
typedef struct { int unused; } SPI_HandleTypeDef;
typedef struct { int unused; } TIM_HandleTypeDef;
typedef struct { int unused; } FDCAN_TxHeaderTypeDef;
#define TIM_CHANNEL_4 4
static inline int HAL_TIM_PWM_Start(TIM_HandleTypeDef *t, unsigned c) { (void)t; (void)c; return 0; }
static inline void TIM_Set_PWM(TIM_HandleTypeDef *t, unsigned c, unsigned p) { (void)t; (void)c; (void)p; }
uint32_t HAL_GetTick(void);
/* Events are dispatched serially on the host. Preserve memory barriers. */
static inline uint32_t __get_PRIMASK(void) { return 0; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __set_PRIMASK(uint32_t p) { (void)p; }
static inline void __DMB(void) { atomic_thread_fence(memory_order_seq_cst); }
#endif
