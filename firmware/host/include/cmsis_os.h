#ifndef _CMSIS_OS_H
#define _CMSIS_OS_H
#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
typedef void *osSemaphoreId;
typedef void *osThreadId;
typedef uint32_t TickType_t;
#define pvPortMalloc malloc
#define vPortFree free
#define configASSERT assert
static inline void vTaskDelay(uint32_t ticks) { (void)ticks; }
static inline void vTaskDelayUntil(uint32_t *last, uint32_t ticks) { *last += ticks; }
static inline void osDelay(uint32_t ticks) { (void)ticks; }
static inline void osThreadSuspendAll(void) {}
static inline void osThreadResumeAll(void) {}
static inline int osSemaphoreRelease(osSemaphoreId s) { (void)s; return 0; }
static inline int osSemaphoreWait(osSemaphoreId s, uint32_t t) { (void)s; (void)t; return 0; }
uint32_t osKernelSysTick(void);
#define osSemaphoreDef(name) int name
#define osSemaphore(name) (&name)
static inline osSemaphoreId osSemaphoreCreate(void *p, int n) { (void)n; return p; }
#endif
