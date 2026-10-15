#ifndef LAB_DWT_H
#define LAB_DWT_H
#include <stdint.h>
float dwt_get_time_ms(void);
uint32_t dwt_get_time_us(void);
float dwt_get_delta(uint32_t *last);
static inline void dwt_delay_s(float seconds) { (void)seconds; }
#endif
