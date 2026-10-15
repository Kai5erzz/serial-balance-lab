/**
 * @file chassis_debug.h
 * @brief Chassis debug transport and page scheduling interface.
 */
#ifndef CHASSIS_DEBUG_H
#define CHASSIS_DEBUG_H

#include <stdint.h>

void chassis_debug_module_init(void);
uint8_t chassis_debug_module_page(void);
uint8_t chassis_debug_module_should_emit(void);

void chassis_debug_begin(void);
void chassis_debug_set(uint8_t channel, float value);
void chassis_debug_transmit(uint8_t page);

#endif /* CHASSIS_DEBUG_H */
