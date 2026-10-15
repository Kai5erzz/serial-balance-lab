//
// Created by Áõ¼Î¿¡ on 25-2-20.
//

#ifndef CTRBOARD_H7_ALL_CRC8_CRC16_H
#define CTRBOARD_H7_ALL_CRC8_CRC16_H
#include "main.h"

extern uint8_t get_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

extern uint32_t verify_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

extern void append_CRC8_check_sum(unsigned char *pchMessage, unsigned int dwLength);

extern uint16_t get_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);

extern uint32_t verify_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

extern void append_CRC16_check_sum(uint8_t *pchMessage, uint32_t dwLength);

#endif //CTRBOARD_H7_ALL_CRC8_CRC16_H
