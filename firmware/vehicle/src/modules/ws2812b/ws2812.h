//
// Created by SuperChen on 2025/11/28.
//

#ifndef CTRBOARD_H7_ALL_WS2812_H
#define CTRBOARD_H7_ALL_WS2812_H

#include "main.h"


#define WS2812_SPI_UNIT     hspi6
extern SPI_HandleTypeDef WS2812_SPI_UNIT;

void WS2812_Ctrl(uint8_t r, uint8_t g, uint8_t b);

#endif //CTRBOARD_H7_ALL_WS2812_H
