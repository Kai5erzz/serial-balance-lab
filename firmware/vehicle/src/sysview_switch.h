/**
 * @file    sysview_switch.h
 * @brief   SystemView 与调试输出模式宏定义开关
 * @note    此文件专供 rm_config.h 与 FreeRTOSConfig.h 共享，不引入任何底层外设或动力学宏定义
 */
#ifndef SYSVIEW_SWITCH_H
#define SYSVIEW_SWITCH_H

/* --------------------------- 调试输出模式选择 (三选一) --------------------------- */
#define CHASSIS_DEBUG_USE_VOFA       1
#define CHASSIS_DEBUG_USE_OCTO       0
#define CHASSIS_DEBUG_USE_SYSTEMVIEW 0

/* ----------------------- MPU / D-Cache 生效开关 (诊断用) ----------------------- */
/* 1 = MPU(non-cacheable DMA段) + D-Cache 全开 (正式配置)
 * 0 = 仅 I-Cache (180us 基线行为, 用于二分定位 UART/DMA 故障) */
#define CACHE_MPU_ENABLE 1

/* ----------------------------- SEGGER SystemView 配置 ----------------------------- */
#if CHASSIS_DEBUG_USE_SYSTEMVIEW
#define SYSTEMVIEW_ENABLE            1
#define SYSTEMVIEW_USE_UART          1      /* 1=使用UART10(复用VOFA串口), 0=使用J-Link RTT */
#define SYSTEMVIEW_UART_BAUDRATE     230400 /* 串口波特率: 230400 (VOFA默认) / 500000 / 921600 */
#else
#define SYSTEMVIEW_ENABLE            0
#define SYSTEMVIEW_USE_UART          0
#endif

#endif /* SYSVIEW_SWITCH_H */
