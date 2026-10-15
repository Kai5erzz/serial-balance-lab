/**
 * @file    sysview_debug.h
 * @brief   SystemView 用户级调试与变量观测助手宏
 * @author  Kai5erzz / Antigravity
 * @date    2026-09-09
 */
#ifndef SYSVIEW_DEBUG_H
#define SYSVIEW_DEBUG_H

#include "rm_config.h"

#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1))

#include "SEGGER_SYSVIEW.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief  注册并记录一个 float 类型的观察变量 (Data Plot)
 * @param  id    变量 ID (0~31)
 * @param  name  变量名称 (在 SystemView 中显示)
 * @param  val   float 数值
 */
void sysview_plot_float(unsigned int id, const char *name, float val);

/**
 * @brief  注册并记录一个 uint32 类型的观察变量 (Data Plot)
 * @param  id    变量 ID (0~31)
 * @param  name  变量名称
 * @param  val   uint32 数值
 */
void sysview_plot_u32(unsigned int id, const char *name, uint32_t val);

#ifdef __cplusplus
}
#endif

/* ------------------------------ 用户便捷宏定义 ------------------------------ */

/* 代码块运行耗时标记（可在甘特图与 Markers 窗口直观显示函数耗时） */
#define SYSVIEW_MARK_START(id)                  SEGGER_SYSVIEW_MarkStart(id)
#define SYSVIEW_MARK_STOP(id)                   SEGGER_SYSVIEW_MarkStop(id)
#define SYSVIEW_MARK(id)                        SEGGER_SYSVIEW_Mark(id)
#define SYSVIEW_NAME_MARKER(id, name)           SEGGER_SYSVIEW_NameMarker(id, name)

/* 终端诊断信息打印 */
#define SYSVIEW_PRINT(msg)                      SEGGER_SYSVIEW_Print(msg)
#define SYSVIEW_PRINTF(fmt, ...)                SEGGER_SYSVIEW_PrintfHost(fmt, ##__VA_ARGS__)
#define SYSVIEW_WARN(msg)                       SEGGER_SYSVIEW_Warn(msg)
#define SYSVIEW_ERROR(msg)                      SEGGER_SYSVIEW_Error(msg)

/* 变量曲线采样绘制 (Data Plot) */
#define SYSVIEW_PLOT_FLOAT(id, name, val)       sysview_plot_float(id, name, val)
#define SYSVIEW_PLOT_U32(id, name, val)         sysview_plot_u32(id, name, val)

#else /* SYSTEMVIEW_ENABLE == 0 */

/* 未开启 SystemView 时，所有宏展开为空操作，零额外 CPU/RAM 开销 */
#define SYSVIEW_MARK_START(id)                  ((void)0)
#define SYSVIEW_MARK_STOP(id)                   ((void)0)
#define SYSVIEW_MARK(id)                        ((void)0)
#define SYSVIEW_NAME_MARKER(id, name)           ((void)0)

#define SYSVIEW_PRINT(msg)                      ((void)0)
#define SYSVIEW_PRINTF(fmt, ...)                ((void)0)
#define SYSVIEW_WARN(msg)                       ((void)0)
#define SYSVIEW_ERROR(msg)                      ((void)0)

#define SYSVIEW_PLOT_FLOAT(id, name, val)       ((void)0)
#define SYSVIEW_PLOT_U32(id, name, val)         ((void)0)

#endif /* SYSTEMVIEW_ENABLE */

#endif /* SYSVIEW_DEBUG_H */
