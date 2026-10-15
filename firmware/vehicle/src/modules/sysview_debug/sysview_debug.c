/**
 * @file    sysview_debug.c
 * @brief   SystemView 用户级调试与变量观测助手实现
 * @author  Kai5erzz / Antigravity
 * @date    2026-09-09
 */
#include "sysview_debug.h"

#if (defined(SYSTEMVIEW_ENABLE) && (SYSTEMVIEW_ENABLE == 1))

#define SYSVIEW_MAX_REGISTERED_VARS 32U
static uint32_t s_registered_mask = 0U;

void sysview_plot_float(unsigned int id, const char *name, float val)
{
    if (id >= SYSVIEW_MAX_REGISTERED_VARS) {
        return;
    }

    if (!SEGGER_SYSVIEW_IsStarted()) {
        return;
    }

    /* 首次调用时自动注册变量元数据 */
    if ((s_registered_mask & (1U << id)) == 0U) {
        SEGGER_SYSVIEW_DATA_REGISTER reg;
        reg.ID = (U32)id;
        reg.DataType = SEGGER_SYSVIEW_TYPE_FLOAT;
        reg.Offset = 0;
        reg.RangeMin = -1000;
        reg.RangeMax = 1000;
        reg.ScalingFactor = 1.0f;
        reg.sName = (name != NULL) ? name : "var";
        reg.sUnit = "";
        SEGGER_SYSVIEW_RegisterData(&reg);
        s_registered_mask |= (1U << id);
    }

    SEGGER_SYSVIEW_DATA_SAMPLE sample;
    sample.ID = (U32)id;
    sample.pValue.pFloat = &val;
    SEGGER_SYSVIEW_SampleData(&sample);
}

void sysview_plot_u32(unsigned int id, const char *name, uint32_t val)
{
    if (id >= SYSVIEW_MAX_REGISTERED_VARS) {
        return;
    }

    if (!SEGGER_SYSVIEW_IsStarted()) {
        return;
    }

    if ((s_registered_mask & (1U << id)) == 0U) {
        SEGGER_SYSVIEW_DATA_REGISTER reg;
        reg.ID = (U32)id;
        reg.DataType = SEGGER_SYSVIEW_TYPE_U32;
        reg.Offset = 0;
        reg.RangeMin = 0;
        reg.RangeMax = 0x7FFFFFFF;
        reg.ScalingFactor = 1.0f;
        reg.sName = (name != NULL) ? name : "var";
        reg.sUnit = "";
        SEGGER_SYSVIEW_RegisterData(&reg);
        s_registered_mask |= (1U << id);
    }

    float fval = (float)val;
    SEGGER_SYSVIEW_DATA_SAMPLE sample;
    sample.ID = (U32)id;
    sample.pValue.pFloat = &fval;
    SEGGER_SYSVIEW_SampleData(&sample);
}

#endif /* SYSTEMVIEW_ENABLE */
