/**
 * @file chassis_debug.c
 * @brief Chassis debug transport and page scheduling.
 */
#include "chassis_debug.h"

#include "rm_config.h"
#include "rm_module.h"

#if CHASSIS_DEBUG_USE_SYSTEMVIEW
#include "sysview_debug.h"
#include "segger_sysview_uart.h"
#endif

#include <math.h>

#if CHASSIS_DEBUG_USE_VOFA
#include "vofa_plus.h"
#endif
#if CHASSIS_DEBUG_USE_OCTO
#include "octolinker.h"
#endif

#define CHASSIS_DEBUG_MAX_CHANNEL_COUNT 64U
#define CHASSIS_DEBUG_VAR_BASE           0U

#if CHASSIS_DEBUG_USE_VOFA
/* Whole instance in the non-cacheable DMA section: dma_buf is sent with UART
 * DMA while D-Cache is enabled. */
static Vofa_Instance_t chassis_vofa __attribute__((section(".sram_dma")));
#endif
#if CHASSIS_DEBUG_USE_OCTO
static Octolinker_Instance_t chassis_octo;
#endif

static uint8_t chassis_debug_page_value = CHASSIS_DEBUG_PAGE;

static float chassis_debug_data[CHASSIS_DEBUG_MAX_CHANNEL_COUNT];
static uint8_t chassis_debug_count = 0U;
static uint8_t chassis_debug_emit_counter = 0U;

void chassis_debug_module_init(void)
{
#if CHASSIS_DEBUG_USE_VOFA
    Vofa_Init(&chassis_vofa, VOFA_MAX_CHANNELS);
#endif
#if CHASSIS_DEBUG_USE_OCTO
    Octolinker_Init(&chassis_octo, &huart10);
#endif
}

uint8_t chassis_debug_module_page(void)
{
    return chassis_debug_page_value;
}

uint8_t chassis_debug_module_should_emit(void)
{
#if CHASSIS_DEBUG_USE_SYSTEMVIEW
    /* SystemView 模式下以较低频率 (~20Hz) 采样底盘关键变量，保证 230400 串口带宽充裕 */
    if (++chassis_debug_emit_counter >= 50U) {
        chassis_debug_emit_counter = 0U;
        return 1U;
    }
    return 0U;
#endif
    uint8_t page = chassis_debug_module_page();
    /* UART10 runs at 230400 baud.  A 32-channel JustFloat frame occupies the
     * wire for about 5.7 ms and is sent with a blocking HAL call.  Preserve
     * control-task bandwidth by reducing the former 1/2-cycle fast pages to
     * 1/8-cycle; ordinary pages need no more than 1/20-cycle diagnostics. */
    uint8_t period = (page == 0U || page == 5U || page == 8U || page == 10U ||
                       page == 11U || page == 12U || page == 13U || page == 14U ||
                       page == 15U || page == 16U || page == 17U ||
                       page == 18U || page == 23U || page == 45U ||
        page == 46U ||
        page == 55U) ? 8U : 20U;

    /* 32-channel pages (52 jump diagnostics, 56 all-in-one, 57 estimator
     * validation) put 132 bytes =
     * 5.73 ms on the wire at 230400 baud.  Vofa_Transmit() DROPS the entire
     * frame when the previous DMA is still running, so an 8 ms schedule (72%
     * utilisation) loses whole frames on any jitter -- jump22 lost 112 ms of
     * continuous data exactly across takeoff, which is where the flip happened
     * and which is why it could not be diagnosed.  10 ms = 57% utilisation and
     * still gives 100 Hz, enough for the ~10 ms jump dynamics. */
    /* 54 是 BUMP 专用诊断页，同为 32 通道，按同一带宽策略以 100 Hz 发送。
     * 611 把它放在 8 ms 快页列表里，但 611 的页 54 被 runtime 页遮蔽、
     * 从未真正发出；914 没有 runtime 页处理器，这里按 914 自己的策略执行。 */
    if (page == 52U || page == 54U || page == 56U || page == 57U ||
        page == 58U || page == 59U || page == 60U || page == 61U ||
        page == 53U || page == 62U || page == 63U || page == 64U) {
        period = 10U;
    }

    if (++chassis_debug_emit_counter >= period) {
        chassis_debug_emit_counter = 0U;
        return 1U;
    }
    return 0U;
}

void chassis_debug_begin(void)
{
    chassis_debug_count = 0U;
    for (uint8_t i = 0U; i < CHASSIS_DEBUG_MAX_CHANNEL_COUNT; i++) {
        chassis_debug_data[i] = 0.0f;
    }
}

void chassis_debug_set(uint8_t channel, float value)
{
    if (channel < CHASSIS_DEBUG_MAX_CHANNEL_COUNT) {
        /* JustFloat uses the IEEE-754 +Inf byte pattern as its frame tail.
         * Never allow a transient NaN/Inf from a diagnostic-only derivative
         * to appear inside the payload and desynchronize the VOFA parser. */
        if (!isfinite(value)) {
            value = 0.0f;
        }
        chassis_debug_data[channel] = value;
        if ((uint8_t)(channel + 1U) > chassis_debug_count) {
            chassis_debug_count = (uint8_t)(channel + 1U);
        }
    }
}

void chassis_debug_transmit(uint8_t page)
{
    if (page == 58U || page == 59U || page == 60U || page == 61U ||
        page == 53U || page == 62U || page == 63U || page == 64U) {
        chassis_debug_count = 32U;
    }
    if (chassis_debug_count == 0U) {
        return;
    }

#if CHASSIS_DEBUG_USE_VOFA
    uint8_t vofa_count = chassis_debug_count;
    if (vofa_count > VOFA_MAX_CHANNELS) {
        vofa_count = VOFA_MAX_CHANNELS;
    }
    Vofa_SetChannelCount(&chassis_vofa, vofa_count);
    for (uint8_t i = 0U; i < vofa_count; i++) {
        Vofa_SetData(&chassis_vofa, i, chassis_debug_data[i]);
    }
    (void)Vofa_Transmit(&chassis_vofa, &huart10);
#elif CHASSIS_DEBUG_USE_OCTO
    (void)page;
    for (uint8_t i = 0U; i < chassis_debug_count; i++) {
        (void)Octolinker_SendF32(&chassis_octo,
                                 (uint16_t)(CHASSIS_DEBUG_VAR_BASE + i),
                                 chassis_debug_data[i]);
    }
#elif CHASSIS_DEBUG_USE_SYSTEMVIEW
    (void)page;
    uint8_t plot_count = chassis_debug_count;
    if (plot_count > 6U) {
        plot_count = 6U; /* 限制前 6 个关键通道，保证波特率余裕 */
    }
    for (uint8_t i = 0U; i < plot_count; i++) {
        sysview_plot_float(i, "chassis_dbg", chassis_debug_data[i]);
    }
    SEGGER_SYSVIEW_UART_Poll();
#else
    (void)page;
#endif
}
