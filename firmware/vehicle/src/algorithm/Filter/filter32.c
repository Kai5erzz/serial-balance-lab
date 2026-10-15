/**
  ******************************************************************************
  * @file    filter32.c
  * @author  Wang Hongxi
  * @version V1.0.1
  * @date    2020/7/7
  * @brief
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "filter32.h"

//#if (__CORTEX_M == (7U))
/**
  * @brief          一阶低通滤波初始化
  * @param[in]      一阶低通滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波系数
  * @retval         返回空
  */
void First_Order_Filter_Init(First_Order_Filter_t *first_order_filter, float frame_period, float num)
{
    first_order_filter->Frame_Period = frame_period;
    first_order_filter->RC = num;
    first_order_filter->Input = 0.0f;
    first_order_filter->Output = 0.0f;
}

/**
  * @brief          一阶低通滤波计算
  * @param[in]      一阶低通滤波结构体
  * @param[in]      测量值
  * @retval         返回滤波输出
  */
float First_Order_Filter_Calculate(First_Order_Filter_t *first_order_filter, float input)
{
    first_order_filter->Input = input;

    // x(t) = x(t-1)*dt/(dt + omega) + u*omega/(dt + omega)
    // X(s) = omega/(s + omega)
    first_order_filter->Output =
        first_order_filter->Output * first_order_filter->RC /
            (first_order_filter->RC + first_order_filter->Frame_Period) +
        first_order_filter->Input * first_order_filter->Frame_Period /
            (first_order_filter->RC + first_order_filter->Frame_Period);

    return first_order_filter->Output;
}

/**
  * @brief          窗口滤波初始化
  * @param[in]      窗口滤波结构体
  * @param[in]      窗口大小
  * @retval         返回空
  */
void Window_Filter_Init(Window_Filter_t *window_filter, uint8_t windowSize)
{
    window_filter->WindowNum = 0;
    window_filter->WindowSize = windowSize;
    window_filter->WindowBuffer = (float *)user_malloc(sizeof(float) * windowSize);
    memset(window_filter->WindowBuffer, 0, windowSize);
}

/**
  * @brief          窗口滤波计算
  * @param[in]      窗口滤波结构体
  * @param[in]      测量值
  * @retval         返回滤波输出
  */
float Window_Filter_Calculate(Window_Filter_t *window_filter, float input)
{
    window_filter->Input = input;
    window_filter->Output = 0;

    window_filter->WindowBuffer[window_filter->WindowNum++] = input;
    if (window_filter->WindowNum >= window_filter->WindowSize)
        window_filter->WindowNum = 0;

    for (uint8_t i = 0; i < window_filter->WindowSize; i++)
        window_filter->Output += window_filter->WindowBuffer[i];

    window_filter->Output /= window_filter->WindowSize;

    return window_filter->Output;
}

/**
  * @brief          IIR滤波初始化
  * @param[in]      IIR滤波结构体
  * @param[in]      间隔的时间，单位 s
  * @param[in]      滤波系数
  * @retval         返回空
  */
void IIR_Filter_Init(IIR_Filter_t *iir_filter, float *num, float *den, uint8_t order)
{
    iir_filter->Order = order;
    iir_filter->Num = (float *)user_malloc(sizeof(float) * order);
    iir_filter->Den = (float *)user_malloc(sizeof(float) * order);
    iir_filter->xbuf = (float *)user_malloc(sizeof(float) * order);
    iir_filter->ybuf = (float *)user_malloc(sizeof(float) * order);
    memcpy(iir_filter->Num, num, sizeof(float) * order);
    memcpy(iir_filter->Den, den, sizeof(float) * order);
}

/**
  * @brief          IIR滤波计算
  * @param[in]      IIR滤波结构体
  * @param[in]      测量值
  * @retval         返回滤波输出
  */
float IIR_Filter_Calculate(IIR_Filter_t *iir_filter, float input)
{
    iir_filter->Input = input;
    for (uint8_t i = iir_filter->Order - 1; i > 0; i--)
    {
        iir_filter->xbuf[i] = iir_filter->xbuf[i - 1];
        iir_filter->ybuf[i] = iir_filter->ybuf[i - 1];
    }
    iir_filter->xbuf[0] = input;
    iir_filter->ybuf[0] = iir_filter->Num[0] * iir_filter->xbuf[0];
    for (uint8_t i = 1; i < iir_filter->Order; i++)
    {
        iir_filter->ybuf[0] += iir_filter->Num[i] * iir_filter->xbuf[i] - iir_filter->Den[i] * iir_filter->ybuf[i];
    }
    iir_filter->Output = iir_filter->ybuf[0];
    return iir_filter->Output;
}

/** 滑动平均滤波器 */

// 初始化平均滤波器结构体
void ave_fil_init(ave_filter_t *ave_fil)
{
    // 将value数组的所有元素清零
    memset(ave_fil->value, 0, ave_filter_times_max * sizeof(float));
    // 将平均值初始化为0
    ave_fil->value_ave = 0;
    // 将当前索引位置初始化为0
    ave_fil->index = 0;
    // 将滤波次数初始化为0
    ave_fil->filter_times = 0;
}

// 更新平均滤波器的值并计算新的平均值
float ave_fil_update(ave_filter_t *ave_fil, float value, uint16_t max)
{
    // 如果传入的滤波次数大于最大允许的滤波次数，则将其设为最大允许值
    if(max > ave_filter_times_max)
    {
        max = ave_filter_times_max;
    }
    // 如果传入的滤波次数与当前的滤波次数不同，则重新初始化滤波器
    if(max != ave_fil->filter_times)
    {
        ave_fil_init(ave_fil);
        // 更新滤波器的滤波次数
        ave_fil->filter_times = max;
    }
    // 从当前的平均值中减去即将被替换的值的贡献
    ave_fil->value_ave -= ave_fil->value[ave_fil->index] / (float)max;
    // 更新value数组中当前索引位置的值
    ave_fil->value[ave_fil->index] = value;
    // 将新值的贡献加到平均值中
    ave_fil->value_ave += ave_fil->value[ave_fil->index] / (float)max;
    // 增加索引位置，如果达到最大滤波次数，则回绕到0
    ave_fil->index++;
    ave_fil->index %= max;

    // 返回更新后的平均值
    return ave_fil->value_ave;
}

//#endif
