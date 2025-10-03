#ifndef __BSP_TIM_H__
#define __BSP_TIM_H__

#include "tim.h"

#define TIM_MAX_INSTANCE 13

typedef struct TimInstance_s TimInstance_s;

struct TimInstance_s {
    TIM_HandleTypeDef *htim;                        // tim句柄
    void (*tim_callback)(TimInstance_s*);                // 定时器回调函数
};

typedef struct{
    TIM_HandleTypeDef *htim;                        // tim句柄
    void (*tim_callback)(TimInstance_s*);           // 定时器回调函数
} TimInitConfig_s;

/**
 * @file bsp_tim.h
 * @brief 注册一个定时器实例，并启动
 * @param config 定时器初始化配置
 * @return Tim实例指针--成功    NULL--失败
 * @note 该函数会检查参数的有效性，并确保不会注册超过最大实例数的定时器。
 *       如果定时器已经注册过，则返回 NULL。
 *       成功注册后，会启动定时器中断。
*/
TimInstance_s* Tim_Register(TimInitConfig_s *config);

#endif
