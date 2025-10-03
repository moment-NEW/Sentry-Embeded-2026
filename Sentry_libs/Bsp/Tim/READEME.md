# bsp_tim

## 概述
`bsp_tim` 是一个用于处理定时器的库，提供了对定时器硬件的封装和操作接口。该库旨在简化定时器的配置和使用，使得开发者可以更方便地实现定时任务和事件。

## 环境
- 硬件平台：STM32
- 相关库：CMSIS, HAL, FreeRTOS

## 软件要求
- 作为一个底层驱动，`bsp_tim`不需要额外的库或软件包。
- 使用 `bsp_tim` 需要在 CubeMX 中开启中断并配置好正确的时间。

## 实例定义
```C
typedef struct TimInstance_s TimInstance_s;

struct TimInstance_s {
    TIM_HandleTypeDef *htim;                        // tim句柄
    void (*tim_callback)(TimInstance_s*);                // 定时器回调函数
};

typedef struct{
    TIM_HandleTypeDef *htim;                        // tim句柄
    void (*tim_callback)(TimInstance_s*);           // 定时器回调函数
} TimInitConfig_s;
```

## 外部接口
```C
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
```

## 使用实例
```C
#include "bsp_tim.h"

void Tim2_Callback(TimInstance_s *instance);

TimInstance_s* tim2 = NULL;
TimInitConfig_s tim2_config = {
    .htim = &htim2,
    .tim_callback = Tim2_Callback,
};

void TestTask(void const * argument){
    tim2 = Tim_Register(&tim2_config);
    while(1){
        osDelay(1);
    }
}

void Tim2_Callback(TimInstance_s *instance) {
    HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);
}
```

## 注意事项
- **如果使用Free RTOS，需要把主函数的中断回调注释，并复制到`bsp_tim.c`中的回调函数中**
- 确保在 CubeMX 中正确配置了定时器和中断。
- 定时器回调函数应尽量简短，以避免阻塞其他任务。
- 在使用定时器时，注意避免资源冲突和中断嵌套问题。