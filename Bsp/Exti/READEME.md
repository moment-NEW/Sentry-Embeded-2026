# bsp_exti

## 概述
`bsp_exti` 是一个用于处理外部中断的库，提供了对外部中断控制器（EXTI）的封装和操作接口。该库旨在简化外部中断的配置和使用，使得开发者可以更方便地处理外部事件。

## 环境
- 硬件平台：STM32
- 相关库：CMSIS, HAL, FreeRTOS

## 软件要求
- 作为一个底层驱动，`bsp_exti`不需要额外的库或软件包。
- 使用 `bsp_exti` 需要在 CubeMX 中开启外部中断功能并配置好正确的模式。

## 使用实例
```C
void Exit0_Callback(ExitInstance_s *instance);

ExitInstance_s* exit0 = NULL;
ExitInitConfig_s exit0_config = {
    .pin = Key_Pin,
    .exit_callback = Exit0_Callback,
};

void Exit0_Callback(ExitInstance_s *instance) {
    HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
}

void TestTask(void const * argument){
    exit0 = Exit_Register(&exit0_config);
    while(1){
        osDelay(1);
    }
}
```

## 注意事项
- 确保在使用外部中断之前，已经在 CUBEMX 正确配置了相关的 GPIO 引脚和中断触发方式。
