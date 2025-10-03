# dev_led

## 概述
`dev_led` 模块用于控制 LED 灯的状态。它提供了简单的接口来打开、关闭和切换 LED 的状态。

## 环境
- 硬件平台：STM32
- 相关库：CMSIS, HAL, FreeRTOS

## 软件要求
- LED 灯的 GPIO 引脚需要在 cubemx 配置为输出模式，并且在使用前需要进行初始化
- 作为用户使用此代码时，仅需考虑.h文件中提供的接口函数即可

## 硬件要求
- 目前仅支持高电平为亮的 LED 灯， 如果是低电平点亮请在 `dev_led.c` 中更换 `GPIO_PIN_SET` 和 `GPIO_PIN_RESET` 的位置
- 如果是外接的 LED ，请注意正负不要接反，正极接到 GPIO 输出引脚，负极接到 GND

## 文件配置
- `MAX_LED_CNT`：最大支持的 LED 数量,C板有红绿蓝三种，可更具实际情况增减
- `LED_FREQ_MAX_THRESHOLD`：LED 闪烁的最大频率阈值，超过此值将不再闪烁
- `LED_PERIOD_TIME`：LED 闪烁的周期时间，单位为毫秒
- `LED_TASK_TIME`：LED 任务的执行间隔时间，单位为毫秒，建议1ms效果最佳

## 使用示例
```C
#include "app_task.h"
#include "dev_led.h"
#include "main.h"

LedInstance_s* led_blue = NULL;
LedInstance_s* led_red = NULL;
LedInstance_s* led_green = NULL;
LedConfig_s led_blue_config = {
    .port = LED_B_GPIO_Port,
    .pin = LED_B_Pin,
    .state = LED_ON,
};
LedConfig_s led_red_config = {
    .port = LED_R_GPIO_Port,
    .pin = LED_R_Pin,
    .state = LED_BLINK,
    .freq = 5,
};
LedConfig_s led_green_config = {
    .port = LED_G_GPIO_Port,
    .pin = LED_G_Pin,
    .state = LED_BREATHING,
};

void TestTask(void const * argument){
    led_blue = Led_Register(&led_blue_config);
    led_green = Led_Register(&led_green_config);
    led_red = Led_Register(&led_red_config);
    while(1){
        Led_Control();
        osDelay(1);
    }
}
```

## 注意事项
1. 在使用 `Led_Register` 函数注册 LED 实例时，请确保传入的配置结构体指针不为 NULL。
2. 在调用 `Led_Set_Mode` 函数时，请确保传入的 LED 实例指针不为 NULL。
3. 在`config`中设置的GPIO端口和引脚必须与实际硬件连接一致，否则 LED 无法正常工作。
4. 如果 LED 的闪烁频率超过 `LED_FREQ_MAX_THRESHOLD`，则会自动将其状态设置为常亮。
5. 注意`LED_TASK_TIME`的值需要更具实际情况更改
6. 目前呼吸灯仅在1ms的任务下有较好效果，随着任务间隔的增加，可能会出现闪烁等情况，那就需要去更改`Led_Control`函数中呼吸灯部分的三个数值了
7. 请注意，这里的`LED_ON`和`LED_OFF`仅代表进行一次模式更换或新注册的初始状态，并不能完全反应当时的亮灭