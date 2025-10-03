#ifndef __DEV_LED_H__
#define __DEV_LED_H__

#include <stdint.h>
#include "gpio.h"
#include <stdbool.h>

#define MAX_LED_CNT 10
#define LED_FREQ_MAX_THRESHOLD 8
#define LED_PERIOD_TIME 2000    //循环时间(ms)
#define LED_TASK_TIME 1         //LED任务周期间隔(ms)

typedef enum {
    LED_OFF = 0,        // LED关闭
    LED_ON,             // LED常亮
    LED_BLINK,          // LED闪烁
    LED_BREATHING,      // LED呼吸灯效果
}LedState_e;

typedef struct {
    GPIO_TypeDef* port;             // LED所在的GPIO端口
    uint16_t pin;                   // LED引脚
    uint8_t freq;                  // LED闪烁频率
    LedState_e state;               // LED初始状态（关闭、常亮、闪烁）
}LedConfig_s;

typedef struct {
    GPIO_TypeDef* port;             // LED所在的GPIO端口
    uint16_t pin;                   // LED引脚
    LedState_e state;               // LED当前状态
    uint8_t freq;                  // LED闪烁频率(ms)
}LedInstance_s;

/**
 * @file dev_led.h
 * @brief 注册LED实例
 * @param led_config led配置结构体指针
 * @return 实例指针--注册成功  NULL--注册失败
 * @note 注册LED实例时会分配内存空间
 *       如果LED频率大于LED_FREQ_MAX_THRESHOLD，则将LED状态设置为常亮
 * @date 2025-8-4
 */
LedInstance_s* Led_Register(LedConfig_s *led_config);

/**
 * @file dev_led.h
 * @brief 控制LED状态
 * @note 把该函数写在某一个任务的循环里即可
 * @date 2025-8-4
 */
void Led_Control(void);

/**
 * @file dev_led.h
 * @brief 设置LED模式
 * @param led LED实例指针
 * @param mode 目标LED模式
 * @param freq 闪烁频率，仅在mode为闪烁时有效
 * @return true--设置成功 false--设置失败
 * @note 该函数可以设置LED的状态和频率
 *       如果LED状态为闪烁模式，则需要设置频率
 *       如果LED状态为常亮模式，则不需要设置频率
 *       如果LED状态为关闭模式，则不需要设置频率
 * @date 2025-8-4
 */
bool Led_Set_Mode(LedInstance_s* led, LedState_e mode, uint8_t freq);
#endif
