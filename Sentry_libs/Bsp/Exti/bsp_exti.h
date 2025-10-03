#ifndef __BSP_EXTI_H__
#define __BSP_EXTI_H__

#include "gpio.h"

#define EXIT_MAX_INSTANCE 16
typedef struct ExitInstance_s ExitInstance_s;

struct ExitInstance_s{
    uint16_t pin;                           // 外部中断引脚
    void (*exit_callback)(ExitInstance_s*);       // 外部中断回调函数
};

typedef struct{
    uint16_t pin;                           // 外部中断引脚
    void (*exit_callback)(ExitInstance_s*); // 外部中断回调函数
} ExitInitConfig_s;

/**
 * @file bsp_exti.h
 * @brief 外部中断注册函数
 * @param config 外部中断配置
 * @return 成功--实例指针  失败--NULL
 */
ExitInstance_s* Exit_Register(ExitInitConfig_s *config);

#endif