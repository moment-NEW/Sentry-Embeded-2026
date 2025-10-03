/**
*   @file dev_servo.c
*   @brief 
*   @author Wenxin HU
*   @date 25-8-10
*   @version 1.0
*   @note
*/
#include "dev_servo.h"
#include "cmsis_os.h"

ServoInstance_s *Servo_Register(ServoInitConfig_s *config)
{
    ServoInstance_s *servo = (ServoInstance_s *)pvPortMalloc(sizeof(ServoInstance_s));
    if (servo == NULL) {
        return NULL; // 内存分配失败
    }

    servo->type = config->type;
    servo->tim_handle = config->tim_handle;
    servo->channel = config->channel;

    //修改定时器配置防止配置错误
    RCC_ClkInitTypeDef clkCfg;
    uint32_t flashLat;
    HAL_RCC_GetClockConfig(&clkCfg, &flashLat);//获取当前时钟配置
    //C板的PWM只提供TIM1和TIM8两个定时器，均挂在在APB2总线上
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    if (clkCfg.APB2CLKDivider == RCC_HCLK_DIV1) servo->tim_Freq = pclk2;
    else servo->tim_Freq = pclk2 * 2U;

    __HAL_TIM_SET_PRESCALER(servo->tim_handle, (servo->tim_Freq / 100000U) - 1);
    servo->tim_PRC = (servo->tim_Freq / 100000U) - 1;
    __HAL_TIM_SET_AUTORELOAD(servo->tim_handle, 2000-1);
    servo->tim_ARR = 2000 - 1; // 设置自动重装载值为20ms，即50Hz

    // 设置PWM脉冲宽度
    if (servo->type == SERVO_NORMAL) {
        // 普通舵机，脉冲宽度范围0.5ms~2.5ms，对应的脉冲宽度为0.01ms~0.05ms
        servo->max_pulse = 2.5f;
        servo->min_pulse = 0.5f;
    } else {
        // 其他类型的舵机可以在这里添加
        return NULL; // 目前不支持的舵机类型
    }

    // 初始化输出位置
    servo->out_position = 0.0f;

    __HAL_TIM_SET_COMPARE(servo->tim_handle, servo->channel, 0);
    HAL_TIM_PWM_Start(servo->tim_handle, servo->channel);

    return servo;
}

bool Servo_Control(ServoInstance_s *servo, float target)
{
    if (servo == NULL || servo->tim_handle == NULL) {
        return false; // 无效的舵机实例
    }

    // 限制目标位置在0到PI之间
    if (target < 0.0f) {
        target = 0.0f;
    } else if (target > PI) {
        target = PI;
    }

    // 计算PWM脉冲宽度
    float pwm_duty = 0.1f*(target/PI) + 0.025f; // 计算占空比
    servo->pwm_Pulse = (uint32_t)(pwm_duty * (servo->tim_ARR + 1)); //

    // 设置PWM脉冲宽度
    __HAL_TIM_SET_COMPARE(servo->tim_handle, servo->channel, servo->pwm_Pulse);

    // 更新输出位置
    servo->out_position = target;

    return true; // 控制成功
}
