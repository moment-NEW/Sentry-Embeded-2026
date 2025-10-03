/**
*   @file dev_servo.h
*   @brief 
*   @author Wenxin HU
*   @date 25-8-10
*   @version 1.0
*   @note
*/
#ifndef DEV_SERVO_H
#define DEV_SERVO_H

#include "tim.h"
#include "stdint.h"
#include "stdbool.h"

#define PI 3.14159265358979323846

typedef enum {
    SERVO_NORMAL = 0,//普通舵机（脉冲频率50Hz,脉冲宽度0.5ms~2.5ms，转动范围0～pi）
}ServoType_e;

typedef struct {
    ServoType_e type; // 舵机类型
    TIM_HandleTypeDef *tim_handle; // 定时器句柄
    uint32_t channel; // PWM通道

    float min_pulse; // 最小脉冲宽度(ms)
    float max_pulse; // 最大脉冲宽度(ms)
    uint32_t tim_Freq; // 定时器频率(Hz)
    uint32_t tim_PRC; // 定时器预分频值
    uint32_t tim_ARR; // 定时器自动重装载值
    uint32_t pwm_Pulse; // PWM脉冲宽度

    float out_position; // 输出位置(0~pi)
}ServoInstance_s;

typedef struct {
    ServoType_e type;
    TIM_HandleTypeDef *tim_handle;
    uint32_t channel;
}ServoInitConfig_s;

/**
 * @brief 注册舵机实例
 * @param config 舵机初始化配置
 * @return 返回舵机实例指针
 */
ServoInstance_s *Servo_Register(ServoInitConfig_s *config);

/**
 * @brief 舵机控制函数
 * @param servo 舵机实例指针
 * @param target 目标位置(0~pi)
 * @return 返回true表示控制成功，false表示控制失败
 */
bool Servo_Control(ServoInstance_s *servo, float target);

#endif //DEV_SERVO_H
