#ifndef _DEV_GIMBALFOLLOW_H_
#define _DEV_GIMBALFOLLOW_H_

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include "cmsis_os.h"
#include "FreeRTOS.h"


#include "alg_pid.h"
#ifndef PI
  #define PI               3.14159265358979f
#endif


#include "alg_pid.h"
/**
 * 
 * 
 * 
 * 
 */
typedef struct 
{
    PidInitConfig_s gimbal_follow_pid_config;

    float up_origin;        // 上云台零点
    float angle_range;      // 角度范围, e.g., 2*PI, 用于过零保护
    float *up_angle_ptr;    // 上云台角度指针 (反馈)
} gimbal_follow_config_s;

/**
 * @brief 云台跟随实例结构体
 */
typedef struct 
{
    float up_origin;        // 上云台零点
    float *up_angle_ptr;    // 上云台角度指针
    float angle_range;      // 角度范围
    
    PidInstance_s *gimbal_follow_pid; // 跟随 PID
    float output;           // 输出值
} gimbal_follow_instance_s;

gimbal_follow_instance_s* GimbalFollow_Register(gimbal_follow_config_s* config);
void GimbalFollow_Unregister(gimbal_follow_instance_s* instance);
bool Follow_Calculate(gimbal_follow_instance_s* instance);









































#endif /*_DEV_GIMBALFOLLOW_H_*/