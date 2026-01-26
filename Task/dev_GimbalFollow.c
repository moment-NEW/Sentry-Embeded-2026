/********
 * Copyright (c) 2026 CGH. All rights reserved.
 * （假装很正式的开头)
 * 
 * @file    dev_GimbalFollow.c
 * @author  CGH
 * @brief   云台跟随模块
 * @details 本模块的意义是使得双yaw云台能够实现下云台跟随上云台旋转，是为了解决上下云台之间没有
 * 滑环导致必须限制下云台旋转角度，使其跟随上云台的问题，而制作的。
 * 具体做法就是，标记一个零点origin，然后读取上云台编码器值，使得二者相减的erro为0，
 * @version V1.0.0
 * @date    2026-01-25
 * 
 * 
 */



#include "dev_GimbalFollow.h"
#include "robot_config.h"


/**
 * @brief 云台跟随注销
 */
void GimbalFollow_Unregister(gimbal_follow_instance_s* instance)
{
    if (instance == NULL) return;
    if (instance->gimbal_follow_pid != NULL)
    {
        user_free(instance->gimbal_follow_pid);
    }
    user_free(instance);
}


/**
 * @brief 云台跟随初始化/注册
 * @note 返回实例指针
 */
gimbal_follow_instance_s* GimbalFollow_Register(gimbal_follow_config_s* config)
{   
    if (config == NULL || config->up_angle_ptr == NULL) return NULL;

    gimbal_follow_instance_s* instance = (gimbal_follow_instance_s*)user_malloc(sizeof(gimbal_follow_instance_s));
    if (instance == NULL) return NULL;
    memset(instance, 0, sizeof(gimbal_follow_instance_s));
    
    instance->up_origin = config->up_origin;
    instance->up_angle_ptr = config->up_angle_ptr;
    instance->angle_range = (config->angle_range != 0.0f) ? config->angle_range : 2.0f * PI; // 默认 2*PI 范围

    // 将角度范围同步到 PID 配置中，以便使用库自带的过零保护
    config->gimbal_follow_pid_config.angle_max = instance->angle_range;
    
    instance->gimbal_follow_pid = Pid_Register(&config->gimbal_follow_pid_config);
    if (instance->gimbal_follow_pid == NULL)
    {
        goto __followfail;
    }
    
    instance->output = 0.0f;
    return instance;

__followfail:
    if (instance) user_free(instance);//算是一个标准化小尝试，实际上有点多余。
    return NULL;
}


/**
 * @brief 云台跟随计算函数
 * @details 修复了 PID 库语义。Target 设为 0 (理想偏移)，Measure 设为当前偏移量。
 * 过零保护由 PID 库内部处理，无需外部 While 循环。
 */
bool Follow_Calculate(gimbal_follow_instance_s* instance)
{
    if (instance == NULL || instance->up_angle_ptr == NULL || instance->gimbal_follow_pid == NULL)
    {
        return false;
    }

    // 计算当前上云台相对于零点的偏移量
    float current_offset = *(instance->up_angle_ptr) - instance->up_origin;
    
    // 理想状态是偏移量为 0，所以 target = 0, measure = current_offset
    // PID 库内部会计算 (target - measure) 即 (0 - current_offset)
    // 注意：如果方向相反，请通过调整 PID 参数中的 Kp 符号来实现，而不是在此处硬编码取反
    Pid_Calculate(instance->gimbal_follow_pid, 0.0f, current_offset);
    
    instance->output = instance->gimbal_follow_pid->output;
    
    return true;
}