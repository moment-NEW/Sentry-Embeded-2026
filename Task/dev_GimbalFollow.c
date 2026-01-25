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


/**
 * @brief 云台跟随初始化/注册
 * @note 返回实例指针
 */
gimbal_follow_instance_s* GimbalFollow_Register(gimbal_follow_config_s* config)
{   
    if (config == NULL) return NULL;

    gimbal_follow_instance_s* instance = (gimbal_follow_instance_s*)pvPortMalloc(sizeof(gimbal_follow_instance_s));
    if (instance == NULL) return NULL;
    memset(instance, 0, sizeof(gimbal_follow_instance_s));
    
    instance->up_origin = config->up_origin;
    instance->up_angle_ptr = config->up_angle_ptr;
    instance->down_angle_ptr = config->down_angle_ptr;
    
   
    instance->gimbal_follow_pid = Pid_Register(&config->gimbal_follow_pid_config);
    if (instance->gimbal_follow_pid == NULL)
    {
        goto __followfail;
    }
    
    // 角度范围设置
    instance->angle_range = (config->angle_range != 0.0f) ? config->angle_range : 2.0f * PI;
    // 死区设置
    instance->dead_zone = config->dead_zone; 

    instance->output = 0.0f;
    return instance;

__followfail:
    if (instance) vPortFree(instance);
    return NULL;
}


/**
 * @brief 云台跟随计算函数
 * @details 增加了回绕处理（最短路径控制）和死区控制
 */
bool Follow_Calculate(gimbal_follow_instance_s* instance)
{
    if (instance == NULL || instance->up_angle_ptr == NULL)
    {
        return false;
    }

    // up_yaw-target=error=虚拟出的下云台位置-0
    float error = *(instance->up_angle_ptr) - instance->up_origin;
    
    
    // 归一化
    float half_range = instance->angle_range / 2.0f;
    while (error > half_range)  error -= instance->angle_range;
    while (error < -half_range) error += instance->angle_range;

    
    if (fabsf(error) < instance->dead_zone)
    {
        error = 0.0f;
    }

    
    //这里将 target 设为 error，measure 设为 0。
    //这样 PID 库内部计算 (target - measure) 刚好等于我们处理后的归一化 error。
    Pid_Calculate(instance->gimbal_follow_pid, error, 0.0f);
    
    instance->output = instance->gimbal_follow_pid->output;
    //最后角度误差再交由速度环处理
    return true;
}