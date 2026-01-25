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

    float up_origin;//上云台零点
    float angle_limit;
    float angle_range;//默认-PI到PI
    float *up_angle_ptr;//上云台角度指针
    float *down_angle_ptr;//下云台角度指针
    float dead_zone;//死区，避免大鸟转转转
} gimbal_follow_config_s;

/**
 * 
 * 
 * 
 * 
 */
typedef struct 
{

    float up_origin;//上云台零点
    float *up_angle_ptr;//上云台角度指针
    float *down_angle_ptr;//下云台角度指针
    float angle_range;//默认-PI到PI
    float dead_zone;//死区，避免大鸟转转转
    
    PidInstance_s *gimbal_follow_pid;//跟随pid
    float output;//输出值
} gimbal_follow_instance_s;















































#endif /*_DEV_GIMBALFOLLOW_H_*/