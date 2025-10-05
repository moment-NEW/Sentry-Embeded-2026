#ifndef APP_GIMBAL_TASK_H
#define APP_GIMBAL_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "main.h"


#include "robot_config.h"
// BSP层
#include "bsp_log.h"
// module层

#include "dev_dr16.h"
#include "dev_motor_dm.h"
#include "dev_motor_dji.h"

// 通信系统
#include "Com_System.h"



//宏定义
#define PC_MODE 1
#define RC_MODE 2
#define TRANS_MODE 3//用于失能，使能之间的过渡
#define DISABLE_MODE 0

// 函数声明
void StartGimbalTask(void const * argument);

#endif // APP_GIMBAL_TASK_H


