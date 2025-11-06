#ifndef APP_CHASSIS_TASK_H
#define APP_CHASSIS_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"

#include "dev_motor_dm.h"
#include "dev_motor_dji.h"

#include "dev_minipc.h"
#include "dev_dr16.h"
#include "Com_System.h"

#include "alg_chassis_calc.h"
#include "alg_pid.h"

#include "bsp_can.h"
#include "bsp_log.h"
#include "bsp_dwt.h"

#include "app_command_task.h"

//宏定义
#define PC_MODE 1
#define RC_MODE 2
#define TRANS_MODE 3//用于失能，使能之间的过渡
#define DISABLE_MODE 0



#endif // APP_CHASSIS_TASK_H