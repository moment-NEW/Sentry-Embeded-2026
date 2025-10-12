#ifndef APP_COMMAND_TASK_H
#define APP_COMMAND_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "dev_dr16.h"
#include "dev_minipc.h"
#include "Com_System.h"

#include "bsp_log.h"

#define PC_MODE 1
#define RC_MODE 2
#define TRANS_MODE 3//用于失能，使能之间的过渡
#define DISABLE_MODE 0


#endif // APP_COMMAND_TASK_H
