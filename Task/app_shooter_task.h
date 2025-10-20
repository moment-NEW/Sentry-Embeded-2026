#ifndef APP_SHOOTER_TASK_H
#define APP_SHOOTER_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"


#include "bsp_log.h"
#include "bsp_can.h"


#include "dev_motor_dji.h"

//状态枚举,
typedef enum{
  SHOOTER_STOP=0,
  SHOOTER_START=1,
  SHOOTER_FIRING=2,
  SHOOTER_RELOAD=3
} ShooterState_t;








#endif // APP_SHOOTER_TASK_H