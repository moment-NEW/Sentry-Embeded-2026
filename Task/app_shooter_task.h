#ifndef APP_SHOOTER_TASK_H
#define APP_SHOOTER_TASK_H

#include "FreeRTOS.h"
#include "cmsis_os.h"


#include "bsp_log.h"
#include "bsp_can.h"


#include "dev_motor_dji.h"


#define MAX_TORQUE 18.0f //正常状态下允许的电机最大扭矩
#define FIRING_TORQUE 22.0f //开火状态下允许的电机最大扭矩
#define FIRE_ORIGIN 0.0f//拨弹盘原点位置
#define SHOOTER_RANGE 0.1f//一颗弹丸的位置
#define HEAT_MAX 260;

//状态枚举,
typedef enum{
  SHOOTER_TEST=-1,
  SHOOTER_STOP=0,
  SHOOTER_START=1,
  SHOOTER_FIRING=2,
  SHOOTER_STUCK=3,
  SHOOTER_TRANS=4,
  SHOOTER_2HOT=5,
  SHOOTER_AUTO=6
} ShooterState_t;








#endif // APP_SHOOTER_TASK_H