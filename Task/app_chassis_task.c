/**
 * 
 * @file app_chassis_task.c
 * @author CGH
 * @brief 底盘控制任务
 * @version V1.0.0
 */
#include "app_chassis_task.h"
//宏定义



//实例声明
ChassisInstance_s *Chassis;
//变量声明
uint8_t control_mode=RC_MODE;//默认遥控器模式

//配置
static ChassisInitConfig_s Chassis_config={
		.type = Omni_Wheel,
		.gimbal_yaw_zero =  2.07979059f,//-2.62492895,//(-10663.0f / 262144.0f) * 2.0f * 3.141593f
		//.gimbal_yaw_half = 0.130077288,//(251481.0f / 262144.0f) * 2.0f * 3.141593f
		.omni_message={
		.wheel_radius= 0.0765f,
	  .chassis_radius= 0.26176f,
		},
    
		.gimbal_follow_pid_config={
		  .kp = -3.0f,
      .ki = 0.0f,
      .kd = 0.0f,
			.dead_zone = 0.2f,
      .i_max = 0.0f,
      .out_max = 2 * 3.141593f,
		},
		.motor_config[0]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 1,
		.topic_name = "ch1",
    .can_config = {
            .can_number = 1,
			.topic_name = "ch1",
      .tx_id = 0x200,
      .rx_id = 0x201,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.8f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  },
		.motor_config[1]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 2,
		.topic_name = "ch2",
    .can_config = {
      .can_number = 1,
			.topic_name = "ch2",
      .tx_id = 0x200,
      .rx_id = 0x202,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.8f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  },.motor_config[2]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 3,
		.topic_name = "ch3",
    .can_config = {
      .can_number = 1,
			.topic_name = "ch3",
      .tx_id = 0x200,
      .rx_id = 0x203,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.8f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  },
	.motor_config[3]={
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .id = 4,
		.topic_name = "ch4",
    .can_config = {
      .can_number = 1,
			.topic_name = "ch4",
      .tx_id = 0x200,
      .rx_id = 0x204,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.8f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    }
  }
	};



//主任务
void StartChassisTask(void const * argument)
{
  /* USER CODE BEGIN StartChassisTask */
  Chassis = Chassis_Register(&Chassis_config);
    if (Chassis == NULL) {
        Log_Error("Chassis Register Failed!");
    }
  /* Infinite loop */
  for(;;)
  {
    switch (control_mode)
    {
    case PC_MODE:
        // Chassis->gimbal_yaw_angle
        break;
    case RC_MODE:
        /* code */
        Chassis_Mode_Choose(Chassis, CHASSIS_NORMAL);
        break;
    case TRANS_MODE:
        /* code */
        break;
    case DISABLE_MODE:
        
        break;
    default:
        break;
    }
    osDelay(1);
  }
  /* USER CODE END StartChassisTask */
}