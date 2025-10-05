/**
 * @file app_gimbal_task.c
 * @author CGH
 * @brief 云台任务
 * @version 1.0
 */
#include "app_gimbal_task.h"
#define DEBUG

//实例声明
DmMotorInstance_s *Down_yaw;
//变量声明
#ifndef DEBUG
uint8_t ControlMode=DISABLE_MODE;
float target_position=0.0;//后续改为上位机提供
#else
uint8_t ControlMode= RC_MODE;
float target_position=0.0,test_speed=0.0,test_position=0.0,target_speed=0.0;
#endif
//电机配置/////////////////////////////////////////
static DmMotorInitConfig_s Down_config = {
    // .control_mode = DM_VELOCITY,    
	.control_mode = DM_POSITION, // 位置控制模式
		.topic_name = "down_yaw",
    .can_config = {
        .can_number = 1,
				.topic_name = "down_yaw",
        .tx_id = 0x009,
        .rx_id = 0x019,
        .can_module_callback = NULL,
    },
		.parameters = {
        .pos_max = 3.141593f,    // [查手册] 电机最大位置范围 (弧度)
        .vel_max = 45.0f,    // [查手册] 电机最大速度范围 (弧度/秒)
        .tor_max = 18.0f,    // [查手册] 电机最大扭矩范围 (N·m)
        .kp_max  = 500.0f,   // [查手册] Kp增益最大值
        .kd_max  = 5.0f,     // [查手册] Kd增益最大值
        .kp_int  = 0.0f,  // [调试设定] 要发送给电机的Kp值 (仅MIT模式)
        .kd_int  = 0.0f,     // [调试设定] 要发送给电机的Kd值 (仅MIT模式)
    },
    .angle_pid_config = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .kf = 0.0,
        .angle_max = 2.0f * 3.141593f,
        .i_max = 100.0,
        .out_max = 400.0,
    },
    .velocity_pid_config = {
        .kp = 0.0,
        .ki = 0.0,
        .kd = 0.0,
        .kf = 0.0,
        .angle_max = 0,
        .i_max = 1000.0,
        .out_max = 2000.0,
    }
};





void StartGimbalTask(void const * argument)
{
    Down_yaw=Motor_DM_Register(&Down_config);
    if(Down_yaw==NULL)
    {
        Log_Error("Down_yaw motor register failed\r\n");
        
    }
    while (Down_yaw->motor_state!=DM_ENABLE)
    {
        Motor_Dm_Cmd(Down_yaw,  DM_CMD_MOTOR_ENABLE);
        Motor_Dm_Transmit(Down_yaw);
    }
    Log_Information("Down_yaw motor enable success\r\n");
 
  for(;;)
  {
		#ifdef DEBUG
		test_speed=Down_yaw->message.out_velocity;
		test_position=Down_yaw->message.out_position;
		#endif
		switch (ControlMode) {
			case PC_MODE:
				break;
			case RC_MODE:
				Motor_Dm_Control(Down_yaw,target_position);
        Motor_Dm_Mit_Control(Down_yaw,0,0,Down_yaw->output);
				Motor_Dm_Transmit(Down_yaw);
				break;
			case DISABLE_MODE:
				break;
		}
    osDelay(1);
  }
  
}