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
DmMotorInstance_s *pitch;
DjiMotorInstance_s *Up_yaw;
//变量声明
#ifndef DEBUG
uint8_t ControlMode=DISABLE_MODE;
float target_position=0.0;//后续改为上位机提供
float target_up_position=0.0;
#else
uint8_t ControlMode= RC_MODE;
float target_position=0.0,test_speed=0.0,test_position=0.0,target_speed=0.0;
float target_up_position=0.0;
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
        .angle_max = 2.0f * PI,
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

static DmMotorInitConfig_s pitch_config = {
    // .control_mode = DM_VELOCITY,    
	.control_mode = DM_POSITION, // 位置控制模式
		.topic_name = "pitch",
    .can_config = {
        .can_number = 1,
				.topic_name = "pitch",
        .tx_id = 0x006,
        .rx_id = 0x016,
        .can_module_callback = NULL,
    },
		.parameters = {
        .pos_max = 3.141593f,    // [查手册] 电机最大位置范围 (弧度)
        .vel_max = 30.0f,    // [查手册] 电机最大速度范围 (弧度/秒)
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
        .angle_max = 2.0f * PI,
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
static  DjiMotorInitConfig_s Up_config = {
    .id = 1 ,                      // 电机ID(1~4)
    .type = GM6020,               // 电机类型
    .control_mode = DJI_POSITION,  // 电机控制模式
		.topic_name = "up_yaw",
    .can_config = {
        .can_number = 1,
				.topic_name = "up_yaw",              // can句柄
        .tx_id = 0x1FE,                     // 发送id 
        .rx_id = 0x205,                     // 接收id
    },
    .reduction_ratio = 1,              // 减速比

    .angle_pid_config = {
        .kp = 0.0,                        // 位置环比例系数
        .ki = 0.0,                        // 位置环积分系数
        .kd = 0.0,                        // 位置环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 2.0f * PI,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 400.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
        .kp = 0.0,                       // 速度环比例系数
        .ki = 0.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 1000.0,                  // 积分限幅
        .out_max = 2000.0,                // 输出限幅(电流输出)
    }
};


void StartGimbalTask(void const * argument)
{
    Down_yaw=Motor_DM_Register(&Down_config);//8006
		pitch=Motor_DM_Register(&pitch_config);//4310
		Up_yaw=Motor_Dji_Register(&Up_config);//6020
		
    if(Down_yaw==NULL)
    {
        Log_Error("Down_yaw motor register failed\r\n");
        
    }
		 if(Up_yaw==NULL)
    {
        Log_Error("Up_yaw motor register failed\r\n");
        
    }
//    while (Down_yaw->motor_state!=DM_ENABLE)
//    {
//        Motor_Dm_Cmd(Down_yaw,  DM_CMD_MOTOR_ENABLE);
//        Motor_Dm_Transmit(Down_yaw);
//    }
//	   while (pitch->motor_state!=DM_ENABLE)
//    {
//        Motor_Dm_Cmd(pitch,  DM_CMD_MOTOR_ENABLE);
//        Motor_Dm_Transmit(pitch);
//    }
    Log_Information("pitch motor enable success\r\n");
 
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
//				Motor_Dm_Control(Down_yaw,target_position);
//        Motor_Dm_Mit_Control(Down_yaw,0,0,Down_yaw->output);
//				Motor_Dm_Transmit(Down_yaw);
			
			//大疆
				Motor_Dji_Control(Up_yaw,target_up_position);
				Motor_Dji_Transmit(Up_yaw);
				break;
			case DISABLE_MODE:
				Motor_Dm_Cmd(Down_yaw,DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(Down_yaw);
				Motor_Dji_Control(Up_yaw,target_position);//暂时的逻辑
				Up_yaw->output=0;
				Motor_Dji_Transmit(Up_yaw);
				break;
			case TRANS_MODE:
	      Motor_Dm_Cmd(Down_yaw,  DM_CMD_MOTOR_ENABLE);
        Motor_Dm_Transmit(Down_yaw);
				break;
		}
		
    osDelay(1);
  }
  
}