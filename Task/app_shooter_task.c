/**
 * @file app_shooter_task.c
 * @author CGH
 * @brief 射击任务
 * 
 */

#include "app_shooter_task.h"


//实例声明
DjiMotorInstance_s *Left_Wheel;
DjiMotorInstance_s *Right_Wheel;


DjiMotorInstance_s *Trigger;
#define DEBUG
#ifdef DEBUG
float test_speed_tr=0.0;
float target_speed_tr=0.0;
#endif
//变量声明
ShooterState_t Shooter_State;
//float target_speed=0.0;//后续改为上位机提供
//float trigger_position=0.0;

////////////////////////////电机配置/////////////////////////////////////////
static  DjiMotorInitConfig_s Left_Config = {
    .id = 3,                      // 电机ID(1~4)
    .type = M3508,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "up_yaw",
    .can_config = {
        .can_number = 1,
				.topic_name = "up_yaw",              // can句柄
        .tx_id = 0x1FE,                     // 发送id 
        .rx_id = 0x207,                     // 接收id
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
static  DjiMotorInitConfig_s Right_Config = {
    .id = 3,                      // 电机ID(1~4)
    .type = M3508,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "up_yaw",
    .can_config = {
        .can_number = 1,
				.topic_name = "up_yaw",              // can句柄
        .tx_id = 0x1FE,                     // 发送id 
        .rx_id = 0x207,                     // 接收id
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





static  DjiMotorInitConfig_s Trigger_Config = {
    .id = 3,                      // 电机ID(1~4)
    .type = M2006,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "up_yaw",
    .can_config = {
        .can_number = 2,
				.topic_name = "up_yaw",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x203,                     // 接收id
    },
    .reduction_ratio = 36,              // 减速比

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

void StartShooterTask(void const * argument)
{
  /* USER CODE BEGIN StartShooterTask */
	Trigger=Motor_Dji_Register(&Trigger_Config);
	
  /* Infinite loop */
  for(;;)
  { 
    //单独设计一个卡弹检测，如果拨弹盘电机的电流输出高于某个阈值一段时间视为卡弹
    switch(Shooter_State){
      case SHOOTER_STOP:
        //清空PID
      case SHOOTER_START:
      //先检查摩擦轮电流，电流大于正常值的时候视为正在经历开火
      case SHOOTER_FIRING:
      //直到检测到电流小于某个值的时候再跳转回普通开火模式
      case SHOOTER_STUCK:
      //回转一段距离避免卡弹
        break;
			default:
				break;
    }
		test_speed_tr=Trigger->message.out_velocity;
		Motor_Dji_Control(Trigger,target_speed_tr);
		Motor_Dji_Transmit(Trigger);
    osDelay(1);
  }
  /* USER CODE END StartShooterTask */
}