/**
 * @file app_shooter_task.c
 * @author CGH
 * @brief 射击任务
 * 
 */

#include "app_shooter_task.h"


#define HALF_RANGE PI

#define right_wheel_vel_lpf_cutoff  25.0f;    // 截止频率（Hz），按需调整
#define right_wheel_vel_sample_freq  1000.0f; // 采样频率（Hz），默认1kHz
//实例声明
DjiMotorInstance_s *Left_Wheel;
DjiMotorInstance_s *Right_Wheel;


DjiMotorInstance_s *Trigger;

extern board_instance_t *board_instance;
#define DEBUG
#ifdef DEBUG
float test_speed_tr=0.0;
float target_speed_tr=0.0;
float test_position_tr=0.0;
float target_wheel_speed=0.0;
float test_output_tr=0.0;
float test_speed_left=0.0f;
uint8_t test_flag=1;
#endif
//拨弹盘原点
float pos_target_tr=FIRE_ORIGIN;
//变量声明
extern ShooterState_t Shooter_State;
extern ShooterState_t Shooter_State_last;
//float target_speed=0.0;//后续改为上位机提供
//float trigger_position=0.0;

LowpassFilter_t *right_wheel_vel_filter = NULL;
FilterInitConfig_t right_wheel_filter_config;

float filtered_right_vel;
float last_time=0.0f;
float heat=0.0f;
float cool=30;
uint8_t CtrlMode=0;



////////////////////////////电机配置/////////////////////////////////////////
static  DjiMotorInitConfig_s Left_Config = {
    .id = 2,                      // 电机ID(1~4)
    .type = M3508,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "LW",
    .can_config = {
        .can_number = 1,
				.topic_name = "LW",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x202,                     // 接收id
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
        .kp = 50.0,                       // 速度环比例系数
        .ki = 1.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 4000.0,                  // 积分限幅
        .out_max = 16000.0,                // 输出限幅(电流输出)
    }
};
static  DjiMotorInitConfig_s Right_Config = {
    .id = 1,                      // 电机ID(1~4)
    .type = M3508,               // 电机类型
    .control_mode = DJI_VELOCITY,  // 电机控制模式
		.topic_name = "RW",
    .can_config = {
        .can_number = 1,
				.topic_name = "RW",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x201,                     // 接收id
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
        .kp = 50.0,                       // 速度环比例系数
        .ki = 1.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 4000.0,                  // 积分限幅
        .out_max = 16000.0,                // 输出限幅(电流输出)
    }
};





static  DjiMotorInitConfig_s Trigger_Config = {
    .id = 3,                      // 电机ID(1~4)
    .type = M2006,               // 电机类型
    // .control_mode = DJI_VELOCITY,  // 电机控制模式
    .control_mode = DJI_POSITION,
		.topic_name = "Trigger",
    .can_config = {
        .can_number = 2,
				.topic_name = "Trigger",              // can句柄
        .tx_id = 0x200,                     // 发送id 
        .rx_id = 0x203,                     // 接收id
    },
    .reduction_ratio = (36.0/19.0)*47.0,              // 减速比

    .angle_pid_config = {
        .kp = 140.0,                        // 位置环比例系数
        .ki = 0.0,                        // 位置环积分系数
        .kd = 0.0,                        // 位置环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max =2*PI,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 400.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
        .kp = 1500.0,                       // 速度环比例系数
        .ki = 200.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 1000.0,                  // 积分限幅
        .out_max = 16000.0,                // 输出限幅(电流输出)
    }
};



  



//整体逻辑是这样：先进摩擦轮start使其转速稳定，稳定态下会进行一发拨弹。
//如果不稳定，则视为开火状态，不拨弹，除非其稳定
void StartShooterTask(void const * argument)
{

  /* USER CODE BEGIN StartShooterTask */
	Trigger=Motor_Dji_Register(&Trigger_Config);
	Left_Wheel=Motor_Dji_Register(&Left_Config);
  Right_Wheel=Motor_Dji_Register(&Right_Config);
  Shooter_State=SHOOTER_STOP;
  #ifdef DEBUG
  //Shooter_State = SHOOTER_TEST;
  #endif
  right_wheel_filter_config.cutoff_freq = right_wheel_vel_lpf_cutoff;
  right_wheel_filter_config.sample_freq = right_wheel_vel_sample_freq;
  right_wheel_filter_config.filter_size = 0; // 对低通滤波器未使用
  right_wheel_vel_filter = LowpassFilter_Register(&right_wheel_filter_config);



   


  //拨弹盘初始化
	pos_target_tr=Trigger->message.out_position;//防止出问题
  // 初始化时按固定方向旋转到原点
while (fabsf(Trigger->message.out_position - FIRE_ORIGIN) > 0.01f) {
    pos_target_tr -= 0.001f;
    Motor_Dji_Control(Trigger, pos_target_tr);
    Motor_Dji_Transmit(Trigger);
		if(Trigger->message.torque_current>MAX_TORQUE)break;
    osDelay(1);
}

  /* Infinite loop */
  for(;;)
  { 
    // CtrlMode=board_instance->received_control_mode;
    // switch (CtrlMode)
    // {
    // case SHOOT_MODE:
    //   if(board_instance->received_shoot_bool==1){
    //   Shooter_State=SHOOTER_START;
    //   }
    //   break;
    // case DISABLE_MODE:
    //   Shooter_State=SHOOTER_STOP;
    //   break;
    
      
    
    // default:
    //   Shooter_State=SHOOTER_STOP;
    //   break;
    // }


    Shooter_State_last=Shooter_State;

    switch(Shooter_State){
      case SHOOTER_READY:
				if(Shooter_State_last==SHOOTER_STOP){
        last_time=0;
        Shooter_State=SHOOTER_TRANS;
        break;
      }
				Pid_Enable(Left_Wheel->velocity_pid);
        Pid_Enable(Right_Wheel->velocity_pid);
				
				target_wheel_speed=SHOOTER_WHEEL_SPEED;
				pos_target_tr=Trigger->message.out_position;
				Motor_Dji_Control(Trigger,pos_target_tr);
				Motor_Dji_Transmit(Trigger);
				
			break;
      case SHOOTER_STOP:
        //清空PID
        //target_wheel_speed=0.0f;
				
        Pid_Disable(Left_Wheel->velocity_pid);
        Pid_Disable(Right_Wheel->velocity_pid);
        Pid_Disable(Trigger->angle_pid);
        Pid_Disable(Trigger->velocity_pid);

        break;
      case SHOOTER_2HOT:
        //过热保护
        //未完待续
       break;
      case SHOOTER_START:
      
      if(Shooter_State_last==SHOOTER_STOP){
        last_time=0;
        Shooter_State=SHOOTER_TRANS;
        break;
      }
      //检查力矩，如果力矩过大则视为开火状态，否则拨弹

      if(Left_Wheel->message.torque_current>FIRING_TORQUE||Right_Wheel->message.torque_current>FIRING_TORQUE){
        Shooter_State=SHOOTER_FIRING;
        continue;
      }
      heat+=10;
      pos_target_tr+=SHOOTER_RANGE;//转一个子弹的角度喵
      pos_target_tr=pos_target_tr>HALF_RANGE?pos_target_tr-2*HALF_RANGE:pos_target_tr;
      pos_target_tr=pos_target_tr<-HALF_RANGE?pos_target_tr+2*HALF_RANGE:pos_target_tr;
      Motor_Dji_Control(Trigger,pos_target_tr);
      target_wheel_speed=SHOOTER_WHEEL_SPEED;
      break;
      case SHOOTER_FIRING:
      //直到检测到电流小于某个值的时候再跳转回不稳定的初始状态
      if(Left_Wheel->message.torque_current<FIRING_TORQUE||Right_Wheel->message.torque_current<FIRING_TORQUE){
        Shooter_State=SHOOTER_START;
        continue;

      }
      
      target_wheel_speed=SHOOTER_WHEEL_SPEED;
      
      
      case SHOOTER_STUCK:
      if(Trigger->message.torque_current<MAX_TORQUE){
        Shooter_State=SHOOTER_FIRING;
        
      }


      last_time++;
      //回转一段距离避免卡弹
      if(last_time>500){
        if(last_time>1000){
          Shooter_State=SHOOTER_STOP;//急停,可能后面需要做个时不时查看是否有问题的机制。
        }
        pos_target_tr-=SHOOTER_RANGE*2;
        pos_target_tr=pos_target_tr>HALF_RANGE?pos_target_tr-2*HALF_RANGE:pos_target_tr;
        pos_target_tr=pos_target_tr<-HALF_RANGE?pos_target_tr+2*HALF_RANGE:pos_target_tr;
        Motor_Dji_Control(Trigger,pos_target_tr);

      }
        break;
      case SHOOTER_TRANS:
				
        Pid_Enable(Left_Wheel->velocity_pid);
        Pid_Enable(Right_Wheel->velocity_pid);
        Pid_Enable(Trigger->angle_pid);
        Pid_Enable(Trigger->velocity_pid);
        break;
      case SHOOTER_AUTO:
      //连射模式
      if(Shooter_State_last==SHOOTER_STOP){
        last_time=0;
        Shooter_State=SHOOTER_TRANS;
        break;
      }
			if(Trigger->message.torque_current>MAX_TORQUE){
				last_time++;
			}
      pos_target_tr+=SHOOTER_RANGE;
      pos_target_tr=pos_target_tr>HALF_RANGE?pos_target_tr-2*HALF_RANGE:pos_target_tr;
      pos_target_tr=pos_target_tr<-HALF_RANGE?pos_target_tr+2*HALF_RANGE:pos_target_tr;
      Motor_Dji_Control(Trigger,pos_target_tr);
      target_wheel_speed=SHOOTER_WHEEL_SPEED;
        break;
      case SHOOTER_TEST:
        //测试模式，直接控制任何一个电机
			Pid_Enable(Trigger->angle_pid);
			Pid_Enable(Trigger->velocity_pid);
		//	Pid_Enable(Left_Wheel->velocity_pid);
      //  Pid_Enable(Right_Wheel->velocity_pid);
				if(test_flag==1){
					pos_target_tr+=0.0007;
					pos_target_tr=pos_target_tr>HALF_RANGE?pos_target_tr-2*HALF_RANGE:pos_target_tr;
					pos_target_tr=pos_target_tr<-HALF_RANGE?pos_target_tr+2*HALF_RANGE:pos_target_tr;
				}
				target_wheel_speed=-2000.0f;
        Motor_Dji_Control(Trigger,pos_target_tr);
				
        break;
			default:
				break;
    }
    
		test_speed_tr=Right_Wheel->message.out_velocity;
		target_speed_tr=Right_Wheel->angle_pid->output;
    test_position_tr=Right_Wheel->message.out_position;
		test_output_tr=Right_Wheel->message.torque_current;
		test_speed_left=Left_Wheel->message.out_velocity;
		float test_left;

    LowpassFilter_Process(right_wheel_vel_filter, Right_Wheel->message.out_velocity, &filtered_right_vel);
    
    
		
		Motor_Dji_Transmit(Trigger);
//		if(board_instance->received_shoot_bool==1){
//      target_wheel_speed=SHOOTER_WHEEL_SPEED;}else{
//        target_wheel_speed=0.0f;

//      }
		Motor_Dji_Control(Right_Wheel,target_wheel_speed);
		


		//Motor_Dji_Control(Left_Wheel,filtered_right_vel);
		Motor_Dji_Control(Left_Wheel,-filtered_right_vel);
		Motor_Dji_Transmit(Right_Wheel);
    osDelay(1);

  }
  /* USER CODE END StartShooterTask */
}