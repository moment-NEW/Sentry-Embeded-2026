/**
 * @file app_gimbal_task.c
 * @author CGH
 * @brief 云台任务
 * @version 1.0
 */
#include "app_gimbal_task.h"
#define DEBUG

//实例声明
//此处做了修改，现在完全不关心下云台电机
DmMotorInstance_s *pitch;
DjiMotorInstance_s *Up_yaw;
extern board_instance_t *board_instance;

//变量声明
uint8_t find_bool=0;
#ifndef DEBUG
uint8_t ControlMode=DISABLE_MODE;
float target_position=0.0;//后续改为上位机提供
float target_up_position=0.0;
#else
uint8_t ControlMode= RC_MODE;
float target_position=0.0,test_speed=0.0,test_position=0.0,target_speed=0.0,test_output=0.0;
float target_up_position=0.0;
float target_pitch_position=0.0;
float temp_position;
float output=0;
#endif
////////////////////////////电机配置/////////////////////////////////////////


static DmMotorInitConfig_s pitch_config = {
     //.control_mode = DM_VELOCITY,    
	.control_mode = DM_POSITION, // 位置控制模式
		.topic_name = "pitch",
    .can_config = {
        .can_number = 2,
				.topic_name = "pitch",
          .tx_id = 0x006,
        // .tx_id = 0x106,
         .rx_id = 0x016,
         
        .can_module_callback = NULL,
    },
		.parameters = {
        .pos_max = 6.2831853,    // [查手册] 电机最大位置范围 (弧度)
        .vel_max = 30.0f,    // [查手册] 电机最大速度范围 (弧度/秒)
        .tor_max = 18.0f,    // [查手册] 电机最大扭矩范围 (N·m)
        .kp_max  = 500.0f,   // [查手册] Kp增益最大值
        .kd_max  = 5.0f,     // [查手册] Kd增益最大值
        .kp_int  = 0.0f,  // [调试设定] 要发送给电机的Kp值 (仅MIT模式)
        .kd_int  = 0.0f,     // [调试设定] 要发送给电机的Kd值 (仅MIT模式)
    },
    .angle_pid_config = {
        .kp = 17,
        .ki = 0.0015,
        .kd = 0.06,
        .kf = 0.0,
        .angle_max = 2.0f * PI,
        .i_max = 100.0,
        .out_max = 400.0,
    },
    .velocity_pid_config = {
        .kp = 0.53,
        .ki = 0.009,
        .kd = 0.017,
        .kf = 0.0,
        .angle_max = 0,
        .i_max = 500.0,
        .out_max = 2000.0,//待商议
    }
};
static  DjiMotorInitConfig_s Up_config = {
    .id = 3 ,                      // 电机ID(1~4)
    .type = GM6020,               // 电机类型
    .control_mode = DJI_POSITION,  // 电机控制模式
		.topic_name = "up_yaw",
    .can_config = {
        .can_number = 2,
				.topic_name = "up_yaw",              // can句柄
        .tx_id = 0x1FE,                     // 发送id 
        .rx_id = 0x207,                     // 接收id
    },
    .reduction_ratio = 1,              // 减速比

    .angle_pid_config = {
        .kp = 160.0,                        // 位置环比例系数
        .ki = 0.0,                        // 位置环积分系数
        .kd = 0.0,                        // 位置环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0.0f,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 100.0,                   // 积分限幅
        .out_max = 400.0,                 // 输出限幅(速度环输入)
    },
    .velocity_pid_config = {
        .kp = 0.0,//100.0,                       // 速度环比例系数
        .ki = 0.0,                        // 速度环积分系数
        .kd = 0.0,                        // 速度环微分系数
        .kf = 0.0,                        // 前馈系数
        .angle_max = 0,                 // 角度最大值(限幅用，为0则不限幅)
        .i_max = 1000.0,                  // 积分限幅
        .out_max = 16000,                // 输出限幅(电流输出)
    }
};

//限幅0.17-0
//速度：0.8，0.001，角度：10
/**
* @brief 重力补偿用代码
* @param position 电机当前位置
*	@return float 输出力矩
*/
float G_feed(float position){
    //[354.533813, -99.411941, 7.604260, -0.441783]
    // float torque = -21.321498 * sin(position + -1.625094) + -21.348101;
	//[ 1.34165963 -1.72110943  0.29976696 -0.17707916]
	//[2.319374, -3.015623, 0.755581, -0.311670]
	//[1.549729, -2.003787, 0.406681, -0.285731]
    float torque = 1.549729*position*position*position +  (-2.003787*position*position) +0.406681*position + ( -0.285731);
    return torque; 
}
////////测试用代码///////////
#ifdef DEBUG
//假设end绝对大于start
/**
 * @brief 生成一个简单的斜坡轨迹，在start和end之间切分出若干个点，
 *        每隔固定的时间间隔切换到下一个点。
 * 
 * @param start 轨迹的起始位置 (rad)。
 * @param end 轨迹的结束位置 (rad)。
 * @param steps 从start到end分的步数。
 * @param interval_ms 切换到下一个点的时间间隔 (毫秒)。
 * 
 * @return float 当前应该对准的目标设定点 (rad)。
 */
float GenerateSimpleRampWithPause(float start, float end, int steps, uint32_t interval_ms)
{
    // --- 静态变量，用于在函数调用间保持状态 ---
    static float target_setpoint = 0.0f; // 当前的目标设定点
    static int current_step = 0;         // 当前所在的步数
    static uint32_t last_update_time = 0; // 上次更新目标点的时间

    // --- 静态变量，用于检测输入参数变化并重置 ---
    static float static_start = -1.0f;
    static float static_end = -1.0f;
    static int static_steps = -1;

    uint32_t current_time = osKernelSysTick(); // 获取当前系统时间

    // 1. 初始化或重置
    // 如果调用时 start, end 或 steps 的值变了，就重新初始化
    if (start != static_start || end != static_end || steps != static_steps) {
        target_setpoint = start;
        current_step = 0;
        last_update_time = current_time;

        // 保存当前的参数，用于下次比较
        static_start = start;
        static_end = end;
        static_steps = steps;
    }

    // 2. 检查是否到达更新时间
    if (current_time - last_update_time >= interval_ms)
    {
        // 如果还没有到达最后一步
        if (current_step < steps)
        {
            current_step++; // 移动到下一步
            last_update_time = current_time; // 更新时间戳
        }
    }
    
    // 3. 计算当前的目标设定点
    if (steps > 0) {
        target_setpoint = start + (end - start) * ((float)current_step / (float)steps);
    } else {
        target_setpoint = start; // 如果步数为0，则目标点始终为起点
    }

    // 确保最终目标点不会超过终点
    if ((end > start && target_setpoint > end) || (end < start && target_setpoint < end)) {
        target_setpoint = end;
    }

    // 4. 返回当前计算出的目标值
    return target_setpoint;
}
/**
 * @brief 生成一个在端点暂停并自动往复的斜坡信号。
 *        此版本基于时间间隔进行更新。
 * 
 * @param min_pos       运动区间的最小值
 * @param max_pos       运动区间的最大值
 * @param steps         从一端到另一端所需的步数
 * @param interval_ms   每一步之间的时间间隔 (毫秒)
 * @param pause_ms      在端点暂停的时间 (毫秒)
 * @return float        当前的目标设定点
 */
float GenerateReversingRamp(float min_pos, float max_pos, int steps, uint32_t interval_ms, uint32_t pause_ms)
{
    // --- 静态变量，用于在函数调用间保持状态 ---
    static float target_setpoint = 0.0f;
    static int current_step = 0;
    static uint32_t last_update_time = 0;
    static char is_forward_trip = 1; // 1: min->max, 0: max->min
    static char is_paused = 0;       // 1: 正在暂停, 0: 正在运动

    // --- 用于初始化的静态变量 ---
    static int initialized = 0;
    uint32_t current_time = osKernelSysTick();

    // 1. 仅在第一次调用时进行初始化
    if (!initialized) {
        target_setpoint = min_pos;
        current_step = 0;
        is_forward_trip = 1;
        is_paused = 0;
        last_update_time = current_time;
        initialized = 1;
    }

    // 2. 状态机逻辑
    if (is_paused) // 如果当前处于暂停状态
    {
        if (current_time - last_update_time >= pause_ms)
        {
            // 暂停结束，准备“掉头”
            is_forward_trip = !is_forward_trip; // 切换方向
            current_step = 0;                   // 重置步数
            is_paused = 0;                      // 退出暂停状态
            last_update_time = current_time;    // 更新时间戳，开始新的运动
        }
    }
    else // 如果当前处于运动状态
    {
        if (current_time - last_update_time >= interval_ms)
        {
            if (current_step < steps)
            {
                current_step++; // 移动到下一步
                last_update_time = current_time;
            }
            
            if (current_step >= steps)
            {
                // 到达端点，开始暂停
                is_paused = 1;
                last_update_time = current_time; // 重置暂停计时器
            }
        }
    }

    // 3. 根据当前状态计算目标点
    float start_pos = is_forward_trip ? min_pos : max_pos;
    float end_pos = is_forward_trip ? max_pos : min_pos;

    if (steps > 0) {
        target_setpoint = start_pos + (end_pos - start_pos) * ((float)current_step / (float)steps);
    } else {
        target_setpoint = start_pos;
    }

    // 4. 返回当前计算出的目标值
    return target_setpoint;
}

#endif



float Forbidden_Zone(float start,float end,float current,float target,float circle_range){
    static float last_target;
    if(fabs(last_target - target) > 1e-2f){
    last_target=target;
    }else{
        return last_target;
    }
	float erro=target-current;
	uint8_t crosses_forbidden_zone=0;
		 if (start < end && target >= start && target <= end) {//假设start<end,反正不是写库，将就着用了
        
        target = (target < (start + end) / 2.0f) ? start : end;
    }
	//手动实现过零保护,和PID里的不一样，修改target而不是current,这样方便后续更改
    if (erro > circle_range / 2.0f) {
        erro -= circle_range; // 例如: 从 10 度到 350 度，erro = 340。应该走 -20 度。 340 - 360 = -20
    } else if (erro < -circle_range / 2.0f) {
        erro += circle_range; // 例如: 从 350 度到 10 度，erro = -340。应该走 +20 度。 -340 + 360 = 20
    }
		float target_ture=current+erro;
	if (start < end) {
        // 从 current 沿 erro 方向移动是否会进入 [start, end] 区间
        if (erro > 0 && current < start && target_ture > end) { // 正向移动穿过
            crosses_forbidden_zone = 1;
        } else if (erro < 0 && current > end && target_ture < start) { // 反向移动穿过
            crosses_forbidden_zone = 1;
        }
    }
    // 情况2：禁区跨越0度（例如, start=300, end=30）
    else { 
        // 此时禁区实际上是 [start, 360) 和 [0, end] 两个区间
        if (erro > 0 && current < start && target_ture > end && target_ture < current) { // 正向移动，且跨越了0点
             crosses_forbidden_zone = 1;
        } else if (erro < 0 && current > end && target_ture < start && target_ture > current) { // 反向移动，且跨越了0点
             crosses_forbidden_zone = 1;
        }
    }

	if(crosses_forbidden_zone){
		if(erro<0){
			erro=erro+circle_range;
		}else{
			erro=erro-circle_range;
		}
	}
    target_ture=current+erro;
	return target_ture;
}

void StartGimbalTask(void const * argument)
{
		#ifdef DEBUG
    uint32_t dwt_cnt_last3 = 0;
    float dt3 = 0.001f;  // 初始dt
    static float lasttime3 = 0;
    
    // 初始化DWT计数器
    dwt_cnt_last3 = DWT->CYCCNT;
		#endif
	
    pitch=Motor_DM_Register(&pitch_config);//4310
    Up_yaw=Motor_Dji_Register(&Up_config);//6020
	
    
		 if(Up_yaw==NULL)
    {
        Log_Error("Up_yaw motor register failed\r\n");
        
    }

	   while (pitch->motor_state!=DM_ENABLE)
    {
        Motor_Dm_Cmd(pitch,  DM_CMD_MOTOR_ENABLE);
        Motor_Dm_Transmit(pitch);
        osDelay(1);
    }
    Log_Information("pitch motor enable success\r\n");
 
  for(;;)
  {
		#ifdef DEBUG
		test_speed=pitch->message.out_velocity;
		test_position=pitch->message.out_position;
		target_speed=pitch->angle_pid->output;
		 dt3 = Dwt_GetDeltaT(&dwt_cnt_last3);
       
        // 限制dt范围，防止异常值
        if (dt3 > 0.01f || dt3 <= 0.0f) {
            dt3 = 0.001f;  // 默认1ms
        }
			Pid_Disable(pitch->velocity_pid);
		#endif
        board_send_message(board_instance, Up_yaw->message.out_position, 0, 0, find_bool);
		switch (ControlMode) {
			case PC_MODE:
				break;
			case RC_MODE:

			//Pitch轴
			//限幅
				#ifdef DEBUG
               
//       target_position=GenerateReversingRamp(0, 1, 50, 6000, 6000); //50个点，间隔2s，端点停止2s
//       Motor_Dm_Pos_Vel_Control(pitch,target_position,10);
//			Motor_Dm_Mit_Control(pitch,0,0,G_feed(pitch->message.out_position));
			#endif
			if(pitch->control_mode==DM_POSITION){
				 target_position=target_position>1?1:target_position;
				 target_position=target_position<0.0?0.0:target_position;
				
			}
				
			 Motor_Dm_Control(pitch,target_position);
			
			 output=pitch->output+G_feed(pitch->message.out_position);
				
            Motor_Dm_Mit_Control(pitch,0,0,output);
				
				
				#ifdef DEBUG
				test_output=pitch->message.torque;
			#endif
			 Motor_Dm_Transmit(pitch);
			
			//大疆
			  temp_position=target_up_position;
	//		  temp_position=Forbidden_Zone(-0.1,1.1,Up_yaw->message.out_position,target_up_position,2*PI);
				Motor_Dji_Control(Up_yaw,temp_position);
				Motor_Dji_Transmit(Up_yaw);
				break;
			case DISABLE_MODE:
				Motor_Dm_Cmd(pitch,DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(pitch);
				Motor_Dji_Control(Up_yaw,target_position);//暂时的逻辑
			//其他同理
			
				Up_yaw->output=0;
				Motor_Dji_Transmit(Up_yaw);
				break;
			case TRANS_MODE:
	            Motor_Dm_Cmd(pitch, DM_CMD_MOTOR_ENABLE);
                Motor_Dm_Transmit(pitch);
				break;
		}
		
    osDelay(1);
  }
  
}