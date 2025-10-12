/**
 * @file app_command_task.c
 * @author CGH
 * @brief 指令处理任务
 * @version V1.0.0
 */
#include "app_command_task.h"

Dr16Instance_s *dr16_instance;
MiniPC_Instance *minipc_instance;
Publisher *Command_publisher;

//变量
uint8_t mode=0;
//配置
/**
 * @brief 根据遥控器的拨杆位置确定控制模式
 * @param dr16 指向遥控器实例的指针
 * @return uint8_t 返回当前的控制模式
 */
uint8_t Mode_Change(Dr16Instance_s *dr16){
  uint8_t mode=0;//DISABLE_MODE

   // 将s1的值左移4位，然后与s2的值进行'或'运算。
  // 这样s1占据高4位，s2占据低4位，形成一个唯一的8位状态值。
  // 例如: s1=1, s2=2  ->  (1 << 4) | 2  ->  0x10 | 0x02  ->  0x12
  uint8_t combined_state = (dr16->dr16_handle.s1 << 4) | dr16->dr16_handle.s2;
  switch (combined_state)
  {
    case 0x11: // s1=1 (上), s2=1 (上)
      mode = PC_MODE;
			return mode;
      break;
    
    case 0x13: // s1=1 (上), s2=3 (中)
      mode = RC_MODE;
		return mode;
      break;

    case 0x12: // s1=1 (上), s2=2 (下)
      

    case 0x31: // s1=3 (中), s2=1 (上)
    
    case 0x32:
    case 0x33:
      
     

    default: // 其他所有情况 (包括 s1=2, s2在任意位置)
      mode = DISABLE_MODE;
      break;
  }
  return mode;
  
}

void StartCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartCommandTask */
	dr16_instance = Dr16_Register(&huart3); // 注册遥控器实例
	Command_publisher=Create_Publisher("dr16_topic",sizeof(Dr16Instance_s));

  
  /* Infinite loop */
  for(;;)
  {
		Publish_Message(Command_publisher, dr16_instance);
    mode=Mode_Change(dr16_instance);
    osDelay(2);
  }
  /* USER CODE END StartCommandTask */
}