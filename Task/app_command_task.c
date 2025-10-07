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


//配置



void StartCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartCommandTask */
	dr16_instance = Dr16_Register(&huart3); // 注册遥控器实例
	Command_publisher=Create_Publisher("dr16_topic",sizeof(dr16_instance));
  /* Infinite loop */
  for(;;)
  {
		Publish_Message(Command_publisher, dr16_instance);
    osDelay(2);
  }
  /* USER CODE END StartCommandTask */
}