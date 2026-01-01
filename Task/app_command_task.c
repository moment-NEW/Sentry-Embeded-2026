/**
 * @file app_command_task.c
 * @author CGH
 * @brief 指令处理任务
 * @version V1.0.0
 */
#include "app_command_task.h"

Dr16Instance_s *dr16_instance;
MiniPC_Instance *minipc_instance;
board_instance_t *board_instance;
Publisher *Command_publisher;
ShooterState_t Shooter_State;
ShooterState_t Shooter_State_last;


//配置
board_config_t board_config = {
.board_id=1,
.can_config={
  .can_number =2,//记得改回来
  .topic_name = "Board_Comm"
},
.message_type = UP2DOWN_MESSAGE_TYPE



};


//变量
uint8_t mode=0,last_mode=0;



void StartCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartCommandTask */
	
	Command_publisher=Create_Publisher("board_topic",sizeof(board_instance_t));
  board_instance = board_init(&board_config);
  
  /* Infinite loop */
  for(;;)
  {
		Publish_Message(Command_publisher, board_instance);
    mode=board_instance->received_control_mode;
    Shooter_State_last=Shooter_State;
    switch (mode)
    {
    case SHOOT_MODE:
      if(board_instance->received_shoot_bool==1){
      Shooter_State=SHOOTER_START;
      }
      break;
    case DISABLE_MODE:
      Shooter_State=SHOOTER_STOP;
      break;
    
      
    
    default:
      Shooter_State=SHOOTER_STOP;
      break;
    }
    osDelay(2);
  }
  /* USER CODE END StartCommandTask */
}