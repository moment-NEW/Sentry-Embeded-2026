/**
 * @file dev_board_communicate.c
 * @author CGH
 * @brief 板间通信
 * 
 */
#include "dev_board_communicate.h"
#include <string.h>

// 函数声明
static uint16_t float2half(float rawdata);
static float half2float(uint16_t halfdata);

board_instance_t* board_init(board_config_t *config)
{
    if(config == NULL)
    {
        #ifdef DEBUG
        Log_Error("Board config is NULL\r\n");
        #endif
        return NULL;
    }
    // 动态分配内存
    board_instance_t* instance = (board_instance_t*)pvPortMalloc(sizeof(board_instance_t));
    if (instance == NULL)
    {
        #ifdef DEBUG
        Log_Error("Failed to allocate memory for board instance\r\n");
        #endif
        return NULL;
    }
    memset(instance, 0, sizeof(board_instance_t));
  
    config->can_config.tx_id = BOARD_TX_ID_BASE + config->board_id;
    config->can_config.rx_id = BOARD_RX_ID_BASE + config->board_id;
    config->can_config.parent_ptr = instance; // 父指针指向实例本身
    config->can_config.can_module_callback = Board_Message_Decode;
    config->can_config.topic_name = "Board_Comm";
    instance->board_id = config->board_id;
    instance->can_instance = Can_Register(&config->can_config);
    
    instance->message_type = config->message_type;
    return instance; // 返回分配好的实例的指针
}

void board_send_message(board_instance_t *instance, float data1, float data2, uint8_t flag1, uint8_t flag2)
{
    if(instance == NULL || instance->can_instance == NULL)
    {
        #ifdef DEBUG
        Log_Error("Board instance or CAN instance is NULL\r\n");
        #endif
        return;
    }

    // 根据消息类型，使用宏填充8字节的数据缓冲区
    switch (instance->message_type)
    {
        case 0: // 发送 down2up_message_t
        {
            down2up_message_t *msg = ACCESS_AS_STRUCT(instance, down2up_message_t);
            msg->control_mode = flag1;
            msg->shoot_bool = flag2;
            msg->up_target = float2half(data1);
            msg->down_yaw_pos = float2half(data2);
            break;
        }
        case 1: // 发送 up2down_message_t
        {
            up2down_message_t *msg = ACCESS_AS_STRUCT(instance, up2down_message_t);
            msg->findbool = flag1;
            msg->up_yaw_pos = float2half(data1);
            break;
        }
        default:
            #ifdef DEBUG
            Log_Error("Unknown message type for board ID %d\r\n", instance->board_id);
            #endif
            return;
    }

    // 将填充好的8字节缓冲区数据拷贝到CAN发送缓冲区并发送
    memcpy(instance->can_instance->tx_buff, instance->data_buffer, 8);
    if(!Can_Transmit(instance->can_instance))
    {
        #ifdef DEBUG
        Log_Error("Board ID %d message send failed\r\n", instance->board_id);
        #endif
    }
}

void Board_Message_Decode(CanInstance_s *can_instance)
{
    if(can_instance == NULL || can_instance->parent_ptr == NULL)
    {
        return;
    }
    board_instance_t *board_instance = (board_instance_t *)can_instance->parent_ptr;

    // 将CAN接收缓冲区的8字节数据拷贝到实例的数据缓冲区
    memcpy(board_instance->data_buffer, can_instance->rx_buff, 8);

    // 根据消息类型，解码数据并存放到解析后的成员中
    switch (board_instance->message_type)
    {
        case 0: // 接收到 down2up_message_t
        {
            
            up2down_message_t *msg = ACCESS_AS_STRUCT(board_instance, up2down_message_t);
            board_instance->received_find_bool = msg->findbool;
            board_instance->received_up_yaw_pos = half2float(msg->up_yaw_pos);
            break;
        }
        case 1: // 接收到 up2down_message_t
        {
            down2up_message_t *msg = ACCESS_AS_STRUCT(board_instance, down2up_message_t);
            board_instance->received_control_mode = msg->control_mode;
            board_instance->received_shoot_bool = msg->shoot_bool;
            board_instance->received_target_up_yaw = half2float(msg->up_target);
            board_instance->received_current_down_yaw = half2float(msg->down_yaw_pos);
            break;
        }
        default:
            #ifdef DEBUG
            Log_Error("Unknown message type for board ID %d\r\n", board_instance->board_id);
            #endif
            return;
    }

}

//0000 0000 0000 0001，将第一位移到最后
//0000 0000 1111 1111,取最后8位，也就是32位移位23位后的前8位被取
//0000 0111 1111 1111 1111 1111 1111,取了23位，也就取到了尾数位

static uint16_t float2half(float rawdata)
{
   uint16_t result;
  

    FloatUnion fu;
    fu.f = rawdata; 
    uint32_t f32_bits = fu.u;//提供32位整形的形式安全访问float

    // 从32位整数中提取 S, E, M
    uint32_t sign_f32     = (f32_bits >> 31) & 0x01;//(实际不取也行，移位后已经是S了)
    uint32_t exp_f32 = (f32_bits >> 23) & 0xFF;//这里如上，实际不用理论上也行
    uint32_t mant_f32 =  f32_bits        & 0x7FFFFF;
    //至此32位证书的S,E,M提取完毕

    if(exp_f32 ==0){//处理特殊值0和无穷大
        result = (sign_f32 << 15);
        return result;
        }else if(exp_f32==255){
            result = (sign_f32 << 15) | 0x7C00 | (mant_f32 >> 13);
            return result;
            }
        
    if(exp_f32 > 142||exp_f32<113){//溢出,报错  （这里都是正数是因为偏移量不一致喵)
        result = (sign_f32 << 15) | 0x7C00; //表示为无穷大
        return result;
    }
    result = (uint16_t)(sign_f32 << 15) | ((exp_f32 - 112) << 10) | (mant_f32 >> 13);//122=127-15

    return result;
   
}
//float的结构：1位符号位，8位指数位，23位尾数位
//half的结构：1位符号位，5位指数位，10位尾数位
//0000 0011 1111 1111（也就是0x3FF）作为掩码取低10位
//0000 0001 1111 0000(也就是0x1F0)作为掩码取中间5位
static float half2float(uint16_t halfdata)
{
    float result;
    uint32_t sign = (halfdata >> 15) & 0x1;//符号位
    uint32_t exp = ((halfdata >> 10) & 0x1F) - 15 + 127;//指数位，半精度浮点偏移量（Bias）是 15,这个127是float偏移量（为了映射到正负数）
    uint32_t mant = (halfdata & 0x3FF) << 13;//尾数位,移位后相当于把低10位放到高23位的位置上
    uint32_t floatdata = (sign << 31) | (exp << 23) | mant;//重组32位float
    memcpy(&result, &floatdata, sizeof(float));
    return result;
}
