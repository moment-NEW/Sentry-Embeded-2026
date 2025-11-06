#include "FreeRTOS.h"

#include "bsp_can.h"


#include "Com_System.h"
#include "app_command_task.h"
#define BOARD_TX_ID_BASE 0x600
//实际发送的id是 BOARD_TX_ID_BASE + board_id，接收的id是 BOARD_RX_ID_BASE + board_id
#define BOARD_RX_ID_BASE 0x700

/**
 * @brief 下发上数据包结构体
 */
typedef struct{
    uint8_t control_mode;
    float down_yaw_pos;
} down2up_message_t;
/**
 * @brief 上发下数据包结构体
 */
typedef struct{
    uint8_t findbool;
    float up_yaw_pos;
} up2down_message_t;
/** 
* @brief 需要发送的数据包结构体
*
*/
typedef struct
{   
    uint8_t data_source;

    uint8_t start;
    uint8_t id;
    uint8_t data[8];
    uint8_t end;
} board_message_t;

/**
 * @brief 实例，用于板间通信
 * 
 */
typedef struct
{
    uint8_t board_id;                            /**< 板子ID */
    CanInstance_s can_instance;                /**< CAN实例 */
    board_message_t board_message;      /**< 板间通信数据包 */
} board_instance_t;

typedef struct 
{
    uint8_t board_id;
    CanInitConfig_s can_config;
    uint8_t message_type; // 0: down2up_message_t, 1: up2down_message_t
} board_config_t;

board_instance_t board_init(board_config_t *config);
void board_send_message(board_instance_t *instance);
