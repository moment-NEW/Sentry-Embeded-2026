#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__

#include "robot_config.h"

#ifdef USER_CAN_FD
#include <stdint.h>
#include "fdcan.h"

#define FDCAN_MAX_REGISTER_CNT 16
#define MX_FDCAN_FILTER_CNT (USER_CAN_CNT *14)

#pragma pack(1)
typedef struct _CanInstance_s
{
    char* topic_name;
    FDCAN_HandleTypeDef *can_handle;                      // FDCAN 句柄
    FDCAN_TxHeaderTypeDef tx_conf;                        // FDCAN 报文发送配置
    uint16_t tx_id;                                       // 发送 id, 即发送的 FDCAN 报文 id
    uint8_t tx_buff[8];                                   // 发送缓存, 可以不用，但建议保留，方便调试
    uint16_t rx_id;                                       // 接收 id, 即接收的 FDCAN 报文 id
    uint8_t rx_buff[8];                                   // 接收缓存, 目前保留，增加了一次 memcpy 操作
    uint8_t rx_len;                                       // 接收长度, 可能为 0-8
    uint16_t cnt;                                        // 通信计数，每次中断+1
    void (*can_module_callback)(struct _CanInstance_s *); // 接收的回调函数, 用于解析接收到的数据，如果增加了 uint8_t *rx_buff 成员，前面的rx_buff[8] 可以删去
    void *id;                                 // 使用 can 外设的模块指针 (即 id 指向的模块拥有此 can 实例, 是父子关系)
}CanInstance_s;

typedef struct
{
    char* topic_name;                  //实例名称
    uint8_t can_number;               //can通道号 1,2,3 分别对应 FDCAN1, FDCAN2, FDCAN3，为了抽象接口向module层隐藏HAL库
    uint16_t tx_id;                    //发送id
    uint16_t rx_id;                    //接收id
    void (*can_module_callback)(struct _CanInstance_s *);   //接收的回调函数, 用于解析接收到的数据
    void *id;                                   //使用 can 外设的父指针 (即 id 指向的模块拥有此 can 实例, 是父子关系)
}CanInitConfig_s;
#pragma pack()
/**
 * @brief FDCAN接收帧结构体。
 * 该结构体用于存储从FDCAN接收的消息，包括消息头和数据缓冲区。
 */
typedef struct {
    FDCAN_RxHeaderTypeDef Header;
    uint8_t 			rx_buff[8];
}FDCAN_RxFrame_TypeDef;

CanInstance_s* Can_Register(const CanInitConfig_s* config);

/**
 * @brief 通过CAN总线发送数据。
 * 该函数将指定的数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @param tx_buff 指向要发送的数据缓冲区的指针，数据长度应为8
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit_External_Tx_Buff( CanInstance_s *instance, uint8_t *tx_buff);
/**
 * @brief 通过CAN总线发送数据,为了避免大修MODULE而写的函数
 * 该函数将实例内部的发送缓冲区数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit( CanInstance_s *instance);
#endif
#endif

