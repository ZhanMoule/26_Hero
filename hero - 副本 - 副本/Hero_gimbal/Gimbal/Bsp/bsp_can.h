/**
* @file bsp_can.h
 * @author He WenXuan (hewenxuan040923@gmail.com)
 * @brief CAN驱动模块
 * @version 0.1
 * @details CAN驱动模块,提供CAN的初始化、发送、接收等功能
 * @date 2025-07-04
 * @copyright  Copyright (c) 2025 HDU—PHOENIX
 */

#ifndef __BSP_CAN_H__
#define __BSP_CAN_H__
#include "robot_config.h"
#ifdef USER_CAN_STANDARD
#include <stdint.h>
#include "can.h"
#include <stdbool.h>
/**
 *@brief 1路CAN最大注册实例数，1M Baud rate下最多建议8个实例
 */
#define CAN_MAX_REGISTER_CNT 8

#pragma pack(1)
/**
 *@brief CAN实例结构体
 */
typedef struct _CanInstance_s
{
    char* topic_name;                                     //实例名称
    CAN_HandleTypeDef *can_handle;                        // CAN 句柄
    CAN_TxHeaderTypeDef tx_header;                          // CAN 报文发送配置
    uint16_t tx_id;                                       // 发送 id, 即发送的 CAN 报文 id
    uint8_t tx_buff[8];                                   // 发送缓存, 可以不用，但建议保留，方便调试
    uint16_t rx_id;                                       // 接收 id, 即接收的 CAN 报文 id
    uint8_t rx_buff[8];                                   // 接收缓存, 目前保留，增加了一次 memcpy 操作，方便监视所以保留
    uint8_t rx_len;                                       // 接收长度, 可能为 0-8
    uint16_t cnt;                                        // 通信计数，每次中断+1
    void (*can_module_callback)(struct _CanInstance_s *); // 接收的回调函数, 用于解析接收到的数据，如果增加了 uint8_t *rx_buff 成员，前面的rx_buff[8] 可以删去
    void *parent_ptr;                                             // 使用 can 外设的模块指针 (即 id 指向的模块拥有此 can 实例, 是父子关系)
}CanInstance_s;
/**
 *@brief CAN初始化配置结构体
 */
typedef struct{
    char* topic_name;                                     //实例名称
    uint8_t can_number;                                  //can 1,2 分别对应 CAN1, CAN2，为了抽象接口向module层隐藏HAL库
    uint16_t tx_id;                                       //发送id
    uint16_t rx_id;                                       //接收id
    void (*can_module_callback)(struct _CanInstance_s *); //接收的回调函数, 用于解析接收到的数据
    void *parent_ptr;                                             //使用 can 外设的父指针 (即 id 指向的模块拥有此 can 实例, 是父子关系)
}CanInitConfig_s;
#pragma pack()
/**
 * @brief CAN接收帧结构体。
 * 该结构体用于存储从CAN接收的消息，包括消息头和数据缓冲区。
 */
typedef struct {
    CAN_RxHeaderTypeDef RxHeader;                         // CAN 消息头
    uint8_t 			rx_buff[8];                       // 数据缓冲区
}CAN_RxFrame_TypeDef;
/**
 * @brief 注册CAN实例并初始化其配置。
 * 该函数根据提供的配置信息注册一个新的CAN实例。如果成功，将返回指向新实例的指针；如果失败，则返回NULL，并通过日志记录错误原因。
 * @param can_config 指向CanInitConfig_s结构体的指针，包含初始化CAN所需的配置参数
 * @return 返回指向新创建的CAN实例的指针，或在发生错误时返回NULL
 */
CanInstance_s* Can_Register(CanInitConfig_s* can_config);
/**
 * @brief 通过CAN总线发送数据。
 * 该函数将指定的数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @param tx_buff 指向要发送的数据缓冲区的指针，数据长度应为8
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit_External_Tx_Buff(const CanInstance_s *instance, const uint8_t *tx_buff);
/**
 * @brief 通过CAN总线发送数据,为了避免大修MODULE而写的函数
 * 该函数将实例内部的发送缓冲区数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit(const CanInstance_s *instance);
#endif //USER_CAN_STANDARD
#endif //__BSP_CAN_H__