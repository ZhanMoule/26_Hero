
/*注:打开虚拟串口:
1.在connectivity中选择USB_OTG_FS
2.在Mode中选择Device_Only
3.在Middleware and Software中选择USB_DEVICE
4.在Class For FS IP中选择第三个(virtual port com虚拟串口)

6.Generate Code即可
*/

#ifndef DEV_MINIPC_H
#define DEV_MINIPC_H
#include <stdint.h>
#include "Com_System.h" //包含消息中心的头文件

// 设置结构体按1字节对齐
#pragma pack(1)
//如果定义哨兵模式的话，大部分代码才会启用
#define SENTRY_MODE
/**
 * @brief USB消息包初始化宏
 * @param struct_ptr 指向要初始化的结构体的指针
 * @param msg_type 消息类型（USB_MessageType_e枚举值）
 * @note 该宏会：
 *       1. 清空整个结构体（32字节）
 *       2. 设置包头为 's'
 *       3. 设置消息类型
 *       4. 设置包尾为 'e'
 */
#define USB_INIT_MESSAGE_PACKET(struct_ptr, msg_type) \
    do { \
        if ((struct_ptr) != NULL) { \
            /* 清空整个结构体 */ \
            memset((struct_ptr), 0, 32); \
            /* 设置包头 */ \
            ((uint8_t*)(struct_ptr))[0] = 's'; \
            /* 设置消息类型 */ \
            ((uint8_t*)(struct_ptr))[1] = (uint8_t)(msg_type); \
            /* 设置包尾 */ \
            ((uint8_t*)(struct_ptr))[31] = 'e'; \
        } \
    } while(0)

/**
 * @brief USB发送消息包初始化宏（专用于send_union）
 * @param union_ptr 指向send_union的指针
 * @param msg_type 消息类型（USB_MessageType_e枚举值）
 */
#define USB_INIT_SEND_PACKET(union_ptr, msg_type) \
    USB_INIT_MESSAGE_PACKET(union_ptr, msg_type)

    
// 消息类型枚举定义
typedef enum {
    // 接收消息类型
    USB_MSG_AIM_RX = 0xA0,          // 自瞄数据接收
    #ifdef SENTRY_MODE
    USB_MSG_CHASSIS_RX = 0xA1,      // 底盘数据接收  
    USB_MSG_GAME_RX = 0xA2,         // 比赛信息接收
    USB_MSG_MODULE_RX = 0xA3,       // 模块信息接收
    #endif
    
    // 发送消息类型
    USB_MSG_AIM_TX = 0xB0,          // 自瞄云台反馈
    #ifdef SENTRY_MODE
    USB_MSG_FRIEND1_TX = 0xB1,      // 我方机器人位置1
    USB_MSG_FRIEND2_TX = 0xB2,      // 我方机器人位置2
    USB_MSG_ENEMY1_TX = 0xB3,       // 敌方机器人位置1
    USB_MSG_ENEMY2_TX = 0xB4,       // 敌方机器人位置2
    USB_MSG_RED_HP_TX = 0xB5,       // 红方血量
    USB_MSG_BLUE_HP_TX = 0xB6,      // 蓝方血量
    USB_MSG_BUILD_HP_TX = 0xB7,     // 建筑血量
    USB_MSG_GAME_INFO_TX = 0xB8,    // 比赛信息
    USB_MSG_OP_FB_TX = 0xB9,        // 操作反馈
    USB_MSG_HIT_FB_TX = 0xBA,       // 受击反馈
    USB_MSG_LAUNCH_TX = 0xBB,       // 发射状态
    #endif
    // 特殊值
    USB_MSG_UNKNOWN = 0xFF,          // 未知消息类型
    DIY_MODE = 0xFE                   // DIY模式
} USB_MessageType_e;




// 定义接收数据包结构（32字节固定长度）
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xA0
    uint8_t find_bool;         // 2 是否追踪
    float yaw;                 // 3 - 6 偏航角
    float pitch;               // 7 - 10 俯仰角
    uint8_t reserved[20];      // 11 - 30 预留空位（填充0）
    char end;                  // 31 帧尾，取 'e'
} Computer_Rx_Message_t;

//底盘通信（上发下）
typedef struct {
    char start;
    char datatype;             // 1 消息类型 0xA1
    float x_speed;             // 2 - 5 x方向速度
    float y_speed;             // 6 - 9 y方向速度
    float yaw;                 // 10 - 13 yaw
    uint8_t reserved[17];      // 14 - 30 预留空位（填充0）
    char end;                  // 31 帧尾，取 'e'
} Chassis_package;
#ifdef SENTRY_MODE
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xA2
    uint8_t type;              // 2 类型  0：无  1：复活  2：买弹丸
    uint8_t content;           // 3 具体内容
    uint8_t reserved[26];      // 4 - 30 预留空位（填充0）
    char end;                  // 31 帧尾，取 'e'
} competition_package;

typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xA3
    uint16_t type;             // 2 类型  0：无  1：小陀螺  2：云台单连发控制
    uint16_t content;          // 4 具体内容
    uint8_t reserved[24];      // 6 - 30 预留空位（填充0）
    char end;                  // 31 帧尾，取 'e'
} modules_package;



// 0xB1 我方机器人位置信息话题1
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB1
    float infantry_3_x;        // 2 - 5 己方 3 号步兵机器人位置 x 轴坐标
    float infantry_3_y;        // 6 - 9 己方 3 号步兵机器人位置 y 轴坐标
    float infantry_4_x;        // 10 - 13 己方 4 号步兵机器人位置 x 轴坐标
    float infantry_4_y;        // 14 - 17 己方 4 号步兵机器人位置 y 轴坐标
    float infantry_5_x;        // 18 - 21 己方 5 号步兵机器人位置 x 轴坐标
    float infantry_5_y;        // 22 - 25 己方 5 号步兵机器人位置 y 轴坐标
    uint8_t reserved[4];       // 26 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} friendly_position_1_package;

// 0xB2 我方机器人位置信息话题2
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB2
    float hero_x;              // 2 - 5 己方 1 号英雄机器人位置 x 轴坐标
    float hero_y;              // 6 - 9 己方 1 号英雄机器人位置 y 轴坐标
    float engineer_x;          // 10 - 13 己方 2 号工程机器人位置 x 轴坐标
    float engineer_y;          // 14 - 17 己方 2 号工程机器人位置 y 轴坐标
    float sentinal_x;          // 18 - 21 己方 7 号哨兵机器人位置 x 轴坐标
    float sentinal_y;          // 22 - 25 己方 7 号哨兵机器人位置 y 轴坐标
    uint8_t reserved[4];       // 26 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} friendly_position_2_package;

// 0xB3 敌方机器人位置1
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB3
    float enemy_infantry_3_x;  // 2 - 5 敌方 3 号步兵机器人位置 x 轴坐标
    float enemy_infantry_3_y;  // 6 - 9 敌方 3 号步兵机器人位置 y 轴坐标
    float enemy_infantry_4_x;  // 10 - 13 敌方 4 号步兵机器人位置 x 轴坐标
    float enemy_infantry_4_y;  // 14 - 17 敌方 4 号步兵机器人位置 y 轴坐标
    float enemy_infantry_5_x;  // 18 - 21 敌方 5 号步兵机器人位置 x 轴坐标
    float enemy_infantry_5_y;  // 22 - 25 敌方 5 号步兵机器人位置 y 轴坐标
    uint8_t reserved[4];       // 26 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} enemy_position_1_package;

// 0xB4 敌方机器人位置2
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB4
    float enemy_infantry_1_x;  // 2 - 5 敌方 1 号英雄机器人位置 x 轴坐标
    float enemy_infantry_1_y;  // 6 - 9 敌方 1 号英雄机器人位置 y 轴坐标
    float enemy_infantry_2_x;  // 10 - 13 敌方 2 号工程机器人位置 x 轴坐标
    float enemy_infantry_2_y;  // 14 - 17 敌方 2 号工程机器人位置 y 轴坐标
    float enemy_infantry_7_x;  // 18 - 21 敌方 7 号哨兵机器人位置 x 轴坐标
    float enemy_infantry_7_y;  // 22 - 25 敌方 7 号哨兵机器人位置 y 轴坐标
    uint8_t reserved[4];       // 26 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} enemy_position_2_package;

// 0xB5 红方血量
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB5
    uint16_t red_1_robot_HP;   // 2 - 3 红 1 英雄机器人血量
    uint16_t red_2_robot_HP;   // 4 - 5 红 2 工程机器人血量
    uint16_t red_3_robot_HP;   // 6 - 7 红 3 步兵机器人血量
    uint16_t red_4_robot_HP;   // 8 - 9 红 4 步兵机器人血量
    uint16_t red_5_robot_HP;   // 10 - 11 红 5 步兵机器人血量
    uint16_t red_7_robot_HP;   // 12 - 13 红 7 哨兵机器人血量
    uint8_t reserved[16];      // 14 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} red_hp_package;

// 0xB6 蓝方血量
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB6
    uint16_t blue_1_robot_HP;  // 2 - 3 蓝 1 英雄机器人血量
    uint16_t blue_2_robot_HP;  // 4 - 5 蓝 2 工程机器人血量
    uint16_t blue_3_robot_HP;  // 6 - 7 蓝 3 步兵机器人血量
    uint16_t blue_4_robot_HP;  // 8 - 9 蓝 4 步兵机器人血量
    uint16_t blue_5_robot_HP;  // 10 - 11 蓝 5 步兵机器人血量
    uint16_t blue_7_robot_HP;  // 12 - 13 蓝 7 哨兵机器人血量
    uint8_t reserved[16];      // 14 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} blue_hp_package;

// 0xB7 建筑血量
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB7
    uint16_t red_outpost_HP;   // 2 - 3  红方前哨站血量
    uint16_t red_base_HP;      // 4 - 5  红方基地血量
    uint16_t blue_outpost_HP;  // 6 - 7 蓝方前哨站血量
    uint16_t blue_base_HP;     // 8 - 9 蓝方基地血量
    uint8_t reserved[20];      // 10 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} building_hp_package;

// 0xB8 比赛信息
typedef struct {
    char start;                   // 0 帧头，取 's'
    char datatype;                // 1 消息类型 0xB8
    uint8_t enemy_team_color;     // 2 敌方颜色 0：红 1：蓝
    uint8_t game_progress;        // 3 当前比赛阶段
    uint16_t stage_remain_time;   // 4 - 5 当前阶段剩余时间
    uint16_t remaining_gold_coin; // 6 - 7 剩余金币数量
    uint8_t reserved[22];         // 8 - 30 预留空位
    char end;                     // 31 帧尾，取 'e'
} game_info_package;

// 0xB9 操作反馈
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB9
    float target_position_x;   // 2 - 5 目标位置 x 轴坐标
    float target_position_y;   // 6 - 9 目标位置 y 轴坐标
    uint8_t cmd_keyboard;      // 10 键盘信息
    uint8_t target_robot_id;   // 11 目标机器人id
    uint8_t reserved[18];      // 12 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} operation_feedback_package;

// 0xBA 受击反馈
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xBA
    uint8_t hitted;            // 2 受击打标识
    uint8_t reserved[27];      // 3 - 30 预留空位
    char end;                  // 31 帧尾，取 'e'
} hit_feedback_package;

// 0xBB 发射状态量
typedef struct {
    char start;                         // 0 帧头，取 's'
    char datatype;                      // 1 消息类型 0xBB
    uint16_t projectile_allowance_17mm; // 2 - 3 17mm允许发弹量
    uint16_t projectile_allowance_42mm; // 4 - 5 42mm允许发弹量
    uint8_t real_heat;                  // 6 发射机构热量 
    uint8_t launching_frequency;        // 7 弹丸射速(单位:Hz)
    uint8_t reserved[22];               // 8 - 30 预留空位
    char end;                           // 31 帧尾，取 'e'
} launch_status_package;
#endif
typedef union {
    Computer_Rx_Message_t norm_aim_pack;        // 自瞄
    #ifdef SENTRY_MODE
    Chassis_package ch_pack;                    // 底盘

    competition_package com_pack;               // 比赛信息
    modules_package mod_pack;                   // 模块信息
    #endif
    uint8_t raw_data[32];                       // 用于访问整体的代码
} message_union;

// 数据源指针结构体

typedef struct {
    float* high_gimbal_yaw;
    float* pitch;
    uint8_t* enemy_team_color;
    uint8_t* mode;
    uint8_t* rune_flag;
    float* low_gimbal_yaw;
} USB_AimTx_DataSource_t;
#ifdef SENTRY_MODE
typedef struct {
    float* infantry_3_x;
    float* infantry_3_y;
    float* infantry_4_x;
    float* infantry_4_y;
    float* infantry_5_x;
    float* infantry_5_y;
} USB_FriendPos1_DataSource_t;

typedef struct {
    uint16_t* red_1_robot_HP;
    uint16_t* red_2_robot_HP;
    uint16_t* red_3_robot_HP;
    uint16_t* red_4_robot_HP;
    uint16_t* red_5_robot_HP;
    uint16_t* red_7_robot_HP;
} USB_RedHP_DataSource_t;

typedef struct {
    uint8_t* enemy_team_color;
    uint8_t* game_progress;
    uint16_t* stage_remain_time;
    uint16_t* remaining_gold_coin;
} USB_GameInfo_DataSource_t;
#endif
// 统一的数据源联合体
typedef union {
    USB_AimTx_DataSource_t aim_tx;
    #ifdef SENTRY_MODE
    USB_FriendPos1_DataSource_t friend_pos_1;
    USB_RedHP_DataSource_t red_hp;
    USB_GameInfo_DataSource_t game_info;
    #endif
} USB_DataSource_Union_t;

// 定义发送数据包结构（32字节固定长度）
typedef struct {
    char start;                // 0 帧头，取 's'
    char datatype;             // 1 消息类型 0xB0
    float high_gimbal_yaw;     // 2 - 5 小云台偏航角
    float pitch;               // 6 - 9 俯仰角
    uint8_t enemy_team_color;  // 10 敌方颜色 0：红 1：蓝
    uint8_t mode;              // 11 模式 0：自瞄 1：符
    uint8_t rune_flag;         // 12 符模式 '0':不可激活 '1':小符 '2':大符
    float low_gimbal_yaw;      // 13 - 16 大云台偏航角
    uint8_t reserved[14];      // 17 - 30 预留空位（填充0）
    char end;                  // 31 帧尾，取 'e'
} Computer_Tx_Message_t;

typedef union {
    Computer_Tx_Message_t self_aim_pack;        // 自瞄
    #ifdef SENTRY_MODE
    friendly_position_1_package friend_pos_1;   // 我方位置1
    friendly_position_2_package friend_pos_2;   // 我方位置2
    enemy_position_1_package enemy_pos_1;       // 敌方位置1
    enemy_position_2_package enemy_pos_2;       // 敌方位置2
    red_hp_package red_hp;                      // 红方血量
    blue_hp_package blue_hp;                    // 蓝方血量
    building_hp_package building_hp;            // 建筑血量
    game_info_package game_info;                // 比赛信息
    operation_feedback_package op_feedback;     // 操作反馈
    hit_feedback_package hit_feedback;          // 受击反馈
    launch_status_package launch_status;        // 发射状态
    #endif
    uint8_t raw_data[32];                       // 用于访问整体的代码
} send_union;
typedef void (*CallbackFunc)(send_union* send_buffer);
// 修改配置结构体，使用枚举类型
typedef struct
{
    message_union message_config;               // 发送数据的来源配置
    USB_MessageType_e message_type;             // 需要的数据类型
    USB_MessageType_e Send_message_type; // 发送数据类型
    CallbackFunc callback;                      // 发送用数据更新回调函数
} MiniPC_Config;
//USB通信实例
//发送时，使用Update函数调用回调函数，向message中填入发送数据然后调用发送函数
//不发送时，调用接收函数,message中会填入接受数据
typedef struct
{
    uint8_t id;                                 // USB通信实例ID
    USB_MessageType_e message_type;             // 消息类型
    CallbackFunc callback;
    message_union message;                      // 32位数据包
    uint8_t activate_flag;                      // 激活标志
    USB_DataSource_Union_t data_source;         // 数据源指针
    USB_MessageType_e Send_Message_Type;    // 发送消息类型
} MiniPC_Instance;


// 简化的配置函数声明
void Minipc_ConfigAimTx(MiniPC_Instance* instance, float* high_yaw, float* pitch, uint8_t* enemy_color, uint8_t* mode, uint8_t* rune_flag, float* low_yaw);

#ifdef SENTRY_MODE
void Minipc_ConfigFriendPos1Tx(MiniPC_Instance* instance, float* inf3_x, float* inf3_y, float* inf4_x, float* inf4_y, float* inf5_x, float* inf5_y);
void Minipc_ConfigRedHPTx(MiniPC_Instance* instance, uint16_t* red1_hp, uint16_t* red2_hp, uint16_t* red3_hp, uint16_t* red4_hp, uint16_t* red5_hp, uint16_t* red7_hp);
void Minipc_ConfigGameInfoTx(MiniPC_Instance* instance, uint8_t* enemy_color, uint8_t* game_progress, uint16_t* remain_time, uint16_t* gold_coin);
#endif
// 统一的更新和发送函数
void Minipc_UpdateAllInstances(void);
//用于单独更新的函数
void Minipc_UpdateInstanceData(MiniPC_Instance* instance);



MiniPC_Instance* Minipc_Register(MiniPC_Config* config);
void USB_Data_Received_Callback(uint8_t* buf, uint32_t len);
void Data_Processing(void);



// 恢复默认对齐方式
#pragma pack()

#endif //DEV_MINIPC_H



