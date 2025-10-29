/**
*   @file dev_motor_unitree.h
*   @brief 
*   @author Wenxin HU
*   @date 25-7-13
*   @version 0.5
*   @note
*/
#ifndef DEV_MOTOR_UNITREE_H
#define DEV_MOTOR_UNITREE_H

#include <stdint.h>
#include <stdbool.h>
#include "bsp_uart.h"

//Unitree写的宏定义限幅函数
#define SATURATE(_IN, _MIN, _MAX) \
{                             \
if ((_IN) <= (_MIN))      \
(_IN) = (_MIN);       \
else if ((_IN) >= (_MAX)) \
(_IN) = (_MAX);       \
}

#define PI 3.14159265358979323846 // 圆周率要换位置
#define UNITREE_MOTOR_ID_MAX 15 //宇树电机ID最大值

typedef enum {
    GOM8010 = 0,
} UnitreeMotorType_e;

//目前宇树电机不打算封装成纯伺服，需要在电机外写控制代码
typedef enum {
    GOM8010_TORQUE = 0,      //纯力矩控制模式，只发送力矩数据
    GOM8010_COMBINATION = 1, //力位混控模式，发送宇树提供的五个控制参数
} UnitreeMotorControlMode_e;

typedef enum {
    UNITREE_NORMAL = 0,           // 正常
    UNITREE_OVER_TEMPERATURE = 1, // 过温
    UNITREE_OVER_CURRENT = 2,     // 过流
    UNITREE_OVER_VOLTAGE = 3,     // 过压
    UNITREE_CODING_ERROR = 4,     // 编码器错误
} UnitreeMotorErr_e;

/* 用于宇树电机通讯编解码 */
#pragma pack(1)
/// @breif 电机控制模式信息
typedef struct {
    uint8_t id : 4;      // 电机ID: 0,1...,13,14 15表示向所有电机广播数据(此时无返回)
    uint8_t status : 3;  // 工作模式: 0.锁定 1.FOC闭环 2.编码器校准 3.保留
    uint8_t reserve : 1; // 保留位
} RisMode_s;             // 控制模式 1Byte

///@brief 电机状态控制信息
typedef struct {
    int16_t tor_des; // 期望关节输出扭矩 unit: N.m      (q8)
    int16_t spd_des; // 期望关节输出速度 unit: rad/s    (q8)
    int32_t pos_des; // 期望关节输出位置 unit: rad      (q15)
    int16_t k_pos;   // 期望关节刚度系数 unit: -1.0-1.0 (q15)
    int16_t k_spd;   // 期望关节阻尼系数 unit: -1.0-1.0 (q15)
} RisCommand_s;      // 控制参数 12Byte

///@brief 电机状态反馈信息
typedef struct {
    int16_t torque;               // 实际关节输出扭矩 unit: N.m     (q8)
    int16_t speed;                // 实际关节输出速度 unit: rad/s   (q8)
    int32_t pos;                  // 实际关节输出位置 unit: rad     (q15)
    int8_t temp;                  // 电机温度: -128~127°C
    UnitreeMotorErr_e MError : 3; // 电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t force : 12;          // 足端气压传感器数据 12bit (0-4095)
    uint8_t none : 1;             // 保留位
} RisFeedback_s;                  // 状态数据 11Byte

///@brief 控制数据包格式
typedef struct {
    uint8_t head[2];  // 包头         2Byte
    RisMode_s mode;   // 电机控制模式  1Byte
    RisCommand_s cmd; // 电机期望数据 12Byte
    uint16_t CRC16;   // CRC          2Byte
} RIS_ControlData_t;  // 主机控制命令     17Byte

///@breif 电机反馈数据包格式
typedef struct {
    uint8_t head[2];   // 包头         2Byte
    RisMode_s mode;    // 电机控制模式  1Byte
    RisFeedback_s fbk; // 电机反馈数据 11Byte
    uint16_t CRC16;    // CRC          2Byte
} RIS_MotorData_s;     // 电机返回数据     16Byte
#pragma pack()

/// @brief 电机指令结构体
typedef struct {
    unsigned short mode; // 0:空闲 1:FOC控制 2:电机标定
    float T;             // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;             // 期望关节速度(电机本身的速度)(rad/s)
    float Pos;           // 期望关节位置(rad)
    float K_P;           // 关节刚度系数(0-25.599)
    float K_W;           // 关节速度系数(0-25.599)
} UnitreeMotorCmd_s;

/// @brief 电机反馈结构体
typedef struct {
    unsigned char motor_id; // 电机ID
    unsigned char mode;     // 0:空闲 1:FOC控制 2:电机标定
    int Temp;               // 温度
    int MError;             // 错误码
    float T;                // 当前实际电机输出力矩(电机本身的力矩)(Nm)
    float W;                // 当前实际电机速度(电机本身的速度)(rad/s)
    float Pos;              // 当前电机位置(rad)
    int correct;            // 接收数据是否完整(1完整，0不完整)
    int footForce;          // 足端力传感器原始数值

    uint16_t calc_crc;
    uint32_t timeout;                // 通讯超时 数量
    uint32_t bad_msg;                // CRC校验错误 数量
    RIS_MotorData_s motor_recv_data; // 电机接收数据结构体
} UnitreeMotorData_s;

/* 用于宇树电机通讯编解码 */

typedef struct {
    uint8_t id;                      //电机id 1-14（15代表广播数据包，暂时不使用）
    UnitreeMotorType_e type;         //电机类型
    UnitreeMotorType_e control_mode; //电机控制模式

    UART_HandleTypeDef* uart_handle; //电机UART句柄
    float reduction_ratio;           //减速比
} UnitreeMotorInitConfig_s;

typedef struct {
    uint8_t id;                             //电机Id 1-14（15代表广播数据包，暂时不使用）
    UnitreeMotorType_e type;                //电机类型
    UartInstance_s* uart_instance;          //电机UART实例
    float reduction_ratio;                  //减速比
    UnitreeMotorControlMode_e control_mode; //电机控制模式

    float out_position;      //电机输出轴角度(-PI~PI)
    float out_velocity;      //电机输出轴速度(rpm)
    float out_torque;
    float position;          //电机转子位置(多圈累加)(rad)
    float rotor_velocity;    //电机转子速度(rad/s)
    float rotor_torque;      //电机转子力矩(N.m)
    float motor_temperature; //电机温度(°C)
    UnitreeMotorErr_e M_err; //电机错误标识: 0.正常 1.过热 2.过流 3.过压 4.编码器故障 5-7.保留
    uint16_t calc_crc;
    uint32_t timeout; //通讯超时 数量
    uint32_t bad_msg; //CRC校验错误 数量

    float T;   // 期望关节的输出力矩(电机本身的力矩)(Nm)
    float W;   // 期望关节速度(电机本身的速度)(rad/s)
    float Pos; // 期望关节位置(rad)
    float K_P; // 关节刚度系数(0-25.599)
    float K_W; // 关节速度系数(0-25.599)
} UnitreeMotorInstance_s;

/**
 * @brief Unitree电机注册函数
 * @param config 宇树电机注册结构体
 * @return 成功：UnitreeMotorInstance_s*，电机实例指针 失败：NULL
 */
UnitreeMotorInstance_s* Motor_Unitree_Register(UnitreeMotorInitConfig_s* config);
/**
 * @brief Unitree电机控制函数
 * @param motor UnitreeMotorInstance_s* 电机实例指针
 * @param T 期望关节的输出力矩(电机本身的力矩)(Nm)
 * @param W 期望关节速度(电机本身的速度)(rad/s)
 * @param Pos 期望关节位置(rad)
 * @param K_P 关节刚度系数(0-25.599)
 * @param K_W 关节速度系数(0-25.599)
 * @return 成功：true 失败：false
 */
bool Motor_Unitree_Control(UnitreeMotorInstance_s* motor,
                           float T, float W, float Pos, float K_P, float K_W);
///@brief Unitree电机刹车函数(将电机控制模式改为0，锁定电机）
bool Motor_Unitree_Stop(UnitreeMotorInstance_s* motor);

/**
 * @brief Unitree电机原始控制函数
 * @param motor UnitreeMotorInstance_s* 电机实例指针
 * @param cmd MotorCmd_s 电机控制指令
 * @return 成功：true 失败：false
 */
bool Motor_Unitree_Control_Raw(UnitreeMotorInstance_s* motor, UnitreeMotorCmd_s* cmd);

#endif //DEV_MOTOR_UNITREE_H
