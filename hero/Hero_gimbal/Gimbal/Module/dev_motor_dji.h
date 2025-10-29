/** 
 *  @file dev_motor_dji.h
 *  @brief 大疆电机控制模块
 *  @details 大疆电机控制模块，提供大疆电机初始化，控制等功能
 *  @date 2025-07-04
 */
#ifndef __DEV_MOTOR_DJI_H__
#define __DEV_MOTOR_DJI_H__

#include "robot_config.h"
#ifdef USER_CAN_STANDARD
#include "bsp_can.h"
#else
#include "bsp_fdcan.h"
#endif
#include "alg_pid.h"
#include <stdbool.h>

#define PI 3.14159265358979323846 // 圆周率要换位置
#define DJI_MOTOR_MAX_CNT 12 // DJI电机最大数量
#define DJI_ECD_ANGLE_COEF 0.00076699 //编码器值转换为弧度系数
#define DJI_ECD_ANGLE_MAX  8192       
#define FLOAT_MAX 16777210.0f

typedef enum{
    DJI_POSITION = 0,     // 位置控制模式
    DJI_VELOCITY = 1,     // 速度控制模式
}DjiMotorControlMode_e;

typedef enum{
    GM6020 = 0,
    M3508 = 1,
    M2006 = 2,
}DjiMotorType_e;

typedef struct{
    float out_position;                 // 电机输出轴角度(-PI~PI)
    float out_velocity;                 // 电机输出轴速度(rpm)
    int16_t rotor_position;             // 电机转子角度编码器值
    int16_t rotor_last_position;        // 电机转子上次角度编码器值
    float rotor_velocity;               // 电机转子速度(rpm)
    int16_t total_angle;                // 电机转子累计旋转总角度
    int16_t torque_current;             // 扭计电流(-20A~20A映射到-16384~16384)
    float temperature;                  // 电机温度(°C)
}DjiMotorMessage_s;


typedef struct{
    char* topic_name;
    DjiMotorType_e type;                  // 电机类型
    DjiMotorControlMode_e control_mode;   // 电机控制模式
    uint8_t id;                           // 电机ID(0~8)

    CanInitConfig_s can_config;           // 电机CAN配置
    uint8_t reduction_ratio;              // 减速比

    PidInitConfig_s angle_pid_config;     // 角度控制PID配置
    PidInitConfig_s velocity_pid_config;  // 速度控制PID配置
}DjiMotorInitConfig_s;

typedef struct{
    char* topic_name;                   // 实例名称
    DjiMotorType_e type;                // 电机类型
    uint8_t id;                         // 电机ID(0~8)
    DjiMotorControlMode_e control_mode; // 电机控制模式
    CanInstance_s *can_instance;        // 电机CAN实例指针

    PidInstance_s *angle_pid;           // 角度控制PID实例指针
    PidInstance_s *velocity_pid;        // 速度控制PID实例指针

    DjiMotorMessage_s message;          // 电机状态信息
    uint8_t reduction_ratio;            // 电机减速比

    float target_position;              // 电机目标角度(-PI~PI)
    float target_velocity;              // 电机目标速度(rpm)

    float output;                       // 电机输出值(电流或电压)
}DjiMotorInstance_s;

/**
 * @brief 注册Dji电机实例
 * @param config DJI电机初始化配置结构体指针
 * @return 成功返回DjiMotorInstance_s指针，失败返回NULL
 * @note 调用前需要确认CAN初始化成功
 * @date 2025-07-03
 */
DjiMotorInstance_s *Motor_Dji_Register(DjiMotorInitConfig_s *config);

/**
 * @brief Dji电机控制函数
 * @details 该函数会根据电机的工作模式自动控制
 * @param motor 电机实例指针
 * @param taget 控制量目标值
 * @return 成功返回true，失败返回false
 * @note 如果电机是速度模式，则target为目标转速 
 * @note 如果电机是位置模式，则targrt为目标角度
 * @date 2025-07-03
 */
bool Motor_Dji_Control(DjiMotorInstance_s *motor, float taget);

/**
 * @brief Dji发送数据函数
 * @param motor 电机实例指针
 * @return 如果成功发送数据返回true，失败返回false 
 * @note 该函数会自动把与该电机发送id一致的电机数据拉取并发送
 * @date 2025-07-03
 */
bool Motor_Dji_Transmit(DjiMotorInstance_s *motor);

/**
 * @brief Dji电机切换工作模式
 * @param motor 电机实例指针
 * @param target_mode 目标工作模式
 * @return 如果成功切换模式返回true，失败返回false
 * @date 2025-07-03
 */
bool Motor_Dji_Change_Mode(DjiMotorInstance_s *motor, DjiMotorControlMode_e target_mode);

/**
 * @brief Dji电机直接设置电流
 * @param motor 电机实例指针
 * @param set_current 目标电流值
 * @return 如果成功返回true，失败返回false
 * @date 2025-08-18
 * @note 该函数是为了给不想用内置pid的用户提供的直接设置电流的接口
 *       主要这里的set_current是映射后的值,单位不是安培(A),映射关系请自行查找手册
 */
bool Motor_Dji_SetCurrent(DjiMotorInstance_s *motor, float set_current);

#endif