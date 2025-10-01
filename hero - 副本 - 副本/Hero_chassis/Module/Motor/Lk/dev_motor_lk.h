#ifndef __DEV_MOTOR_LK_H__
#define __DEV_MOTOR_LK_H__

#include "bsp_can.h"
#include "alg_pid.h"

#define LK_MOTOR_MAX_CNT 4 // LK电机最大数量

typedef enum{
    MF9025 = 0,
}LkMotorType_e;

typedef enum{
    LK_POSITION = 0,     // 位置控制模式
    LK_VELOCITY = 1,     // 速度控制模式
}LkMotorControlMode_e;

typedef enum{
    LOW_VOLTAGE         = 0x01,     // 低电压错误
    HIGH_VOLTAGE        = 0x02,     // 高电压错误
    DRIVE_TEMPERATURE   = 0x04,     // 驱动器温度错误
    MOTOR_TEMPERATURE   = 0x08,     // 电机温度错误
    MOTOR_OVERCURRENT   = 0x10,     // 电机过电流错误
    MOTOR_SHORT_CIRCUIT = 0x20,     // 电机短路错误
    MOTOR_STALL         = 0x40,     // 电机停转错误
    INPUT_ERROR         = 0x80,     // 输入错误
}LkMotorErr_e;

typedef struct{
    uint8_t id;                           // 电机ID(1~4)
    LkMotorType_e type;                   // 电机类型
    LkMotorControlMode_e control_mode;    // 电机控制模式

    CanInitConfig_s can_config;           // 电机CAN配置
    uint8_t reduction_ratio;              // 减速比

    PidInitConfig_s angle_pid_config;     // 角度控制PID配置
    PidInitConfig_s velocity_pid_config;  // 速度控制PID配置
}LkMotorInitConfig_s;

typedef struct{
    uint8_t id;                             // 电机ID(1~4)
    LkMotorType_e type;                     // 电机类型
    CanInstance_s *can_instance;            // 电机CAN实例
    uint8_t reduction_ratio;                // 减速比
    LkMotorControlMode_e control_mode;      // 电机控制模式

    PidInstance_s *angle_pid;               // 角度控制PID
    PidInstance_s *velocity_pid;            // 速度控制PID
    
    float out_position;                     // 电机输出轴角度(-PI~PI)
    float out_velocity;                     // 电机输出轴速度(rpm)
    int16_t rotor_position;                 // 电机转子角度编码器值
    int16_t rotor_last_position;            // 电机转子上次角度编码器值
    float rotor_velocity;                   // 电机转子速度(rpm)
    int16_t total_angle;                    // 电机转子累计旋转总角度
    int16_t torque_current;                 // 扭计电流(-33A~33A映射到-2048~2048)
    uint8_t motor_temperature;              // 电机温度(°C)
    
    float target_position;                  // 电机目标角度(-PI~PI)
    float target_velocity;                  // 电机目标速度(rpm)

    float output;                           // 电机设定值(电流或电压)
}LkMotorInstance_s;

LkMotorInstance_s *Motor_Lk_Register(LkMotorInitConfig_s *config);
bool Motor_Lk_Control(LkMotorInstance_s *motor, float target);
bool Motor_LK_Transmit(LkMotorInstance_s *motor);

#endif