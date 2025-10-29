#ifndef CHASSIS__CALC_H
#define CHASSIS__CALC_H
// #include "robot_config.h"
#include "dev_motor_dji.h"
#include "alg_pid.h"
#define  CHASSIS
/**
 * @brief 底盘类型枚举
 */
typedef enum {    
    Omni_Wheel = 0,     ///< 全向轮
    Mecanum_Wheel = 1,  ///< 麦克纳姆轮  
    Steering_Wheel = 2, ///< 舵轮
}CarType;

/**
 * @brief 底盘工作模式枚举
 */
typedef enum {
    CHASSIS_NORMAL = 0,           ///< 底盘独立行走模式
    CHASSIS_FOLLOW_GIMBAL = 1,    ///< 底盘跟随云台行走模式
    CHASSIS_GYROSCOPE = 2,        ///< 小陀螺模式
    CHASSIS_SLOW = 3,           ///< 静止/低速模式
    //CHASSIS_SZUPUP = 5,         ///< 爬坡模式(保留)
    //CHASSIS_FLY=6,             ///< 飞坡模式(保留)
    //CHASSIS_GYROSCOPE_BIANSU=7   ///< 变速小陀螺模式
} ChassisAction;

/**
 * @brief 底盘速度结构体
 */
typedef struct {
    float Vx;                     ///< 底盘X轴速度(m/s)
    float Vy;                     ///< 底盘Y轴速度(m/s)
    float Vw;                     ///< 底盘旋转角速度(rad/s)
}Chassis_Speed;

typedef struct{
    float wheel_radius;                     ///< 轮子半径(m)
    float chassis_radius;                   ///< 底盘旋转半径(m)
}Chassis_Omni_Message_s;

typedef struct{
    float wheel_radius;                     ///< 轮子半径(m)
    float length_a;                         ///< 底盘前后半长度(m)
    float length_b;                         ///< 底盘左右半长度(m) 
}Chassis_Mecanum_Steering_Message_s;

/**
 * @brief 底盘初始化配置结构体
 */
typedef struct {
    CarType type;                      ///< 底盘类型
    DjiMotorInitConfig_s motor_config[8];      ///< 电机初始化配置,配置信息为id=1的电机信息          
    PidInitConfig_s gimbal_follow_pid_config; ///< 云台跟随PID配置
    Chassis_Omni_Message_s omni_message; ///< 全向轮/舵轮参数
    Chassis_Mecanum_Steering_Message_s mecanum_steering_message; ///< 麦克纳姆轮
    float gimbal_yaw_zero;                  ///< 云台偏航零点角度
}ChassisInitConfig_s;

/**
 * @brief 底盘实例结构体
 */
typedef struct {
    CarType type;                      ///< 底盘类型
    Chassis_Omni_Message_s omni_message; ///< 全向轮参数
    Chassis_Mecanum_Steering_Message_s mecanum_steering_message; ///< 麦克纳姆轮/舵轮
    Chassis_Speed Chassis_speed,absolute_chassis_speed; ///< 底盘速度
    PidInstance_s *gimbal_follow_pid;       ///< 云台跟随PID实例指针
    DjiMotorInstance_s *chassis_motor[8];   ///< 底盘电机实例数组(1-4为驱动电机,5-8为转向电机)
    float out_speed[4];                     ///< 电机输出速度数组(rpm)
    float out_angle[4];                     ///< 电机输出角度数组(rad)
	float gimbal_yaw_zero;                  ///< 云台偏航零点角度
    float gimbal_yaw_angle;
}ChassisInstance_s;

/**
 * @brief 注册并初始化底盘实例
 * @param Chassis_config 底盘初始化配置结构体指针
 * @return 成功返回底盘实例指针，失败返回NULL
 * @note 需要先初始化CAN总线
 * @date 2025-07-03
 */
ChassisInstance_s *Chassis_Register(ChassisInitConfig_s *Chassis_config);

/**
 * @brief 底盘运动控制函数
 * @param chassis 底盘实例指针
 * @return 控制成功返回true，失败返回false
 * @note 会根据当前模式进行相应控制
 * @date 2025-07-09
 */
bool Chassis_Control(ChassisInstance_s *Chassis);

/**
 * @brief 底盘计算函数，根据传入底盘实例的Chassis_speed计算实际电机速度
 * @param Chassis 底盘实例指针
 * @return 计算是否成功
 */
bool Chassis_Calc(ChassisInstance_s *Chassis);

/**
 * @brief 底盘模式选择函数
 * @param ChassisAction 底盘实例指针
 * @note 根据当前模式调整实际控制参数
 * @date 2025-07-09
 */
void Chassis_Mode_Choose(ChassisInstance_s* Chassis,ChassisAction target_mode);
#endif