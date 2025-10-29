#ifndef __PID_H_
#define __PID_H_

#include "stdint.h"
#include <stdbool.h>

typedef struct{
    float kp;                        //比例系数
    float ki;                        //计数系数
    float kd;                        //微分系数
    float kf;                        //前馈系数
    float i_max;                     //积分限幅
    float out_max;                   //输出限幅
    float dead_zone;                 //死区
    float angle_max;                 //角度限幅
    float i_variable_min;            //变速积分下限,,设0时值与i_variable_min相同
    float i_variable_max;            //变速积分上限,设0时值与i_variable_min相同
    uint8_t d_first;                 //微分先行开关
    float target[2];                 //两次设定值
    float now[2];                    //两次实际值
    float err[2];                    //两次误差
    float p_out;                     //比例输出
    float i_out;                     //积分输出
    float d_out;                     //微分输出
    float f_out;                     //前馈输出
    float output;                    //输出值
}PidInstance_s;           

typedef struct{           
    float kp;                        //比例系数
    float ki;                        //计数系数
    float kd;                        //微分系数
    float kf;                        //前馈系数
    float angle_max;                 //角度最大值(限幅用，为0则不限幅)
    float i_max;                     //积分限幅(大于输出限幅时，默认大小与输出限幅相等)
    float out_max;                   //输出限幅
    float dead_zone;                 //死区
    float i_variable_min;            //变速积分下限,积分分离的值,为0时没有积分分离
    float i_variable_max;            //变速积分上限，为0时只有积分分离,小于min时值与i_variable_min相同
    uint8_t d_first;                 //微分先行开关(1开0关)
}PidInitConfig_s;

/**
 * @file alg_pid.h
 * @brief PID控制器注册函数
 * @details 该函数用于注册一个PID控制器实例，初始化相关参数。
 * @param config Pid初始化配置结构体指针
 * @return PidInstance_s指针，注册成功返回指针，失败返回NULL
 * @date 2025-07-01
 */
PidInstance_s *Pid_Register(PidInitConfig_s *config);

/**
 * @file alg_pid.h
 * @brief PID控制器计算函数
 * @details 该函数用于计算PID控制器的输出值。
 * @param PID PidInstance_s指针，PID控制器实例
 * @param target 目标值
 * @param now 当前值
 * @return 成功返回计算后的输出值，失败返回0.0f
 * @date 2025-07-01
 */
float Pid_Calculate(PidInstance_s *pid, float target, float now);

/**
 * @file alg_pid.h
 * @brief PID控制器清理函数
 * @details 该函数用于清理把Pid实例中的所有参数清零
 * @param pid PidInstance_s指针，PID控制器实例
 * @return 成功返回true，失败返回false
 * @note 该函数通常在电机失联后调用，以清除PID控制器的状态
 * @note 该函数在调用后不可恢复，只能通过重启程序解决
 * @date 2025-07-01
 */
bool Pid_Clean(PidInstance_s *pid);

/**
 * @file alg_pid.h
 * @brief PID控制器重置函数
 * @param pid PidInstance_s指针，PID控制器实例
 * @return 成功返回true，失败返回false
 * @date 2025-07-01
 */
bool Pid_Reset(PidInstance_s *pid);

#endif
