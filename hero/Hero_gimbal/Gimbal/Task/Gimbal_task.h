#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__  

#include "Gimbal_Fun.h"
#include "minipc.h"
extern Dr16Instance_s *rc;
extern DmMotorInstance_s  *gimbal_pitch_motor;
extern DmMotorInitConfig_s  *gimbal_pitch_motor_config;

extern DmMotorInstance_s  *yaw_motor;
extern DmMotorInitConfig_s  *yaw_motor_config;

extern DmMotorInstance_s  *rammer_motor;
extern DmMotorInitConfig_s  *rammer_motor_config;

extern DjiMotorInstance_s *shoot_motor1;
extern DjiMotorInstance_s *shoot_motor2;

extern CanInstance_s *board_data_exchange;

extern Gimbal_data gimbal_data;

extern MiniPC_Instance* miniPC_ins;

extern float imu_angle_rad[2];

/*************************遥控器***************************
 *                                                            *
 *   -----------------------------------------------------    *
 *   |     (上-1)                               (上-1)   |    *
 *   |SW_L|(中-3)                          SW_R|(中-3)   |    *
 *   |     (下-2)                               (下-2)   |    *
 *   |                                                   |    *
 *   |    | ^ |                                | ^ |     |    *
 *   |    | 3 |左摇杆                     右摇杆| 1 |     |    *
 *   | ---     ---                          ---     ---  |    *
 *   |<           2>                       <           0>|    *
 *   | ---     ---                          ---     ---  |    *
 *   |    |   |                                |   |     |    *
 *   |    |   |                                |   |     |    *
 *   |                                                   |    *
 *   -----------------------------------------------------    *
 *                                                            *
 * 
 * ***********************************************************/
/***********************模式选择*******************************
 *
 * ******s[1]**********s[0]***********************************
 *        1              1    对整车云台部分和底盘部分失能
 *        1              3    遥控器使能，键盘失能
 *        1              2    键盘使能，键盘失能
 
 *        3              1    普通模式：右摇杆控制p轴yaw轴，左摇杆无作用
 *        3              2    小陀螺模式，摩擦轮启动，拨轮上拨退弹，下拨上弹
 *        3              3    自瞄模式：p轴yaw轴数据来源小电脑
 * 
 * 
 *        2              1			普通行进模式，左摇杆控制行进，右摇杆控制p和yaw
 *        2              3      地盘跟随模式
 *        2              2     小陀螺
 * ***********************************************************/

#endif 
