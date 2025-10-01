#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__  

#include "Gimbal_Fun.h"
extern RC_instance *rc;
extern DmMotorInstance_s  *gimbal_pitch_motor;
extern DmMotorInitConfig_s  *gimbal_pitch_motor_config;

extern DmMotorInstance_s  *yaw_motor;
extern DmMotorInitConfig_s  *yaw_motor_config;

extern DmMotorInstance_s  *rammer_motor;
extern DmMotorInitConfig_s  *rammer_motor_config;

extern DjiMotorInstance_s *shoot_motor1;
extern DjiMotorInstance_s *shoot_motor2;

extern CanInstance_s *board_data_exchange;

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
 *        3              1    普通模式：云台yaw 轴和pitch轴角度均为0保持不动，左摇杆控制底盘方向，右摇杆控制p轴yaw轴
 *        3              2    小陀螺模式，保持云台由遥控器控制，底盘自旋
 *        3              3    云台跟随模式：左摇杆控制yaw轴转向以及车的前进，右摇杆控制pitch轴
 * 
 * 2：打开摩擦轮，右滚轮控制拨弹轮
 *        2              1
 *        2              3      单发模式，左摇杆控制p轴，
 *        2              2     自瞄模式
 * ***********************************************************/

#endif 
