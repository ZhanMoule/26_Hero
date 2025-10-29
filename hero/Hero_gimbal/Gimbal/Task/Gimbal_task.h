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

/*************************ң����***************************
 *                                                            *
 *   -----------------------------------------------------    *
 *   |     (��-1)                               (��-1)   |    *
 *   |SW_L|(��-3)                          SW_R|(��-3)   |    *
 *   |     (��-2)                               (��-2)   |    *
 *   |                                                   |    *
 *   |    | ^ |                                | ^ |     |    *
 *   |    | 3 |��ҡ��                     ��ҡ��| 1 |     |    *
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
/***********************ģʽѡ��*******************************
 *
 * ******s[1]**********s[0]***********************************
 *        1              1    ��������̨���ֺ͵��̲���ʧ��
 *        1              3    ң����ʹ�ܣ�����ʧ��
 *        1              2    ����ʹ�ܣ�����ʧ��
 
 *        3              1    ��ͨģʽ����ҡ�˿���p��yaw�ᣬ��ҡ��������
 *        3              2    С����ģʽ��Ħ���������������ϲ��˵����²��ϵ�
 *        3              3    ����ģʽ��p��yaw��������ԴС����
 * 
 * 
 *        2              1			��ͨ�н�ģʽ����ҡ�˿����н�����ҡ�˿���p��yaw
 *        2              3      ���̸���ģʽ
 *        2              2     С����
 * ***********************************************************/

#endif 
