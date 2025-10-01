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
 *        3              1    ��ͨģʽ����̨yaw ���pitch��ǶȾ�Ϊ0���ֲ�������ҡ�˿��Ƶ��̷�����ҡ�˿���p��yaw��
 *        3              2    С����ģʽ��������̨��ң�������ƣ���������
 *        3              3    ��̨����ģʽ����ҡ�˿���yaw��ת���Լ�����ǰ������ҡ�˿���pitch��
 * 
 * 2����Ħ���֣��ҹ��ֿ��Ʋ�����
 *        2              1
 *        2              3      ����ģʽ����ҡ�˿���p�ᣬ
 *        2              2     ����ģʽ
 * ***********************************************************/

#endif 
