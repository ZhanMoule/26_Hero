#include "PID.h"

/**************************����PID���˹��㱣����û���κ�����**********************************/

int Limit_Min_Max(int value,int min,int max);

/**
 * @brief PID�����ʼ��
 * @param PID PID����
 * @param kp 
 * @param ki 
 * @param kd 
 * @param i_max 
 * @param out_max 
 */
void PID_init(PID_struct_t *PID,
              float kp,
              float ki,
              float kd,
              float i_max,
              float out_max)//PID��ʼ������
{
  PID->kp      = kp;
  PID->ki      = ki;
  PID->kd      = kd;
  PID->i_max   = i_max;//�����޷�
  PID->out_max = out_max;//����޷�
}

 void PID_Protect_Angle(PID_struct_t *pid)
{
	if(pid->ref - pid->fdb > 4096)
	{
		pid->fdb+=8190;
	}
	else if(pid->ref - pid->fdb < -4096)
	{
		pid->fdb-=8190;
	}
}

 void PID_Protect_Ink(PID_struct_t *pid)
{
	if(pid->ref - pid->fdb > 180)
	{
		pid->fdb+=360;
	}
	else if(pid->ref - pid->fdb < -180)
	{
		pid->fdb-=360;
	}
}

float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb)//PID���㺯����Ŀ�꣬ʵ�ʣ�
{
  PID->ref = ref;
  PID->fdb = fdb;

	PID_Protect_Angle(PID);//���㱣��

  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;
  
  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}

float PID_Calc_Ink(PID_struct_t *PID, float ref, float fdb)//PID���㺯����Ŀ�꣬ʵ�ʣ�
{
  PID->ref = ref;
  PID->fdb = fdb;

	PID_Protect_Ink(PID);//���㱣��

  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;
  
  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}

float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb)//PID���㺯����Ŀ�꣬ʵ�ʣ�
{
  PID->ref = ref;
  PID->fdb = fdb;

  PID->err[1] = PID->err[0];
  PID->err[0] = PID->ref - PID->fdb;

  PID->p_out  = PID->kp * PID->err[0];
  PID->i_out += PID->ki * PID->err[0];
  PID->d_out  = PID->kd * (PID->err[0] - PID->err[1]);
  PID->i_out=Limit_Min_Max(PID->i_out, -PID->i_max, PID->i_max);
  
  PID->output = PID->p_out + PID->i_out + PID->d_out;
  PID->output=Limit_Min_Max(PID->output, -PID->out_max, PID->out_max);
  return PID->output;
}

/**
 * @brief ����һ���������� value ��ָ������Сֵ min �����ֵ max ֮��
 * @param value ����ֵ
 * @param min ��Сֵ
 * @param max ���ֵ
 * @return 
 */
int Limit_Min_Max(int value,int min,int max)
{
	if(value<min)
		return min;
	else if(value>max)
		return max;
	else return value;
}

