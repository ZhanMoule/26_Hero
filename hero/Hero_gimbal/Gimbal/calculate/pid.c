#include "pid.h"
#include "bsp_can.h"
//#include "Gimbal_Task.h"
#define PI 3.1415926
void pid_I(Pid_struct *Ipid)//电流环
{
	if (Ipid->goal > Ipid->outmax)
		Ipid->goal = Ipid->outmax - 1;
	else
	if (Ipid->goal < -Ipid->outmax)
		Ipid->goal = -Ipid->outmax + 1;
	Ipid->elast = Ipid->e;
	Ipid->e = Ipid->goal - Ipid->now;
	Ipid->p = Ipid->kp * Ipid->e;//比例
	Ipid->d = Ipid->kd * (Ipid->e - Ipid->elast);//微分
	if (Ipid->e < Ipid->emax && Ipid->e > -Ipid->emax)//积分
		if (Ipid->i < -Ipid->imax)
			Ipid->i = -Ipid->imax;
		else 
		if (Ipid->i > Ipid->imax)
			Ipid->i = Ipid->imax;
		else
			Ipid->i += Ipid->ki * Ipid->e;
	else
		Ipid->i = 0;
	Ipid->out = Ipid->p + Ipid->i + Ipid->d;
	if (Ipid->out > Ipid->outmax)
		Ipid->out = Ipid->outmax;
	else
	if (Ipid->out < -Ipid->outmax)
		Ipid->out = -Ipid->outmax;
}

void pid_V(Pid_struct *Vpid)//速度环
{
	if (Vpid->goal > Vpid->outmax)
		Vpid->goal = Vpid->outmax - 1;
	else if (Vpid->goal < -Vpid->outmax)
		Vpid->goal = -Vpid->outmax + 1;
	Vpid->elast = Vpid->e;
	Vpid->e = Vpid->goal - Vpid->now;
	Vpid->p = Vpid->kp * Vpid->e;
	Vpid->d = Vpid->kd * (Vpid->e - Vpid->elast);
	if (Vpid->e < Vpid->emax && Vpid->e > -Vpid->emax)
	{
		if (Vpid->i < -Vpid->imax)
			Vpid->i = -Vpid->imax;
		else if (Vpid->i > Vpid->imax)
			Vpid->i = Vpid->imax;
		else
			Vpid->i += Vpid->ki * Vpid->e;
	}
	else
		Vpid->i = 0;
	Vpid->out = Vpid->p + Vpid->i + Vpid->d;
	if (Vpid->out > Vpid->outmax)
		Vpid->out = Vpid->outmax;
	else if (Vpid->out < -Vpid->outmax)
		Vpid->out = -Vpid->outmax;
}

void pid_A(Pid_struct *Apid)//角度环
{
	Apid->elast = Apid->e;
	float error0 = Apid->goal - Apid->now;
	while(error0 < -PI)
		error0 += 2*PI;
	while(error0 > PI)
		error0 -= 2*PI;
	Apid->e = error0;
	Apid->p = Apid->kp * Apid->e;
	Apid->d = Apid->kd * (Apid->e - Apid->elast);
	if (Apid->e < Apid->emax && Apid->e > -Apid->emax)
	{
		if (Apid->i < -Apid->imax)
			Apid->i = -Apid->imax;
		else if (Apid->i > Apid->imax)
			Apid->i = Apid->imax;
		else
			Apid->i += Apid->ki * Apid->e;
	}
	else
		Apid->i = 0;
	Apid->out = Apid->p + Apid->i + Apid->d;
	if (Apid->out > Apid->outmax)
		Apid->out = Apid->outmax;
	else if (Apid->out < -Apid->outmax)
		Apid->out = -Apid->outmax;
}

void pid_A_imu(Pid_struct *Apid)
{
	Apid->elast = Apid->e;
	
	
	
	float error0 = Apid->goal - Gimbal.IMUData.IMU_Yaw_angle;
	
	
	
	
	while(error0 < -PI)
		error0 += 2*PI;
	while(error0 > PI)
		error0 -= 2*PI;
	Apid->e = error0;
	Apid->p = Apid->kp * Apid->e;
	Apid->d = Apid->kd * (Apid->e - Apid->elast);
	if (Apid->e < Apid->emax && Apid->e > -Apid->emax)
	{
		if (Apid->i < -Apid->imax)
			Apid->i = -Apid->imax;
		else if (Apid->i > Apid->imax)
			Apid->i = Apid->imax;
		else
			Apid->i += Apid->ki * Apid->e;
	}
	else
		Apid->i = 0;
	Apid->out = Apid->p + Apid->i + Apid->d;
	if (Apid->out > Apid->outmax)
		Apid->out = Apid->outmax;
	else if (Apid->out < -Apid->outmax)
		Apid->out = -Apid->outmax;

}
void pid_DM(Pid_struct *Vpid)
{
	if (Vpid->goal > Vpid->outmax)
		Vpid->goal = Vpid->outmax - 1;
	else if (Vpid->goal < -Vpid->outmax)
		Vpid->goal = -Vpid->outmax + 1;
	Vpid->elast = Vpid->e;
	Vpid->e = Vpid->goal - Vpid->now;
	Vpid->p = Vpid->kp * Vpid->e;
	Vpid->d = Vpid->kd * (Vpid->e - Vpid->elast);
	if (Vpid->i < -Vpid->imax)
		Vpid->i = -Vpid->imax;
	else if (Vpid->i > Vpid->imax)
		Vpid->i = Vpid->imax;
	else
		Vpid->i += Vpid->ki * Vpid->e;
	Vpid->out = Vpid->p + Vpid->i + Vpid->d;
	if (Vpid->out > Vpid->outmax)
		Vpid->out = Vpid->outmax;
	else if (Vpid->out < -Vpid->outmax)
		Vpid->out = -Vpid->outmax;
}

void NoCurrent_speed_control(Cal_struct *motor)//单环控速度
{
	pid_V(&motor->Vpid);
}

void DM_speed_control(Cal_struct *motor)
{
	pid_DM(&motor->Vpid);
}

void NoCurrent_position_control(Cal_struct *motor)//双环控角度
{
	pid_A(&motor->Apid);
	motor->Vpid.goal =motor->Apid.out;
	pid_V(&motor->Vpid);
}

void DM_position_control(Cal_struct *motor)
{
	pid_A(&motor->Apid);
	motor->Vpid.goal = motor->Apid.out;
	pid_DM(&motor->Vpid);
}

void DM_position_control_imu(Cal_struct *motor)
{
	pid_A_imu(&motor->Apid);
	motor->Vpid.goal = motor->Apid.out;
	pid_DM(&motor->Vpid);
}

void Chassis_PID_Init(Cal_struct *Motor)//底盘pid初始化
{
	Motor->Vpid.kp = 2200;
	Motor->Vpid.ki = 0;
	Motor->Vpid.kd = 0;
	Motor->Vpid.emax = 1;
	Motor->Vpid.imax = 500;
	Motor->Vpid.outmax = 2000;
}

void Wheel1_PID_Init(Cal_struct *Motor)//轮子pid初始化
{
	Motor->Vpid.kp = 8;
	Motor->Vpid.ki = 0.6;
	Motor->Vpid.kd = 0.083;
	Motor->Vpid.emax = 100;
	Motor->Vpid.imax = 8000;
	Motor->Vpid.outmax = 16383;
}

void Wheel2_PID_Init(Cal_struct *Motor)//轮子pid初始化
{
	Motor->Vpid.kp = 8;
	Motor->Vpid.ki = 0.6;
	Motor->Vpid.kd = 0.083;
	Motor->Vpid.emax = 100;
	Motor->Vpid.imax = 8000;
	Motor->Vpid.outmax = 16383;
}

void Wheel3_PID_Init(Cal_struct *Motor)//轮子pid初始化
{
	Motor->Vpid.kp = 8;
	Motor->Vpid.ki = 0.6;
	Motor->Vpid.kd = 0.083;
	Motor->Vpid.emax = 100;
	Motor->Vpid.imax = 8000;
	Motor->Vpid.outmax = 16383;
}

void Wheel4_PID_Init(Cal_struct *Motor)//轮子pid初始化
{
	Motor->Vpid.kp = 8;
	Motor->Vpid.ki = 0.6;
	Motor->Vpid.kd = 0.083;
	Motor->Vpid.emax = 100;
	Motor->Vpid.imax = 8000;
	Motor->Vpid.outmax = 16383;
}

void Yaw_PID_Init(Cal_struct *Motor)
{
	Motor->Vpid.kp = 0.75;
	Motor->Vpid.ki = 0.002;
	Motor->Vpid.kd = 0;
	Motor->Vpid.emax = 3;
	Motor->Vpid.imax = 4;
	Motor->Vpid.outmax = 8;
	Motor->Apid.kp = 26;
	Motor->Apid.ki = 0;
	Motor->Apid.kd = 35;
	Motor->Apid.emax = 1;
	Motor->Apid.imax = 3;
	Motor->Apid.outmax = 8;
	Motor->Apid.goal = Motor->Apid.now;//Motor->Apid.now
}

void Yaw_PID_Init_imu(Cal_struct *Motor)
{
	Motor->Vpid.kp = 0.85;
	Motor->Vpid.ki = 0.002;
	Motor->Vpid.kd = 0;
	Motor->Vpid.emax = 3;
	Motor->Vpid.imax = 0.4;
	Motor->Vpid.outmax = 8;
	Motor->Apid.kp = 12;
	Motor->Apid.ki = 0;
	Motor->Apid.kd = 5;
	Motor->Apid.emax = 1;
	Motor->Apid.imax = 3;
	Motor->Apid.outmax = 8;

}

void Rammer_PID_Init(Cal_struct *Motor)
{
	Motor->Vpid.kp = 0.35;
	Motor->Vpid.ki = 0.08;
	Motor->Vpid.kd = 1.5;
	Motor->Vpid.emax = 30;
	Motor->Vpid.imax = 10;
	Motor->Vpid.outmax = 3;
	Motor->Apid.kp = 18;
	Motor->Apid.ki = 0.005;
	Motor->Apid.kd = 5;
	Motor->Apid.emax = 1;
	Motor->Apid.imax = 10;
	Motor->Apid.outmax = 5;
	Motor->Apid.goal = Motor->Apid.now;
}
