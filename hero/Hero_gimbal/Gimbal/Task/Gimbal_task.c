#include "Gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "math.h"

float pitch_angle = 0.0f;
float yaw_angle = 0.0f;
float rammer_speed = 0.0f;
float rammer_angle =0.0f;
int shoot_speed = 6000;
double g_force=0.0f;

int enable_flag = 0;   // 使能标志，1为使能
int shoot_mode = 0;	   // 发射模式，1为发射
int norm_aim_mode = 0; // 自瞄模式，1为自瞄
int shoot_single_flag=0;//单发模式，1为单发

void Gimbal_task()
{
	Gimbal_Init();
	
	enable_flag = 0;
	norm_aim_mode = 0;

	yaw_angle = IMU_angle[0];
	pitch_angle = IMU_angle[1];

	gimbal_data.yaw_position = yaw_motor->message.position;
	gimbal_data.chassis_vx = 0.0f;
	gimbal_data.chassis_vx = 0.0f;
	gimbal_data.chassis_mode = 0;

	while (1)
	{
		gimbal_data.yaw_position = yaw_motor->message.position;
		Minipc_UpdateAllInstances();
		data_pack();
		Can_Transmit(board_data_exchange);

		imu_angle_rad[0] = (IMU_angle[0]-180) * 3.14159265358979323846f / 180.0f;
		imu_angle_rad[1] = (IMU_angle[1]-180) * 3.14159265358979323846f / 180.0f;
		
		if (rc->dr16_handle.s1 == 1 && rc->dr16_handle.s2 == 1 || rc->dr16_handle.s1 == 0 || rc->dr16_handle.s2 == 0)
		{ // 双拨杆都在上时失能，其他情况都使能
			if (enable_flag == 1)
			{
				shoot_mode =0;

				gimbal_data.chassis_mode = 0;

				Motor_Dm_Cmd(rammer_motor, DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(rammer_motor);

				Motor_Dm_Cmd(gimbal_pitch_motor, DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(gimbal_pitch_motor);

				Motor_Dm_Cmd(yaw_motor, DM_CMD_MOTOR_DISABLE);
				Motor_Dm_Transmit(yaw_motor);
				
				
			}
			enable_flag = 0;
		}
		else
		{
			if (enable_flag == 0)
			{
				norm_aim_mode = 0;

				yaw_angle = IMU_angle[0];
				pitch_angle = IMU_angle[1];

				Motor_Dm_Cmd(rammer_motor, DM_CMD_MOTOR_ENABLE);
				Motor_Dm_Transmit(rammer_motor);

				Motor_Dm_Cmd(gimbal_pitch_motor, DM_CMD_MOTOR_ENABLE);
				Motor_Dm_Transmit(gimbal_pitch_motor);

				Motor_Dm_Cmd(yaw_motor, DM_CMD_MOTOR_ENABLE);
				Motor_Dm_Transmit(yaw_motor);
				
				rammer_angle=rammer_motor->message.position;
			}
			enable_flag = 1;
		}

		if (enable_flag == 1)
		{

			if (norm_aim_mode == 1)
			{
				yaw_angle = 180+((miniPC_ins->message.norm_aim_pack.yaw) * 180.0f / 3.14159265358979323846f);
				pitch_angle = 180+((miniPC_ins->message.norm_aim_pack.pitch) * 180.0f / 3.14159265358979323846f);
			}
			else if (norm_aim_mode == 0)
			{
				pitch_angle -= rc->dr16_handle.ch1 * 0.0003;
				if (pitch_angle > 190)
					pitch_angle = 190;
				if (pitch_angle < 150)
					pitch_angle = 150;

				yaw_angle -= rc->dr16_handle.ch0 * 0.0003;
				if (yaw_angle > 360)
					yaw_angle -= 360;
				if (yaw_angle < 0)
					yaw_angle += 360;
			}

			gimbal_data.chassis_vx = rc->dr16_handle.ch2 / 132.0;
			gimbal_data.chassis_vy = rc->dr16_handle.ch3 / 132.0;
			if (rc->dr16_handle.s1 == 2)
			{
				if (rc->dr16_handle.s2 == 1)
				{
					gimbal_data.chassis_mode = 1;
				}
				if (rc->dr16_handle.s2 == 3)
				{
					gimbal_data.chassis_mode = 2;
				}
				if (rc->dr16_handle.s2 == 2)
				{
					gimbal_data.chassis_mode = 3;
				}
			}
			if (rc->dr16_handle.s1 == 3)
			{
				if (rc->dr16_handle.s2 == 1)
				{
					shoot_mode = 0;
					norm_aim_mode = 0;
				}
				if (rc->dr16_handle.s2 == 3)
				{ 
					shoot_mode = 1;
					norm_aim_mode = 0;
				}
				if (rc->dr16_handle.s2 == 2)
				{
					shoot_mode = 1;
					norm_aim_mode = 1;
				}
			}

		} 
		if (shoot_mode == 1)
		{
			Motor_Dji_Control(shoot_motor1, shoot_speed);//+
			Motor_Dji_Control(shoot_motor2, -shoot_speed);//-
			
			//if (shoot_single_flag==1){
				
			if (rc->dr16_handle.wheel > 330 && shoot_single_flag==1)
			{	
				rammer_angle=round(rammer_angle*3/PI)*(PI/3);
				rammer_angle+= PI/3;		
				if(rammer_angle>3.141592653)
					rammer_angle-= 2*3.141592653;
					shoot_single_flag =0;
			}
			else if (rc->dr16_handle.wheel < -330 && shoot_single_flag==1)
			{
				rammer_angle=ceil(rammer_angle*3/PI)*(PI/3);
				rammer_angle-= PI/3;
				if(rammer_angle<-3.141592653)
					rammer_angle+= 2*3.141592653;
					shoot_single_flag =0;
			}
			if (rc->dr16_handle.wheel ==0) shoot_single_flag =1;
			
					
		}
		else if(shoot_mode == 0)
		{
			rammer_speed = 0;
			Motor_Dji_Control(shoot_motor1, 0);
			Motor_Dji_Control(shoot_motor2, 0);
		}
//		if(IMU_angle[1]>150){
//			g_force=11.817*pow(gimbal_pitch_motor->message.position,4)+115.48*pow(gimbal_pitch_motor->message.position,3)+421.88*pow(gimbal_pitch_motor->message.position,2)+684.12*gimbal_pitch_motor->message.position+417.2;
//		}
//		else{
//			g_force=1;
//		}
		
		Motor_Dji_Transmit(shoot_motor1);
		Motor_Dji_Transmit(shoot_motor2);
  
		// Motor_Dm_Control(gimbal_pitch_motor,0);
		if(gimbal_pitch_motor->velocity_pid->err[0]>0.5||gimbal_pitch_motor->velocity_pid->err[0]<-0.5)
			gimbal_pitch_motor->velocity_pid->i_out=0;
		Motor_Dm_Control(gimbal_pitch_motor, -Pid_Calculate(gimbal_pitch_motor->angle_pid, pitch_angle, IMU_angle[1]));
		if(gimbal_pitch_motor->velocity_pid->err[0]>0.5||gimbal_pitch_motor->velocity_pid->err[0]<-0.5)
			gimbal_pitch_motor->velocity_pid->i_out=0;
		Motor_Dm_Mit_Control(gimbal_pitch_motor, 0.0f, 0.0f, -gimbal_pitch_motor->output);
		Motor_Dm_Transmit(gimbal_pitch_motor); // 发送控制报文
		
		if(yaw_motor->velocity_pid->err[0]>0.3||yaw_motor->velocity_pid->err[0]<-0.3)
			yaw_motor->velocity_pid->i_out=0;
		// Motor_Dm_Control(yaw_motor, Pid_Calculate(yaw_motor->angle_pid, yaw_angle, IMU_angle[0]));
		Motor_Dm_Control(yaw_motor, Pid_Calculate(yaw_motor->angle_pid, yaw_angle, IMU_angle[0]));
		if(yaw_motor->velocity_pid->err[0]>0.3||yaw_motor->velocity_pid->err[0]<-0.3)
			yaw_motor->velocity_pid->i_out=0;
		Motor_Dm_Mit_Control(yaw_motor, 0.0f, 0.0f, yaw_motor->output);
		Motor_Dm_Transmit(yaw_motor);

		Motor_Dm_Control(rammer_motor, rammer_angle);
		Motor_Dm_Mit_Control(rammer_motor, 0.0f, 0.0f, rammer_motor->output);
		Motor_Dm_Transmit(rammer_motor);

		osDelay(1);
	}
}