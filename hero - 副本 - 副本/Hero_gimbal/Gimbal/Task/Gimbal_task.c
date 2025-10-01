#include "Gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"

float pitch_angle =0.0f;
float yaw_angle =0.0f;
float rammer_speed=0.0f;

float chassis_vx=0.0f;
float chassis_vy=0.0f;


void Gimbal_task(){
		
    Gimbal_Init();
	
		yaw_angle =INS_angle[0];
		
    while (1){
				data_pack();
				Can_Transmit(board_data_exchange);	
			
			if (rc->data.RChandle.switch_left==1 && rc->data.RChandle.switch_right==1){//如果拨杆s1和s2都拨到最上面，整车失能
				//云台失能
					Motor_Dm_Cmd(yaw_motor,DM_CMD_MOTOR_DISABLE);
					Motor_Dm_Transmit(yaw_motor);
				
					Motor_Dm_Cmd(gimbal_pitch_motor,DM_CMD_MOTOR_DISABLE);
					Motor_Dm_Transmit(gimbal_pitch_motor);
				
					Motor_Dm_Cmd(rammer_motor,DM_CMD_MOTOR_DISABLE);
					Motor_Dm_Transmit(rammer_motor);
				
					Motor_Dji_Control(shoot_motor1, 0);	
					Motor_Dji_Control(shoot_motor2, 0);
				//底盘失能
				
				}
				
			
				
				
			
				if(rc->data.RChandle.switch_left==1){
						pitch_angle+=rc->data.RChandle.rocker_ry*0.000001;	
						if(pitch_angle>0.6)
								pitch_angle=0.6;
						if(pitch_angle<-0.2)
								pitch_angle=-0.2;
						
						yaw_angle-=rc->data.RChandle.rocker_rx*0.000005;									
						if(yaw_angle>2*PI)
						  	yaw_angle-=2*PI;
						if(yaw_angle<-2*PI)
								yaw_angle+=2*PI;
				}
				if(rc->data.RChandle.switch_left==3){
						if(rc->data.Keyboard.key_bits.rocker_roll>1300){
								rammer_speed=0.2;
						}
						else if(rc->data.Keyboard.key_bits.rocker_roll<700)
								rammer_speed=-0.2;
						else 
								rammer_speed=0;			
				}
				else
						rammer_speed=0;	
				
				if (rc->data.RChandle.switch_left==3){
						Motor_Dji_Control(shoot_motor1, -3000);	
						Motor_Dji_Control(shoot_motor2, 3000);	
				}
				else{
						Motor_Dji_Control(shoot_motor1, 0);	
						Motor_Dji_Control(shoot_motor2, 0);	
						
				}
				
				Motor_Dji_Transmit(shoot_motor1);
				Motor_Dji_Transmit(shoot_motor2);
				
				Motor_Dm_Control(gimbal_pitch_motor,pitch_angle);
        Motor_Dm_Mit_Control(gimbal_pitch_motor, 0.0f, 0.0f, gimbal_pitch_motor->output); 
        Motor_Dm_Transmit(gimbal_pitch_motor); //发送控制报文
				
				
				Motor_Dm_Control(yaw_motor, Pid_Calculate(yaw_motor->angle_pid, yaw_angle, INS_angle[0]));
        Motor_Dm_Mit_Control(yaw_motor, 0.0f, 0.0f, yaw_motor->output);
        Motor_Dm_Transmit(yaw_motor); 
				
				Motor_Dm_Control(rammer_motor, rammer_speed);
        Motor_Dm_Mit_Control(rammer_motor, 0.0f, 0.0f, rammer_motor->output);
        Motor_Dm_Transmit(rammer_motor); 
		
				osDelay(1); 
    }


}