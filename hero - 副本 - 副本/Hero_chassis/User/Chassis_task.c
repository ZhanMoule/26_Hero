#include "Chassis_task.h"

void Chassis_task(){		
		Chassis_Motor_Init();
		
		Can_Transmit(board_data_exchange);
		data_unpack(board_data_exchange);		
	
		Chassis->absolute_chassis_speed.Vx=0.0;
		Chassis->absolute_chassis_speed.Vy=0.0;
		
		Chassis_Mode_Choose(Chassis,CHASSIS_NORMAL);
	
		while(1){						
				Can_Transmit(board_data_exchange);
				data_unpack(board_data_exchange);	

				if(gimbal_data.chassis_mode == 0){
						Chassis_Mode_Choose(Chassis,CHASSIS_SLOW);
				}
				else if(gimbal_data.chassis_mode == 1){					
						Chassis_Mode_Choose(Chassis,CHASSIS_NORMAL);
						
        }
        else if (gimbal_data.chassis_mode  == 2){
						Chassis_Mode_Choose(Chassis,CHASSIS_FOLLOW_GIMBAL);

        }
				else if (gimbal_data.chassis_mode  == 3){
					  Chassis_Mode_Choose(Chassis,CHASSIS_GYROSCOPE);
        }
	
				Chassis->absolute_chassis_speed.Vx=gimbal_data.chassis_vx;
				Chassis->absolute_chassis_speed.Vy=-gimbal_data.chassis_vy;							

        Chassis->gimbal_yaw_angle=gimbal_data.yaw_position;
        Chassis_Control(Chassis);			
				
		    osDelay(1);
			
		}
}
