#include "chassis_fun.h"
#include "bsp_can.h"
#include "main.h"

ChassisInstance_s *Chassis;

CanInstance_s *board_data_exchange;

Gimbal_data gimbal_data;

CanInitConfig_s board_data_exchange_config ={
		.topic_name = "data_exchange",
		.can_number = 2,
    .tx_id = 0x301,
    .rx_id = 0x300,
    .can_module_callback = data_unpack
};

static ChassisInitConfig_s Chassis_config={
    .type = Mecanum_Wheel,
		.gimbal_yaw_zero = -1.9,
		.mecanum_steering_message={
		.wheel_radius= 0.12f,
	  .length_a = 0.230f,
    .length_b = 0.330f,
		}, 
		.gimbal_follow_pid_config ={
				.kp = 12.0f,
				.ki = 0.002f,
				.kd = 0.000015f,
				.i_max = 1800.0f,
				.out_max = 8192.0f,
		},
		.motor_config[1] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "1",
    .can_config = {
    .can_number=1,
		.rx_id=0x201,	
		.tx_id=0x200,
    },
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[0] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "2",
    .can_config = {
    .can_number=1,
		.rx_id=0x202,	
		.tx_id=0x200,
    },
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[3] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .reduction_ratio = 19.0f,
    .topic_name = "3",
    .can_config = {
    .can_number=1,
		.rx_id=0x203,	
		.tx_id=0x200,
		},
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  },
    .motor_config[2] = {
    .type = M3508,
    .control_mode = DJI_VELOCITY,
    .topic_name = "4",
   .can_config = {
    .can_number=1,
		.rx_id=0x204, 
		.tx_id=0x200,
    },
    .reduction_ratio = 19.0f,
    .velocity_pid_config={
      .kp = 10.0f,
      .ki = 0.0f,
      .kd = 0.0f,
      .i_max = 1800.0f,
      .out_max = 8192.0f,
    },
  }
};


void Chassis_Motor_Init(){
		board_data_exchange = Can_Register(&board_data_exchange_config);	
		Chassis = Chassis_Register(&Chassis_config);
}


void data_unpack(CanInstance_s *can_instance)
{
    if(can_instance == NULL){
        gimbal_data.chassis_mode = 0;
				gimbal_data.chassis_vx = 0.0f;
				gimbal_data.chassis_vy = 0.0f;
        return;
    }
		else{
		  

		}
}