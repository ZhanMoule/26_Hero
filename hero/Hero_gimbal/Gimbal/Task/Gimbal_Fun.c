#include "Gimbal_Fun.h"
#include "can.h"
#include <stdio.h>
#include <string.h>

MiniPC_Instance *miniPC_ins;
 
Dr16Instance_s *rc;

DjiMotorInstance_s *shoot_motor1;
DjiMotorInstance_s *shoot_motor2;

CanInstance_s *board_data_exchange;

Gimbal_data gimbal_data;

float imu_angle_rad[2];


MiniPC_Config miniPC_config = {
    .callback = NULL,
    .message_type = USB_MSG_AIM_RX, // 自瞄数据
    .Send_message_type = USB_MSG_AIM_TX // 发送数据类型
	 
};



CanInitConfig_s board_data_exchange_config ={
	.can_number = 2,
    .tx_id = 0x300,
    .rx_id = 0x301,
	.topic_name = "board",	
};

/*dm电机注册-----Pitch轴*/
DmMotorInstance_s  *gimbal_pitch_motor  =NULL;
DmMotorInitConfig_s gimbal_pitch_motor_config = {
    .can_config={
            .can_number = 1,
            .tx_id = 0x05,
            .rx_id = 0x15,
        },
		.control_mode =DM_VELOCITY,
		.parameters = { 
        .pos_max = 3.141593f,
        .vel_max = 45.0f,   
        .tor_max = 18.0f,
        .kp_max = 0.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.topic_name = "pitch",
    .type = J4310,
		.angle_pid_config={
        .kp = 0.5f ,  
        .ki =  0.0f, 
        .kd =  15.0f,
        .angle_max = 360.0f,
        .i_max = 3.0f,
        .out_max = 8.0f,
        },
    .velocity_pid_config={
        .kp = 1.3f,
        .ki = 0.07f,
        .kd = 0.0f,
        .i_max = 8.0f,
        .out_max = 18.0f,
        },
   
};

DmMotorInstance_s  *yaw_motor =NULL;
DmMotorInitConfig_s yaw_motor_config={
		.topic_name = "yaw_motor",
		.control_mode = DM_VELOCITY,
		.type = J4310,
		.can_config ={
					.can_number=2,
					.tx_id = 0x03, 
					.rx_id = 0x14,
		},
		.parameters = {
        .pos_max = 3.141593f,
        .vel_max = 45.0f,   
        .tor_max = 18.0f,
        .kp_max = 0.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.velocity_pid_config={
				.kp=0.6f,
				.ki=0.07f,
				.kd=0.0015f,
				.i_max =4,
				.out_max =18,
		},
		.angle_pid_config={
				.kp=0.6f,
				.ki=0.0f,
				.kd=15.0f,
				.angle_max = 360.0f,
				.i_max=3,
				.out_max=45,
		},
};
   
DmMotorInstance_s  *rammer_motor = NULL;
DmMotorInitConfig_s rammer_motor_config={
		.topic_name = "rammer",
		.control_mode = DM_POSITION,
		.parameters = { 
        .pos_max = 3.14159f,
        .vel_max = 3.0f,   
        .tor_max = 3.0f,
        .kp_max = 0.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.can_config ={
					.can_number=2,
					.tx_id = 0x05, 
					.rx_id = 0x15,
		},
		.velocity_pid_config={
				.kp=0.35f,
				.ki=0.01f,
				.kd=1.5,
				.i_max =10,
				.out_max =3,
		},
		.angle_pid_config={
				.kp=5.0f,
				.ki=0.0f,
				.kd=0.0f,
				.angle_max = 2*PI,
				.i_max=10,
				.out_max=0.5,
		},
};

/* 注册摩擦轮 */
DjiMotorInitConfig_s shoot_motor1_config={
        .id = 1,
        .type = M3508,
				.topic_name = "dji_motor1",
        .control_mode = DJI_VELOCITY,
        .reduction_ratio = 1,
        .velocity_pid_config={
            .kp = 20.0f,
            .ki = 0.0f,
            .kd = 8.0f,
            .angle_max = 0.0f,
            .i_max = 1000.0f,
            .out_max = 16384.0f,
        },
        .can_config={
            .can_number = 1,
            .tx_id = 0x200,
            .rx_id = 0x201,
        }
    };
DjiMotorInitConfig_s shoot_motor2_config={
        .id = 2,
        .type = M3508,
				.topic_name = "dji_motor2",
        .control_mode = DJI_VELOCITY,
        .reduction_ratio = 1,
        .velocity_pid_config={
            .kp = 20.0f,
            .ki = 0.0f,
            .kd = 8.0f,
            .angle_max = 0.0f,
            .i_max = 1000.0f,
            .out_max = 16384.0f,
        },
        .can_config={
            .can_number = 1,
            .tx_id = 0x200,
            .rx_id = 0x202,
        }
    };


		
//板间通信函数
void data_pack()
{
    // 缩放数据到合适的位数
    // yaw 范围 -2π 到 2π (-6.283 到 6.283)，乘以 500 得到 -3142 到 3142
    int16_t yaw_scaled = (int16_t)(gimbal_data.yaw_position * 500);
    int16_t vx_scaled = (int16_t)(gimbal_data.chassis_vx * 10);  // 8位: -100~100
    int16_t vy_scaled = (int16_t)(gimbal_data.chassis_vy * 10);  // 8位: -100~100
    
    // 打包到4字节
    uint32_t packed_data = 0;
    packed_data |= ((uint32_t)(gimbal_data.chassis_mode & 0x03)) << 30;  // 2位: 位置30-31
    packed_data |= ((uint32_t)(yaw_scaled & 0x0FFF)) << 18;              // 12位: 位置18-29
    packed_data |= ((uint32_t)(vx_scaled & 0x00FF)) << 10;               // 8位: 位置10-17
    packed_data |= ((uint32_t)(vy_scaled & 0x00FF)) << 2;                // 8位: 位置2-9
    // 低2位保留为0
    
    // 直接写入发送缓冲区的前4个字节
    board_data_exchange->tx_buff[0] = (packed_data >> 0) & 0xFF;
    board_data_exchange->tx_buff[1] = (packed_data >> 8) & 0xFF;
    board_data_exchange->tx_buff[2] = (packed_data >> 16) & 0xFF;
    board_data_exchange->tx_buff[3] = (packed_data >> 24) & 0xFF;
    
    // 将剩余的4个字节清零
    board_data_exchange->tx_buff[4] = 0;
    board_data_exchange->tx_buff[5] = 0;
    board_data_exchange->tx_buff[6] = 0;
    board_data_exchange->tx_buff[7] = 0;					
}



		
void Gimbal_Init(){//将云台各部进行注册
	rc = Dr16_Register(&huart3);//遥控器
	
	board_data_exchange = Can_Register(&board_data_exchange_config);//板间通信
	
  shoot_motor1 = Motor_Dji_Register(&shoot_motor1_config);
  shoot_motor2 = Motor_Dji_Register(&shoot_motor2_config);
	
	yaw_motor = Motor_DM_Register(&yaw_motor_config);
	Motor_Dm_Cmd(yaw_motor,DM_CMD_MOTOR_DISABLE);
	Motor_Dm_Transmit(yaw_motor);
		
	gimbal_pitch_motor =  Motor_DM_Register(&gimbal_pitch_motor_config);
	Motor_Dm_Cmd(gimbal_pitch_motor,DM_CMD_MOTOR_DISABLE);
	Motor_Dm_Transmit(gimbal_pitch_motor);
	
	rammer_motor = Motor_DM_Register(&rammer_motor_config);
	Motor_Dm_Cmd(rammer_motor,DM_CMD_MOTOR_DISABLE);
	Motor_Dm_Transmit(rammer_motor);
	
	miniPC_ins=Minipc_Register(&miniPC_config);
  Minipc_ConfigAimTx(miniPC_ins, &imu_angle_rad[0], &imu_angle_rad[1], 0, 0, 0, 0);
}