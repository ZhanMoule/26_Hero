#include "Gimbal_Fun.h"
#include "can.h"
#include <stdio.h>
#include <string.h>

RC_instance *rc;

DjiMotorInstance_s *shoot_motor1;
DjiMotorInstance_s *shoot_motor2;

CanInstance_s *board_data_exchange;

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
		.control_mode =DM_POSITION,
		.parameters = { 
        .pos_max = 3.14159f,
        .vel_max = 3.0f,   
        .tor_max = 3.0f,
        .kp_max = 25.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.topic_name = "pitch",
    .type = J4310,
		.angle_pid_config={
        .kp = 5.0f ,  
        .ki =  0.0f, 
        .kd =  0.0f,
        .angle_max = 3.1415926f,
        .i_max = 3.0f,
        .out_max = 3.0f,
        },
    .velocity_pid_config={
        .kp =- 0.3f,
        .ki = -0.00001f,
        .kd = 0.0f,
        .i_max = 13.0f,
        .out_max = 7.0f,
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
        .vel_max = 20.0f,   
        .tor_max = 18.0f,
        .kp_max = 0.0f,
        .kd_max = 0.0f,
        .kp_int = 0.0f,
        .kd_int = 0.0f,
    },
		.velocity_pid_config={
				.kp=0.80f,
				.ki=0.00025f,
				.kd=0.0,
				.i_max =4,
				.out_max =8,
		},
		.angle_pid_config={
				.kp=15.0f,
				.ki=0,
				.kd=0,
				.angle_max = 2*PI,
				.i_max=3,
				.out_max=8,
		},
};
   
DmMotorInstance_s  *rammer_motor = NULL;
DmMotorInitConfig_s rammer_motor_config={
		.topic_name = "rammer",
		.control_mode = DM_VELOCITY,
		.parameters = { 
        .pos_max = 3.14159f,
        .vel_max = 3.0f,   
        .tor_max = 3.0f,
        .kp_max = 25.0f,
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
				.kp=18.0f,
				.ki=0.005,
				.kd=5,
				.angle_max = 2*PI,
				.i_max=10,
				.out_max=5,
		},
};

/* 注册摩擦轮 */
DjiMotorInitConfig_s shoot_motor1_config={
        .id = 2,
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
            .rx_id = 0x202,
        }
    };
DjiMotorInitConfig_s shoot_motor2_config={
        .id = 1,
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
            .rx_id = 0x201,
        }
    };


		
//板间通信函数
void data_pack()
{
    // 摇杆数据处理（保持不变）转化为无符号数据
//    uint16_t rx = (uint16_t)(rc->data.RChandle.rocker_rx + 660) & 0x7FF;
//    uint16_t ry = (uint16_t)(rc->data.RChandle.rocker_ry + 660) & 0x7FF;
//    uint16_t lx = (uint16_t)(rc->data.RChandle.rocker_lx + 660) & 0x7FF;
//    uint16_t ly = (uint16_t)(rc->data.RChandle.rocker_ly + 660) & 0x7FF;

//    // 拨杆开关处理（保持不变）
//    uint8_t sw_left = (rc->data.RChandle.switch_left - 1) & 0x03;
//    uint8_t sw_right = (rc->data.RChandle.switch_right - 1) & 0x03;

//    // 打包数据（保持不变）压缩到6字节中
//    board_data_exchange->tx_buff[0] = (uint8_t)(rx & 0xFF);
//    board_data_exchange->tx_buff[1] = (uint8_t)(((rx >> 8) & 0x07) | ((ry & 0x1F) << 3));
//    board_data_exchange->tx_buff[2] = (uint8_t)(((ry >> 5) & 0x3F) | ((lx & 0x03) << 6));
//    board_data_exchange->tx_buff[3] = (uint8_t)((lx >> 2) & 0xFF);
//    board_data_exchange->tx_buff[4] = (uint8_t)(((lx >> 10) & 0x01) | ((ly & 0x7F) << 1));
//    board_data_exchange->tx_buff[5] = (uint8_t)(((ly >> 7) & 0x0F) | (sw_left << 4) | (sw_right << 6));
//    
//    // INS角度打包（使用字节6）
//    float yaw = INS_angle[0];
//    // 边界保护将yaw轴角度保持在360度内
//    if (yaw < -2.0f * PI) yaw = -2.0f * PI;
//    if (yaw > 2.0f * PI) yaw = 2.0f * PI;
//    
//    // 线性映射：-2π~2π → 0~255
//    uint8_t yaw_byte = (uint8_t)((yaw + 2.0f * PI) * (255.0f / (4.0f * PI)));
//    board_data_exchange->tx_buff[6] = yaw_byte;															
}



		
void Gimbal_Init(){//将云台各部进行注册
	rc = Remote_Register();//遥控器
	
	board_data_exchange = Can_Register(&board_data_exchange_config);//板间通信
	
  shoot_motor1 = Motor_Dji_Register(&shoot_motor1_config);
  shoot_motor2 = Motor_Dji_Register(&shoot_motor2_config);
	
	yaw_motor = Motor_DM_Register(&yaw_motor_config);
	Motor_Dm_Cmd(yaw_motor,DM_CMD_MOTOR_ENABLE);
	Motor_Dm_Transmit(yaw_motor);
		
	gimbal_pitch_motor =  Motor_DM_Register(&gimbal_pitch_motor_config);
	Motor_Dm_Cmd(gimbal_pitch_motor,DM_CMD_MOTOR_ENABLE);
	Motor_Dm_Transmit(gimbal_pitch_motor);
	
	rammer_motor = Motor_DM_Register(&rammer_motor_config);
	Motor_Dm_Cmd(rammer_motor,DM_CMD_MOTOR_ENABLE);
	Motor_Dm_Transmit(rammer_motor);
}
