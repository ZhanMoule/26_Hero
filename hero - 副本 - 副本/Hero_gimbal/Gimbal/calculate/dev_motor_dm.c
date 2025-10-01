#include "dev_motor_dm.h"

static DMMotorInstance_s *motor_dm_instances;

static void Motor_DM_Decode(CanInstance_s *can_instance){
    if(can_instance == NULL){
        return;
    }
    uint8_t *rx_buff = can_instance->rx_buff;
    DMMotorInstance_s *motor = (DMMotorInstance_s *)can_instance->id;

    uint16_t tmp = (uint16_t)((rx_buff[0] << 4) | (rx_buff[1] >> 4));
    motor->error_code = (float)(tmp >> 4);

    motor->last_position = motor->angle;
    tmp = (uint16_t)((rx_buff[1] << 8) | rx_buff[2]);
    motor->angle = ((float)tmp) * (P_MAX - P_MIN) / ((float)((1 << 16) - 1)) + P_MIN;

    tmp = (uint16_t)((rx_buff[3] << 4) | rx_buff[4] >> 4);
    motor->velocity = ((float)tmp) * (V_MAX - V_MIN) / ((float)((1 << 12) - 1)) + V_MIN;

    tmp = (uint16_t)(((rx_buff[4] & 0xff) << 8) | rx_buff[5]);
    motor->torque = ((float)tmp) * (T_MAX - T_MIN) / ((float)((1 << 12) - 1)) + T_MIN;

    motor->T_MOS = (float)rx_buff[6];
    motor->T_Rotor = (float)rx_buff[7];
	
	motor->out_position = motor->angle;
	motor->out_velocity = motor->velocity;
}

DMMotorInstance_s *Motor_DM_Register(DMMotorInitConfig_s *config) {
    if (config == NULL) {
        return 0;
    }
    // 初始化电机基本信息
    DMMotorInstance_s *motor_instance = (DMMotorInstance_s *) pvPortMalloc (sizeof(DMMotorInstance_s));
    memset(motor_instance, 0, sizeof(DMMotorInstance_s));
	
	motor_instance->control_mode = config->control_mode;
    // 初始化PID
    motor_instance->angle_pid = Pid_Register(&config->angle_pid_config);
    motor_instance->velocity_pid = Pid_Register(&config->velocity_pid_config);
    // 注册电机到CAN总线
	config->can_config.can_module_callback = Motor_DM_Decode;
    config->can_config.id = motor_instance;
    motor_instance->can_instance = Can_Register(&config->can_config);
    if (motor_instance->can_instance == NULL) {
        vPortFree(motor_instance);
        return NULL;
    }
    motor_dm_instances = motor_instance;
    return motor_instance;
}

bool Motor_DM_Control(DMMotorInstance_s *motor, float target) {
    if(motor == NULL || motor->can_instance == NULL) {
        return false;
    }
	float out_current = 0.0f;
	memset(motor->can_instance->tx_buff, 0 , 8);

    switch (motor->control_mode) {
		case POSITION:
			motor->target_position = target;
			motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->out_position);
			out_current = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->out_velocity);
			break;
		case VELOCITY:
			motor->target_velocity = target;
			out_current = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->out_velocity);
			break;
		default:
			return false;
	}
    out_current = fminf(fmaxf(out_current, T_MIN), T_MAX);

	uint16_t result = (int)((out_current - T_MIN) * ((float)((1 << 12) - 1)) / (T_MAX - T_MIN));
	
    motor->can_instance->tx_buff[6] = (uint8_t)(result>>8);
    motor->can_instance->tx_buff[7] = (uint8_t)(result & 0xff);

    Can_Transmit(motor->can_instance); 

    return true;
}

bool Motor_DM_Change_Mode(DMMotorInstance_s *motor, DMMotorControlMode_e target_mode){
    if(motor == NULL) {
        return false;
    }
    motor->control_mode = target_mode;
    return true;
}

bool DM_Motor_cmd(DMMotorInstance_s *motor, DMMotor_Mode_e cmd) {
	if (motor == NULL || motor->can_instance == NULL ){
		return false;
	}
	int cnt = 0;
	while (!motor->error_code) {
		memset(motor->can_instance->tx_buff, 0xFF, 7);
		motor->can_instance->tx_buff[7] = (uint8_t)cmd;
		Can_Transmit(motor->can_instance);
		osDelay(2);
		cnt++;
		if (cnt == 10){
			return false;
		}
	}
	
	return true;
}
