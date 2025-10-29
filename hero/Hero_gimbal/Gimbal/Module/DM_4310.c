#include "DM_4310.h"
#include <stdlib.h>

static int float_to_uint(const float x_float, const float x_min, const float x_max, const int bits){
    const float span = x_max - x_min;
    const float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
static float uint_to_float(const int x_int, const float x_min, const float x_max, const int bits){
    const float span = x_max - x_min;
    const float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}
static float  Angle_Normalize (float angle){
    while (angle > 3.141593f)
        angle -=  2*3.141593f;
    while (angle < -3.141593f)
        angle +=  2*3.141593f;
    return angle;
}

static void Motor_Dm_Decode( CanInstance_s *can_instance){
    if(can_instance == NULL){
        return;
    }
    const uint8_t *rx_buff = can_instance->rx_buff;
    DmMotorInstance_s *motor = can_instance->parent_ptr;
    motor->motor_state = rx_buff[0] >> 4;
    motor->message.pos_int = rx_buff[1] << 8 | rx_buff[2];
    motor->message.vel_int = rx_buff[3] << 4 | rx_buff[4] >> 4;
    motor->message.tor_int = (rx_buff[4] & 0xF) << 8 | rx_buff[5];
    motor->message.last_position = motor->message.position;
    motor->message.last_out_position = motor->message.out_position;
    motor->message.position = uint_to_float(motor->message.pos_int, -motor->parameters.pos_max, motor->parameters.pos_max, 16); // (-P_MAX,P_MAX)
    motor->message.out_position = Angle_Normalize(motor->message.position);  // (-PI,PI)
    motor->message.out_velocity = uint_to_float(motor->message.vel_int, -motor->parameters.vel_max, motor->parameters.vel_max, 12); // (-V_MAX,V_MAX)
    motor->message.torque = uint_to_float(motor->message.tor_int, -motor->parameters.tor_max, motor->parameters.tor_max, 12); // (-T_MAX,T_MAX)
    motor->message.tem_mos = (float)(rx_buff[6]);
    motor->message.tem_rotor = (float)(rx_buff[7]);

    int res1=0, res2=0;
    if(motor->message.position < motor->message.last_position){
        res1 = motor->message.position + motor->parameters.pos_max - motor->message.last_position;    //正转，delta=+
        res2 = motor->message.position - motor->message.last_position;                        //反转    delta=-
    }
    else{
        res1 = motor->message.position - motor->message.last_position;                        //正转    delta +
        res2 = motor->message.position - motor->parameters.pos_max - motor->message.last_position ;   //反转    delta -
    }
    if(abs(res1)<abs(res2))
        motor->message.total_angle += res1;
    else
        motor->message.total_angle += res2;
    if(fabsf(motor->message.total_angle) > FLOAT_MAX){
        motor->message.total_angle = 0;
    }
}



DmMotorInstance_s *Motor_DM_Register(DmMotorInitConfig_s *config){
    if (config == NULL)
    {
        return NULL;
    }
    DmMotorInstance_s *motor_instance = (DmMotorInstance_s *)pvPortMalloc(sizeof(DmMotorInstance_s));
    if (motor_instance == NULL)
    {
        return NULL; // 内存分配失败
    }
    motor_instance->topic_name = config->topic_name;
    config->can_config.topic_name = config->topic_name;
    motor_instance->control_mode = config->control_mode;
    motor_instance->parameters.pos_max = config->parameters.pos_max;
    motor_instance->parameters.vel_max = config->parameters.vel_max;
    motor_instance->parameters.tor_max = config->parameters.tor_max;
    motor_instance->parameters.kp_max = config->parameters.kp_max;
    motor_instance->parameters.kd_max = config->parameters.kd_max;

    config->can_config.parent_ptr = motor_instance;
    config->can_config.can_module_callback = Motor_Dm_Decode;
    motor_instance->can_instance = Can_Register(&config->can_config);
    motor_instance->angle_pid = Pid_Register(&config->angle_pid_config);
    motor_instance->velocity_pid = Pid_Register(&config->velocity_pid_config);
    return motor_instance;
}

/**
 * @brief DM电机命令帧
 */
uint8_t dm_cmd_frame[4][8] ={
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC}, // 使能
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD}, // 失能
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFE}, // 保存位置零点
    {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFB}  // 清除电机错误
};

bool Motor_Dm_Cmd(DmMotorInstance_s *motor, DmMotorMode_e cmd){
    if (motor == NULL || motor->can_instance == NULL || cmd >= 4)
    {
        return false;
    }
    memcpy(motor->can_instance->tx_buff, dm_cmd_frame[cmd], 8);  // 复制8个字节
    return true;
}
bool Motor_Dm_Mit_Control(const DmMotorInstance_s *motor, const float pos, const float vel, const float tor){
    if (motor == NULL || motor->can_instance == NULL || motor->motor_state != DM_ENABLE){
        return false;
    }
        const uint16_t pos_tmp = float_to_uint(pos, -motor->parameters.pos_max, motor->parameters.pos_max, 16);
        const uint16_t vel_tmp = float_to_uint(vel, -motor->parameters.vel_max, motor->parameters.vel_max, 12);
        const uint16_t kp_tmp = float_to_uint(motor->parameters.kp_int, 0, motor->parameters.kp_max, 12);
        const uint16_t kd_tmp = float_to_uint(motor->parameters.kd_int, 0, motor->parameters.kd_max, 12);
        const uint16_t tor_tmp = float_to_uint(tor, -motor->parameters.tor_max, motor->parameters.tor_max, 12);

        motor->can_instance->tx_buff[0] = (pos_tmp >> 8);
        motor->can_instance->tx_buff[1] = pos_tmp;
        motor->can_instance->tx_buff[2] = (vel_tmp >> 4);
        motor->can_instance->tx_buff[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
        motor->can_instance->tx_buff[4] = kp_tmp;
        motor->can_instance->tx_buff[5] = (kd_tmp >> 4);
        motor->can_instance->tx_buff[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
        motor->can_instance->tx_buff[7] = tor_tmp;
        return true;
}

bool Motor_Dm_Control(DmMotorInstance_s *motor, const float target) {
    if(motor == NULL || motor->can_instance == NULL || motor->motor_state != DM_ENABLE) {
        return false;
    }
    memset(motor->can_instance->tx_buff, 0 , 8);
    if (motor->control_mode == DM_POSITION) {
        motor->target_position = target;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->message.out_position);
        motor->output= Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.out_velocity);
    }
    else
        if (motor->control_mode == DM_VELOCITY) {
            motor->target_velocity = target;
            motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->message.out_velocity);
        }else {
        return false;
    }
    return true;
}

bool Motor_Dm_Pos_Vel_Control(const DmMotorInstance_s *motor, float pos, float vel){
    if (motor == NULL || motor->can_instance == NULL || motor->motor_state != DM_ENABLE){
        return false;
    }
    const uint8_t* pos_buf = (uint8_t*)&pos;
    const uint8_t* vel_buf = (uint8_t*)&vel;

    motor->can_instance->tx_buff[0] = *pos_buf;
    motor->can_instance->tx_buff[1] = *(pos_buf+1);
    motor->can_instance->tx_buff[2] = *(pos_buf+2);
    motor->can_instance->tx_buff[3] = *(pos_buf+3);
    motor->can_instance->tx_buff[4] = *vel_buf;
    motor->can_instance->tx_buff[5] = *(vel_buf+1);
    motor->can_instance->tx_buff[6] = *(vel_buf+2);
    motor->can_instance->tx_buff[7] = *(vel_buf+3);
    return true;
}


bool Motor_Dm_Transmit(const DmMotorInstance_s *motor){
    if (motor == NULL || motor->can_instance == NULL){
        return false;
    }
    return Can_Transmit(motor->can_instance);
}