#include "dev_motor_lk.h"
#include "string.h"
#include "FreeRTOS.h"

uint8_t idx = 0; // 电机实例索引
LkMotorInstance_s *motor_LK_instances[LK_MOTOR_MAX_CNT] = {0}; // 存储电机实例的数组

static void Motor_Lk_Decode(CanInstance_s *can_instance){
    if(can_instance == NULL || can_instance->parent_ptr == NULL) {
        return;
    }
    LkMotorInstance_s *motor = (LkMotorInstance_s *)can_instance->parent_ptr;
    switch(can_instance->rx_buff[0]){
        case 0xA1: // 广播模式反馈数据
        motor->motor_temperature    = can_instance->rx_buff[1];
        motor->torque_current = (int16_t)(can_instance->rx_buff[3] << 8 | can_instance->rx_buff[2]);
        motor->rotor_velocity = (int16_t)(can_instance->rx_buff[5] << 8 | can_instance->rx_buff[4]);
        motor->rotor_position = (int16_t)(can_instance->rx_buff[7] << 8 | can_instance->rx_buff[6]);
        break;
        default:
        break;
    }
}

LkMotorInstance_s *Motor_Lk_Register(LkMotorInitConfig_s *config) {
    if (config == NULL || idx >= LK_MOTOR_MAX_CNT) {
        return NULL;
    }
    // 分配内存
    LkMotorInstance_s *motor = (LkMotorInstance_s *)pvPortMalloc(sizeof(LkMotorInstance_s));
    memset(motor, 0, sizeof(LkMotorInstance_s)); // 清空内存
    // 初始化电机基本信息
    motor->id = config->id;
    motor->reduction_ratio = config->reduction_ratio;
    motor->control_mode = config->control_mode;
    motor->type = config->type;
    // 初始化PID
    motor->angle_pid = Pid_Register(&config->angle_pid_config);
    motor->velocity_pid = Pid_Register(&config->velocity_pid_config);
    // 注册电机到CAN总线
    config->can_config.can_module_callback = Motor_Lk_Decode;
    config->can_config.parent_ptr = motor;
    motor->can_instance = Can_Register(&config->can_config);

    if (motor->angle_pid == NULL || motor->velocity_pid == NULL || motor->can_instance == NULL) {
        vPortFree(motor); // 释放内存
        return NULL;
    }
    motor_LK_instances[idx] = motor; // 将电机实例添加到电机数组中
    idx++;
    return motor;
}

bool Motor_Lk_Control(LkMotorInstance_s *motor, float target){
    if(motor == NULL || motor->can_instance == NULL) {
        return false;
    }
    if(motor->control_mode == LK_VELOCITY){
        motor->target_velocity = target;
        motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->rotor_velocity);//目前是编码器
    }else if(motor->control_mode == LK_POSITION){
        motor->target_position = target;
        motor->target_velocity = Pid_Calculate(motor->angle_pid, motor->target_position, motor->out_position);
        motor->output = Pid_Calculate(motor->velocity_pid, motor->target_velocity, motor->rotor_velocity);//目前是编码器
    }else{
        return false;
    }
    return true;
}

bool Motor_LK_Transmit(LkMotorInstance_s *motor){
    if(motor == NULL || motor->can_instance == NULL) {
        return false;
    }
    memset(motor->can_instance->tx_buff, 0, sizeof(motor->can_instance->tx_buff));
    for (int i=0; i<idx; i++) {
        motor->can_instance->tx_buff[(motor_LK_instances[i]->id - 1) * 2] = ((int16_t)motor_LK_instances[i]->output) & 0xFF; 
        motor->can_instance->tx_buff[(motor_LK_instances[i]->id - 1) * 2 + 1] = ((int16_t)motor_LK_instances[i]->output >> 8) & 0xFF; 
    }
    if(Can_Transmit(motor->can_instance) == false) {
        return false;
    }
    return true;
}

