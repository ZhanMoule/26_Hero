#include "dev_motor_unitree.h"
#include "bsp_uart.h"
#include "alg_crc.h"
#include "string.h"
#include "FreeRTOS.h"

uint8_t Unitree_Motor_idx = 0;                                                    // 电机实例索引
UnitreeMotorInstance_s* Unitree_motor_instance[UNITREE_MOTOR_ID_MAX] = {0}; // 存储电机实例的数组

static void Motor_Unitree_Decode(UartInstance_s* uart_instance) {
    if (uart_instance == NULL ) {
        return;
    }

    for (uint8_t i = 0; i < Unitree_Motor_idx; i++) {
        if (Unitree_motor_instance[i]->uart_instance == uart_instance) {
            // 找到对应的电机实例
            RIS_MotorData_s rx_data;
            memcpy(&rx_data, uart_instance->rx_buff, sizeof(RIS_MotorData_s)); // 拷贝接收数据到结构体
            if (rx_data.head[0] != 0xFD || rx_data.head[1] != 0xEE) {
                Unitree_motor_instance[i]->bad_msg++;
                break; // 数据包头不正确
            }
            if (rx_data.mode.id != Unitree_motor_instance[i]->id && rx_data.mode.id != 15) {
                Unitree_motor_instance[i]->bad_msg++;
                break; // 数据包ID不匹配
            }
            Unitree_motor_instance[i]->calc_crc = Crc_Ccitt_Calculate(0, (uint8_t*)&rx_data, sizeof(RIS_MotorData_s) - sizeof(rx_data.CRC16));
            if (rx_data.CRC16 != Unitree_motor_instance[i]->calc_crc) {
                Unitree_motor_instance[i]->bad_msg++;
                break; // CRC校验失败
            }
            else {
                Unitree_motor_instance[i]->motor_temperature = rx_data.fbk.temp; // 温度
                Unitree_motor_instance[i]->M_err = rx_data.fbk.MError; // 错误码
                Unitree_motor_instance[i]->rotor_velocity = ((float)rx_data.fbk.speed / 256.0f) * 6.28318f;
                Unitree_motor_instance[i]->rotor_torque = ((float)rx_data.fbk.torque) / 256.0f;
                Unitree_motor_instance[i]->position = 6.28318f * ((float)rx_data.fbk.pos) / 32768.0f;
                Unitree_motor_instance[i]->out_torque = Unitree_motor_instance[i]->rotor_torque * Unitree_motor_instance[i]->reduction_ratio;
                Unitree_motor_instance[i]->out_velocity = Unitree_motor_instance[i]->rotor_velocity / Unitree_motor_instance[i]->reduction_ratio;
                float tmp_position = Unitree_motor_instance[i]->position / Unitree_motor_instance[i]->reduction_ratio;
                if (tmp_position > PI) {
                    while (tmp_position > PI) {tmp_position -= 2 * PI;}
                } else if (tmp_position < -PI) {
                    while (tmp_position < -PI) {tmp_position += 2 * PI;}
                }
                Unitree_motor_instance[i]->out_position = tmp_position;
                break;
            }
        }
    }
}

UnitreeMotorInstance_s* Motor_Unitree_Register(UnitreeMotorInitConfig_s* config) {
    if (config == NULL || Unitree_Motor_idx >= UNITREE_MOTOR_ID_MAX) {
        return NULL;
    }
    // 分配内存
    UnitreeMotorInstance_s* motor = (UnitreeMotorInstance_s*)pvPortMalloc(sizeof(UnitreeMotorInstance_s));
    memset(motor, 0, sizeof(UnitreeMotorInstance_s)); //清空内存
    //初始化电机基本信息
    motor->id = config->id;
    motor->type = config->type;
    motor->control_mode = config->control_mode;
    motor->reduction_ratio = config->reduction_ratio;
    //注册电机的UART(RS485)实例
    UartInitConfig_s uart_config = {
        .uart_handle = config->uart_handle,
        .mode = UART_IT_MODE,
        .tx_len = 17,
        .rx_len = 16,
        .uart_module_callback = Motor_Unitree_Decode, // 设置回调函数
    };
    motor->uart_instance = Uart_Register(&uart_config);
    if (motor->uart_instance == NULL) {
        //uart注册失败
        vPortFree(motor); // 释放内存
        return NULL;
    }
    //电机内部数据归零
    motor->out_position = 0.0f;
    motor->out_velocity = 0.0f;
    motor->position = 0.0f;
    motor->rotor_velocity = 0.0f;
    motor->motor_temperature = 0.0f;
    motor->M_err = UNITREE_NORMAL;
    motor->calc_crc = 0;
    motor->bad_msg = 0;
    motor->timeout = 0; // 初始化超时计数

    Unitree_motor_instance[Unitree_Motor_idx++] = motor; //将电机实例加入到电机数组
    return motor;
}

bool Motor_Unitree_Control(UnitreeMotorInstance_s* motor,
                           float T, float W, float Pos, float K_P, float K_W) {
    if (motor == NULL || motor->uart_instance == NULL) return false;
    if (motor->control_mode == GOM8010_TORQUE) {
        motor->T = T;
        motor->W = 0.0f;
        motor->Pos = 0.0f;
        motor->K_P = 0.0f;
        motor->K_W = 0.0f;

        RIS_ControlData_t command;
        command.head[0] = 0xFE;
        command.head[1] = 0xEE;

        SATURATE(motor->id, 0, 15);
        SATURATE(motor->T, -127.99f, 127.99f);

        command.mode.id = motor->id;
        command.mode.status = 1;
        command.cmd.k_pos = 0.0f;
        command.cmd.k_spd = 0.0f;
        command.cmd.pos_des = 0.0f;
        command.cmd.spd_des = 0.0f;
        command.cmd.tor_des = motor->T * 256.0f;
        command.CRC16 = Crc_Ccitt_Calculate(0, (uint8_t*)&command, sizeof(RIS_ControlData_t) - sizeof(command.CRC16));

        Uart_Transmit(motor->uart_instance, (uint8_t*)&command);

        return true;
    }
    else if (motor->control_mode == GOM8010_COMBINATION) {
        motor->T = T;
        motor->W = W;
        motor->Pos = Pos;
        motor->K_P = K_P;
        motor->K_W = K_W;

        RIS_ControlData_t command;
        command.head[0] = 0xFE;
        command.head[1] = 0xEE;

        SATURATE(motor->id, 0, 15);
        SATURATE(motor->K_P, 0.0f, 25.599f);
        SATURATE(motor->K_W, 0.0f, 25.599f);
        SATURATE(motor->T, -127.99f, 127.99f);
        SATURATE(motor->W, -804.00f, 804.00f);
        SATURATE(motor->Pos, -411774.0f, 411774.0f);

        command.mode.id = motor->id;
        command.mode.status = 1;
        command.cmd.k_pos = motor->K_P / 25.6f * 32768.0f;
        command.cmd.k_spd = motor->K_W / 25.6f * 32768.0f;
        command.cmd.pos_des = motor->Pos / 6.28318f * 32768.0f;
        command.cmd.spd_des = motor->W / 6.28318f * 256.0f;
        command.cmd.tor_des = motor->T * 256.0f;
        command.CRC16 = Crc_Ccitt_Calculate(0, (uint8_t*)&command, sizeof(RIS_ControlData_t) - sizeof(command.CRC16));

        Uart_Transmit(motor->uart_instance, (uint8_t*)&command);
        return true;
    }
    else {
        return false;
    }
}

bool Motor_Unitree_Stop(UnitreeMotorInstance_s* motor) {
    if (motor == NULL || motor->uart_instance == NULL) return false;

    RIS_ControlData_t command;
    command.head[0] = 0xFE;
    command.head[1] = 0xEE;

    SATURATE(motor->id, 0, 15);

    command.mode.id = motor->id;
    command.mode.status = 0; // 停止模式
    command.cmd.k_pos = 0.0f;
    command.cmd.k_spd = 0.0f;
    command.cmd.pos_des = 0.0f;
    command.cmd.spd_des = 0.0f;
    command.cmd.tor_des = 0.0f;
    command.CRC16 = Crc_Ccitt_Calculate(0, (uint8_t*)&command, sizeof(RIS_ControlData_t) - sizeof(command.CRC16));

    Uart_Transmit(motor->uart_instance, (uint8_t*)&command);
    return true;
}

bool Motor_Unitree_Control_Raw(UnitreeMotorInstance_s* motor, UnitreeMotorCmd_s* cmd) {
    if (motor == NULL || motor->uart_instance == NULL) return false;
    RIS_ControlData_t command;
    command.head[0] = 0xFE;
    command.head[1] = 0xEE;

    command.mode.id = motor->id;
    command.mode.status = cmd->mode; // 使用传入的模式
    command.cmd.k_pos = cmd->K_P / 25.6f * 32768.0f;
    command.cmd.k_spd = cmd->K_W / 25.6f * 32768.0f;
    command.cmd.pos_des = cmd->Pos / 6.28318f * 32768.0f;
    command.cmd.spd_des = cmd->W / 6.28318f * 256.0f;
    command.cmd.tor_des = cmd->T * 256.0f;
    command.CRC16 = Crc_Ccitt_Calculate(0, (uint8_t*)&command, sizeof(RIS_ControlData_t) - sizeof(command.CRC16));

    Uart_Transmit(motor->uart_instance, (uint8_t*)&command);
    return true;
}
