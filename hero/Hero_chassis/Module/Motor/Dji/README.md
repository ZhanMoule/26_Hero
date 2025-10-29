# dev_motor_dji的
## 概述
dev_motor_dji是一个用于驱动大疆电机的程序，提供了大疆电机的控制功能。

## 开发环境
- 硬件平台：STM32F4系列(RoboMasterC板)
- 开发工具：Keil MDK-ARM
- 相关库：CMSIS, HAL

## 适配电机
- 大疆M2006、M3508、GM6020

## 软件要求
- 需要bsp_can、alg_pid库
- 需要正确配置CAN总线

## 实例
```c
#include "dev_motor_dji.h"
DjiMotorInstance_s *motor;
DjiMotorInitConfig_s config = {
    .reduction_ratio = 19f,
    .control_mode = DJI_VELOCITY,
    .type = M2006,
    .topic_name = "dji_motor",
    .velocity_pid_config = {
        .kp = 1.0f,
        .ki = 0.0f,
        .kd = 0.0f,
        .max_output = 10000.0f,
    .can_config = {
        .can_number = 1,
        .tx_id = 0x200,
        .rx_id = 0x201,
    }
    }

    void test_task(void) {
        motor = Motor_Dji_Register(&config);
        while (1) {
            Motor_Dji_Control(motor, 1000); // 以1000rpm的速度运行电机
            Motor_Dji_Transmit(motor); // 发送控制命令
            osDelay(1);
        }
    }
};



