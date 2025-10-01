#ifndef __SHOOT_TASK_H__
#define __SHOOT_TASK_H__

#include "dev_motor_dji.h"
#include "FreeRTOS.h"
#include <stdbool.h>
#include "main.h"
#include "cmsis_os.h"
#include "bsp_uart.h"
#include "dev_remote_control.h"
#include "DM_4310.h"
#include "dev_motor_dji.h"
#include "IMU_Task.h"

typedef struct {
    float chassis_vx;
    float chassis_vy;
    int16_t chassis_mode;
		float yaw_position;
} Gimbal_data;

void Gimbal_Init();
void data_pack();

#endif
