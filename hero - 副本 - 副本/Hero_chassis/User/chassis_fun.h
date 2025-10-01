#ifndef CHASSIS_FUN_H
#define CHASSIS_FUN_H

#include <stdbool.h>
#include "dev_motor_dji.h"
#include "dev_remote_control.h"
#include "dev_motor_dm.h"
#include "alg_chassis_calc.h"

typedef struct {
    float chassis_vx;
    float chassis_vy;
    int16_t chassis_mode;
		int16_t yaw_position;
} Gimbal_data;


void Chassis_Motor_Init();

void data_unpack(CanInstance_s *can_instance);


#endif // CHASSIS_MOTOR_H