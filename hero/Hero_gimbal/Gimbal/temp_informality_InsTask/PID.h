#ifndef __PID_H_
#define __PID_H_

#include "main.h"
#include "stm32f4xx.h"

typedef struct
{
  float kp;
  float ki;
  float kd;

  float i_max;
  float out_max;
  
  float ref; //设定
  float fdb; //反馈
  float err[2];

  float p_out;
  float i_out;
  float d_out;
  float output;
}PID_struct_t;

void PID_init(PID_struct_t *PID,float kp,float ki,float kd,float i_max,float out_max);
float PID_Calc_Angle(PID_struct_t *PID, float ref, float fdb);//PID运算函数（目标，实际)
float PID_Calc_Speed(PID_struct_t *PID, float ref, float fdb);//PID运算函数（目标，实际）
float PID_Calc_Ink(PID_struct_t *PID, float ref, float fdb);//PID运算函数（目标，实际）
//float PID_Calc_Follow(PID_struct_t *PID, float ref, float fdb);//PID运算函数（目标，实际）
int Limit_Min_Max(int value,int min,int max);

#endif


