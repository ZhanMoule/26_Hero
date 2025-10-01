#include "alg_pid.h"
#include <math.h>
#include "FreeRTOS.h"
#include <string.h>
#include <stdbool.h>

PidInstance_s *Pid_Register(PidInitConfig_s *config)
{
    if (config == NULL)
    {
        // 配置参数错误
        return NULL;
    }
    PidInstance_s *pid_instance = (PidInstance_s *)pvPortMalloc(sizeof(PidInstance_s));
    memset(pid_instance, 0, sizeof(PidInstance_s));
    if(pid_instance == NULL)
    {
        vPortFree(pid_instance);
        return NULL;
    }   
    pid_instance->kp = config->kp;
    pid_instance->ki = config->ki;
    pid_instance->kd = config->kd;
    pid_instance->kf = config->kf;
    pid_instance->angle_max = config->angle_max;
    if (config->i_max > config->out_max)
    {
        // 积分限幅大于输出限幅,
        pid_instance->i_max = config->out_max;
    }
    else
    {
        pid_instance->i_max = config->i_max;
    }
    pid_instance->out_max = config->out_max;
    pid_instance->dead_zone = config->dead_zone;
    if (config->i_variable_max == 0 || config->i_variable_min > config->i_variable_max)
    {
        // 没有变速积分
        pid_instance->i_variable_min = config->i_variable_min;
        pid_instance->i_variable_max = config->i_variable_min;
    }
    else
    {
        pid_instance->i_variable_min = config->i_variable_min;
        pid_instance->i_variable_max = config->i_variable_max;
    }
    pid_instance->d_first = config->d_first;
    return pid_instance;
}
static void Pid_Protect(PidInstance_s *pid)
{
    float half_angle = pid->angle_max / 2;
    if (pid->target[0] - pid->now[0] > half_angle)
    {
        pid->now[0] += pid->angle_max;
    }
    else if (pid->target[0] - pid->now[0] < -half_angle)
    {
        pid->now[0] -= pid->angle_max;
    }
}
/**
 * @brief 限制一个整数变量 value 在指定的最小值 min 和最大值 max 之间
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 计算结果
 */
static float Pid_Limit(float value, float min, float max)
{
    if (value < min)
        return min;
    else if (value > max)
        return max;
    else
        return value;
}

float Pid_Calculate(PidInstance_s *pid, float target, float now)
{
    // 前置工作
    pid->target[0] = target;
    pid->now[0] = now;
    // 过零保护
    if (pid->angle_max != 0){
        Pid_Protect(pid);
    }
    // 计算误差
    pid->err[0] = pid->target[0] - pid->now[0];
    // 判断死区
    if (fabsf(pid->err[0]) < pid->dead_zone){
        pid->err[0] = 0.0f;
    }
    // 计算比例项
    pid->p_out = pid->kp * pid->err[0];
    // 计算积分项
    float i_speed_ratio = 0; // 积分速度比率
    // 变速积分
    if (fabsf(pid->err[0]) < pid->i_variable_min || pid->i_variable_min == 0){
        i_speed_ratio = 1;
    }
    else if (fabsf(pid->err[0]) > pid->i_variable_min && fabsf(pid->err[0]) < pid->i_variable_max){
        i_speed_ratio = (pid->i_variable_max - fabs(pid->err[0])) / (pid->i_variable_max - pid->i_variable_min);
    }
    else if (fabsf(pid->err[0]) >= pid->i_variable_max){
        i_speed_ratio = 0;
    }
    pid->i_out += pid->ki * pid->err[0] * i_speed_ratio;
    // 积分限幅
    pid->i_out = Pid_Limit(pid->i_out, -pid->i_max, pid->i_max);
    // 计算微分项
    if (pid->d_first == 0){
        // 没有微分先行
        pid->d_out = pid->kd * (pid->err[0] - pid->err[1]);
    }
    else{
        // 微分先行
        pid->d_out = -pid->kd * (pid->now[0] - pid->now[1]);
    }
    // 计算前馈项
    pid->f_out = pid->kf * (pid->target[0] - pid->target[1]);
    // 计算输出
    pid->output = pid->p_out + pid->i_out + pid->d_out + pid->f_out;
    // 输出限幅
    pid->output = Pid_Limit(pid->output, -pid->out_max, pid->out_max);
    // 善后操作
    pid->err[1] = pid->err[0];
    pid->target[1] = pid->target[0];
    pid->now[1] = pid->now[0];
    return pid->output;
}

bool Pid_Reset(PidInstance_s *pid)
{
    if (pid == NULL)
        return false;
    // 重置PID实例
    pid->i_out = 0.0f;
    pid->err[0] = pid->err[1] = 0.0f;
    pid->target[0] = pid->target[1] = 0.0f;
    pid->now[0] = pid->now[1] = 0.0f;
    pid->output = 0.0f;
    return true;
}

bool Pid_Clean(PidInstance_s *pid)
{
    if (pid == NULL)
        return false;
    memset(pid, 0, sizeof(PidInstance_s)); // 把kp,ki等全部清零，在电机失联后调用
    return true;
}
