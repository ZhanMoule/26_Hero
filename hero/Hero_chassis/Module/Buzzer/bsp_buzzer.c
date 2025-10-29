/**
 * @file       bsp_buzzer.c
 * @brief      蜂鸣器驱动模块源文件
 * @details    实现蜂鸣器的注册、初始化、音符播放和音乐播放功能
 * @author     Zheng PengZe (2579523281@qq.com)
 * @version    V0.1.0
 * @date       2025-07-11
 * @copyright  Copyright (c) 2024-2025 Phoenix Team
 */

#include "bsp_buzzer.h"
#include "main.h"
#include "FreeRTOS.h"
#include <string.h>

#define BEAT_DELAY_TIME    500        // 一拍持续时间(ms)
#define TIMER_CLOCK_FREQ   84000000   // 定时器时钟频率(Hz)，根据实际情况修改

BuzzerInstance_s *buzzer_register(Buzzer_Init_Config_s *config)
{
    BuzzerInstance_s *buzzer = (BuzzerInstance_s *)pvPortMalloc(sizeof(BuzzerInstance_s)); // 分配空间
    if (buzzer == NULL)
    {
        return NULL;
    }
    
    memset(buzzer, 0, sizeof(BuzzerInstance_s)); // 分配的空间未必是0,所以要先清空
    
    buzzer->htim = config->htim;
    buzzer->channel = config->channel;

    if(buzzer == NULL){
        vPortFree(buzzer);// 释放空间
        return NULL;
    }
    
    return buzzer;
}

void buzzer_init(BuzzerInstance_s *buzzer)
{
    HAL_TIM_Base_Start(buzzer->htim);
    HAL_TIM_PWM_Start(buzzer->htim, buzzer->channel);
    buzzer_off(buzzer);
}

/**
 * @brief  蜂鸣器开启（内部函数）
 * @param  buzzer: 蜂鸣器实例
 * @param  psc: 定时器预分频值
 * @param  pwm: PWM比较值
 */
static void buzzer_on(BuzzerInstance_s *buzzer, uint16_t psc, uint16_t pwm)
{
    __HAL_TIM_PRESCALER(buzzer->htim, psc);
    __HAL_TIM_SetCompare(buzzer->htim, buzzer->channel, pwm);
}

void buzzer_off(BuzzerInstance_s *buzzer)
{
    __HAL_TIM_SetCompare(buzzer->htim, buzzer->channel, 0);
}

/**
 * @brief  蜂鸣器延时函数（内部使用）
 * @param  time: 延时时间（以拍为单位）
 */
static void buzzer_delay(float time)
{
    HAL_Delay(time * BEAT_DELAY_TIME);
}

void buzzer_play_frequency(BuzzerInstance_s *buzzer, uint32_t frequency, float time, uint16_t vol)
{
    uint32_t arr_psc = (uint32_t)(TIMER_CLOCK_FREQ / frequency);
    uint16_t psc = arr_psc / 65535;
    uint16_t arr = arr_psc / (psc + 1) - 1;

    __HAL_TIM_SET_AUTORELOAD(buzzer->htim, arr);
    __HAL_TIM_SET_PRESCALER(buzzer->htim, psc);
    __HAL_TIM_SetCompare(buzzer->htim, buzzer->channel, vol);

    HAL_Delay(time * BEAT_DELAY_TIME);
    buzzer_off(buzzer);
}

void buzzer_play_note(BuzzerInstance_s *buzzer, uint16_t note, uint16_t octave, float time, uint16_t vol)
{
    const float NOTE_FREQ[] = {
        261.63f, 293.66f, 329.63f, 349.23f, 392.00f, 440.00f, 493.88f
    };
    
    if (note == 0)  // 休止符
    {
        buzzer_off(buzzer);
        buzzer_delay(time);
        return;
    }
    
    float target_freq = NOTE_FREQ[note - 1] * (1 << octave);
    uint32_t arr_psc = (uint32_t)(TIMER_CLOCK_FREQ / target_freq);
    uint16_t psc = arr_psc / 65535;
    uint16_t arr = arr_psc / (psc + 1) - 1;
    
    __HAL_TIM_SET_AUTORELOAD(buzzer->htim, arr);
    __HAL_TIM_SET_PRESCALER(buzzer->htim, psc);
    __HAL_TIM_SetCompare(buzzer->htim, buzzer->channel, vol);
    
    HAL_Delay(time * BEAT_DELAY_TIME);
    buzzer_off(buzzer);
}

void buzzer_play_sheet(BuzzerInstance_s *buzzer, float (*sheet)[3], uint16_t len, uint16_t vol)
{
    // 播放音符序列
    // sheet是二维数组，每行包含[音符, 八度, 节拍]
    // 音符范围1-7，八度为0-2，节拍为拍子数(将被转换为时间)
    
    for (uint16_t i = 0; i < len; i++)
    {
        uint16_t note = (uint16_t)sheet[i][0];  // 音符，转为整数
        uint16_t octave = (uint16_t)sheet[i][1]; // 八度，转为整数
        float time = sheet[i][2];                // 节拍值，保持浮点数
        
        if (note == 0)  // 休止符
        {
            buzzer_off(buzzer);
            buzzer_delay(time);
        }
        else
        {
            buzzer_play_note(buzzer, note, octave, time, vol);
        }
    }
}
