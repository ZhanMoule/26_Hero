/**
 * @file       bsp_buzzer.h
 * @brief      蜂鸣器驱动模块头文件
 * @details    提供蜂鸣器的注册、初始化、音符播放和音乐播放功能
 * @author     Zheng PengZe (2579523281@qq.com)
 * @version    V0.1.0
 * @date       2025-07-12
 * @copyright  Copyright (c) 2024-2025 Phoenix Team
 */
 
#ifndef _BSP_BUZZER_H_
#define _BSP_BUZZER_H_

#include "main.h"
#include <stdint.h>

/**
 * @brief 蜂鸣器实例结构体
 * @details 存储蜂鸣器控制所需的定时器和通道信息
 */
typedef struct 
{
    TIM_HandleTypeDef *htim;  // 定时器句柄，用于PWM控制
    uint32_t channel;         // PWM通道，指定定时器的输出通道
} BuzzerInstance_s;

/**
 * @brief 蜂鸣器初始化配置结构体
 * @details 存储创建蜂鸣器实例所需的配置信息
 */
typedef struct
{
    TIM_HandleTypeDef *htim;  // 定时器句柄，用于PWM控制
    uint32_t channel;         // PWM通道，指定定时器的输出通道
} Buzzer_Init_Config_s;

/**
 * @brief  蜂鸣器注册函数
 * @param  config: 蜂鸣器配置结构体，包含定时器和PWM通道信息
 * @return 蜂鸣器实例指针，失败返回NULL
 * @note   必须先调用此函数注册，再调用buzzer_init初始化
 * @warning 返回值必须检查，若为NULL表示内存分配失败
 */
BuzzerInstance_s *buzzer_register(Buzzer_Init_Config_s *config);

/**
 * @brief  蜂鸣器初始化
 * @param  buzzer: 蜂鸣器实例，由buzzer_register函数创建
 * @note   使用前必须先调用buzzer_register注册蜂鸣器
 * @details 启动定时器和PWM输出，配置初始状态
 */
void buzzer_init(BuzzerInstance_s *buzzer);

/**
 * @brief  蜂鸣器关闭
 * @param  buzzer: 蜂鸣器实例
 * @details 通过设置PWM比较值为0停止蜂鸣器发声
 */
void buzzer_off(BuzzerInstance_s *buzzer);

/**
 * @brief  蜂鸣器按频率播放
 * @param  buzzer: 蜂鸣器实例
 * @param  frequency: 频率值(Hz)，直接指定发声频率
 * @param  time: 持续时间(拍)，将与BEAT_DELAY_TIME相乘得到毫秒数
 * @param  vol: 音量大小，控制PWM占空比
 * @warning time与BEAT_DELAY_TIME相乘必须为整数，否则会有精度损失
 */
void buzzer_play_frequency(BuzzerInstance_s *buzzer, uint32_t frequency, float time, uint16_t vol);

/**
 * @brief  蜂鸣器按音符播放
 * @param  buzzer: 蜂鸣器实例
 * @param  note: 音符(1-7，0为休止符)，对应do-si七个基本音
 * @param  octave: 八度(0-2)，控制音高，0为低音，1为中音，2为高音
 * @param  time: 持续拍子，将与BEAT_DELAY_TIME相乘得到毫秒数
 * @param  vol: 音量大小，控制PWM占空比
 * @warning time与BEAT_DELAY_TIME相乘必须为整数，否则会有精度损失
 * @details 内部将音符和八度转换为对应的频率，然后播放
 */
void buzzer_play_note(BuzzerInstance_s *buzzer, uint16_t note, uint16_t octave, float time, uint16_t vol);

/**
 * @brief  蜂鸣器播放乐谱
 * @param  buzzer: 蜂鸣器实例
 * @param  sheet: 音符数组[音符,八度,节拍]，音符和八度为整数，节拍为浮点数
 * @param  len: 序列长度，数组的行数
 * @param  vol: 音量大小，控制PWM占空比
 * @warning 节拍值与BEAT_DELAY_TIME相乘必须为整数，否则会有精度损失
 * @details 按序播放sheet数组中的每个音符，实现音乐播放
 */
void buzzer_play_sheet(BuzzerInstance_s *buzzer, float (*sheet)[3], uint16_t len, uint16_t vol);

#endif // _BSP_BUZZER_H_
