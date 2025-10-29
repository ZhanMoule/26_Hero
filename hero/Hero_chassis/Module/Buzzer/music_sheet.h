/**
 * @file       music_sheet.h
 * @brief      音乐乐谱集合头文件
 * @details    提供各种预定义的音乐乐谱数组
 * @author     Zheng PengZe (2579523281@qq.com)
 * @version    V0.1.0
 * @date       2025-07-11
 * @copyright  Copyright (c) 2024-2025 Phoenix Team
 */

#ifndef _MUSIC_SHEET_H_
#define _MUSIC_SHEET_H_

#include "stdint.h"

/* 新闻联播主题曲乐谱 */
extern float news_theme[][3];
extern uint16_t news_theme_len;

/* 猪猪侠主题曲乐谱 */
extern float pig_hero_theme[][3];
extern uint16_t pig_hero_theme_len;

/* 两只老虎乐谱 */
extern float two_tigers[][3];
extern uint16_t two_tigers_len;

/* 小星星乐谱 */
extern float twinkle_star[][3];
extern uint16_t twinkle_star_len;

/* 生日快乐歌乐谱 */
extern float happy_birthday[][3];
extern uint16_t happy_birthday_len;

/* 欢乐颂乐谱 */
extern float ode_to_joy[][3];
extern uint16_t ode_to_joy_len;

/* 茉莉花乐谱 */
extern float jasmine_flower[][3];
extern uint16_t jasmine_flower_len;

/* 爱的罗曼史乐谱 */
extern float romance_de_amor[][3];
extern uint16_t romance_de_amor_len;

#endif // _MUSIC_SHEET_H_
