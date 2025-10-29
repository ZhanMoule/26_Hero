/**
*   @file dev_dr16.h
*   @brief 
*   @author Wenxin HU
*   @date 25-8-6
*   @version 1.0
*   @note
*/
#ifndef __DEV_DR16_H__
#define __DEV_DR16_H__

#include "bsp_uart.h"
#include "robot_config.h"

#define REMOTE_CONTROL_FRAME_SIZE 18

#pragma pack(1)
typedef union {
    struct{
        uint16_t w     : 1;
        uint16_t s     : 1;
        uint16_t a     : 1;
        uint16_t d     : 1;
        uint16_t shift : 1;
        uint16_t ctrl  : 1;
        uint16_t q     : 1;
        uint16_t e     : 1;
        uint16_t r     : 1;
        uint16_t f     : 1;
        uint16_t g     : 1;
        uint16_t z     : 1;
        uint16_t x     : 1;
        uint16_t c     : 1;
        uint16_t v     : 1;
        uint16_t b     : 1;
    };
    uint16_t keys;  // 用于整体读取所有按键状态
} Dr16KeyboardData_t;

typedef struct {
    //鼠标
    int16_t x;
    int16_t y;
    int16_t z;//鼠标滚轮
    uint8_t press_l;
    uint8_t press_r;
} Dr16MouseData_t;

typedef struct
{   //摇杆
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    //三位开关
    int8_t s1;
    int8_t s2;
    // 拨杆
    int16_t wheel;
}Dr16HandleData_t;

typedef struct {
    UartInstance_s *uart_instance;      // UART实例指针

    Dr16KeyboardData_t dr16_keyboard;   //键盘数据
    Dr16MouseData_t dr16_mouse;         //鼠标数据
    Dr16HandleData_t dr16_handle;       //摇杆数据
}Dr16Instance_s;

#pragma pack()  // 恢复默认对齐

/**
 * @file dev_dr16.h
 * @brief 初始化遥控器
 * @return Dr16Instance_s* 指向遥控器实例的指针
 * @note 该函数会分配内存并初始化遥控器实例
 * @date 2025-07-17
 */
Dr16Instance_s *Dr16_Register(UART_HandleTypeDef *husart) ;

#endif //DEV_DR16_H