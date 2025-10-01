#ifndef DEV_REMOTE_CONTROL_H
#define DEV_REMOTE_CONTROL_H
#include "stdint.h"
#include "stdbool.h"
#include "stddef.h" // for size_t
#include "bsp_uart.h" // 假设有一个USART实例的头文件
#include "usart.h"


#define SUPER_MODE 0 // 超级模式
//可能需要测试代码来查看实际内存情况，但是（感觉不如）实践出真知~
#ifndef SUPER_MODE
/* 遥控器数据参数定义 */

#define RC_CH_VALUE_MIN    ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX    ((uint16_t)1684)

#define    KEY_PRESSED_OFFSET_W        ((uint16_t)0x01<<0)
#define    KEY_PRESSED_OFFSET_S        ((uint16_t)0x01<<1)
#define    KEY_PRESSED_OFFSET_A        ((uint16_t)0x01<<2)
#define    KEY_PRESSED_OFFSET_D        ((uint16_t)0x01<<3)
#define    KEY_PRESSED_OFFSET_SHIFT    ((uint16_t)0x01<<4)
#define    KEY_PRESSED_OFFSET_CTRL     ((uint16_t)0x01<<5)
#define    KEY_PRESSED_OFFSET_Q        ((uint16_t)0x01<<6)
#define    KEY_PRESSED_OFFSET_E        ((uint16_t)0x01<<7)
#define    KEY_PRESSED_OFFSET_R        ((uint16_t)0x01<<8)
#define    KEY_PRESSED_OFFSET_F        ((uint16_t)0x01<<9)
#define    KEY_PRESSED_OFFSET_G        ((uint16_t)0x01<<10)
#define    KEY_PRESSED_OFFSET_Z        ((uint16_t)0x01<<11)
#define    KEY_PRESSED_OFFSET_X        ((uint16_t)0x01<<12)
#define    KEY_PRESSED_OFFSET_C        ((uint16_t)0x01<<13)
#define    KEY_PRESSED_OFFSET_V        ((uint16_t)0x01<<14)
#define    KEY_PRESSED_OFFSET_B        ((uint16_t)0x01<<15)

/* 检测键盘按键状态 
   若对应按键被按下，则逻辑表达式的值为1，否则为0 */
#define    IF_KEY_PRESSED         (  all_keys  )
#define    IF_KEY_PRESSED_W       ( (all_keys & KEY_PRESSED_OFFSET_W)    != 0 )
#define    IF_KEY_PRESSED_S       ( (all_keys & KEY_PRESSED_OFFSET_S)    != 0 )
#define    IF_KEY_PRESSED_A       ( (all_keys & KEY_PRESSED_OFFSET_A)    != 0 )
#define    IF_KEY_PRESSED_D       ( (all_keys & KEY_PRESSED_OFFSET_D)    != 0 )
	#define    IF_KEY_PRESSED_Q       ( (all_keys & KEY_PRESSED_OFFSET_Q)    != 0 )
	#define    IF_KEY_PRESSED_E       ( (all_keys & KEY_PRESSED_OFFSET_E)    != 0 )
	#define    IF_KEY_PRESSED_G       ( (all_keys & KEY_PRESSED_OFFSET_G)    != 0 )
	#define    IF_KEY_PRESSED_X       ( (all_keys & KEY_PRESSED_OFFSET_X)    != 0 )
	#define    IF_KEY_PRESSED_Z       ( (all_keys & KEY_PRESSED_OFFSET_Z)    != 0 )
	#define    IF_KEY_PRESSED_C       ( (all_keys & KEY_PRESSED_OFFSET_C)    != 0 )
	#define    IF_KEY_PRESSED_B       ( (all_keys & KEY_PRESSED_OFFSET_B)    != 0 )
	#define    IF_KEY_PRESSED_V       ( (all_keys & KEY_PRESSED_OFFSET_V)    != 0 )
	#define    IF_KEY_PRESSED_F       ( (all_keys & KEY_PRESSED_OFFSET_F)    != 0 )
	#define    IF_KEY_PRESSED_R       ( (all_keys & KEY_PRESSED_OFFSET_R)    != 0 )
	#define    IF_KEY_PRESSED_CTRL    ( (all_keys & KEY_PRESSED_OFFSET_CTRL) != 0 )
	#define    IF_KEY_PRESSED_SHIFT   ( (all_keys & KEY_PRESSED_OFFSET_SHIFT) != 0 )

#endif






/* 键盘状态位域结构体 */
#pragma pack(1)
#ifdef SUPER_MODE

typedef union {
    struct {
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
        uint16_t rocker_roll   : 16; // 摇杆滚轮
    } key_bits;
    uint32_t keys;  // 用于整体读取所有按键状态
} Keyboard_t;

#else
// 重新定义的键盘结构体 - 无位域
typedef struct {
    uint8_t w;
    uint8_t s;
    uint8_t a;
    uint8_t d;
    uint8_t shift;
    uint8_t ctrl;
    uint8_t q;
    uint8_t e;
    uint8_t r;
    uint8_t f;
    uint8_t g;
    uint8_t z;
    uint8_t x;
    uint8_t c;
    uint8_t v;
    uint8_t b;
    
} Keyboard_t;
#endif // SUPER_MODE
#ifdef SUPER_MODE

typedef struct {
    int16_t rocker_rx       : 11; // 右水平
    int16_t rocker_ry       : 11; // 右竖直
    int16_t rocker_lx       : 11; // 左水平
    int16_t rocker_ly       : 11; // 左竖直
    uint8_t switch_left     : 2;  // 左侧开关
    uint8_t switch_right    : 2;  // 右侧开关
   

    
    int16_t mouse_x         : 16; // 鼠标X轴
    int16_t mouse_y         : 16; // 鼠标Y轴
    int16_t mouse_wheel     : 16; // 鼠标滚轮(建议看看大疆写的是什么几把玩意)
    uint8_t press_l         : 8;  // 鼠标左键（0/1）
    uint8_t press_r         : 8;  // 鼠标右键（0/1）
} RC_Handle_t;

#else
// 重新定义的遥控器手柄结构体 - 无位域
typedef struct {
    int16_t rocker_rx;      // 右水平
    int16_t rocker_ry;      // 右竖直
    int16_t rocker_lx;      // 左水平
    int16_t rocker_ly;      // 左竖直
    uint8_t switch_left;    // 左侧开关
    uint8_t switch_right;   // 右侧开关
    uint16_t rocker_roll; // 摇杆滚轮

    int16_t mouse_x;        // 鼠标X轴
    int16_t mouse_y;        // 鼠标Y轴
    int16_t mouse_wheel;    // 鼠标滚轮
    uint8_t press_l;        // 鼠标左键（0/1）
    uint8_t press_r;        // 鼠标右键（0/1）
} RC_Handle_t;
#endif // SUPER_MODE

//#pragma pack()  // 重置为默认对齐方式（可能导致keyboard和rchandel的内存位置不紧密，暂时注释）
typedef struct{
    bool is_connected; // 是否连接
    //const uint8_t *sbus_buf; // SBUS接收缓冲区
    UartInstance_s *uart_instance; // UART实例指针
    
    struct {
        RC_Handle_t RChandle; // 摇杆控制器数据
        Keyboard_t Keyboard; // 键盘控制器数据
    } data;
} RC_instance;


#pragma pack()  // 恢复默认对齐
/**
 * @brief 初始化遥控器
 * 
 * @return RC_instance* 指向遥控器实例的指针
 * @note 该函数会分配内存并初始化遥控器实例
 * @date 2025-07-17
 */
RC_instance *Remote_Register(void);

/**
 * @brief 遥控器UART接收回调函数
 * @param uart_instance UART实例指针
 */
void RemoteControlRxCallback(UartInstance_s *uart_instance);
#endif // DEV_REMOTE_CONTROL_H

/*************************遥控器***************************
 *                                                            *
 *   -----------------------------------------------------    *
 *   |     (上-1)                               (上-1)   |    *
 *   |SW_L|(中-3)                          SW_R|(中-3)   |    *
 *   |     (下-2)                               (下-2)   |    *
 *   |                                                   |    *
 *   |    | ^ |                                | ^ |     |    *
 *   |    | 3 |左摇杆                     右摇杆| 1 |     |    *
 *   | ---     ---                          ---     ---  |    *
 *   |<           2>                       <           0>|    *
 *   | ---     ---                          ---     ---  |    *
 *   |    |   |                                |   |     |    *
 *   |    |   |                                |   |     |    *
 *   |                                                   |    *
 *   -----------------------------------------------------    *
 *                                                            *
 * 
 * ***********************************************************/
/***********************模式选择*******************************
 * ******s[1]**********s[0]***********************************
 *        1              1    跟随模式，底盘正常运动，云台yaw  轴和pitch轴角度均为0保持不动即可
 *        1              3    普通模式，底盘正常运动，云台yaw轴和pitch轴可变，按照云台方向行进
 *        1              2    发射模式，底盘不动，左摇杆  向下控制拨弹，摩擦轮启动，右摇杆控
 *        3              1    陀螺模式，云台保持朝向不变
 *        3              2    舵机测试
 *        3              3    云台失能
 * ***********************************************************/

