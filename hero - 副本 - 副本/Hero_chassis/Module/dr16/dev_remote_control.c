/**
 * @file dev_remote_control.c
 * @brief 遥控器的实现文件
 * @author CGH
 * @date 2025-07-07
 */
#include "dev_remote_control.h"
#include <string.h>  // 使用 memcpy 函数
#include <stdlib.h>   // 使用 abs 函数
#include "FreeRTOS.h"

#define REMOTE_CONTROL_FRAME_SIZE 18 // 遥控器接收的buffer大小
#define USART_T USART3 // 这里定义遥控器使用的串口实例,如果需要更改请修改此处
// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
RC_instance rc_ins[2];//储存现在和上一时刻的遥控器状态
#ifndef SUPER_MODE
uint16_t all_keys;
#endif 
static uint8_t time=0;//0表示当前，1表示过去

static struct {
    Keyboard_t *keyboard;
    RC_Handle_t *rchandle;
} *rc_data_ptr;


/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
static void sbus_receive(UartInstance_s *uart_instance){
    #ifndef SUPER_MODE
	RC_instance *rc_instance=uart_instance->id;
	//摇杆
	rc_instance->data.RChandle.rocker_rx = (((int16_t)uart_instance->rx_buff[0] | ((int16_t)uart_instance->rx_buff[1] << 8)) & 0x07FF)-1024;
	rc_instance->data.RChandle.rocker_ry = ((((int16_t)uart_instance->rx_buff[1] >> 3) | ((int16_t)uart_instance->rx_buff[2] << 5)) & 0x07FF)-1024;
	rc_instance->data.RChandle.rocker_lx = ((((int16_t)uart_instance->rx_buff[2] >> 6) | ((int16_t)uart_instance->rx_buff[3] << 2) |((int16_t)uart_instance->rx_buff[4] << 10)) & 0x07FF)-1024;
	rc_instance->data.RChandle.rocker_ly = ((((int16_t)uart_instance->rx_buff[4] >> 1) | ((int16_t)uart_instance->rx_buff[5]<<7)) & 0x07FF)-1024;
	//三位开关
	rc_instance->data.RChandle.switch_left = ((uart_instance->rx_buff[5] >> 4) & 0x000C) >> 2;
        test_data=((uart_instance->rx_buff[5] >> 4) & 0x000C) >> 2;
        test_data2=rc_instance->data.RChandle.switch_left;
	rc_instance->data.RChandle.switch_right = ((uart_instance->rx_buff[5] >> 4) & 0x0003);
	//鼠标
	rc_instance->data.RChandle.mouse_x = ((int16_t)uart_instance->rx_buff[6]) | ((int16_t)uart_instance->rx_buff[7] << 8);
	rc_instance->data.RChandle.mouse_y = ((int16_t)uart_instance->rx_buff[8]) | ((int16_t)uart_instance->rx_buff[9] << 8);
	rc_instance->data.RChandle.mouse_wheel = ((int16_t)uart_instance->rx_buff[10]) | ((int16_t)uart_instance->rx_buff[11] << 8); 
	rc_instance->data.RChandle.press_l = uart_instance->rx_buff[12];
	rc_instance->data.RChandle.press_r = uart_instance->rx_buff[13];
	//键盘
	all_keys = ((int16_t)uart_instance->rx_buff[14]) | ((int16_t)uart_instance->rx_buff[15] << 8);
	//wheel
	rc_instance->data.RChandle.rocker_roll = ((int16_t)uart_instance->rx_buff[16] | (int16_t)uart_instance->rx_buff[17] << 8) - 1024;
	//接收到错误的消息，全部消息置0
	if ((abs(rc_instance->data.RChandle.rocker_rx) > 660)||(abs(rc_instance->data.RChandle.rocker_ry) > 660)||(abs(rc_instance->data.RChandle.rocker_lx) > 660)||(abs(rc_instance->data.RChandle.rocker_ly) > 660))	{
			memset(&rc_instance->data,0,sizeof(rc_instance->data));
	}
    #endif
    #ifdef SUPER_MODE
    RC_instance *rc_instance=uart_instance->id;
    const uint8_t *sbus_buf = uart_instance->rx_buff; // 获取接收缓冲区
    //memcpy(&rc_instance->data, uart_instance->rx_buff, REMOTE_CONTROL_FRAME_SIZE);// 使用预定义的帧大小常量
    memcpy(&rc_instance->data, uart_instance->rx_buff, sizeof(rc_instance->data)); // 或者使用目标结构体的大小（更安全）
   if ((abs(rc_instance->data.RChandle.rocker_rx) > 660)||(abs(rc_instance->data.RChandle.rocker_ry) > 660)||(abs(rc_instance->data.RChandle.rocker_lx) > 660)||(abs(rc_instance->data.RChandle.rocker_ly) > 660))	{
	   memset(&rc_instance->data,0,sizeof(rc_instance->data));
   }

   //摇杆减去偏置值(有时间改成用或操作符号来减去偏置值)
   rc_instance->data.RChandle.rocker_lx -= 1024;
   rc_instance->data.RChandle.rocker_ly -= 1024;
   rc_instance->data.RChandle.rocker_rx -= 1024;
   rc_instance->data.RChandle.rocker_ry -= 1024;
   #endif
}

/**
 * @brief 初始化遥控器
 * @param rc 遥控器实例指针
 * @param uart_config UART初始化配置结构体指针
 */
RC_instance *Remote_Register(void){
    // 注册UART实例
    RC_instance *rc_instance = (RC_instance *)pvPortMalloc(sizeof(RC_instance));
    if (rc_instance == NULL) {
        return NULL; // 内存分配失败
    }
    memset(rc_instance, 0, sizeof(RC_instance)); // 清空内存
    
    // 配置UART实例 - 分配内存
    UartInitConfig_s *uart_config = (UartInitConfig_s *)pvPortMalloc(sizeof(UartInitConfig_s));
    if (uart_config == NULL) {
        vPortFree(rc_instance);
        return NULL;
    }
    
    uart_config->uart_handle = &huart3;
    uart_config->mode = UART_DMA_MODE;
    uart_config->rx_len = REMOTE_CONTROL_FRAME_SIZE;
    uart_config->uart_module_callback = sbus_receive;
    uart_config->id = rc_instance;

    rc_instance->uart_instance = Uart_Register(uart_config); // 注册UART实例
    
    // 检查 uart_instance 是否创建成功
    if (rc_instance->uart_instance == NULL) {       
	vPortFree(rc_instance);
        return NULL;
    }  
    return rc_instance; // 返回遥控器实例指针
}