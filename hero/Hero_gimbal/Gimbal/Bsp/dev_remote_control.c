/**
*   @file dev_dr16.c
*   @brief
*   @author Wenxin HU
*   @date 25-8-6
*   @version 1.0
*   @todo 键盘数据处理
*         异常数据处理
*         适配其他开发板
*/
#include "dev_remote_control.h"
#include "string.h"

static void Dr16_Receive(UartInstance_s *uart_instance) {
    uint8_t data[REMOTE_CONTROL_FRAME_SIZE];
    Dr16Instance_s* dr16_instance = (Dr16Instance_s *)uart_instance->id; // 获取遥控器实例指针
    memcpy(data,uart_instance->rx_buff,REMOTE_CONTROL_FRAME_SIZE);

    //接收数据校验,数据异常则重新开启并清空缓存区
    if (((data[5] >> 4) & 0x000C) >> 2 == 0) {
        HAL_UART_DMAStop(dr16_instance->uart_instance->uart_handle);
        memset(uart_instance->rx_buff,0,uart_instance->rx_len);
        HAL_UART_Receive_DMA(uart_instance->uart_handle, uart_instance->rx_buff, uart_instance->rx_len); // 重新启动DMA接收
    }

    dr16_instance->dr16_handle.ch0 = (((int16_t)data[0] | ((int16_t)data[1] << 8)) & 0x07FF)-1024;;
    dr16_instance->dr16_handle.ch1 = ((((int16_t)data[1] >> 3) | ((int16_t)data[2] << 5)) & 0x07FF)-1024;
    dr16_instance->dr16_handle.ch2 = ((((int16_t)data[2] >> 6) | ((int16_t)data[3] << 2) |((int16_t)data[4] << 10)) & 0x07FF)-1024;
    dr16_instance->dr16_handle.ch3 = ((((int16_t)data[4] >> 1) | ((int16_t)data[5]<<7)) & 0x07FF)-1024;
    dr16_instance->dr16_handle.s1 = ((data[5] >> 4) & 0x000C) >> 2; // 左侧开关
    dr16_instance->dr16_handle.s2 = ((data[5] >> 4) & 0x0003); // 右侧开关
    dr16_instance->dr16_handle.wheel = ((int16_t)data[16] | (int16_t)data[17] << 8) - 1024;// 摇杆滚轮

    dr16_instance->dr16_mouse.x = ((int16_t)data[6]) | ((int16_t)data[7] << 8);//鼠标x
    dr16_instance->dr16_mouse.y = ((int16_t)data[8]) | ((int16_t)data[9] << 8);//鼠标y
    dr16_instance->dr16_mouse.z = ((int16_t)data[10]) | ((int16_t)data[11] << 8); //鼠标滚轮
    dr16_instance->dr16_mouse.press_l = data[12];//鼠标左键
    dr16_instance->dr16_mouse.press_r = data[13];//鼠标右键

    dr16_instance->dr16_keyboard.keys = ((int16_t)data[14]) | ((int16_t)data[15] << 8);// 键盘按键状态

}

Dr16Instance_s *Dr16_Register(UART_HandleTypeDef *husart) {
    // 注册UART实例
    Dr16Instance_s *dr16_instance = (Dr16Instance_s *)user_malloc(sizeof(Dr16Instance_s));
    if (dr16_instance == NULL) {
        return NULL; // 内存分配失败
    }
    memset(dr16_instance, 0, sizeof(Dr16Instance_s)); // 清空内存

    UartInitConfig_s uart_config;
    // 配置UART实例
    uart_config.uart_handle = husart;
    uart_config.mode = UART_DMA_MODE;
    uart_config.rx_len = REMOTE_CONTROL_FRAME_SIZE;
    uart_config.uart_module_callback = Dr16_Receive;
    uart_config.id = dr16_instance;
    dr16_instance->uart_instance = NULL;

    dr16_instance->uart_instance = Uart_Register(&uart_config); // 注册UART实例

    if (dr16_instance->uart_instance == NULL) {
        user_free(dr16_instance);
        return NULL;
    }
    return dr16_instance;
}