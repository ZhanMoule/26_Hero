/**
* @file bsp_uart.c
 * @author Wenxin HU
 * @brief UART驱动模块
 * @version 0.1
 * @details UART,提供UART的初始化、发送、接收等功能
 * @date 2025-07-11
 * @copyright  Copyright (c) 2025 HDU—PHOENIX
 * @todo
 *       1. 阻塞模式更好的实现方式
 *       2. 增加不定长接收功能
 */

#include "bsp_uart.h"
#include "usart.h"
#include "FreeRTOS.h"
#include <string.h>
#include <string.h>

//预处理句柄，将HAL的句柄重定义为库使用的句柄
#if defined(UART_1)
static UART_HandleTypeDef* uart1_handle = &huart1; // UART1句柄
#else
static UART_HandleTypeDef* uart1_handle = NULL; // 如果没有定义UART_1,则句柄为NULL
#endif
#if defined(UART_3)
static UART_HandleTypeDef* uart3_handle = &huart3; // UART3句柄
#else
static UART_HandleTypeDef* uart3_handle = NULL; // 如果没有定义UART_3,则句柄为NULL
#endif
#if defined(UART_6)
static UART_HandleTypeDef* uart6_handle = &huart6; // UART6句柄
#else
static UART_HandleTypeDef* uart6_handle = NULL; // 如果没有定义UART_6,则句柄为NULL
#endif
#if defined(UART_2)
static UART_HandleTypeDef* uart2_handle = &huart2; // UART2句柄
#else
static UART_HandleTypeDef* uart2_handle = NULL; // 如果没有定义UART_2,则句柄为NULL
#endif
#if defined(UART_4)
static UART_HandleTypeDef* uart4_handle = &huart4; // UART4句柄
#else
static UART_HandleTypeDef* uart4_handle = NULL; // 如果没有定义UART_4,则句柄为NULL
#endif
#if defined(UART_5)
static UART_HandleTypeDef* uart5_handle = &huart5; // UART5句柄
#else
static UART_HandleTypeDef* uart5_handle = NULL; // 如果没有定义UART_5,则句柄为NULL
#endif
#if defined(UART_7)
static UART_HandleTypeDef* uart7_handle = &huart7; // UART7句柄
#else
static UART_HandleTypeDef* uart7_handle = NULL; // 如果没有定义UART_7,则句柄为NULL
#endif
#if defined(UART_8)
static UART_HandleTypeDef* uart8_handle = &huart8; // UART8句柄
#else
static UART_HandleTypeDef* uart8_handle = NULL; // 如果没有定义UART_8,则句柄为NULL
#endif
#if defined(UART_10)
static UART_HandleTypeDef* uart10_handle = &huart10; // UART10句柄
#else
static UART_HandleTypeDef* uart10_handle = NULL; // 如果没有定义UART_10,则句柄为NULL
#endif
#if defined(UART_9)
static UART_HandleTypeDef* uart9_handle = &huart9; // UART9句柄
#else
static UART_HandleTypeDef* uart9_handle = NULL; // 如果没有定义UART_9,则句柄为NULL
#endif


//串口并非总线，是一对一的形式，所以一个实例对应一个物理串口，并非CAN的一个物理CAN对应多个CAN示例
uint8_t id = 0; // 用于标识UART实例的唯一id,从0开始
UartInstance_s *uart_instance[UART_MAX_CNT]; // UART实例数组,用于存储UART注册的实例

/**
 * @brief 重定向的串口回调函数
 * @param huart 接收串口中断的串口句柄
 */
static void Uart_RxCallback(UART_HandleTypeDef *huart) {
    static UART_HandleTypeDef* _uart_handle;
    if (huart == NULL) {
        return;
    }
    if (uart1_handle != NULL && huart == uart1_handle) {
    } else if (uart3_handle != NULL && huart == uart3_handle) {
    } else if (uart6_handle != NULL && huart == uart6_handle) {
    } else if (uart2_handle != NULL && huart == uart2_handle) {
    } else if (uart4_handle != NULL && huart == uart4_handle) {
    } else if (uart5_handle != NULL && huart == uart5_handle) {
    } else if (uart7_handle != NULL && huart == uart7_handle) {
    } else if (uart8_handle != NULL && huart == uart8_handle) {
    } else if (uart9_handle != NULL && huart == uart9_handle) {
    } else if (uart10_handle != NULL && huart == uart10_handle) {
    } else {
        return; // 如果不是合法的串口句柄，返回NULL
    }
    for (uint8_t i = 0; i < id; i++) {
        if (uart_instance[i]->uart_handle == huart) { // 找到对应的UART实例
            if (uart_instance[i]->uart_module_callback != NULL) {
                uart_instance[i]->uart_module_callback(uart_instance[i]); // 调用回调函数处理接收到的数据
            }

            //重新打开接收中断
            HAL_UART_Receive_IT(huart, uart_instance[i]->rx_buff, uart_instance[i]->rx_len);

            break; // 找到后退出循环
        }
    }
}

UartInstance_s *Uart_Register(UartInitConfig_s *config) {
    if (config == NULL || id >= UART_MAX_CNT) {
        return NULL;//如果空间已满或者没有配置信息，返回NULL
    }

    //检验注册的串口是否合法（在开启的范围内）
    if (config->uart_handle == NULL) {
        return NULL;
    }
    if (uart1_handle != NULL && config->uart_handle == uart1_handle) {
    } else if (uart3_handle != NULL && config->uart_handle == uart3_handle) {
    } else if (uart6_handle != NULL && config->uart_handle == uart6_handle) {
    } else if (uart2_handle != NULL && config->uart_handle == uart2_handle) {
    } else if (uart4_handle != NULL && config->uart_handle == uart4_handle) {
    } else if (uart5_handle != NULL && config->uart_handle == uart5_handle) {
    } else if (uart7_handle != NULL && config->uart_handle == uart7_handle) {
    } else if (uart8_handle != NULL && config->uart_handle == uart8_handle) {
    } else if (uart9_handle != NULL && config->uart_handle == uart9_handle) {
    } else if (uart10_handle != NULL && config->uart_handle == uart10_handle) {
    } else {
        return NULL; // 如果不是合法的串口句柄，返回NULL
    }

    for (uint8_t i = 0; i < id ; i++) {
        if (uart_instance[i]->uart_handle == config->uart_handle) {
            return NULL; // 如果已经注册过了，返回NULL
        }
    }

    //开始分配空间 1用来区分实例和数组
    UartInstance_s *uart_instance1 = (UartInstance_s *)pvPortMalloc(sizeof(UartInstance_s));
    memset(uart_instance1, 0, sizeof(UartInstance_s));//清空空间

    uart_instance1->uart_handle = config->uart_handle; // 设置串口句柄
    uart_instance1->mode = config->mode; // 设置串口通讯模式
    uart_instance1->uart_module_callback = config->uart_module_callback; // 设置回调函数

    if (uart_instance1 == NULL || config->rx_len > UART_RX_BUFF_LEN) {
        vPortFree(uart_instance1); // 分配失败，释放内存
        return NULL;
    }

    uart_instance[id++] = uart_instance1; // 将实例添加到UART实例数组中

    if (uart_instance1->mode == UART_IT_MODE) {
        HAL_UART_Receive_IT(uart_instance1->uart_handle, uart_instance1->rx_buff, uart_instance1->rx_len); // 启用中断接收
    }else if (uart_instance1->mode == UART_DMA_MODE) {
        HAL_UART_Receive_DMA(uart_instance1->uart_handle, uart_instance1->rx_buff, uart_instance1->rx_len);
    }

    return uart_instance1;
}

bool Uart_Transmit(UartInstance_s *uart_instance, uint8_t *data) {
    if (uart_instance == NULL || data == NULL) {
        return false; // 无效的参数
    }
    memset(&uart_instance->tx_buff,0,sizeof(uart_instance->tx_buff));
    
    for (size_t i = 0; i < UART_RX_BUFF_LEN; ++i) {
        if (data[i] == '\n') {
            uart_instance->tx_len = i + 1;
            break;
        }
    }

    memcpy(uart_instance->tx_buff, data, uart_instance->tx_len);

    if (uart_instance->mode == UART_BLOCKING_MODE) {
        HAL_UART_Transmit(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len, HAL_MAX_DELAY);
    } else if (uart_instance->mode == UART_IT_MODE) {
        HAL_UART_Transmit_IT(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else if (uart_instance->mode == UART_DMA_MODE) {
        HAL_UART_Transmit_DMA(uart_instance->uart_handle, uart_instance->tx_buff, uart_instance->tx_len);
    } else {
        return false; // 不支持的模式
    }

    return true; // 发送成功
}
bool Uart_Blocking_Receive(UartInstance_s* uart_instance) {
    if (uart_instance->mode != UART_BLOCKING_MODE) {
        return false; // 如果不是阻塞模式，返回false
    }

    if (HAL_UART_Receive(uart_instance->uart_handle, uart_instance->rx_buff, uart_instance->rx_len, HAL_MAX_DELAY) == HAL_OK ) {
        return true;
    }else {
        return false; // 接收失败
    }
}



//将HAL库的接收回调重定向到库的接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    Uart_RxCallback(huart);
}
