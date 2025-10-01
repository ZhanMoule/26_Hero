# bsp_can

## 概述
`bsp_can` 是一个用于处理 CAN 总线通信的库，提供了对 CAN 硬件的封装和操作接口。该库旨在简化 CAN 总线的配置和使用，使得开发者可以更方便地实现 CAN 通信功能。

## 环境
- 硬件平台：STM32
- 相关库：CMSIS, HAL, FreeRTOS

## 软件要求
- `bsp_can`可以选择使用log辅助调试
- 使用 `bsp_can` 需要在 CubeMX 中开启中断并配置好正确的 CAN 参数。

## 使用教程
1. 正确配置cubemx
2. 在 robot_config.h 中关于配置 CAN 相关参数，需要与cubemx配置完全一致
   ```c
    /* CAN 初始化配置选项 */
    
    /* 选择 CAN 类型 */
    #define USER_CAN_STANDARD
    
    /* 选择 can1 or can2 */
    #define USER_CAN1
    #define USER_CAN2
    
    /* 选择 can1 fifo 0 or 1 */
    #define USER_CAN1_FIFO_0
    #define USER_CAN1_FIFO_1
    
    /* 选择 can2 fifo 0 or 1 */
    #define USER_CAN2_FIFO_0
    #define USER_CAN2_FIFO_1
    
    /* 选择过滤器模式 */
    #define USER_CAN_FILTER_MASK_MODE
    #define USER_CAN_FILTER_LIST_MODE
   ```
3. 配置好can_config后，直接注册实例即可，第一次注册实例的时候会初始化can
4. 发送数据前需要先将发送数据赋值给实例的tx_buff

## 示例
    ```c
    include "bsp_can.h"
    CanInstance_s *test;
    CanInitConfig_s test_config = {
        .topic_name = "test",
        .can_number = 1,
        .tx_id = 0x200,
        .rx_id = 0x001,
        .can_module_callback = test_decode,
        .id = NULL
    };
    uint8_t tx_buf[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

    void Can_Task(void const * argument){
        test = Can_Register(&test_config);
        if(test == NULL){
            Log_Error("CAN Register Failed!");
            while(1);
        }
        tset->tx_buff = tx_buf;
        for(;;){
            test_status = Can_Transmit(test);
            osDelay(1);
        }
    }
```