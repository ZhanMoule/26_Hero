# bsp_fdcan

## 概述
`bsp_fdcan` 是一个用于处理FDCAN的库，提供了对FDCAN硬件的封装和操作接口。该库旨在简化FDCAN的配置和使用，使得开发者可以更方便地实现CAN通信。

## 环境
- 硬件平台：STM32
- 相关库：CMSIS, HAL, FreeRTOS

## 注意事项
- 使用 `bsp_fdcan` 前需要在 `robot_config.h` 文件中设置好正确的参数,且请确保和CUBEMX中的配置完全一致
- `bsp_fdcan` 需要包含 `bsp_log` 和`robot_config.h`才可正常使用，否则会编译报错
- `bsp_fdcan` 已经加入了log功能，设计配置失败等情况请查看log输出(具体配置请查看 `bsp_log` 相关说明)
- 所有以 `user_` 开头的变量或函数等均在`robot_config.h`中定义

## 使用方法
1. 在CUBEMX中配置好FDCAN的相关参数
2. 将本仓库添加到工程
3. 在 `robot_config.h` 中配置好相关参数
4. 配置好 `bsp_log` 等相关文件
5. 在需要使用的地方包含 `bsp_fdcan.h` 头文件
6. 定义 `CanInitConfig_s` 结构体并初始化
7. 调用 `Can_Register` 函数注册FDCAN实例
8. 可以使用 `Can_Transmit` 函数发送数据
9. 收到数据自动进中断处理把数据拿到rx_buffer中，并回调用户函数
10. 建议使用log功能辅助调试

## 使用示例
```C
CanInstance_s *test;
CanInitConfig_s test_config = {
    .topic_name = "test",
    .can_channel = 1,
    .tx_id = 0x200,
    .rx_id = 0x001,
    .can_module_callback = test_decode,
    .id = NULL
};

void CAN_Task(void const * argument){
    test = Can_Register(&test_config);
    test->tx_buff = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
    while(1){
        Can_Transmit(test, tx_buf);
        osDelay(1);
    }
}
```
