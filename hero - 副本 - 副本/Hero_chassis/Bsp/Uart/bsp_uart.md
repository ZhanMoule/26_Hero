## 1. UART模块使用注意事项
1. 只允许开启C板的UART1 UART6 和UART3（DBUS）
2. 请宏定义每一个使用的UART
```c++
// #define UART_1 //如果使用UART1,需要定义此宏
#define UART_3 //如果使用UART2,需要定义此宏
#define UART_6 //如果使用UART2,需要定义此宏
```
3. 请为每一个UART都打开中断，**如果你打开了DMA中断接收也要打开！！！**，尽量不要使用阻塞模式收发数据
4. DMA模式请一定打开循环模式（如DR16接收数据），库内只会在注册时开启一次DMA接收

## 2.开放接口
1. 实例本身的回调函数：
```C
typedef struct _ {
    UART_HandleTypeDef* uart_handle; // 串口句柄
    uint8_t tx_buff[50];             // 发送缓存
    uint8_t rx_buff[50];             // 接收缓存
    uint16_t tx_len;                 // 发送长度
    uint16_t rx_len;                 // 接收长度
    UartMode_e mode;                 // 串口通讯模式

    void (*uart_module_callback)(struct _*); // 接收的回调函数,用于解析接收到的数据
    void* id;                                //使用uart外设的模块指针(即id指向的模块拥有此uart实例,是父子关系)
} UartInstance_s;
```
用户可用通过自定义回调函数来处理接收到的数据，回调函数的参数为UartInstance_s指针。
2. 实例注册函数：**请在注册的时候就确定uart的工作模式和收发长度！**
```C
/**
 * @file bsp_uart.h
 * @brief UART实例注册函数
 * @param config UART初始化配置结构体指针
 * @return instance指针--注册成功   NULL--注册失败
 * @date 2025-06-30
 */
UartInstance_s* Uart_Register(UartInitConfig_s* config);
```
3. 发送数据函数
```C
/**
 * @file bsp_uart.h
 * @brief UART发送数据函数
 * @param uart_instance UART实例指针
 * @param data 发送数据指针
 * @return true--发送成功   false--发送失败
 * @date 2025-06-30
 */
bool Uart_Transmit(UartInstance_s* uart_instance, uint8_t* data);
```
4. 阻塞式接收函数（只允许在配置为阻塞模式时使用）
```C
bool Uart_Blocking_Receive(UartInstance_s* uart_instance);
```
## 3. 参考例程
```C
//定义回调函数
void Uart3CallBack(UartInstance_s *uart_instance) {
    // 这里可以添加UART3的接收处理逻辑
    // 例如解析接收到的数据，或者将数据发送到其他模块
}
//定义cofig结构体
UartInitConfig_s uart3_config = {
    .uart_handle = &huart3,
    .mode = DMA_Mode,
    .rx_len = 18,
    .tx_len = 0,
    .uart_module_callback = Uart3CallBack, // 这里可以设置回调函数
    .id = NULL // 如果不需要，可以设置为NULL
  };
  
//注册uart3实例
UartInstance_s *uart3_instance = Uart_Register(&uart3_config);
```