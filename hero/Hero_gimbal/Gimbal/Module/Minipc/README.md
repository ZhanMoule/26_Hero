# 环境配置
在CubeMX里打开USB_Device，保证时钟树正常，将usb_cdc_if.c和.h文件替换成该工程的文件。
如果你不想复制usb_cdc文件，下面将给出简单的配置流程：
### 修改 usbd_cdc_if.c
为了将USB接收的数据传递给上层应用，我们需要修改`usbd_cdc_if.c`以实现一个回调函数。

**1. 添加头文件和回调函数定义**

在`usbd_cdc_if.c`文件顶部的`/* USER CODE BEGIN INCLUDE */`区域，添加`dev_minipc.h`的引用。然后在`/* USER CODE BEGIN PV */`区域，定义回调函数指针类型和静态变量。

```c
/* USER CODE BEGIN INCLUDE */
#include "dev_minipc.h"
/* USER CODE END INCLUDE */

/* ... */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
// 定义一个函数指针类型，用于USB接收回调
typedef void (*USB_RxCallback_t)(uint8_t* data, uint32_t length);
// 声明一个静态的回调函数指针变量
static USB_RxCallback_t usb_rx_callback = NULL;
/* USER CODE END PV */
```
**2. 添加回调注册函数**
```c
/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */
void USB_RegisterRxCallback(USB_RxCallback_t callback);
/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/* ... */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
// 回调注册函数实现
void USB_RegisterRxCallback(USB_RxCallback_t callback) {
    usb_rx_callback = callback;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
```

**3. 修改 CDC_Receive_FS 函数**

这是最关键的一步。修改CDC_Receive_FS函数，当接收到数据时，调用我们注册的回调函数。

```c
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  // 调用回调函数处理接收到的数据
  if (usb_rx_callback != NULL && *Len > 0)
  {
    usb_rx_callback(Buf, *Len);
  }

  // 准备下一次接收
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
```
在/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */区域声明注册函数，并在/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */区域实现它。  


注：如果你使用的不是哨兵，则可以将#define SENTRY_MODE此行注释
# 使用方法
首先需要定义一个minipc实例，例如：  
```
MiniPC_Instance* miniPC_ins;
```
再根据你需要的消息类型，声明其配置文件结构体，例如：
```
MiniPC_Config miniPC_config = {
      .message_type = USB_MSG_MODULE_RX,
      .callback = NULL
  };
```
或者
```
MiniPC_Config miniPC_config2 = {
    .callback = NULL,
    .message_type = USB_MSG_AIM_RX, // 自瞄数据
    .Send_message_type = USB_MSG_AIM_TX // 发送数据类型
};
```
其中，message_type是接受的数据类型，Send_message_type 是发送数据类型
然后在任务初始化或者main初始化时对其进行注册
```
minipc_ins2 = Minipc_Register(&Chassis_config);
```
接下来就可以对你要发送的数据进行配置。每个不同的数据包都有对应的配置函数，例如自瞄数据包：
```
Minipc_ConfigAimTx(miniPC_ins2, &bmi088_test->gyro[0], &test2, &test3, &test4, &test5, &test6);//只是例子
```
其中第一个参数是minipc实例，其余参数是根据数据包而定的变量的指针

或者，你也可以自定义一个回调函数，并且把Send_message_type改为DIY_MODE
然后如下配置：
```
//测试代码
float testdata3=0.114514f;
//自定义的回调函数
void callback(send_union* send_buffer) {
    // 处理回调逻辑
    miniPC_ins->data_source.aim_tx.pitch =&testdata3 ; // 示例数据
}
```
这样就会在打包后，以自定义的方式填入数据并且发送  
但是注意此时消息包的数据类型是0XFE，如果需要自定协议，需要更改message_type结构体  


接着调用Minipc_UpdateAllInstances(),就可以将所有实例的数据发送，数据包内的数据会自动更新。  
之后，接收的消息会储存在实例的message联合体中，你可以用需要的方式访问。 
## 测试事项
本库能自动过滤实例不感兴趣的消息类型，因此测试的时候要注意发送的数据格式。下面将给出两个例子：  
底盘测试包(上发下)  
73 A3 01 00 00 28 41 00 00 A2 C1 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 65  
自瞄测试包(下发上)  
73 A0 01 00 00 28 41 00 00 A2 C1 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 01 00 00 65  
在上位机发送以上两个包后，相关message_type的实例应当可以在"实例->message.norm_aim_pack"(底盘测试包)"实例->message.ch_pack"(自瞄测试包)中看到相关数据。

## 注意事项
本库中包含大量Printf，因此如果勾选了Microlib，并且没有对printf重定向，将会导致各种故障。  
此外，请注意，配置发送数据的时候使用的是指针，用意是避免用户需要手动循环填入会更新的数据的麻烦。  
也是正因如此，本库不需要在循环中调用任何形式的config，只需要使用update相关接口即可。
## 更新计划
1.计划加入更多通信包的支持  
2.计划加入可以直接在循环中调用的回调函数，每次调用update的时候不需要使用指针，直接传入值也可以
## 相关知识
请参考USB_CDC库的设计，以及位域和宏定义等知识 