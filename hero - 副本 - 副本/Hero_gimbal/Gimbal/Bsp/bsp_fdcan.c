#include "bsp_fdcan.h" 
#include <string.h>
#include <stdbool.h>
#include "FreeRTOS.h"
//#include "bsp_log.h"

#ifdef USER_CAN_FD

#ifdef USER_CAN1
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t idx1;
// ReSharper disable once CppDeclaratorNeverUsed
static CanInstance_s *fdcan1_instance[FDCAN_MAX_REGISTER_CNT]; // CAN1 实例数组,
#endif

#ifdef USER_CAN2
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t idx2;
// ReSharper disable once CppDeclaratorNeverUsed
static CanInstance_s *fdcan2_instance[FDCAN_MAX_REGISTER_CNT]; // CAN2
#endif

#ifdef USER_CAN3
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t idx3;
// ReSharper disable once CppDeclaratorNeverUsed
static CanInstance_s *fdcan3_instance[FDCAN_MAX_REGISTER_CNT]; // CAN3
#endif

/* 过滤器编号 */
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t can_filter_index = 0;

/* fdcan初始化标志 */
static bool fdcan_init_flag = true;
/* 接收帧 */
static FDCAN_RxFrame_TypeDef *FDCAN_RxFIFO0Frame;
static FDCAN_RxFrame_TypeDef *FDCAN_RxFIFO1Frame;
/* 私有函数 ---------------------------------------------------------------------*/
/**
 * @brief 初始化 FDCAN 过滤器配置。
 *
 * 该函数根据用户定义的宏（USER_CAN1, USER_CAN2, USER_CAN3）初始化相应的 FDCAN 实例的过滤器。对于每个启用的 FDCAN 实例，它首先确定 ID 类型（标准或扩展），然后选择接收 FIFO（FIFO0 或 FIFO1）。如果两个 FIFO 都没有元素，则记录错误并返回 false。接着，配置过滤器参数，包括 ID 类型、过滤器索引、过滤器类型和过滤器配置。此外，还配置了全局过滤器以拒绝不匹配的标准 ID 和扩展 ID 以及远程帧，并设置了 FIFO 水印中断。如果在配置过程中出现任何错误，将记录错误日志并重试直到成功。
 * 注意：此函数依赖于 HAL 库提供的 FDCAN 相关 API。
 * @return 如果所有 FDCAN 实例的过滤器配置成功，返回 true；否则返回 false。
 */
static void Can_Filter_Init(void){
#ifdef USER_CAN1_FIFO_0
    FDCAN_FilterTypeDef FDCAN1_FilterFifo0Config;
    FDCAN1_FilterFifo0Config.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterFifo0Config.FilterIndex = can_filter_index; // 过滤器编号，用几路 CAN 就以此类推 0、1、2
    FDCAN1_FilterFifo0Config.FilterType = FDCAN_FILTER_MASK; // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    FDCAN1_FilterFifo0Config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN1_FilterFifo0Config.FilterID1 = 0x00000000; // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    FDCAN1_FilterFifo0Config.FilterID2 = 0x00000000; // 过滤器 ID2

    while (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterFifo0Config) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs fifo watermark failed");
    }
    can_filter_index++;
#endif
#ifdef USER_CAN1_FIFO_1
    FDCAN_FilterTypeDef FDCAN1_FilterFifo1Config;
    FDCAN1_FilterFifo1Config.IdType = FDCAN_STANDARD_ID;
    FDCAN1_FilterFifo1Config.FilterIndex = can_filter_index; // 过滤器编号
    FDCAN1_FilterFifo1Config.FilterType = FDCAN_FILTER_MASK; // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    FDCAN1_FilterFifo1Config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    FDCAN1_FilterFifo1Config.FilterID1 = 0x00000000; // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    FDCAN1_FilterFifo1Config.FilterID2 = 0x00000000; // 过滤器 ID2

    while (HAL_FDCAN_ConfigFilter(&hfdcan1, &FDCAN1_FilterFifo1Config) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs fifo watermark failed");
    }
    can_filter_index++;
#endif
#ifdef USER_CAN2_FIFO_0
    FDCAN_FilterTypeDef FDCAN2_FilterFifo0Config;
    FDCAN2_FilterFifo0Config.IdType = FDCAN_STANDARD_ID;
    FDCAN2_FilterFifo0Config.FilterIndex = can_filter_index; // 过滤器编号
    FDCAN2_FilterFifo0Config.FilterType = FDCAN_FILTER_MASK; // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    FDCAN2_FilterFifo0Config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN2_FilterFifo0Config.FilterID1 = 0x00000000; // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    FDCAN2_FilterFifo0Config.FilterID2 = 0x00000000; // 过滤器 ID2

    while (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterFifo0Config) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs fifo watermark failed");
    }
    can_filter_index++;
#endif
#ifdef USER_CAN2_FIFO_1
    FDCAN_FilterTypeDef FDCAN2_FilterFifo1Config;
    FDCAN2_FilterFifo1Config.IdType = FDCAN_STANDARD_ID;
    FDCAN2_FilterFifo1Config.FilterIndex = can_filter_index; // 过滤器编号
    FDCAN2_FilterFifo1Config.FilterType = FDCAN_FILTER_MASK; // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    FDCAN2_FilterFifo1Config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    FDCAN2_FilterFifo1Config.FilterID1 = 0x00000000; // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    FDCAN2_FilterFifo1Config.FilterID2 = 0x00000000; // 过滤器 ID2

    while (HAL_FDCAN_ConfigFilter(&hfdcan2, &FDCAN2_FilterFifo1Config) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan2, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs fifo watermark failed");
    }
    can_filter_index++;
#endif
#ifdef USER_CAN3_FIFO_0
    FDCAN_FilterTypeDef FDCAN3_FilterFifo0Config;
    FDCAN3_FilterFifo0Config.IdType = FDCAN_STANDARD_ID;
    FDCAN3_FilterFifo0Config.FilterIndex = can_filter_index; // 过滤器编号
    FDCAN3_FilterFifo0Config.FilterType = FDCAN_FILTER_MASK; // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    FDCAN3_FilterFifo0Config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    FDCAN3_FilterFifo0Config.FilterID1 = 0x00000000; // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    FDCAN3_FilterFifo0Config.FilterID2 = 0x00000000; // 过滤器 ID2

    while (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterFifo0Config) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO0, 1) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs fifo watermark failed");
    }
    can_filter_index++;
#endif
#ifdef USER_CAN3_FIFO_1
    FDCAN_FilterTypeDef FDCAN3_FilterFifo1Config;
    FDCAN3_FilterFifo1Config.IdType = FDCAN_STANDARD_ID;
    FDCAN3_FilterFifo1Config.FilterIndex = can_filter_index; // 过滤器编号
    FDCAN3_FilterFifo1Config.FilterType = FDCAN_FILTER_MASK; // 过滤器 Mask 模式 关乎到 ID1ID2 的配置
    FDCAN3_FilterFifo1Config.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    FDCAN3_FilterFifo1Config.FilterID1 = 0x00000000; // 过滤器 ID1，只要 ID2 配置为 0x00000000，就不会过滤任何 ID
    FDCAN3_FilterFifo1Config.FilterID2 = 0x00000000; // 过滤器 ID2

    while (HAL_FDCAN_ConfigFilter(&hfdcan3, &FDCAN3_FilterFifo1Config) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs filter failed");
    } // 拒绝接收匹配不成功的标准 ID 和扩展 ID, 不接受远程帧
    while (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) !=
        HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs global filter failed");
        }
    // 水印中断，接受 1 条消息触发中断
    while (HAL_FDCAN_ConfigFifoWatermark(&hfdcan3, FDCAN_CFG_RX_FIFO1, 1) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs fifo watermark failed");
    }
    can_filter_index++;
#endif
}

/**
 * @brief 初始化CAN服务。
 *
 * 该函数根据用户配置的宏定义（USER_CAN1, USER_CAN2, USER_CAN3）来初始化相应的FDCAN硬件实例。对于每个启用的FDCAN实例，首先尝试启动它。如果启动失败，则记录错误日志并继续重试直到成功。随后，根据USER_CANx_FIFOx的值选择激活Rx FIFO 0或Rx FIFO 1的消息接收中断通知。如果激活中断通知失败，同样会记录错误日志，并且持续尝试直到成功为止。
 * 注意：此函数依赖于HAL库提供的FDCAN相关API。
 * @return 无返回值（void）。
 */
static void Can_Service_Init(void){
#ifdef USER_CAN1
    while (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        Log_Error("FDCAN1 Starts Failed");
    }
#endif
#ifdef USER_CAN2
    while (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        Log_Error("FDCAN2 Starts Failed");
    }
#endif
#ifdef USER_CAN3
    while (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        Log_Error("FDCAN3 Starts Failed");
    }
#endif
#ifdef USER_CAN1_FIFO_0
    while (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN1 Fifo0 configs interruption failed");
    }
#endif

#ifdef USER_CAN1_FIFO_1
    while (HAL_FDCAN_ActivateNotification(&hfdcan1,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN1 Fifo1 configs interruption failed");
    }
#endif


#ifdef USER_CAN2_FIFO_0
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN2 Fifo0 configs interruption failed");
    }
#endif

#ifdef USER_CAN2_FIFO_1
    while (HAL_FDCAN_ActivateNotification(&hfdcan2,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN2 Fifo1 configs interruption failed");
    }
#endif

#ifdef USER_CAN3_FIFO_0
    while (HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN3 Fifo0 configs interruption failed");
    }
#endif
#ifdef USER_CAN3_FIFO_1
    while (HAL_FDCAN_ActivateNotification(&hfdcan3,FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Log_Error("FDCAN3 Fifo1 configs interruption failed");
    }
#endif
}


/**
 * @brief 初始化CAN模块。
 * 该函数首先尝试初始化CAN过滤器，如果成功，则继续初始化CAN服务，并记录一条通过日志。如果CAN过滤器初始化失败，则直接记录一条错误日志。
 * @todo 需要添加超时警告机制，防止初始化过程中的死循环。
 * @return 无返回值（void）。
 */
static void Can_Init(void) {
    Can_Filter_Init();
    Can_Service_Init();
    Log_Passing("Can Init successfully");
    fdcan_init_flag = false;
}

// ReSharper disable once CppNotAllPathsReturnValue
static FDCAN_HandleTypeDef* Select_FDCAN_Handle(const uint8_t can_channel){
#ifdef USER_CAN1
    if (can_channel == 1){
        return &hfdcan1;
    }
#endif
#ifdef USER_CAN2
    if (can_channel == 2){
        return &hfdcan2;
    }
#endif
#ifdef USER_CAN3
    if (can_channel == 3){
        return &hfdcan3;
    }
#endif
}

/* 公共函数 ------------------------------------------------------------------*/

/**
 * @brief 注册一个新的CAN实例。
 *
 * 根据提供的配置信息创建并初始化一个新的CAN实例。如果配置信息为空或内存分配失败，则函数将返回NULL。成功注册后，新的CAN实例将被添加到相应的FDCAN实例列表中，并返回指向新实例的指针。
 *
 * @param config 指向包含CAN初始化配置信息（如句柄、发送ID等）的CanInitConfig_s结构的指针。
 *
 * @return 如果成功注册则返回指向新创建的CanInstance_s结构的指针；如果配置无效或内存分配失败则返回NULL。
 */
CanInstance_s *Can_Register(const CanInitConfig_s *config) {
    if (config == NULL || config->can_channel == 0) {
        Log_Error("%s CanInitConfig Is Null", config->topic_name);
        return NULL; // 参数检查
    }
    if (fdcan_init_flag) {
        Can_Init();
    }
    CanInstance_s *instance = user_malloc(sizeof(CanInstance_s)); // 分配空间
    if (instance == NULL) {
        Log_Error("%s CanInstance Malloc Failed", config->topic_name);
        return NULL; // 内存分配失败
    }
    memset(instance, 0, sizeof(CanInstance_s));
    instance->topic_name = config->topic_name;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instance->tx_conf.Identifier = config->tx_id;
    instance->can_handle = Select_FDCAN_Handle(config->can_channel);
    // config->can_handle->Init.FrameFormat
    instance->tx_conf.IdType = FDCAN_STANDARD_ID; // 标准 ID
    // instance->tx_conf.IdType = instance->can_handle->Init.FrameFormat;
    instance->tx_conf.TxFrameType = FDCAN_DATA_FRAME;
    instance->tx_conf.DataLength = FDCAN_DLC_BYTES_8;
    instance->tx_conf.ErrorStateIndicator = FDCAN_ESI_ACTIVE; // 传输节点 error active
    instance->tx_conf.BitRateSwitch = FDCAN_BRS_OFF; // FDCAN 帧发送 / 接收不带波特率可变
    instance->tx_conf.FDFormat = FDCAN_CLASSIC_CAN; // 设置为经典 CAN 帧格式
    instance->tx_conf.TxEventFifoControl = FDCAN_NO_TX_EVENTS; // 不存储 Tx events 事件
    instance->tx_conf.MessageMarker = 0;

#ifdef USER_CAN1
    if (instance->can_handle == &hfdcan1) {
        fdcan1_instance[idx1++] = instance;
    }
#endif
#ifdef USER_CAN2
    if (instance->can_handle == &hfdcan2){
        fdcan2_instance[idx2++] = instance;
    }
    #endif
#ifdef USER_CAN3
    if (instance->can_handle == &hfdcan3)
    {
        fdcan3_instance[idx3++] = instance;
    }
#endif

    Log_Passing("%s Register Successfully", instance->topic_name);
    return instance;
}

bool Can_Transmit(const CanInstance_s *instance, const uint8_t *tx_buff) {
    while (HAL_FDCAN_GetTxFifoFreeLevel(instance->can_handle) == 0) {
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(instance->can_handle, &instance->tx_conf, tx_buff) == HAL_OK) {
        return true;
    }
    return false;
}
/**
 * @brief FDCAN接收FIFO中断的回调函数。
 * 该函数处理接收FIFO中的消息。它检索消息，并在消息标识符与已注册的CAN实例之一匹配时调用相应的回调函数。
 *
 * @param FDCAN_RxFIFOxFrame 指向包含接收到的FDCAN消息的FDCAN_RxFrame_TypeDef结构的指针。
 * @param idx 表示fdcan_instance数组中CanInstance_s元素的数量的索引。
 * @param fdcan_instance CanInstance_s结构数组，每个结构包含特定CAN实例的配置和回调。
 */
static void FDCAN_RxFifoCallback(const FDCAN_RxFrame_TypeDef *FDCAN_RxFIFOxFrame, const uint8_t idx, CanInstance_s *fdcan_instance) {
            if (idx == 0)
                return;
            // ReSharper disable once CppDFAUnreachableCode
            for (uint8_t i = 0; i < idx; i++) {
                if ( FDCAN_RxFIFOxFrame->Header.Identifier == fdcan_instance[i].rx_id) {
                    if (fdcan_instance[i].can_module_callback != NULL)
                    {
                        fdcan_instance[i].rx_len = FDCAN_RxFIFOxFrame->Header.DataLength;
                        memcpy(fdcan_instance[i].rx_buff, FDCAN_RxFIFOxFrame->rx_buff,fdcan_instance->rx_len);
                        fdcan_instance[i].cnt++; // 通信计数+1
                        if(fdcan_instance[i].cnt>=0xFFFF) fdcan_instance[i].cnt=0; //防止溢出
                        fdcan_instance[i].can_module_callback(&fdcan_instance[i]);
                    }
                    break;
                }
            }
}


/**
 * @brief FDCAN接收FIFO0中断的回调函数。
 *
 * 该函数处理接收FIFO0中的消息。当检测到新的消息时，它会根据配置调用相应的用户定义的回调函数。
 *
 * @param hfdcan 指向包含指定FDCAN配置信息的FDCAN_HandleTypeDef结构的指针。
 * @param RxFifo0ITs 触发此回调的中断源。
 */
// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO0Frame->Header, FDCAN_RxFIFO0Frame->rx_buff);

#ifdef USER_CAN1_FIFO_0
        if (hfdcan == &hfdcan1){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO0Frame,idx1,*fdcan1_instance);
        }
#endif
#ifdef USER_CAN2_FIFO_0
        if (hfdcan == &hfdcan2){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO0Frame,idx2,*fdcan2_instance);
        }
#endif
#ifdef USER_CAN3_FIFO_0
        if (hfdcan == &hfdcan3){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO0Frame,idx3,*fdcan3_instance);
        }
#endif
    }
}

/**
 * @brief FDCAN接收FIFO1中断的回调函数。
 *
 * 该函数处理来自FDCAN接收FIFO1中的消息。当接收到新消息时，它会根据配置调用相应的用户定义回调函数。
 *
 * @param hfdcan 指向包含指定FDCAN配置信息的FDCAN_HandleTypeDef结构的指针。
 * @param RxFifo1ITs 触发此回调的中断源。
 */
// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
    if (RxFifo1ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) {
        HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &FDCAN_RxFIFO1Frame->Header, FDCAN_RxFIFO0Frame->rx_buff);
#ifdef USER_CAN1_FIFO_1
        if (hfdcan == &hfdcan1){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO1Frame,idx1, *fdcan1_instance);
        }
#endif
#ifdef USER_CAN2_FIFO_1
        if (hfdcan == &hfdcan2){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO1Frame,idx2,*fdcan2_instance);
        }
#endif
#ifdef USER_CAN3_FIFO_1
        if (hfdcan == &hfdcan3){
            FDCAN_RxFifoCallback(FDCAN_RxFIFO1Frame,idx3,*fdcan3_instance);
        }
#endif
    }
}

//对HAL_FDCAN_ErrorStatusCallback的重载
//@todo 待完善
// ReSharper disable once CppParameterMayBeConstPtrOrRef
// ReSharper disable once CppParameterMayBeConst
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs){
    if (ErrorStatusITs & FDCAN_IR_BO) {
        CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
    }
}

#endif
