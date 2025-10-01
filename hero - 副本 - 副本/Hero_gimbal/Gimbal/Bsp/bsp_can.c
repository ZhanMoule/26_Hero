/**
* @file bsp_can.c
 * @author He WenXuan (hewenxuan040923@gmail.com)
 * @brief CAN驱动模块
 * @version 0.1
 * @details CAN驱动模块,提供CAN的初始化、发送、接收等功能
 * @date 2025-07-04
 * @update 2025-08-26
 *       1. 增加对fifo0和fifo1的支持，用户可根据需要选择使用哪个fifo，而非指定使用fifo0或fifo1、
 *       2. 完成之前的to do
 *          @done 1. 与HAL库和STM32F4彻底解耦，提供更灵活的接口
 *          @done 2. 支持更多的报错信息提醒，方便调试
 *          @done 3. 增加对CAN FD的支持 //对两个文件进行了分离
 * @copyright  Copyright (c) 2025 HDU—PHOENIX
 * @todo 1.Can_Transmit函数中增加发送超时机制,防止死等待,以及对HAL库函数的重写，提升性能
 */


#include "bsp_can.h"
#ifdef USER_CAN_STANDARD
#include "FreeRTOS.h"
#include "bsp_log.h"
#include <string.h>
#include <stdbool.h>
/* 私有类型定义 -----------------------------------------------------------------*/
#ifdef USER_CAN1
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t can_idx1;
// ReSharper disable once CppDeclaratorNeverUsed
static CanInstance_s *can1_instance[CAN_MAX_REGISTER_CNT]; // CAN1 实例数组,
#endif
#ifdef USER_CAN2
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t can_idx2;
// ReSharper disable once CppDeclaratorNeverUsed
static CanInstance_s *can2_instance[CAN_MAX_REGISTER_CNT]; // CAN2
#endif

/* 过滤器编号 */
// ReSharper disable once CppDeclaratorNeverUsed
static uint8_t can_filter_index = 0;

/* can初始化标志 */
static bool can_init_flag = true;

/* 接收帧 */
static CAN_RxFrame_TypeDef CAN_RxFIFO0Frame;
static CAN_RxFrame_TypeDef CAN_RxFIFO1Frame;
static uint8_t can_fifo_select_flag = 0; // 0表示使用fifo0,1表示使用fifo1
/* 私有函数 ---------------------------------------------------------------------*/
/**
 * @brief 初始化 CAN 过滤器配置。
 * 该函数根据用户定义的宏（USER_CANx_FIFO_x）配置 CAN 过滤器。
 * @note 此函数依赖于HAL库提供的CAN相关API。
 * @return void,失败会卡死在while循环
 * @update 2025-08-26
 *        1. 默认全部使用FIFO0和FIFO1
 */
// ReSharper disable once CppUnusedFunction
static void Can_ID_Mask_Mode_Filter_Init(void) {
    /* 配置 CAN1_FIFO_0 过滤器 */
    /* 配置 CAN1 FIFO0 结构体 */
    CAN_FilterTypeDef filter_can1_config;
    filter_can1_config.FilterActivation = CAN_FILTER_ENABLE;
    filter_can1_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_can1_config.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_can1_config.FilterIdHigh = 0x0000;
    filter_can1_config.FilterIdLow = 0x0000;
    filter_can1_config.FilterMaskIdHigh = 0x0000;
    filter_can1_config.FilterMaskIdLow = 0x0000;
    filter_can1_config.FilterBank = 0;
#ifdef USER_CAN1_FIFO_0
    filter_can1_config.FilterFIFOAssignment = CAN_RX_FIFO0;
#endif
#ifdef USER_CAN1_FIFO_1
    filter_can1_config.FilterFIFOAssignment = CAN_RX_FIFO1;
#endif

    while (HAL_CAN_ConfigFilter(&hcan1, &filter_can1_config) != HAL_OK) {
        Log_Error("CAN1 FIFO0 Filter Config Error");
    }
#ifdef USER_CAN2
    CAN_FilterTypeDef filter_can2_config;
    filter_can2_config.FilterActivation = CAN_FILTER_ENABLE;
    filter_can2_config.FilterMode = CAN_FILTERMODE_IDMASK;
    filter_can2_config.FilterScale = CAN_FILTERSCALE_32BIT;
#ifdef USER_CAN2_FIFO_0
    filter_can2_config.FilterFIFOAssignment = CAN_RX_FIFO0;
#endif
#ifdef USER_CAN2_FIFO_1
    filter_can2_config.FilterFIFOAssignment = CAN_RX_FIFO1;
#endif
    filter_can2_config.FilterIdHigh = 0x0000;
    filter_can2_config.FilterIdLow = 0x0000;
    filter_can2_config.FilterMaskIdHigh = 0x0000;
    filter_can2_config.FilterMaskIdLow = 0x0000;
    filter_can2_config.FilterBank = 14;
    filter_can2_config.SlaveStartFilterBank = 14;

    while (HAL_CAN_ConfigFilter(&hcan2, &filter_can2_config) != HAL_OK) {
        Log_Error("CAN2 FIFO0 Filter Config Error");
    }
#endif
}
/**
 * @brief 使用ID列表模式初始化CAN过滤器。
 * 该函数根据传入的CAN实例配置过滤器，以便只接收特定ID的报文。
 * @param instance 指向CanInstance_s结构体的指针，包含初始化CAN所需的配置参数
 * @return 如果过滤器配置成功，返回true；否则返回false，并通过日志记录错误原因
 * @note 此函数依赖于HAL库提供的CAN相关API。
 * @update 2025-08-26
 *        1. 增加对fifo0和fifo1的支持,用户可根据需要选择使用哪个fifo，而非指定使用fifo0或fifo1
 */
static bool Can_ID_List_Mode_Filter_Init(const CanInstance_s *instance) {
    CAN_FilterTypeDef can_filter_config;
#if defined (USER_CAN1)
    /* CAN1过滤器索引 0~13给CAN1*/
    static uint8_t can1_filter_idx = 0;
#endif
#if defined (USER_CAN2)
    /* CAN2过滤器索引 14~27给CAN2*/
    static uint8_t can2_filter_idx = 14;
#endif
    /* 选择fifo */
#if defined(USER_CAN1_FIFO_0) || defined(USER_CAN2_FIFO_0)
#if defined(USER_CAN1_FIFO_1) || defined(USER_CAN2_FIFO_1)
    /* 如果同时定义了FIFO0和FIFO1,则交替使用 */
    can_filter_config.FilterFIFOAssignment = can_fifo_select_flag ? CAN_RX_FIFO0 : CAN_RX_FIFO1;
#else
    /* 如果只定义了FIFO0,则全部使用FIFO0 */
    can_filter_config.FilterFIFOAssignment = CAN_RX_FIFO0;
#endif
#elif defined (USER_CAN1_FIFO_1) || defined(USER_CAN2_FIFO_1)
    /* 如果只定义了FIFO1,则全部使用FIFO1 */
    can_fifo_config.FilterFIFOAssignment = CAN_RX_FIFO1;
#endif
#if defined (USER_CAN1)
#if defined (USER_CAN2)
    /* 根据can_handle判断是CAN1还是CAN2,然后自增 */
    can_filter_config.FilterBank = instance->can_handle == &hcan1 ? (can1_filter_idx++) : (can2_filter_idx++);
#else
    /* 只有CAN1 */
    /* @note : CAN1是主机，CAN2是从机，不存在只有CAN2的情况*/
    can_filter_config.FilterBank = can1_filter_idx++;
#endif
#endif
    /* 过滤器模式选择，使用id list模式,即只有将rx_id添加到过滤器中才会接收到,其他报文会被过滤 */
    can_filter_config.FilterMode = CAN_FILTERMODE_IDLIST;
    /* 使用16位id模式,即只有低16位有效 */
    can_filter_config.FilterScale = CAN_FILTERSCALE_16BIT;
    /* 从机过滤器起始编号 */
    can_filter_config.SlaveStartFilterBank = 14;
    /*过滤器寄存器的低16位,因为使用标准ID,所以只有低11位有效,高5位要填0 */
    can_filter_config.FilterIdLow = instance->rx_id << 5;
    /* 启用过滤器 */
    can_filter_config.FilterActivation = CAN_FILTER_ENABLE;

    if (HAL_CAN_ConfigFilter(instance->can_handle, &can_filter_config) != HAL_OK) {
        Log_Error("%s : CAN Filter Config Error", instance->topic_name);
        return false;
    }
    can_fifo_select_flag = !can_fifo_select_flag; // 切换fifo标志
    return true;
}
/**
 * @brief 初始化 CAN 服务。
 * 该函数根据用户配置的宏定义（USER_CANx, USER_CANx_FIFO_y）来初始化相应的 CAN 硬件实例。
 * @note 此函数依赖于HAL库提供的CAN相关API。
 * @return void,失败会卡死在while循环
 */
static void Can_Service_Init(void) {
#ifdef USER_CAN1
    while (HAL_CAN_Start(&hcan1) != HAL_OK) {
        Log_Error("CAN1 Starts Failed");
    }
#ifdef USER_CAN2
    while (HAL_CAN_Start(&hcan2) != HAL_OK) {
        Log_Error("CAN2 Starts Failed");
    }
#endif
#endif
#ifdef USER_CAN1_FIFO_0
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Log_Error("CAN1 FIFO0 Interruption Config Error");
    }
#endif
#ifdef USER_CAN2_FIFO_0
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Log_Error("CAN2 FIFO0 Interruption Config Error");
    }
#endif

#ifdef USER_CAN1_FIFO_1
    while (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        Log_Error("CAN1 FIFO1 Interruption Config Error");
    }
#endif
#ifdef USER_CAN2_FIFO_1
    while (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK) {
        Log_Error("CAN2 FIFO1 Interruption Config Error");
    }
#endif
}

/**
 * @brief 初始化CAN模块。
 * 该函数首先尝试初始化CAN过滤器，如果成功，则继续初始化CAN服务，并记录一条通过日志。如果CAN过滤器初始化失败，则直接记录一条错误日志。
 * @todo 需要添加超时警告机制，防止初始化过程中的死循环,但是能做到初始化失败的也是神人了
 * @return 无返回值（void）。
 */
static void Can_Init(void) {
#if defined USER_CAN_FILTER_MASK_MODE
    Can_ID_Mask_Mode_Filter_Init();
    Log_Passing("Can MASK Filter Init successfully");
#endif
    Can_Service_Init();
    Log_Passing("Can Service Init successfully");
    can_init_flag = false;
    Log_Passing("Can Init successfully");
}


/**
 * @brief 根据CAN编号选择对应的CAN句柄。
 * 该函数根据传入的CAN编号返回相应的CAN句柄指针。如果编号无效或对应的CAN未启用，则返回NULL。
 * @param can_number CAN编号，1表示CAN1，2表示CAN2。
 * @return 指向对应CAN句柄的指针，如果编号无效则返回NULL。
 */

static CAN_HandleTypeDef *CAN_Select_Handle(const uint8_t can_number) {
#ifdef USER_CAN1
    if (can_number == 1) {
        return &hcan1;
    }
#endif
#ifdef USER_CAN2
    if (can_number == 2) {
        return &hcan2;
    }
#endif
    return NULL; //防止编译警告,我无法想象有人在没开启can的情况下能调用这个函数
}

/**
 * @brief 检查CAN实例注册的有效性。
 * 该函数验证传入的CAN初始化配置是否完整且符合要求，包括检查配置是否为空、必要字段是否填写、CAN编号是否合法以及ID是否冲突等。
 * @param config 指向CanInitConfig_s结构体的指针，包含初始化CAN所需的配置参数
 * @return 如果配置有效且符合要求，返回true；否则返回false，并通过日志记录错误原因
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static bool Can_Register_Check(CanInitConfig_s *config) {
    /* 检查配置是否为空 */
    if (config == NULL) {
        Log_Error("Can : Register Failed, Config is NULL");
        return false;
    }
    /* 检查是否正常配置 */
    if (config->topic_name == NULL || config->rx_id == 0 ||
        config->tx_id == 0 || !(config->can_number == 1 || config->can_number == 2)) {
        Log_Error("Can : Register Failed, Config is Incomplete");
        return false;
    }
#ifdef USER_CAN1
    if (config->can_number == 1) {
        /* 检查是否超过CAN1最大实例数 */
        if (can_idx1 == CAN_MAX_REGISTER_CNT) {
            Log_Error("%s : Can1 Register Failed, Max Register Count Reached", config->topic_name);
            return false;
        }
        /* 检查ID是否冲突 */
        for (uint8_t i = 0; i < can_idx1; i++) {
            if (can1_instance[i]->rx_id == config->rx_id) {
                Log_Error("%s : Can1 Register Failed, Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#ifdef USER_CAN2
    if (config->can_number == 2) {
        /* 检查是否超过CAN2最大实例数 */
        if (can_idx2 == CAN_MAX_REGISTER_CNT) {
            Log_Error("%s : Can2 Register Failed, Max Register Count Reached", config->topic_name);
            return false;
        }
        for (uint8_t i = 0; i < can_idx2; i++) {
            if (can2_instance[i]->rx_id == config->rx_id) {
                Log_Error("%s : Can2 Register Failed, Rx ID 0x%03X Already Exists", config->topic_name, config->rx_id);
                return false;
            }
        }
    }
#endif
#endif
    return true;
}

/**
 * @brief 将CAN实例注册到对应的CAN实例数组中。
 * 该函数根据CAN实例的句柄将其添加到相应的CAN实例数组中，并更新实例计数器。如果注册成功，将通过日志记录相关信息。
 * @param instance 指向要注册的CanInstance_s结构体的指针
 */
static void Can_Register_To_x_Instance(CanInstance_s *instance) {
#ifdef USER_CAN1
    if (instance->can_handle == &hcan1) {
        can1_instance[can_idx1] = instance;
        can_idx1++;
        Log_Passing("%s : Can1 Register Successfully, Tx ID:0x%03X, Rx ID:0x%03X", instance->topic_name,
                    instance->tx_id,
                    instance->rx_id);
    }
#ifdef USER_CAN2
    if (instance->can_handle == &hcan2) {
        can2_instance[can_idx2] = instance;
        can_idx2++;
        Log_Passing("%s : Can2 Register Successfully, Tx ID:0x%03X, Rx ID:0x%03X", instance->topic_name,
                    instance->tx_id,
                    instance->rx_id);
    }
#endif
#endif
}

/**
 * @brief 注册CAN实例并初始化其配置。
 * 该函数根据提供的配置信息注册一个新的CAN实例。如果成功，将返回指向新实例的指针；如果失败，则返回NULL，并通过日志记录错误原因。
 * @param can_config 指向CanInitConfig_s结构体的指针，包含初始化CAN所需的配置参数
 * @return 返回指向新创建的CAN实例的指针，或在发生错误时返回NULL
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
CanInstance_s *Can_Register(CanInitConfig_s *can_config) {
    /* 检查注册条件 */
    if (Can_Register_Check(can_config) == false) {
        return NULL;
    }
    /* 如果是第一次注册CAN实例，则初始化CAN模块 */
    if (can_init_flag) {
        Can_Init();
    }
    /* 分配内存并初始化CAN实例 */
    CanInstance_s *instance = user_malloc(sizeof(CanInstance_s));
    if (instance == NULL) {
        Log_Error("%s : Can Register Failed, No Memory", can_config->topic_name);
        return NULL;
    }

		
    /* 清空内存 */
    memset(instance, 0, sizeof(CanInstance_s));
    /* 注册实例名称 */
    instance->topic_name = can_config->topic_name;
    /* 选择并注册CAN句柄 */
    instance->can_handle = CAN_Select_Handle(can_config->can_number);
    /* 配置发送ID */
    instance->tx_id = can_config->tx_id;
    /* 配置CAN发送报文头 */
    instance->tx_header = (CAN_TxHeaderTypeDef){
        /* 指定标准标识符。该参数必须是 0 到 0x7FF 之间的数值 */
        .StdId = can_config->tx_id,
        /* 指定扩展标识符。该参数必须是 0 到 0x1FFFFFFF 之间的数值 */
        .ExtId = 0x00, //。
        /* 指定要发送消息的标识符类型。该参数可取值参考 CAN_identifier_type */
        .IDE = CAN_ID_STD,
        /* 指定要发送消息的帧类型。该参数可取值参考 CAN_remote_transmission_request */
        .RTR = CAN_RTR_DATA,
        /* 指定要发送帧的长度。该参数必须是 0 到 8 之间的数值。 */
        .DLC = 0x08,
        /* 指定是否在帧发送开始时将时间戳计数器值发送到 DATA6 和 DATA7（替换 pData[6] 和 pData[7]）。 */
        .TransmitGlobalTime = DISABLE
    };
    /* 配置接收ID */
    instance->rx_id = can_config->rx_id;
    /* 配置接收回调函数 */
    instance->can_module_callback = can_config->can_module_callback;
    /* 配置父指针 */
    instance->parent_ptr = can_config->parent_ptr;
#if defined USER_CAN_FILTER_LIST_MODE
    /* 使用ID列表过滤器模式初始化CAN过滤器 */
    Can_ID_List_Mode_Filter_Init(instance);
#endif
    /* 将实例注册到对应的CAN实例数组中 */
    Can_Register_To_x_Instance(instance);
    /* 返回注册的CAN实例指针 */
    return instance;
}

/**
 * @brief 通过CAN总线发送数据。
 * 该函数将指定的数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @param tx_buff 指向要发送的数据缓冲区的指针，数据长度应为8
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit_External_Tx_Buff(const CanInstance_s *instance, const uint8_t *tx_buff) {
    /* 检查实例和发送缓冲区是否有效 */
    if (instance == NULL || tx_buff == NULL) {
        Log_Error("Can Transmit Failed, Instance or Tx Buff is NULL");
        return false;
    }
    /*@todo 重写并整合下面两个HAL库的函数可以减少一次寻找空邮箱的操作*/
    /* 等待直到有可用的发送邮箱 */
    while (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle) == 0) {
    }
    /* 将消息添加到发送邮箱队列 */
    /* @note 这里虽然使用了CAN_TX_MAILBOX0，但是在函数内部会进行寻找空邮箱再发送的操作 */
    if (HAL_CAN_AddTxMessage(instance->can_handle, &instance->tx_header, (uint8_t *) tx_buff,
                             (uint32_t *) CAN_TX_MAILBOX0) == HAL_OK) {
        return true;
    }
    /* 如果添加失败，返回false */
    return false;
}
/**
 * @brief 通过CAN总线发送数据,为了避免大修MODULE而写的函数
 * 该函数将实例内部的发送缓冲区数据通过给定的CAN实例发送出去。如果发送成功，返回true；否则返回false。
 * @param instance 指向已注册的CanInstance_s结构体的指针，表示要使用的CAN实例
 * @return 如果数据发送成功则返回true，否则返回false
 */
bool Can_Transmit(const CanInstance_s *instance) {
    while (HAL_CAN_GetTxMailboxesFreeLevel(instance->can_handle) == 0) {
        uint8_t count;
        count++;
        if (count > 100) {
            break;
        }
    }

    if (HAL_CAN_AddTxMessage(instance->can_handle, &instance->tx_header, (uint8_t *) instance->tx_buff,
                             (uint32_t *) CAN_TX_MAILBOX0) == HAL_OK) {
        return true;
    }
    /* 如果添加失败，返回false */
    return false;
}

/**
 * @brief 处理CAN接收FIFO中的消息。
 * 该函数遍历接收到的CAN消息，并根据消息的标准标识符调用相应的回调函数进行处理。
 * @param CAN_RxFIFOxFrame 指向包含接收到的CAN消息的CAN_RxFrame_TypeDef结构的指针
 * @param idx 已注册的CAN实例数量
 * @param can_instance 指向已注册的CanInstance_s结构体数组的指针
 */
// ReSharper disable once CppParameterMayBeConstPtrOrRef
static void USER_CAN_RxFifo0MsgPendingCallback(CAN_RxFrame_TypeDef *CAN_RxFIFOxFrame, const uint8_t idx,
                                               CanInstance_s **can_instance) {
    /* 检查是否为注册的CAN实例 */
    if (idx == 0) {
        return;
    }
    /* 遍历已注册的CAN实例，查找匹配的接收ID */
    for (uint8_t i = 0; i < idx; i++) {
        if (CAN_RxFIFOxFrame->RxHeader.StdId == can_instance[i]->rx_id) {
            /* 如果找到匹配的ID且回调函数不为空，则调用回调函数处理接收到的数据 */
            if (can_instance[i]->can_module_callback != NULL) {
                /* 更新接收长度并复制接收到的数据 */
                can_instance[i]->rx_len = CAN_RxFIFOxFrame->RxHeader.DLC;
                memcpy(can_instance[i]->rx_buff, CAN_RxFIFOxFrame->rx_buff, can_instance[i]->rx_len);
                can_instance[i]->cnt++; // 通信计数+1
                if(can_instance[i]->cnt>=0xFFFF) can_instance[i]->cnt=0; //防止溢出
                /* 调用用户定义的回调函数 */
                can_instance[i]->can_module_callback(can_instance[i]);
            }
            break;
        }
    }
}

/* 以下为HAL库回调函数,重定义 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // ReSharper disable once CppLocalVariableMayBeConst
    /* 获取FIFO0中的消息数量 */
    uint8_t rx_fifo_0_msg_count = HAL_CAN_GetRxFifoFillLevel(hcan,CAN_RX_FIFO0);
    /* 如果消息数量超过7条，记录警告日志 */
    if (rx_fifo_0_msg_count >= 7) {
        Log_Warning("RX FIFO0 Message Count is %d", rx_fifo_0_msg_count);
    }
    /* 从FIFO0中获取接收的消息 */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN_RxFIFO0Frame.RxHeader, CAN_RxFIFO0Frame.rx_buff);
    /* 根据CAN句柄调用相应的回调函数处理接收到的消息 */
#ifdef USER_CAN1
    if (hcan == &hcan1) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO0Frame, can_idx1, can1_instance);
    }
#ifdef USER_CAN2
    if (hcan == &hcan2) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO0Frame, can_idx2, can2_instance);
    }
#endif
#endif
}

/* 以下为HAL库回调函数,重定义 */
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    // ReSharper disable once CppLocalVariableMayBeConst
    /* 获取FIFO1中的消息数量 */
    uint8_t rx_fifo_1_msg_count = HAL_CAN_GetRxFifoFillLevel(hcan,CAN_RX_FIFO1);
    /* 如果消息数量超过7条，记录警告日志 */
    if (rx_fifo_1_msg_count >= 7) {
        Log_Warning("RX FIFO1 Message Count is %d", rx_fifo_1_msg_count);
    }
    /* 从FIFO1中获取接收的消息 */
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &CAN_RxFIFO1Frame.RxHeader, CAN_RxFIFO1Frame.rx_buff);
    /* 根据CAN句柄调用相应的回调函数处理接收到的消息 */
#ifdef USER_CAN1
    if (hcan == &hcan1) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO1Frame, can_idx1, can1_instance);
    }
#ifdef USER_CAN2
    if (hcan == &hcan2) {
        USER_CAN_RxFifo0MsgPendingCallback(&CAN_RxFIFO1Frame, can_idx2, can2_instance);
    }
#endif
#endif
}
#endif