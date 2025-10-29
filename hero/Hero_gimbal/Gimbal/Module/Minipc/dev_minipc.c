/**
 * @file dev_minipc.c
 * @brief 上下位机通信
 * 
 * @details 该模块实现了上下位机之间的通信协议，包括数据包的定义和发送接收函数。
 * @note 实际上模块实现了不定长的数据包接收和解析，但是分包比较麻烦，这里又限制为了32个字节
 *
 * @author CGH
 * @editor HeWenXuan
 */


//牢大不允许我碎碎念（哭）

/**
 * 数据流过程：
 * 数据填入缓冲区→触发中断，转移到环形缓冲区→主动读取数据，转移到结构体→消息中心
 */
#include <stdint.h>
#include "dev_minipc.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "bsp_log.h"
#define USB_RX_BUFFER_SIZE 32//64个字节，2个完整包
#define USB_MAX_INSTANCE 8//允许的最多实例数量



//环形缓冲区,有数据到达会自动储存到这个缓冲区内

typedef struct {
    uint8_t buffer[USB_RX_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} USB_RingBuffer_t;
extern USBD_HandleTypeDef hUsbDeviceFS; 
USB_RingBuffer_t usb_rx_ring_buffer = {0};
// static Publisher *USB_Topics[USB_MAX_INSTANCE] = {NULL};
static uint8_t USB_Instance_Count = 255;//当前USB实例数量
// 在文件顶部声明（修改变量类型）
static message_union* packet_pointer = NULL;  // 指针类型
MiniPC_Instance* minipc_instances[USB_MAX_INSTANCE] = {NULL};

/**
 * @brief 注册MiniPC实例
 * 
 * @param config MiniPC配置
 * @return MiniPC_Instance* 注册的实例指针
 */
MiniPC_Instance* Minipc_Register(MiniPC_Config* config) {
    // 检查实例数量限制
    uint8_t current_count = (USB_Instance_Count == 255) ? 0 : USB_Instance_Count;
    if (current_count >= USB_MAX_INSTANCE) {
        Log_Error("Error: Maximum USB instances reached");
        return NULL;
    }
    
    // 分配内存
    MiniPC_Instance* instance = (MiniPC_Instance*)pvPortMalloc(sizeof(MiniPC_Instance));
    if(instance == NULL) {
        Log_Error("Failed to allocate memory for MiniPC_Instance");
        return NULL;
    }

    //一个比较蠢的初始化逻辑。懒得修了
    if (USB_Instance_Count == 255) {
        USB_Instance_Count = 0;
    }
    
    instance->id = USB_Instance_Count;
    instance->callback = config->callback;
    instance->message_type = config->message_type;
    instance->activate_flag = 0;
    instance->Send_Message_Type = config->Send_message_type;
    
    // 初始化接收消息结构体（只用于接收）
    if (packet_pointer != NULL) {
        instance->message = *packet_pointer;
    } else {
        memset(&instance->message, 0, sizeof(message_union));
    }
    
    // 存储实例
    minipc_instances[USB_Instance_Count] = instance;
    
    // 递增计数器
    USB_Instance_Count++;
    
    // 注册USB接收回调函数
    USB_RegisterRxCallback(USB_Data_Received_Callback);

    Log_Information("Registered MiniPC instance %d at index %d", instance->id, instance->id);
    return instance;
}

/**
 * @brief 设置指定消息类型的数据就绪标志
 * @param msg_type 消息类型
 */
static void USB_SetDataReadyFlags(uint8_t msg_type) {
    for (uint8_t i = 0; i < USB_Instance_Count+1; i++) {
        MiniPC_Instance* instance = minipc_instances[i];
        if (instance == NULL ) continue;

        // 检查这个实例是否关心这种消息类型
        
        if (instance->message_type == msg_type) {
            instance->activate_flag = 1; // 设置标志位
            instance->message = *packet_pointer; // 更新指针
            
        }else{
            instance->activate_flag = 0;
        }
        
    }
}

/**
 * @brief 从USB环形缓冲区读取数据
 * 
 * @param buffer 用于存储读取数据的缓冲区
 * @param max_len 缓冲区的最大长度
 * @return uint16_t 实际读取的字节数
 */
uint16_t Computer_Read_Data(uint8_t* buffer, uint16_t max_len) {
    uint16_t read_count = 0;
    
    if(buffer == NULL || max_len == 0) {
        return 0;
    }
    
    //TODO:可能需要临界区保护
    while(usb_rx_ring_buffer.count > 0 && read_count < max_len) {
        buffer[read_count] = usb_rx_ring_buffer.buffer[usb_rx_ring_buffer.tail];
        usb_rx_ring_buffer.tail = (usb_rx_ring_buffer.tail + 1) % USB_RX_BUFFER_SIZE;//自动绕回缓冲区开头
        usb_rx_ring_buffer.count--;
        read_count++;
    }
    
    return read_count;
}

// 检查缓冲区中是否有数据
static uint16_t Computer_Available_Data(void) {
    return usb_rx_ring_buffer.count;
}

/** 
 * @brief USB数据接收回调函数
 * 
 * @param buf 接收到的数据缓冲区
 * @param len 接收到的数据长度
 */
void USB_Data_Received_Callback(uint8_t* buf, uint32_t len) {
    if(buf == NULL || len == 0) {
        return;
    }
    
    for(uint32_t i = 0; i < len; i++) {
        if(usb_rx_ring_buffer.count < USB_RX_BUFFER_SIZE) {
            usb_rx_ring_buffer.buffer[usb_rx_ring_buffer.head] = buf[i];
            usb_rx_ring_buffer.head = (usb_rx_ring_buffer.head + 1) % USB_RX_BUFFER_SIZE;
            usb_rx_ring_buffer.count++;
        } else {
            // 缓冲区已满，可以考虑丢弃最老的数据或者报警
            // 这里选择丢弃新数据，也可以选择覆盖最老数据
            break;
        }
    }
    Data_Processing();
}
/**
 * @brief 处理接收到的USB数据
 * 
 * @note 该函数对外开放，以实现主动与被动的接受数据处理
 * @details 改进的数据包解析逻辑，避免数据丢失
 */
void Data_Processing(void) {
    static message_union packet;
    static uint8_t packet_state = 0;  // 0: 寻找包头, 1: 正在接收数据包
    static uint8_t packet_index = 0;  // 当前数据包接收索引
    uint8_t byte;
    
    while(Computer_Read_Data(&byte, 1) == 1) {
        switch(packet_state) {
            case 0: // 寻找包头
                if(byte == 's') {
                    // 清零数据包缓冲区
                    memset(packet.raw_data, 0, sizeof(packet.raw_data));
                    packet.raw_data[0] = 's';
                    packet_index = 1;
                    packet_state = 1;
                }
                break;
                
            case 1: // 接收数据包
                if(packet_index < 32) {  // 防止数组越界
                    packet.raw_data[packet_index] = byte;
                    packet_index++;

                    if(packet_index >= 32) {
                        // 接收完整个数据包，验证包尾
                        Log_Debug("Packet received: end byte = 0x%02X (should be 'e'=0x65)", packet.raw_data[31]);
                        packet_pointer=&packet;//把指针暴露出去喵
                        if(packet.raw_data[31] == 'e') {
                            // 处理接收到的有效数据包
                            Log_Passing("Valid packet received, type = 0x%02X", packet.raw_data[1]);
                            USB_SetDataReadyFlags(packet.raw_data[1]);
                        } else {
                            Log_Warning("Invalid packet: wrong end byte 0x%02X", packet.raw_data[31]);
                        }
                        // 重置状态，准备接收下一个数据包
                        packet_state = 0;
                        packet_index = 0;
                    }
                } else {
                    // 数据包长度异常，重置状态
                    Log_Warning("Packet length error, resetting state");
                    packet_state = 0;
                    packet_index = 0;
                }
                break;
        }
    }
}

/**
 * @brief 发送数据到上位机
 * 
 * @param yaw 偏航角
 * @param pitch 垂直角
 * @param color 敌方颜色
 * @param mode 模式
 * @param rune 符文标志
 * @note 如果不是哨兵，可以直接使用这个
 */
void Computer_Tx(float yaw, float pitch, uint8_t color, uint8_t mode, uint8_t rune) {
    Computer_Tx_Message_t packet;//最好改个名字,发送也可以使用union

    // 填充固定字段
    packet.start = 's';
    packet.datatype = 0xB0;
    packet.high_gimbal_yaw = yaw;
    packet.low_gimbal_yaw =0.0;
    packet.pitch = pitch;
    packet.enemy_team_color = color;
    packet.mode = mode;
    packet.rune_flag = rune;
    packet.end = 'e';

    // 清空预留区域,填充为0
    memset(packet.reserved, 0, sizeof(packet.reserved));

    // 发送数据包
    CDC_Transmit_FS((uint8_t*)&packet,32);
}

/**
 * @brief 配置自瞄发送数据源 - 简化接口
 * @param instance 实例指针
 * @param high_yaw 云台yaw角指针
 * @param pitch 俯仰角指针  
 * @param enemy_color 敌方颜色指针
 * @param mode 模式指针
 * @param rune_flag 符标志指针
 * @param low_yaw 底盘yaw角指针
 */
void Minipc_ConfigAimTx(MiniPC_Instance* instance, float* high_yaw, float* pitch, uint8_t* enemy_color, uint8_t* mode, uint8_t* rune_flag, float* low_yaw) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_AIM_TX) {
        Log_Error("Error: Invalid instance for AIM_TX");
        return;
    }
    
    // 记录指针
    instance->data_source.aim_tx.high_gimbal_yaw = high_yaw;
    instance->data_source.aim_tx.pitch = pitch;
    instance->data_source.aim_tx.enemy_team_color = enemy_color;
    instance->data_source.aim_tx.mode = mode;
    instance->data_source.aim_tx.rune_flag = rune_flag;
    instance->data_source.aim_tx.low_gimbal_yaw = low_yaw;
    
    
    Log_Information("Configured AIM_TX data source for instance %d", instance->id);
}

#ifdef SENTRY_MODE
/**
 * @brief 配置友方位置1发送数据源 
 */
void Minipc_ConfigFriendPos1Tx(MiniPC_Instance* instance, float* inf3_x, float* inf3_y, float* inf4_x, float* inf4_y, float* inf5_x, float* inf5_y) {
    if (instance == NULL || instance->message_type != USB_MSG_FRIEND1_TX) {
        Log_Error("Error: Invalid instance for FRIEND1_TX");
        return;
    }
    
    instance->data_source.friend_pos_1.infantry_3_x = inf3_x;
    instance->data_source.friend_pos_1.infantry_3_y = inf3_y;
    instance->data_source.friend_pos_1.infantry_4_x = inf4_x;
    instance->data_source.friend_pos_1.infantry_4_y = inf4_y;
    instance->data_source.friend_pos_1.infantry_5_x = inf5_x;
    instance->data_source.friend_pos_1.infantry_5_y = inf5_y;
    
    
    Log_Information("Configured FRIEND1_TX data source for instance %d", instance->id);
}
#endif
/**
 * @brief 更新单个实例的数据
 * @param instance 实例指针
 */
void Minipc_UpdateInstanceData(MiniPC_Instance* instance) {
     // 检查USB设备是否已由主机配置。如果未配置，则不发送数据。
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return; // USB未连接或未准备好，直接返回
    }

    if (instance == NULL) {
        return;
    }
    
    // 动态分配发送缓冲区
    send_union* send_buffer = (send_union*)pvPortMalloc(sizeof(send_union));
    if (send_buffer == NULL) {
        Log_Error("Failed to allocate send buffer for instance %d", instance->id);
        return;
    }
    
    // 初始化发送缓冲区
    memset(send_buffer, 0, sizeof(send_union));
    
    // 配置包头包尾
    USB_INIT_MESSAGE_PACKET(send_buffer, instance->Send_Message_Type);
    
    switch (instance->Send_Message_Type) {
        case USB_MSG_AIM_TX: {
            Computer_Tx_Message_t* msg = &send_buffer->self_aim_pack;
            USB_AimTx_DataSource_t* src = &instance->data_source.aim_tx;
            
            // 填充数据
            if (src->high_gimbal_yaw != NULL) msg->high_gimbal_yaw = *(src->high_gimbal_yaw);
            if (src->pitch != NULL) msg->pitch = *(src->pitch);
            if (src->enemy_team_color != NULL) msg->enemy_team_color = *(src->enemy_team_color);
            if (src->mode != NULL) msg->mode = *(src->mode);
            if (src->rune_flag != NULL) msg->rune_flag = *(src->rune_flag);
            if (src->low_gimbal_yaw != NULL) msg->low_gimbal_yaw = *(src->low_gimbal_yaw);
            
            Log_Debug("Sending AIM_TX: yaw=%.2f, pitch=%.2f, color=%d, mode=%d", 
                   msg->high_gimbal_yaw, msg->pitch, msg->enemy_team_color, msg->mode);
            break;
        }
        #ifdef SENTRY_MODE
        case USB_MSG_FRIEND1_TX: {
            friendly_position_1_package* msg = &send_buffer->friend_pos_1;
            USB_FriendPos1_DataSource_t* src = &instance->data_source.friend_pos_1;
            
            if (src->infantry_3_x != NULL) msg->infantry_3_x = *(src->infantry_3_x);
            if (src->infantry_3_y != NULL) msg->infantry_3_y = *(src->infantry_3_y);
            if (src->infantry_4_x != NULL) msg->infantry_4_x = *(src->infantry_4_x);
            if (src->infantry_4_y != NULL) msg->infantry_4_y = *(src->infantry_4_y);
            if (src->infantry_5_x != NULL) msg->infantry_5_x = *(src->infantry_5_x);
            if (src->infantry_5_y != NULL) msg->infantry_5_y = *(src->infantry_5_y);
            break;
        }
        #endif
        case DIY_MODE:{
            //这里可以加入自定义的回调函数处理
            instance->callback(send_buffer);
        }
        default:
            Log_Warning("Unknown Send_Message_Type: %d", instance->Send_Message_Type);
            vPortFree(send_buffer);
            return;
    }
    
    // 发送数据
    CDC_Transmit_FS((uint8_t*)send_buffer, 32);
    
    // 立即释放内存
    vPortFree(send_buffer);
}

/**
 * @brief 更新所有已配置数据源的发送实例 
 */
void Minipc_UpdateAllInstances(void) {
    Log_Information("Updating %d instances", USB_Instance_Count);
    
    
    uint8_t actual_count = (USB_Instance_Count == 0) ? 1 : USB_Instance_Count;
    
    for (uint8_t i = 0; i < actual_count; i++) {
        MiniPC_Instance* instance = minipc_instances[i];
        Log_Debug("Checking instance at index %d: %p", i, (void*)instance);
        
        if (instance != NULL) {
            if(instance->Send_Message_Type==NULL){
                Log_Warning("Instance %d has no Send_Message_Type, skipping", instance->id);
                continue;
            }
            Minipc_UpdateInstanceData(instance);
        }
    }
}





