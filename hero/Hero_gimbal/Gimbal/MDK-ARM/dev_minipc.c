/**
 * @file dev_minipc.c
 * @brief ����λ��ͨ��
 * 
 * @details ��ģ��ʵ��������λ��֮���ͨ��Э�飬�������ݰ��Ķ���ͷ��ͽ��պ�����
 * @note ʵ����ģ��ʵ���˲����������ݰ����պͽ��������Ƿְ��Ƚ��鷳������������Ϊ��32���ֽ�
 *
 * @author CGH
 * @editor HeWenXuan
 */


//�δ�������������ޣ�

/**
 * ���������̣�
 * �������뻺�����������жϣ�ת�Ƶ����λ�������������ȡ���ݣ�ת�Ƶ��ṹ�����Ϣ����
 */
#include <stdint.h>
#include "dev_minipc.h"
#include "usbd_cdc_if.h"
#include <string.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "bsp_log.h"
#define USB_RX_BUFFER_SIZE 32//64���ֽڣ�2��������
#define USB_MAX_INSTANCE 8//��������ʵ������



//���λ�����,�����ݵ�����Զ����浽�����������

typedef struct {
    uint8_t buffer[USB_RX_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} USB_RingBuffer_t;
extern USBD_HandleTypeDef hUsbDeviceFS; 
USB_RingBuffer_t usb_rx_ring_buffer = {0};
// static Publisher *USB_Topics[USB_MAX_INSTANCE] = {NULL};
static uint8_t USB_Instance_Count = 255;//��ǰUSBʵ������
// ���ļ������������޸ı������ͣ�
static message_union* packet_pointer = NULL;  // ָ������
MiniPC_Instance* minipc_instances[USB_MAX_INSTANCE] = {NULL};

/**
 * @brief ע��MiniPCʵ��
 * 
 * @param config MiniPC����
 * @return MiniPC_Instance* ע���ʵ��ָ��
 */
MiniPC_Instance* Minipc_Register(MiniPC_Config* config) {
    // ���ʵ����������
    uint8_t current_count = (USB_Instance_Count == 255) ? 0 : USB_Instance_Count;
    if (current_count >= USB_MAX_INSTANCE) {
        Log_Error("Error: Maximum USB instances reached");
        return NULL;
    }
    
    // �����ڴ�
    MiniPC_Instance* instance = (MiniPC_Instance*)pvPortMalloc(sizeof(MiniPC_Instance));
    if(instance == NULL) {
        Log_Error("Failed to allocate memory for MiniPC_Instance");
        return NULL;
    }

    //һ���Ƚϴ��ĳ�ʼ���߼�����������
    if (USB_Instance_Count == 255) {
        USB_Instance_Count = 0;
    }
    
    instance->id = USB_Instance_Count;
    instance->callback = config->callback;
    instance->message_type = config->message_type;
    instance->activate_flag = 0;
    instance->Send_Message_Type = config->Send_message_type;
    
    // ��ʼ��������Ϣ�ṹ�壨ֻ���ڽ��գ�
    if (packet_pointer != NULL) {
        instance->message = *packet_pointer;
    } else {
        memset(&instance->message, 0, sizeof(message_union));
    }
    
    // �洢ʵ��
    minipc_instances[USB_Instance_Count] = instance;
    
    // ����������
    USB_Instance_Count++;
    
    // ע��USB���ջص�����
    USB_RegisterRxCallback(USB_Data_Received_Callback);

    Log_Information("Registered MiniPC instance %d at index %d", instance->id, instance->id);
    return instance;
}

/**
 * @brief ����ָ����Ϣ���͵����ݾ�����־
 * @param msg_type ��Ϣ����
 */
static void USB_SetDataReadyFlags(uint8_t msg_type) {
    for (uint8_t i = 0; i < USB_Instance_Count+1; i++) {
        MiniPC_Instance* instance = minipc_instances[i];
        if (instance == NULL ) continue;

        // ������ʵ���Ƿ����������Ϣ����
        
        if (instance->message_type == msg_type) {
            instance->activate_flag = 1; // ���ñ�־λ
            instance->message = *packet_pointer; // ����ָ��
            
        }else{
            instance->activate_flag = 0;
        }
        
    }
}

/**
 * @brief ��USB���λ�������ȡ����
 * 
 * @param buffer ���ڴ洢��ȡ���ݵĻ�����
 * @param max_len ����������󳤶�
 * @return uint16_t ʵ�ʶ�ȡ���ֽ���
 */
uint16_t Computer_Read_Data(uint8_t* buffer, uint16_t max_len) {
    uint16_t read_count = 0;
    
    if(buffer == NULL || max_len == 0) {
        return 0;
    }
    
    //TODO:������Ҫ�ٽ�������
    while(usb_rx_ring_buffer.count > 0 && read_count < max_len) {
        buffer[read_count] = usb_rx_ring_buffer.buffer[usb_rx_ring_buffer.tail];
        usb_rx_ring_buffer.tail = (usb_rx_ring_buffer.tail + 1) % USB_RX_BUFFER_SIZE;//�Զ��ƻػ�������ͷ
        usb_rx_ring_buffer.count--;
        read_count++;
    }
    
    return read_count;
}

// ��黺�������Ƿ�������
static uint16_t Computer_Available_Data(void) {
    return usb_rx_ring_buffer.count;
}

/** 
 * @brief USB���ݽ��ջص�����
 * 
 * @param buf ���յ������ݻ�����
 * @param len ���յ������ݳ���
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
            // ���������������Կ��Ƕ������ϵ����ݻ��߱���
            // ����ѡ���������ݣ�Ҳ����ѡ�񸲸���������
            break;
        }
    }
    Data_Processing();
}
/**
 * @brief ������յ���USB����
 * 
 * @note �ú������⿪�ţ���ʵ�������뱻���Ľ������ݴ���
 * @details �Ľ������ݰ������߼����������ݶ�ʧ
 */
void Data_Processing(void) {
    static message_union packet;
    static uint8_t packet_state = 0;  // 0: Ѱ�Ұ�ͷ, 1: ���ڽ������ݰ�
    static uint8_t packet_index = 0;  // ��ǰ���ݰ���������
    uint8_t byte;
    
    while(Computer_Read_Data(&byte, 1) == 1) {
        switch(packet_state) {
            case 0: // Ѱ�Ұ�ͷ
                if(byte == 's') {
                    // �������ݰ�������
                    memset(packet.raw_data, 0, sizeof(packet.raw_data));
                    packet.raw_data[0] = 's';
                    packet_index = 1;
                    packet_state = 1;
                }
                break;
                
            case 1: // �������ݰ�
                if(packet_index < 32) {  // ��ֹ����Խ��
                    packet.raw_data[packet_index] = byte;
                    packet_index++;

                    if(packet_index >= 32) {
                        // �������������ݰ�����֤��β
                        Log_Debug("Packet received: end byte = 0x%02X (should be 'e'=0x65)", packet.raw_data[31]);
                        packet_pointer=&packet;//��ָ�뱩¶��ȥ��
                        if(packet.raw_data[31] == 'e') {
                            // ������յ�����Ч���ݰ�
                            Log_Passing("Valid packet received, type = 0x%02X", packet.raw_data[1]);
                            USB_SetDataReadyFlags(packet.raw_data[1]);
                        } else {
                            Log_Warning("Invalid packet: wrong end byte 0x%02X", packet.raw_data[31]);
                        }
                        // ����״̬��׼��������һ�����ݰ�
                        packet_state = 0;
                        packet_index = 0;
                    }
                } else {
                    // ���ݰ������쳣������״̬
                    Log_Warning("Packet length error, resetting state");
                    packet_state = 0;
                    packet_index = 0;
                }
                break;
        }
    }
}

/**
 * @brief �������ݵ���λ��
 * 
 * @param yaw ƫ����
 * @param pitch ��ֱ��
 * @param color �з���ɫ
 * @param mode ģʽ
 * @param rune ���ı�־
 * @note ��������ڱ�������ֱ��ʹ�����
 */
void Computer_Tx(float yaw, float pitch, uint8_t color, uint8_t mode, uint8_t rune) {
    Computer_Tx_Message_t packet;//��øĸ�����,����Ҳ����ʹ��union

    // ���̶��ֶ�
    packet.start = 's';
    packet.datatype = 0xB0;
    packet.high_gimbal_yaw = yaw;
    packet.low_gimbal_yaw =0.0;
    packet.pitch = pitch;
    packet.enemy_team_color = color;
    packet.mode = mode;
    packet.rune_flag = rune;
    packet.end = 'e';

    // ���Ԥ������,���Ϊ0
    memset(packet.reserved, 0, sizeof(packet.reserved));

    // �������ݰ�
    CDC_Transmit_FS((uint8_t*)&packet,32);
}

/**
 * @brief �������鷢������Դ - �򻯽ӿ�
 * @param instance ʵ��ָ��
 * @param high_yaw ��̨yaw��ָ��
 * @param pitch ������ָ��  
 * @param enemy_color �з���ɫָ��
 * @param mode ģʽָ��
 * @param rune_flag ����־ָ��
 * @param low_yaw ����yaw��ָ��
 */
void Minipc_ConfigAimTx(MiniPC_Instance* instance, float* high_yaw, float* pitch, uint8_t* enemy_color, uint8_t* mode, uint8_t* rune_flag, float* low_yaw) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_AIM_TX) {
        
        return;
    }
    
    // ��¼ָ��
    instance->data_source.aim_tx.high_gimbal_yaw = high_yaw;
    instance->data_source.aim_tx.pitch = pitch;
    instance->data_source.aim_tx.enemy_team_color = enemy_color;
    instance->data_source.aim_tx.mode = mode;
    instance->data_source.aim_tx.rune_flag = rune_flag;
    instance->data_source.aim_tx.low_gimbal_yaw = low_yaw;
    
    
    
}

#ifdef SENTRY_MODE
/**
 * @brief �����ѷ�λ��1��������Դ 
 */
void Minipc_ConfigFriendPos1Tx(MiniPC_Instance* instance, float* inf3_x, float* inf3_y, float* inf4_x, float* inf4_y, float* inf5_x, float* inf5_y) {
    if (instance == NULL || instance->message_type != USB_MSG_FRIEND1_TX) {
        
        return;
    }
    
    instance->data_source.friend_pos_1.infantry_3_x = inf3_x;
    instance->data_source.friend_pos_1.infantry_3_y = inf3_y;
    instance->data_source.friend_pos_1.infantry_4_x = inf4_x;
    instance->data_source.friend_pos_1.infantry_4_y = inf4_y;
    instance->data_source.friend_pos_1.infantry_5_x = inf5_x;
    instance->data_source.friend_pos_1.infantry_5_y = inf5_y;
    
    
    
}
#endif
/**
 * @brief ���µ���ʵ��������
 * @param instance ʵ��ָ��
 */
void Minipc_UpdateInstanceData(MiniPC_Instance* instance) {
     // ���USB�豸�Ƿ������������á����δ���ã��򲻷������ݡ�
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return; // USBδ���ӻ�δ׼���ã�ֱ�ӷ���
    }

    if (instance == NULL) {
        return;
    }
    
    // ��̬���䷢�ͻ�����
    send_union* send_buffer = (send_union*)pvPortMalloc(sizeof(send_union));
    if (send_buffer == NULL) {
        
        return;
    }
    
    // ��ʼ�����ͻ�����
    memset(send_buffer, 0, sizeof(send_union));
    
    // ���ð�ͷ��β
    USB_INIT_MESSAGE_PACKET(send_buffer, instance->Send_Message_Type);
    
    switch (instance->Send_Message_Type) {
        case USB_MSG_AIM_TX: {
            Computer_Tx_Message_t* msg = &send_buffer->self_aim_pack;
            USB_AimTx_DataSource_t* src = &instance->data_source.aim_tx;
            
            // �������
            if (src->high_gimbal_yaw != NULL) msg->high_gimbal_yaw = *(src->high_gimbal_yaw);
            if (src->pitch != NULL) msg->pitch = *(src->pitch);
            if (src->enemy_team_color != NULL) msg->enemy_team_color = *(src->enemy_team_color);
            if (src->mode != NULL) msg->mode = *(src->mode);
            if (src->rune_flag != NULL) msg->rune_flag = *(src->rune_flag);
            if (src->low_gimbal_yaw != NULL) msg->low_gimbal_yaw = *(src->low_gimbal_yaw);
            
            
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
            //������Լ����Զ���Ļص���������
            instance->callback(send_buffer);
        }
        default:
            
            vPortFree(send_buffer);
            return;
    }
    
    // ��������
    CDC_Transmit_FS((uint8_t*)send_buffer, 32);
    
    // �����ͷ��ڴ�
    vPortFree(send_buffer);
}

/**
 * @brief ������������������Դ�ķ���ʵ�� 
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