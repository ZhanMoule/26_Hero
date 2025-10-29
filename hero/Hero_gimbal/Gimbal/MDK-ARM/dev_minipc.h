
/*ע:�����⴮��:
1.��connectivity��ѡ��USB_OTG_FS
2.��Mode��ѡ��Device_Only
3.��Middleware and Software��ѡ��USB_DEVICE
4.��Class For FS IP��ѡ�������(virtual port com���⴮��)

6.Generate Code����
*/

#ifndef DEV_MINIPC_H
#define DEV_MINIPC_H
#include <stdint.h>
#include "Com_System.h" //������Ϣ���ĵ�ͷ�ļ�

// ���ýṹ�尴1�ֽڶ���
#pragma pack(1)
//��������ڱ�ģʽ�Ļ����󲿷ִ���Ż�����
#define SENTRY_MODE
/**
 * @brief USB��Ϣ����ʼ����
 * @param struct_ptr ָ��Ҫ��ʼ���Ľṹ���ָ��
 * @param msg_type ��Ϣ���ͣ�USB_MessageType_eö��ֵ��
 * @note �ú�᣺
 *       1. ��������ṹ�壨32�ֽڣ�
 *       2. ���ð�ͷΪ 's'
 *       3. ������Ϣ����
 *       4. ���ð�βΪ 'e'
 */
#define USB_INIT_MESSAGE_PACKET(struct_ptr, msg_type) \
    do { \
        if ((struct_ptr) != NULL) { \
            /* ��������ṹ�� */ \
            memset((struct_ptr), 0, 32); \
            /* ���ð�ͷ */ \
            ((uint8_t*)(struct_ptr))[0] = 's'; \
            /* ������Ϣ���� */ \
            ((uint8_t*)(struct_ptr))[1] = (uint8_t)(msg_type); \
            /* ���ð�β */ \
            ((uint8_t*)(struct_ptr))[31] = 'e'; \
        } \
    } while(0)

/**
 * @brief USB������Ϣ����ʼ���꣨ר����send_union��
 * @param union_ptr ָ��send_union��ָ��
 * @param msg_type ��Ϣ���ͣ�USB_MessageType_eö��ֵ��
 */
#define USB_INIT_SEND_PACKET(union_ptr, msg_type) \
    USB_INIT_MESSAGE_PACKET(union_ptr, msg_type)

    
// ��Ϣ����ö�ٶ���
typedef enum {
    // ������Ϣ����
    USB_MSG_AIM_RX = 0xA0,          // �������ݽ���
    #ifdef SENTRY_MODE
    USB_MSG_CHASSIS_RX = 0xA1,      // �������ݽ���  
    USB_MSG_GAME_RX = 0xA2,         // ������Ϣ����
    USB_MSG_MODULE_RX = 0xA3,       // ģ����Ϣ����
    #endif
    
    // ������Ϣ����
    USB_MSG_AIM_TX = 0xB0,          // ������̨����
    #ifdef SENTRY_MODE
    USB_MSG_FRIEND1_TX = 0xB1,      // �ҷ�������λ��1
    USB_MSG_FRIEND2_TX = 0xB2,      // �ҷ�������λ��2
    USB_MSG_ENEMY1_TX = 0xB3,       // �з�������λ��1
    USB_MSG_ENEMY2_TX = 0xB4,       // �з�������λ��2
    USB_MSG_RED_HP_TX = 0xB5,       // �췽Ѫ��
    USB_MSG_BLUE_HP_TX = 0xB6,      // ����Ѫ��
    USB_MSG_BUILD_HP_TX = 0xB7,     // ����Ѫ��
    USB_MSG_GAME_INFO_TX = 0xB8,    // ������Ϣ
    USB_MSG_OP_FB_TX = 0xB9,        // ��������
    USB_MSG_HIT_FB_TX = 0xBA,       // �ܻ�����
    USB_MSG_LAUNCH_TX = 0xBB,       // ����״̬
    #endif
    // ����ֵ
    USB_MSG_UNKNOWN = 0xFF,          // δ֪��Ϣ����
    DIY_MODE = 0xFE                   // DIYģʽ
} USB_MessageType_e;




// ����������ݰ��ṹ��32�ֽڹ̶����ȣ�
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xA0
    uint8_t find_bool;         // 2 �Ƿ�׷��
    float yaw;                 // 3 - 6 ƫ����
    float pitch;               // 7 - 10 ������
    uint8_t reserved[20];      // 11 - 30 Ԥ����λ�����0��
    char end;                  // 31 ֡β��ȡ 'e'
} Computer_Rx_Message_t;

//����ͨ�ţ��Ϸ��£�
typedef struct {
    char start;
    char datatype;             // 1 ��Ϣ���� 0xA1
    float x_speed;             // 2 - 5 x�����ٶ�
    float y_speed;             // 6 - 9 y�����ٶ�
    float yaw;                 // 10 - 13 yaw
    uint8_t reserved[17];      // 14 - 30 Ԥ����λ�����0��
    char end;                  // 31 ֡β��ȡ 'e'
} Chassis_package;
#ifdef SENTRY_MODE
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xA2
    uint8_t type;              // 2 ����  0����  1������  2������
    uint8_t content;           // 3 ��������
    uint8_t reserved[26];      // 4 - 30 Ԥ����λ�����0��
    char end;                  // 31 ֡β��ȡ 'e'
} competition_package;

typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xA3
    uint16_t type;             // 2 ����  0����  1��С����  2����̨����������
    uint16_t content;          // 4 ��������
    uint8_t reserved[24];      // 6 - 30 Ԥ����λ�����0��
    char end;                  // 31 ֡β��ȡ 'e'
} modules_package;



// 0xB1 �ҷ�������λ����Ϣ����1
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB1
    float infantry_3_x;        // 2 - 5 ���� 3 �Ų���������λ�� x ������
    float infantry_3_y;        // 6 - 9 ���� 3 �Ų���������λ�� y ������
    float infantry_4_x;        // 10 - 13 ���� 4 �Ų���������λ�� x ������
    float infantry_4_y;        // 14 - 17 ���� 4 �Ų���������λ�� y ������
    float infantry_5_x;        // 18 - 21 ���� 5 �Ų���������λ�� x ������
    float infantry_5_y;        // 22 - 25 ���� 5 �Ų���������λ�� y ������
    uint8_t reserved[4];       // 26 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} friendly_position_1_package;

// 0xB2 �ҷ�������λ����Ϣ����2
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB2
    float hero_x;              // 2 - 5 ���� 1 ��Ӣ�ۻ�����λ�� x ������
    float hero_y;              // 6 - 9 ���� 1 ��Ӣ�ۻ�����λ�� y ������
    float engineer_x;          // 10 - 13 ���� 2 �Ź��̻�����λ�� x ������
    float engineer_y;          // 14 - 17 ���� 2 �Ź��̻�����λ�� y ������
    float sentinal_x;          // 18 - 21 ���� 7 ���ڱ�������λ�� x ������
    float sentinal_y;          // 22 - 25 ���� 7 ���ڱ�������λ�� y ������
    uint8_t reserved[4];       // 26 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} friendly_position_2_package;

// 0xB3 �з�������λ��1
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB3
    float enemy_infantry_3_x;  // 2 - 5 �з� 3 �Ų���������λ�� x ������
    float enemy_infantry_3_y;  // 6 - 9 �з� 3 �Ų���������λ�� y ������
    float enemy_infantry_4_x;  // 10 - 13 �з� 4 �Ų���������λ�� x ������
    float enemy_infantry_4_y;  // 14 - 17 �з� 4 �Ų���������λ�� y ������
    float enemy_infantry_5_x;  // 18 - 21 �з� 5 �Ų���������λ�� x ������
    float enemy_infantry_5_y;  // 22 - 25 �з� 5 �Ų���������λ�� y ������
    uint8_t reserved[4];       // 26 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} enemy_position_1_package;

// 0xB4 �з�������λ��2
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB4
    float enemy_infantry_1_x;  // 2 - 5 �з� 1 ��Ӣ�ۻ�����λ�� x ������
    float enemy_infantry_1_y;  // 6 - 9 �з� 1 ��Ӣ�ۻ�����λ�� y ������
    float enemy_infantry_2_x;  // 10 - 13 �з� 2 �Ź��̻�����λ�� x ������
    float enemy_infantry_2_y;  // 14 - 17 �з� 2 �Ź��̻�����λ�� y ������
    float enemy_infantry_7_x;  // 18 - 21 �з� 7 ���ڱ�������λ�� x ������
    float enemy_infantry_7_y;  // 22 - 25 �з� 7 ���ڱ�������λ�� y ������
    uint8_t reserved[4];       // 26 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} enemy_position_2_package;

// 0xB5 �췽Ѫ��
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB5
    uint16_t red_1_robot_HP;   // 2 - 3 �� 1 Ӣ�ۻ�����Ѫ��
    uint16_t red_2_robot_HP;   // 4 - 5 �� 2 ���̻�����Ѫ��
    uint16_t red_3_robot_HP;   // 6 - 7 �� 3 ����������Ѫ��
    uint16_t red_4_robot_HP;   // 8 - 9 �� 4 ����������Ѫ��
    uint16_t red_5_robot_HP;   // 10 - 11 �� 5 ����������Ѫ��
    uint16_t red_7_robot_HP;   // 12 - 13 �� 7 �ڱ�������Ѫ��
    uint8_t reserved[16];      // 14 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} red_hp_package;

// 0xB6 ����Ѫ��
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB6
    uint16_t blue_1_robot_HP;  // 2 - 3 �� 1 Ӣ�ۻ�����Ѫ��
    uint16_t blue_2_robot_HP;  // 4 - 5 �� 2 ���̻�����Ѫ��
    uint16_t blue_3_robot_HP;  // 6 - 7 �� 3 ����������Ѫ��
    uint16_t blue_4_robot_HP;  // 8 - 9 �� 4 ����������Ѫ��
    uint16_t blue_5_robot_HP;  // 10 - 11 �� 5 ����������Ѫ��
    uint16_t blue_7_robot_HP;  // 12 - 13 �� 7 �ڱ�������Ѫ��
    uint8_t reserved[16];      // 14 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} blue_hp_package;

// 0xB7 ����Ѫ��
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB7
    uint16_t red_outpost_HP;   // 2 - 3  �췽ǰ��վѪ��
    uint16_t red_base_HP;      // 4 - 5  �췽����Ѫ��
    uint16_t blue_outpost_HP;  // 6 - 7 ����ǰ��վѪ��
    uint16_t blue_base_HP;     // 8 - 9 ��������Ѫ��
    uint8_t reserved[20];      // 10 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} building_hp_package;

// 0xB8 ������Ϣ
typedef struct {
    char start;                   // 0 ֡ͷ��ȡ 's'
    char datatype;                // 1 ��Ϣ���� 0xB8
    uint8_t enemy_team_color;     // 2 �з���ɫ 0���� 1����
    uint8_t game_progress;        // 3 ��ǰ�����׶�
    uint16_t stage_remain_time;   // 4 - 5 ��ǰ�׶�ʣ��ʱ��
    uint16_t remaining_gold_coin; // 6 - 7 ʣ��������
    uint8_t reserved[22];         // 8 - 30 Ԥ����λ
    char end;                     // 31 ֡β��ȡ 'e'
} game_info_package;

// 0xB9 ��������
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB9
    float target_position_x;   // 2 - 5 Ŀ��λ�� x ������
    float target_position_y;   // 6 - 9 Ŀ��λ�� y ������
    uint8_t cmd_keyboard;      // 10 ������Ϣ
    uint8_t target_robot_id;   // 11 Ŀ�������id
    uint8_t reserved[18];      // 12 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} operation_feedback_package;

// 0xBA �ܻ�����
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xBA
    uint8_t hitted;            // 2 �ܻ����ʶ
    uint8_t reserved[27];      // 3 - 30 Ԥ����λ
    char end;                  // 31 ֡β��ȡ 'e'
} hit_feedback_package;

// 0xBB ����״̬��
typedef struct {
    char start;                         // 0 ֡ͷ��ȡ 's'
    char datatype;                      // 1 ��Ϣ���� 0xBB
    uint16_t projectile_allowance_17mm; // 2 - 3 17mm��������
    uint16_t projectile_allowance_42mm; // 4 - 5 42mm��������
    uint8_t real_heat;                  // 6 ����������� 
    uint8_t launching_frequency;        // 7 ��������(��λ:Hz)
    uint8_t reserved[22];               // 8 - 30 Ԥ����λ
    char end;                           // 31 ֡β��ȡ 'e'
} launch_status_package;
#endif
typedef union {
    Computer_Rx_Message_t norm_aim_pack;        // ����
    #ifdef SENTRY_MODE
    Chassis_package ch_pack;                    // ����

    competition_package com_pack;               // ������Ϣ
    modules_package mod_pack;                   // ģ����Ϣ
    #endif
    uint8_t raw_data[32];                       // ���ڷ�������Ĵ���
} message_union;

// ����Դָ��ṹ��

typedef struct {
    float* high_gimbal_yaw;
    float* pitch;
    uint8_t* enemy_team_color;
    uint8_t* mode;
    uint8_t* rune_flag;
    float* low_gimbal_yaw;
} USB_AimTx_DataSource_t;
#ifdef SENTRY_MODE
typedef struct {
    float* infantry_3_x;
    float* infantry_3_y;
    float* infantry_4_x;
    float* infantry_4_y;
    float* infantry_5_x;
    float* infantry_5_y;
} USB_FriendPos1_DataSource_t;

typedef struct {
    uint16_t* red_1_robot_HP;
    uint16_t* red_2_robot_HP;
    uint16_t* red_3_robot_HP;
    uint16_t* red_4_robot_HP;
    uint16_t* red_5_robot_HP;
    uint16_t* red_7_robot_HP;
} USB_RedHP_DataSource_t;

typedef struct {
    uint8_t* enemy_team_color;
    uint8_t* game_progress;
    uint16_t* stage_remain_time;
    uint16_t* remaining_gold_coin;
} USB_GameInfo_DataSource_t;
#endif
// ͳһ������Դ������
typedef union {
    USB_AimTx_DataSource_t aim_tx;
    #ifdef SENTRY_MODE
    USB_FriendPos1_DataSource_t friend_pos_1;
    USB_RedHP_DataSource_t red_hp;
    USB_GameInfo_DataSource_t game_info;
    #endif
} USB_DataSource_Union_t;

// ���巢�����ݰ��ṹ��32�ֽڹ̶����ȣ�
typedef struct {
    char start;                // 0 ֡ͷ��ȡ 's'
    char datatype;             // 1 ��Ϣ���� 0xB0
    float high_gimbal_yaw;     // 2 - 5 С��̨ƫ����
    float pitch;               // 6 - 9 ������
    uint8_t enemy_team_color;  // 10 �з���ɫ 0���� 1����
    uint8_t mode;              // 11 ģʽ 0������ 1����
    uint8_t rune_flag;         // 12 ��ģʽ '0':���ɼ��� '1':С�� '2':���
    float low_gimbal_yaw;      // 13 - 16 ����̨ƫ����
    uint8_t reserved[14];      // 17 - 30 Ԥ����λ�����0��
    char end;                  // 31 ֡β��ȡ 'e'
} Computer_Tx_Message_t;

typedef union {
    Computer_Tx_Message_t self_aim_pack;        // ����
    #ifdef SENTRY_MODE
    friendly_position_1_package friend_pos_1;   // �ҷ�λ��1
    friendly_position_2_package friend_pos_2;   // �ҷ�λ��2
    enemy_position_1_package enemy_pos_1;       // �з�λ��1
    enemy_position_2_package enemy_pos_2;       // �з�λ��2
    red_hp_package red_hp;                      // �췽Ѫ��
    blue_hp_package blue_hp;                    // ����Ѫ��
    building_hp_package building_hp;            // ����Ѫ��
    game_info_package game_info;                // ������Ϣ
    operation_feedback_package op_feedback;     // ��������
    hit_feedback_package hit_feedback;          // �ܻ�����
    launch_status_package launch_status;        // ����״̬
    #endif
    uint8_t raw_data[32];                       // ���ڷ�������Ĵ���
} send_union;
typedef void (*CallbackFunc)(send_union* send_buffer);
// �޸����ýṹ�壬ʹ��ö������
typedef struct
{
    message_union message_config;               // �������ݵ���Դ����
    USB_MessageType_e message_type;             // ��Ҫ����������
    USB_MessageType_e Send_message_type; // ������������
    CallbackFunc callback;                      // ���������ݸ��»ص�����
} MiniPC_Config;
//USBͨ��ʵ��
//����ʱ��ʹ��Update�������ûص���������message�����뷢������Ȼ����÷��ͺ���
//������ʱ�����ý��պ���,message�л������������
typedef struct
{
    uint8_t id;                                 // USBͨ��ʵ��ID
    USB_MessageType_e message_type;             // ��Ϣ����
    CallbackFunc callback;
    message_union message;                      // 32λ���ݰ�
    uint8_t activate_flag;                      // �����־
    USB_DataSource_Union_t data_source;         // ����Դָ��
    USB_MessageType_e Send_Message_Type;    // ������Ϣ����
} MiniPC_Instance;


// �򻯵����ú�������
void Minipc_ConfigAimTx(MiniPC_Instance* instance, float* high_yaw, float* pitch, uint8_t* enemy_color, uint8_t* mode, uint8_t* rune_flag, float* low_yaw);

#ifdef SENTRY_MODE
void Minipc_ConfigFriendPos1Tx(MiniPC_Instance* instance, float* inf3_x, float* inf3_y, float* inf4_x, float* inf4_y, float* inf5_x, float* inf5_y);
void Minipc_ConfigRedHPTx(MiniPC_Instance* instance, uint16_t* red1_hp, uint16_t* red2_hp, uint16_t* red3_hp, uint16_t* red4_hp, uint16_t* red5_hp, uint16_t* red7_hp);
void Minipc_ConfigGameInfoTx(MiniPC_Instance* instance, uint8_t* enemy_color, uint8_t* game_progress, uint16_t* remain_time, uint16_t* gold_coin);
#endif
// ͳһ�ĸ��ºͷ��ͺ���
void Minipc_UpdateAllInstances(void);
//���ڵ������µĺ���
void Minipc_UpdateInstanceData(MiniPC_Instance* instance);



MiniPC_Instance* Minipc_Register(MiniPC_Config* config);
void USB_Data_Received_Callback(uint8_t* buf, uint32_t len);
void Data_Processing(void);



// �ָ�Ĭ�϶��뷽ʽ
#pragma pack()

#endif //DEV_MINIPC_H