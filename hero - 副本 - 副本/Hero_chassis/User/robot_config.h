#include"main.h"

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