#ifndef COM_SYSTEM_H
#define COM_SYSTEM_H
#include "Queue.h"
#include <stdint.h>
#include <stdbool.h>

// #include "RB_Tree.h"


// 参数计数宏
#define GET_ARG_COUNT(...) GET_ARG_COUNT_(__VA_ARGS__, 2, 1, 0)
#define GET_ARG_COUNT_(_1, _2, N, ...) N

// 连接宏
#define CONCAT(a, b) a##b
#define EXPAND_CONCAT(a, b) CONCAT(a, b)

#define Create_Subscriber(...) EXPAND_CONCAT(Create_Subscriber_, GET_ARG_COUNT(__VA_ARGS__))(__VA_ARGS__)

#define MAX_TOPIC_NAME_LEN 32
#define DEFAULT_QUEUE_SIZE 1


typedef struct Publisher Publisher;
typedef struct Subscriber Subscriber;

struct Subscriber{
    struct Circular_Queue_t *queue; /* 用于存储接收到的消息 */
    uint8_t data_len;
    Subscriber *next_subs; /* 指向下一个订阅者,通过链表访问所有订阅者 */
    bool is_dynamic; /* 是否为动态分配(预留接口，目前未使用) */
    bool callback_enabled; /* 是否启用回调函数 */
    void (*callback)(void*); /* 回调函数,当收到新消息时调用 */
};
struct Publisher {
    char topic_name[MAX_TOPIC_NAME_LEN + 1];
    uint8_t data_len;
    Publisher *next_pub; /* 指向下一个发布者,通过链表访问所有发布者 */
    Subscriber *first_subs; /* 指向第一个订阅了该话题的订阅者,通过链表访问所有订阅者 */
};




/// @brief 创建一个发布者
/// @param topic_name 绑定的话题名称
/// @param data_len 传输数据结构大小
/// @return 建立完成的发布者指针
Publisher* Create_Publisher(const char* topic_name, uint8_t data_len);

/// @brief 创建一个订阅者
/// @param topic_name 绑定的话题名称
/// @param data_len 传输数据结构大小
/// @return 建立完成的订阅者指针
Subscriber* Create_Subscriber_2(const char* topic_name, uint8_t data_len);

/// @brief 创建一个订阅者
/// @param topic_name  绑定的话题名称
/// @param data_len 传输数据结构大小
/// @param queue_size 指定存储消息的队列大小
/// @note 如果队列满了,会自动覆盖最旧的数据
/// @return 建立完成的订阅者指针
Subscriber* Create_Subscriber_3(const char* topic_name, uint8_t data_len , uint8_t queue_size);

/// @brief 创建一个订阅者
/// @param topic_name 绑定的话题名称
/// @param data_len 传输数据结构大小
/// @param queue_size 指定存储消息的队列大小
/// @param callback 绑定一个回调函数，接收到消息自动进行调用
/// @note 还没写完，回调还没写
/// @return 建立完成的订阅者指针
Subscriber* Create_Subscriber_4(const char* topic_name, uint8_t data_len, uint8_t queue_size, void (*callback)(void*));

/// @brief 从指定订阅者的队列中获取最新消息
/// @param subscriber 订阅者指针
/// @param data 获取数据指针
/// @return 调用是否成功
bool Get_Message(Subscriber* subscriber, void* data);

/// @brief 控制指定发布者向所有订阅者发布消息
/// @param publisher 发布者指针
/// @param data 发布数据指针
/// @return 调用是否成功
bool Publish_Message(Publisher* publisher, void* data);


static void CheckName(const char *name);
static void CheckLen(uint8_t len1, uint8_t len2);

#endif