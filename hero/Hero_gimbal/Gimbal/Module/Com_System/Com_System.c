#include "Com_System.h"
Circular_Queue* cir666;

static Publisher Publisher_Root={
    .topic_name = "Root",
    .data_len = 0,
    .first_subs = NULL,
    .next_pub = NULL,
};


Publisher* Create_Publisher(const char* topic_name, uint8_t data_len) {
    CheckName(topic_name);
    Publisher *node = &Publisher_Root;
    while (node->next_pub != NULL) {
        node = node->next_pub;
        if (strcmp(node->topic_name, topic_name) == 0) // 如果已经注册了相同的话题,直接返回结点指针
        {   
            CheckLen(node->data_len, data_len);
            return node;
        }
    }
    node->next_pub = (Publisher *)malloc(sizeof(Publisher));
    memset(node->next_pub, 0, sizeof(Publisher));
    node->next_pub->data_len = data_len;
    strcpy(node->next_pub->topic_name, topic_name);
    return node->next_pub;
}


//TODO: 没写完捏
Subscriber* Create_Subscriber_2(const char* topic_name, uint8_t data_len) {
    Publisher *current_topic_pub = Create_Publisher(topic_name, data_len);
    Subscriber * sub = (Subscriber *)malloc(sizeof(Subscriber));
    memset(sub, 0, sizeof(Subscriber));
    sub->data_len = data_len;
    //队列处理
    sub->queue = Create_Circular_Queue(DEFAULT_QUEUE_SIZE, data_len);
    sub->is_dynamic = false; // 目前未使用
    sub->callback_enabled = false; 
    sub->callback = NULL; 

    if (current_topic_pub->first_subs == NULL) {
        current_topic_pub->first_subs = sub; // 如果是第一个订阅者,直接赋值
        return sub;
    } else {
        Subscriber *node = current_topic_pub->first_subs;
        while (node->next_subs != NULL) {
            node = node->next_subs; // 找到最后一个订阅者
        }
        node->next_subs = sub;
        return sub; // 将新订阅者添加到链表末尾
    }
}


Subscriber* Create_Subscriber_3(const char* topic_name, uint8_t data_len, uint8_t queue_size) {
    Publisher *current_topic_pub = Create_Publisher(topic_name, data_len);
    Subscriber * sub = (Subscriber *)malloc(sizeof(Subscriber));
    memset(sub, 0, sizeof(Subscriber));
    sub->data_len = data_len;
    //队列处理
    sub->queue = Create_Circular_Queue(queue_size, data_len);
    sub->is_dynamic = false; // 目前未使用
    sub->callback_enabled = false; 
    sub->callback = NULL; 

    if (current_topic_pub->first_subs == NULL) {
        current_topic_pub->first_subs = sub; // 如果是第一个订阅者,直接赋值
        return sub;
    } else {
        Subscriber *node = current_topic_pub->first_subs;
        while (node->next_subs != NULL) {
            node = node->next_subs; // 找到最后一个订阅者
        }
        node->next_subs = sub;
        return sub; // 将新订阅者添加到链表末尾
    }
}



Subscriber* Create_Subscriber_4(const char* topic_name, uint8_t data_len, uint8_t queue_size, void (*callback)(void*)) {
    Publisher *current_topic_pub = Create_Publisher(topic_name, data_len);
    Subscriber * sub = (Subscriber *)malloc(sizeof(Subscriber));
    memset(sub, 0, sizeof(Subscriber));
    sub->data_len = data_len;
    //队列处理
    sub->queue = Create_Circular_Queue(queue_size, data_len);
    sub->is_dynamic = false; // 目前未使用
    sub->callback_enabled = true; 
    sub->callback = callback; // 设置回调函数

    if (current_topic_pub->first_subs == NULL) {
        current_topic_pub->first_subs = sub; // 如果是第一个订阅者,直接赋值
        return sub;
    } else {
        Subscriber *node = current_topic_pub->first_subs;
        while (node->next_subs != NULL) {
            node = node->next_subs; // 找到最后一个订阅者
        }
        node->next_subs = sub;
        return sub; // 将新订阅者添加到链表末尾
    }
}


bool Get_Message(Subscriber* subscriber, void* data) {
    return Dequeue(subscriber->queue,data);
}


bool Publish_Message(Publisher* publisher, void* data) {
    static Subscriber *iter;
    iter = publisher->first_subs;
    while(iter){
        if (Enqueue(iter->queue, data)){
            iter = iter->next_subs;
        } 
        else{
            return false;
        }
    }
    return true;
}
static void CheckName(const char *name)
{
    if (strnlen(name, MAX_TOPIC_NAME_LEN + 1) >= MAX_TOPIC_NAME_LEN)
    {
        while (1)
            ; // 进入这里说明话题名超出长度限制
    }
}

static void CheckLen(uint8_t len1, uint8_t len2)
{
    if (len1 != len2)
    {
        while (1)
            ; // 进入这里说明相同话题的消息长度却不同
    }
}
