#ifndef QUEUE_H
#define QUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

// 循环队列结构体
struct Circular_Queue_t {
    void *data;           /* 数据缓冲区 */
    uint16_t front;       /* 队头索引 */
    uint16_t rear;        /* 队尾索引 */
    uint16_t capacity;    /* 队列容量 */
    uint16_t element_size; /* 每个元素的大小 */
    uint16_t count;       /* 当前元素数量 */
};

typedef struct Circular_Queue_t Circular_Queue;

/* 创建循环队列 */
Circular_Queue* Create_Circular_Queue(uint16_t capacity, uint16_t element_size);

/* 销毁循环队列 */
void Destroy_Circular_Queue(Circular_Queue* queue);

/* 入队操作 */
bool Enqueue(Circular_Queue* queue, const void* data);

/* 出队操作 */
bool Dequeue(Circular_Queue* queue, void* data);

/* 检查队列是否为空 */
bool Is_Queue_Empty(const Circular_Queue* queue);

/* 检查队列是否已满 */
bool Is_Queue_Full(const Circular_Queue* queue);

/* 获取队列中元素数量 */
uint16_t Get_Queue_Size(const Circular_Queue* queue);

/* 清空队列 */
void Clear_Queue(Circular_Queue* queue);

#endif // QUEUE_H