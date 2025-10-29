#include "Queue.h"

/* 创建循环队列 */
Circular_Queue* Create_Circular_Queue(uint16_t capacity, uint16_t element_size) {
    if (capacity == 0 || element_size == 0) {
        return NULL;
    }
    
    Circular_Queue* queue = (Circular_Queue*)malloc(sizeof(Circular_Queue));
    if (queue == NULL) {
        return NULL;
    }
    
    // 分配数据缓冲区
    queue->data = malloc(capacity * element_size);
    if (queue->data == NULL) {
        free(queue);
        return NULL;
    }
    
    queue->front = 0;
    queue->rear = 0;
    queue->capacity = capacity;
    queue->element_size = element_size;
    queue->count = 0;
    
    return queue;
}

/* 销毁循环队列 */
void Destroy_Circular_Queue(Circular_Queue* queue) {
    if (queue != NULL) {
        if (queue->data != NULL) {
            free(queue->data);
        }
        free(queue);
    }
}

/* 入队操作 */
bool Enqueue(Circular_Queue* queue, const void* data) {
    if (queue == NULL || data == NULL) {
        return false;
    }
    
    // 检查队列是否已满
    if (Is_Queue_Full(queue)) {
        queue->front = (queue->front + 1) % queue->capacity;
        queue->count--;
    }
    
    // 计算插入位置的地址
    void* insert_pos = (char*)queue->data + (queue->rear * queue->element_size);
    
    // 拷贝数据到队列
    memcpy(insert_pos, data, queue->element_size);
    
    // 更新队尾指针和元素计数
    queue->rear = (queue->rear + 1) % queue->capacity;
    queue->count++;
    
    return true;
}

/* 出队操作 */
bool Dequeue(Circular_Queue* queue, void* data) {
    if (queue == NULL || data == NULL) {
        return false;
    }
    
    // 检查队列是否为空
    if (Is_Queue_Empty(queue)) {
        return false;
    }
    
    // 计算队头元素的地址
    void* front_pos = (char*)queue->data + (queue->front * queue->element_size);
    
    // 将队头数据拷贝到输出参数
    memcpy(data, front_pos, queue->element_size);
    
    // 更新队头指针和元素计数
    queue->front = (queue->front + 1) % queue->capacity;
    queue->count--;
    
    return true;
}

/* 检查队列是否为空 */
bool Is_Queue_Empty(const Circular_Queue* queue) {
    if (queue == NULL) {
        return true;
    }
    return (queue->count == 0);
}

/* 检查队列是否已满 */
bool Is_Queue_Full(const Circular_Queue* queue) {
    if (queue == NULL) {
        return true;
    }
    return (queue->count == queue->capacity);
}

/* 获取队列中元素数量 */
uint16_t Get_Queue_Size(const Circular_Queue* queue) {
    if (queue == NULL) {
        return 0;
    }
    return queue->count;
}

/* 清空队列 */
void Clear_Queue(Circular_Queue* queue) {
    if (queue != NULL) {
        queue->front = 0;
        queue->rear = 0;
        queue->count = 0;
    }
}