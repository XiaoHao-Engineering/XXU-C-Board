/* 包含头文件 */
#include "../user_queue.h"
#include <stdlib.h>
#include <string.h>

/* 函数体 */

void Queue_Init(Queue *queue, const uint16_t max_size) {
    queue->max_size = max_size;
}

// 若队列已达到最大长度上限，则丢弃新数据
void Queue_Push(Queue *queue, const void *data, const uint16_t len) {
    if (queue->size >= queue->max_size) {
        return;
    }

    Node *newNode = (Node*)malloc(sizeof(Node));
    void *mem = malloc(len);
    memcpy(mem, data, len);

    newNode->data = mem;
    newNode->len  = len;
    newNode->next = NULL;

    if (queue->rear == NULL) {
        queue->front = queue->rear = newNode;
    } else {
        queue->rear->next = newNode;
        queue->rear = newNode;
    }

    queue->size++;
}

Node* Queue_Pop(Queue *queue) {
    if (queue->front == NULL) {
        return NULL;
    }
    
    queue->popped = queue->front;
    queue->front = queue->front->next;
    
    if (queue->front == NULL) {
        queue->rear = NULL;
    }

    queue->size--;
    return queue->popped;
}

uint8_t Queue_IsFull(const Queue *queue) {
    return queue->size >= queue->max_size;
}

uint8_t Queue_IsEmpty(const Queue *queue) {
    return queue->front == NULL;
}

uint16_t Queue_GetSize(const Queue *queue) {
    return queue->size;
}

void Queue_FreeNode(Node *node) {
    if (node == NULL)
        return;
    
    if (node->data != NULL)
        free(node->data);
    
    free(node);
}

void Queue_Free(Queue *queue) {
    while (!Queue_IsEmpty(queue)) {
        Node *node = Queue_Pop(queue);
        Queue_FreeNode(node);
    }
}
