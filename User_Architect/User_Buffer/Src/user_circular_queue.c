/* 包含头文件 */
#include "../user_circular_queue.h"
#include <string.h>
#include <stdlib.h>


/* 函数体 */

void Circular_Queue_Init(CIRCULAR_QUEUE *queue, const uint16_t packet_size, const uint16_t capacity) {
    queue->buffer = (uint8_t *)malloc(packet_size * capacity);

    queue->packet_size = packet_size;
    queue->capacity = capacity;
}

static uint8_t Circular_Queue_Is_Empty(const CIRCULAR_QUEUE *queue) {
    return queue->count == 0;
}

static uint8_t Circular_Queue_Is_Full(const CIRCULAR_QUEUE *queue) {
    return queue->count == queue->capacity;
}

uint16_t Circular_Queue_Get_Count(const CIRCULAR_QUEUE *queue) {
    return queue->count;
}

uint16_t Circular_Queue_Get_Free_Space(const CIRCULAR_QUEUE *queue) {
    return queue->capacity - queue->count;
}

void Circular_Queue_Enqueue(CIRCULAR_QUEUE *queue, const uint8_t *packet) {
    uint8_t *dest = queue->buffer + (queue->tail * queue->packet_size);
    memcpy(dest, packet, queue->packet_size);
    queue->tail = (queue->tail + 1) % queue->capacity;

    if (Circular_Queue_Is_Full(queue)) {
        queue->head = (queue->head + 1) % queue->capacity;
    } else {
        queue->count++;
    }
}

uint8_t Circular_Queue_Dequeue(CIRCULAR_QUEUE *queue, uint8_t *packet) {
    if (Circular_Queue_Is_Empty(queue))
        return 0;

    const uint8_t *src = queue->buffer + (queue->head * queue->packet_size);
    memcpy(packet, src, queue->packet_size);
    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;

    return 1;
}

uint8_t Circular_Queue_Discard(CIRCULAR_QUEUE *queue) {
    if (Circular_Queue_Is_Empty(queue))
        return 0;

    queue->head = (queue->head + 1) % queue->capacity;
    queue->count--;

    return 1;
}

uint8_t Circular_Queue_Peek(const CIRCULAR_QUEUE *queue, uint8_t *packet) {
    if (Circular_Queue_Is_Empty(queue))
        return 0;

    const uint8_t *src = queue->buffer + (queue->head * queue->packet_size);
    memcpy(packet, src, queue->packet_size);

    return 1;
}

void Circular_Queue_Clear(CIRCULAR_QUEUE *queue) {
    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
}
