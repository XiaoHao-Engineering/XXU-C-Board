/* 包含头文件 */
#include "../user_ring_buffe.h"

/* 私有函数声明 */
static void RingBuffer_AddReadIndex(RING_BUFFER *buffer, uint16_t length);
static uint8_t RingBuffer_Read(const RING_BUFFER *buffer, uint16_t index);
static uint16_t RingBuffer_GetRemain(const RING_BUFFER *buffer);

/* 私有函数 */

static void RingBuffer_AddReadIndex(RING_BUFFER *buffer, const uint16_t length) {
    buffer->read_index = (length + buffer->read_index) % RING_BUFFER_SIZE;
}

static uint8_t RingBuffer_Read(const RING_BUFFER *buffer, const uint16_t index) {
    return buffer->buffer[index % RING_BUFFER_SIZE];
}

static uint16_t RingBuffer_GetRemain(const RING_BUFFER *buffer) {
    return RING_BUFFER_SIZE - RingBuffer_GetLength(buffer);
}

/* 函数体 */

uint16_t RingBuffer_GetLength(const RING_BUFFER *buffer) {
    return (buffer->write_index + RING_BUFFER_SIZE - buffer->read_index) % RING_BUFFER_SIZE;
}


uint16_t RingBuffer_Put(RING_BUFFER *buffer, const uint8_t *data, const uint16_t length) {
    if (RingBuffer_GetRemain(buffer) < length) {
        const uint16_t overflow = length - RingBuffer_GetRemain(buffer);
        RingBuffer_AddReadIndex(buffer, overflow);
    }
    
    // 将数据写入缓冲区
    if (buffer->write_index + length < RING_BUFFER_SIZE) {
        memcpy(buffer->buffer + buffer->write_index, data, length);
        buffer->write_index += length;
    } else {
        const uint16_t first_len = RING_BUFFER_SIZE - buffer->write_index;
        
        memcpy(buffer->buffer + buffer->write_index, data, first_len);
        memcpy(buffer->buffer, data + first_len, length - first_len);
        buffer->write_index = length - first_len;
    }
    return length;
}

uint16_t RingBuffer_GetWith_H_T(RING_BUFFER *buffer, uint8_t *data, const char *head, const char *tail) {
    const uint16_t buff_len = RingBuffer_GetLength(buffer);
    const uint16_t head_len = (uint16_t)strlen(head);
    const uint16_t tail_len = (uint16_t)strlen(tail);
    
    if (buff_len >= head_len + tail_len) {
        for (int32_t frame_start_index = buff_len - head_len - tail_len; frame_start_index >= 0; frame_start_index--) {
            uint16_t read_index = buffer->read_index + frame_start_index;

            // 匹配帧头
            uint8_t head_match = 1;
            for (uint16_t i = 0; i < head_len; i++)
                head_match &= RingBuffer_Read(buffer, read_index + i) == head[i];
            if (!head_match)
                continue;

            for (int32_t tail_index = head_len; tail_index <= buff_len - frame_start_index - tail_len; tail_index++) {

                // 匹配帧尾
                uint8_t tail_match = 1;
                for (uint16_t i = 0; i < tail_len; i++) {
                    tail_match &= RingBuffer_Read(buffer, read_index + tail_index + i) == tail[i];
                }
                if (!tail_match)
                    continue;

                const uint16_t frame_len = tail_index + tail_len;

                if (read_index >= RING_BUFFER_SIZE)
                    read_index = read_index - RING_BUFFER_SIZE;

                if (read_index + frame_len <= RING_BUFFER_SIZE)
                    memcpy(data, buffer->buffer + read_index, frame_len);
                else {
                    const uint16_t first_len = RING_BUFFER_SIZE - read_index;

                    memcpy(data, buffer->buffer + read_index, first_len);
                    memcpy(data + first_len, buffer->buffer, frame_len - first_len);
                }

                RingBuffer_AddReadIndex(buffer, frame_start_index + frame_len);
                return frame_len;
            }
        }
    }
    return 0;
}

uint16_t RingBuffer_GetWith_HT_Len(RING_BUFFER *buffer, uint8_t *data, const char *head, const char *tail, const uint16_t len) {
    const uint16_t buff_len = RingBuffer_GetLength(buffer);
    const uint16_t head_len = (uint16_t)strlen(head);
    const uint16_t tail_len = (uint16_t)strlen(tail);

    if (buff_len < len || len < head_len + tail_len)
        return 0;

    for (int32_t frame_start_index = buff_len - len; frame_start_index >= 0; frame_start_index--) {
        uint16_t read_index = buffer->read_index + frame_start_index;

        // 匹配帧头
        uint8_t head_match = 1;
        for (uint16_t i = 0; i < head_len; i++)
            head_match &= RingBuffer_Read(buffer, read_index + i) == head[i];
        if (!head_match)
            continue;

        // 匹配帧尾
        const uint16_t tail_start_offset = len - tail_len;
        uint8_t tail_match = 1;
        for (uint16_t i = 0; i < tail_len; i++)
            tail_match &= RingBuffer_Read(buffer, read_index + tail_start_offset + i) == tail[i];
        if (!tail_match)
            continue;

        uint16_t real_read_index = read_index;
        if (real_read_index >= RING_BUFFER_SIZE)
            real_read_index = real_read_index - RING_BUFFER_SIZE;

        if (real_read_index + len <= RING_BUFFER_SIZE)
            memcpy(data, buffer->buffer + real_read_index, len);
        else {
            const uint16_t first_len = RING_BUFFER_SIZE - real_read_index;

            memcpy(data, buffer->buffer + real_read_index, first_len);
            memcpy(data + first_len, buffer->buffer, len - first_len);
        }

        RingBuffer_AddReadIndex(buffer, frame_start_index + len);
        return len;
    }
    return 0;
}

uint16_t RingBuffer_GetWith_H_Len(RING_BUFFER *buffer, uint8_t *data, const char *head, const uint16_t len) {
    const uint16_t buff_len = RingBuffer_GetLength(buffer);
    const uint16_t head_len = (uint16_t)strlen(head);
    
    if (buff_len >= len) {
        for (int32_t frame_start_index = buff_len - len; frame_start_index >= 0; frame_start_index--) {
            uint16_t read_index = buffer->read_index + frame_start_index;

            // 匹配帧头
            uint8_t head_match = 1;
            for (uint16_t i = 0; i < head_len; i++)
                head_match &= RingBuffer_Read(buffer, read_index + i) == head[i];
            if (!head_match)
                continue;

            if (read_index >= RING_BUFFER_SIZE)
                read_index = read_index - RING_BUFFER_SIZE;

            if (read_index + len <= RING_BUFFER_SIZE)
                memcpy(data, buffer->buffer + read_index, len);
            else {
                const uint16_t first_len = RING_BUFFER_SIZE - read_index;

                memcpy(data, buffer->buffer + read_index, first_len);
                memcpy(data + first_len, buffer->buffer, len - first_len);
            }

            RingBuffer_AddReadIndex(buffer, frame_start_index + len);
            return len;
        }
    }
    return 0;
}


uint16_t RingBuffer_GetWith_Len(RING_BUFFER *buffer, uint8_t *data, const uint16_t len) {
    const uint16_t buff_len = RingBuffer_GetLength(buffer);
    
    if (buff_len >= len) {
        if (buffer->read_index + len <= RING_BUFFER_SIZE)
            memcpy(data, buffer->buffer + buffer->read_index, len);
        else {
            const uint16_t first_len = RING_BUFFER_SIZE - buffer->read_index;

            memcpy(data, buffer->buffer + buffer->read_index, first_len);
            memcpy(data + first_len, buffer->buffer, len - first_len);
        }

        RingBuffer_AddReadIndex(buffer, len);
        return len;
    }
    return 0;
}

uint16_t RingBuffer_GetWith_H_H(RING_BUFFER *buffer, uint8_t *data, const char *head) {
    const uint16_t buff_len = RingBuffer_GetLength(buffer);
    const uint16_t head_len = (uint16_t)strlen(head);
    
    if (buff_len >= 2 * head_len) {
        for (int32_t frame_start_index = 0; frame_start_index <= buff_len - head_len; frame_start_index++) {
            uint16_t read_index = buffer->read_index + frame_start_index;

            // 匹配第一个帧头
            uint8_t head_match = 1;
            for (uint16_t i = 0; i < head_len; i++)
                head_match &= RingBuffer_Read(buffer, read_index + i) == head[i];
            if (!head_match)
                continue;

            for (int32_t next_head_index = head_len; next_head_index <= buff_len - head_len - frame_start_index; next_head_index++) {

                // 匹配下一个帧头
                uint8_t next_head_match = 1;
                for (uint16_t i = 0; i < head_len; i++)
                    next_head_match &= RingBuffer_Read(buffer, read_index + next_head_index + i) == head[i];
                if (!next_head_match)
                    continue;

                const uint16_t frame_len = next_head_index;

                if (read_index >= RING_BUFFER_SIZE)
                    read_index = read_index - RING_BUFFER_SIZE;

                if (read_index + frame_len <= RING_BUFFER_SIZE)
                    memcpy(data, buffer->buffer + read_index, frame_len);
                else {
                    const uint16_t first_len = RING_BUFFER_SIZE - read_index;

                    memcpy(data, buffer->buffer + read_index, first_len);
                    memcpy(data + first_len, buffer->buffer, frame_len - first_len);
                }

                RingBuffer_AddReadIndex(buffer, frame_start_index + frame_len);
                return frame_len;
            }
        }
    }
    return 0;
}