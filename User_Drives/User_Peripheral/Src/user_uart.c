#include "../../../Core/Inc/bsp.h"
#ifdef HAL_UART_MODULE_ENABLED
/* 包含头文件 */
#include "../user_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

/* 私有变量 */
static UART_DRIVES *uart_drives[UART_NUM];
static uint8_t uart_num = 0;
static uint8_t is_init_loop_event_sign = 0;

/* 私有函数 */

// 处理接收和发送队列，该函数会自动注册在全局注册表
static void UART_QueueHandle(void) {
    for (uint8_t uart_index = 0; uart_index < uart_num; uart_index++) {
        UART_DRIVES *uart = uart_drives[uart_index];

        // 处理接收队列
        if (RingBuffer_GetLength(&uart->rx_ringBuffer) && uart->receive_new_data == 1) {
            for (uint8_t i = 0; i < uart->callback_num; i++) {
                uart->callbacks[i](uart);
            }
            uart->receive_new_data = 0;
        }

        // 处理发送队列
        /* 不用自己维护的 uart->status 判断忙闲：那个标志只能靠 TxCplt 回调复位，
           一旦回调丢了就永久卡在 SENDING，现象是"只发得出第一帧就再没数据"。
           直接问 HAL 的 gState —— RX 正在收不影响 TX 的 gState。
           另外发送失败时要把节点释放掉，否则队列会堵死。 */
        if (!Queue_IsEmpty(&uart->tx_queue) &&
            uart->huart->gState == HAL_UART_STATE_READY) {

            Node* temp = Queue_Pop(&uart->tx_queue);
            if (temp != NULL) {
                if (HAL_UART_Transmit_DMA(uart->huart, (uint8_t*)temp->data, temp->len) != HAL_OK) {
                    Queue_FreeNode(temp);          /* 发不出去就还回去，别卡住队列 */
                } else {
                    uart->status = UART_SENDING;   /* 仅用于观察 */
                }
            }
        }
    }
}

/* 函数体 */

void UART_Init(UART_DRIVES* user_uart, UART_HandleTypeDef* huart) {
    user_uart->huart = huart;
    user_uart->status = UART_IDLE;
    user_uart->is_buffer_a = 1;

    Queue_Init(&user_uart->tx_queue, TX_QUEUE_LEN);

    uart_drives[uart_num] = user_uart;
    uart_num++;

    HAL_UARTEx_ReceiveToIdle_DMA(huart, user_uart->rx_buffer_a, UART_BUFFER_SIZE);

    if (is_init_loop_event_sign == 0) {
        loop_event[loop_event_num] = &UART_QueueHandle;
        loop_event_num++;
        is_init_loop_event_sign = 1;
    }
}

void UART_RegisterCallback(UART_DRIVES* user_uart, const UART_Callback callback) {
    user_uart->callbacks[user_uart->callback_num] = callback;
    user_uart->callback_num++;
}

void UART_Send_String(UART_DRIVES* user_uart, const char* str) {
    Queue_Push(&user_uart->tx_queue, (char*)str, strlen(str));
}

void UART_Send_Data(UART_DRIVES* user_uart, const char* data, const uint16_t len) {
    Queue_Push(&user_uart->tx_queue, (char*)data, len);
}


uint16_t UART_GetDataWithHT(UART_DRIVES* user_uart, uint8_t *data, const char *head, const char *tail) {
    return RingBuffer_GetWith_H_T(&user_uart->rx_ringBuffer, data, head, tail);
}

uint16_t UART_GetDataWithHTLen(UART_DRIVES* user_uart, uint8_t *data, const char *head, const char *tail, const uint16_t len) {
    return RingBuffer_GetWith_HT_Len(&user_uart->rx_ringBuffer, data, head, tail, len);
}

uint16_t UART_GetDataWithHLen(UART_DRIVES* user_uart, uint8_t *data, const char *head, const uint16_t len) {
    return RingBuffer_GetWith_H_Len(&user_uart->rx_ringBuffer, data, head, len);
}

uint16_t UART_GetDataWithLen(UART_DRIVES* user_uart, uint8_t *data, uint16_t len) {
    return RingBuffer_GetWith_Len(&user_uart->rx_ringBuffer, data, len);
}

uint16_t UART_GetDataWithH(UART_DRIVES* user_uart, uint8_t *data, const char *head) {
    return RingBuffer_GetWith_H_H(&user_uart->rx_ringBuffer, data, head);
}

uint16_t UART_GetAllDate(UART_DRIVES* user_uart, uint8_t *data) {
    return RingBuffer_GetWith_Len(&user_uart->rx_ringBuffer, data, RingBuffer_GetLength(&user_uart->rx_ringBuffer));
}

void UART_Printf(UART_DRIVES* user_uart, const char* format, ...) {
    static char buffer[UART_BUFFER_SIZE];
    va_list args;

    va_start(args, format);
    const int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len > 0 && len < sizeof(buffer)) {
        Queue_Push(&user_uart->tx_queue, buffer, len);
    }
}

int32_t UART_Scanf(UART_DRIVES* user_uart, const char *format, ...) {
    static uint8_t buffer[UART_BUFFER_SIZE];
    const uint16_t len = UART_GetAllDate(user_uart, buffer);
    
    if (len == 0)
        return -1;

    buffer[len] = '\0';  // 确保字符串结束
    
    va_list args;
    va_start(args, format);
    const int32_t result = vsscanf((const char*)buffer, format, args);
    va_end(args);
    
    return result;
}

/* 覆写中断回调函数 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    for (uint8_t uart_index = 0; uart_index < uart_num; uart_index++) {
        UART_DRIVES *uart = uart_drives[uart_index];

        if (huart != uart->huart)
            continue;

        if (uart->tx_queue.popped) {
            Queue_FreeNode(uart->tx_queue.popped);
            uart->tx_queue.popped = NULL;
            uart->status = UART_IDLE;
        }
    }
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    for (uint8_t uart_index = 0; uart_index < uart_num; uart_index++) {
        UART_DRIVES *uart = uart_drives[uart_index];

        if (uart->huart != huart)
            continue;

        if (uart->is_buffer_a) {
            HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buffer_b, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
            RingBuffer_Put(&uart->rx_ringBuffer, uart->rx_buffer_a, Size);
            uart->is_buffer_a = 0;
            uart->receive_new_data = 1;
        } else {
            HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buffer_a, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
            RingBuffer_Put(&uart->rx_ringBuffer, uart->rx_buffer_b, Size);
            uart->is_buffer_a = 1;
            uart->receive_new_data = 1;
        }
    }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    for (uint8_t uart_index = 0; uart_index < uart_num; uart_index++) {
        UART_DRIVES *uart = uart_drives[uart_index];

        if (uart->huart != huart)
            continue;

        HAL_UART_DMAStop(huart);
        const DMA_HandleTypeDef *hdma = huart->hdmarx;
        __HAL_DMA_DISABLE(hdma);
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        if (uart->is_buffer_a) {
            HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buffer_b, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
            uart->is_buffer_a = 0;
        } else {
            HAL_UARTEx_ReceiveToIdle_DMA(uart->huart, uart->rx_buffer_a, UART_BUFFER_SIZE);
            __HAL_DMA_DISABLE_IT(uart->huart->hdmarx, DMA_IT_HT);
            uart->is_buffer_a = 1;
        }
        break;
    }
}

#endif /* HAL_UART_MODULE_ENABLED */

