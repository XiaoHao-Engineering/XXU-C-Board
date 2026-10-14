#include "../../../Core/Inc/bsp.h"
#ifdef HAL_UART_MODULE_ENABLED

#include "../user_serialplot.h"

static SERIALPLOT_DRIVES* serialplot_drives[SERIALPLOT_NUM];
static uint8_t serialplot_num = 0;

static void  Serialplot_Handle(void* user_uart);
static void  Serialplot_Output(void* arg);
static void  Serialplot_Parse(SERIALPLOT_DRIVES* sp, const uint8_t* str, uint16_t len);
static float Serialplot_StrToFloat(const uint8_t* str, uint16_t len);
static uint8_t Serialplot_Is_Space(uint8_t c);

void Serialplot_Init(SERIALPLOT_DRIVES* sp, UART_DRIVES* user_uart,
                     const SERIALPLOT_VAR* vars, const uint8_t var_num,
                     const uint32_t period_ms) {
    sp->user_uart = user_uart;
    sp->vars = vars;
    sp->var_num = var_num;
    sp->var_index = -1;
    sp->var_value = 0.0f;
    sp->data_num = 0;
    sp->checksum = SERIALPLOT_CHECKSUM_ENABLE;
    sp->frame_header = SERIALPLOT_FRAME_HEAD;

    UART_RegisterCallback(user_uart, Serialplot_Handle);

    serialplot_drives[serialplot_num] = sp;
    serialplot_num++;

    SysTick_InitTask(&sp->task, (void*)sp, 100, period_ms, Task_REPEAT, Serialplot_Output);
    SysTick_StartTask(&sp->task);
}

/* 指定本帧要发送的变量地址，个数不要超过 SERIALPLOT_MAX_CH */
void Serialplot_Set_Data(SERIALPLOT_DRIVES* sp, const uint8_t number, ...) {
    va_list args;
    va_start(args, number);

    const uint8_t n = (number > SERIALPLOT_MAX_CH) ? SERIALPLOT_MAX_CH : number;
    for (uint8_t i = 0; i < n; i++) {
        sp->data[i] = (const void*)va_arg(args, void*);
    }
    sp->data_num = n;

    va_end(args);
}

int8_t Serialplot_Get_Index(const SERIALPLOT_DRIVES* sp) {
    return sp->var_index;
}

float Serialplot_Get_Value(const SERIALPLOT_DRIVES* sp) {
    return sp->var_value;
}

/* 每 period_ms 把 data[] 打包发出：帧头 + N×float32 +（可选）1 字节累加和 */
static void Serialplot_Output(void* arg) {
    SERIALPLOT_DRIVES* sp = (SERIALPLOT_DRIVES*)arg;

    if (sp->data_num == 0) {
        return;
    }

    uint16_t len = 1;
    sp->tx_buf[0] = sp->frame_header;

    for (uint8_t i = 0; i < sp->data_num; i++) {
        memcpy(&sp->tx_buf[len], sp->data[i], sizeof(float));
        len += sizeof(float);
    }

    if (sp->checksum) {
        uint8_t sum = 0;
        for (uint16_t i = 1; i < len; i++) {
            sum += sp->tx_buf[i];
        }
        sp->tx_buf[len] = sum;
        len++;
    }

    UART_Send_Data(sp->user_uart, (char*)sp->tx_buf, len);
}

/* 接收上位机指令，形如 name=value# */
static void Serialplot_Handle(void* user_uart) {
    const UART_DRIVES* uart = (UART_DRIVES*)user_uart;

    for (uint8_t index = 0; index < serialplot_num; index++) {
        SERIALPLOT_DRIVES* sp = serialplot_drives[index];

        if (sp->user_uart->huart != uart->huart) {
            continue;
        }

        uint16_t len = RingBuffer_GetLength(&sp->user_uart->rx_ringBuffer);
        if (len == 0) {
            continue;
        }
        if (len > SERIALPLOT_RX_LEN - 1) {
            len = SERIALPLOT_RX_LEN - 1;
        }
        len = RingBuffer_GetWith_Len(&sp->user_uart->rx_ringBuffer, sp->rx_buf, len);

        uint16_t i = 0;
        while (i < len) {
            while (i < len && sp->rx_buf[i] == '#') {
                i++;
            }
            uint16_t start = i;
            while (i < len && sp->rx_buf[i] != '#') {
                i++;
            }
            if (i >= len) {
                break;      /* 没有收到结束符，这条指令直接丢弃（不做跨包拼接）*/
            }

            uint16_t end = i;

            /* ★ 去掉这一段首尾的空白字符（空格 / \r / \n / \t）：很多上位机（含 SerialPlot 的
               Command 面板）发完一条指令会补一个换行，不修剪的话下一条指令的名字永远匹配不上，
               表现为"只有第一条指令生效，后面的全被忽略"。 */
            while (start < end && Serialplot_Is_Space(sp->rx_buf[start])) {
                start++;
            }
            while (end > start && Serialplot_Is_Space(sp->rx_buf[end - 1])) {
                end--;
            }
            if (end > start) {
                Serialplot_Parse(sp, &sp->rx_buf[start], end - start);
            }

            i++;
        }
    }
}

static uint8_t Serialplot_Is_Space(const uint8_t c) {
    return (c == ' ' || c == '\r' || c == '\n' || c == '\t') ? 1 : 0;
}

static void Serialplot_Parse(SERIALPLOT_DRIVES* sp, const uint8_t* str, const uint16_t len) {
    uint16_t eq = 0;
    while (eq < len && str[eq] != '=') {
        eq++;
    }
    if (eq == 0 || eq >= len) {
        sp->var_index = -1;
        return;
    }

    sp->var_value = Serialplot_StrToFloat(&str[eq + 1], len - eq - 1);
    sp->var_index = -1;

    for (uint8_t i = 0; i < sp->var_num; i++) {
        const char* name = sp->vars[i].name;
        uint16_t j = 0;
        while (j < eq && name[j] != 0 && name[j] == (char)str[j]) {
            j++;
        }
        if (j != eq || name[j] != 0) {
            continue;
        }

        sp->var_index = (int8_t)i;
        float value = sp->var_value;
        if (value < sp->vars[i].min) {
            value = sp->vars[i].min;
        }
        if (value > sp->vars[i].max) {
            value = sp->vars[i].max;
        }
        if (sp->vars[i].value != NULL) {
            *sp->vars[i].value = value;
        }
        if (sp->vars[i].on_change != NULL) {
            sp->vars[i].on_change();
        }
        break;
    }
}

/* 手写浮点解析，避免依赖 libm 的 strtof */
static float Serialplot_StrToFloat(const uint8_t* str, const uint16_t len) {
    float value = 0.0f;
    float scale = 1.0f;
    uint16_t i = 0;
    uint8_t negative = 0;
    uint8_t dot = 0;

    if (i < len && str[i] == '-') {
        negative = 1;
        i++;
    }

    for (; i < len; i++) {
        const uint8_t c = str[i];
        if (c == '.') {
            dot = 1;
            continue;
        }
        if (c < '0' || c > '9') {
            break;
        }
        if (dot) {
            scale *= 0.1f;
            value += (float)(c - '0') * scale;
        } else {
            value = value * 10.0f + (float)(c - '0');
        }
    }

    return negative ? -value : value;
}

#endif /* HAL_UART_MODULE_ENABLED */
