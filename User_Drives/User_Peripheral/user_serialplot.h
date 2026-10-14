#ifndef USER_SERIALPLOT_H
#define USER_SERIALPLOT_H
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED

#include <stdarg.h>
#include "user_uart.h"
#include "../../Core/Inc/bsp_config.h"
#include "../../User_Architect/user_systick.h"

#define SERIALPLOT_MAX_CH      (12)    /* 最多绘图通道数 */
#define SERIALPLOT_NAME_LEN    (20)    /* 变量名最大长度 */
#define SERIALPLOT_RX_LEN      (64)    /* 接收缓冲长度 */
#define SERIALPLOT_FRAME_HEAD  (0xAB)  /* 帧头 */

/* 是否在数据帧尾部附加 1 字节累加和
   ★ 必须和上位机 SerialPlot 里设的帧格式一致，否则波形乱跳或完全不动。
     1 = 0xAB + N×float32(小端) + 1 字节累加和（默认，与原 A 板工程一致，已验证能用）
     0 = 0xAB + N×float32(小端)
     SerialPlot 的 Custom frame 模式（Frame start 填 AB、勾上 checksum）对应 1，
     Simple binary 模式（只有 start byte + 纯数据）对应 0。
     两种模式的填法见 docs/上机调试流程.md；tools\serialplot.py 两种格式都能自动识别。 */
#define SERIALPLOT_CHECKSUM_ENABLE   (1)

/* 一条可在线修改的变量 */
typedef struct {
    const char* name;
    float*      value;
    float       min;
    float       max;
    void      (*on_change)(void);  /* 改完后的回调，可为 NULL */
} SERIALPLOT_VAR;

typedef struct {
    UART_DRIVES* user_uart;
    uint8_t      checksum;
    uint8_t      frame_header;
    const void*  data[SERIALPLOT_MAX_CH];
    uint8_t      data_num;
    const SERIALPLOT_VAR* vars;
    uint8_t      var_num;
    int8_t       var_index;                           /* 最近命中的变量号，未命中为 -1 */
    float        var_value;
    uint8_t      rx_buf[SERIALPLOT_RX_LEN];
    uint8_t      tx_buf[2 + SERIALPLOT_MAX_CH * 4];
    SysTick_Task task;
} SERIALPLOT_DRIVES;

void    Serialplot_Init(SERIALPLOT_DRIVES* sp, UART_DRIVES* user_uart,
                        const SERIALPLOT_VAR* vars, uint8_t var_num, uint32_t period_ms);

/* 指定本帧要发送的变量地址，全部按 float 发送 */
void    Serialplot_Set_Data(SERIALPLOT_DRIVES* sp, uint8_t number, ...);

int8_t  Serialplot_Get_Index(const SERIALPLOT_DRIVES* sp);
float   Serialplot_Get_Value(const SERIALPLOT_DRIVES* sp);

#endif /* HAL_UART_MODULE_ENABLED */
#endif // USER_SERIALPLOT_H
