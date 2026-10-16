#include "../../../Core/Inc/bsp.h"
#ifdef HAL_UART_MODULE_ENABLED
/* 包含头文件 */
#include "../user_vision.h"

/* 私有变量 */
static VISION_DRIVES* vision_drives[VISION_NUM];
static uint8_t vision_num = 0;

/* 私有函数 */
static void Vision_Handle(void* user_uart);
static void Vision_Update(void* arg);

/* 函数体 */

/* CRC-16/MODBUS：init 0xFFFF、反射、poly 0xA001。
   ★ 必须和视觉侧逐位一致，否则每一包都会被判为坏帧（表现为"永远离线"）。*/
uint16_t Vision_CRC16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFFU;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t j = 0; j < 8U; j++) {
            if (crc & 0x0001U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

/* ★★ 视觉发送改由【主循环】驱动（不再挂 1ms SysTick 任务）：
   这样 UART 收发完全不占 1ms 中断预算，主循环慢一点只是发送率略降，不影响功能。
   发送周期由 HAL_GetTick() 自计时限制为 VISION_TX_PERIOD_MS（默认 10ms=100Hz）。
   注：这里的 HAL_GetTick 在主循环里是正常推进的（不会像在 ISR 里那样被冻住）。*/
static uint32_t s_vision_last_tx_ms = 0U;

static void Vision_LoopHandle(void) {
    const uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - s_vision_last_tx_ms) < VISION_TX_PERIOD_MS) {
        return;                     /* 还没到下一个发送周期，直接返回 */
    }
    s_vision_last_tx_ms = now;
    Vision_Update(&user_vision);
}
void Vision_Init(VISION_DRIVES* vision, UART_DRIVES* uart) {
    vision->uart = uart;

    /* 帧头固定，之后每包只更新数据段 */
    vision->tx_packet.head[0] = (uint8_t)'A';
    vision->tx_packet.head[1] = (uint8_t)'A';
    vision->tx_packet.mode = VISION_TX_MODE_IDLE;
    vision->tx_packet.q[0] = 1.0f;   /* 单位四元数，避免视觉在没数据时算出一个奇怪姿态 */
    vision->tx_packet.q[1] = 0.0f;
    vision->tx_packet.q[2] = 0.0f;
    vision->tx_packet.q[3] = 0.0f;
    vision->tx_packet.yaw = 0.0f;
    vision->tx_packet.yaw_vel = 0.0f;
    vision->tx_packet.pitch = 0.0f;
    vision->tx_packet.pitch_vel = 0.0f;
    vision->tx_packet.bullet_speed = 0.0f;
    vision->tx_packet.bullet_count = 0U;

    vision->target_yaw = 0.0f;
    vision->target_yaw_vel = 0.0f;
    vision->target_yaw_acc = 0.0f;
    vision->target_pitch = 0.0f;
    vision->target_pitch_vel = 0.0f;
    vision->target_pitch_acc = 0.0f;
    vision->rx_mode = VISION_RX_MODE_NO_CONTROL;

    /* ★ float 镜像（绘图模式 9 用）：结构体是全局变量本来就被 BSS 清零，
       但显式初始化更稳妥（换静态/局部实例、或反复 Init 时不会留旧值）。*/
    vision->rx_mode_f = 0.0f;
    vision->target_yaw_deg = 0.0f;
    vision->target_pitch_deg = 0.0f;
    vision->target_yaw_vel_deg = 0.0f;
    vision->target_pitch_vel_deg = 0.0f;
    vision->online_f = 0.0f;
    vision->frame_count_f = 0.0f;
    vision->crc_error_f = 0.0f;

    vision->is_online = 0;
    vision->last_rx_ms = 0U;
    vision->frame_count = 0U;
    vision->crc_error_count = 0U;
    vision->timeout_count = 0U;

    /* 挂到通用串口层：收够一段就回调 Vision_Handle（和 CAN 层同一个套路）*/
    UART_RegisterCallback(uart, Vision_Handle);

    /* 1kHz 任务：发包 + 离线判定。延迟 100ms 启动，让 UART 先跑起来 */
    /* ★ 发送任务注册到【主循环事件表】（与 CAN/UART 队列处理同一机制），
       不再用 SysTick_InitTask —— 彻底从 1ms 中断里移出来 */
    loop_event[loop_event_num] = &Vision_LoopHandle;
    loop_event_num++;    SysTick_StartTask(&vision->task);

    vision_drives[vision_num] = vision;
    vision_num++;
}

/* 收到一段串口数据：从环形缓冲里按"帧头 + 定长"取包，校验 CRC 后解析。
   一包不合法就丢掉这一包（不猜、不补），避免坏帧把云台甩到错误角度。 */
static void Vision_Handle(void* user_uart) {
    const UART_DRIVES* uart = (const UART_DRIVES*)user_uart;

    for (uint8_t index = 0; index < vision_num; index++) {
        VISION_DRIVES* vision = vision_drives[index];

        if (vision->uart != uart) {
            continue;
        }

        /* 一次回调最多取几帧：留一点余量，又不至于在回调里耗太久 */
        for (uint8_t n = 0; n < VISION_RX_MAX_PER_LOOP; n++) {
            if (UART_GetDataWithHLen(vision->uart, (uint8_t*)&vision->rx_packet,
                                     VISION_FRAME_HEAD, VISION_RX_PACKET_SIZE)
                != VISION_RX_PACKET_SIZE) {
                break;      /* 缓冲里没有完整的一包了 */
            }

            /* CRC 覆盖除最后 2 字节以外的全部内容 */
            const uint16_t crc_calc = Vision_CRC16((const uint8_t*)&vision->rx_packet,
                                                   VISION_RX_PACKET_SIZE - 2U);
            if (crc_calc != vision->rx_packet.crc16) {
                vision->crc_error_count++;   /* 坏帧：计数并丢弃 */
                continue;
            }

            /* 到这一步才认这包数据 —— 之后云台动作的依据就是它 */
            vision->target_yaw = vision->rx_packet.yaw;
            vision->target_yaw_vel = vision->rx_packet.yaw_vel;
            vision->target_yaw_acc = vision->rx_packet.yaw_acc;
            vision->target_pitch = vision->rx_packet.pitch;
            vision->target_pitch_vel = vision->rx_packet.pitch_vel;
            vision->target_pitch_acc = vision->rx_packet.pitch_acc;
            vision->rx_mode = vision->rx_packet.mode;

            vision->frame_count++;
            vision->last_rx_ms = HAL_GetTick();

            if (vision->is_online == 0) {
                vision->is_online = 1;       /* 重新上线 */
            }

            /* ★ float 镜像同步（串口绘图模式 9 = "画视觉发过来的数据"）：
               放在最后，online_f 才能反映"本帧刚收到 -> 在线 = 1"。
               角度统一 ×VISION_PKG2DEG 由【弧度】换算成【度】（包内是弧度）。*/
            vision->rx_mode_f = (float)vision->rx_mode;
            vision->target_yaw_deg = vision->target_yaw * VISION_PKG2DEG;
            vision->target_pitch_deg = vision->target_pitch * VISION_PKG2DEG;
            vision->target_yaw_vel_deg = vision->target_yaw_vel * VISION_PKG2DEG;
            vision->target_pitch_vel_deg = vision->target_pitch_vel * VISION_PKG2DEG;
            vision->online_f = (float)vision->is_online;
            vision->frame_count_f = (float)vision->frame_count;
            vision->crc_error_f = (float)vision->crc_error_count;
        }
    }
}

/* 周期任务：发包 + 离线判定 */
static void Vision_Update(void* arg) {
    VISION_DRIVES* vision = (VISION_DRIVES*)arg;

    /* 离线判定：超时只置标志，不动数据（目标角由应用层决定"保持"还是"回手动"）*/
    if (vision->is_online != 0
        && (HAL_GetTick() - vision->last_rx_ms) > VISION_OFFLINE_MS) {
        vision->is_online = 0;
        vision->timeout_count++;
        vision->frame_count_f = (float)vision->frame_count;
        vision->online_f = (float)vision->is_online;   /* 镜像跟着掉 0，绘图才不会一直显示"在线" */
    }

    /* 组包并发出：CRC 覆盖除最后 2 字节以外的全部内容 */
    vision->tx_packet.crc16 = Vision_CRC16((const uint8_t*)&vision->tx_packet,
                                           VISION_TX_PACKET_SIZE - 2U);
    UART_Send_Data(vision->uart, (const char*)&vision->tx_packet, VISION_TX_PACKET_SIZE);
    vision->tx_count++;
    vision->tx_count_f = (float)vision->tx_count;
}

/* 应用层接口 -------------------------------------------------------------*/

void Vision_Set_Tx_Mode(VISION_DRIVES* vision, const uint8_t mode) {
    vision->tx_packet.mode = mode;
}

void Vision_Set_Tx_Attitude(VISION_DRIVES* vision,
                            const float qw, const float qx, const float qy, const float qz,
                            const float yaw, const float yaw_vel,
                            const float pitch, const float pitch_vel) {
    vision->tx_packet.q[0] = qw;
    vision->tx_packet.q[1] = qx;
    vision->tx_packet.q[2] = qy;
    vision->tx_packet.q[3] = qz;
    vision->tx_packet.yaw = yaw;
    vision->tx_packet.yaw_vel = yaw_vel;
    vision->tx_packet.pitch = pitch;
    vision->tx_packet.pitch_vel = pitch_vel;
}

void Vision_Set_Tx_Bullet(VISION_DRIVES* vision, const float speed, const uint16_t count) {
    vision->tx_packet.bullet_speed = speed;
    vision->tx_packet.bullet_count = count;
}

uint8_t Vision_Is_Online(const VISION_DRIVES* vision) {
    return vision->is_online;
}

uint8_t Vision_Get_Rx_Mode(const VISION_DRIVES* vision) {
    return vision->rx_mode;
}

uint32_t Vision_Get_Frame_Count(const VISION_DRIVES* vision) {
    return vision->frame_count;
}

#endif /* HAL_UART_MODULE_ENABLED */
