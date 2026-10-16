#ifndef USER_VISION_H
#define USER_VISION_H
#include "main.h"
#ifdef HAL_UART_MODULE_ENABLED

/* 包含头文件 */
#include "user_uart.h"
#include "../../User_Architect/user_systick.h"

/* ---------------------------------------------------------------------------
 * 视觉串口对接层（驱动层）
 *
 * 协议（与算法组对齐，参考 RM 步兵常用约定）：
 *   1. 帧头固定 "AA"（两个字节 'A','A'）
 *   2. 收发都是【小端 packed 结构体】
 *   3. CRC16 范围 = 整包【除去最后 2 字节 crc16】，算法 = CRC-16/MODBUS
 *      （init 0xFFFF、反射、poly 0xA001）—— 必须和视觉侧完全一致，否则一包都收不下
 *   4. 视觉 -> 电控：mode / yaw / yaw_vel / yaw_acc / pitch / pitch_vel / pitch_acc
 *      帧长 = 3 + 6*4 + 2 = 29 字节
 *   5. 电控 -> 视觉：mode / q[4](wxyz) / yaw / yaw_vel / pitch / pitch_vel /
 *                    bullet_speed / bullet_count(uint16)
 *      帧长 = 3 + 9*4 + 2 + 2 = 43 字节
 *
 * ★ 注意视觉给的是【角度 + 角速度 + 角加速度】而不是单一角度：
 *   角速度/角加速度可以直接当【前馈】用，跟踪移动目标时明显更跟手。
 *
 * 分层：本文件在【驱动层】，只负责"帧收发 + 校验 + 在线判定"，
 *       不管这些角度怎么用（那是应用层 gimbal.c 的事）。
 * ------------------------------------------------------------------------- */

/* 宏定义 ------------------------------------------------------------------*/

#define VISION_FRAME_HEAD        "AA"    /* ⚠️ 帧头是【两个 ASCII 字符 A A】= 字节 41 41，
                                                 不是十六进制的单字节 0xAA！
                                                 在串口助手 HEX 里要按 41 41 找。
                                                 （命名沿用参考工程，RingBuffer 取帧按此串搜索）*/
/* ★★ 视觉包内角度单位（必须与视觉保持一致！）
   参考工程内部角度【全部是弧度】(dvc_bmi088.h 的 Yaw总角度(rad)、dvc_motor.h 的
   外部角度反馈值(rad)、crt_gimbal.h 的 pitch 限位 -0.446/0.81 rad)，且 dvc_vision
   解析后【原值透传不做换算】-> 所以包里的 yaw/pitch/yaw_vel/pitch_vel 都是【弧度/弧度每秒】。
   本工程内部惯用【度】，因此在收发边界统一换算，开关放在这里：
     1 = 包内是弧度（与参考工程一致，默认）
     0 = 包内是度
   若上电后发现云台转角差 57.3 倍，把这个开关翻一下即可，不用改别的代码。 */
#define VISION_ANGLE_UNIT_RAD    (1)

#if VISION_ANGLE_UNIT_RAD
#define VISION_DEG2PKG           (0.0174532925f)   /* 度   -> 包内(弧度) */
#define VISION_PKG2DEG           (57.29578f)       /* 包内(弧度) -> 度  */
#else
#define VISION_DEG2PKG           (1.0f)
#define VISION_PKG2DEG           (1.0f)
#endif
#define VISION_OFFLINE_MS        (50U)   /* 连续多少 ms 没收到有效帧 = 视觉离线 */
#define VISION_RX_PACKET_SIZE    (29U)   /* 视觉 -> 电控 帧长 */
#define VISION_TX_PACKET_SIZE    (43U)   /* 电控 -> 视觉 帧长 */
#define VISION_RX_MAX_PER_LOOP   (4U)    /* 单次回调最多取几帧，防止一次吃太多 */
/* ★ 发包周期 ms：一帧 43 字节，115200 下一个 UART 最多约 11520 B/s。
   1ms(1kHz) = 43kB/s 远远超出，会把发送队列灌满、UART 长期忙 -> 必须放慢。
   10ms(100Hz) = 4.3kB/s，占带宽约 37%，够用且稳妥。 */
#define VISION_TX_PERIOD_MS      (10U)    /* 发包周期 ms：1kHz=43kB/s 超出 115200 能力，故 100Hz */

/* ★ 发送包 pitch 字段的语义（协议不变，只是填什么）：
   0 = 发 "yaw框架倾角(IMU/AHRS) + 枪管相对yaw框架角(编码器)" 的合成 = 枪口世界仰角
       —— 本车 IMU 装在 yaw/pitch 连接处（yaw 框架），看不到枪管自己的俯仰，必须相加
   1 = 只发 IMU/AHRS 的 pitch —— 参考工程的填法，适用于 IMU 直接装在 pitch 框架上的情况
   ★ 判定方法：只转 pitch（枪管）时看绘图模式 6 的 qpitch 变不变
       变 -> IMU 在 pitch 框架上 -> 填 1
       不变 -> IMU 在 yaw 框架上   -> 填 0（本车默认） */
#define VISION_PITCH_USE_IMU_ONLY   (1)

/* ★ 弹速换算系数：弹速[m/s] = 摩擦轮角速度[rad/s] × 本系数
   （参考工程用的就是 0.03。★ 必须实测标定：换个摩擦轮/电压就得重标）*/
#define VISION_BULLET_SPEED_COEF    (0.03f)

/* 摩擦轮 RPM -> rad/s 的换算（RPM × 2π/60）*/
#define VISION_RPM_TO_RAD_S         (0.10471976f)

/* 电控 -> 视觉 的工作模式 */
#define VISION_TX_MODE_IDLE       (0)    /* 不参与 */
#define VISION_TX_MODE_AUTOAIM    (1)    /* 自瞄 */
#define VISION_TX_MODE_SMALL_BUFF (2)    /* 小符 */
#define VISION_TX_MODE_BIG_BUFF   (3)    /* 大符 */

/* 视觉 -> 电控 的控制模式 */
#define VISION_RX_MODE_NO_CONTROL       (0)   /* 不给控制 */
#define VISION_RX_MODE_CONTROL_ONLY     (1)   /* 只给控制，不开火 */
#define VISION_RX_MODE_CONTROL_AND_FIRE (2)   /* 给控制 + 请求开火（★ 电控仍是否决方）*/

/* 类型定义 ----------------------------------------------------------------*/

/* 视觉发给电控的原始帧（内存布局 = 线上布局，必须 packed）*/
typedef struct __attribute__((packed)) {
    uint8_t  head[2];      /* 'A','A' */
    uint8_t  mode;         /* VISION_RX_MODE_xxx */
    float    yaw;          /* 目标 yaw 角 deg */
    float    yaw_vel;      /* 目标 yaw 角速度 deg/s（前馈用）*/
    float    yaw_acc;      /* 目标 yaw 角加速度 deg/s^2 */
    float    pitch;        /* 目标 pitch 角 deg */
    float    pitch_vel;    /* 目标 pitch 角速度 deg/s */
    float    pitch_acc;    /* 目标 pitch 角加速度 deg/s^2 */
    uint16_t crc16;        /* 覆盖前面所有字节 */
} VISION_RX_PACKET;

/* 电控发给视觉的原始帧 */
typedef struct __attribute__((packed)) {
    uint8_t  head[2];      /* 'A','A' */
    uint8_t  mode;         /* VISION_TX_MODE_xxx */
    float    q[4];         /* 姿态四元数 w,x,y,z（视觉做完整三维变换要用）*/
    float    yaw;          /* 当前 yaw deg */
    float    yaw_vel;      /* 当前 yaw 角速度 deg/s */
    float    pitch;        /* 当前 pitch deg */
    float    pitch_vel;    /* 当前 pitch 角速度 deg/s */
    float    bullet_speed; /* 当前弹速 m/s（视觉算弹道用）*/
    uint16_t bullet_count; /* 剩余弹量 */
    uint16_t crc16;
} VISION_TX_PACKET;

/* 视觉设备对象 */
typedef struct {
    UART_DRIVES* uart;             /* 底层串口（初始化时注入，与本层解耦）*/

    /* 原始帧 */
    VISION_RX_PACKET rx_packet;
    VISION_TX_PACKET tx_packet;

    /* ★ 解析后的数据：应用层只读这些，不直接碰 rx_packet */
    float    target_yaw;
    float    target_yaw_vel;
    float    target_yaw_acc;
    float    target_pitch;
    float    target_pitch_vel;
    float    target_pitch_acc;
    uint8_t  rx_mode;

    /* 状态与统计（排错用，可进串口绘图）*/
    uint8_t  is_online;            /* 1 = 在线 */
    uint32_t last_rx_ms;           /* 最近一次收到有效帧的时刻 */
    uint32_t frame_count;          /* 累计有效帧数（看帧率用）*/
    uint32_t crc_error_count;      /* CRC 错帧数（链路质量）*/
    uint32_t timeout_count;        /* 离线次数（掉线统计）*/
    float    frame_count_f;        /* frame_count 的 float 镜像（进绘图通道用）*/
    uint32_t tx_count;              /* ★ 已【发送】的帧数：绘图上看到它在涨 = 真的在往 UART1 发 */
    float    tx_count_f;            /* tx_count 的 float 镜像（进绘图通道用）*/

    SysTick_Task task;             /* 1kHz：发包 + 离线判定 */
} VISION_DRIVES;

/* 函数声明 ----------------------------------------------------------------*/

void Vision_Init(VISION_DRIVES* vision, UART_DRIVES* uart);

/* CRC-16/MODBUS：和视觉侧必须一致 */
uint16_t Vision_CRC16(const uint8_t* data, uint16_t len);

/* 应用层 -> 本层：把要回传给视觉的数据填进去（下一包自动带出去）*/
void Vision_Set_Tx_Mode(VISION_DRIVES* vision, uint8_t mode);
void Vision_Set_Tx_Attitude(VISION_DRIVES* vision,
                            float qw, float qx, float qy, float qz,
                            float yaw, float yaw_vel,
                            float pitch, float pitch_vel);
void Vision_Set_Tx_Bullet(VISION_DRIVES* vision, float speed, uint16_t count);

/* 应用层 <- 本层：读状态 */
uint8_t Vision_Is_Online(const VISION_DRIVES* vision);
uint8_t Vision_Get_Rx_Mode(const VISION_DRIVES* vision);
uint32_t Vision_Get_Frame_Count(const VISION_DRIVES* vision);

#endif /* HAL_UART_MODULE_ENABLED */
#endif // USER_VISION_H
