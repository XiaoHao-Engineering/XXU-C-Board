#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电控 -> 视觉  四元数通信测试工具
================================

作用：扮演"视觉程序"的接收端，验证 MCU 通过 UART1 发来的 43 字节帧能不能
      被正确解析 —— 尤其是四元数 q[4]。也可以每秒回发一个假目标角，
      用来顺带验证"视觉 -> 电控"的反向链路。

★ 接线（最容易搞错的一步）
    MCU UART1_TX  ->  USB转TTL 的 RX
    MCU UART1_RX  <-  USB转TTL 的 TX      （只用 --send 时才需要）
    MCU GND       --  USB转TTL 的 GND     （必须共地）
    ⚠️ 测试时请把【视觉模块】从 UART1 上拔掉：
       UART 是点对点，两个发送端同时接上去会互相打架。

协议（与固件 User_Drives/User_Peripheral/user_vision.h 完全一致）
    帧头 "AA" + 小端 packed 结构 + CRC-16/MODBUS（覆盖除最后 2 字节外的全部）

    电控 -> 视觉   43 字节:
      head(2) mode(1) q[4](w,x,y,z) yaw yaw_vel pitch pitch_vel bullet_speed
      bullet_count(uint16) crc16(uint16)

    视觉 -> 电控   29 字节:
      head(2) mode(1) yaw yaw_vel yaw_acc pitch pitch_vel pitch_acc crc16(uint16)

用法
    python vision_rx_test.py COM7            只接收并打印
    python vision_rx_test.py COM7 --send     接收 + 每秒回发一个假目标角
    python vision_rx_test.py COM7 --raw      同时打印原始十六进制（对协议用）
    python vision_rx_test.py COM7 --dump     启动后直接存一段原始数据到 vision_raw.bin

判断标准（脚本会自动给出结论）
    ✅ |q| ≈ 1.0                    四元数是规范化的
    ✅ q 随云台转动而改变            姿态在实时更新
    ⚠️ q 恒为 (1,0,0,0)              单位四元数没更新 -> EKF 没跑 / IMU 没喂进去
    ⚠️ q 全为 0                      非法四元数（固件侧 bug）
    ⚠️ |q| 明显偏离 1                四元数没归一化 -> EKF 有问题
"""

import argparse
import math
import struct
import sys
import time

try:
    import serial
except ImportError:
    print("缺少 pyserial，请先安装：pip install pyserial")
    sys.exit(1)

# ---------------------------------------------------------------- 协议定义
MAGIC = b'AA'

TX_FMT = '<2sB9fHH'   # 电控 -> 视觉   2+1+36+2+2 = 43 字节
TX_LEN = struct.calcsize(TX_FMT)
RX_FMT = '<2sB6fH'    # 视觉 -> 电控   2+1+24+2   = 29 字节
RX_LEN = struct.calcsize(RX_FMT)

MODE_TX = {0: 'IDLE', 1: 'AUTOAIM', 2: 'SMALL_BUFF', 3: 'BIG_BUFF'}


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS：init 0xFFFF、反射、poly 0xA001（与固件逐位一致）"""
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = ((crc >> 1) ^ 0xA001) & 0xFFFF
            else:
                crc >>= 1
    return crc


def pack_rx(mode: int, yaw: float, yaw_vel: float, yaw_acc: float,
            pitch: float, pitch_vel: float, pitch_acc: float) -> bytes:
    """打包一帧"视觉 -> 电控"（29 字节），用于 --send"""
    body = struct.pack(RX_FMT, MAGIC, mode & 0xFF,
                       yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, 0)
    body = body[:-2] + struct.pack('<H', crc16_modbus(body[:-2]))
    return body


def quat_to_norm(q):
    return math.sqrt(sum(v * v for v in q))


def main():
    ap = argparse.ArgumentParser(description='电控->视觉 四元数通信测试')
    ap.add_argument('port', help='串口号，例如 COM7')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--send', action='store_true', help='每秒回发一个假目标角')
    ap.add_argument('--raw', action='store_true', help='打印原始十六进制')
    ap.add_argument('--dump', action='store_true', help='把原始字节存到 vision_raw.bin')
    args = ap.parse_args()

    print('=' * 66)
    print('电控 -> 视觉  四元数通信测试')
    print('  串口 %s @ %d' % (args.port, args.baud))
    print('  期望帧长 %d 字节（视觉->电控发出去的是 %d 字节）' % (TX_LEN, RX_LEN))
    print('  采样 5 秒，每 0.5 秒打一行...')
    print('=' * 66)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print('打开串口失败：%s' % e)
        print('  -> 检查串口号、是否被别的程序占用（SerialPlot 要先断开）')
        return 1

    buf = bytearray()
    frames = 0
    crc_err = 0
    head_err = 0
    last = 0
    dump_fp = open('vision_raw.bin', 'wb') if args.dump else None

    t0 = time.time()
    t_report = t0
    f_report = 0
    last_q = None
    q_changed = False
    send_t = t0
    fake_yaw = 0.0

    try:
        while time.time() - t0 < 5.0:
            chunk = ser.read(256)
            if chunk:
                if dump_fp:
                    dump_fp.write(chunk)
                if args.raw:
                    print('  RAW %s' % chunk[:TX_LEN].hex(' '))
                buf += chunk

            # 从缓冲里找帧头并取一帧
            while True:
                i = buf.find(MAGIC)
                if i < 0:
                    if len(buf) > 1:
                        buf = buf[-1:]
                    break
                if len(buf) - i < TX_LEN:
                    if i > 0:
                        del buf[:i]
                    break
                frame = bytes(buf[i:i + TX_LEN])
                del buf[:i + TX_LEN]

                if crc16_modbus(frame[:-2]) != struct.unpack('<H', frame[-2:])[0]:
                    crc_err += 1
                    continue

                (_, mode, q0, q1, q2, q3,
                 yaw, yaw_vel, pitch, pitch_vel,
                 bullet_speed, bullet_count, _) = struct.unpack(TX_FMT, frame)
                frames += 1
                last_q = (q0, q1, q2, q3)
                if last_q is not None and abs(q0 - 1.0) > 1e-4:
                    q_changed = True

                now = time.time()
                if now - t_report >= 0.5:
                    fps = (frames - f_report) / (now - t_report)
                    f_report = frames
                    t_report = now
                    n = quat_to_norm(last_q)
                    print('帧/s %5.1f | CRC错 %4d | q=(%7.4f %7.4f %7.4f %7.4f) |q|=%6.4f | '
                          'yaw %7.2f pitch %7.2f | mode=%s | 弹速 %5.1f 弹量 %d'
                          % (fps, crc_err, q0, q1, q2, q3, n,
                             yaw, pitch, MODE_TX.get(mode, mode),
                             bullet_speed, bullet_count))

            if args.send and time.time() - send_t >= 1.0:
                send_t = time.time()
                fake_yaw += 1.0
                ser.write(pack_rx(1, fake_yaw, 0.0, 0.0, 0.0, 0.0, 0.0))

    except KeyboardInterrupt:
        print('\n手动停止')
    finally:
        ser.close()
        if dump_fp:
            dump_fp.close()

    # ------------------------------------------------------------ 结论
    print('')
    print('=' * 66)
    if frames == 0:
        print('❌ 一帧都没收到')
        print('   1) 接线：MCU UART1_TX -> USB转TTL 的 RX，且必须共地')
        print('   2) 串口号对不对（用设备管理器确认，别和串口绘图那个口搞混）')
        print('   3) 视觉模块是否还挂在 UART1 上（两个发送端会打架，要拔掉）')
        print('   4) 固件有没有真的在发：SerialPlot 模式下 vfps 那个通道看不看得到')
        return 1
    if crc_err > 0:
        print('⚠️ 收到 %d 帧，但有 %d 帧 CRC 错 —— 多半是波特率或线材/干扰问题'
              % (frames, crc_err))
    else:
        print('✅ 收到 %d 帧，CRC 全部正确 —— 链路和协议都没问题' % frames)

    if last_q is None:
        print('（没有拿到四元数）')
    else:
        n = quat_to_norm(last_q)
        print('   最后一帧四元数 q(w,x,y,z) = (%.4f, %.4f, %.4f, %.4f)' % last_q)
        print('   范数 |q| = %.4f' % n)
        if abs(last_q[0]) < 1e-6 and abs(last_q[1]) < 1e-6 \
           and abs(last_q[2]) < 1e-6 and abs(last_q[3]) < 1e-6:
            print('❌ q 全为 0 —— 非法四元数（固件侧 bug，检查 qe_q 初值）')
        elif abs(n - 1.0) > 0.05:
            print('❌ |q| 明显偏离 1（%.4f）—— 四元数没归一化，EKF 有问题' % n)
        elif not q_changed:
            print('⚠️ q 恒为单位四元数 (1,0,0,0) —— 没在更新：')
            print('     · ATTITUDE_EKF_ENABLE 是不是 1')
            print('     · BMI088 有没有数据（mode 0 看 yaw/pitch 动不动）')
        else:
            print('✅ 四元数合法（|q|≈1）且在实时变化 —— 视觉侧可以放心用')
    if args.send:
        print('   （本次还回发了 %d 帧假目标角，可用于验证反向链路）' % int(time.time() - t0))
    print('=' * 66)
    return 0


if __name__ == '__main__':
    sys.exit(main())
