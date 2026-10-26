#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
电控 -> 视觉 发送帧监视器

用途：抓取 MCU 通过 USART1 发给视觉的 43 字节帧，逐字段解码打印。

接线方式（任选其一）：
  A) 把 MCU 的 USART1_TX 同时接到一个 USB-TTL 的 RX（并联监听，不影响原链路）
  B) 在视觉那台电脑上跑（先关掉视觉程序，独占串口）

用法：
  python tools/vision_tx_monitor.py --port COM7
  python tools/vision_tx_monitor.py --port COM7 --head both      # 同时认 AA AA 和 41 41
  python tools/vision_tx_monitor.py --port COM7 --raw            # 额外打印整帧 hex
  python tools/vision_tx_monitor.py --port COM7 --head aa        # 只认 AA AA

字段布局（43 字节）：
  0-1   head          u8[2]   帧头
  2     mode          u8      0=空闲/不控制 1=自瞄(控制) 2=其它
  3-18  q[4]          f32[4]  四元数 w,x,y,z（单位四元数）
  19-22 yaw           f32     弧度
  23-26 yaw_vel       f32     弧度/秒
  27-30 pitch         f32     弧度
  31-34 pitch_vel     f32     弧度/秒
  35-38 bullet_speed  f32     m/s
  39-40 bullet_count  u16     发数
  41-42 crc16         u16     CRC-16/MODBUS（小端）
"""
import argparse
import math
import struct
import sys
from datetime import datetime

FRAME_LEN = 43
HEAD_AA = b"\xAA\xAA"
HEAD_41 = b"\x41\x41"


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def quat_to_euler_deg(w, x, y, z):
    """返回 (roll, pitch, yaw) 单位：度"""
    # roll (x 轴)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y 轴)
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    # yaw (z 轴)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def parse_frame(frame: bytes):
    (mode,) = struct.unpack_from("<B", frame, 2)
    q = struct.unpack_from("<4f", frame, 3)
    yaw, yaw_vel, pitch, pitch_vel, bullet_speed = struct.unpack_from("<5f", frame, 19)
    (bullet_count,) = struct.unpack_from("<H", frame, 39)
    (crc_rx,) = struct.unpack_from("<H", frame, 41)
    return dict(mode=mode, q=q, yaw=yaw, yaw_vel=yaw_vel, pitch=pitch,
                pitch_vel=pitch_vel, bullet_speed=bullet_speed,
                bullet_count=bullet_count, crc_rx=crc_rx)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--head", choices=["both", "aa", "41"], default="both")
    ap.add_argument("--raw", action="store_true", help="额外打印整帧 hex")
    ap.add_argument("--raw-only", action="store_true",
                    help="只打印 43 字节原始数据（hex），不打印解析表格")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        print("需要 pyserial：pip install pyserial")
        return 1

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
    except Exception as e:
        print(f"打开 {args.port} 失败：{e}")
        return 1

    heads = []
    if args.head in ("both", "aa"):
        heads.append(HEAD_AA)
    if args.head in ("both", "41"):
        heads.append(HEAD_41)

    if not args.raw_only:
        print(f"监听 {args.port} @ {args.baud}  帧头={args.head}  帧长={FRAME_LEN}")
        print("-" * 118)
        print(f"{'时刻':<13}{'mode':>4} {'yaw°':>9} {'yawVel':>9} {'pitch°':>9} "
              f"{'pitVel':>9} {'弹速':>7} {'弹量':>5}  {'|q|':>6}  CRC")
        print("-" * 118)

    buf = bytearray()
    n_ok = n_bad = 0
    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                buf.extend(chunk)
            # 在缓冲里找帧
            while True:
                pos = -1
                for h in heads:
                    p = buf.find(h)
                    if p >= 0 and (pos < 0 or p < pos):
                        pos = p
                if pos < 0:
                    break
                if len(buf) - pos < FRAME_LEN:
                    if pos:
                        del buf[:pos]
                    break
                frame = bytes(buf[pos:pos + FRAME_LEN])
                del buf[:pos + FRAME_LEN]

                if args.raw_only:
                    print(frame.hex(" ").upper())
                    n_ok += 1
                    continue

                crc_calc = crc16_modbus(frame[:41])
                d = parse_frame(frame)
                crc_ok = (crc_calc == d["crc_rx"])
                n_ok += 1 if crc_ok else 0
                n_bad += 0 if crc_ok else 1

                qn = math.sqrt(sum(v * v for v in d["q"]))
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                if crc_ok:
                    crc_str = "ok"
                else:
                    crc_str = "BAD(%04X!=%04X)" % (crc_calc, d["crc_rx"])
                print(f"{ts:<13}{d['mode']:>4} "
                      f"{math.degrees(d['yaw']):>9.3f} "
                      f"{math.degrees(d['yaw_vel']):>9.2f} "
                      f"{math.degrees(d['pitch']):>9.3f} "
                      f"{math.degrees(d['pitch_vel']):>9.2f} "
                      f"{d['bullet_speed']:>7.2f} {d['bullet_count']:>5}  "
                      f"{qn:>6.4f}  {crc_str}")
                if args.raw:
                    print("     hex:", frame.hex(" ").upper())
                    r, p, y = quat_to_euler_deg(*d["q"])
                    print(f"     q=({d['q'][0]:.6f},{d['q'][1]:.6f},{d['q'][2]:.6f},{d['q'][3]:.6f})"
                          f"  -> roll={r:+.3f}° pitch={p:+.3f}° yaw={y:+.3f}°")
            # 缓冲过大就丢弃前半（防止一直找不到帧）
            if len(buf) > 4096:
                del buf[:2048]
    except KeyboardInterrupt:
        print("-" * 118)
        print(f"收到 {n_ok + n_bad} 帧：CRC 正常 {n_ok}，异常 {n_bad}")
    finally:
        ser.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
