#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从 MCU 内存里读取"电控 -> 视觉"的 43 字节发包内容（只读，不改任何东西）

原理：ELFs 里有 user_vision 的地址；通过 DAPLink(pyocd) 读 RAM，
      在 user_vision 结构体内扫描出 tx_packet（校验：帧头 41 41 + mode<=3 + |q|≈1 + CRC 正确）。

用法：
  python tools/read_tx_packet.py                       # 自动找 build/*.elf 和唯一探针
  python tools/read_tx_packet.py --elf build/xx.elf --probe 8d58cf65
  python tools/read_tx_packet.py --dump 0x20000920     # 直接 dump 某个地址 512 字节
"""
import argparse
import glob
import math
import os
import struct
import subprocess
import sys

FRAME = 43
GCC_BIN = r"C:\Users\xhao0\AppData\Local\stm32cube\bundles\gnu-tools-for-stm32\14.3.1+st.2\bin"


def crc16_modbus(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc >> 1) ^ 0xA001) if (crc & 1) else (crc >> 1)
    return crc


def find_symbol(elf, name):
    nm = os.path.join(GCC_BIN, "arm-none-eabi-nm.exe")
    if not os.path.exists(nm):
        nm = "arm-none-eabi-nm"
    out = subprocess.run([nm, elf], capture_output=True, text=True).stdout
    for line in out.splitlines():
        p = line.split()
        if len(p) == 3 and p[2] == name:
            return int(p[0], 16)
    return None


def decode(frame):
    mode = frame[2]
    q = struct.unpack_from("<4f", frame, 3)
    yaw, yaw_vel, pitch, pitch_vel, speed = struct.unpack_from("<5f", frame, 19)
    cnt, crc = struct.unpack_from("<H2B", frame, 39) if False else (struct.unpack_from("<H", frame, 39)[0],
                                                                   struct.unpack_from("<H", frame, 41)[0])
    return mode, q, yaw, yaw_vel, pitch, pitch_vel, speed, cnt, crc


def quat_euler_deg(q):
    w, x, y, z = q
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    sp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def scan_frames(buf):
    """在缓冲里找出所有通过校验的 tx_packet，返回 [(offset, frame)]"""
    hits = []
    for i in range(0, len(buf) - FRAME + 1):
        if buf[i] != 0x41 or buf[i + 1] != 0x41:
            continue
        fr = bytes(buf[i:i + FRAME])
        mode = fr[2]
        if mode > 3:
            continue
        q = struct.unpack_from("<4f", fr, 3)
        n = math.sqrt(sum(v * v for v in q))
        if not (0.85 < n < 1.15) or not all(math.isfinite(v) for v in q):
            continue
        crc_calc = crc16_modbus(fr[:41])
        crc_rx = struct.unpack_from("<H", fr, 41)[0]
        hits.append((i, fr, crc_calc, crc_rx, n))
    return hits


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--elf", default=None)
    ap.add_argument("--probe", default=None)
    ap.add_argument("--target", default="stm32f407ig")
    ap.add_argument("--len", type=int, default=1024, help="从 user_vision 起读多少字节")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    elf = args.elf
    if not elf:
        cands = glob.glob(os.path.join(root, "build", "*.elf"))
        if not cands:
            print("找不到 build/*.elf")
            return 1
        elf = max(cands, key=os.path.getmtime)
    print("ELF      :", elf)
    print("ELF 时间 :", __import__("datetime").datetime.fromtimestamp(os.path.getmtime(elf)))

    addr = find_symbol(elf, "user_vision")
    if addr is None:
        print("ELF 里找不到 user_vision 符号")
        return 1
    print("user_vision @ 0x%08X" % addr)

    try:
        from pyocd.core.helpers import ConnectHelper
    except ImportError:
        print("需要 pyocd：pip install pyocd")
        return 1

    probe = args.probe or ConnectHelper.get_all_connected_probes()[0].unique_id
    print("探针     :", probe)
    session = ConnectHelper.session_with_chosen_probe(unique_id=probe, target_override=args.target)
    if session is None:
        print("连接失败")
        return 1
    with session:
        tgt = session.target
        buf = tgt.read_memory_block8(addr, args.len)
    print("读到     : %d 字节" % len(buf))
    print()

    hits = scan_frames(buf)
    if not hits:
        print("✗ 在 user_vision + %d 字节范围内没找到合法的 tx_packet" % args.len)
        print("  （可能：ELF 与固件仍不一致 / tx_packet 在更后面 / 没在发）")
        return 2

    print("找到 %d 个候选，取第一个合法帧：" % len(hits))
    off, fr, crc_calc, crc_rx, qn = hits[0]
    print("  tx_packet 偏移 = +%d" % off)
    print()
    print("=== 43 字节原始数据（hex） ===")
    print("  " + fr.hex(" ").upper())
    print()
    mode, q, yaw, yaw_vel, pitch, pitch_vel, speed, cnt, crc = decode(fr)
    print("=== 逐字段 ===")
    print("  mode        = %d" % mode)
    print("  q(w,x,y,z)  = (%.6f, %.6f, %.6f, %.6f)   |q| = %.6f" % (q[0], q[1], q[2], q[3], qn))
    print("  yaw         = %+.6f rad = %+.3f deg" % (yaw, math.degrees(yaw)))
    print("  yaw_vel     = %+.6f rad/s = %+.3f deg/s" % (yaw_vel, math.degrees(yaw_vel)))
    print("  pitch       = %+.6f rad = %+.3f deg" % (pitch, math.degrees(pitch)))
    print("  pitch_vel   = %+.6f rad/s = %+.3f deg/s" % (pitch_vel, math.degrees(pitch_vel)))
    print("  bullet_speed= %+.6f m/s" % speed)
    print("  bullet_count= %d" % cnt)
    print("  crc16       = 0x%04X (计算 0x%04X) %s" % (crc_rx, crc_calc, "CRC OK" if crc_calc == crc_rx else "CRC BAD"))
    r, p, y = quat_euler_deg(q)
    print()
    print("=== 由 q 反解的欧拉角（视觉那边看到的是这个） ===")
    print("  roll  = %+.3f deg" % r)
    print("  pitch = %+.3f deg" % p)
    print("  yaw   = %+.3f deg" % y)
    print()
    print("=== 对比提示 ===")
    print("  视觉日志若打印 quat/yaw，应与此处的 q 和 yaw 完全一致；")
    print("  若视觉打印的 yaw ≈ 这里 q 反解的 yaw，而你们控制用的 yaw_fb 不同，")
    print("  说明【两套 yaw 积分器已经分叉】，需要在发包前把 q 的 yaw 对齐到 yaw_fb。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
