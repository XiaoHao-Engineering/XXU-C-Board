#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
串口绘图格式自动识别（不用管 SerialPlot 怎么设）
================================================

作用：直接从串口读一段原始字节，自动推断固件【实际】发的是什么格式：
       帧长多少 / 几个通道 / 有没有校验 / 校验算法对不对，
      并把解出来的数值打印出来。

这样就能一次分清：
    · 是【固件发的格式】和 SerialPlot 的期望不一致
    · 还是 SerialPlot 的设置问题（那么本脚本会解得好好的）

用法：
    pip install pyserial
    python tools/plot_detect.py COM7
    python tools/plot_detect.py COM7 --baud 115200 --secs 2
"""

import argparse
import sys
from collections import Counter


def analyse(buf):
    """在原始字节里找 0xAB 的位置，并穷举"通道数 × 有无校验"看哪个组合能通过校验"""
    head = 0xAB
    pos = [i for i, b in enumerate(buf) if b == head]
    if not pos:
        print('  缓冲里一个 0xAB 都没有 —— 帧头不对，或没在发数据')
        return

    gaps = Counter(b - a for a, b in zip(pos, pos[1:]))
    print('  找到 %d 个 0xAB；相邻间距出现最多的是：%s'
          % (len(pos), ', '.join('%d字节×%d次' % (g, c) for g, c in gaps.most_common(4))))

    print('')
    print('  穷举验证（对每个 0xAB 位置都试一遍）：')
    hit = []
    for n in range(1, 13):
        for cs in (True, False):
            flen = 1 + n * 4 + (1 if cs else 0)
            ok = bad = 0
            for p in pos:
                fr = buf[p:p + flen]
                if len(fr) < flen:
                    continue
                if cs:
                    if (sum(fr[1:1 + n * 4]) & 0xFF) == fr[flen - 1]:
                        ok += 1
                    else:
                        bad += 1
                else:
                    ok += 1
            if ok >= 3 and bad == 0:
                hit.append((n, cs, flen, ok))
                print('    ★ 完全匹配：%2d 通道, 校验=%-5s 帧长=%2d 字节, 通过 %d 帧'
                      % (n, '有(和)' if cs else '无', flen, ok))
    if not hit:
        print('    （没有任何组合能全部通过校验 —— 见下面的原始字节）')

    print('')
    print('  原始字节（前 5 个 0xAB 起的 40 字节，十六进制）：')
    for p in pos[:5]:
        seg = buf[p:p + 40]
        print('    AB @%-5d  %s' % (p, ' '.join('%02X' % b for b in seg)))

    # 用最可能的组合解码
    if hit:
        n, cs, flen, _ = hit[0]
        print('')
        print('  按【%d 通道 + %s校验】解码前 3 帧：' % (n, '有' if cs else '无'))
        import struct
        cnt = 0
        for p in pos:
            fr = buf[p:p + flen]
            if len(fr) < flen:
                continue
            if cs and (sum(fr[1:1 + n * 4]) & 0xFF) != fr[flen - 1]:
                continue
            vals = struct.unpack('<%df' % n, fr[1:1 + n * 4])
            print('    帧%d: %s' % (cnt + 1, ' '.join('%9.4f' % v for v in vals)))
            cnt += 1
            if cnt >= 3:
                break


def main():
    ap = argparse.ArgumentParser(description='串口绘图格式自动识别')
    ap.add_argument('port', help='串口号，例如 COM7（绘图那个口）')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--secs', type=float, default=2.0, help='采样时长，默认 2 秒')
    a = ap.parse_args()

    try:
        import serial
    except ImportError:
        print('缺少 pyserial，请先执行：pip install pyserial')
        return 1

    print('=' * 70)
    print('串口绘图格式自动识别   端口 %s @ %d，采样 %.1f 秒' % (a.port, a.baud, a.secs))
    print('※ 请先在 SerialPlot 里点 Disconnect，避免两个程序抢同一个串口')
    print('=' * 70)
    try:
        ser = serial.Serial(a.port, a.baud, timeout=0.1)
    except Exception as e:
        print('打开串口失败：%s' % e)
        return 1

    buf = bytearray()
    t = 0.0
    while t < a.secs:
        chunk = ser.read(512)
        if chunk:
            buf += chunk
        t += 0.1
    ser.close()

    print('  共收到 %d 字节（%d 字节/秒）' % (len(buf), len(buf) / a.secs))
    print('')
    analyse(bytes(buf))
    print('')
    print('=' * 70)
    print('把上面的"★ 完全匹配"那一行告诉我，就知道该把 SerialPlot 设成什么了')
    return 0


if __name__ == '__main__':
    sys.exit(main())
