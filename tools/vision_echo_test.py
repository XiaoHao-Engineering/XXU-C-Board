#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PC 假视觉 —— 四元数回环实验（yaw / pitch 的符号与数值对比）
==========================================================

用途（这个实验到底在看什么）
    PC 扮演"假视觉"，接在电控的 USART1（视觉口）上：
      1) 收  电控发来的 43 字节包（电控 -> 视觉），解析出四元数 q[4]
      2) 算  用 q 解出 yaw / pitch（欧拉角，标准 ZYX）
      3) 回  把这两个角【原样】当"目标角"发回去（29 字节，视觉 -> 电控，mode=1 只控制云台）
    于是电控收到的"目标角"就等于【它自己报出来的姿态角】。
    在串口绘图上看这两条曲线（mode 2 的 gyaw / gtgt，或 mode 5 的 vtgt / vyaw）：
      · 两条曲线重合          -> 协议、单位、符号全都对，回环成功
      · 两条曲线反号          -> 电控内部存在反向（本脚本回发的就是它自己解出的角度）
      · 两条曲线差 57.3 倍     -> 度 / 弧度不匹配，用 --deg 再试一次

★ 接线（最容易搞错的一步；注意本实验会占用 USART1）
     MCU USART1_TX  ->  USB转TTL-A 的 RX
     MCU USART1_RX  <-  USB转TTL-A 的 TX
     MCU GND        --  USB转TTL-A 的 GND        （必须共地，不共地必出 CRC 错）
     ⚠️ 串口绘图走的是 USART6，和视觉口不是同一个串口。本实验占用 USART1 后绘图照样能用，
        但需要【两个 USB 转 TTL】：A 接 USART1（跑本脚本），B 接 USART6（跑串口绘图/SerialPlot）。
     ⚠️ 测试时把真正的【视觉模块】从 USART1 上拔掉：UART 是点对点，两个发送端会互相打架。

协议（已确认，与固件 user_vision.h 一致；两份包都是小端 packed + CRC-16/MODBUS）
    电控 -> 视觉   43 字节   '<2sB9fHH'
      head(2) = 'A','A'（0x41 0x41，注意不是 0xAA）| mode(u8) |
      q[4](w,x,y,z) + yaw + yaw_vel + pitch + pitch_vel + bullet_speed（共 9 个 float）|
      bullet_count(u16) | crc16(u16)
    视觉 -> 电控   29 字节   '<2sB6fH'
      head(2) = 'A','A' | mode(u8) |
      yaw + yaw_vel + yaw_acc + pitch + pitch_vel + pitch_acc（共 6 个 float）|
      crc16(u16)
    ★ 回发包（29B）里的角度单位是【弧度】（与参考工程一致）；加 --deg 就改成"直接写度"。
    ★ 收到的 43B 包里【自带】的 yaw/pitch 字段，实测单位是【度】
      （见 --selftest 第 6 项：用用户真实抓包验证，四元数解出的 -163.40° 与包内 -163.56° 吻合）。
    CRC 覆盖除最后 2 字节以外的全部内容；crc16(b"123456789") == 0x4B37。

用法
    python tools/vision_echo_test.py COM27                 # 收 + 100Hz 回发（默认 mode=1、弧度）
    python tools/vision_echo_test.py COM27 --rate-hz 50    # 回发频率（Hz）
    python tools/vision_echo_test.py COM27 --mode 1        # 回发包里的 mode：0/1/2
    python tools/vision_echo_test.py COM27 --deg           # 回发包按"度"打包（默认弧度）
    python tools/vision_echo_test.py COM27 --no-echo       # 只收不回（对照组）
    python tools/vision_echo_test.py COM27 --raw           # 打印原始 hex（排错用）
    python tools/vision_echo_test.py COM27 --sine          # ★ 改成"主动发正弦目标角"（见下一节）
    python tools/vision_echo_test.py --selftest            # 自测，不需要串口、不需要 pyserial

正弦跟随测试（--sine：假视觉主动激励，看云台跟得怎么样）
    和上面的"回环"不同：--sine 下脚本【不再回发电控自己的姿态】，而是自己产生正弦目标角发出去，
    由云台（需要先让电控打开 yaw_auto）去跟随。这样在绘图模式 10 上就能量出跟随质量。

    命令示例
      python tools/vision_echo_test.py COM27 --sine
          # 默认：yaw = 0 + 30°·sin(2π·0.3Hz·t)，pitch = 5 + 10°·sin(2π·0.3Hz·t)，100Hz 回发，mode=1
      python tools/vision_echo_test.py COM27 --sine --amp-yaw 20 --freq-yaw 0.2 --amp-pitch 5 --freq-pitch 0.2
      python tools/vision_echo_test.py COM27 --sine --off-pitch 10 --phase-deg 90   # 90° = 李萨如图形
      python tools/vision_echo_test.py COM27 --sine --wait-echo                     # 收到帧后才开始发
    参数（角度单位都是【度】，频率单位 Hz；都是目标角本身的值，不是增量）
      --amp-yaw 30      yaw 正弦幅值（默认 30）
      --amp-pitch 10    pitch 正弦幅值（默认 10）
      --freq-yaw 0.3    yaw 正弦频率（默认 0.3）
      --freq-pitch 0.3  pitch 正弦频率（默认 0.3）
      --off-yaw 0       yaw 正弦中心（默认 0）
      --off-pitch 5     pitch 正弦中心（默认 5，pitch 给个中心值更真实）
      --phase-deg 0     yaw 与 pitch 的相位差（默认 0；给 90 画李萨如图形）
      --wait-echo       打开后仍要求"先收到电控的有效帧"再开始发（默认关：
                        正弦是主动激励，不必等电控先说话）

    ★ 接线（和回环实验完全一样，两个 USB 转 TTL）
        USB-TTL-A：RX <- MCU USART1_TX，TX -> MCU USART1_RX，GND -- GND（跑本脚本，发正弦）
        USB-TTL-B：接 MCU USART6，跑串口绘图 —— ★ 要发 `yaw_auto=1#` 让云台接管，
                   这个命令走 USART6 下行，所以【B 的 TX 必须接到 MCU 的 USART6_RX】，
                   否则命令根本到不了板子，云台永远不跟（只发一次，或重上电后再发）。
    ★ 怎么看拟合（绘图模式 10 的 vis_yaw/my_yaw 与 tgt_pitch/my_pitch 两对曲线）
        1) 滞后：实际角（my_yaw/my_pitch）比目标角晚一点点是正常的（控制延迟）；
           频率越高滞后越明显。两条曲线的过零点时间差 ÷ 周期 × 360° 就是相位滞后。
        2) 幅值衰减：实际曲线的峰-峰值应该 ≈ 目标峰-峰值的 2×amp；明显变小说明
           "跟不上/带宽不够"（前馈、kp、kd 不够，或视觉数据延迟大 —— 看 vdelay）。
        3) 静差：--off-yaw/--off-pitch 给了中心值时，实际曲线的中轴应落在同一水平线上；
           整条曲线平移一段 = 有静差（前馈/积分不够）。
        4) 反号：实际曲线与目标曲线反相（180°）就是电控内部方向反了 —— 先解决符号，
           再谈跟随（可用上面的回环实验一起确认）。
        5) 非线性：目标还在正弦，实际却削顶/平顶 = 撞到了角度限位或电流限幅，
           把 --amp-* 或 --off-* 调小一点。
    ⚠️ 提醒：--mode 0 时电控【不会用】这份数据控制云台（只验链路），要看跟随必须 mode=1 且 yaw_auto=1。

判据（脚本会自动给结论）
    ✅ 帧数持续增长 + CRC 全对        -> 链路和协议都没问题
    ✅ |q| ≈ 1                        -> 四元数是规范化的
    ✅ 绘图上 gyaw 与 vtgt(gtgt) 重合  -> 单位、符号都对，回环成功
    ⚠️ 绘图上两条曲线反号              -> 电控内部反向（脚本会打印这句提示）
    ⚠️ 差 57.3 倍                      -> 度/弧度不一致，加 --deg 再试
    ❌ 一帧都收不到                    -> 接线 / 串口号 / 视觉模块没拔

依赖：标准库 + pyserial（缺的时候会提示 pip install pyserial）
"""

import argparse
import math
import struct
import sys
import time


def _fix_pipe_encoding():
    """管道/重定向时 Windows 控制台默认用 GBK，emoji（✅⚠️❌）会 UnicodeEncodeError。

    只在【输出不是终端】时把 errors 改成 replace（不支持的字符退化成 '?'），
    交互式控制台走 WriteConsoleW，本来就能显示 emoji，不受影响。
    """
    for stream in (sys.stdout, sys.stderr):
        try:
            if not stream.isatty():
                stream.reconfigure(errors='replace')
        except Exception:
            pass


# ---------------------------------------------------------------- 协议定义
MAGIC = b'AA'          # 帧头是两个 ASCII 字符 'A','A'，即 0x41 0x41（不是 0xAA！）

RX_FMT = '<2sB9fHH'    # 电控 -> 视觉   2 + 1 + 36 + 2 + 2 = 43 字节
RX_LEN = struct.calcsize(RX_FMT)
TX_FMT = '<2sB6fH'     # 视觉 -> 电控   2 + 1 + 24 + 2     = 29 字节
TX_LEN = struct.calcsize(TX_FMT)

assert (RX_LEN, TX_LEN) == (43, 29), '协议结构体长度不对，检查 RX_FMT / TX_FMT'

MODE_TXT = {0: '不给控制', 1: '只控制云台', 2: '控制云台+请求开火'}


# ---------------------------------------------------------------- 基础工具
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


def _clamp(v, lo=-1.0, hi=1.0):
    return lo if v < lo else (hi if v > hi else v)


def _wrap180(a):
    """把角度差折到 (-180, 180]，yaw 跨 ±180 时用"""
    while a <= -180.0:
        a += 360.0
    while a > 180.0:
        a -= 360.0
    return a


def quat_to_euler(q):
    """四元数 (w, x, y, z) -> (roll, pitch, yaw)，单位【度】，标准 ZYX"""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(_clamp(2.0 * (w * y - z * x)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quat_norm(q):
    return math.sqrt(sum(v * v for v in q))


# 正弦模式（--sine）的默认参数：角度用【度】，频率用 Hz
SINE_DEFAULTS = {
    'amp_yaw': 30.0, 'amp_pitch': 10.0,
    'freq_yaw': 0.3, 'freq_pitch': 0.3,
    'off_yaw': 0.0, 'off_pitch': 5.0,
    'phase_deg': 0.0,
}


def to_packet_units(yaw_deg, pitch_deg, use_deg):
    """把【度】的目标角换算成"回发包里要写的数"。

    ★ 单位处理只在这里做一次：默认按【弧度】打包（与参考工程一致），
      --deg 时直接写【度】。回环模式和正弦模式都调这一个函数，不另写一套。
    """
    if use_deg:
        return yaw_deg, pitch_deg
    return math.radians(yaw_deg), math.radians(pitch_deg)


def sine_angles(t, amp_yaw, freq_yaw, off_yaw, amp_pitch, freq_pitch, off_pitch, phase_deg):
    """t 秒时的正弦目标角，返回 (yaw_deg, pitch_deg)。

    yaw(t)   = off_yaw   + amp_yaw  * sin(2π·freq_yaw·t)
    pitch(t) = off_pitch + amp_pitch* sin(2π·freq_pitch·t + phase_deg)

    ★ t 用调用方的【单调时钟】（time.monotonic 减去启动时刻），不用帧计数，
      这样发送抖动不会累积成相位误差。
    """
    yaw = off_yaw + amp_yaw * math.sin(2.0 * math.pi * freq_yaw * t)
    pitch = off_pitch + amp_pitch * math.sin(2.0 * math.pi * freq_pitch * t
                                             + math.radians(phase_deg))
    return yaw, pitch


# ---------------------------------------------------------------- 打包 / 解析
def pack_rx43(mode, q, yaw, yaw_vel, pitch, pitch_vel, bullet_speed, bullet_count):
    """打包一帧"电控 -> 视觉"（43 字节）。回环实验不需要发这个包，留给自测用。"""
    body = struct.pack(RX_FMT, MAGIC, mode & 0xFF,
                       q[0], q[1], q[2], q[3],
                       yaw, yaw_vel, pitch, pitch_vel, bullet_speed,
                       bullet_count & 0xFFFF, 0)
    return body[:-2] + struct.pack('<H', crc16_modbus(body[:-2]))


def pack_tx29(mode, yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc):
    """打包一帧"视觉 -> 电控"（29 字节）。

    yaw / pitch 的单位由调用者决定：默认传【弧度】，--deg 时直接传【度】。
    """
    body = struct.pack(TX_FMT, MAGIC, mode & 0xFF,
                       yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc, 0)
    return body[:-2] + struct.pack('<H', crc16_modbus(body[:-2]))


def parse_rx43(frame: bytes, verify_crc: bool = True):
    """解析 43 字节"电控 -> 视觉"帧。

    返回 dict（含 crc_ok 与解出的 roll/pitch/yaw 角度）；帧头或长度不对返回 None。
    verify_crc=False 时跳过 CRC 校验（crc_ok 记为 None），
    用于"尾部 CRC 被截断/是示意值"的抓包。
    """
    if len(frame) != RX_LEN or frame[:2] != MAGIC:
        return None
    if verify_crc:
        crc_ok = (crc16_modbus(frame[:-2]) == struct.unpack('<H', frame[-2:])[0])
    else:
        crc_ok = None

    (_, mode, q0, q1, q2, q3,
     yaw, yaw_vel, pitch, pitch_vel,
     bullet_speed, bullet_count, _) = struct.unpack(RX_FMT, frame)
    q = (q0, q1, q2, q3)
    roll_deg, pitch_deg, yaw_deg = quat_to_euler(q)
    return {
        'mode': mode, 'q': q, 'qnorm': quat_norm(q),
        'yaw': yaw, 'yaw_vel': yaw_vel, 'pitch': pitch, 'pitch_vel': pitch_vel,
        'bullet_speed': bullet_speed, 'bullet_count': bullet_count,
        'crc_ok': crc_ok,
        'roll_deg': roll_deg, 'pitch_deg': pitch_deg, 'yaw_deg': yaw_deg,
    }


def parse_tx29(frame: bytes, verify_crc: bool = True):
    """解析 29 字节"视觉 -> 电控"帧（自测用；回环时不解析自己发的包）。"""
    if len(frame) != TX_LEN or frame[:2] != MAGIC:
        return None
    if verify_crc:
        crc_ok = (crc16_modbus(frame[:-2]) == struct.unpack('<H', frame[-2:])[0])
    else:
        crc_ok = None
    (_, mode, yaw, yaw_vel, yaw_acc,
     pitch, pitch_vel, pitch_acc, _) = struct.unpack(TX_FMT, frame)
    return {
        'mode': mode, 'crc_ok': crc_ok,
        'yaw': yaw, 'yaw_vel': yaw_vel, 'yaw_acc': yaw_acc,
        'pitch': pitch, 'pitch_vel': pitch_vel, 'pitch_acc': pitch_acc,
    }


def compare_packet_angles(p):
    """把"包内自带的 yaw/pitch 字段"和"四元数解出的 yaw/pitch"比一比。

    固件里没写明包内角度字段的单位，这里把【度】和【弧度】两种可能都试一遍，
    取"yaw 差得最少"的那个当结论 —— 顺便把单位试出来。

    返回 (说明文本, 是否同号 or None, 轴向提示 or None)
    """
    yw_raw, pt_raw = p['yaw'], p['pitch']
    yaw_e, pitch_e, roll_e = p['yaw_deg'], p['pitch_deg'], p['roll_deg']

    best = None      # (|Δyaw|, 单位名, Δyaw, Δpitch, 包内 pitch 折成度)
    for unit, k in (('度', 1.0), ('弧度', math.degrees(1.0))):
        dy = _wrap180(yw_raw * k - yaw_e)
        dp = pt_raw * k - pitch_e
        if best is None or abs(dy) < abs(best[0]):
            best = (dy, unit, dy, dp, pt_raw * k)
    _, unit, dy, dp, pkt_pitch_deg = best

    same_sign = None
    if pkt_pitch_deg != 0.0 and yaw_e != 0.0:
        same_sign = (yw_raw * (1.0 if unit == '度' else math.degrees(1.0))) * yaw_e > 0.0

    text = ('包内自带 yaw=%+.3f pitch=%+.3f 按【%s】解释 | Δyaw=%+.2f° Δpitch=%+.2f° '
            '（解出 roll=%+.2f°）'
            % (yw_raw, pt_raw, unit, dy, dp, roll_e))

    hint = None
    if abs(dp) > 1.0 and abs(pkt_pitch_deg - roll_e) < 1.0:
        hint = ('轴向提示：包内 pitch=%+.2f° 与四元数解出的【roll】=%+.2f° 吻合，'
                '而解出的 pitch=%+.2f° 差 %.2f° -> 固件侧 pitch 疑似用了 q 的 x 分量'
                '（x/y 轴向约定不同），对照绘图 mode6 的 qpitch / qroll'
                % (pkt_pitch_deg, roll_e, pitch_e, dp))
    return text, same_sign, hint


# ---------------------------------------------------------------- 自测
# 用户真实抓到的 43 字节帧（末尾 CRC 被截断、是示意值 -> 自测时跳过 CRC 校验）。
# 只取前 43 字节，正好是完整一帧：AA + mode + q[4] + yaw/yaw_vel/pitch/pitch_vel/bspd + cnt + crc
SAMPLE_FRAME_HEX = (
    '41 41 00 37 E3 12 BE 0F 4B 0B BC 49 15 11 3D 78 BE 7C 3F '
    '12 90 23 C3 E0 A8 CC 3F 01 BA 85 40 00 00 00 00 00 00 00 '
    '00 00 00 00 00 00 01 02 03')

# 用户给的四元数（用来验证解析 + 欧拉角；手算值如下）
Q_USER = (0.9951, 0.0332, -0.0015, -0.0721)
Q_USER_ANGLES = (3.801014, 0.103255, -8.256126)      # (roll, pitch, yaw) 度，独立手算
# 由 yaw=-8.28°、pitch=3.82° 正解（ZYX）得到的四元数，用来做反解一致性检查
Q_FROM_ANGLES = (0.99683649, 0.00240619, 0.03324265, -0.07215366)


def do_selftest():
    """自测：不需要串口、不需要 pyserial。

    覆盖 CRC 标准向量 / 43B 打包-解析 round-trip / 欧拉角数值 /
    ZYX 反解一致性 / 29B 打包-解析 round-trip / 用户真实抓包一致性。
    """
    n_ok = [0]
    n_fail = [0]

    def check(name, ok, detail=''):
        if ok:
            n_ok[0] += 1
            print('[ OK ] %s%s' % (name, ('   ' + detail) if detail else ''))
        else:
            n_fail[0] += 1
            print('[FAIL] %s%s' % (name, ('   ' + detail) if detail else ''))

    print('=' * 78)
    print('自测 --selftest：构造帧 round-trip + CRC 标准向量 + 真实抓包校验')
    print('  结构体：RX_FMT=%r -> %d 字节 / TX_FMT=%r -> %d 字节'
          % (RX_FMT, RX_LEN, TX_FMT, TX_LEN))
    print('=' * 78)

    # ---- 1. CRC 标准向量 ----
    c = crc16_modbus(b'123456789')
    check('CRC-16/MODBUS 标准向量 crc16(b"123456789") == 0x4B37',
          c == 0x4B37, '算出 0x%04X' % c)

    # ---- 2. 43B round-trip：单位四元数 ----
    f = pack_rx43(0, (1.0, 0.0, 0.0, 0.0), 0.0, 0.0, 0.0, 0.0, 15.0, 7)
    check('43B 帧长 = 43 字节', len(f) == RX_LEN, '实际 %d' % len(f))
    p = parse_rx43(f)
    check('43B 帧可解析且 CRC 校验通过（单位四元数）',
          p is not None and p['crc_ok'] is True,
          '' if p else '解析失败')
    if p:
        check('  四元数原样解回 (1,0,0,0)',
              max(abs(a - b) for a, b in zip(p['q'], (1.0, 0.0, 0.0, 0.0))) < 1e-6,
              'q=(%.6f %.6f %.6f %.6f)' % p['q'])
        check('  单位四元数 -> roll=pitch=yaw=0°',
              max(abs(p['roll_deg']), abs(p['pitch_deg']), abs(p['yaw_deg'])) < 1e-4,
              'roll=%+.6f° pitch=%+.6f° yaw=%+.6f°'
              % (p['roll_deg'], p['pitch_deg'], p['yaw_deg']))
        check('  其余字段解回（mode/bullet_speed/bullet_count）',
              p['mode'] == 0 and abs(p['bullet_speed'] - 15.0) < 1e-4
              and p['bullet_count'] == 7,
              'mode=%d bspd=%.1f cnt=%d' % (p['mode'], p['bullet_speed'], p['bullet_count']))
        check('  |q| = 1', abs(p['qnorm'] - 1.0) < 1e-6, '|q|=%.6f' % p['qnorm'])

    # ---- 3. 43B round-trip：用户给的四元数 + 欧拉角 ----
    f = pack_rx43(0, Q_USER, 0.0, 0.0, 0.0, 0.0, 0.0, 0)
    p = parse_rx43(f)
    check('43B 帧可解析且 CRC 校验通过（用户四元数）',
          p is not None and p['crc_ok'] is True, '' if p else '解析失败')
    if p:
        check('  四元数 4 个分量 round-trip 一致',
              max(abs(a - b) for a, b in zip(p['q'], Q_USER)) < 1e-6,
              'q=(%.6f %.6f %.6f %.6f)  |q|=%.6f' % (p['q'] + (p['qnorm'],)))
        check('  解出 yaw ≈ -8.28°（手算 %.4f°）' % Q_USER_ANGLES[2],
              abs(p['yaw_deg'] - Q_USER_ANGLES[2]) < 0.01,
              '得 %+.6f°' % p['yaw_deg'])
        check('  解出 roll ≈ %+.4f° / pitch ≈ %+.4f°（手算值）'
              % (Q_USER_ANGLES[0], Q_USER_ANGLES[1]),
              abs(p['roll_deg'] - Q_USER_ANGLES[0]) < 0.01
              and abs(p['pitch_deg'] - Q_USER_ANGLES[1]) < 0.01,
              '得 roll=%+.6f° pitch=%+.6f°' % (p['roll_deg'], p['pitch_deg']))

        print('')
        print('  ── 关于给定期望值 yaw≈-8.28° / pitch≈3.82° 的说明 ──')
        print('  yaw  ：对上了（本工具公式算出 %+.4f°）' % p['yaw_deg'])
        print('  pitch：用【你给的公式】算出来是 %+.4f°，不是 +3.82°；' % p['pitch_deg'])
        print('         而 +3.82° 恰好落在【roll】上（公式算出 roll=%+.4f°）。' % p['roll_deg'])
        print('         同一现象在你给的真实 43B 抓包里也出现（见下面第 6 项）：')
        print('         包内 pitch=+4.18° ≈ 四元数解出的 roll +4.15°，而解出的 pitch 只有 +0.38°。')
        print('         => 固件侧"pitch"疑似用了 q 的 x 分量（x/y 轴向约定不同）。')
        print('            本脚本严格按你给的公式实现，没有擅自改轴/改符号；现场以绘图对比为准。')
        print('')

    # ---- 4. ZYX 反解一致性（正解 -> 反解应原样回来）----
    roll4, pitch4, yaw4 = quat_to_euler(Q_FROM_ANGLES)
    check('ZYX 反解一致性：q(yaw=-8.28°, pitch=3.82°) -> yaw/pitch 复现（±0.01°）',
          abs(yaw4 + 8.28) < 0.01 and abs(pitch4 - 3.82) < 0.01 and abs(roll4) < 0.01,
          '得 yaw=%+.4f° pitch=%+.4f° roll=%+.4f°' % (yaw4, pitch4, roll4))

    # ---- 5. 29B round-trip ----
    t = pack_tx29(1, -0.1445, 0.0, 0.0, 0.0667, 0.0, 0.0)
    check('29B 帧长 = 29 字节', len(t) == TX_LEN, '实际 %d' % len(t))
    q5 = parse_tx29(t)
    check('29B 帧可解析且 CRC 校验通过', q5 is not None and q5['crc_ok'] is True,
          '' if q5 else '解析失败')
    if q5:
        check('  29B round-trip：mode/yaw/pitch 一致',
              q5['mode'] == 1 and abs(q5['yaw'] + 0.1445) < 1e-6
              and abs(q5['pitch'] - 0.0667) < 1e-6,
              'mode=%d yaw=%+.4f rad pitch=%+.4f rad'
              % (q5['mode'], q5['yaw'], q5['pitch']))
        check('  29B 里的 yaw=-0.1445 rad 就是 -8.28°（度/弧度换算一致）',
              abs(math.degrees(q5['yaw']) + 8.2751) < 0.02,
              '= %+.2f°' % math.degrees(q5['yaw']))

    # ---- 6. 用户真实抓包（43B）一致性 ----
    raw = bytes(int(v, 16) for v in SAMPLE_FRAME_HEX.split())
    frame = raw[:RX_LEN]
    check('真实抓包至少有 43 字节', len(raw) >= RX_LEN, '抓包共 %d 字节' % len(raw))
    pr = parse_rx43(frame, verify_crc=False)
    check('真实抓包前 43 字节可解析（帧头 AA / mode / 9 float / u16）',
          pr is not None, '（尾部 CRC 是示意值，跳过 CRC 校验）')
    if pr:
        check('  |q| ≈ 1（真实抓包的四元数是规范化的）',
              abs(pr['qnorm'] - 1.0) < 0.01,
              'q=(%.4f %.4f %.4f %.4f) |q|=%.4f' % (pr['q'] + (pr['qnorm'],)))
        check('  四元数解出 yaw ≈ 包内自带 yaw 字段（%.2f）'
              % pr['yaw'],
              abs(_wrap180(pr['yaw_deg'] - pr['yaw'])) < 0.3,
              '解出 %+.2f° / 包内 %+.2f -> 包内角度字段单位是【度】'
              % (pr['yaw_deg'], pr['yaw']))
        print('[注意] 该帧：包内 pitch=%+.2f° ≈ 四元数解出的 roll=%+.2f°（解出 pitch=%+.2f°）；'
              % (pr['pitch'], pr['roll_deg'], pr['pitch_deg']))
        print('       包内 yaw_vel=%.2f pitch_vel=%.2f bullet_speed=%.2f bullet_count=%d'
              % (pr['yaw_vel'], pr['pitch_vel'], pr['bullet_speed'], pr['bullet_count']))

    # ---- 7. 正弦模式（--sine）：默认值 / 波形 / 单位复用 / 打包解回 ----
    d = build_parser().parse_args([])
    check('--sine 默认关闭、--wait-echo 默认关，参数默认值符合文档（30/10/0.3/0.3/0/5/0）',
          d.sine is False and d.wait_echo is False
          and (d.amp_yaw, d.amp_pitch) == (30.0, 10.0)
          and (d.freq_yaw, d.freq_pitch) == (0.3, 0.3)
          and (d.off_yaw, d.off_pitch, d.phase_deg) == (0.0, 5.0, 0.0),
          'amp=%.1f/%.1f freq=%.2f/%.2f off=%.1f/%.1f phase=%.1f'
          % (d.amp_yaw, d.amp_pitch, d.freq_yaw, d.freq_pitch,
             d.off_yaw, d.off_pitch, d.phase_deg))

    T = 1.0 / d.freq_yaw                       # 默认 0.3Hz -> 周期 3.333s
    y_q, p_q = sine_angles(T / 4.0, d.amp_yaw, d.freq_yaw, d.off_yaw,
                           d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    check('正弦 t=T/4（1/4 周期）= 峰值：yaw=+30°、pitch=+15°（误差 < 0.01°）',
          abs(y_q - 30.0) < 0.01 and abs(p_q - 15.0) < 0.01,
          '得 yaw=%+.4f° pitch=%+.4f°' % (y_q, p_q))
    y_3q, p_3q = sine_angles(3.0 * T / 4.0, d.amp_yaw, d.freq_yaw, d.off_yaw,
                             d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    check('正弦 t=3T/4（3/4 周期）= 谷值：yaw=-30°、pitch=-5°（误差 < 0.01°）',
          abs(y_3q + 30.0) < 0.01 and abs(p_3q + 5.0) < 0.01,
          '得 yaw=%+.4f° pitch=%+.4f°' % (y_3q, p_3q))
    y_0, p_0 = sine_angles(0.0, d.amp_yaw, d.freq_yaw, d.off_yaw,
                           d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    check('正弦 t=0 时回到中心值：yaw=0°、pitch=5°',
          abs(y_0) < 1e-9 and abs(p_0 - 5.0) < 1e-9,
          '得 yaw=%+.6f° pitch=%+.6f°' % (y_0, p_0))
    y_h, p_h = sine_angles(T / 2.0, d.amp_yaw, d.freq_yaw, d.off_yaw,
                           d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    check('正弦频率正确：t=T/2（半周期）又回到中心值',
          abs(y_h) < 0.01 and abs(p_h - 5.0) < 0.01,
          '得 yaw=%+.4f° pitch=%+.4f°' % (y_h, p_h))
    y_ph, p_ph = sine_angles(0.0, 30.0, 0.3, 0.0, 10.0, 0.3, 5.0, 90.0)
    check('--phase-deg 90：t=0 时 pitch 已到峰值（+15°）、yaw 仍在中心（0°）',
          abs(p_ph - 15.0) < 1e-9 and abs(y_ph) < 1e-9,
          '得 yaw=%+.6f° pitch=%+.6f°' % (y_ph, p_ph))
    a1 = sine_angles(12.3456, d.amp_yaw, d.freq_yaw, d.off_yaw,
                     d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    for tt in (0.0, 1.0, 2.5, 99.0):            # 先采样别的 t，再回到 12.3456
        sine_angles(tt, d.amp_yaw, d.freq_yaw, d.off_yaw,
                    d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    a2 = sine_angles(12.3456, d.amp_yaw, d.freq_yaw, d.off_yaw,
                     d.amp_pitch, d.freq_pitch, d.off_pitch, d.phase_deg)
    check('正弦只由 t 决定（与帧计数/调用历史无关 -> 发送抖动不累积成相位误差）',
          a1 == a2, 't=12.3456s 两次得 yaw=%+.4f° pitch=%+.4f°' % a2)

    yaw_pkt, pitch_pkt = to_packet_units(y_q, p_q, False)     # 默认：弧度
    check('单位处理复用同一个 to_packet_units：默认弧度（30° -> 0.5236 rad）',
          abs(yaw_pkt - math.radians(30.0)) < 1e-12
          and abs(pitch_pkt - math.radians(15.0)) < 1e-12,
          'yaw=%.6f rad pitch=%.6f rad' % (yaw_pkt, pitch_pkt))
    t29 = pack_tx29(1, yaw_pkt, 0.0, 0.0, pitch_pkt, 0.0, 0.0)
    q29 = parse_tx29(t29)
    check('正弦目标角 打包 -> 解回 一致（弧度、CRC 通过、解回 30°/15°）',
          q29 is not None and q29['crc_ok'] is True
          and abs(math.degrees(q29['yaw']) - 30.0) < 0.01
          and abs(math.degrees(q29['pitch']) - 15.0) < 0.01,
          '解回 yaw=%+.4f° pitch=%+.4f°' % (math.degrees(q29['yaw']),
                                            math.degrees(q29['pitch'])))
    yaw_d, pitch_d = to_packet_units(y_q, p_q, True)          # --deg：直接写度
    check('--deg 时 to_packet_units 直接给"度"（不再乘 radians）',
          yaw_d == 30.0 and pitch_d == 15.0,
          'yaw=%g pitch=%g' % (yaw_d, pitch_d))
    q29d = parse_tx29(pack_tx29(0, yaw_d, 0.0, 0.0, pitch_d, 0.0, 0.0))
    check('--deg 的包解回仍是 30/15（度）、mode=0 也照样解对',
          q29d is not None and q29d['crc_ok'] is True and q29d['mode'] == 0
          and abs(q29d['yaw'] - 30.0) < 1e-5 and abs(q29d['pitch'] - 15.0) < 1e-5,
          '解回 yaw=%+.4f pitch=%+.4f' % (q29d['yaw'], q29d['pitch']))

    print('')
    print('=' * 78)
    print('自测结果：%d 项 OK / %d 项 FAIL -> %s'
          % (n_ok[0], n_fail[0], '全部通过 ✅' if n_fail[0] == 0 else '有失败 ❌'))
    print('=' * 78)
    return 0 if n_fail[0] == 0 else 1


# ---------------------------------------------------------------- 主程序
def build_parser():
    """命令行参数（抽成函数是为了让 --selftest 能直接校验默认值）"""
    ap = argparse.ArgumentParser(
        description='PC 假视觉：收 43B 四元数 -> 解 yaw/pitch -> 回发 29B 目标角（回环对比）；'
                    '或用 --sine 主动发正弦目标角，测云台跟随效果')
    ap.add_argument('port', nargs='?', help='串口号，例如 COM27（--selftest 时可省略）')
    ap.add_argument('--baud', type=int, default=115200, help='波特率（默认 115200）')
    ap.add_argument('--rate-hz', type=float, default=100.0, dest='rate_hz',
                    help='发送频率 Hz（默认 100）')
    ap.add_argument('--mode', type=int, default=1, choices=(0, 1, 2),
                    help='发包里的 mode：0=不给控制 1=只控制云台 2=控制+请求开火（默认 1）。'
                         '★ --sine 时必须 1 或 2 云台才会接管；mode=0 只验链路不控云台')
    ap.add_argument('--deg', action='store_true',
                    help='包里的角度按【度】写（默认按弧度；现场发现固件要"度"时用这个）')
    ap.add_argument('--no-echo', action='store_true', dest='no_echo',
                    help='只收不回（对照组，单独验证接收链路）')
    ap.add_argument('--raw', action='store_true', help='打印原始 hex（排错用）')
    # ---- 正弦模式（主动激励，用来量云台跟随）----
    ap.add_argument('--sine', action='store_true',
                    help='★ 正弦模式：不回发电控自己的姿态，改为主动发正弦目标角（默认关）')
    ap.add_argument('--amp-yaw', type=float, default=SINE_DEFAULTS['amp_yaw'], dest='amp_yaw',
                    help='正弦模式：yaw 正弦幅值，单位【度】（默认 30）')
    ap.add_argument('--amp-pitch', type=float, default=SINE_DEFAULTS['amp_pitch'], dest='amp_pitch',
                    help='正弦模式：pitch 正弦幅值，单位【度】（默认 10）')
    ap.add_argument('--freq-yaw', type=float, default=SINE_DEFAULTS['freq_yaw'], dest='freq_yaw',
                    help='正弦模式：yaw 正弦频率，单位 Hz（默认 0.3）')
    ap.add_argument('--freq-pitch', type=float, default=SINE_DEFAULTS['freq_pitch'], dest='freq_pitch',
                    help='正弦模式：pitch 正弦频率，单位 Hz（默认 0.3）')
    ap.add_argument('--off-yaw', type=float, default=SINE_DEFAULTS['off_yaw'], dest='off_yaw',
                    help='正弦模式：yaw 正弦中心值，单位【度】（默认 0）')
    ap.add_argument('--off-pitch', type=float, default=SINE_DEFAULTS['off_pitch'], dest='off_pitch',
                    help='正弦模式：pitch 正弦中心值，单位【度】（默认 5，pitch 给个中心值更真实）')
    ap.add_argument('--phase-deg', type=float, default=SINE_DEFAULTS['phase_deg'], dest='phase_deg',
                    help='正弦模式：pitch 相对 yaw 的相位差，单位【度】（默认 0；给 90 画李萨如图形）')
    ap.add_argument('--wait-echo', action='store_true', dest='wait_echo',
                    help='正弦模式：仍要求"先收到电控的有效帧"再开始发（默认关 —— '
                         '正弦是主动激励，不必等电控先说话）')
    ap.add_argument('--selftest', action='store_true', help='跑自测（不需要串口）')
    return ap


def main():
    _fix_pipe_encoding()
    ap = build_parser()
    args = ap.parse_args()

    # ---- 自测分支：不碰串口、不需要 pyserial ----
    if args.selftest:
        return do_selftest()

    if not args.port:
        ap.error('缺少串口号，例如：python tools/vision_echo_test.py COM27')

    # ---- pyserial 延迟导入：只在上机路径上才要求装 ----
    try:
        import serial
    except ImportError:
        print('缺少 pyserial，请先安装：pip install pyserial')
        return 1

    rate = args.rate_hz if args.rate_hz > 0 else 100.0
    unit = '°' if args.deg else 'rad'

    print('=' * 78)
    if args.sine:
        print('PC 假视觉 —— ★正弦跟随测试（主动发正弦目标角，看云台跟得怎么样）')
        print('  串口 %s @ %d | 发送 %.1f Hz | mode=%d(%s) | 包内角度单位=%s | %s'
              % (args.port, args.baud, rate, args.mode, MODE_TXT.get(args.mode, '?'),
                 '度(--deg)' if args.deg else '弧度',
                 '只发不收(--no-echo)' if args.no_echo else '发 + 收'))
        print('  波形：yaw(t) = %g + %g·sin(2π·%g·t)° ；pitch(t) = %g + %g·sin(2π·%g·t + %g)°'
              % (args.off_yaw, args.amp_yaw, args.freq_yaw,
                 args.off_pitch, args.amp_pitch, args.freq_pitch, args.phase_deg))
        print('  帧长：收 %d 字节 / 发 %d 字节（正弦模式不依赖收包，收到只是顺手看电控状态）'
              % (RX_LEN, TX_LEN))
        print('  接线：USB-TTL-A 接 USART1（RX<-MCU_TX、TX->MCU_RX、必须共地）；')
        print('        串口绘图用 USB-TTL-B 接 USART6，★ B 的 TX 必须接到 MCU 的 USART6_RX，')
        print('        否则 yaw_auto=1# 这条下行命令到不了板子，云台永远不会跟。')
        print('  判据：绘图模式 10 把 vis_yaw/my_yaw、tgt_pitch/my_pitch 叠看：'
              '滞后多少 / 幅值是否衰减 / 有没有静差 / 有没有反号。')
        if args.mode == 0:
            print('  ⚠️ 当前 mode=0：电控【不会】用这份数据控制云台（只验链路）；'
                  '要看跟随请加 --mode 1。')
    else:
        print('PC 假视觉 —— 四元数回环实验（收 43B 解 yaw/pitch，回发 29B 目标角）')
        print('  串口 %s @ %d | 回发 %.1f Hz | mode=%d(%s) | 回发角度单位=%s | %s'
              % (args.port, args.baud, rate, args.mode, MODE_TXT.get(args.mode, '?'),
                 '度(--deg)' if args.deg else '弧度',
                 '只收不回(--no-echo)' if args.no_echo else '收 + 回发'))
        print('  帧长：收 %d 字节 / 发 %d 字节，两份包都是小端 packed + CRC-16/MODBUS'
              % (RX_LEN, TX_LEN))
        print('  接线：USART1_TX -> USB转TTL 的 RX，USART1_RX <- USB转TTL 的 TX，必须共地；')
        print('        串口绘图在 USART6，本实验要占用 USART1，所以需要两个 USB 转 TTL。')
        print('  判据：绘图上 gyaw/gtgt（或 vtgt/vyaw）两条曲线应【重合】；反号=电控内部反向。')
    print('=' * 78)

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.002)
    except Exception as e:
        print('打开串口失败：%s' % e)
        print('  -> 检查串口号，以及是否被别的程序占用（串口助手/SerialPlot 要先断开这个口）')
        return 1

    buf = bytearray()
    frames = 0            # 有效帧
    crc_err = 0           # CRC 错帧
    lost = 0              # 为重新同步丢掉的字节
    sent = 0              # 已发送帧数
    latest = None         # 最近一帧的解析结果
    send_yaw = send_pitch = 0.0   # 上一帧真正写进包里的值（单位由 --deg 决定）

    t0 = time.time()
    t_mono = time.monotonic()     # ★ 正弦相位用的单调时钟起点（不用帧计数，抖动不累积）
    t_report = t0
    f_report = 0
    next_send = t0
    last_sent_yaw_deg = None
    same_cnt = flip_cnt = 0
    warned_no_rx = False
    warned_no_send = False
    rate_warned = False
    axis_hint_printed = False

    try:
        while True:
            chunk = ser.read(256)
            if chunk:
                if args.raw:
                    print('  RAW %s' % chunk[:RX_LEN].hex(' '))
                buf += chunk

            # ---- 从缓冲里找帧头、取一帧、校验 CRC ----
            while True:
                i = buf.find(MAGIC)
                if i < 0:
                    if len(buf) > 1:           # 最后 1 字节可能是"半个帧头"
                        lost += len(buf) - 1
                        del buf[:len(buf) - 1]
                    break
                if i > 0:
                    lost += i
                    del buf[:i]
                if len(buf) < RX_LEN:
                    break                       # 半帧，等下一次读
                p = parse_rx43(bytes(buf[:RX_LEN]), verify_crc=True)
                if p is None or not p['crc_ok']:
                    crc_err += 1
                    lost += 1
                    del buf[:1]                 # 只丢 1 字节，重新同步（别丢整帧）
                    continue
                del buf[:RX_LEN]
                frames += 1
                latest = p

            now = time.time()

            # ---- 没收到数据时的提示（1 秒后提醒一次）----
            if latest is None and not warned_no_rx and now - t0 > 1.0:
                warned_no_rx = True
                print('⚠️ 1 秒内没收到有效帧 —— 查接线（TX/RX 是否交叉、有没有共地）、'
                      '串口号、波特率，以及视觉模块是否还挂在 USART1 上')

            # ---- 发送（按时间片，简单可靠；不落后于累积的 next_send）----
            if not args.no_echo and now >= next_send:
                tgt = None                      # 本拍要发的目标角 (yaw_deg, pitch_deg)
                if args.sine:
                    # ★ 正弦模式是【主动激励】：默认不必等电控先发帧；--wait-echo 才等
                    if latest is None and args.wait_echo:
                        if not warned_no_send:
                            warned_no_send = True
                            print('  （--wait-echo 已打开：还没收到电控的有效帧，先不发正弦）')
                    else:
                        ts = time.monotonic() - t_mono
                        tgt = sine_angles(ts, args.amp_yaw, args.freq_yaw, args.off_yaw,
                                          args.amp_pitch, args.freq_pitch, args.off_pitch,
                                          args.phase_deg)
                elif latest is not None:
                    # 回环模式：保持"没收到有效帧前不回发"（避免把云台拉到 0°）
                    tgt = (latest['yaw_deg'], latest['pitch_deg'])
                elif not warned_no_send:
                    warned_no_send = True
                    print('  （还没收到有效帧，暂不回发 —— 避免把云台拉到 0°）')

                if tgt is not None:
                    # ★ 单位处理统一走 to_packet_units：回环 / 正弦共用同一套，不另写一遍
                    send_yaw, send_pitch = to_packet_units(tgt[0], tgt[1], args.deg)
                    try:
                        ser.write(pack_tx29(args.mode, send_yaw, 0.0, 0.0,
                                            send_pitch, 0.0, 0.0))
                    except Exception as e:
                        print('串口写失败：%s（USB 掉了？）' % e)
                        break
                    sent += 1
                    last_sent_yaw_deg = tgt[0]
                    next_send += 1.0 / rate
                    if next_send < now - 0.5:   # 落后太多（比如刚开串口）就重新对齐
                        next_send = now + 1.0 / rate
                if next_send <= now:
                    next_send = now + 1.0 / rate

            # ---- 实际发送速率跟不上时提醒一次（时间片调度的上限 = 主循环速率）----
            if (not args.no_echo and not rate_warned and now - t0 > 2.0
                    and sent < 0.8 * rate * (now - t0)):
                rate_warned = True
                print('⚠️ 实际发送仅 %.1f Hz，低于请求的 %.1f Hz —— 主循环被"串口读超时"限制了：'
                      '把 --rate-hz 调小一点，或先关掉 --raw（它每帧都打印，最拖速度）'
                      % (sent / (now - t0), rate))

            # ---- 每 0.5 秒打一行，方便对照绘图 ----
            if now - t_report >= 0.5 and (latest is not None or args.sine):
                fps = (frames - f_report) / (now - t_report)
                f_report, t_report = frames, now
                if latest is not None:
                    w, x, y, z = latest['q']
                    print('收 %5.1f fps | q=(%7.4f %7.4f %7.4f %7.4f) |q|=%6.4f | '
                          '解出 yaw=%7.2f° pitch=%7.2f°'
                          % (fps, w, x, y, z, latest['qnorm'],
                             latest['yaw_deg'], latest['pitch_deg']))
                else:
                    print('收   --.- fps | （还没收到有效帧；%s）'
                          % ('正弦模式照样在发' if args.sine else '检查接线/串口号/波特率'))

                if args.sine:
                    # 打印用当前时刻的波形值（--no-echo / --wait-echo 没在发时也照样显示"目标"）
                    ts = time.monotonic() - t_mono
                    disp_yaw, disp_pitch = sine_angles(ts, args.amp_yaw, args.freq_yaw,
                                                       args.off_yaw, args.amp_pitch,
                                                       args.freq_pitch, args.off_pitch,
                                                       args.phase_deg)
                    if args.no_echo:
                        print('正弦 t=%.2fs | 目标 yaw=%+.1f° pitch=%+.1f° | (no-echo) 只收不回 | '
                              '累计 收 %d 帧 / CRC错 %d'
                              % (ts, disp_yaw, disp_pitch, frames, crc_err))
                    else:
                        print('正弦 t=%.2fs | 目标 yaw=%+.1f° pitch=%+.1f° | '
                              '发回 yaw=%+.4f %s pitch=%+.4f %s | 已发 %d 帧'
                              % (ts, disp_yaw, disp_pitch,
                                 send_yaw, unit, send_pitch, unit, sent))
                elif args.no_echo:
                    print('(no-echo) 只收不回 | 累计 收 %d 帧 / CRC错 %d / 丢 %d 字节'
                          % (frames, crc_err, lost))
                else:
                    print('发 回: yaw=%+.4f %s pitch=%+.4f %s  mode=%d | '
                          '累计 收 %d 帧 / CRC错 %d / 发 %d 帧'
                          % (send_yaw, unit, send_pitch, unit, args.mode,
                             frames, crc_err, sent))

                if latest is not None:
                    text, same_sign, hint = compare_packet_angles(latest)
                    print('  · %s' % text)
                    if same_sign is not None:
                        if same_sign:
                            same_cnt += 1
                        else:
                            flip_cnt += 1
                    if hint and not axis_hint_printed:
                        axis_hint_printed = True
                        print('  ⚠ %s' % hint)

    except KeyboardInterrupt:
        print('\n手动停止（Ctrl+C）')
    except Exception as e:
        print('\n串口异常：%s' % e)
    finally:
        try:
            ser.close()
        except Exception:
            pass

    # ------------------------------------------------------------ 总结
    dt = time.time() - t0
    print('')
    print('=' * 78)
    print('总结：运行 %.1f s | 收 %d 帧（平均 %.1f fps）| CRC 错 %d 帧 | 丢弃 %d 字节 | %s %d 帧'
          % (dt, frames, (frames / dt) if dt > 0 else 0.0, crc_err, lost,
             '发送' if args.sine else '回发', sent))

    if frames == 0 and not args.sine:
        print('❌ 一帧都没收到，按顺序查：')
        print('   1) 接线：MCU USART1_TX -> USB转TTL 的 RX（交叉接！），并且必须共地')
        print('   2) 串口号：别和串口绘图那个口（USART6）搞混')
        print('   3) 视觉模块是否还挂在 USART1 上（两个发送端会打架，要拔掉）')
        print('   4) 波特率是不是 115200（固件 USART1 的配置）')
        return 1
    if crc_err > 0:
        print('⚠️ 有 %d 帧 CRC 错 —— 多半是波特率、线材、共地或干扰问题' % crc_err)
    else:
        print('✅ CRC 全部正确 —— 链路和协议都没问题')

    if args.sine:
        # 正弦模式：给"怎么看跟随"的收尾提示
        print('')
        print('正弦模式提示（要看到"跟随"，必须做这三件事）：')
        print('  1) 让电控打开视觉接管：在【串口绘图那个串口】发 yaw_auto=1#')
        print('     ⚠️ 这条命令走 USART6 下行 —— 串口绘图那个 USB-TTL 的 TX 必须接到 MCU 的')
        print('        USART6_RX，否则命令根本到不了板子，云台不会跟（发一次就行，'
              '重上电要再发一次）。')
        print('  2) 绘图切到模式 10：板子发 plot_mode=10#，上位机设 #Channels 4 / Frame Size 16')
        print('     （tools/serialplot.py COMx 10，或 SerialPlot 里 Frame Start AB）。')
        print('  3) 叠看两对曲线：vis_yaw/my_yaw 与 tgt_pitch/my_pitch，重点看滞后多少、'
              '幅值有没有衰减、')
        print('     有没有静差、有没有反号（pitch 这一对是"内部目标关节角 vs 实际关节角"）。')
        if args.mode == 0:
            print('  ⚠️ 本次 mode=0：电控不会用这份数据控制云台，只验了链路；'
                  '要看跟随请用 --mode 1。')
        print('  本次正弦：yaw = %g + %g·sin(2π·%g·t)° ，pitch = %g + %g·sin(2π·%g·t + %g)° ，'
              '%.1f Hz 共 %d 帧'
              % (args.off_yaw, args.amp_yaw, args.freq_yaw, args.off_pitch, args.amp_pitch,
                 args.freq_pitch, args.phase_deg, rate, sent))
        if frames == 0:
            print('  （本次没收到电控的帧：正弦模式不影响发送，但收不到就没法在脚本里对比两侧，'
                  '建议顺手确认 USART1 接线）')
    else:
        print('')
        print('符号判断：')
        if last_sent_yaw_deg is not None and latest is not None:
            print('  解出 yaw=%+.2f°  vs  上一帧回发 yaw=%+.2f°  -> %s'
                  % (latest['yaw_deg'], last_sent_yaw_deg,
                     '同符号' if last_sent_yaw_deg * latest['yaw_deg'] >= 0 else '反号'))
        print('  ✅ 本脚本回发的就是它自己解出的角度（同符号），'
              '若绘图上两者反号，说明电控内部存在反向')
        if same_cnt + flip_cnt > 0:
            print('  · 包内自带 yaw 与四元数解出 yaw 的符号：同号 %d 次 / 反号 %d 次'
                  % (same_cnt, flip_cnt))
            if flip_cnt > same_cnt:
                print('    ⚠️ 反号占多数 -> 电控发出的 yaw 字段与它自己的四元数约定不一致（固件侧符号问题）')
        if latest is not None:
            print('  最后一次：q=(%.4f %.4f %.4f %.4f) |q|=%.4f -> yaw=%+.2f° pitch=%+.2f° '
                  '（包内 yaw=%+.2f pitch=%+.2f）'
                  % (latest['q'] + (latest['qnorm'], latest['yaw_deg'], latest['pitch_deg'],
                                    latest['yaw'], latest['pitch'])))
        print('  提示：--deg 只改"包里的角度单位"；若绘图上是 57.3 倍关系，加 --deg 再跑一次。')
        print('  提示：想看"云台跟随正弦"的效果，用 --sine（见 docstring 的「正弦跟随测试」）。')
    print('=' * 78)
    return 0


if __name__ == '__main__':
    sys.exit(main())
