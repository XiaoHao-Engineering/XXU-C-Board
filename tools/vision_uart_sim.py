#!/usr/bin/env python3
import argparse
import math
import struct
import sys
import threading
import time
from dataclasses import dataclass

try:
    import serial
except ImportError:
    print("缺少 pyserial。先执行: pip install pyserial", file=sys.stderr)
    raise

HEAD0 = ord('A')
HEAD1 = ord('A')

# 视觉原始协议（与你当前改好的 dvc_vision.h 对齐）
# Vision -> Gimbal:
#   uint8 head[2], uint8 mode,
#   float yaw, yaw_vel, yaw_acc, pitch, pitch_vel, pitch_acc,
#   uint16 crc16
RX_FMT_NOCRC = '<BBBffffff'
RX_FMT = '<BBBffffffH'
RX_SIZE = struct.calcsize(RX_FMT)

# Gimbal -> Vision:
#   uint8 head[2], uint8 mode,
#   float q[4], yaw, yaw_vel, pitch, pitch_vel, bullet_speed,
#   uint16 bullet_count,
#   uint16 crc16
TX_FMT_NOCRC = '<BBBfffffffffH'
TX_FMT = '<BBBfffffffffHH'
TX_SIZE = struct.calcsize(TX_FMT)


@dataclass
class VisionToGimbal:
    mode: int
    yaw: float
    yaw_vel: float
    yaw_acc: float
    pitch: float
    pitch_vel: float
    pitch_acc: float


@dataclass
class GimbalToVision:
    mode: int
    q0: float
    q1: float
    q2: float
    q3: float
    yaw: float
    yaw_vel: float
    pitch: float
    pitch_vel: float
    bullet_speed: float
    bullet_count: int


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = ((crc >> 1) ^ 0xA001) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


def pack_vision_to_gimbal(pkt: VisionToGimbal) -> bytes:
    body = struct.pack(
        RX_FMT_NOCRC,
        HEAD0,
        HEAD1,
        pkt.mode & 0xFF,
        pkt.yaw,
        pkt.yaw_vel,
        pkt.yaw_acc,
        pkt.pitch,
        pkt.pitch_vel,
        pkt.pitch_acc,
    )
    crc = crc16_modbus(body)
    return body + struct.pack('<H', crc)


def unpack_gimbal_to_vision(frame: bytes) -> GimbalToVision:
    if len(frame) != TX_SIZE:
        raise ValueError(f'长度错误: 期望 {TX_SIZE}, 实际 {len(frame)}')

    body = frame[:-2]
    recv_crc = struct.unpack('<H', frame[-2:])[0]
    calc_crc = crc16_modbus(body)
    if calc_crc != recv_crc:
        raise ValueError(f'CRC错误: calc=0x{calc_crc:04X}, recv=0x{recv_crc:04X}')

    vals = struct.unpack(TX_FMT, frame)
    h0, h1 = vals[0], vals[1]
    if h0 != HEAD0 or h1 != HEAD1:
        raise ValueError(f'帧头错误: got=({h0:#04x},{h1:#04x})')

    return GimbalToVision(
        mode=vals[2],
        q0=vals[3],
        q1=vals[4],
        q2=vals[5],
        q3=vals[6],
        yaw=vals[7],
        yaw_vel=vals[8],
        pitch=vals[9],
        pitch_vel=vals[10],
        bullet_speed=vals[11],
        bullet_count=vals[12],
    )


class Reader(threading.Thread):
    def __init__(self, ser: serial.Serial):
        super().__init__(daemon=True)
        self.ser = ser
        self.running = True
        self.buf = bytearray()

    def run(self):
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    self.buf.extend(data)
                    self._parse_frames()
            except Exception as e:
                print(f'[RX] 串口读取异常: {e}', file=sys.stderr)
                time.sleep(0.05)

    def stop(self):
        self.running = False

    def _parse_frames(self):
        while True:
            if len(self.buf) < TX_SIZE:
                return

            start = -1
            for i in range(0, len(self.buf) - 1):
                if self.buf[i] == HEAD0 and self.buf[i + 1] == HEAD1:
                    start = i
                    break

            if start < 0:
                if len(self.buf) > 1:
                    self.buf = self.buf[-1:]
                return

            if start > 0:
                del self.buf[:start]

            if len(self.buf) < TX_SIZE:
                return

            frame = bytes(self.buf[:TX_SIZE])
            del self.buf[:TX_SIZE]

            try:
                pkt = unpack_gimbal_to_vision(frame)
                print(
                    '[RX OK] '
                    f'mode={pkt.mode} '
                    f'q=({pkt.q0:.4f},{pkt.q1:.4f},{pkt.q2:.4f},{pkt.q3:.4f}) '
                    f'yaw={pkt.yaw:.4f} yaw_vel={pkt.yaw_vel:.4f} '
                    f'pitch={pkt.pitch:.4f} pitch_vel={pkt.pitch_vel:.4f} '
                    f'bullet_speed={pkt.bullet_speed:.2f} bullet_count={pkt.bullet_count}'
                )
            except Exception as e:
                print(f'[RX] 丢弃一帧: {e}')


def main():
    parser = argparse.ArgumentParser(description='视觉串口仿真（对齐 AA 双字节帧头协议）')
    parser.add_argument('--port', required=True, help='串口号，例如 COM6 或 /dev/ttyUSB0')
    parser.add_argument('--baud', type=int, default=115200, help='波特率')
    parser.add_argument('--hz', type=float, default=100.0, help='发送频率')
    parser.add_argument('--mode', type=int, default=1, help='发送给电控的 mode: 0/1/2')
    parser.add_argument('--fire', action='store_true', help='快捷设置 mode=2')
    parser.add_argument('--yaw-amp', type=float, default=0.35, help='yaw 正弦振幅(rad)')
    parser.add_argument('--pitch-amp', type=float, default=0.12, help='pitch 正弦振幅(rad)')
    parser.add_argument('--yaw-bias', type=float, default=0.0, help='yaw 偏置(rad)')
    parser.add_argument('--pitch-bias', type=float, default=0.0, help='pitch 偏置(rad)')
    parser.add_argument('--yaw-freq', type=float, default=0.5, help='yaw 正弦频率(Hz)')
    parser.add_argument('--pitch-freq', type=float, default=0.5, help='pitch 正弦频率(Hz)')
    args = parser.parse_args()

    if args.fire:
        args.mode = 2

    period = 1.0 / args.hz

    ser = serial.Serial(
        port=args.port,
        baudrate=args.baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.02,
    )

    reader = Reader(ser)
    reader.start()

    print(f'[INFO] 已打开串口 {args.port} @ {args.baud}')
    print(f'[INFO] RX_SIZE={RX_SIZE}, TX_SIZE={TX_SIZE}, 发送频率={args.hz}Hz')
    print('[INFO] Ctrl+C 退出')

    t0 = time.time()
    last_print = 0.0

    try:
        while True:
            t = time.time() - t0

            wy = 2.0 * math.pi * args.yaw_freq
            wp = 2.0 * math.pi * args.pitch_freq

            yaw = args.yaw_bias + args.yaw_amp * math.sin(wy * t)
            yaw_vel = args.yaw_amp * wy * math.cos(wy * t)
            yaw_acc = -args.yaw_amp * wy * wy * math.sin(wy * t)

            pitch = args.pitch_bias + args.pitch_amp * math.sin(wp * t + math.pi / 3.0)
            pitch_vel = args.pitch_amp * wp * math.cos(wp * t + math.pi / 3.0)
            pitch_acc = -args.pitch_amp * wp * wp * math.sin(wp * t + math.pi / 3.0)

            pkt = VisionToGimbal(
                mode=args.mode,
                yaw=yaw,
                yaw_vel=yaw_vel,
                yaw_acc=yaw_acc,
                pitch=pitch,
                pitch_vel=pitch_vel,
                pitch_acc=pitch_acc,
            )
            frame = pack_vision_to_gimbal(pkt)
            ser.write(frame)

            if t - last_print > 0.5:
                last_print = t
                print(
                    '[TX] '
                    f'mode={pkt.mode} '
                    f'yaw={pkt.yaw:.4f} yaw_vel={pkt.yaw_vel:.4f} yaw_acc={pkt.yaw_acc:.4f} '
                    f'pitch={pkt.pitch:.4f} pitch_vel={pkt.pitch_vel:.4f} pitch_acc={pkt.pitch_acc:.4f}'
                )

            time.sleep(period)

    except KeyboardInterrupt:
        print('\n[INFO] 退出')
    finally:
        reader.stop()
        reader.join(timeout=0.5)
        ser.close()


if __name__ == '__main__':
    main()
