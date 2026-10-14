# -*- coding: utf-8 -*-
"""XXU-RoboMaster-Sentry 串口绘图 / 在线调参上位机

用法:
    python tools/serialplot.py                    # 自动列出串口（默认模式 2）
    python tools/serialplot.py COM4               # 直接指定串口
    python tools/serialplot.py COM4 4             # 指定串口和【绘图模式】0~4
    python tools/serialplot.py COM4 4 921600      # 再指定波特率
    python tools/serialplot.py COM4 4 115200 1    # 再强制"带 1 字节校验和"

    ★ 帧格式默认【自动识别】（靠"数据后面紧跟的是不是帧头"来判断），
      一般不需要管。固件的开关是 user_serialplot.h 的 SERIALPLOT_CHECKSUM_ENABLE。

    注意: 如果你用的是 SerialPlot（hyOzd 那个 Qt 上位机），本脚本用不上，
          直接按 docs/上机调试流程.md 的"SerialPlot 配置"一节填参数即可。

绘图模式（要和固件一致，也可以在板子运行中用 plot_mode=N# 在线切换）:
    0 = 姿态        yaw / pitch / omega / vx                        (4 通道)
    1 = 底盘 4 轮   目标 / 反馈 转子转速                             (8 通道)
    2 = 云台 yaw    gyaw / gtgt / gimu / gcmd / gspd                (5 通道)  ← 固件默认
    3 = 云台 pitch  pitch / ptgt / pspd / 外环积分 / 内环积分 / 电流  (6 通道)
    4 = 射击机构    frpm / frpm_f / frpm_t / ferr / fcur / bpitch / bv0 / fstate (8 通道)

    ★ 换模式只要改这个命令行参数（或在线发 plot_mode=N#），
      不需要改脚本里的 NAMES、也不需要重新编译固件。

注意（C 板）: 串口绘图走 USART1，CubeMX 里配的是 115200（和 A 板一致）。
              要是以后把 USART1 提到 921600 提高刷新率，
              这里也要跟着改（命令行最后一个参数）。

协议（和固件 user_serialplot.c 一致）:
    板子 -> PC :  0xAB + N x float32(小端) + 1 字节累加和
    PC -> 板子 :  name=value#        例如  wheel_kp=18.5#

依赖: pyserial, matplotlib
"""
import sys, struct, threading, time
from collections import deque
import tkinter as tk

import serial
from serial.tools import list_ports
import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ------------------------------------------------------------------ 配置
HEADER   = 0xAB
BAUD     = 115200       # 和固件 USART1 的波特率一致
MAX_PTS  = 500          # 每条曲线保留的点数

# 每个绘图模式对应的通道名。
# ★ 必须和固件 User_Application/Src/chassis_control.c 里
#   Chassis_Apply_Plot_Mode() 的 switch 一一对应，顺序也不能错。
MODE_NAMES = {
    # 0 = 姿态
    0: ["yaw", "pitch", "omega", "vx"],
    # 1 = 底盘 4 轮 目标 / 反馈转子转速
    1: ["FL tgt", "FL fdb", "FR tgt", "FR fdb",
        "RL tgt", "RL fdb", "RR tgt", "RR fdb"],
    # 2 = 云台 yaw 调试（固件默认）
    2: ["gyaw", "gtgt", "gimu", "gcmd", "gspd"],
    # 3 = 云台 pitch 调试
    #     ★ pid_i 是外环(角度环)积分、psd_i 是内环(速度环)积分。
    #       顶重力的是【内环积分 psd_i】—— 稳态时它应稳定在某个非零值上，
    #       pid_i 接近 0。松手后 pitch 慢慢下沉且 psd_i 顶不上去，
    #       就加大 pitch_spd_ki 或 pitch_spd_mi。
    3: ["pitch", "ptgt", "pspd", "pid_i", "psd_i", "pcur"],
    # 4 = 射击机构 + CAN 总线排错
    #     frpm 原始转速 / frpm_f 滤波后 / frpm_t 目标 / ferr 拨弹轮位置误差
    #     fcur 拨弹轮滤波后电流 / bpitch 弹道解算角 / bv0 解算用初速
    #     fstate 状态机 0=STOP 1=SPINUP 2=FIRE 3=REVERSE
    #     online 三电机反馈：bit0=左轮 bit1=右轮 bit2=拨弹轮（0=都没反馈）
    #     ready  摩擦轮是否已到转速（1=允许拨弹）
    #     can1_rx / can2_rx 两条 CAN 总线的【原始收帧数】累计值
    #       CAN1 有 4 底盘轮 + yaw 共 5 个电调 -> 约 +5000/s
    #       CAN2 只有 pitch 时约 +1000/s；加上 2 摩擦轮 + 拨弹轮应约 +4000/s
    #   ★ 排错：fstate 在变 但 online=0，且 can2_rx 只按 ~1000/s 涨
    #           -> 那 3 个电调根本没在发报文（查接线 / 供电 / ESC ID）
    4: ["frpm", "frpm_f", "frpm_t", "ferr", "fcur", "bpitch", "bv0", "fstate",
        "online", "ready", "can1_rx", "can2_rx"],
}

DEFAULT_MODE = 2        # 和固件 chassis_config.h 的 CHASSIS_SERIALPLOT_MODE 一致

# 当前使用的通道名（main() 里按模式重新赋值）
NAMES = list(MODE_NAMES[DEFAULT_MODE])


def pick_port():
    ports = list(list_ports.comports())
    if not ports:
        print("没有找到任何串口，检查 USB 线")
        sys.exit(1)
    print("可用串口:")
    for i, p in enumerate(ports):
        print("  [%d] %s  %s" % (i, p.device, p.description))
    if len(sys.argv) > 1:
        return sys.argv[1]
    s = input("选一个（填 COM 号或序号）: ").strip()
    if s.isdigit() and int(s) < len(ports):
        return ports[int(s)].device
    return s


def detect_frame_format(buf, data_len):
    """判断帧里到底带不带 1 字节校验和（只应在已经同步到帧头时调用）。

    两种假设都要用【连续两帧】互相印证，避免误判。
    设 d = 4×通道数（一帧的数据字节数）：
      不带校验和：帧长 1+d，所以第 2、3 帧的帧头分别在 1+d 和 2+2d
      带 1 字节： 帧长 1+d+1，校验和落在 1+d 处，
                  第 2 帧帧头在 1+d+1 处

    （只靠"1+d 处是不是 0xAB"判断的话，校验和恰好等于 0xAB 时会误判成
      "不带"，概率 1/256；要求两帧头部都对得上之后降到 1/65536。）

    返回: 'nocs' 不带 / 'cs' 带 / 'wait' 数据不够 / 'bad' 都不像
    """
    d = data_len
    if len(buf) < 2 + 2 * d + 1:
        return "wait"          # 还看不到第 3 帧的帧头，两种假设都判不了
    if buf[1 + d] == HEADER and buf[2 + 2 * d] == HEADER:
        return "nocs"
    if (sum(buf[1:1 + d]) & 0xFF) == buf[1 + d] and buf[1 + d + 1] == HEADER:
        return "cs"
    return "bad"


class App:
    def __init__(self, root, ser, nch, checksum=None):
        self.ser, self.nch = ser, nch
        self.data_len = 4 * nch
        # 帧长自适应：固件可能带 1 字节校验和（SERIALPLOT_CHECKSUM_ENABLE），
        # 也可能不带（默认不带，对应 SerialPlot 的 Simple binary 模式）。
        # checksum=None -> 自动识别；True/False -> 命令行强制。
        self.has_checksum = checksum
        # 先按"带校验和"（较长）来等数据，识别出来之后再改
        self.frame_len = 1 + self.data_len + 1
        self.lock = threading.Lock()
        self.ch = [deque([0.0] * MAX_PTS, maxlen=MAX_PTS) for _ in range(nch)]
        self.n_frames = 0
        self.n_bad = 0
        self.n_bytes = 0          # 累计收到的字节数
        self.err = None           # reader 线程的异常（有的话）
        self.running = True

        root.title("XXU-RoboMaster-Sentry  SerialPlot  @%s" % ser.port)
        root.geometry("1000x700")

        # ---- 图 ----
        self.fig = Figure(figsize=(9, 5), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("samples")
        self.ax.set_ylabel("value")
        self.lines = [self.ax.plot([], [], lw=1, label=NAMES[i])[0]
                      for i in range(nch)]
        self.ax.legend(loc="upper right", fontsize=8, ncol=2)
        self.canvas = FigureCanvasTkAgg(self.fig, master=root)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ---- 调参输入 ----
        bar = tk.Frame(root)
        bar.pack(fill="x", padx=6, pady=4)
        tk.Label(bar, text="在线调参 :").pack(side="left")
        self.entry = tk.Entry(bar, width=34)
        self.entry.pack(side="left", padx=4)
        self.entry.bind("<Return>", lambda e: self.send())
        tk.Button(bar, text="发送", command=self.send).pack(side="left")
        tk.Label(bar, text="例: gimbal_world_kp=0.5#   gimbal_ff=0.3#   head_kd=0.5#").pack(side="left", padx=10)

        self.status = tk.Label(root, text="", anchor="w")
        self.status.pack(fill="x", padx=6)

        threading.Thread(target=self.reader, daemon=True).start()
        root.after(50, self.refresh)

    # ----------------------------------------------------- 收
    def reader(self):
        buf = bytearray()
        while self.running:
            try:
                n = self.ser.in_waiting
                if n == 0:
                    time.sleep(0.004)
                    continue
                chunk = self.ser.read(n)
                if not chunk:
                    time.sleep(0.004)
                    continue
                self.n_bytes += len(chunk)
                buf += chunk

                while len(buf) >= self.frame_len:
                    i = buf.find(HEADER)
                    if i < 0:
                        buf.clear()
                        break
                    if i > 0:
                        del buf[:i]
                        continue

                    # ---- 帧格式自适应（只判一次）----
                    if self.has_checksum is None:
                        verdict = detect_frame_format(buf, self.data_len)
                        if verdict == "wait":
                            break
                        if verdict == "bad":
                            # 两种都不像，说明这个"帧头"是数据里的巧合，丢掉重找
                            self.n_bad += 1
                            del buf[:1]
                            continue
                        self.has_checksum = (verdict == "cs")
                        self.frame_len = 1 + self.data_len + (1 if self.has_checksum else 0)
                        print("[serialplot] 帧格式: %s（帧长 %d 字节）"
                              % ("帧头 + N×float + 1 字节校验和" if self.has_checksum
                                 else "帧头 + N×float（无校验和）", self.frame_len))

                    if len(buf) < self.frame_len:
                        break
                    frame = bytes(buf[:self.frame_len])
                    ok = ((sum(frame[1:-1]) & 0xFF) == frame[-1]) if self.has_checksum else True
                    if ok:
                        vals = struct.unpack_from("<%df" % self.nch, frame, 1)
                        with self.lock:
                            for k in range(self.nch):
                                self.ch[k].append(vals[k])
                            self.n_frames += 1
                        del buf[:self.frame_len]
                    else:
                        self.n_bad += 1
                        del buf[:1]

            except Exception as e:
                # 以前这里一抛异常线程就静默死掉，界面上帧数冻住但看不出原因
                self.err = repr(e)
                time.sleep(0.05)

    # ----------------------------------------------------- 发
    def send(self):
        cmd = self.entry.get().strip()
        if not cmd:
            return
        if not cmd.endswith("#"):
            cmd += "#"
        try:
            self.ser.write(cmd.encode("ascii"))
            self.entry.delete(0, "end")
        except Exception as e:
            print("发送失败:", e)

    # ----------------------------------------------------- 画
    def refresh(self):
        with self.lock:
            data = [list(c) for c in self.ch]
            nf, nb = self.n_frames, self.n_bad
        x = range(len(data[0]))
        for i, ln in enumerate(self.lines):
            ln.set_data(x, data[i])
        if nf > 0:
            self.ax.set_xlim(0, len(data[0]))
            lo = min(min(d) for d in data)
            hi = max(max(d) for d in data)
            pad = (hi - lo) * 0.1 or 1.0
            self.ax.set_ylim(lo - pad, hi + pad)
        txt = "帧 %d   校验错 %d   字节 %d   %s" % (
            nf, nb, self.n_bytes,
            " ".join("%s=%.0f" % (NAMES[i].replace(" ", ""), data[i][-1])
                     for i in range(min(self.nch, 5))))
        if self.err:
            txt += "    [读取异常] " + self.err
        elif nf == 0 and self.n_bytes == 0:
            txt += "    [一个字节都没收到 -> 检查接线/COM号]"
        self.status.config(text=txt)
        self.canvas.draw_idle()
        if self.running:
            self.root.after(50, self.refresh)

    def close(self):
        self.running = False
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    global NAMES

    # 第 2 个参数是【绘图模式】0~4（不是裸通道数），通道名与通道数都由它定，
    # 这样就不会出现"通道数比 NAMES 多 -> 画标签时 IndexError"的问题。
    if len(sys.argv) > 2:
        try:
            mode = int(sys.argv[2])
        except ValueError:
            print("第 2 个参数应该是绘图模式 0~4，收到 %r" % sys.argv[2])
            sys.exit(1)
        if mode not in MODE_NAMES:
            print("模式 %d 不存在。可选: %s" % (mode, sorted(MODE_NAMES)))
            sys.exit(1)
    else:
        mode = DEFAULT_MODE

    NAMES = list(MODE_NAMES[mode])
    nch = len(NAMES)

    baud = int(sys.argv[3]) if len(sys.argv) > 3 else BAUD

    # 可选第 4 个参数：强制校验和（0=不带，1=带）。不给就自动识别。
    checksum = None
    if len(sys.argv) > 4:
        checksum = (sys.argv[4].strip() != "0")

    port = pick_port()
    try:
        ser = serial.Serial(port, baud, timeout=0.05)
    except Exception as e:
        print("打开 %s 失败: %s" % (port, e))
        print("（串口被别的程序占用？关掉串口助手/SerialPlot 再试）")
        sys.exit(1)

    print("已连接 %s @%d，绘图模式 %d，通道数 %d，帧长 %d 字节"
          % (port, baud, mode, nch, 1 + 4 * nch + 1))
    print("通道: %s" % " | ".join(NAMES))
    print("帧格式会在收到第一帧数据后自动识别（带/不带校验和），识别结果打印在下面。")
    print("板子端要发 100Hz，波形应该动起来。不动就看下面几条。")
    print("提示: 板子在运行中也能切换通道，直接用调参发 plot_mode=N#，"
          "然后重启本脚本并换成对应的模式号。")

    root = tk.Tk()
    app = App(root, ser, nch, checksum)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
