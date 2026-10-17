# -*- coding: utf-8 -*-
"""XXU-RoboMaster-Sentry 串口绘图 / 在线调参上位机

用法:
    python tools/serialplot.py                    # 自动列出串口
    python tools/serialplot.py COM4               # 直接指定串口
    python tools/serialplot.py COM4 0 921600      # 第 3 个参数是波特率（默认 115200）
    python tools/serialplot.py COM4 0 115200 1    # 再强制"带 1 字节校验和"

    ★ 帧格式默认【自动识别】（靠"数据后面紧跟的是不是帧头"来判断），
      一般不需要管。固件的开关是 user_serialplot.h 的 SERIALPLOT_CHECKSUM_ENABLE。

    注意: 如果你用的是 SerialPlot（hyOzd 那个 Qt 上位机），本脚本用不上，
          直接按 docs/上机调试流程.md 的"SerialPlot 配置"一节填参数即可。

★ 固件只有【一个固定绘图模式】：6 通道，上电即生效。
  没有模式切换，也没有 plot_mode 命令（第 2 个命令行参数只是为了兼容老习惯，
  现在只有 0 一个模式，填别的会直接提示不用填）。
  通道（从上到下，顺序不能改）:
    ch1  yaw_enc_abs     YAW 电机【单圈绝对角】    度 0~360  GM6020 13 位绝对编码器
    ch2  yaw_enc_rel     YAW【相对角】             度 ±180   对齐后的累计角
    ch3  pitch_enc_abs   pitch 电机【单圈绝对角】   度 0~360
    ch4  pitch_enc_rel   pitch【相对角】           度 ±180
    ch5  target_yaw      遥控器发的【目标 YAW】    度 ±180
    ch6  target_pitch    遥控器发的【目标 PITCH】  度
  线上帧长 = 1(帧头 0xAB) + 24(6 x float32) + 1(累加和) = 26 字节。

  ★ 用 SerialPlot（hyOzd）时填：Frame Start AB / #Channels 6 / Frame Size 24
    （Frame Size 是【数据字节数】= 6 x 4 = 24，不含帧头和校验和，填 26 是错的）

注意（C 板）: 串口绘图走 USART6，CubeMX 里配的是 115200（和 A 板一致）。
              要是以后把 USART6 提到 921600 提高刷新率，
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
BAUD     = 115200       # 和固件 USART6 的波特率一致
MAX_PTS  = 500          # 每条曲线保留的点数

# 通道名 —— 固件只有【这一个】固定模式（6 通道），上电即生效，没有模式切换。
# ★ 必须和固件 User_Application/Src/chassis_control.c 里
#   Chassis_Apply_Plot_Mode() 的顺序一一对应，不能错位。
#   协议帧长 = 1(帧头 AB) + 24(6 x float32) + 1(累加和) = 26 字节。
#
#   ch1  yaw_enc_abs     YAW 电机【单圈绝对角】     度 0~360  GM6020 13 位绝对编码器
#   ch2  yaw_enc_rel     YAW【相对角】              度 +-180   对齐后的累计角（user_gimbal.yaw）
#   ch3  pitch_enc_abs   pitch 电机【单圈绝对角】   度 0~360
#   ch4  pitch_enc_rel   pitch【相对角】            度 +-180   （user_gimbal.pitch）
#   ch5  target_yaw      遥控器发的【目标 YAW】     度 +-180   （user_gimbal.target_yaw）
#   ch6  target_pitch    遥控器发的【目标 PITCH】   度         （user_gimbal.target_pitch）
#
#   ★ 判据：拨右摇杆左右 -> ch5 目标 YAW 变、ch2/ch4（相对角）跟着走；
#     绝对角 ch1/ch3 只在 0~360 之间循环（过零点会从 360 跳回 0，不是坏数据）。
#   ★ SerialPlot（hyOzd）里必须设：Frame Start AB / #Channels 6 / Frame Size 24。
MODE_NAMES = {
    0: ["yaw_enc_abs", "yaw_enc_rel", "pitch_enc_abs",
        "pitch_enc_rel", "target_yaw", "target_pitch"],
}

DEFAULT_MODE = 0        # 固件只有一个固定模式（6 通道），这个 0 只是个占位键

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

    # 固件只有【一个固定模式】（6 通道），所以第 2 个参数只是为了兼容老命令：
    # 传 0 或不传都行，传别的直接提示"不用填"并退出（免得按错的通道数解析）。
    if len(sys.argv) > 2 and str(sys.argv[2]).strip() not in ("", "0", "DEFAULT", "default"):
        print("串口绘图现在【只有一个固定模式】（6 通道 / Frame Size 24），不用再传模式号；"
              "收到 %r。直接 python tools/serialplot.py COM4 即可。" % sys.argv[2])
        sys.exit(1)
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

    print("已连接 %s @%d，固定单模式，通道数 %d，帧长 %d 字节"
          % (port, baud, nch, 1 + 4 * nch + 1))
    print("通道: %s" % " | ".join(NAMES))
    print("帧格式会在收到第一帧数据后自动识别（带/不带校验和），识别结果打印在下面。")
    print("板子端发 100Hz，波形应该动起来。不动就按 docs/上机调试流程.md 的排查表看。")
    print("提示: 固件只有一个固定模式，没有 plot_mode 了；"
          "在线调参直接发 name=value#（例: gimbal_world_kp=0.5#）。")

    root = tk.Tk()
    app = App(root, ser, nch, checksum)
    root.protocol("WM_DELETE_WINDOW", lambda: (app.close(), root.destroy()))
    root.mainloop()


if __name__ == "__main__":
    main()
