# 新乡学院 RoboMaster 哨兵底盘框架

大疆 A 板（STM32F427IIH6）麦轮 / 全向轮底盘框架。CMake + Ninja + VSCode。
A 板装在**云台**上，板载 IMU 只解算 **Yaw + Pitch** 两轴；云台 yaw 用 **GM6020** 的绝对编码器。

> **V1.3** —— 云台**锁世界方向**（IMU 反馈 + 底盘角速度前馈）；航向补偿按模式切换
> （手动=车头方向，跟随/小陀螺=云台指向）；修复编码器 8191/8192 角度换算导致的行进累积偏差。
>
> **V1.2** —— 已上车实测：四轮运动方向正确；GM6020 云台 yaw 遥控正常、掉线保护正常、上电不冲。
>
> 控制原理（小陀螺走直线 / 跟随）另见 [`docs/小陀螺与跟随_控制原理.md`](docs/小陀螺与跟随_控制原理.md)。

## 架构

```
上层  User_Application/   chassis.c          底盘对象（逆解 / 正解 / 下发电机）
                          chassis_control.c  遥控映射、模式切换、航向补偿、在线调参
                          gimbal.c           云台 yaw 角度环 + 云台相对底盘角度
                          chassis_config.h   所有可调参数集中在这里
中层  User_Algorithm/     User_Chassis/      麦轮 / 全向轮运动学
                          User_Attitude/     互补滤波姿态解算（Yaw / Pitch）
                          user_pid / user_cf / user_coord ...
底层  User_Drives/        CAN、DJI 电机（含 GM6020）、DBUS、UART、SPI、MPU6500
      Core/Src/bsp.c      全局实例表 + 主循环事件表
工具  tools/              serialplot.py      串口绘图 / 在线调参上位机
```

## 运行时任务

| 频率 | 在哪 | 干什么 |
|---|---|---|
| **1 kHz** | SysTick → `chassis_task` | 遥控 → 速度 → 航向补偿 → 逆解 → 下发电机 |
| **1 kHz** | SysTick → `gimbal_task` | 读 GM6020 绝对角度 → 云台 yaw 角度环 |
| **1 kHz** | SysTick → `MPU6500_Handle` | 读 IMU → 姿态解算 |
| **500 Hz** | SysTick → `DJI_Motor_Execute` | 组帧发 4 个 M3508 + GM6020 |
| 100 Hz | SysTick → `Serialplot_Output` | 串口绘图上报 |
| 10 kHz | TIM2 中断 | 往 JScope 喂 `vx / vy / omega` |
| 事件 | CAN1_RX0 中断 | 解析电机反馈（含云台编码器）+ 跑 PID |
| 事件 | USART1 中断 | DBUS 空闲中断判帧尾 |
| 事件 | USART2 中断 | 接收串口调参指令 |
| 事件 | `while(1)` | 排空 CAN 发送队列、UART 收发队列 |

## 控制链路

```
遥控器 ──DBUS──▶ USART1 ──▶ user_dbus
                              │
                    ┌─────────┴──────────┐
                    ▼                    ▼
        Gimbal_Update @1kHz      ChassisControl_Update @1kHz
          └ GM6020 角度环           ├ 断线 >100ms → 速度清零
              │                     ├ 左拨杆 sw1 切模式
        绝对编码器反馈              ├ 左摇杆 → 平移 (ch3/ch2)
              │                     ├ 右摇杆 → 自转 (ch0)
              │                     ├ 航向补偿：参考系 → 车体系
              │                     └ Chassis_Set_Velocity → Chassis_Update
              │                               │
        云台相对底盘角度 ──────────────────────▶ (作为航向源 θ)
                                                │
                                    Chassis_Update
                                      ├ 逆解：(vx,vy,ω) → 4 轮线速度 m/s
                                      └ × 减速比 → 转子 RPM → 下发电机目标
                                                │
                            DJI_Motor_Execute @500Hz 组帧
                                                │
                              CAN1 @1Mbps ──▶ C620 ×4 + GM6020 ──▶ 电机
                                                │
                            反馈 0x201~0x204 / 0x205~0x20B ──▶ 轮速环 / 云台角度环
```

## 电机 ID 与轮序（重要）

**代码里的轮序**（`chassis_kinematics.h`）：`0=左前 1=右前 2=左后 3=右后`

**本车实际电机 ID 不是按轮序顺排的**，`main.c` 里按实际接线绑定：

| 逻辑轮子 | 变量 | 实际电机 ID |
|---|---|---|
| 左前 | `user_wheel_fl` | **2** |
| 右前 | `user_wheel_fr` | **3** |
| 左后 | `user_wheel_rl` | **1** |
| 右后 | `user_wheel_rr` | **4** |

```c
/* Core/Src/main.c */
DJI_Motor_Init(&user_wheel_fl, &user_can_1, 2, 0.0f, M3508_gear, Rotor_speed, ...);
DJI_Motor_Init(&user_wheel_fr, &user_can_1, 3, 0.0f, M3508_gear, Rotor_speed, ...);
DJI_Motor_Init(&user_wheel_rl, &user_can_1, 1, 0.0f, M3508_gear, Rotor_speed, ...);
DJI_Motor_Init(&user_wheel_rr, &user_can_1, 4, 0.0f, M3508_gear, Rotor_speed, ...);
```

> **ID 绑错 = 置换错位**，运动学会整个乱掉（平移变自转、自转变打滑），
> 而且 **`CHASSIS_REV_*` 怎么翻都修不好**。所以这一步必须对。

**换车 / 换电调后，用 `test_wheel` 在线确认一遍**（见「上车标定流程」）。
更规范的做法是用 C620 拨码把 ID 改成 `左前=1 右前=2 左后=3 右后=4`，代码就不用特殊处理了。

## 轮子方向（`CHASSIS_REV_*`）

麦轮底盘**左右两侧电机是镜像安装**的，同一命令下两侧实际转向相反，用 `CHASSIS_REV_*` 补偿：

```c
/* User_Application/chassis_config.h */
#define CHASSIS_REV_FL    (1)
#define CHASSIS_REV_FR    (-1)     /* 本车翻右侧 */
#define CHASSIS_REV_RL    (1)
#define CHASSIS_REV_RR    (-1)
```

**判定"某个轮子反了"的可靠办法**：架空，推前进杆，手轻贴轮子**最高点** ——
手被朝**车头**推就是对的，朝**车尾**推就是反的。

> ⚠️ 不要用"顺时针/逆时针"判断 —— 那取决于你站在车的哪一侧，很容易看错。

## 遥控器映射

DBUS 通道：`0=右手水平 1=右手垂直 2=左手水平 3=左手垂直`

| 摇杆 | 通道 | 功能 |
|---|---|---|
| 左摇杆 **前后** | `ch3` | 前进 / 后退 |
| 左摇杆 **左右** | `ch2` | 左 / 右平移 |
| 右摇杆 **左右** | `ch0` | **原地自转**（比例）|
| 右摇杆 **上下** | `ch1` | **云台 yaw 角度**（可在线改 `gimbal_ch`）|
| 左拨杆 `sw1` | — | 切模式 |

通道号在 `chassis_config.h` 里，想换随时改：

```c
#define CHASSIS_RC_VX_CH    (3)     /* 前后 */
#define CHASSIS_RC_VY_CH    (2)     /* 左右 */
#define CHASSIS_RC_W_CH     (0)     /* 自转 */
#define CHASSIS_SPIN_DIR_CH (2)     /* 小陀螺模式的转向 */
```

> ⚠️ **航向源必须是"云台锁住世界方向"时才成立** ——
> 现在云台环用的是"云台相对底盘角"，云台会跟着底盘转，所以 θ 恒为 0，**小陀螺实际是画圈、跟随会原地自转**。
> 原因和修法见 [`docs/小陀螺与跟随_控制原理.md`](docs/小陀螺与跟随_控制原理.md) 第六节。

### 三种模式（`sw1` 切换）

| sw1 | 模式 | 自转 ω | 行进方向 |
|---|---|---|---|
| 上（1） | **跟随** | 把云台夹角控到 0（底盘跟着云台转） | 航向补偿 |
| 中（3） | **手动** | `ch0` 比例给定 | 航向补偿 |
| 下（2） | **小陀螺** | 恒定 `spin_w`，方向由 `ch2` 定 | 航向补偿 |

自转量由 `Chassis_Spin_Omega()` 统一给出（手动 = 比例，小陀螺 = 恒定），
最后再乘一个 `CHASSIS_OMEGA_DIR` 做整体极性校正。

### 航向补偿 = 小陀螺能走直线的原因

遥控器的 vx/vy 是**云台系**的速度，底盘却一直在自转，所以下发前先转到**车体系**：

```
vx_body = vx·cos(θ) − vy·sin(θ)
vy_body = vx·sin(θ) + vy·cos(θ)
```

`θ` = `Gimbal_Get_Yaw()` = **云台相对底盘的角度**。

**⇒ 自转时 θ 每一圈都在变，车体系速度跟着反向旋转，底盘就沿直线走而不是画圈。**

`main.c` 里已经挂好：

```c
static float Chassis_Heading_Get(void) {
    return Gimbal_Get_Yaw(&user_gimbal);
}
/* ... */
ChassisControl_Set_HeadingSource(Chassis_Heading_Get);
```

想换成别的角度源（比如 IMU），只改这个函数即可。

## 云台 yaw（GM6020）

### 为什么用它

GM6020 自带 **13 位绝对编码器**（`rotor_angle` 0~8191 对应 0~360°），
正适合给出**「云台相对底盘」的角度** —— 这是小陀螺走直线和跟随的唯一输入。

| 数据 | 来源 | 用途 |
|---|---|---|
| 云台相对底盘角度 | GM6020 绝对编码器 | 航向补偿 + 跟随 |
| 云台世界姿态 yaw | 板载 MPU6500 | 云台自身控制 / 瞄准 |

### 接线

| 项 | 值 |
|---|---|
| CAN | **CAN1**（和 4 个 M3508 同一条总线） |
| 电机 ID | `GIMBAL_YAW_MOTOR_ID`，默认 **1** |
| 控制帧 | `0x1FE`（ID 1~4）/ `0x2FE`（ID 5~7），电流模式 |
| 反馈帧 | `0x205` ~ `0x20B` |
| 控制模式 | 框架的 `Rotor_angle`（多圈角度，单位度） |

> GM6020 的 `0x205` 是**反馈帧 ID**，不是电机 ID；电机 ID 是 1。
> C620 和 GM6020 是**两套控制帧协议**（量程、ID 都不同），不能合并成一帧。

### 三层文件

| 层 | 文件 | 职责 |
|---|---|---|
| 底层 | `User_Drives/User_Motor/user_dji_motor.c/h` | **框架已有**，直接用 `GM6020` 型号 + `Rotor_angle` 模式 |
| 上层 | `User_Application/gimbal.c/h` | 云台相对底盘角度换算 + yaw 角度环 |

### 对外接口

```c
float Gimbal_Get_Yaw(&user_gimbal);   /* 云台相对底盘 deg，逆时针为正，±180 */
Gimbal_Set_Zero(&user_gimbal);        /* 把当前位置记成云台正前方 */
```

### 上电要核对的 2 个值（都在 `gimbal.h`）

| 宏 | 怎么定 |
|---|---|
| `GIMBAL_YAW_MOTOR_ID` | GM6020 拨码/上位机设的 ID，默认 **1** |
| `GIMBAL_YAW_DIR` | 手推云台松手：**回位收敛 = `+1`**，越跑越远 = `-1` |

> **`GIMBAL_YAW_ZERO_DEG` 已经不用标定了** —— V1.2 起了"上电首次对齐"（见下），
> 上电后第一帧反馈到达时会自动以**当前位置**为零点。那个宏只在第一帧之前有效。

### 上电首次对齐（V1.2）

**绝对编码器只知道"转子在哪"，不知道"云台朝哪个世界方向"。**
如果上电直接下发目标，云台会**猛地冲向编码器零位**（最多 180°）。

所以 `Gimbal_Update` 里加了这段：

```c
if (!gimbal->is_aligned) {
    if (!gimbal->is_online) {
        return;                 /* 还没收到反馈 → 一条指令都不发 */
    }
    Gimbal_Set_Zero(gimbal);    /* 以【当前位置】为零点 */
    gimbal->is_aligned = 1;
}
```

**⇒ 上电云台纹丝不动；云台掉线重连后也会重新对齐。**

### 掉线保护（V1.2）

**判据**：`user_gimbal_yaw_motor.rx_count` 连续 100 ms 不增长（= 丢 100 帧）就判定掉线。

```c
#define GIMBAL_OFFLINE_MS  (100)
uint8_t Gimbal_Is_Online (const GIMBAL_DRIVES* g);   /* 反馈是否正常 */
uint8_t Gimbal_Is_Aligned(const GIMBAL_DRIVES* g);   /* 是否已对齐 */
uint8_t Gimbal_Is_Reliable(const GIMBAL_DRIVES* g);  /* 能否给底盘当航向源 */
```

**`Gimbal_Is_Reliable()` 的判定表：**

| 情况 | 返回值 | 小陀螺 |
|---|---|---|
| **从没连上过**（裸车调试）| **1** | ✅ 允许（θ 恒为 0，不会乱跑）|
| **连上过又掉线** | **0** | ❌ **禁止自转**（θ 是冻结值，会让车跑偏）|
| 在线但未对齐 | 0 | ❌ 禁止 |
| 在线且已对齐 | 1 | ✅ 允许 |

**⇒ 云台掉线时：禁止自转 + 不做航向补偿，但前后左右平移仍可用。**

**⚠️ 想看舵机反馈通不通**：在调试器里看 `user_gimbal_yaw_motor.rx_count` 涨不涨。

### ★ 云台锁世界方向（V1.3）

**这是「小陀螺走直线」的关键前提。**

```
问题：如果云台"相对底盘固定"，那么底盘一自转，云台就跟着一起转
        -> 云台相对底盘角 theta 恒定不变
        -> 航向补偿 R(theta) 退化成恒等变换
        -> 小陀螺实际是【画圈】而不是走直线 ✗

做法：让云台【锁住世界方向】（钉在世界上不动）
        -> 底盘自转时 theta 才会真实变化
        -> 航向补偿才有意义 ✓
```

**实现 = 位置环 + 底盘角速度前馈，反馈用板载 IMU 的 yaw：**

```c
/* gimbal.c —— 1kHz */
imu_yaw = Attitude_Get_Yaw(&user_attitude);      /* A 板在云台 yaw 上，测的就是云台世界朝向 */

/* 摇杆 -> 世界目标角速度 -> 积分成世界目标角
   松杆时 target_yaw 不变 => 云台锁在那个世界方向 */
target_yaw += stick * max_w * DT * RAD2DEG * GIMBAL_YAW_DIR;

/* 位置环 + 前馈 */
error = Wrap180(target_yaw - imu_yaw);
rpm   = GIMBAL_YAW_DIR * (world_kp * error
                          - chassis->omega * RAD_S_TO_RPM);
                          ↑ 前馈：抵消底盘自转
```

| 项 | 作用 |
|---|---|
| **位置环** | 让云台世界朝向追上 `target_yaw` |
| **前馈** | ⭐ 底盘以 `omega` 自转时，云台必须相对底盘**反向转同样角速度**才能在世界系不动 |
| **`GIMBAL_LOCK_WORLD`** | `1`（默认）；改 `0` 退回纯速度控制（IMU 有问题时用）|

**⚠️ 局限：IMU 的 yaw 没有磁力计做绝对参考，会缓慢漂移。**
短时间（几秒~几十秒）足够用；长时间小陀螺后需要重新对齐。

**在线可调：** `gimbal_world_kp`（位置环增益）、`gimbal_max_w`（满杆角速度）。

### 航向补偿按模式切换（V1.3）

```c
/* chassis_control.c */
const uint8_t use_heading = (src_ok && (s_mode != CHASSIS_MODE_MANUAL)) ? 1 : 0;
const float theta_rad = (use_heading ? Heading() : 0.0f) * DEG2RAD;
```

| 模式 | 正方向 | 实现 |
|---|---|---|
| **手动** | **车头** | `theta = 0` ⟹ `R(0)` 是恒等变换，等于不补偿 |
| **跟随 / 小陀螺** | **云台指向** | 用航向源 `theta` |

**⇒ 好处：手动模式推前进就是车头方向，不会因为云台转了而改变；**
**跟随/小陀螺则以云台为参考，才能边走边瞄。**

### 遥控器通道（V1.2 已开启）

```c
#define GIMBAL_RC_ENABLE  (1)        /* 开启遥控 */
#define GIMBAL_RC_CH      (1)        /* 0=右手水平 1=右手垂直 2=左手水平 3=左手垂直 */
#define GIMBAL_RC_RANGE   (180.0f)   /* 满杆 = ±180° */
```

**⇒ 右摇杆【上下】控制云台 yaw。**

**⚠️ V1.3 起改成「锁世界方向」**（见下面「云台锁世界」一节）：
推杆云台在**世界系**里匀速转，松杆**锁在当前世界方向**，被外力推动会自动回位。
**通道号可以在线改**：`gimbal_ch=0#` / `1#` / `2#` / `3#`。

## 板载 IMU（MPU6500，装在云台上）

### 接线

| 功能 | 引脚 |
|---|---|
| SPI5 | `PF7` SCK / `PF8` MISO / `PF9` MOSI（5.625 Mbit/s，Mode 3） |
| 片选 CS | `PF6`（普通 GPIO，低有效） |
| MPU6500 中断 | `PB8`（未用，轮询读取） |
| 磁力计 Set/Reset / INT | `PE2` / `PE3`（未用） |
| 串口绘图 | `USART2`（`PD5` / `PD6`，115200） |

### 三层文件

| 层 | 文件 | 职责 |
|---|---|---|
| 底层 | `User_Drives/User_Peripheral/user_spi.c/h` | SPI 收发 + 片选 |
| 底层 | `User_Drives/User_Sensor/user_mpu6500.c/h` | 寄存器、量程换算、**零偏标定** |
| 中层 | `User_Algorithm/User_Attitude/attitude.c/h` | 互补滤波，输出 `pitch` / `yaw` |

### 对外接口

```c
float Attitude_Get_Yaw(&user_attitude);      /* deg，逆时针为正 */
float Attitude_Get_Pitch(&user_attitude);    /* deg，抬头为正 */
Attitude_Reset_Yaw(&user_attitude);          /* 把当前 yaw 置 0 */
MPU6500_Calibrate_Start(&user_mpu6500);      /* 重新标定零偏 */
```

### 零偏标定与漂移测试

1. 上电后**保持静止约 1 秒**（1000 次采样）自动完成陀螺零偏标定
2. **测漂移**：静止 1 分钟，看 `yaw` 漂了多少度 —— 这就是你这块板子的真实零漂
3. `yaw` 是**陀螺积分**，没有绝对参考（磁力计 IST8310 未接），**必然缓慢漂移**

> 大俯仰角下 yaw 用 `gz·cos(pitch) − gx·sin(pitch)` 投影回世界垂直轴。

### 上电必须核对的 2 件事

| 项 | 怎么核对 | 不对就改 |
|---|---|---|
| **轴向** | 云台放平，`pitch` 应 ≈ 0；抬头时 `pitch` 增大 | `attitude.h` 的 `ATTITUDE_SIGN_X/Y/Z` |
| **偏航正方向** | 手动**逆时针**转云台，`yaw` 应增大 | `ATTITUDE_SIGN_Z` 符号 |

## 串口绘图 / 在线调参

### 协议

| 方向 | 格式 |
|---|---|
| 板子 → 上位机 | `0xAB` + N×`float32`(小端) + 1 字节累加和 |
| 上位机 → 板子 | `变量名=值#`，例如 `wheel_kp=18.5#` |

**通道由 `CHASSIS_SERIALPLOT_MODE` 决定（三选一）：**

| 值 | 通道 | 用途 |
|---|---|---|
| **2** ⭐ 当前 | `gyaw / gtgt / graw / gspd` | **云台 yaw 调试**（反馈角 / 目标角 / 电机多圈角 / 转速）|
| 1 | 8 通道：4 轮的 目标/反馈 转子转速 | 底盘轮速调试 |
| 0 | `yaw / pitch / omega / vx` | 姿态 |

> ⭐ **看 `graw` 就能判断云台通不通**：恒为 0 = 反馈没收到。
> **看 `gtgt` 就能判断遥控通道对不对**：推杆会变 = 对。

### 上位机

**推荐用本仓库自带的 Python 工具**（已按上面的协议写好，自动处理帧同步）：

```powershell
python tools\serialplot.py            # 自动列出串口，选一个
python tools\serialplot.py COM4       # 直接指定
```

需要 `pyserial` 和 `matplotlib`。窗口里能看实时曲线，底部有**帧率**和**校验错计数**，
还有一个输入框可以直接敲 `变量名=值` 回车发送。

**SerialPlot（hyOzd）也能用**，参数照这个填：

| 项 | 值 |
|---|---|
| Baud rate | 115200 |
| **Number of channels** | **和 `CHASSIS_SERIALPLOT_MODE` 对应**（当前 = **4**）|
| **Data format** | **float**（4 字节）|
| **Endianness** | **Little endian** |
| **Frame start** | `AB`（十六进制）|
| **Checksum** | **Simple sum** |

### 可在线调的 17 个变量

| 变量名 | 含义 | 范围 |
|---|---|---|
| **`test_wheel`** | 单轮测试：`1/2/3/4` 只驱动对应轮子，负值反转，**`0`=退出** | −4 ~ 4 |
| `test_rpm` | 单轮测试转速 | 0 ~ 3000 |
| `spin_w` | 小陀螺自转角速度 | 0 ~ 20 |
| `omega_dir` | 自转方向校正 | −1 ~ 1 |
| `dead` | 摇杆死区 | 0 ~ 100 |
| `head_kp` / `head_ki` / `head_kd` | 航向 PID（跟随模式）| — |
| `wheel_kp` / `wheel_ki` / `wheel_kd` | 4 个轮速 PID（**一起改**）| — |
| `rev_fl` / `rev_fr` / `rev_rl` / `rev_rr` | 4 个轮子方向 | −1 ~ 1 |
| **`src_protect`** | **航向源保护开关**：`0` = 关掉（云台没接也能用小陀螺）| 0 ~ 1 |
| **`gimbal_ch`** | **云台遥控通道**：`0`/`1`/`2`/`3` | 0 ~ 3 |

**在线改动立刻生效，断电就丢。** 调满意了再写回 `chassis_config.h` 固化。

变量表在 `chassis_control.c` 的 `chassis_tunable[]`，加一行就能多调一个参数。

上报周期 `CHASSIS_SERIALPLOT_PERIOD_MS`（默认 10ms = 100Hz）。
115200 下 4 通道每帧 18 字节、8 通道 34 字节，理论上限都在 300Hz 以上。

## 上车标定流程

**按顺序做，每步确认了再下一步。**

### 1. 单轮测试 —— 确认「哪个 ID 在哪个角」+「电机是否在线」

架空底盘，遥控器开机，串口连上位机，依次发：

```
test_wheel=1#      → 只有【左前】轮转吗？
test_wheel=2#      → 只有【右前】轮转吗？
test_wheel=3#      → 只有【左后】轮转吗？
test_wheel=4#      → 只有【右后】轮转吗？
test_wheel=0#      → 退出
```

每条记录：**哪个角在转 / 是不是只有它 / 转向如何**，
同时看串口曲线里那个轮子的「反馈」跟不跟得上「目标」（恒为 0 = 掉线）。

**对不上就改 `main.c` 里 4 个 `DJI_Motor_Init` 的 ID 数字**。

### 2. 轮子方向

架空，推前进杆，**手轻贴每个轮子最高点**：朝车头 = 对，朝车尾 = 反。

不用重烧，直接在线翻：

```
rev_fl=-1#     rev_fr=-1#     rev_rl=-1#     rev_rr=-1#
```

**判据：4 个轮子同向 + 车真的往前走。** 定好后写回 `CHASSIS_REV_*`。

### 3. 三个动作验收

| 摇杆 | 期望 |
|---|---|
| 左摇杆前后 | 4 轮同向，车前进 / 后退 |
| 左摇杆左右 | 对角图案，车左右平移 |
| 右摇杆左右 | 左两个 vs 右两个反向，车**原地自转** |

**如果「平移变成自转、自转变成打滑」→ 电机 ID 绑定错了，回第 1 步。**

### 4. 参数

| 项 | 位置 | 起始值 |
|---|---|---|
| 轮径 / 轴距 / 轮距 | `CHASSIS_WHEEL_R` / `WHEELBASE` / `TRACK` | 量实际值 |
| 轮速 PID | `CHASSIS_SPEED_KP / KI / KD` | 15 / 0.15 / 0 |
| 航向 PID | `CHASSIS_FOLLOW_*` | 30 / 0 / 0 |
| 云台零点 / 方向 | `GIMBAL_YAW_ZERO_DEG` / `_DIR` | 见上文 |
| IMU 轴向 | `ATTITUDE_SIGN_*` | 见上文 |

**小提示**：轻微拨杆时电机抖动，是「力矩打不过静摩擦 + 积分蓄力」的固有现象。
在线加大死区 / 降积分就能缓解：

```
dead=30#     wheel_kp=25#     wheel_ki=0.05#
```

### 5. 云台 yaw（锁世界方向，V1.3）

```
① 上电 → 云台应该【纹丝不动】
② 推右摇杆上下
     ├─ 云台匀速转，松杆【停住】   → ✅ 正常
     ├─ 不转         → 试 gimbal_ch=0# / 2# / 3#
     ├─ 转【反】了   → 改 gimbal.h 的 GIMBAL_YAW_DIR = -1
     └─ 松杆后缓慢自转 / 抖 → 减小 gimbal_world_kp
③ ⭐ 关键验证：松杆后用手轻推云台再松手
     ├─ 云台【自动回到原位】 → ✅ 锁世界生效
     └─ 推走就不回来了       → IMU 没数据，或 world_kp 太小
④ 开小陀螺，看串口 gtgt 和 gimu 两条线
     ├─ 两条线重合   → ✅ 云台锁住了世界方向
     └─ 分开发散     → 调大 gimbal_world_kp
⑤ 拔掉云台 CAN 线，等 1 秒
     ├─ 小陀螺推不动      → 掉线保护生效 ✅
     └─ 平移仍可用        → ✅ 保护行为正确
⑥ 插回 CAN 线 → 自转自动恢复，云台重新对齐
```

> ⚠️ **第一次测云台前，先把 `GIMBAL_YAW_MAX_OUT` 从 3000 调到 300**，
> 万一 `GIMBAL_YAW_DIR` 反了也不会暴力。确认收敛后调回 3000。

## 底盘运动学与上层 API

麦轮（X 型）和四轮全向轮的**逆解 + 正解**都已实现，改一行切换：

```c
#define CHASSIS_TYPE      CHASSIS_OMNI4   /* chassis_config.h */
```

```c
Chassis_Set_Velocity(&user_chassis, vx, vy, omega);    /* m/s, m/s, rad/s */
Chassis_Update(&user_chassis);                         /* 逆解 + 下发电机 */

float vx, vy, omega;
Chassis_Get_Velocity(&user_chassis, &vx, &vy, &omega); /* 正解，里程计 */
```

坐标系：**x 向前，y 向左，omega 逆时针为正**。

**安全兜底**：遥控器断线 >100ms 自动停车；
`vx/vy` 限 **±2 m/s**、`ω` 限 **±3 rad/s**、单轮功率上限 **200 W**、摇杆死区 ±10。
全在 `chassis_config.h`。

> **功率上限别调太小**：72W 时每轮只有 3A，起步会堵转（表现为「只有一个轮子转」）。
> 200W ≈ 8.3A 是实测可用的值，上限建议不超过 300W。

## ⚠️ 已知问题 / 待办

### 1. ✅ 已修（V1.3）：云台环用「相对底盘角度」做反馈

**原症状：** 小陀螺画圈（ω=4、v=1 时半径仅 0.25 m）；跟随变原地自转。
**原因：** 云台跟着底盘转 → θ ≡ 0 → 航向补偿退化成恒等变换。
**修法：** 云台改成**锁世界方向**（IMU 做反馈 + 底盘角速度前馈），见上面「云台锁世界方向」一节。
**原理推导：** [`docs/小陀螺与跟随_控制原理.md`](docs/小陀螺与跟随_控制原理.md)。

### 2. ✅ 已修（V1.3）：跟随环 `Kp` 参数严重失配

```c
#define CHASSIS_FOLLOW_KP  (30.0f)     /* 输出 = 30 × θ */
#define CHASSIS_MAX_W      (3.0f)      /* 输出被夹到 ±3 rad/s */
```

```
θ > 3/30 = 0.1°  →  输出就饱和到最大转速
        ⇒ 全程是【开关控制】，必然超调振荡
```

**已把默认值从 `30` 改成 `0.05`。** 在线可调：`head_kp=0.05#`

### 3. 其他

- **`tools/serialplot.py` 只在 Python 下可用**；串口助手只能调参、看不了曲线。
- **加热（PB5）未启用** —— IMU 零偏会随温度漂，长时间云台 yaw 会偏。
- **磁力计 IST8310 未接**（MCU 够不到 MPU6500 的 AUX I²C），yaw 无绝对参考。

## 常用操作

| 想干什么 | 怎么做 | 输出看哪个面板 |
|---|---|---|
| **编译 + 烧录** | `Ctrl+Shift+B` | 终端 → `[OK] 烧录完成` |
| **编译 + 烧录 + 调试** | `F5` | 调试控制台 → 停在 `main` |
| 只编译 | `Ctrl+Shift+P` → 任务 → `编译` | 问题面板 |
| 检查 ST-Link | 任务 → `检查 ST-Link 连接` | 终端 → `Voltage: 3.xxV` |
| **串口调参** | `python tools\serialplot.py` | 曲线 + 底部状态栏 |
| 清理重编 | 任务 → `清理` | — |

**烧录成功的标志是 `Download verified successfully`**（脚本带 `-v` 校验）。

## CubeMX 重新生成代码之后

CubeMX 会用模板**覆盖 `CMakeLists.txt`**，所以生成完必须跑一次：

```
Ctrl+Shift+P → 任务: 运行任务 → CubeMX 生成后修补
```

它会还原 `CMakeLists.txt`，并检查 6 个 USER CODE 钩子有没有被删掉。
**输出 `全部检查通过` 才算成功**，之后才能编译。

## 依赖说明

- 姿态解算用 `sqrtf` / `atan2f` / `cosf` / `sinf`，`CMakeLists.txt` 里已加 `target_link_libraries(... m)`。
- 加热（PB5）**没有启用**，CubeMX 里也没配。
- 磁力计 IST8310 **未接入**，所以 MPU6500 的 yaw 会漂；云台相对底盘角度靠 GM6020 编码器，不漂。
- 新增了 `.c` 文件后必须**重跑一次 CMake configure**（`GLOB_RECURSE` 只在 configure 时展开）。
- 「没接线」的库还有很多（ADRC、LADRC、FIR、牛顿迭代、STP23、ADC、RNG 等），
  都编译好了但没人调用，被 `--gc-sections` 裁掉，所以**不占 Flash**。
- `docs/小陀螺与跟随_控制原理.md` —— 小陀螺走直线 / 跟随的完整原理推导。
- `tools/serialplot.py` 需要 `pyserial` + `matplotlib`：
  `python -m pip install pyserial matplotlib -i https://mirrors.aliyun.com/pypi/simple/`

---

## 致谢

本框架的**三层架构设计、底层驱动库的组织方式，以及串口绘图调参的思路**，
参考了 **昆明理工大学「昆蓬南冥」战队** 开源的 RoboMaster A 板工程。

在本项目复刻、精简与扩展的过程中，那份开源代码提供了关键的方向与参考。

**感谢昆蓬南冥战队的开源精神。**
