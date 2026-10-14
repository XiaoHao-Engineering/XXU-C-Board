# XXU-RoboMaster-Sentry（C 板）

**版本 V2.0** ｜ 新乡学院 RoboMaster 哨兵控制工程 —— **RoboMaster 开发板 C 型（STM32F407IGH6）**。

> 分层结构、文件路径、命名与注释风格与 A 板原工程一致；移植过程与 CubeMX 改动见
> [docs/C板移植说明.md](docs/C板移植说明.md)。
> **上机调试请直接看 [docs/上机调试流程.md](docs/上机调试流程.md)**（硬件核对 → 编译烧录 →
> 上电自检 → 分模块验证 → 参数整定 → 故障速查，每步都有通过判据和排错表）。

---

## 一、四层结构

```
Core/                板级层（CubeMX 生成 + bsp）
User_Drives/         底层驱动：搬运工，不认识业务
  ├─ User_Peripheral/  uart can spi pwm timer adc rng serialplot
  ├─ User_Basic/       led buzzer
  ├─ User_Sensor/      hwt906  bmi088（板载 IMU）  stp23  mpu6500(备查)
  ├─ User_Motor/       dji_motor
  └─ user_dji_bus       DBUS 遥控器
User_Architect/      基础组件：缓冲区 / 编解码 / 延时 / 滴答任务
User_Algorithm/      中层算法：只吃数字，不碰硬件
  ├─ User_Controller/  pid  pid_adv  inc_pid  adrc  ladrc  cascade  parallel  pid_tolerance
  ├─ User_Attitude/    attitude（互补滤波姿态解算）
  ├─ User_Chassis/     chassis_kinematics
  └─ user_coord  user_cf  user_fir  user_newton  user_ballistics  user_waveform
User_Application/    上层业务
  ├─ chassis  chassis_control  chassis_config
  ├─ gimbal（yaw + pitch 两个轴）
  ├─ shooter  shooter_config（2 摩擦轮 + 1 拨弹轮）
  └─ user_startup_music
User_Lib/            曲库      SEGGER_RTT/  RTT + JScope
Drivers/ Middlewares/  ST 官方 HAL / CMSIS
docs/                 架构分析 / 控制原理 / 移植说明 / 上机流程
```

```
遥控器/上位机 → User_Application（底盘策略、云台双轴、射击状态机、自动瞄准）
              → User_Algorithm（运动学、PID/串级、姿态解算、弹道解算、FIR）
              → User_Drives（CAN 帧、UART 队列、SPI、PWM）
              → HAL / Core（中断、DMA、定时器）
```

---

## 二、外设与 CAN 分配

| 功能 | 外设 / 引脚 |
|---|---|
| CAN1 (PD0/PD1, 1Mbps) | 底盘 4× M3508（ID 1~4）+ 云台 yaw GM6020（ID 1）|
| CAN2 (PB5/PB6, 1Mbps) | 云台 **pitch** GM6020（ID 4）+ 摩擦轮左 M3508（ID 1）+ 摩擦轮右 M3508（ID 2）+ 拨弹轮 M2006（ID 3）|
| DBUS 遥控器 | **USART3**：PC10(TX)/PC11(RX)，100000，**9 位数据 + 偶校验** |
| HWT906 | **USART6**：PG14(TX)/PG9(RX)，115200 |
| 串口绘图 / 在线调参 | **USART1**：PA9(TX)/PB7(RX)，115200 |
| 板载 BMI088 | **SPI1**：PB3(SCK)/PB4(MISO)/PA7(MOSI)，Mode 3；CS = PA4（加速度计）、PB0（陀螺仪）|
| 蜂鸣器 | **TIM4_CH3 = PD14** |
| RGB 状态灯 | 红 PH12，绿 PH11，蓝 PH10（低电平点亮）|
| JScope 节拍 | **TIM10**，10kHz |
| IST8310 磁力计 / IMU 恒温 | CubeMX 已配好，**未启用** |

### 为什么 pitch 挂 CAN2

按 8 字节标准帧约 130 位、反馈 1kHz、发送 500Hz 估算：

| 方案 | CAN1 | CAN2 |
|---|---|---|
| 只有底盘 + yaw | 约 **78%** | 0% |
| pitch 挂 CAN1 | 约 **91%** ❌ 超出 CAN 可靠区间 | — |
| **pitch 挂 CAN2（当前）** | 约 **78%** | 约 **78%** |

> CAN1 本身 78% 就偏高（一般建议 ≤70~80%）。以后若 CAN 出问题，
> 首先考虑把底盘 1~2 个轮子挪到 CAN2 平衡负载。
>
> 发送周期 `MOTOR_EXECUTE_PERIOD = 2`(ms)，所以电流指令 2ms 更新一次；
> 想更快就改成 1（代价是每条总线多 500 帧/s）。

---

## 三、功能模块

### 云台

| 轴 | 电机 | 控制方式 |
|---|---|---|
| yaw | GM6020 (ID1) @CAN1 | `Rotor_speed`；**锁世界外环**写在 `gimbal.c`（位置环 + 陀螺阻尼 + 角速度前馈）|
| pitch | GM6020 (ID4) @CAN2 | `Rotor_angle` + 注入 `Cascade_Controller`（外环角度 → 内环速度 → 电流）|

- `GIMBAL_PITCH_CASCADE = 0` 可退回单环（电机只跑速度环，角度环在 `gimbal.c` 里算）。
- **软限位** `GIMBAL_PITCH_MIN_DEG / MAX_DEG`（默认 -30° ~ +15°）。
- **pitch 遥控用左上角拨轮**（DBUS `roll`）；yaw 用右摇杆垂直。

#### yaw 抗惯性（V2.0 重点）

外环是"角度误差 → 转速目标"，1 RPM = 6 deg/s，所以开环时间常数 **τ ≈ 6/world_kp 秒**。
V1 用的 `world_kp = 1.0` → **τ ≈ 6 秒**，这就是"云台偏软、走起来被惯性拖着"的根因。V2.0 三件套：

| 项 | 宏 | V1 → V2.0 | 说明 |
|---|---|---|---|
| 位置环增益 | `GIMBAL_WORLD_KP` | 1.0 → **3.0** | τ 6s → 2s；稳了可再试 5.0 |
| **陀螺阻尼** | `GIMBAL_WORLD_KD` | 新增 **0.17** | 用 IMU 的 yaw 角速度做阻尼。量纲 RPM/(deg/s)，纯运动学值 = 1/6≈0.167；偏软加到 0.33，一抖就降 |
| 阻尼源低通 | `GIMBAL_WORLD_D_ALPHA` | 新增 0.25 | 陀螺噪声会直接进转速指令，越小滤得越狠、相位滞后越大 |
| 底盘角速度前馈 | `GIMBAL_FF_GAIN` | 0.3 → **1.0** | 运动学上精确抵消底盘自转就是 1.0；0.3 只补 30%，剩 70% 让位置环追 → 滞后+摆动 |
| 外环输出限幅 | `GIMBAL_WORLD_MAX_RPM` | 新增 60 | 原来**完全没有限幅**，跨 ±180° 时会一脚踹满速度环 |
| **外环死区** | `GIMBAL_WORLD_DEAD_DEG` | 新增 **0.3°** | ★ **治"停着不动也慢慢抖"**：IMU 的 yaw 是纯陀螺积分、**没有绝对参考**，静止时自己会缓慢漂移；外环不设死区就会一直追这个漂移 → 慢慢抖/慢慢转。死区内只清 P 项，阻尼与前馈仍工作（pitch 外环早就有同样的死区）|

> ⚠️⚠️ **`GIMBAL_YAW_KI` 必须保持 0** —— 这是踩过坑的：
> 这个速度环用的是老 `user_pid.c`，`Iout += ki*err` 里**没有 dt**，而它被 1kHz 的 CAN 反馈调用，
> 等效积分增益 = `ki × 1000`。再叠加 GM6020 的 `rotor_speed` 是**整数**（静止时读数在 0/±1 之间跳），
> `Ki > 0` 会立刻形成**静止极限环** —— 表现就是"刚上电云台就慢慢抖"。
> **要刚度请用外环的 `world_kp` + `world_kd`，不要动速度环的 Ki。**

**整定顺序：先加阻尼（`gimbal_world_kd`）再提增益（`gimbal_world_kp`），反过来必振荡。**
两个都能在线调。相关实现：`attitude.c` 现在把世界系 yaw 角速度 `yaw_rate` 暴露出来
（V1 里算完就丢了），`gimbal.c` 对它有低通后进阻尼项。

> ⚠️ **`GIMBAL_IMU_ON_GIMBAL` 必须按实车填对**：
> `1` = IMU 装在云台上（`imu_yaw` 就是云台世界朝向，直接用）；
> `0` = IMU 装在底盘上（云台世界朝向 = 底盘 yaw + 云台相对底盘的编码器角）。
> **填错会让锁世界锁错对象（云台跟着底盘转）。**
> 10 秒判定法：① 固定底盘、手动转云台，看串口 `gimu`/`yaw` 变不变；② 固定云台、手动转底盘，再看。哪个有反应，IMU 就在哪。

> ⚠️ **pitch 上电姿态决定零点**：收到第一帧后会把"当前位置"标定成
> `GIMBAL_PITCH_ZERO_BIAS_DEG`。所以**每次上电前把枪管摆到同一姿态**
> （建议水平，此时该宏填 0），否则角度和软限位都会漂。
>
> ⚠️ **顶重力的是内环积分**：稳态时外环误差为 0、内环目标是 0 RPM，
> 要让电机输出恒定力矩抵消重力，只能靠内环积分维持。
> 所以 `pitch_spd_ki` 不能为 0；**松手后枪管慢慢下沉 = 该值或积分限幅太小**。
>
> ⚠️ **零位积分泄放（`pitch_spd_zb`）必须保持 0**，开了会把顶重力的积分放掉。

pitch 两个环用的是 `User_Algorithm/User_Controller/user_pid_adv.c`（增强 PID）：

- 显式 `dt`（所以 `ki` 单位是 输出/(误差·s)、`kd` 是 输出/(误差/s)，数值比 `user_pid` 大是正常的）
- 首拍保护（使能第一拍不产生微分/前馈冲击）· 微分先行 · D 项一阶低通
- 三段独立限幅（积分/微分/总输出）· 双域抗积分饱和
- 变速积分 · 积分分离（**用衰减而不是硬清零**，否则大阶跃会瞬间丢掉顶重力的积分）
- 死区 · 目标速度前馈 · 方向枚举 · 零位泄放（pitch 禁用）
- 全套 setter + P/I/D/F 分项可观测，方便定位是哪一项在闹

> 没有直接改 `user_pid.c` 加 `dt`：那会改变 `ki/kd` 的量纲，
> 底盘 4 个轮速环和 yaw 已调好的参数会全部失配。所以单独做一份只给 pitch 用。

### 射击机构（2 摩擦轮 + 1 拨弹轮）

| 部件 | 电机 | 电调 | CAN2 ID | 反馈帧 | 控制模式 |
|---|---|---|---|---|---|
| 摩擦轮左 | M3508 | C620 | 1 | 0x201 | `Rotor_speed` |
| 摩擦轮右 | M3508 | C620 | 2 | 0x202 | `Rotor_speed` |
| 拨弹轮 | M2006 | C610 | 3 | 0x203 | `Rotor_angle` + 串级 |

三个电机发送共用 `0x200` 帧，各占 2 字节，不冲突。

- **上层状态机**：`STOP → SPINUP → FIRE ⇄ REVERSE`
- **遥控映射（sw2）**：上(1)=单发、中(3)=停火、下(2)=连发；扳机就是 sw2 档位本身
- 单发 = 拨到单发档那一下算一次触发；连发 = 停在连发档期间持续触发
- **摩擦轮就绪才允许拨弹**，否则会把弹丸卡在两个摩擦轮之间
- **卡弹判据** = 位置没跟上 **且** 电流大 **且** 同时持续一段时间 → 自动反转退弹
  （只看电流会被拨盘的机械冲击误触发）
- **拨弹轮**位置式一步一发；对外参数用**拨盘角**，内部按 36:1 换算成转子角
- **FIR 滤波**（`user_fir.c` → CMSIS-DSP）用于拨弹轮电流（卡弹判定）与摩擦轮转速（弹速估计）

### 弹道解算

`User_Algorithm/user_ballistics.c`，用 `user_newton.c` 的牛顿迭代解俯仰角：

```
f(θ) = d·tanθ − g·d²/(2·v0²·cos²θ) − Δh = 0
```

- 先用判别式 `disc = d² − 4A(A+Δh)`（`A = g·d²/2v0²`）**预判可解性**：
  `disc < 0` 表示该初速打不到该距离，立即返回几何角兜底，**一次迭代都不做**。
- 初值取低伸弹道的解析根，牛顿几步收敛；迭代次数上限夹在 `BALLISTICS_MAX_ITERS`。
- 解算跑在**独立的 20ms 任务**里，**绝对不能放回 1kHz 控制任务**（见第七节）。
- 弹速模型 `v0 = v0_b + v0_k × 摩擦轮转速(RPM)`，**必须按自己的弹丸实测标定**。
- 距离/高度目前用在线调参给（`dist` / `height`）；接视觉后调
  `Shooter_Set_Target_Distance()` 即可。

---

## 四、编译 / 烧录

```powershell
.\tools\setup_env.ps1     # 把工具链加进用户 PATH，只需跑一次
```

VS Code 里用任务（`Ctrl+Shift+B` 是「编译并烧录」）：

| 任务 | 作用 |
|---|---|
| 编译并烧录 | 默认任务 |
| 编译 | 只编译（缺配置会自动配置一次）|
| 烧录 / 检查 ST-Link 连接 | — |
| 重新配置 CMake | 切换 Debug / Release / RelWithDebInfo / MinSizeRel |
| CubeMX 生成后修补 | CubeMX 重新生成代码后必跑 |

```powershell
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel
.\tools\flash.ps1 -Build
```

> ⚠️ **上机飞请用 Release 或 MinSizeRel**（Debug 是 `-Og -g`，1kHz 中断里跑控制逻辑差距明显）。
>
> ⚠️ CubeMX 重新生成代码后一定要跑 `.\tools\cubemx_postgen.ps1`（还原被模板覆盖的
> `CMakeLists.txt`、检查 USER CODE 钩子）。

**编译速度**（实测 89 个编译单元、串行约 40 秒）：

- `Drivers/CMSIS/DSP/Source` 整树有 **564** 个 `.c`，而本工程只用到 5 个函数 →
  只编译 **7** 个文件（644 → 89 个编译单元）。新增 DSP 函数的做法见 `CMakeLists.txt` 注释。
- 并行度用 `--parallel`（自动按本机核心数），不是写死的 `-j`。
- `CONFIGURE_DEPENDS` 只加在**你**会增删文件的目录上；驱动树不加，省掉每次构建的目录扫描。

---

## 五、串口绘图 / 在线调参

**主用 SerialPlot（hyOzd 的 Qt 上位机）**，帧格式与 A 板原工程一致、无需重配：

| 方向 | 格式 |
|---|---|
| 板子 → 上位机 | `0xAB` + N×`float32`(小端) + 1 字节累加和 |
| 上位机 → 板子 | `变量名=值#`（首尾空白会被自动忽略）|

帧尾校验和由 `user_serialplot.h` 的 `SERIALPLOT_CHECKSUM_ENABLE` 控制（默认 1）。
自带的 `tools/serialplot.py` 会自动识别带不带校验和，命令行第 2 个参数是绘图模式号。

### 绘图模式（可在线切换）

上电默认由 `chassis_config.h` 的 `CHASSIS_SERIALPLOT_MODE` 决定，
运行中用 `plot_mode=N#` 在线切换（**换模式后 SerialPlot 里的通道数要同步改并重开串口**）：

| 模式 | 通道数 | 通道 |
|---|---|---|
| 0 | 4 | `yaw pitch omega vx` |
| 1 | 8 | 4 轮的目标/反馈转子转速 |
| **2**（默认）| 5 | `gyaw gtgt gimu gcmd gspd`（云台 yaw）|
| **3** | 6 | `pitch ptgt pspd pid_i psd_i pcur`（云台 pitch）|
| **4** | 12 | `frpm frpm_f frpm_t ferr fcur bpitch bv0 fstate online ready can1_rx can2_rx`（射击 + CAN 排错）|

> ★ 模式 3 里的 `psd_i` 就是**顶重力的内环积分**；模式 4 的 `fstate` 是状态机
> （0=STOP 1=SPINUP 2=FIRE 3=REVERSE）。
>
> ★ **模式 4 排查"电机掉线"**：`online` 是三个电机的反馈状态打包（bit0=摩擦轮左
> bit1=摩擦轮右 bit2=拨弹轮，7=都在线），`ready` 是摩擦轮是否到转速，
> `can1_rx` / `can2_rx` 是两条总线的**原始收帧数**（不看 ID）。
> **判断表：**
>
> | 现象 | 结论 |
> |---|---|
> | `can2_rx` 约 **+1000/s 且不涨更快** | CAN2 上**只有 pitch 在发** → 那 3 个电调根本没在发报文：查接线 / 24V 供电 / **ESC ID（左1、右2、拨弹3）** |
> | `can2_rx` 约 **+4000/s** 但 `online`=0 | 报文在发但 ID 对不上 → **ESC ID 配错了** |
> | `online`=7 但 `frpm`=0 | 有反馈、在发指令但电机不转 → 查电调是否处于使能状态 / 电机相线 |
> | `fstate` 恒为 0（不随 sw2 变）| 遥控 sw2 档位没被识别（上=1 单发 / 中=3 停火 / 下=2 连发）|
>
> ★ **调参表不受绘图模式影响** —— 任何模式下都能发 `pitch_spd_ki=300#` 改参数。

常用调参示例：

```
plot_mode=3#                                 切换绘图模式
wheel_kp=15#  wheel_ki=0.15#  wheel_kd=0#    底盘轮速 PID
rev_fl=1# rev_fr=-1#                         轮子装反了改 -1
spin_w=4#  omega_dir=1#  dead=10#            小陀螺 / 方向 / 死区
gimbal_world_kd=0.17#  gimbal_world_kp=2#    云台 yaw 抗惯性（先阻尼后增益）
gimbal_ff=1.0#  gimbal_kp=30#  gimbal_ki=0#  ★ki 必须保持 0（见第三节警告）
pitch_spd_ki=300# pitch_ang_kp=12#           pitch 内环/外环
fire_rpm=6000#  step_deg=45#  fire_int=120#  射击
feed_spd_kp=8#  feed_ang_kp=6#               拨弹轮
dist=4.5#  height=0#  auto_aim=1#            弹道解算
v0_k=0.004#  v0_b=4.0#                       弹速模型（必须实测标定）
```

> **完整的可调变量清单**（约 60 个，按模块分组、带默认值，可直接复制到 SerialPlot 的
> Command 面板）见 [docs/上机调试流程.md](docs/上机调试流程.md) 第 3.4 节。

---

## 六、电机底层加固

| 问题 | 处理 |
|---|---|
| **首帧编码器基准误判跨圈**<br>电机对象是全局 `{0}`，第一帧 `last_rotor_angle` 恒为 0；上电若停在编码器 > 4096 处，"跨半圈"判据会误判，`total_angle` 立刻注入 ±180° 恒定偏差且永不自愈 | 加 `encoder_init_flag`：第一帧只建立基准，不做跨圈判断 |
| **掉线后电流"冻结"**（安全问题）<br>反馈一断 `Calculate` 不再执行，`Get_Output()` 冻结在最后的值，每 2ms 把同一个旧电流继续发给电调 → 反馈线断了电机反而持续出力 | 新增 `DJI_Motor_Check_Alive()`（每 2ms）：超时判掉线 → 调控制器 `Reset()` 清积分 → 强制输出 0，**并照样把 0 写进帧发出去**（不发帧电调会保留旧指令）|
| **功率折算对所有电调都用 20A** | 按型号区分：C620 ±20A / C610 ±10A / GM6020 ±3A，并加 int16 溢出保护 |
| **`Get_Current()` 单位标错** | 改成 `raw/16384 × max_current × 1000`（真 mA）|

顺带：**过温降额**（70℃ 起线性降额、85℃ 断输出，`DJI_MOTOR_TEMP_LIMIT_ENABLE` 可关）、
`DJI_Motor_Set_Angle_Here()`（绝对角度闭环的启动标定）、`DJI_Motor_Set_State()` 空指针保护、
控制器接口新增 `Reset`（`Cascade`/`Parallel` 会向下传播给子控制器）。

**对底盘的影响**：`user_pid.c` 算法一行未改，底盘整定结果保留。但
① 掉线保护会让"反馈断了的轮子立刻停"（以前会继续跑）；
② 过温降额是全新行为（不想要就关宏）；
③ 首帧修复对轮速环无影响（轮速用的是 `rotor_speed` 而非角度）。

---

## 七、为什么不能把重计算放进 1kHz 任务

**本工程所有任务都在 SysTick 中断里执行**（`user_systick.c` 的 `SysTick_Handle`），
所以任何任务的调用链都直接压在中断栈上，而且**任务超时会直接吃掉下一个 tick**。

`Newton_Solve` 内部有 `float J[36]`（144 字节）等局部数组，加上**本工程是软件浮点**
（`-mcpu=cortex-m4 -mthumb`，没有 `-mfpu`），`tanf/cosf` 一次要上千个周期。
把它放进 1kHz 任务里、又不判可解性的话，无解时会跑满迭代次数 →
**中断超过 1ms → 整套控制被饿死；同时 1KB 的栈会爆**。

因此：弹道解算挂在独立的 20ms 任务上、迭代次数夹住、并加了可解性预判。
另外链接脚本里 **栈 1KB → 2KB、堆 512B → 1KB**（RAM 只用到约 27%）。

---

## 八、已知限制

| 项 | 说明 |
|---|---|
| pitch 上电标定 | 每次上电姿态必须一致（相对标定，与 yaw 的"上电对准前方"同理）|
| `v0_k` / `v0_b` | 默认值只是占位，**必须实测标定** |
| 弹道模型无空气阻力 | 只做重力补偿；远距离（>6~8m）有系统偏差 |
| 距离靠手输 | 无视觉/测距输入，`dist` 目前用在线调参给 |
| `Servo_angle` 模式未实现 | `user_dji_motor.c` 里 case 是注释掉的 |
| CAN1 负载约 78% | 目前可用；再加班需挪轮子到 CAN2 |
| 增强 PID 的 `dt` 写死 | 默认 0.001，等于 CAN 反馈周期；改反馈率要同步改 |
| MIDI 曲库里 `bad_apple` | 885 音符约 177 秒，未被使用；开机音乐已加 1.5s 总时长上限 |
| IST8310 / IMU 恒温 | 板载但未启用 |
| BMI088 SPI 偏慢 | 预分频 256（328kHz），每拍阻塞读约 463µs，且控制任务组跑在最低优先级；若直行时也有顿挫，把 SPI 预分频改成 `_16`（5.25MHz，BMI088 上限 10MHz）|
| yaw 抗惯性参数是起步值 | `gimbal_world_kp/kd/ff` 需上机按"先阻尼后增益"整定 |

---

## 九、版本历史

### V2.0

**新增云台 yaw 抗惯性（这是"偏软"的根治）**
- 量化根因：外环时间常数 τ ≈ 6/world_kp，V1 的 `world_kp=1.0` → **τ≈6 秒**
- 新增**陀螺阻尼**项：`attitude.c` 原来把世界系 yaw 角速度 `yaw_rate` 算完就丢，现在暴露出来
  （`Attitude_Get_Yaw_Rate()`），经一阶低通后进外环做阻尼
- 外环增益 1.0→**2.0**（τ 6s→3s，稳了再往 3.0/5.0 加）、前馈 0.3→**1.0**（0.3 只有运动学正确值的 30%）
- **外环加输出限幅** `GIMBAL_WORLD_MAX_RPM=60`（原来完全没有限幅）
- 新增 `GIMBAL_IMU_ON_GIMBAL` 处理"IMU 装在云台还是底盘"这个前提
- **速度环 `GIMBAL_YAW_KI` 保持 0**：试过给 20，结果"刚上电云台就慢慢抖"（静止极限环，原因见第三节）

**修掉小陀螺"卡一下"的真 bug**（原怀疑的 ±180° 经审计**不成立**）
- `chassis_control.c` 小陀螺换向原来是 `(ch2 < 0) ? -spin_w : spin_w`，**没有死区**：
  摇杆回中/漂移时 omega 会在 ±4 rad/s 之间 **1kHz 反复翻转** → 已改成**死区 + 磁滞保持上次方向**
- 新增**自转角速度速率限制** `CHASSIS_W_SLEW=30 rad/s²`（原来全工程零速率限制）
- 修配置矛盾：`CHASSIS_MAX_W` 3.0 会把 `CHASSIS_SPIN_W` 4.0 **静默钳掉**，改成 5.0
- 跟随环跨 ±180° 时禁止积分（`head_ki>0` 时会出现"反向卡住不动"）

**掉线可观测性**
- 串口绘图模式 4 从 8 通道扩到 **10 通道**，新增 `online`（三电机反馈状态打包）
  与 `ready`（摩擦轮是否到转速），可区分"电机真掉线"和"拨杆档位没识别"

**其它**
- `shooter.h` / `gimbal.h` 补上 `user_systick.h`，变成自包含头文件

### V1.x
- 从 A 板移植到 C 板；新增云台 pitch（GM6020 ID4 @CAN2，串级 + 增强 PID）、
  射击机构（2×M3508 + 1×M2006 @CAN2，状态机 + FIR）、弹道解算（牛顿迭代）
- 电机底层加固：首帧编码器基准保护、掉线保护、按电调型号区分电流满量程、过温降额
- 串口绘图绘图模式可运行时切换（`plot_mode`）；CMSIS-DSP 只编译 7 个文件（644→89 编译单元）；
  栈 1KB→2KB、堆 512B→1KB；开机音乐加总时长上限

---

## 十、文档

| 文件 | 内容 |
|---|---|
| [docs/上机调试流程.md](docs/上机调试流程.md) | ★ **从烧录到验证全部功能的全套流程**：硬件核对 → 编译烧录 → 上电自检 → 串口绘图/调参 → 分模块验证（每步带通过判据与排错表）→ 参数整定 → 故障速查 |
| [docs/C板移植说明.md](docs/C板移植说明.md) | A 板 → C 板改了什么、CubeMX 要动哪些地方 |
| [docs/代码架构分析.md](docs/代码架构分析.md) | 框架分层与调用关系 |
| [docs/小陀螺与跟随_控制原理.md](docs/小陀螺与跟随_控制原理.md) | 小陀螺 / 跟随的控制推导 |
| [docs/C板工程与2027哨兵工程_底层与算法对比报告.docx](docs/C板工程与2027哨兵工程_底层与算法对比报告.docx) | 与 2027 哨兵参考工程（H723）的底层与算法对比 |
| [docs/A板原版README.md](docs/A板原版README.md) · [补遗](docs/A板原版README_补遗.md) | 原 A 板工程的 README（标定流程、调参指南等）|
