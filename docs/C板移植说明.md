# A 板 → C 板移植说明

> 原工程：`C:\Users\xhao0\Desktop\XXU-RoboMaster-Sentry\XXU-RoboMaster-Sentry-main`（RoboMaster 开发板 **A 型**，STM32F427IIH6）
> 移植目标：本工程（RoboMaster 开发板 **C 型**，STM32F407IGH6）
> 原则：**上中下层结构、文件路径、命名风格、注释风格与原工程完全一致**；只把「板级相关」的部分换成 C 板的。

---

## 一、整体结论

| 项 | 状态 |
|---|---|
| 目录分层（`Core` / `User_Drives` / `User_Algorithm` / `User_Application` / `User_Architect` / `User_Lib` / `SEGGER_RTT`） | ✅ 完全照搬，一个目录都没删没改 |
| `User_Algorithm`（PID / ADRC / LADRC / 级联 / 并联 / 坐标 / 波形 / FIR / CF / 姿态） | ✅ 原样搬过来，只改了 `attitude.c/h` 里 IMU 的类型名 |
| `User_Architect`（缓冲 / 队列 / 环形缓冲 / 编解码 / 延时 / SysTick） | ✅ 原样 |
| `User_Application`（底盘 / 云台 / 底盘控制 / 开机音乐） | ✅ 原样，只改了 2 行注释里的 `TIM2` → `TIM10` |
| `User_Drives`（UART / CAN / SPI / PWM / TIMER / ADC / RNG / Serialplot / DJI 电机 / DBUS / LED / 蜂鸣器 / HWT906 / STP23） | ✅ 全部原样，代码一行没动（它们都是「把 HAL 句柄传进来」的写法，本来就与板子无关） |
| 板级差异（引脚 / 外设实例 / 时钟 / 中断） | 🔁 只改这一层：`bsp.h`、`bsp_config.h`、`bsp.c`、`main.c`、`stm32f4xx_it.c`、`.ioc`、`CMakeLists.txt` |
| 板载 IMU 驱动 | 🆕 C 板物理上是 BMI088，A 板的 MPU6500 驱动在 C 板上跑不了，所以按原工程风格新写了 `user_bmi088.c/h`（原 `user_mpu6500.c/h` 保留备查） |

---

## 二、硬件对照表

| 功能 | A 板（原工程） | C 板（本工程） | 说明 |
|---|---|---|---|
| MCU | STM32F427IIH6，180MHz，2MB Flash / 256KB RAM | **STM32F407IGH6，168MHz，1MB Flash / 192KB RAM** | 链接脚本换 `STM32F407xx_FLASH.ld` |
| CAN1 | PD0 / PD1 | **PD0 / PD1** | 一致 |
| CAN2 | PB12 / PB13 | **PB5 / PB6** | 改了 |
| 遥控器 DBUS | USART1：PB6 / PB7，100000 + 偶数校验 | **USART3：PC10 / PC11，100000 + 9bit + 偶数校验** | 见下方 ⚠️ 备注 |
| 姿态传感器 HWT906 | USART3：PC10 / PC11，115200 | **USART6：PG9 / PG14，115200** | 改了 |
| 串口绘图 / 在线调参 | USART2：PD5 / PD6，115200 | **USART1：PA9 / PB7，115200** | C 板没有 USART2，波特率和 A 板保持一致 |
| 板载 IMU | MPU6500 @ SPI5（PF7/PF8/PF9，CS = PF6） | **BMI088 @ SPI1（PB3/PB4/PA7，CS1 = PA4 加速度计 / CS2 = PB0 陀螺仪）** | 芯片换了，驱动重写 |
| 磁力计 | IST8310 未接 | **IST8310 板载（PF0 = SDA / PF1 = SCL，I2C2）** | 本次**未启用**（原来也没启用）；I2C2 已由 CubeMX 初始化好，随时能加 |
| 蜂鸣器 | TIM12_CH1，PH6 | **TIM4_CH3，PD14** | 改了 |
| 状态灯 | 红灯 PE11 / 绿灯 PF14 | **RGB 三色灯：红 = PH12 / 绿 = PH11 / 蓝 = PH10** | 引脚已在 CubeMX 里加好 |
| 可控电源 | PG13(POWER_5V)、PH2~PH5(POWER_24V_1~4) | **无此功能** | C 板没有可控电源开关，相关初始化已删掉 |
| 采样节拍（JScope） | TIM2，10kHz | **TIM10，10kHz（ARR = 16800-1）** | 见下方说明 |
| IMU 恒温加热 | 无 | PF6 / TIM10_CH1 | C 板有，本次按原工程同样**不启用**（只把定时器拿来做节拍用） |

### ⚠️ 关于 DBUS 的校验位（顺便帮 A 板挑出个隐患）

STM32F4 的 HAL 里，`UART_WORDLENGTH_8B` 表示「8 位**含**校验位」，`UART_WORDLENGTH_9B` 才是「8 数据位 + 1 校验位」：

| 配置 | 实际帧格式 | 对不对 |
|---|---|---|
| `WORDLENGTH_8B` + `PARITY_EVEN` | 7 数据位 + 偶校验（共 8 位） | ❌ A 板 USART1 就是这个，DBUS 其实是 8E1，配错了 |
| `WORDLENGTH_9B` + `PARITY_EVEN` | **8 数据位 + 偶校验（共 9 位）** | ✅ 这才是 DBUS 要的 8E1 |

- C 板 CubeMX 里 USART3 已经是 **9bit + EVEN**，正好是对的，**不用改**。
- A 板那份建议你有空也把 USART1 的 Word Length 改成 9 Bits。

### 为什么 JScope 节拍从 TIM2 换到 TIM10

- C 板蜂鸣器**物理上就接在 PD14 / TIM4_CH3**，逃不掉。
- 而 `user_pwm.c` 的 `PWM_Set_Frequency()` 会去改定时器的 `PSC` 和 `ARR`（靠改这两个寄存器变调）。
- 所以只要节拍挂在 TIM4 上，一放开机音乐，采样节拍就跟着音调乱跑。
- 于是把 10kHz 采样节拍挪到 **TIM10**（C 板上 TIM10_CH1 是 IMU 恒温加热通道，本次不启用，正好空闲）。
- `JScope_Transmit(100)` 隐含「100µs 一次」，所以 TIM10 也要配成 10kHz：`TIM10_CLK = 168MHz`，`PSC = 0`，`ARR = 16800-1`。

---

## 三、CubeMX 需要改动的地方（照这个清单核对就行）

本工程的 `Desktop.ioc` **已经替你改好了**，下面这份清单是「你已经配好的 + 我改的 + 需要你确认的」全量对照。

### 3.1 已经帮你改好（打开 CubeMX 就是对的）

| # | 位置 | 改动 | 为什么 |
|---|---|---|---|
| 1 | `PH10` | 新增 GPIO_Output，Label = `LED_BLUE`，Pull-up，初始 High | C 板 RGB 灯蓝 |
| 2 | `PH11` | 新增 GPIO_Output，Label = `LED_GREEN`，Pull-up，初始 High | C 板 RGB 灯绿 |
| 3 | `PH12` | 新增 GPIO_Output，Label = `LED_RED`，Pull-up，初始 High | C 板 RGB 灯红 |
| 4 | `PA4` | 加 Label = `IMU_CS_ACCEL`，PinState 改成 **High** | BMI088 加速度计片选，空闲必须拉高 |
| 5 | `PB0` | 加 Label = `IMU_CS_GYRO`，PinState 改成 **High** | BMI088 陀螺仪片选，空闲必须拉高 |
| 6 | `TIM10` | `Prescaler = 0`，`Period = 16800-1`（原来 167 / 999 = 1kHz） | 改成 10kHz，给 JScope 当节拍 |
| 7 | NVIC | 勾上 **`TIM1_UP_TIM10_IRQn`** | TIM10 的更新中断要进中断向量，否则节拍不来 |

> 改完这 7 条，在 CubeMX 里 **GENERATE CODE**，然后跑一次
> `powershell -File tools/cubemx_postgen.ps1`
> 把被模板覆盖的 `CMakeLists.txt` 还原回来（并自动检查 USER CODE 钩子有没有丢）。

### 3.2 你原来就配对的，不用动

| 外设 | 配置 | 结论 |
|---|---|---|
| CAN1 | PD0/PD1，1Mbps（Prescaler 3 / BS1 10TQ / BS2 3TQ） | ✅ 对 |
| CAN2 | PB5/PB6，1Mbps | ✅ 对 |
| USART3 | PC10/PC11，100000，9bit，EVEN | ✅ 正是 DBUS 要的 |
| USART6 | PG9/PG14，115200 8N1 | ✅ 给 HWT906 用 |
| SPI1 | PB3/PB4/PA7，Mode 3（CPOL High / CPHA 2Edge），Master，2Lines | ✅ BMI088 要 Mode 3 |
| TIM4 | PD14，CH3 PWM，PSC 83 / ARR 999 ≈ 1kHz | ✅ 蜂鸣器，`PWM_Set_Frequency` 会自己重算 PSC/ARR，初值不影响 |
| I2C2 | PF0/PF1，Fast | ✅ IST8310 用，本次没启用 |
| DMA | USART1_RX(Normal)、USART1_TX、USART3_RX(**Circular**)、USART6_RX/TX | ✅ 和代码用法对得上（DBUS 用 Circular，其余 ReceiveToIdle 用 Normal） |
| 时钟 | HSE 12M → PLLM 6 / PLLN 168 / PLLP 2，HCLK 168M，APB1 42M，APB2 84M | ✅ C 板上限 |

### 3.3 建议你决定的一件事：SPI1 预分频（BMI088 的读写速度）

⚠️ **这个是本次检查里唯一值得改的性能项，但改之前要先看下面的「JScope 抢占」注意事项。**

SPI1 挂在 APB2 = 84MHz，预分频 256 → **328.125 kbit/s**，也就是**一个字节 24.4µs**。
BMI088 每 1ms 要读三个事务：

| 事务 | 字节数 | 耗时 @256 |
|---|---|---|
| 加速度数据（命令 + dummy + 6 字节） | 8 | 195µs |
| 温度（命令 + dummy + 2 字节） | 4 | 98µs |
| 角速度（命令 + 6 字节） | 7 | 171µs |
| **合计** | **19** | **≈ 464µs** |

`BMI088_Handle()` 是在 **SysTick 中断里**跑的，1ms 一次、每次占 464µs ≈ **46% 的中断时间**。
功能上没问题（SysTick 优先级是最低的 15，饿不死 CAN/UART），但确实浪费。

| 预分频 | 时钟 | 每字节 | 19 字节耗时 | 占 1ms |
|---|---|---|---|---|
| 256（现在） | 328 kbit/s | 24.4 µs | 464 µs | 46% |
| **64（推荐）** | **1.3125 Mbit/s** | **6.1 µs** | **116 µs** | **11.6%** |
| 32（激进） | 2.625 Mbit/s | 3.05 µs | 58 µs | 5.8% |
| 16（不建议，除非下面那条一起改） | 5.25 Mbit/s | 1.5 µs | 29 µs | 2.9% |

在 CubeMX 里改：`SPI1 → Parameter Settings → Baud Rate Prescaler`。

#### ⚠️ 为什么不能无脑拉到最快：JScope 会抢占 SPI 传输

- `BMI088_Handle()` 用的是 **阻塞式** `HAL_SPI_TransmitReceive`，靠 CPU 一个字节一个字节搬。
- 而 **TIM10 的 JScope 节拍是 10kHz、NVIC 优先级 0**（最高），**SysTick 是 15**（最低）——
  也就是说 JScope 中断**可以随时插进 SPI 传输中间**。
- JScope 回调本身很短（3 个 float 赋值 + 一次 RTT memcpy，约 **2µs**），
  但这 2µs 里 SPI 仍在收字节：**只要字节间隔小于 2µs，就会 SPI 溢出（OVR）→ 这一帧 IMU 数据错位**（HAL 默认不查 OVR，直接就返回 HAL_OK）。

按这个算：

| 每字节时间 | 2µs 抢占的影响 |
|---|---|
| 24.4 µs（256） | 安全，余量 12 倍 |
| 6.1 µs（64） | 安全，余量 3 倍 ← **推荐** |
| 3.05 µs（32） | 临界 |
| 1.5 µs（16） | 大概率偶发错帧 |

> 顺带说明：A 板 MPU6500 用的是 SPI5 @5.625MHz（每字节 1.78µs）+ TIM2 JScope，
> 结构上**同样存在这个问题**，只是 15 字节一次性搬完、窗口小，平时不容易撞上。

**想同时拿到高速和稳定**，二选一：
1. 把 JScope 节拍频率从 10kHz 降下来（改 `TIM10` 的 `ARR`，同时 `JScope_Transmit(100)` 的 100 要改成对应的 µs 数）；或
2. 直接把 JScope 关掉：`main.c` 里 `JScope_Init(&htim10)` 改成 `JScope_Init(NULL)`（它传 NULL 就不启动定时器），
   采样节拍 ISR 一没，SPI 想跑多快跑多快。

> 这条**没有帮你直接改**，因为 SPI 时钟和 JScope 取舍关系到信号完整性和调试手段，想让你自己拍板。
> 当前预分频 256 能稳定工作，只是 CPU 白烧。

### 3.4 A 板有、C 板没有的（已从代码里去掉，知道一下就行）

- `USART2`（原串口绘图口）→ 换成 USART1
- `TIM2`（原采样节拍）→ 换成 TIM10
- `TIM12`（原蜂鸣器）→ 换成 TIM4_CH3
- `SPI5`（原 MPU6500）→ 换成 SPI1
- `PG13 / PH2~PH5`（可控 5V / 24V 电源）→ C 板没有
- `PB8 / PE2 / PE3`（MPU6500 中断 / 磁力计复位 / 磁力计中断）→ C 板这些脚用途不同，未使用

---

## 四、代码改动清单（改了什么，一行不多）

### 4.1 `Core/Inc/bsp_config.h`
- 加 `BMI088_NUM` / `BMI088_CALLBACK_NUM`（照 `MPU6500_NUM` 的样子写）。
- `MPU6500_NUM` 保留（原驱动还在，只是不参与构建）。
- 时钟宏 `APB1_TIM_CLK` 等本来就是 `HAL_RCC_GetPCLK1Freq()` 算出来的，板子无关，**没动**。

### 4.2 `Core/Inc/bsp.h` / `Core/Src/bsp.c`
- `user_timer_2` → `user_timer_10`（顺带改名，跟真实实例对上）。
- `user_spi_5` → `user_spi_accel` + `user_spi_gyro`（BMI088 是双片选，要两个 SPI 实例）。
- `user_mpu6500` → `user_bmi088`，类型 `MPU6500_DRIVES` → `BMI088_DRIVES`。
- 其它全局实例（CAN / 灯 / 蜂鸣器 / DBUS / 电机 / PID / 云台 / 底盘 / serialplot / attitude）**一字未改**。

### 4.3 `Core/Src/main.c`
只改 `USER CODE` 区，CubeMX 生成的部分没碰：
```c
JScope_Init(&htim10);                                   /* 原 &htim2  */
TIMER_Init(&user_timer_10, &htim10, TIM10_CLK);         /* 原 htim2 / APB1_TIM_CLK */
UART_Init(&user_imu_uart, &huart6);                     /* 原 &huart3  */
LED_Init(&user_red_led,   LED_RED_GPIO_Port,   LED_RED_Pin,   1);   /* 原 PE11 */
LED_Init(&user_green_led, LED_GREEN_GPIO_Port, LED_GREEN_Pin, 1);   /* 原 PF14 */
Buzzer_Init(&user_buzzer_1, &htim4, TIM_CHANNEL_3, TIM4_CLK);       /* 原 htim12/CH1/90000000 */
DBUS_Init(&user_dbus, &huart3);                         /* 原 &huart1  */
SPI_Init(&user_spi_accel, &hspi1, IMU_CS_ACCEL_GPIO_Port, IMU_CS_ACCEL_Pin);  /* 新 */
SPI_Init(&user_spi_gyro,  &hspi1, IMU_CS_GYRO_GPIO_Port,  IMU_CS_GYRO_Pin);   /* 新 */
BMI088_Init(&user_bmi088, &user_spi_accel, &user_spi_gyro, 1);                /* 原 MPU6500_Init */
Attitude_Init(&user_attitude, &user_bmi088, 0.001f);                          /* 原 &user_mpu6500 */
UART_Init(&user_serialplot_uart, &huart1);              /* 原 &huart2  */
```
底盘电机 / PID / 云台 / 航向源 / SysTick 任务那些**全部原样**。

### 4.4 `Core/Src/stm32f4xx_it.c` / `Core/Inc/stm32f4xx_it.h`
- `DBUS_Handler(&huart1)` → 挪到 **`USART3_IRQHandler`** 里的 `DBUS_Handler(&huart3)`。
- `SysTick_Handler` 里补 `SysTick_Handle();`（原工程有）。
- 新增 `TIM1_UP_TIM10_IRQHandler` → `HAL_TIM_IRQHandler(&htim10)`。
- 原工程的 `USART2_IRQHandler` 那段「USER CODE 手工补中断」的注释和代码删掉了（C 板没有 USART2；USART1/3/6 的 CubeMX 都勾了 global interrupt，不需要补）。

### 4.5 `Core/Src/gpio.c` / `Core/Inc/main.h`
- `main.h` 里加 CubeMX 风格的引脚标签：`LED_RED_Pin/GPIO_Port`、`LED_GREEN_*`、`LED_BLUE_*`、`IMU_CS_ACCEL_*`、`IMU_CS_GYRO_*`。
- `gpio.c` 里加 PH10/11/12 的初始化，并把 PA4/PB0 的初始电平改成高、速度改成 `VERY_HIGH`（原工程 `IMU_CS` 就是这么写的）。
- 删掉 A 板的 `MAG_RST / MAG_INT / IMU_INT / POWER_5V / POWER_24V_x` 初始化。

### 4.6 `Core/Src/tim.c`
- TIM10：`Prescaler 167→0`、`Period 999→16800-1`，并在 `HAL_TIM_Base_MspInit` 里加上 `TIM1_UP_TIM10_IRQn` 的 NVIC 使能。

### 4.7 新增：`User_Drives/User_Sensor/user_bmi088.c/.h`
按 `user_mpu6500.c/.h` 的风格一比一写：
- 同样的目录位置、同样的 `#include "../../../Core/Inc/bsp.h"` + `#ifdef HAL_SPI_MODULE_ENABLED` 包法。
- 同样的接口：`BMI088_Init / BMI088_RegisterCallback / BMI088_Calibrate_Start / BMI088_Is_Calibrated`。
- 同样的字段名：`gyro[]` / `accel[]` / `temperature` / `gyro_bias[]` / `calibrated` / `is_online` / `callbacks[]` / `task`，所以 `attitude.c` 只改了类型名。
- 同样的 1000 次上电零偏标定逻辑、同样的注释风格。
- 差异只在硬件本身：
  - BMI088 **双片选**（加速度计 PA4 / 陀螺仪 PB0），所以结构体里有两个 `SPI_DRIVES*`。
  - BMI088 加速度计 **SPI 读固定多 1 个 dummy 字节**，数据从第 3 个字节开始。
  - 加速度计上电默认在 I2C 模式，要先空读一次芯片 ID 把 SPI 激活。
  - 量程换成 BMI088 的：陀螺 ±2000dps（16.384 LSB/°/s），加速度 ±6g（5461.333 LSB/g）；陀螺 1kHz/116Hz 滤波，加速度 800Hz 正常带宽。
  - 多了 `accel_id` / `gyro_id` 两个回读字段（0x1E / 0x0F），调试时一眼能看出 SPI 通没通。

### 4.8 `User_Algorithm/User_Attitude/attitude.c/.h`
- `#include "../../User_Drives/User_Sensor/user_mpu6500.h"` → `user_bmi088.h`。
- `MPU6500_DRIVES* user_mpu6500` → `BMI088_DRIVES* user_bmi088`（字段名也跟着改）。
- 算法本体（互补滤波、俯仰投影、`Wrap180`）**一行没改**。

### 4.9 构建系统
| 文件 | 改动 |
|---|---|
| `CMakeLists.txt` + `cmake/CMakeLists.app.txt` | `STM32F427xx` → `STM32F407xx`；链接脚本 → `STM32F407xx_FLASH.ld`；其余（`-mcpu=cortex-m4`、软浮点、`-lm`、`objcopy` 生成 hex/bin、工程名 `XXU-RoboMaster-Sentry`）完全照旧 |
| `Core/Startup/startup_stm32f407xx.s` | 新增。原工程的启动文件是放在 `Core/Startup/` 下靠 `Core/*.*` 通配吃进去的，不在根目录 |
| `Drivers/CMSIS/DSP/` | 把原工程的整棵 CMSIS-DSP 源码树搬过来（见下方 ⚠️） |
| `tools/cubemx_postgen.ps1` | 检查项换成 C 板的（`DBUS_Handler(&huart3)`、`TIM1_UP_TIM10_IRQHandler`、`BMI088_Init` 等） |
| `.vscode/c_cpp_properties.json` | `STM32F427xx` → `STM32F407xx` |
| `.vscode/launch.json` | SVD 换成 `STM32F407.svd` |

#### ⚠️ 为什么 CMSIS-DSP 必须跟着源码一起编

`user_coord.c` 和 `user_fir.c` 真的在调用 CMSIS-DSP：

```c
arm_dot_prod_f32(...);   arm_sin_cos_f32(...);   arm_sin_f32(...);
arm_fir_init_f32(...);   arm_fir_f32(...);
```

- 原工程的 `file(GLOB_RECURSE "Drivers/*.*")` 会把 `Drivers/CMSIS/DSP/Source/**` 全部吃进去一起编译 —— 所以 A 板的 DSP 是**用软浮点编出来的**，和主工程 ABI 一致。
- C 板 CubeMX 给的 `Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a` 里那个 **`lf` = 硬件浮点 ABI**。本工程是软浮点（`-mfloat-abi` 默认 soft，为了「不用硬件浮点、避免 NoCP」，和原工程一致），两者 ABI 不兼容：`arm_sin_cos_f32(theta, ...)` 这种「float 按值传参」的函数会从错误的寄存器取参数，结果直接是错的。
- 所以把原工程的 DSP 源码树整套搬到 `Drivers/CMSIS/DSP/`，跟原工程一样从源码编 —— 路径 `Drivers/CMSIS/DSP/Include` 也和 `user_fir.h` 里写死的 `"../../Drivers/CMSIS/DSP/Include/arm_math.h"` 对得上，**用户代码一个字都不用改**。
- 代价：首次编译多编 ~560 个 DSP 源文件，之后增量编译不受影响。

---

## 五、上电自检顺序（照这个查最快）

| 步 | 看什么 | 不对就查 |
|---|---|---|
| 1 | 绿灯常亮 + 红灯 800ms 闪 | PH11/PH12 焊没焊反、`LED_Init` 第 4 个参数（低电平点亮 = 传 1，反了传 0） |
| 2 | 开机音乐响 | PD14 / TIM4_CH3；`Buzzer_Init` 传的是 `TIM4_CLK`（84MHz）不是 A 板的 90MHz |
| 3 | 串口 115200 收到 ASCII `SERIALPLOT OK` | USART1 PA9(TX) / PB7(RX)，波特率 |
| 4 | 遥控器推杆能动 | 必须是 **USART3**（PC10/PC11），且 Word Length = 9 Bits + Even |
| 5 | CAN 上电机能转 | CAN2 换成 PB5/PB6 了；终端电阻 |
| 6 | 在线调参能改参数 | 上位机发 `wheel_kp=18.5#`，回读 `Serialplot` |
| 7 | 打印/看 `user_bmi088.accel_id == 0x1E && gyro_id == 0x0F` | 不通就查 SPI1 Mode 3、CS 是不是 PA4/PB0、`SPI_Init` 传的引脚对不对 |
| 8 | 放平后 `pitch ≈ 0`，逆时针转头 `yaw` 增大 | 改 `attitude.h` 的 `ATTITUDE_SIGN_X/Y/Z` |

> ⚠️ 上电后**保持静止约 1 秒**，BMI088 要完成 1000 次陀螺零偏标定，这期间 `gyro` 输出是 0。

---

## 六、遗留 / 可以继续做的

1. **IST8310 磁力计没启用**（和原工程保持一致）。C 板是板载的、走 I2C2（PF0/PF1），接上以后 yaw 就有绝对参考、不再漂。要做的话新增 `user_ist8310.c/h`，在 `Attitude_Update` 里补一个磁力计融合分支。
2. **IMU 恒温加热没启用**（和原工程保持一致，A 板也没这功能）。C 板的加热在 PF6 / TIM10_CH1；要用的话注意 TIM10 现在兼职做着 JScope 节拍，`PWM_Set_Frequency` 会改 ARR，得再挪一次。
3. **A 板 USART1 的 DBUS 校验位建议改成 9 Bits**（见上文 ⚠️）。
4. `user_mpu6500.c/h` 保留在 `User_Sensor/` 里备查，仍然参与编译（不占 Flash，`--gc-sections` 会回收）。不想留着直接删掉这两个文件即可。
5. `cmake/stm32cubemx/CMakeLists.txt` 是 CubeMX 生成的、给「CubeMX 原生 CMake 结构」用的；本工程走的是 `CMakeLists.txt`（扁平 GLOB 结构），它不参与构建，CubeMX 重新生成时会自己刷新。

---

## 七、移植完成后的验证结果

用 `arm-none-eabi-gcc 14.3.1` 和 `CMakeLists.txt` 里**完全一样的编译/链接参数**，把全部源码编了一遍：

| 项 | 结果 |
|---|---|
| 编译 | **644 个源文件（641 个 C + 3 个汇编）全部通过，零 error** |
| 链接 | **通过**（`-Wl,-gc-sections -T STM32F407xx_FLASH.ld -lm`） |
| FLASH 占用 | 44448 B / 1024 KB（4.24%） |
| RAM 占用 | 31760 B / 128 KB（24.23%），CCMRAM 0 B |
| 向量表 | `0x08000000` 处 = `0x20020000`（`_estack` = 128KB RAM 顶端 ✓）、`0x08001E95`（`Reset_Handler`+thumb ✓） |
| 关键符号 | `main` / `BMI088_Init` / `BMI088_Handle` / `Attitude_Update` / `DBUS_Handler` / `TIM1_UP_TIM10_IRQHandler` / `ChassisControl_Task` / `HAL_TIM_PeriodElapsedCallback` / `HAL_UARTEx_RxEventCallback` / `HAL_CAN_RxFifo0MsgPendingCallback` / `user_bmi088` / `user_chassis` 全部就位 |
| 新增/改写的文件加 `-Wall -Wextra` | **零告警**（`user_uart.c` 有 1 条 `-Wsign-compare`，那是原工程本来就有的，没动） |

> 链接后**没有未定义符号**，说明改用源码编译的 CMSIS-DSP（`arm_sin_cos_f32` / `arm_fir_f32` 等）全部解析成功。

### 已经在真机上验证过 CubeMX 重新生成

你在 CubeMX 里点过 GENERATE CODE 之后复查过一遍，结论：**该保住的都保住了**。

| 项 | 结果 |
|---|---|
| `Desktop.ioc` 里我加的 LED 标签（PH10/11/12）、`IMU_CS_ACCEL/GYRO` 标签、`TIM1_UP_TIM10_IRQn`、`TIM10` 的 PSC/ARR | 全在（CubeMX 只是把 `Mcu.Pin` 列表重新排了序） |
| `Core/Src/gpio.c`、`Core/Inc/main.h` | CubeMX 按它的格式重写了一遍，内容等价（LED 三色 + BMI088 双片选初始电平都是对的） |
| `Core/Src/tim.c` | `PSC=0 / ARR=16800-1` + `TIM1_UP_TIM10_IRQn` 的 NVIC 使能都在 |
| `Core/Src/main.c`、`stm32f4xx_it.c` 里的 `USER CODE` | 全部保留（`JScope_Init(&htim10)`、`DBUS_Init(&user_dbus, &huart3)`、`BMI088_Init(...)`、`SysTick_Handle()`、`DBUS_Handler(&huart3)`） |
| 根 `CMakeLists.txt` | **没有被覆盖**（本工程里没有 `CMakeLists_template.txt`，CubeMX 不会重写它）。保险起见跑一下 `tools\cubemx_postgen.ps1` 也不亏 |
| `Core/Startup/`、`Core/Src/bsp.c` | 原样没动 |

> 注意：CubeMX 会把 `PA4/PB0` 的 GPIO Speed 重置成默认的 `LOW`。本次已经把它们同步成 `GPIO_SPEED_FREQ_VERY_HIGH`
> （和 A 板 `IMU_CS` 一个写法）；`.ioc` 里也补上了 `PA4.GPIO_Speed` / `PB0.GPIO_Speed`，以后再生成就不会掉。

### ⚠️ 关于「我这边没能跑 CMake」

我这边的沙箱环境里 `cmake -G Ninja` 会在 try_compile 阶段卡死（连一个 `int main(void){return 0;}` 的空工程也一样卡），
所以上面是用**和 CMakeLists 逐字一致的参数**直接调 `arm-none-eabi-gcc` 验证的，没有真正走过 `cmake --build`。
参数是同源的，代码本身已经证明能编过、能链上；你那边按 VS Code 的「重新配置 CMake」+「编译」正常走一遍即可。
