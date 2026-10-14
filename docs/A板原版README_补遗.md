## 板载 IMU 与串口调参

### 一、新增文件与分层

| 层 | 文件 | 作用 |
|---|---|---|
| 底层 | `User_Drives/User_Peripheral/user_spi.c/h` | SPI 收发 + 片选管理 |
| 底层 | `User_Drives/User_Sensor/user_mpu6500.c/h` | MPU6500 寄存器读写、量程换算、零偏标定 |
| 底层 | `User_Drives/User_Peripheral/user_serialplot.c/h` | 串口绘图 + 在线调参 |
| 中层 | `User_Algorithm/User_Attitude/attitude.c/h` | 互补滤波姿态解算（roll / pitch / yaw） |
| 上层 | `User_Application/chassis_control.c/h` | 小陀螺行进、跟随、模式切换 |

### 二、A 板板载 IMU 接线

| 功能 | 引脚 |
|---|---|
| SPI5 | `PF7` SCK / `PF8` MISO / `PF9` MOSI |
| 片选 CS | `PF6`（普通 GPIO，低有效） |
| MPU6500 中断 | `PB8`（暂未使用，轮询读取） |
| 磁力计 Set/Reset | `PE2` |
| 磁力计中断 | `PE3` |
| 串口绘图 | `USART2`（PD5 / PD6，115200） |

> **磁力计（IST8310）暂未接入。** yaw 目前是陀螺积分，**会漂** —— 先测出漂移量再决定要不要接。

### 三、串口绘图 / 在线调参

**发给上位机**：`0xAB` 帧头 + N×`float` + 校验和（1 字节累加和）
默认 4 通道：`vx / vy / omega / yaw`

**上位机发给板子**：`变量名=值#`，例如 `wheel_kp=18.5#`

| 变量名 | 含义 | 范围 |
|---|---|---|
| `spin_w` | 小陀螺自转角速度 | 0 ~ 20 |
| `dead` | 摇杆死区 | 0 ~ 100 |
| `head_kp` / `head_ki` / `head_kd` | 航向 PID | — |
| `wheel_kp` / `wheel_ki` / `wheel_kd` | 4 个轮速 PID（一起改） | — |

**上报周期**：`CHASSIS_SERIALPLOT_PERIOD_MS`（默认 10ms = 100Hz）。
115200 波特率下 4 通道每帧 18 字节，理论最高约 640Hz；通道越多越慢。要更高刷新率就把 USART2 提到 921600。

### 四、三种底盘模式（遥控器左拨杆 sw1）

| sw1 | 模式 | 自转 ω | 行进方向 |
|---|---|---|---|
| 上（1） | **跟随** | 把航向夹角控到 0 | 按航向补偿 |
| 中（3） | **手动** | `ch2` 直接给 | 按航向补偿 |
| 下（2） | **小陀螺** | 恒定 `spin_w`，方向由 `ch2` 定 | 按航向补偿 |

### 五、航向补偿（小陀螺走直线的关键）

遥控器的 vx/vy 被当作**参考系**的速度，下发前先按航向角转到**车体系**：

```
vx_body = vx·cos(θ) − vy·sin(θ)
vy_body = vx·sin(θ) + vy·cos(θ)
```

`θ` 由 `ChassisControl_Set_HeadingSource()` 注册的函数给出，含义是
**「参考系相对底盘」的角度（deg，逆时针为正）**。

| 你的情况 | 航向源返回什么 |
|---|---|
| **IMU 装在底盘上**（现在） | `Attitude_Get_Yaw()` —— 参考系是**世界系**，可以直接测「小陀螺走直线」 |
| **IMU 装在云台上** | 云台 yaw 编码器 → 云台相对底盘的角度，参考系是**云台系** |
| 什么都没接 | `ChassisControl_NoHeading()` 返回 0，退化为车体系直控 |

`main.c` 里已经挂好了 IMU 版本：

```c
static float Chassis_HeadingSource(void) {
    return Attitude_Get_Yaw(&user_attitude);
}
```

**以后接云台编码器，只把这个函数换成读编码器，别的代码一行都不用动。**

### 六、零偏标定与漂移测试

1. 上电后**保持底盘静止约 1 秒**（默认 1000 次采样），自动完成陀螺零偏标定
2. 之后 `user_mpu6500.gyro[]` 已减掉零偏
3. **测漂移**：静止 1 分钟，看 `yaw` 漂了多少度 —— 这就是你这块板子的真实零漂水平
   - 用串口绘图把 `yaw` 画出来，或 JScope 看
4. `MPU6500_Calibrate_Start(&user_mpu6500)` 可以随时重新标定

### 七、上电必须现场核对的两件事

| 项 | 怎么核对 | 不对就改 |
|---|---|---|
| **传感器轴向** | 车放平看 `roll` / `pitch` 是否 ≈ 0；倾斜时符号对不对 | `attitude.h` 的 `ATTITUDE_SIGN_X/Y/Z` |
| **航向正方向** | 手动逆时针转车头，看 `yaw` 是不是**增大** | `ATTITUDE_SIGN_Z` 的符号 |

### 八、依赖说明

- 姿态解算用到 `sqrtf` / `atan2f`，所以 `CMakeLists.txt`（和它的基准副本 `cmake/CMakeLists.app.txt`）里**加了 `-lm`**。
  这一步同时修掉了框架里 `user_coord.c` 的潜在链接问题。
- `user_cf.c` 的互补滤波**从「未接线」变成了在用工**。
- 加热（PB5）**没有启用**，CubeMX 里也没配。

