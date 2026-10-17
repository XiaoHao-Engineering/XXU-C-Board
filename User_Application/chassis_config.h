#ifndef CHASSIS_CONFIG_H
#define CHASSIS_CONFIG_H

/* 底盘参数集中配置 新乡学院 RoboMaster 哨兵 */

/* 底盘类型 ----------------------------------------------------------------*/
#define CHASSIS_MECANUM   (0)   /* 麦轮 */
#define CHASSIS_OMNI4     (1)   /* 四轮全向轮 */
#define CHASSIS_TYPE      CHASSIS_MECANUM

/* 机械参数 ----------------------------------------------------------------*/
#define CHASSIS_WHEELBASE (0.300f)         /* 轴距 m */
#define CHASSIS_TRACK     (0.300f)         /* 轮距 m */
#define CHASSIS_WHEEL_R   (0.076f)         /* 轮半径 m */
#define CHASSIS_RATIO     (3591.0f / 187.0f)   /* M3508 减速比 */

/* 轮序 0=左前 1=右前 2=左后 3=右后；装反了把对应项改成 -1（本车左右镜像安装，翻右侧）*/
#define CHASSIS_REV_FL    (1)
#define CHASSIS_REV_FR    (-1)
#define CHASSIS_REV_RL    (1)
#define CHASSIS_REV_RR    (-1)

/* 速度限幅 ----------------------------------------------------------------*/
/* ⚠️ CHASSIS_MAX_W 同时钳住小陀螺自转角速度（Chassis_Set_Velocity 内有幅值钳位），
   所以必须 >= CHASSIS_SPIN_W，否则 spin_w 调到 3 以上会【静默钳掉】、调了没反应。 */
#define CHASSIS_MAX_VX    (2.0f)    /* m/s */
#define CHASSIS_MAX_VY    (2.0f)    /* m/s */
#define CHASSIS_MAX_W     (5.0f)    /* rad/s */

/* 自转角速度速率限制 rad/s²：小陀螺换向/模式切换时不让 omega 阶跃。
   1kHz 下 30 rad/s² = 每拍最多变 0.03；满幅 ±4 反向约 0.27s，不顿挫也不迟钝。0=关闭 */
#define CHASSIS_W_SLEW    (30.0f)

/* 遥控器映射 --------------------------------------------------------------*/
/* DBUS 通道号：0=右手水平 1=右手垂直 2=左手水平 3=左手垂直 */
#define CHASSIS_RC_VX_CH  (3)       /* 前后：左摇杆垂直 */
#define CHASSIS_RC_VY_CH  (2)       /* 左右：左摇杆水平 */
#define CHASSIS_RC_W_CH   (0)       /* 手动模式原地旋转：右摇杆水平 */

/* ★ 手动模式是否允许"右摇杆左右 = 底盘原地自转"。
   V2.1 起右摇杆左右已经给云台 yaw 用了，所以默认 1 -> 0（关掉），
   底盘自转只保留在小陀螺/跟随模式里，右摇杆左右【只】控制 yaw。
   想恢复成"手动模式也能用右摇杆转底盘"，改回 1 即可（但会和 yaw 抢同一个通道）。 */
#define CHASSIS_RC_W_ENABLE (0)
#define CHASSIS_SPIN_DIR_CH (2)     /* 小陀螺模式转向：左摇杆水平 */

#define CHASSIS_RC_DEAD   (10)

#define CHASSIS_SPIN_W    (4.0f)    /* 自转角速度 rad/s */

/* 自转方向校正：轮子方向改过之后转反了就填 -1 ----------------------------*/
#define CHASSIS_OMEGA_DIR (1.0f)

/* 跟随 --------------------------------------------------------------------*/
/* Kp 单位 rad/s per deg：0.05 让满量程偏差(180 度)对应 9 rad/s，落在 MAX=3 的限幅内
   才是线性比例（30 那种取值只要偏 0.1 度就饱和，等于开关控制，必然来回振荡）*/
#define CHASSIS_FOLLOW_KP (0.05f)
#define CHASSIS_FOLLOW_KI (0.0f)
/* Kd 是【底盘实测角速度】的阻尼系数（不是 theta 的微分）：omega = Kp*theta - Kd*omega_实测。
   0.3 表示底盘满速 3rad/s 时抵消 0.9rad/s 的输出，能在到点前提前刹车；还摆加大、迟钝减小 */
#define CHASSIS_FOLLOW_KD (0.3f)
#define CHASSIS_FOLLOW_MAX (6.0f)

/* 轮速 PID ----------------------------------------------------------------*/
#define CHASSIS_SPEED_KP       (15.0f)
#define CHASSIS_SPEED_KI       (0.15f)
#define CHASSIS_SPEED_KD       (0.0f)
#define CHASSIS_SPEED_MAX_OUT  (16384.0f)   /* 电流环满量程 */
#define CHASSIS_SPEED_MAX_IOUT (4000.0f)

#define CHASSIS_WHEEL_POWER_MAX (200.0f) /* 单轮功率上限 W */

#define CHASSIS_PERIOD_MS   (1)     /* 底盘控制任务周期 ms */

/* 串口绘图 / 在线调参 ------------------------------------------------------*/
#define CHASSIS_SERIALPLOT_PERIOD_MS (10U)   /* 10ms=100Hz。12通道最大帧 50B -> 5.0kB/s，不到 115200 的一半。原来周期太小 -> 发送队列溢出、帧被截断 -> SerialPlot 永远同步不上（现象：117kbps 跑满但 0sps、校验全错）*/

/* 航向补偿：1=按参考系方向行进（小陀螺走直线） ----------------------------*/
/* 手动模式    -> 以【车头】为正方向（不补偿）；跟随/小陀螺 -> 以【云台指向】为正方向
   （补偿）；要求云台锁住世界方向，见 gimbal.h 的 GIMBAL_LOCK_WORLD */
#define CHASSIS_HEADING_COMP  (1)

/* 航向源（云台）掉线保护 --------------------------------------------------*/
/* 1=掉线时完全停车（最安全）；0=只停自转，平移仍可用（方便调试） */
#define CHASSIS_SOURCE_LOST_STOP  (0)

/* 串口绘图通道选择 --------------------------------------------------------*/
/* ★★ 串口绘图现在【只有一个固定模式】，上电即生效，没有模式切换、没有 plot_mode 命令。
   6 个通道（从上到下，顺序不可改）：
     ch1  yaw_enc_abs    YAW 电机单圈绝对角     度 0~360
     ch2  yaw_enc_rel    YAW 相对角             度 ±180
     ch3  pitch_enc_abs  pitch 电机单圈绝对角   度 0~360
     ch4  pitch_enc_rel  pitch 相对角           度 ±180
     ch5  target_yaw     遥控器发的目标 YAW     度 ±180
     ch6  target_pitch   遥控器发的目标 PITCH   度
   SerialPlot 里必须设：Frame Start=AB / #Channels=6 / Frame Size=24（6×4）
   另一端的开关（SERIALPLOT_CHECKSUM_ENABLE=1）决定线上帧长 26 字节。
   通道定义在 chassis_control.c 的 Chassis_Apply_Plot_Mode()；
   原来的 CHASSIS_SERIALPLOT_MODE 宏（多模式默认值）已随多模式一起删除。 */

#endif /* CHASSIS_CONFIG_H */
