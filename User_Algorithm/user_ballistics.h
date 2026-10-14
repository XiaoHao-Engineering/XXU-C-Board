#ifndef USER_BALLISTICS_H
#define USER_BALLISTICS_H

#include "main.h"
#include "user_newton.h"

/* 弹道解算（中层纯算法，零硬件依赖）：由【水平距离 m / 高度差 m / 弹丸初速 m/s】解枪口俯仰角 deg，
   用本工程自带的 NEWTON_SOLVER（1 维）解关键公式 f(theta) = d*tan(theta) - g*d^2/(2*v0^2*cos^2(theta)) - dh = 0，
   第二项即重力下坠补偿（比 atan2(dh, d) 多出来的部分）。 */
#define BALLISTICS_GRAVITY          (9.80665f)   /* 重力加速度 m/s^2 */

/* 弹丸初速模型 v0 = v0_b + v0_k * 摩擦轮转速(RPM)（线性近似）：
   默认取 3500RPM -> 18m/s、6000RPM -> 28m/s；
   ⚠️ 这两个系数必须按自己的摩擦轮/弹丸实测标定（打两发不同转速测出膛速度即可）。 */
#define BALLISTICS_V0_K_DEF         (0.004f)
#define BALLISTICS_V0_B_DEF         (4.0f)

/* 求解器收敛阈值：残差单位是【米】，1e-5 m = 10µm 足够；
   user_newton 默认的 1e-6 在 float32 下不一定摸得到（会出现"解已对但 converged=0"）。 */
#define BALLISTICS_SOLVER_EPS       (1e-5f)

/* ★★ 迭代次数上限必须夹住：user_newton 默认 100 次是给多维方程组留的余量，
   而本工程是【软件浮点】，一次 tanf/cosf 要上千个周期，100 次迭代就是 1~2ms，
   足以把一个 1ms 周期的中断填满、把整套控制饿死。1 维弹道方程正常 3~5 次收敛。 */
#define BALLISTICS_MAX_ITERS        (20)

/* 残差兜底：就算求解器没报收敛，残差小于这个值（1mm）也认为结果可用 */
#define BALLISTICS_RESIDUAL_TOL     (1e-3f)

/* 距离 / 初速的合理范围：0.3~30 m、初速 >= 5 m/s，超了直接判无效，不浪费迭代 */
#define BALLISTICS_DISTANCE_MIN     (0.3f)
#define BALLISTICS_DISTANCE_MAX     (30.0f)
#define BALLISTICS_V0_MIN           (5.0f)

typedef struct {
    NEWTON_SOLVER solver;
    float solve_param[3];         /* 求解参数暂存：[0]=距离 m  [1]=高度差 m  [2]=初速 m/s */

    float distance_m;
    float height_m;
    float v0;
    float v0_k;
    float v0_b;

    float pitch_deg;              /* 解出来的俯仰角 deg（要打中所需）*/
    float pitch_direct_deg;       /* 直接指向目标的几何角 deg（迭代初值 + 解不出来时的兜底）*/
    float residual;               /* 收敛后残差 m */
    float max_range;              /* 当前初速下的理论最大射程 m（只用于观察）*/
    float disc;                   /* 可解性判别式：<0 = 这个距离用这个初速打不到，必须提前返回 */
    /* converged=1 求解器报收敛；valid=1 本次结果可用；unreachable=1 超过射程无实数解没求解 */
    uint8_t converged;
    uint8_t valid;
    uint8_t unreachable;
} BALLISTICS_DRIVES;

void    Ballistics_Init(BALLISTICS_DRIVES* ballistics);

/* 由摩擦轮转速 RPM 更新初速估计 m/s */
void    Ballistics_Set_V0_From_Rpm(BALLISTICS_DRIVES* ballistics, float rpm);

/* 解算。返回 1=结果可用；0=参数不合理或没解出来（此时 pitch_deg 退回几何角，保证上层有安全值）*/
uint8_t Ballistics_Solve(BALLISTICS_DRIVES* ballistics, float distance_m, float height_m);

float   Ballistics_Get_Pitch(const BALLISTICS_DRIVES* ballistics);

#endif // USER_BALLISTICS_H
