#include "../../Core/Inc/bsp.h"

#include <math.h>
#include "../user_ballistics.h"

static float Ballistics_Equation(const float* x, const float* param);

/* 函数体 */

/* 被求解的方程（无空气阻力斜抛）。残差单位【米】：按该角度打出后，落点比目标高/低多少米。
   消掉飞行时间 t 得关键公式 f(theta) = d*tan(theta) - g*d^2/(2*v0^2*cos^2(theta)) - dh = 0；
   未知数 x[0]=俯仰角 theta（弧度）；param[0]=水平距离 d(m)、[1]=高度差 dh(m，高了为正)、[2]=初速 v0(m/s) */
static float Ballistics_Equation(const float* x, const float* param) {
    const float theta = x[0];
    const float d  = param[0];
    const float dh = param[1];
    const float v0 = param[2];

    const float cos_theta = cosf(theta);

    /* 除零保护：cos(theta) 接近 0（枪口竖直）或参数不合理时返回一个很大的残差，
       让牛顿迭代往回收，而不是产生 inf/nan（一旦出 nan，迭代会一路污染下去）。 */
    if (fabsf(cos_theta) < 1e-3f || d < 1e-3f || v0 < 1e-3f) {
        return 1.0e6f;
    }

    return d * tanf(theta)
         - BALLISTICS_GRAVITY * d * d / (2.0f * v0 * v0 * cos_theta * cos_theta)
         - dh;
}

void Ballistics_Init(BALLISTICS_DRIVES* ballistics) {
    memset(ballistics, 0, sizeof(BALLISTICS_DRIVES));

    /* n = 1：只解一个未知数（俯仰角）。
       ⚠️ Newton_Init 不重置 func_idx，所以 Newton_SetFunc 全工程只能对这个求解器调一次。 */
    Newton_Init(&ballistics->solver, 1);
    ballistics->solver.eps = BALLISTICS_SOLVER_EPS;
    /* ★ 必须夹住迭代次数，原因见 user_ballistics.h */
    ballistics->solver.max_iter = BALLISTICS_MAX_ITERS;
    Newton_SetFunc(&ballistics->solver, Ballistics_Equation);

    ballistics->distance_m = 3.0f;
    ballistics->height_m = 0.0f;
    ballistics->v0_k = BALLISTICS_V0_K_DEF;
    ballistics->v0_b = BALLISTICS_V0_B_DEF;
    ballistics->v0 = BALLISTICS_V0_MIN;
    ballistics->pitch_deg = 0.0f;
    ballistics->pitch_direct_deg = 0.0f;
    ballistics->residual = 0.0f;
    ballistics->converged = 0;
    ballistics->valid = 0;
}

void Ballistics_Set_V0_From_Rpm(BALLISTICS_DRIVES* ballistics, const float rpm) {
    float v0 = ballistics->v0_b + ballistics->v0_k * rpm;

    if (v0 < BALLISTICS_V0_MIN) {
        v0 = BALLISTICS_V0_MIN;
    }
    ballistics->v0 = v0;
}

uint8_t Ballistics_Solve(BALLISTICS_DRIVES* ballistics, const float distance_m, const float height_m) {
    ballistics->distance_m = distance_m;
    ballistics->height_m = height_m;
    ballistics->unreachable = 0;

    /* 直接指向目标的几何角：既当牛顿迭代的初值（兜底），也当解不出来时的输出 */
    ballistics->pitch_direct_deg = atan2f(height_m, distance_m) * 57.29578f;
    ballistics->pitch_deg = ballistics->pitch_direct_deg;

    /* 理论最大射程（平地）：v0^2/g。只用于观察，不参与求解 */
    ballistics->max_range = (ballistics->v0 * ballistics->v0) / BALLISTICS_GRAVITY;

    if (distance_m < BALLISTICS_DISTANCE_MIN || distance_m > BALLISTICS_DISTANCE_MAX
        || ballistics->v0 < BALLISTICS_V0_MIN) {
        ballistics->converged = 0;
        ballistics->valid = 0;
        ballistics->residual = 0.0f;
        ballistics->disc = 0.0f;
        return 0;
    }

    /* ★★ 可解性预判（这一步是"死机"的根治点，不能删）：令 u = tan(theta)，
       cos^2 用 1/(1+u^2) 换掉，方程化为 A*u^2 - d*u + (A + dh) = 0，A = g*d^2/(2*v0^2)，
       判别式 disc = d^2 - 4*A*(A + dh)。disc < 0 = 没有实数解，也就是这个初速根本打不到这个距离。
       必须在这里提前返回：否则牛顿迭代会一直不收敛、把 100 次 tanf/cosf 全跑完，
       而本工程是软件浮点，在 1ms 中断里跑满 100 次足以把整套控制饿死。 */
    const float a_coef = BALLISTICS_GRAVITY * distance_m * distance_m
                       / (2.0f * ballistics->v0 * ballistics->v0);
    ballistics->disc = distance_m * distance_m - 4.0f * a_coef * (a_coef + height_m);

    if (ballistics->disc < 0.0f) {
        ballistics->unreachable = 1;
        ballistics->converged = 0;
        ballistics->valid = 0;
        ballistics->residual = 0.0f;
        /* 一次迭代都不做，pitch 保持"指向目标"的几何角 */
        return 0;
    }

    ballistics->solve_param[0] = distance_m;
    ballistics->solve_param[1] = height_m;
    ballistics->solve_param[2] = ballistics->v0;

    /* ★ 初值取低伸弹道的解析解 u = (d - sqrt(disc)) / (2A)，牛顿迭代几步就收敛；
       A 很小时（近距离/高初速）这个式子会 0/0，退回几何角。 */
    float x0[1];
    if (a_coef > 1.0e-6f) {
        const float u_low = (distance_m - sqrtf(ballistics->disc)) / (2.0f * a_coef);
        x0[0] = atanf(u_low);
    } else {
        x0[0] = ballistics->pitch_direct_deg * 0.0174532925f;
    }

    ballistics->converged = Newton_Solve(&ballistics->solver, ballistics->solve_param, x0);
    ballistics->residual = Newton_GetResidual(&ballistics->solver);

    const float solved_deg = Newton_GetX(&ballistics->solver)[0] * 57.29578f;

    /* 结果可用性：① 求解器报收敛；或 ② 残差已经足够小（float32 下不一定压到 1e-6，
       这时 converged=0 但其实解已经对了）；再夹一下角度，防止求解器跑飞给出离谱的角。 */
    const uint8_t ok = (ballistics->converged != 0)
                    || (ballistics->residual < BALLISTICS_RESIDUAL_TOL);

    if (ok && solved_deg > -80.0f && solved_deg < 80.0f) {
        ballistics->pitch_deg = solved_deg;
        ballistics->valid = 1;
    } else {
        ballistics->pitch_deg = ballistics->pitch_direct_deg;
        ballistics->valid = 0;
    }

    return ballistics->valid;
}

float Ballistics_Get_Pitch(const BALLISTICS_DRIVES* ballistics) {
    return ballistics->pitch_deg;
}
