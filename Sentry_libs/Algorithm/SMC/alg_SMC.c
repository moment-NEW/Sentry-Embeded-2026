/**
*   @file SMC.c
*   @brief 
*   @author Wenxin HU
*   @date 25-12-1
*   @version 1.0
*   @note
*/
#include "alg_SMC.h"
#include "math.h"

float real_rational_power(float x, int p, int q)
{
    if (x >= 0.0) return powf(x, (float)p / q);
    /* x<0：仅当 q 为奇数时实根存在 */
    if (q & 1)                /* 快速判奇 */
        return -powf(-x, (float)p / q);
    else                      /* 偶数分母，数学上进复数域 */
        return 0;           /* 或者你自己决定怎么处理 */
}

float SMC_Calc(SMC_s* smc, float theta, float target_theta, float omega, float target_omega) {
    float x1 = target_theta - theta;
    float x2 = target_omega - filter(smc->last_omiga,omega,0.1f);

    smc->x1 = x1;
    smc->x2 = x2;

    float s = smc->alpha * x1 + smc->c * x2 + smc->beta * (real_rational_power(x2,smc->p,smc->q));

    smc->s_sum += smc->T*Sat(s, smc->k);
    smc->s_sum = clamp(smc->s_sum, -smc->i_max, smc->i_max);

    float I = smc->k2 * smc->s_sum;

    smc->torque = (smc->J / (smc->c + smc->beta * (smc->p * 1.0f / smc->q) * real_rational_power(x2,smc->p - smc->q,smc->q)))
        * (smc->k1 * sqrtf(fabsf(s)) * Sat(s, smc->k) + I + smc->alpha * x2);

    if (smc->flag) {
        smc->last_torque = smc->torque;
    }
    smc->torque = clamp(smc->torque, -smc->out_max, smc->out_max);
    smc->torque = filter(smc->last_torque,smc->torque, 0.001f);
    smc->last_torque = smc->torque;
    return smc->torque;
}

inline float sgnf(float x){ return (x>0)-(x<0); }

void TrjGen_Init(TrjGen* g, float theta0, float vmax, float amax){
    g->theta = theta0; g->omega = 0; g->vmax = vmax; g->amax = amax;
}

// 每个控制周期调用，返回限速后的theta_ref，并可同时拿到omega_ref
float TrjGen_Step(TrjGen* g, float theta_target, float dt){
    float e = theta_target - g->theta;
    float dir = sgnf(e);

    // 为了能在目标前刹停，计算需要的制动速度上限
    float v_stop = sqrtf(fmaxf(0.0f, 2.0f * g->amax * fabsf(e)));
    float v_des  = dir * fminf(g->vmax, v_stop);

    // 以amax限加速度地逼近期望速度
    float dv = v_des - g->omega;
    float dv_lim = fmaxf(-g->amax*dt, fminf(g->amax*dt, dv));
    g->omega += dv_lim;

    // 积分得到角度参考
    g->theta += g->omega * dt;
    return g->theta;
}
