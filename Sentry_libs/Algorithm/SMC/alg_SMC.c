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


float Sat(float u,float k) {
    if (u > 1.0f/k) return 1;
    else if (u < -1.0f/k) return -1;
    else return k*u;
}

float clamp(float u, float min, float max) {
    if (u > max) return max;
    if (u < min) return min;
    return u;
}

// k=0.01
float filter(float last_u,float u,float k) {
    return last_u*(1-k)+u*k;
}




float real_rational_power(float x, int p, int q)
{
    if (x >= 0.0) return powf(x, (float)p / q);
    /* x<0：仅当 q 为奇数时实根存在 */
    if (q & 1)                /* 快速判奇 */
        return -powf(-x, (float)p / q);
    else                      /* 偶数分母，数学上进复数域 */
        return 0;           /* 或者你自己决定怎么处理 */
}

float SMC_Calc(SMC_s* smc, float theta, float target_theta, float omega) {
    float x1 = target_theta - theta;
    float x2 = 0 - filter(smc->last_omiga,omega,0.1f);

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
    smc->torque = filter(smc->last_torque,smc->torque, 0.93f);
    smc->last_torque = smc->torque;
		smc->s_test=s;//调试用
    return smc->torque;
}




