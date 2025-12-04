/**
*   @file SMC.h
*   @brief 简要的用于控制位置的滑膜控制器
*   @author Wenxin HU
*   @date 25-12-1
*   @version 1.0
*   @note
*/
#ifndef ALG_SMC_H
#define ALG_SMC_H
#include "stdint.h"
#include "stm32h7xx_hal.h"
#include "core_cm7.h"

inline float Sat(float u,float k) {
    if (u > 1.0f/k) return 1;
    else if (u < -1.0f/k) return -1;
    else return k*u;
}

inline float clamp(float u, float min, float max) {
    if (u > max) return max;
    if (u < min) return min;
    return u;
}

// k=0.01
inline float filter(float last_u,float u,float k) {
    return last_u*(1-k)+u*k;
}

typedef struct {
    // 滑模面 s=alpha*x1+c*x2+beta*x2^(p/q)
    float alpha;
    float c;
    float beta;
    uint8_t p;
    uint8_t q;

    float k1;
    float k2;
    float k; // 决定sat区间
    float J;
    float i_max;
    float out_max;

    float x1;
    float x2;
    float s_sum;

    uint8_t flag;
    float T; //采样周期
    float torque;
    float last_torque;
    float last_omiga;
}SMC_s;


float SMC_Calc(SMC_s* smc, float theta, float target_theta, float omega);

inline void DWT_Init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 允许 DWT
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // 启用 cycle counter
}

inline uint32_t DWT_GetCycle(void) {
    return DWT->CYCCNT;
}

inline float DWT_GetMicroseconds(uint32_t start, uint32_t end) {
    uint32_t cycles = end - start;
    return (float)cycles / (HAL_RCC_GetHCLKFreq() / 1e6f);
}

#endif //SMC_H
