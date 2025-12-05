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





typedef struct {
    // 滑模面 s=alpha*x1+c*x2+beta*x2^(p/q)
    float alpha;// x1系数
    float c;// x2系数
    float beta;// x2^(p/q)系数
    uint8_t p;// 分子
    uint8_t q;// 分母

    float k1;// 增强项系数
    float k2;// 抗饱和项系数
    float k; // 决定sat区间
    float J;// 惯量
    float i_max;// 积分限幅
    float out_max;// 输出限幅

    float x1;// 位置误差
    float x2;// 速度误差
    float s_sum;// 滑模面积分
		float s_test;//观察s的窗口变量
	
    uint8_t flag;// 初始化标志位
    float T; //采样周期
    float torque;// 输出力矩
    float last_torque;// 上次输出力矩
    float last_omiga;// 上次角速度
}SMC_s;

/**
 * @brief SMC控制器计算函数
 * @param smc SMC控制器参数结构体指针
 * @param theta 当前角度
 * @param target_theta 目标角度
 * @param omega 当前角速度
 * @return float 输出力矩
 */
float SMC_Calc(SMC_s* smc, float theta, float target_theta, float omega);


#endif //SMC_H
