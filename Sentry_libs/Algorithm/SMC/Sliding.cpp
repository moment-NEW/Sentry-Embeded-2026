//
// Created by Lenovo on 2024/3/15.
//

#include "Sliding.hpp"
#include "Sliding_C_Interface.h"

void cSMC::Init()
{
    smc.param.J = 0;
    smc.param.K = 0;
    smc.param.c = 0;
    smc.param.epsilon = 0;
    smc.flag = EXPONENT;
    smc.u_max = 0;
    smc.limit = 0;

    smc.error.tar_now = 0;
    smc.error.tar_last = 0;
    smc.error.tar_differential = 0;

    smc.error.p_error = 0;
    smc.error.v_error = 0;
    smc.error.v_error_integral = 0;
//    smc.error.v_error_integral_max = 0;
    smc.error.pos_error_eps = 0;
    smc.error.vol_error_eps = 0;
    smc.error.pos_get = 0;
    smc.error.vol_get = 0;
}

void cSMC::SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EXPONENT,POWER,VELSMC 参数设定
    smc.param.J = J;
    smc.param.K = K;
    smc.param.c = c;
    smc.error.pos_error_eps = pos_esp;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();
}
void cSMC::SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///TFSMC 参数设定
    smc.param.J = J;
    smc.param.K = K;
    smc.param.p = p;
    smc.param.q = q;
    smc.error.pos_error_eps = pos_esp;
    smc.param.beta = beta;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();
}

void cSMC::SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp) { ///EISMC 参数设定
    smc.param.J = J;
    smc.param.K = K;
    smc.param.c1 = c1;
    smc.param.c2 = c2;
    smc.error.pos_error_eps = pos_esp;
    smc.flag = flag;
    smc.param.epsilon = epsilon;
    smc.u_max = u_max;
    smc.limit = limit;
    OutContinuation();
}

void cSMC::ErrorUpdate(float target, float pos_now, float vol_now) //位置环误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.p_error = pos_now - target;
    smc.error.v_error = vol_now - smc.error.tar_differential;
    smc.error.tar_last = smc.error.tar_now;

    smc.error.p_error_integral += (float)(smc.error.p_error * SAMPLE_PERIOD); ///位置误差积分项

    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}

void cSMC::ErrorUpdate(float target,float vol_now) //速度环误差更新
{
    smc.error.tar_now = target;
    smc.error.tar_differential = (float)((smc.error.tar_now - smc.error.tar_last)/SAMPLE_PERIOD);

//    smc.error.tar_differential_second = (float)((smc.error.tar_differential- smc.error.tar_differential_last)/SAMPLE_PERIOD); ///二阶导

    smc.error.v_error = vol_now - smc.error.tar_now;

    smc.error.v_error_integral += (float)(smc.error.v_error * SAMPLE_PERIOD); ///速度误差积分项
//    if(std::abs(smc.error.v_error_integral) > smc.error.v_error_integral_max) //积分限幅
//    {
//        smc.error.v_error_integral = smc.error.v_error_integral_max;
//    }

    smc.error.tar_last = smc.error.tar_now;

//    smc.error.tar_differential_last = smc.error.tar_differential; ///二阶导更新

}
void cSMC::Clear()
{
    smc.error.tar_now = 0;
    smc.error.tar_last = 0;
    smc.error.tar_differential = 0;

    smc.error.p_error = 0;
    smc.error.v_error = 0;
    smc.error.v_error_integral = 0;
//    smc.error.v_error_integral_max = 0;
    smc.error.pos_error_eps = 0;
    smc.error.vol_error_eps = 0;
    smc.error.pos_get = 0;
    smc.error.vol_get = 0;

    smc.error.tar_differential_second = 0;
    smc.error.tar_differential_last = 0;
    smc.error.p_error_integral = 0;
}
void cSMC::Integval_Clear()
{
    smc.error.v_error_integral = 0;
    smc.error.p_error_integral = 0;
}

float cSMC::SmcCalculate()
{
    float u,fun;

    switch (smc.flag) {
        case EXPONENT:///线性滑模面，指数趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            smc.s = smc.param.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun + smc.error.tar_differential_second); //控制器计算,指数趋近率
            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率

            break;
        case POWER:///线性滑模面，幂次趋近率

            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            smc.s = smc.param.c * smc.error.p_error + smc.error.v_error; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
//            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.K * (std::pow(std::abs(smc.s),smc.param.epsilon)) * fun + smc.error.tar_differential_second); //控制器计算,幂次趋近率
            u =  smc.param.J * ( (-smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.K * (std::pow(std::abs(smc.s),smc.param.epsilon)) * fun); //控制器计算,幂次趋近率

            break;
        case TFSMC:///tfsmc
            static float pos_pow;//tfsmc 位置 幂
            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            pos_pow = std::pow(std::abs(smc.error.p_error),smc.param.q/smc.param.p);
            if(smc.error.p_error<=0) pos_pow = -pos_pow;


            smc.s = smc.param.beta * pos_pow + smc.error.v_error; //滑模面

            fun = Sat(smc.s);//饱和函数消除抖动

            if(smc.error.p_error!=0)
            {
                u = smc.param.J * (smc.error.tar_differential_second//目标值的二阶导 暂定是否删除，需测试
                             -smc.param.K * smc.s //s*K
                             -smc.param.epsilon * fun  //epsilon*SAT(S)
                             -smc.error.v_error * ((smc.param.q * smc.param.beta) * pos_pow) / (smc.param.p * smc.error.p_error)); //控制器计算
            }
            else u = 0;
            break;
        case VELSMC:///比例积分滑模面，指数趋近律，速度控制
            smc.s = smc.error.v_error + smc.param.c * smc.error.v_error_integral; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
            u =  smc.param.J * (smc.error.tar_differential - (smc.param.c * smc.error.v_error) - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算，速度控制
            break;

        case EISMC:///比例积分滑模面，指数趋近律，位置控制
            if (std::abs(smc.error.p_error) - smc.error.pos_error_eps < 0)
            {
                smc.error.p_error = 0;
                return 0;
            }

            smc.s = smc.param.c1 * smc.error.p_error + smc.error.v_error + smc.param.c2 * smc.error.p_error_integral; //滑模面
            fun = Sat(smc.s);//饱和函数消除抖动
            u =  smc.param.J * ( (-smc.param.c1 * smc.error.v_error)- smc.param.c2 * smc.error.p_error - smc.param.K * smc.s - smc.param.epsilon * fun); //控制器计算,指数趋近率
            break;
    }

    smc.error.error_last = smc.error.p_error; //更新上一步的误差

    //控制量限幅
    if (u > smc.u_max)
    {
        u = smc.u_max;
    }
    if (u < -smc.u_max)
    {
        u = -smc.u_max;
    }
    smc.u = u;
    return u;
}

void cSMC::OutContinuation()
{
    if(smc.param.K != 0 && smc.param.c2 != 0)
    {
        smc.error.p_error_integral = (smc.param_last.K / smc.param.K) * (smc.param_last.c2 / smc.param.c2) * smc.error.p_error_integral;
        smc.error.v_error_integral = (smc.param_last.K / smc.param.K) * (smc.param_last.c / smc.param.c) * smc.error.v_error_integral;
    }
    smc.param_last = smc.param;
}


// 符号函数
float cSMC::Signal(float s)
{
    if (s > 0)
        return 1;
    else if (s == 0)
        return 0;
    else
        return -1;
}

//饱和函数
float cSMC::Sat(float s)
{
    float y;
    y = s / smc.param.epsilon;
    if (std::abs(y) <= smc.limit)
        return y;
    else
        return Signal(y);
}

const Sliding &cSMC::getSmc() const {
    return smc;
}

float cSMC::Out() {
    return smc.u;
}

void cSMC::SetOut(float out) {
    smc.u = out;
}

// ============================================================================
// C Interface - For use in gimbal_task.c
// ============================================================================

// Global SMC instance for C code
static cSMC g_smc_pitch;
static cSMC g_smc_roll;

#ifdef __cplusplus
extern "C" {
#endif

// Pitch SMC interface
void smc_pitch_init(void) {
    g_smc_pitch.Init();
}

void smc_pitch_set_param_eismc(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, float pos_eps) {
    g_smc_pitch.SetParam(J, K, c1, c2, epsilon, limit, u_max, EISMC, pos_eps);
}

void smc_pitch_error_update(float target, float pos_now, float vol_now) {
    g_smc_pitch.ErrorUpdate(target, pos_now, vol_now);
}

float smc_pitch_calculate(void) {
    return g_smc_pitch.SmcCalculate();
}

float smc_pitch_get_out(void) {
    return g_smc_pitch.Out();
}

float smc_pitch_get_s(void) {
    const Sliding &smc = g_smc_pitch.getSmc();
    return smc.s;
}

void smc_pitch_clear(void) {
    g_smc_pitch.Clear();
}

void smc_pitch_integral_clear(void) {
    g_smc_pitch.Integval_Clear();
}

// Roll SMC interface
void smc_roll_init(void) {
    g_smc_roll.Init();
}

void smc_roll_set_param_eismc(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, float pos_eps) {
    g_smc_roll.SetParam(J, K, c1, c2, epsilon, limit, u_max, EISMC, pos_eps);
}

void smc_roll_error_update(float target, float pos_now, float vol_now) {
    g_smc_roll.ErrorUpdate(target, pos_now, vol_now);
}

float smc_roll_calculate(void) {
    return g_smc_roll.SmcCalculate();
}

float smc_roll_get_out(void) {
    return g_smc_roll.Out();
}

void smc_roll_clear(void) {
    g_smc_roll.Clear();
}

void smc_roll_integral_clear(void) {
    g_smc_roll.Integval_Clear();
}

// Config getter implementations - allows Keil debugger to monitor/modify parameters
static SMC_Config pitch_config = {0};
static SMC_Config roll_config = {0};

// Bind returns a pointer to a live config you can modify in Keil
SMC_Config* smc_pitch_config_bind(void) {
    const Sliding &smc = g_smc_pitch.getSmc();
    // initialize from current params once
    pitch_config.flag = smc.flag;
    pitch_config.J = smc.param.J;
    pitch_config.K = smc.param.K;
    pitch_config.c = smc.param.c;
    pitch_config.c1 = smc.param.c1;
    pitch_config.c2 = smc.param.c2;
    pitch_config.epsilon = smc.param.epsilon;
    pitch_config.limit = smc.limit;
    pitch_config.u_max = smc.u_max;
    pitch_config.pos_eps = smc.error.pos_error_eps;
    return &pitch_config;
}

SMC_Config* smc_roll_config_bind(void) {
    const Sliding &smc = g_smc_roll.getSmc();
    roll_config.flag = smc.flag;
    roll_config.J = smc.param.J;
    roll_config.K = smc.param.K;
    roll_config.c = smc.param.c;
    roll_config.c1 = smc.param.c1;
    roll_config.c2 = smc.param.c2;
    roll_config.epsilon = smc.param.epsilon;
    roll_config.limit = smc.limit;
    roll_config.u_max = smc.u_max;
    roll_config.pos_eps = smc.error.pos_error_eps;
    return &roll_config;
}

// Apply config into internal SMC safely (supports all modes)
void smc_pitch_apply_config(void) {
    Rmode mode = (Rmode)pitch_config.flag;
    switch (mode) {
        case EXPONENT:
        case POWER:
        case VELSMC:
            g_smc_pitch.SetParam(
                pitch_config.J,
                pitch_config.K,
                pitch_config.c,
                pitch_config.epsilon,
                pitch_config.limit,
                pitch_config.u_max,
                mode,
                pitch_config.pos_eps
            );
            break;
        case EISMC:
            g_smc_pitch.SetParam(
                pitch_config.J,
                pitch_config.K,
                pitch_config.c1,
                pitch_config.c2,
                pitch_config.epsilon,
                pitch_config.limit,
                pitch_config.u_max,
                EISMC,
                pitch_config.pos_eps
            );
            break;
        default:
            break;
    }
}

void smc_roll_apply_config(void) {
    Rmode mode = (Rmode)roll_config.flag;
    switch (mode) {
        case EXPONENT:
        case POWER:
        case VELSMC:
            g_smc_roll.SetParam(
                roll_config.J,
                roll_config.K,
                roll_config.c,
                roll_config.epsilon,
                roll_config.limit,
                roll_config.u_max,
                mode,
                roll_config.pos_eps
            );
            break;
        case EISMC:
            g_smc_roll.SetParam(
                roll_config.J,
                roll_config.K,
                roll_config.c1,
                roll_config.c2,
                roll_config.epsilon,
                roll_config.limit,
                roll_config.u_max,
                EISMC,
                roll_config.pos_eps
            );
            break;
        default:
            break;
    }
}

#ifdef __cplusplus
}
#endif