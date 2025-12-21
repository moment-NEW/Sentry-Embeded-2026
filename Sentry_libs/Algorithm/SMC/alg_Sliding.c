#include "alg_Sliding.h"


#define SMC_MAX_CNT 6
static float smc_signal(float s) {
    if (s > 0.0f) return 1.0f;
    if (s < 0.0f) return -1.0f;
    return 0.0f;
}

static float smc_sat(const Sliding *smc, float s) {
    if (!smc) return 0.0f;

    // 保护：epsilon 非法时退化为 sign(s)，避免除 0 或 NaN/Inf 传播
    if (!isfinite(smc->param.epsilon) || smc->param.epsilon <= 0.0f || !isfinite(s)) {
        return smc_signal(s);
    }

    float y = s / smc->param.epsilon;
    // 保护：limit 非法时同样退化为 sign
    if (!isfinite(smc->limit) || smc->limit <= 0.0f || !isfinite(y)) {
        return smc_signal(s);
    }

    if (fabsf(y) <= smc->limit)
        return y;
    return smc_signal(y);
}

static void smc_out_continuation(Sliding *smc) {
    if (smc->param.K != 0.0f && smc->param.c2 != 0.0f) {
        smc->error.p_error_integral =
            (smc->param_last.K / smc->param.K) * (smc->param_last.c2 / smc->param.c2) * smc->error.p_error_integral;
        // For v_error_integral scaling: use c1 if c is zero (EISMC mode), else use c
        if (smc->param.c != 0.0f && smc->param_last.c != 0.0f) {
            smc->error.v_error_integral =
                (smc->param_last.K / smc->param.K) * (smc->param_last.c / smc->param.c) * smc->error.v_error_integral;
        } else if (smc->param.c1 != 0.0f && smc->param_last.c1 != 0.0f) {
            smc->error.v_error_integral =
                (smc->param_last.K / smc->param.K) * (smc->param_last.c1 / smc->param.c1) * smc->error.v_error_integral;
        }
    }
    smc->param_last = smc->param;
}

void smc_init(Sliding *smc) {
    
    smc->param.J = 0.0f;
    smc->param.K = 0.0f;
    smc->param.c = 0.0f;
    smc->param.c1 = 0.0f;
    smc->param.c2 = 0.0f;
    smc->param.p = 0.0f;
    smc->param.q = 0.0f;
    smc->param.beta = 0.0f;
    smc->param.epsilon = 0.0f;
    smc->flag = EXPONENT;
    smc->u_max = 0.0f;
    smc->limit = 0.0f;

    smc->error.tar_now = 0.0f;
    smc->error.tar_last = 0.0f;
    smc->error.tar_differential = 0.0f;
    smc->error.tar_differential_last = 0.0f;
    smc->error.tar_differential_second = 0.0f;

    smc->error.p_error = 0.0f;
    smc->error.v_error = 0.0f;
    smc->error.v_error_integral = 0.0f;
    smc->error.p_error_integral = 0.0f;
    smc->error.pos_error_eps = 0.0f;
    smc->error.vol_error_eps = 0.0f;
    smc->error.pos_get = 0.0f;
    smc->error.vol_get = 0.0f;
    smc->error.error_last = 0.0f;

    smc->u = 0.0f;
    smc->s = 0.0f;

    smc->param_last = smc->param;
}

void smc_set_param_exp(Sliding *smc, float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_eps) {
    if (!smc) return;
    smc->param.J = J;
    smc->param.K = K;
    smc->param.c = c;
    smc->param.epsilon = epsilon;
    smc->limit = limit;
    smc->u_max = u_max;
    smc->flag = flag;
    smc->error.pos_error_eps = pos_eps;
    smc_out_continuation(smc);
}

void smc_set_param_tfsmc(Sliding *smc, float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_eps) {
    if (!smc) return;
    smc->param.J = J;
    smc->param.K = K;
    smc->param.p = p;
    smc->param.q = q;
    smc->param.beta = beta;
    smc->param.epsilon = epsilon;
    smc->limit = limit;
    smc->u_max = u_max;
    smc->flag = flag;
    smc->error.pos_error_eps = pos_eps;
    smc_out_continuation(smc);
}

void smc_set_param_eismc(Sliding *smc, float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_eps) {
    if (!smc) return;
    smc->param.J = J;
    smc->param.K = K;
    smc->param.c1 = c1;
    smc->param.c2 = c2;
    smc->param.epsilon = epsilon;
    smc->limit = limit;
    smc->u_max = u_max;
    smc->flag = flag;
    smc->error.pos_error_eps = pos_eps;
    smc_out_continuation(smc);
}

void smc_error_update_pos(Sliding *smc, float target, float pos_now, float vol_now) {
    if (!smc) return;
    smc->error.tar_now = target;
    smc->error.tar_differential = (smc->error.tar_now - smc->error.tar_last) / SAMPLE_PERIOD;
    smc->error.tar_differential_second = (smc->error.tar_differential - smc->error.tar_differential_last) / SAMPLE_PERIOD;

    smc->error.p_error = pos_now - target;
    smc->error.v_error = vol_now - smc->error.tar_differential;
    smc->error.tar_last = smc->error.tar_now;

    smc->error.p_error_integral += smc->error.p_error * SAMPLE_PERIOD;
    // 增加积分限幅
    if (smc->error.p_error_integral > P_ERROR_INTEGRAL_MAX) smc->error.p_error_integral = P_ERROR_INTEGRAL_MAX;
    else if (smc->error.p_error_integral < -P_ERROR_INTEGRAL_MAX) smc->error.p_error_integral = -P_ERROR_INTEGRAL_MAX;
    // 防护：检查积分是否为NaN/Inf
    if (!isfinite(smc->error.p_error_integral)) smc->error.p_error_integral = 0.0f;

    smc->error.tar_differential_last = smc->error.tar_differential;
}

void smc_error_update_vel(Sliding *smc, float target, float vol_now) {
    if (!smc) return;
    smc->error.tar_now = target;
    smc->error.tar_differential = (smc->error.tar_now - smc->error.tar_last) / SAMPLE_PERIOD;

    smc->error.v_error = vol_now - smc->error.tar_now;
    smc->error.v_error_integral += smc->error.v_error * SAMPLE_PERIOD;
    // 增加积分限幅
    if (smc->error.v_error_integral > V_ERROR_INTEGRAL_MAX) smc->error.v_error_integral = V_ERROR_INTEGRAL_MAX;
    else if (smc->error.v_error_integral < -V_ERROR_INTEGRAL_MAX) smc->error.v_error_integral = -V_ERROR_INTEGRAL_MAX;
    // 防护：检查积分是否为NaN/Inf
    if (!isfinite(smc->error.v_error_integral)) smc->error.v_error_integral = 0.0f;

    smc->error.tar_last = smc->error.tar_now;
}

void smc_clear(Sliding *smc) {
    if (!smc) return;
    smc->error.tar_now = 0.0f;
    smc->error.tar_last = 0.0f;
    smc->error.tar_differential = 0.0f;
    smc->error.tar_differential_last = 0.0f;
    smc->error.tar_differential_second = 0.0f;

    smc->error.p_error = 0.0f;
    smc->error.v_error = 0.0f;
    smc->error.v_error_integral = 0.0f;
    smc->error.p_error_integral = 0.0f;
    smc->error.pos_error_eps = 0.0f;
    smc->error.vol_error_eps = 0.0f;
    smc->error.pos_get = 0.0f;
    smc->error.vol_get = 0.0f;
    smc->error.error_last = 0.0f;

    smc->u = 0.0f;
    smc->s = 0.0f;
}

void smc_integral_clear(Sliding *smc) {
    if (!smc) return;
    smc->error.v_error_integral = 0.0f;
    smc->error.p_error_integral = 0.0f;
}

float smc_calculate(Sliding *smc) {
    if (!smc) return 0.0f;
    float u = 0.0f;
    float fun = 0.0f;

    // 备份原始误差用于计算，但在死区内将其视为0
    float p_err = smc->error.p_error;
    float v_err = smc->error.v_error;

    if (fabsf(p_err) < smc->error.pos_error_eps) {
        p_err = 0.0f;
        v_err = 0.0f; // 进入死区后通常也忽略微小速度波动
    }

    switch (smc->flag) {
        case EXPONENT:
            smc->s = smc->param.c * p_err + v_err;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * ((-smc->param.c * v_err) - smc->param.K * smc->s - smc->param.epsilon * fun);
            break;

        case POWER:
            smc->s = smc->param.c * p_err + v_err;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * ((-smc->param.c * v_err)
                - smc->param.K * smc->s
                - smc->param.K * powf(fabsf(smc->s), smc->param.epsilon) * fun);
            break;

        case TFSMC: {
            static float pos_pow;
            pos_pow = powf(fabsf(p_err), smc->param.q / smc->param.p);
            if (p_err < 0.0f) pos_pow = -pos_pow;

            smc->s = smc->param.beta * pos_pow + v_err;
            fun = smc_sat(smc, smc->s);

            if (fabsf(p_err) > 1e-6f) {
                u = smc->param.J * (
                    smc->error.tar_differential_second
                    - smc->param.K * smc->s
                    - smc->param.epsilon * fun
                    - v_err * ((smc->param.q * smc->param.beta) * pos_pow) / (smc->param.p * p_err)
                );
            } else {
                u = -smc->param.J * smc->param.K * smc->s;
            }
            break;
        }

        case VELSMC:
            smc->s = v_err + smc->param.c * smc->error.v_error_integral;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * (smc->error.tar_differential
                - (smc->param.c * v_err)
                - smc->param.K * smc->s
                - smc->param.epsilon * fun);
            break;

        case EISMC:
            // EISMC 模式下，即使 p_err 为 0，积分项 p_error_integral 依然起作用
            smc->s = smc->param.c1 * p_err + v_err + smc->param.c2 * smc->error.p_error_integral;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * (
                (-smc->param.c1 * v_err)
                - smc->param.c2 * p_err
                - smc->param.K * smc->s
                - smc->param.epsilon * fun
            );
            break;
    }

    smc->error.error_last = smc->error.p_error;

    // 防护：检查输出是否为NaN/Inf
    if (!isfinite(u)) u = 0.0f;

    if (u > smc->u_max) u = smc->u_max;
    if (u < -smc->u_max) u = -smc->u_max;
    smc->u = u;
    return u;
}

float smc_get_out(const Sliding *smc) {
    return smc ? smc->u : 0.0f;
}

void smc_set_out(Sliding *smc, float out) {
    if (!smc) return;
    smc->u = out;
}

const Sliding *smc_get_state_const(const Sliding *smc) {
    return smc;
}





/**
 * @brief 根据配置结构体初始化滑模控制器实例
 * @param smc 滑模控制器实例指针
 * @param config 配置结构体指针
 * 
 */
void smc_init_with_config(Sliding *smc, const SlidingConfig *config) {
    if (!config) return;
    smc_init(smc);  // 先调用原init
    // 根据flag调用相应set_param函数
    switch (config->flag) {
        case EXPONENT:
            smc_set_param_exp(smc, config->J, config->K, config->c, config->epsilon, config->limit, config->u_max, config->flag, config->pos_eps);
            break;
        case POWER:
            // 如果有power模式，添加类似调用
            break;
        case TFSMC:
            smc_set_param_tfsmc(smc, config->J, config->K, config->p, config->q, config->beta, config->epsilon, config->limit, config->u_max, config->flag, config->pos_eps);
            break;
        case VELSMC:
            // 速度模式参数需调整
            break;
        case EISMC:
            smc_set_param_eismc(smc, config->J, config->K, config->c1, config->c2, config->epsilon, config->limit, config->u_max, config->flag, config->pos_eps);
            break;
    }
}


static uint8_t smc_idx = 0;
static Sliding *smc_instances[SMC_MAX_CNT];

Sliding *smc_register(SlidingConfig *config) {
    if (!config || smc_idx >= SMC_MAX_CNT) return NULL;
    Sliding *smc = (Sliding *)pvPortMalloc(sizeof(Sliding));  // 动态分配内存
    if (!smc) return NULL;
    memset(smc, 0, sizeof(Sliding));  // 清空内存
    smc_init_with_config(smc, config);  // 使用配置初始化
    smc_instances[smc_idx++] = smc;  // 添加到实例数组
    return smc;
}