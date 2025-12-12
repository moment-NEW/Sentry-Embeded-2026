#include "alg_Sliding.h"


#define SMC_MAX_CNT 6
static float smc_signal(float s) {
    if (s > 0.0f) return 1.0f;
    if (s < 0.0f) return -1.0f;
    return 0.0f;
}

static float smc_sat(const Sliding *smc, float s) {
    float y = s / smc->param.epsilon;
    if (fabsf(y) <= smc->limit)
        return y;
    return smc_signal(y);
}

static void smc_out_continuation(Sliding *smc) {
    if (smc->param.K != 0.0f && smc->param.c2 != 0.0f) {
        smc->error.p_error_integral =
            (smc->param_last.K / smc->param.K) * (smc->param_last.c2 / smc->param.c2) * smc->error.p_error_integral;
        smc->error.v_error_integral =
            (smc->param_last.K / smc->param.K) * (smc->param_last.c / smc->param.c) * smc->error.v_error_integral;
    }
    smc->param_last = smc->param;
}

void smc_init(Sliding *smc) {
    smc=pvPortMalloc(sizeof(Sliding));
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
    smc->error.tar_differential_last = smc->error.tar_differential;
}

void smc_error_update_vel(Sliding *smc, float target, float vol_now) {
    if (!smc) return;
    smc->error.tar_now = target;
    smc->error.tar_differential = (smc->error.tar_now - smc->error.tar_last) / SAMPLE_PERIOD;

    smc->error.v_error = vol_now - smc->error.tar_now;
    smc->error.v_error_integral += smc->error.v_error * SAMPLE_PERIOD;

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

    switch (smc->flag) {
        case EXPONENT:
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }
            smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * ((-smc->param.c * smc->error.v_error) - smc->param.K * smc->s - smc->param.epsilon * fun);
            break;

        case POWER:
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }
            smc->s = smc->param.c * smc->error.p_error + smc->error.v_error;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * ((-smc->param.c * smc->error.v_error)
                - smc->param.K * smc->s
                - smc->param.K * powf(fabsf(smc->s), smc->param.epsilon) * fun);
            break;

        case TFSMC: {
            static float pos_pow;
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }
            pos_pow = powf(fabsf(smc->error.p_error), smc->param.q / smc->param.p);
            if (smc->error.p_error < 0.0f) pos_pow = -pos_pow;

            smc->s = smc->param.beta * pos_pow + smc->error.v_error;
            fun = smc_sat(smc, smc->s);

            if (smc->error.p_error != 0.0f) {
                u = smc->param.J * (
                    smc->error.tar_differential_second
                    - smc->param.K * smc->s
                    - smc->param.epsilon * fun
                    - smc->error.v_error * ((smc->param.q * smc->param.beta) * pos_pow) / (smc->param.p * smc->error.p_error)
                );
            } else {
                u = 0.0f;
            }
            break;
        }

        case VELSMC:
            smc->s = smc->error.v_error + smc->param.c * smc->error.v_error_integral;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * (smc->error.tar_differential
                - (smc->param.c * smc->error.v_error)
                - smc->param.K * smc->s
                - smc->param.epsilon * fun);
            break;

        case EISMC:
            if (fabsf(smc->error.p_error) - smc->error.pos_error_eps < 0.0f) {
                smc->error.p_error = 0.0f;
                return 0.0f;
            }
            smc->s = smc->param.c1 * smc->error.p_error + smc->error.v_error + smc->param.c2 * smc->error.p_error_integral;
            fun = smc_sat(smc, smc->s);
            u = smc->param.J * (
                (-smc->param.c1 * smc->error.v_error)
                - smc->param.c2 * smc->error.p_error
                - smc->param.K * smc->s
                - smc->param.epsilon * fun
            );
            break;
    }

    smc->error.error_last = smc->error.p_error;

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