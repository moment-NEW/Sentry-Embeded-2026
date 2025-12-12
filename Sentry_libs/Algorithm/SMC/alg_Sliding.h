#ifndef SLIDING_H
#define SLIDING_H



#include <math.h>
#include "stdint.h"

#include "arm_math.h"


#include "cmsis_os.h"


#ifdef __cplusplus
extern "C" {
#endif

#define SAMPLE_PERIOD 0.002f
#define V_ERROR_INTEGRAL_MAX 2000.0f
#define P_ERROR_INTEGRAL_MAX 2000.0f

typedef enum {
    EXPONENT = 0,
    POWER,
    TFSMC,
    VELSMC,
    EISMC
} Rmode;

typedef struct {
    float tar_now;
    float tar_last;
    float tar_differential;
    float tar_differential_last;
    float tar_differential_second;

    float pos_get;
    float vol_get;

    float p_error;
    float v_error;

    float p_error_integral;
    float v_error_integral;

    float pos_error_eps;
    float vol_error_eps;
    float error_last;
} RError;

typedef struct {
    float J;
    float K;
    float c;

    float c1;
    float c2;

    float p;
    float q;
    float beta;
    float epsilon;
} SlidingParam;

typedef struct {
    float J;
    float K;
    float c;
    float c1;
    float c2;
    float p;
    float q;
    float beta;
    float epsilon;
    float limit;
    float u_max;
    float pos_eps;
    Rmode flag;
} SlidingConfig;

typedef struct {
    float u;
    float s;

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max;
    Rmode flag;
    float limit;
} Sliding;

Sliding *smc_register(SlidingConfig *config);
void smc_init(Sliding *smc);

void smc_set_param_exp(Sliding *smc, float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_eps);
void smc_set_param_tfsmc(Sliding *smc, float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_eps);
void smc_set_param_eismc(Sliding *smc, float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_eps);

void smc_error_update_pos(Sliding *smc, float target, float pos_now, float vol_now);
void smc_error_update_vel(Sliding *smc, float target, float vol_now);

void smc_clear(Sliding *smc);
void smc_integral_clear(Sliding *smc);

float smc_calculate(Sliding *smc);

float smc_get_out(const Sliding *smc);
void smc_set_out(Sliding *smc, float out);

const Sliding *smc_get_state_const(const Sliding *smc);
void smc_init_with_config(Sliding *smc, const SlidingConfig *config);
#ifdef __cplusplus
}
#endif
#endif // SLIDING_H