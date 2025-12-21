/**
 * @file Sliding_C_Interface.h
 * @brief C interface for SMC controller (for use in C code)
 */

#ifndef SLIDING_C_INTERFACE_H
#define SLIDING_C_INTERFACE_H

#ifdef __cplusplus
extern "C" {
#endif

// Pitch SMC interface
void smc_pitch_init(void);
void smc_pitch_set_param_eismc(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, float pos_eps);
void smc_pitch_error_update(float target, float pos_now, float vol_now);
float smc_pitch_calculate(void);
float smc_pitch_get_out(void);
float smc_pitch_get_s(void);
void smc_pitch_clear(void);
void smc_pitch_integral_clear(void);

// Roll SMC interface
void smc_roll_init(void);
void smc_roll_set_param_eismc(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, float pos_eps);
void smc_roll_error_update(float target, float pos_now, float vol_now);
float smc_roll_calculate(void);
float smc_roll_get_out(void);
float smc_roll_get_s(void);
void smc_roll_clear(void);
void smc_roll_integral_clear(void);

// Config binding for monitoring in Keil debugger
typedef struct {
    int flag;      // Rmode: EXPONENT=0, POWER=1, TFSMC=2, VELSMC=3, EISMC=4
    float J;
    float K;
    float c;       // For EXPONENT/POWER/VELSMC
    float c1;      // For EISMC
    float c2;      // For EISMC
    float epsilon;
    float limit;
    float u_max;
    float pos_eps;
} SMC_Config;

// Bind returns a pointer to a live config struct you can edit in Keil
SMC_Config* smc_pitch_config_bind(void);


// Apply reads the bound config and updates internal smc.param safely
void smc_pitch_apply_config(void);


#ifdef __cplusplus
}
#endif

#endif // SLIDING_C_INTERFACE_H
