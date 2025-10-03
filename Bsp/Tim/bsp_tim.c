#include "bsp_tim.h"
#include <string.h>
#include "FreeRTOS.h"

static uint8_t idx = 0;
static TimInstance_s* tim_instance[TIM_MAX_INSTANCE];
TimInstance_s* Tim_Register(TimInitConfig_s *config) {
    if(config == NULL || config->htim == NULL || idx >= TIM_MAX_INSTANCE) {
        return NULL;        // 参数错误或已注册实例数超过最大限制
    }
    for(uint8_t i = 0; i < idx; i++){
        if(config->htim == tim_instance[i]->htim){
            return NULL;    // 已经注册过了
        }
    }

    TimInstance_s *instance = (TimInstance_s *)pvPortMalloc(sizeof(TimInstance_s));
    memset(instance, 0, sizeof(TimInstance_s));
    if (instance == NULL) {
        vPortFree(instance);
        return NULL;
    }

    instance->htim = config->htim;
    instance->tim_callback = config->tim_callback;

    HAL_TIM_Base_Start_IT(instance->htim);  // 启动定时器中断

    tim_instance[idx++] = instance;
    return instance;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
/***********************把main.c中的HAL_TIM_PeriodElapsedCallback内容替换下面内容***************************/
  if (htim->Instance == TIM14){
    HAL_IncTick();
    return;
  }
/***********************把main.c中的HAL_TIM_PeriodElapsedCallback内容替换上面内容***************************/
  
    for(uint8_t i = 0; i < idx; i++) {
        if(tim_instance[i]->htim == htim) {
            if(tim_instance[i]->tim_callback != NULL) {
                tim_instance[i]->tim_callback(tim_instance[i]);
            }
            break;
        }
    }
}