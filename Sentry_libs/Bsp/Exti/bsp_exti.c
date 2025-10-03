#include "bsp_exti.h"
#include <string.h>
#include "FreeRTOS.h"

static uint8_t idx = 0;
static ExitInstance_s* exit_instance[EXIT_MAX_INSTANCE];

ExitInstance_s* Exit_Register(ExitInitConfig_s *config) {
    if(config == NULL || idx >= EXIT_MAX_INSTANCE){
        return NULL;
    }
    for(uint8_t i = 0; i < idx; i++){
        if(config->pin == exit_instance[i]->pin){
            return NULL;    // 已经注册过了
        }
    }

    ExitInstance_s *instance = (ExitInstance_s *)pvPortMalloc(sizeof(ExitInstance_s));
    memset(instance, 0, sizeof(ExitInstance_s));
    if (instance == NULL) {
        vPortFree(instance);
        return NULL;
    }
    instance->pin = config->pin;
    instance->exit_callback = config->exit_callback;

    exit_instance[idx++] = instance;
    return instance;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    for (uint8_t i = 0; i < idx; i++) {
        if (exit_instance[i]->pin == GPIO_Pin) {
            if (exit_instance[i]->exit_callback != NULL) {
                exit_instance[i]->exit_callback(exit_instance[i]);
            }
            break;
        }
}
}