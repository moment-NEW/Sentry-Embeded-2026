#include "dev_led.h"
#include <stdlib.h>
#include <stdbool.h>
#include "FreeRTOS.h"

static LedInstance_s* led_instances[MAX_LED_CNT] = {NULL};
static uint8_t idx = 0;

#define LED_ONCE_TIME       (LED_PERIOD_TIME/LED_TASK_TIME/10)
#define LED_CHANGE_TIME     (LED_PERIOD_TIME/LED_TASK_TIME/20)

/**
 * @brief 注册新的LED实例
 * @param led_config LED配置指针，包含GPIO端口、引脚、类型、频率和初始状态
 * @return LED_Instance* 返回LED实例指针，若注册失败则返回NULL
 */
LedInstance_s* Led_Register(LedConfig_s *led_config)
{
    if (led_config == NULL || idx >= MAX_LED_CNT) {
        return NULL;
    }
    LedInstance_s* LED_Instance = (LedInstance_s*)pvPortMalloc(sizeof(LedInstance_s));
    if (LED_Instance == NULL) {
        return NULL;
    }
    LED_Instance->port = led_config->port;
    LED_Instance->pin = led_config->pin;
    LED_Instance->state = led_config->state;
    LED_Instance->freq = led_config->freq;

    if(LED_Instance->freq > LED_FREQ_MAX_THRESHOLD){
        LED_Instance->state = LED_ON;
    }

    if(LED_Instance->state == LED_ON){
        HAL_GPIO_WritePin(LED_Instance->port, LED_Instance->pin, GPIO_PIN_SET);
    }else {
        HAL_GPIO_WritePin(LED_Instance->port, LED_Instance->pin, GPIO_PIN_RESET);
    }

    led_instances[idx++] = LED_Instance;
    return LED_Instance;
}

bool Led_Set_Mode(LedInstance_s* led, LedState_e mode, uint8_t freq){
    if (led == NULL) {
        return false;
    }
    led->state = mode;
    if(led->state == LED_BLINK){ //只有闪烁模式才需要设置频率
        led->freq = freq > LED_FREQ_MAX_THRESHOLD ? LED_FREQ_MAX_THRESHOLD : freq;
        HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
    }else if(led->state == LED_ON){
        HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_SET);
    }else if(led->state == LED_OFF || led->state == LED_BREATHING){
        HAL_GPIO_WritePin(led->port, led->pin, GPIO_PIN_RESET);
    }

    return true;
}

void Led_Control(void){
    if (idx == 0) {
        return;// 没有LED实例
    }
    static uint16_t cnt = 0;
    static uint16_t breath_cnt = 0;
    static uint8_t led_breath_down = 0.0f;
    static uint8_t led_breath_mode = 0;

    if(cnt % LED_CHANGE_TIME == 0){
        for (uint8_t i = 0; i < idx; i++) {
            if(led_instances[i]->state != LED_BLINK) {
                continue;       
            }
            if((cnt % LED_ONCE_TIME) == LED_CHANGE_TIME && (cnt / LED_ONCE_TIME) < led_instances[i]->freq) {
                HAL_GPIO_WritePin(led_instances[i]->port, led_instances[i]->pin, GPIO_PIN_SET);
            }else{
                HAL_GPIO_WritePin(led_instances[i]->port, led_instances[i]->pin, GPIO_PIN_RESET);
            }
        }
    }

    for (uint8_t i = 0; i < idx; i++) {
        if(led_instances[i]->state != LED_BREATHING) continue;
        
        if(breath_cnt % 15 == led_breath_down){
            HAL_GPIO_WritePin(led_instances[i]->port, led_instances[i]->pin, GPIO_PIN_RESET);
        } else if(breath_cnt % 15 == 0){
            HAL_GPIO_WritePin(led_instances[i]->port, led_instances[i]->pin, GPIO_PIN_SET);
        }
    }

    breath_cnt++;
    breath_cnt %= 2250;

    if(led_breath_mode == 0 && breath_cnt % 75 == 0){
        led_breath_down ++;
        if(led_breath_down == 15){
            led_breath_mode = 1;
        }
    } else if(led_breath_mode == 1 && breath_cnt % 75 == 0){
        led_breath_down --;
        if(led_breath_down == 0){
            led_breath_mode = 0;
        }
    }

    cnt++;
    cnt %= (LED_PERIOD_TIME/LED_TASK_TIME);
}


