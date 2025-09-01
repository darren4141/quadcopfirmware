#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm.h"
#include "esp_log.h"
#include "esp_err.h"

static const char *TAGPWM = "PWM";

volatile float targetPWMpct = -1.0f;
volatile float currentPWMpct = 0.0f;

static inline uint32_t duty_from_percent(uint32_t pct){
    if (pct > 100) pct = 100;
    uint32_t max_duty = (1U << LEDC_DUTY_RES) - 1U;
    // Round properly
    return (pct * max_duty + 50) / 100;
}

void incrementTargetPWMpct(float size){
    if(size < 0 && size > targetPWMpct){
        targetPWMpct = 0;
    }else{
        targetPWMpct += size;
    }
}

void setTargetPWMpct(float newTargetPWMpct){
    targetPWMpct = newTargetPWMpct;
}

float getTargetPWMpct(){
    return targetPWMpct;
}

void pwm_setter_task(){
    // Configure shared LEDC timer
    ledc_timer_config_t tcfg = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_DUTY_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

    // Configure four channels (bind each to a pin)
    ledc_channel_config_t ch[] = {
        { .gpio_num=D0_PIN, .speed_mode=LEDC_MODE, .channel=CH_D0, .timer_sel=LEDC_TIMER, .intr_type=LEDC_INTR_DISABLE, .duty=0, .hpoint=0 },
        { .gpio_num=D2_PIN, .speed_mode=LEDC_MODE, .channel=CH_D2, .timer_sel=LEDC_TIMER, .intr_type=LEDC_INTR_DISABLE, .duty=0, .hpoint=0 },
        { .gpio_num=D3_PIN, .speed_mode=LEDC_MODE, .channel=CH_D3, .timer_sel=LEDC_TIMER, .intr_type=LEDC_INTR_DISABLE, .duty=0, .hpoint=0 },
        { .gpio_num=D8_PIN, .speed_mode=LEDC_MODE, .channel=CH_D8, .timer_sel=LEDC_TIMER, .intr_type=LEDC_INTR_DISABLE, .duty=0, .hpoint=0 },
    };
    for (int i = 0; i < 4; ++i) {
        ESP_ERROR_CHECK(ledc_channel_config(&ch[i]));
    }

    // Timing
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(STEP_MS);

    while (1) {
        if(currentPWMpct == targetPWMpct){
            vTaskDelayUntil(&next, period);
            continue;        
        }

        if(targetPWMpct == -1){
            targetPWMpct = 0;
        }

        uint32_t duty = duty_from_percent(targetPWMpct);

        // Apply the same duty to all four channels
        ledc_set_duty(LEDC_MODE, CH_D0, duty);
        ledc_update_duty(LEDC_MODE, CH_D0);

        ledc_set_duty(LEDC_MODE, CH_D2, duty);
        ledc_update_duty(LEDC_MODE, CH_D2);

        ledc_set_duty(LEDC_MODE, CH_D3, duty);
        ledc_update_duty(LEDC_MODE, CH_D3);

        ledc_set_duty(LEDC_MODE, CH_D8, duty);
        ledc_update_duty(LEDC_MODE, CH_D8);

        printf("PWM DUTY: %lu\n", duty);
        ESP_LOGI(TAGPWM, "PWM DUTY: %lu\n", duty);

        // Next step
        next += period;
        vTaskDelay(pdMS_TO_TICKS(1000));

        currentPWMpct = targetPWMpct;
    }
}