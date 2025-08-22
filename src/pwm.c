#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define D0_PIN 0
#define D2_PIN 2
#define D3_PIN 21
#define D8_PIN 19

#define STEP_MS         500                
#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_DUTY_RES   LEDC_TIMER_10_BIT   
#define LEDC_FREQ_HZ    1000                

#define CH_D0 LEDC_CHANNEL_0
#define CH_D2 LEDC_CHANNEL_1
#define CH_D3 LEDC_CHANNEL_2
#define CH_D8 LEDC_CHANNEL_3

static inline uint32_t duty_from_percent(uint32_t pct)
{
    if (pct > 100) pct = 100;
    uint32_t max_duty = (1U << LEDC_DUTY_RES) - 1U;
    // Round properly
    return (pct * max_duty + 50) / 100;
}

static void pwm_cycle_task(void *arg)
{
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

    // Duty cycle sequence: 100% -> 50% -> 0%
    const uint32_t steps_pct[] = {100, 50, 0};
    const size_t n_steps = sizeof(steps_pct)/sizeof(steps_pct[0]);

    // Timing
    TickType_t next = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(STEP_MS);

    size_t idx = 0;
    while (1) {
        uint32_t duty = duty_from_percent(steps_pct[idx]);

        // Apply the same duty to all four channels
        ledc_set_duty(LEDC_MODE, CH_D0, duty);
        ledc_update_duty(LEDC_MODE, CH_D0);

        ledc_set_duty(LEDC_MODE, CH_D2, duty);
        ledc_update_duty(LEDC_MODE, CH_D2);

        ledc_set_duty(LEDC_MODE, CH_D3, duty);
        ledc_update_duty(LEDC_MODE, CH_D3);

        ledc_set_duty(LEDC_MODE, CH_D8, duty);
        ledc_update_duty(LEDC_MODE, CH_D8);

        printf("PWM DUTY: %lu\n", steps_pct[idx]);

        // Next step
        next += period;
        vTaskDelay(pdMS_TO_TICKS(1000));
        idx = (idx + 1) % n_steps;
    }
}