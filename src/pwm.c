#include <stdio.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "pwm.h"
#include "esp_log.h"
#include "esp_err.h"
#include "pid.h"

static const char *TAGPWM = "PWM";

volatile float targetPWMpct = -1.0f;
volatile float currentPWMpct = 0.0f;

/**
 * 1 -> W       ASCEND
 * 2 -> S       DESCEND
 * 3 -> A       ROLL CW
 * 4 -> D       ROLL CCW
 * 5 -> I/K     YAW FWD/BWD
 * 6 -> J/L     PITCH FWD/BWD
 */
volatile uint8_t modeSelector = 0;

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

void setLedcWithOffset(float basePWM, float offset0, float offset1, float offset2, float offset3){
    ledc_set_duty(LEDC_MODE, CH_D0, basePWM + offset0);
    ledc_update_duty(LEDC_MODE, CH_D0);

    ledc_set_duty(LEDC_MODE, CH_D2, basePWM + offset1);
    ledc_update_duty(LEDC_MODE, CH_D2);

    ledc_set_duty(LEDC_MODE, CH_D3, basePWM + offset2);
    ledc_update_duty(LEDC_MODE, CH_D3);

    ledc_set_duty(LEDC_MODE, CH_D8, basePWM + offset3);
    ledc_update_duty(LEDC_MODE, CH_D8);

    printf("PWM0: %lu | PWM1: %lu | PWM2: %lu | PWM3: %lu\n", basePWM + offset0, basePWM + offset1, basePWM + offset2, basePWM + offset3);
    ESP_LOGI(TAGPWM, "PWM0: %lu | PWM1: %lu | PWM2: %lu | PWM3: %lu\n", basePWM + offset0, basePWM + offset1, basePWM + offset2, basePWM + offset3);
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

    pid_controller yawAdjustController;
    pid_controller pitchAdjustController;

    PID_initialize(yawAdjustController);
    PID_initialize(pitchAdjustController);

    PID_setEDeadband(yawAdjustController, 0.1);
    PID_setEDeadband(pitchAdjustController, 0.1);

    PID_setUpperBound(yawAdjustController, 100);
    PID_setUpperBound(pitchAdjustController, 100);

    while (1) {
        float yawAdjust;
        float pitchAdjust;

        if(modeSelector <= 4){
            float currentYaw = 0;
            setTarget(yawAdjustController, 0);
            yawAdjust = PID_calculate(yawAdjustController, currentYaw);

            float currentPitch = 0;
            setTarget(pitchAdjustController, 0);
            pitchAdjust = PID_calculate(pitchAdjustController, currentPitch);

        }

        if(currentPWMpct == targetPWMpct){
            vTaskDelayUntil(&next, period);
            continue;        
        }

        if(targetPWMpct == -1){
            targetPWMpct = 0;
        }

        if(modeSelector == 1){
            setLedcWithOffset(HOVER_FF + ASCEND_PWM, 
                -yawAdjust + pitchAdjust, 
                -yawAdjust - pitchAdjust, 
                yawAdjust + pitchAdjust, 
                yawAdjust - pitchAdjust);
        }else if(modeSelector == 2){
            setLedcWithOffset(HOVER_FF + DESCEND_PWM, 
                -yawAdjust + pitchAdjust, 
                -yawAdjust - pitchAdjust, 
                yawAdjust + pitchAdjust, 
                yawAdjust - pitchAdjust);
        }else if(modeSelector == 3){
            setLedcWithOffset(HOVER_FF, 
                -yawAdjust + pitchAdjust + ROLL_PWM, 
                -yawAdjust - pitchAdjust - ROLL_PWM, 
                yawAdjust + pitchAdjust + ROLL_PWM, 
                yawAdjust - pitchAdjust - ROLL_PWM); 
        }else if(modeSelector == 4){
            setLedcWithOffset(HOVER_FF, 
                -yawAdjust + pitchAdjust - ROLL_PWM, 
                -yawAdjust - pitchAdjust + ROLL_PWM, 
                yawAdjust + pitchAdjust - ROLL_PWM, 
                yawAdjust - pitchAdjust + ROLL_PWM); 
        }

        // Next step
        next += period;
        vTaskDelay(pdMS_TO_TICKS(1000));

        currentPWMpct = targetPWMpct;
    }
}