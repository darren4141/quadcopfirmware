#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define TAG "SERVER"
#define LED_GPIO 15          // onboard LED is active-LOW
#define RING_N   20          // samples used for zeroing average

// Push newest YPR sample into server's ring buffer
void imu_push_ypr(const float ypr[3]);

void wifi_ap_start(void);

esp_err_t server_init(void);

#ifdef __cplusplus
}
#endif
