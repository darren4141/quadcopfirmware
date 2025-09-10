#pragma once
#include "esp_http_server.h"

#define TAG "SERVER"
#define LED_GPIO 15          // onboard LED is active-LOW
#define RING_N   20          // samples used for zeroing average

// Push newest YPR sample into server's ring buffer
void imu_push_ypr_to_server(const float yaw, const float pitch, const float roll);

void pwm_push_to_server(const int *vals, const int mode);

void wifi_ap_start(void);

esp_err_t server_init(void);

esp_err_t ws_handler(httpd_req_t *req);
