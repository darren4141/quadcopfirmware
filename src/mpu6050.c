#include "mpu6050.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"
#include "server.h"

static const char *TAGMPU = "MPU6050";

// --- Low-level I2C helpers ---
static inline esp_err_t i2c_write_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(port, addr, buf, 2, pdMS_TO_TICKS(50));
}

static inline esp_err_t i2c_read_reg(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *val) {
    return i2c_master_write_read_device(port, addr, &reg, 1, val, 1, pdMS_TO_TICKS(50));
}

static inline esp_err_t i2c_read_bytes(i2c_port_t port, uint8_t addr, uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(port, addr, &reg, 1, data, len, pdMS_TO_TICKS(50));
}

// --- Public API ---

esp_err_t mpu6050_i2c_init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl, uint32_t freq_hz) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda,
        .scl_io_num = scl,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = freq_hz,
        .clk_flags = 0
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(port, &cfg), TAGMPU, "param_config");
    ESP_RETURN_ON_ERROR(i2c_driver_install(port, cfg.mode, 0, 0, 0), TAGMPU, "driver_install");
    return ESP_OK;
}

static void set_scales(mpu6050_t *dev, uint8_t accel_fs_sel, uint8_t gyro_fs_sel) {
    // From datasheet
    switch (accel_fs_sel & 3) {
        case 0: dev->accel_lsb_per_g = 16384.0f; break; // ±2g
        case 1: dev->accel_lsb_per_g = 8192.0f;  break; // ±4g
        case 2: dev->accel_lsb_per_g = 4096.0f;  break; // ±8g
        default:dev->accel_lsb_per_g = 2048.0f;  break; // ±16g
    }
    switch (gyro_fs_sel & 3) {
        case 0: dev->gyro_lsb_per_dps = 131.0f;  break; // ±250
        case 1: dev->gyro_lsb_per_dps = 65.5f;   break; // ±500
        case 2: dev->gyro_lsb_per_dps = 32.8f;   break; // ±1000
        default:dev->gyro_lsb_per_dps = 16.4f;   break; // ±2000
    }
}

esp_err_t mpu6050_init(mpu6050_t *dev, i2c_port_t port, uint8_t addr) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    memset(dev, 0, sizeof(*dev));
    dev->i2c_port = port;
    dev->addr = addr ? addr : MPU6050_ADDR_DEFAULT;

    // Wake up device
    ESP_RETURN_ON_ERROR(i2c_write_reg(port, dev->addr, MPU6050_REG_PWR_MGMT_1, 0x00), TAGMPU, "wake");

    // Set DLPF = 42Hz (CONFIG=3) for stable sampling
    ESP_RETURN_ON_ERROR(i2c_write_reg(port, dev->addr, MPU6050_REG_CONFIG, 3), TAGMPU, "dlpf");

    // Gyro ±250 dps
    ESP_RETURN_ON_ERROR(i2c_write_reg(port, dev->addr, MPU6050_REG_GYRO_CONFIG, 0<<3), TAGMPU, "gyro");

    // Accel ±2g
    ESP_RETURN_ON_ERROR(i2c_write_reg(port, dev->addr, MPU6050_REG_ACCEL_CONFIG, 0<<3), TAGMPU, "accel");

    // Sample rate: with DLPF on, base=1kHz; divider=4 -> 200 Hz
    ESP_RETURN_ON_ERROR(i2c_write_reg(port, dev->addr, MPU6050_REG_SMPLRT_DIV, 4), TAGMPU, "smplrt");

    set_scales(dev, 0, 0);

    // Filter defaults
    dev->q0 = 1.0f; dev->q1 = dev->q2 = dev->q3 = 0.0f;
    dev->kp = 2.0f;   // Proportional gain (increase for faster response)
    dev->ki = 0.0f;   // Integral gain (0 to start; small value helps steady-state)
    dev->exInt = dev->eyInt = dev->ezInt = 0.0f;
    dev->last_update_us = esp_timer_get_time();
    return ESP_OK;
}

esp_err_t mpu6050_config(mpu6050_t *dev, uint8_t accel_fs_sel, uint8_t gyro_fs_sel,
                         uint8_t dlpf_cfg, uint8_t smplrt_div) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    ESP_RETURN_ON_ERROR(i2c_write_reg(dev->i2c_port, dev->addr, MPU6050_REG_ACCEL_CONFIG, (accel_fs_sel & 3) << 3), TAGMPU, "acc_cfg");
    ESP_RETURN_ON_ERROR(i2c_write_reg(dev->i2c_port, dev->addr, MPU6050_REG_GYRO_CONFIG,  (gyro_fs_sel  & 3) << 3), TAGMPU, "gyr_cfg");
    ESP_RETURN_ON_ERROR(i2c_write_reg(dev->i2c_port, dev->addr, MPU6050_REG_CONFIG,       dlpf_cfg & 7), TAGMPU, "dlpf_cfg");
    ESP_RETURN_ON_ERROR(i2c_write_reg(dev->i2c_port, dev->addr, MPU6050_REG_SMPLRT_DIV,   smplrt_div), TAGMPU, "sr_div");
    set_scales(dev, accel_fs_sel, gyro_fs_sel);
    return ESP_OK;
}

esp_err_t mpu6050_whoami(mpu6050_t *dev, uint8_t *out) {
    if (!dev || !out) return ESP_ERR_INVALID_ARG;
    return i2c_read_reg(dev->i2c_port, dev->addr, MPU6050_REG_WHO_AM_I, out);
}

esp_err_t mpu6050_read_raw(mpu6050_t *dev,
                           int16_t *ax, int16_t *ay, int16_t *az,
                           int16_t *gx, int16_t *gy, int16_t *gz) {
    if (!dev) return ESP_ERR_INVALID_ARG;
    uint8_t buf[14];
    esp_err_t err = i2c_read_bytes(dev->i2c_port, dev->addr, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) return err;
    if (ax) *ax = (int16_t)((buf[0] << 8) | buf[1]);
    if (ay) *ay = (int16_t)((buf[2] << 8) | buf[3]);
    if (az) *az = (int16_t)((buf[4] << 8) | buf[5]);
    if (gx) *gx = (int16_t)((buf[8] << 8) | buf[9]);
    if (gy) *gy = (int16_t)((buf[10] << 8) | buf[11]);
    if (gz) *gz = (int16_t)((buf[12] << 8) | buf[13]);
    return ESP_OK;
}

esp_err_t mpu6050_calibrate(mpu6050_t *dev, int samples) {
    if (!dev || samples <= 0) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAGMPU, "Calibrating... keep the sensor still (%d samples)", samples);

    double ax_sum=0, ay_sum=0, az_sum=0;
    double gx_sum=0, gy_sum=0, gz_sum=0;

    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        esp_err_t err = mpu6050_read_raw(dev, &ax, &ay, &az, &gx, &gy, &gz);
        if (err != ESP_OK) return err;

        ax_sum += (double)ax; ay_sum += (double)ay; az_sum += (double)az;
        gx_sum += (double)gx; gy_sum += (double)gy; gz_sum += (double)gz;

        vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz -> 5 ms
    }

    double ax_avg = ax_sum / samples;
    double ay_avg = ay_sum / samples;
    double az_avg = az_sum / samples;
    double gx_avg = gx_sum / samples;
    double gy_avg = gy_sum / samples;
    double gz_avg = gz_sum / samples;

    // Convert to g and dps, store biases. Assume Z sees +1g at rest.
    dev->ax_bias = (float)(ax_avg / dev->accel_lsb_per_g);
    dev->ay_bias = (float)(ay_avg / dev->accel_lsb_per_g);
    dev->az_bias = (float)(az_avg / dev->accel_lsb_per_g - 1.0); // subtract gravity

    dev->gx_bias = (float)(gx_avg / dev->gyro_lsb_per_dps);
    dev->gy_bias = (float)(gy_avg / dev->gyro_lsb_per_dps);
    dev->gz_bias = (float)(gz_avg / dev->gyro_lsb_per_dps);

    ESP_LOGI(TAGMPU, "Accel bias (g): ax=%.4f ay=%.4f az=%.4f (g includes gravity offset removed on Z)",
             dev->ax_bias, dev->ay_bias, dev->az_bias);
    ESP_LOGI(TAGMPU, "Gyro  bias (dps): gx=%.3f gy=%.3f gz=%.3f",
             dev->gx_bias, dev->gy_bias, dev->gz_bias);
    return ESP_OK;
}

// --- Mahony IMU (no magnetometer) ---
static void mahony_update_imu(mpu6050_t *d,
                              float gx_dps, float gy_dps, float gz_dps,
                              float ax_g,  float ay_g,  float az_g,
                              float dt) {
    // Convert gyro to rad/s
    const float DEG2RAD = (float)M_PI / 180.0f;
    float gx = gx_dps * DEG2RAD;
    float gy = gy_dps * DEG2RAD;
    float gz = gz_dps * DEG2RAD;

    // Normalize accelerometer
    float norm = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    if (norm > 1e-6f) {
        ax_g /= norm; ay_g /= norm; az_g /= norm;

        // Estimated gravity from quaternion
        float q0 = d->q0, q1 = d->q1, q2 = d->q2, q3 = d->q3;
        float vx = 2.0f*(q1*q3 - q0*q2);
        float vy = 2.0f*(q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // Error is cross product between measured and estimated gravity
        float ex = (ay_g * vz - az_g * vy);
        float ey = (az_g * vx - ax_g * vz);
        float ez = (ax_g * vy - ay_g * vx);

        // Integral + proportional feedback
        if (d->ki > 0.0f) {
            d->exInt += ex * d->ki * dt;
            d->eyInt += ey * d->ki * dt;
            d->ezInt += ez * d->ki * dt;
        }
        gx += d->kp * ex + d->exInt;
        gy += d->kp * ey + d->eyInt;
        gz += d->kp * ez + d->ezInt;
    }

    // Integrate quaternion rate: qDot = 0.5 * q ⊗ [0, gx, gy, gz]
    float q0 = d->q0, q1 = d->q1, q2 = d->q2, q3 = d->q3;
    float halfdt = 0.5f * dt;

    d->q0 += (-q1*gx - q2*gy - q3*gz) * halfdt;
    d->q1 += ( q0*gx + q2*gz - q3*gy) * halfdt;
    d->q2 += ( q0*gy - q1*gz + q3*gx) * halfdt;
    d->q3 += ( q0*gz + q1*gy - q2*gx) * halfdt;

    // Normalize quaternion
    float qnorm = 1.0f / sqrtf(d->q0*d->q0 + d->q1*d->q1 + d->q2*d->q2 + d->q3*d->q3);
    d->q0 *= qnorm; d->q1 *= qnorm; d->q2 *= qnorm; d->q3 *= qnorm;
}

void mpu6050_quat_to_ypr(float q0, float q1, float q2, float q3,
                         float *yaw_deg, float *pitch_deg, float *roll_deg) {
    // ZYX Euler (yaw around Z, pitch around Y, roll around X)
    float sinp = 2.0f * (q0*q2 - q3*q1);
    if (sinp > 1.0f) sinp = 1.0f;
    if (sinp < -1.0f) sinp = -1.0f;

    float yaw   = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));
    float pitch = asinf(sinp);
    float roll  = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));

    const float RAD2DEG = 180.0f / (float)M_PI;
    if (yaw_deg)   *yaw_deg   = yaw   * RAD2DEG;
    if (pitch_deg) *pitch_deg = pitch * RAD2DEG;
    if (roll_deg)  *roll_deg  = roll  * RAD2DEG;
}

esp_err_t mpu6050_update_ypr(mpu6050_t *dev, float *yaw_deg, float *pitch_deg, float *roll_deg) {
    if (!dev) return ESP_ERR_INVALID_ARG;

    int16_t axr, ayr, azr, gxr, gyr, gzr;
    esp_err_t err = mpu6050_read_raw(dev, &axr, &ayr, &azr, &gxr, &gyr, &gzr);
    if (err != ESP_OK) return err;

    // Convert to g and dps, apply biases
    float ax = (axr / dev->accel_lsb_per_g) - dev->ax_bias;
    float ay = (ayr / dev->accel_lsb_per_g) - dev->ay_bias;
    float az = (azr / dev->accel_lsb_per_g) - dev->az_bias;

    float gx = (gxr / dev->gyro_lsb_per_dps) - dev->gx_bias;
    float gy = (gyr / dev->gyro_lsb_per_dps) - dev->gy_bias;
    float gz = (gzr / dev->gyro_lsb_per_dps) - dev->gz_bias;

    // dt from esp_timer
    uint64_t now = esp_timer_get_time();
    float dt = (dev->last_update_us == 0) ? 0.005f : (now - dev->last_update_us) / 1e6f; // seconds
    if (dt < 1e-6f || dt > 0.1f) dt = 0.005f; // clamp to reasonable
    dev->last_update_us = now;

    mahony_update_imu(dev, gx, gy, gz, ax, ay, az, dt);
    mpu6050_quat_to_ypr(dev->q0, dev->q1, dev->q2, dev->q3, yaw_deg, pitch_deg, roll_deg);
    return ESP_OK;
}

void mpu6050_reset_filter(mpu6050_t *dev) {
    if (!dev) return;
    dev->q0 = 1.0f; dev->q1 = dev->q2 = dev->q3 = 0.0f;
    dev->exInt = dev->eyInt = dev->ezInt = 0.0f;
    dev->last_update_us = esp_timer_get_time();
}

void mpu6050_task(void *pvParameters) {
    if (pvParameters == NULL) {
        ESP_LOGE(TAGMPU, "mpu6050_task: NULL pvParameters");
        vTaskDelete(NULL);
        return;
    }

    mpu6050_t *imu = (mpu6050_t *)pvParameters;

    TickType_t period = pdMS_TO_TICKS(MPU6050_TASK_PERIOD_MS);
    if (period == 0) period = 1;  // guard: avoid xTaskDelayUntil assert

    TickType_t last = xTaskGetTickCount();
    float yaw, pitch, roll;

    for (;;) {
        esp_err_t err = mpu6050_update_ypr(imu, &yaw, &pitch, &roll);
        if (err == ESP_OK) {
            // ESP_LOGI("YPR", "yaw=%7.2f°, pitch=%7.2f°, roll=%7.2f°", yaw, pitch, roll);
            imu_push_ypr(yaw, pitch, roll);
        } else {
            ESP_LOGW(TAGMPU, "update_ypr failed: %s", esp_err_to_name(err));
        }
        vTaskDelayUntil(&last, period);
    }
}

esp_err_t imu_boot(mpu6050_t *imu){
    ESP_RETURN_ON_ERROR(mpu6050_i2c_init(MPU6050_I2C_PORT, I2C_SDA, I2C_SCL, MPU6050_I2C_FREQ_HZ), TAGMPU, "MPU i2c init");
    ESP_RETURN_ON_ERROR(mpu6050_init(imu, MPU6050_I2C_PORT, MPU6050_ADDR_DEFAULT), TAGMPU, "MPU init");
    
    uint8_t who = 0;
    ESP_RETURN_ON_ERROR(mpu6050_whoami(imu, &who), TAGMPU, "whoami");
    ESP_LOGI(TAGMPU, "WHO_AM_I = 0x%02X (expect 0x68)", who);

    imu->kp = 3.0f;   // 2.0–5.0 good for snappy response
    imu->ki = 0.02f;  // small integral helps gyro bias; set 0.0f if you see drift overshoot
    
    ESP_RETURN_ON_ERROR(mpu6050_calibrate(imu, 500), TAGMPU, "calibrate");

    return ESP_OK;
}