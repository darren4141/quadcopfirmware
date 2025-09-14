#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/i2c.h"
#include "mpu6050_dmp.h"
#include "server.h"

uint8_t mpu_addr = 0;

static const char *TAGMPU = "MPU6050_DMP";

//MPU6050 I2C helpers
static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, addr, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val){
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, val, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t i2c_write_bytes(uint8_t addr, uint8_t reg, const uint8_t * data, size_t len){
    uint8_t tmp[1] = {reg};
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, tmp, 1, true);
    i2c_master_write(cmd, (uint8_t*)data, len, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

esp_err_t mpu_detect(void){
    uint8_t who = 0;
    if(i2c_read_reg(MPU_ADDR_LOW, MPU_REG_WHO_AM_I, &who) == ESP_OK && who == 0x68){
        mpu_addr = MPU_ADDR_LOW;
        return ESP_OK;
    }
    if(i2c_read_reg(MPU_ADDR_HIGH, MPU_REG_WHO_AM_I, &who) == ESP_OK && who == 0x68){
        mpu_addr = MPU_ADDR_HIGH;
        return ESP_OK;
    }
    return ESP_FAIL;
}

esp_err_t mpu_basic_init(void) {

    //DEVICE_RESET
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_PWR_MGMT_1, 0x80));
    vTaskDelay(pdMS_TO_TICKS(100));

    //CLKSEL to PLL with X-Gyro reference
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_PWR_MGMT_1, 0x01));
    vTaskDelay(pdMS_TO_TICKS(10));

    // SIG_COND_RESET - reset all DMP/FIFO/I2C_MST
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, 0x00));

    // Reset gyro
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_SIG_PATH_RST, 0x07));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Config + sample rate (safe defaults)

    //DLFP
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_CONFIG, 0x03));
    //Sample rate divider
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_SMPLRT_DIV, 4));
    //Gyro config
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_GYRO_CONFIG, 0x00));
    //Accel config
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_ACCEL_CONFIG, 0x00));
    return ESP_OK;
}


//MPU6050 DMP memory helpers
static esp_err_t mpu_set_memory_bank(uint8_t bank) {
    return i2c_write_reg(mpu_addr, MPU_REG_BANK_SEL, bank);
}
static esp_err_t mpu_set_memory_start_address(uint8_t addr) {
    return i2c_write_reg(mpu_addr, MPU_REG_MEM_START_ADDR, addr);
}

// Write a single byte to MEM_R_W at current bank/start address
static esp_err_t mpu_mem_write_byte(uint8_t val) {
    return i2c_write_bytes(mpu_addr, MPU_REG_MEM_R_W, &val, 1);
}
// Read a single byte from MEM_R_W at current bank/start address
static esp_err_t mpu_mem_read_byte(uint8_t *val) {
    uint8_t reg = MPU_REG_MEM_R_W;
    return i2c_master_write_read_device(I2C_PORT, mpu_addr, &reg, 1, val, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

// Ultra-conservative writer with verify + logging
static esp_err_t mpu_write_memory_block(const uint8_t *data, uint16_t len, uint8_t bank, uint8_t addr) {
    uint16_t i = 0;
    uint8_t curBank = bank;
    uint8_t curAddr = addr;

    while (i < len) {
        ESP_ERROR_CHECK(mpu_set_memory_bank(curBank));
        ESP_ERROR_CHECK(mpu_set_memory_start_address(curAddr));

        // Write one byte
        esp_err_t err = mpu_mem_write_byte(data[i]);
        if (err != ESP_OK) {
            ESP_LOGE(TAGMPU, "MEM write timeout @ i=%u bank=%u addr=%u (val=0x%02X)", i, curBank, curAddr, data[i]);
            return err;
        }

        // Verify
        uint8_t rd = 0;
        ESP_ERROR_CHECK(mpu_set_memory_bank(curBank));
        ESP_ERROR_CHECK(mpu_set_memory_start_address(curAddr));
        err = mpu_mem_read_byte(&rd);
        if (err != ESP_OK || rd != data[i]) {
            ESP_LOGE(TAGMPU, "Verify fail @ i=%u bank=%u addr=%u wrote=0x%02X read=0x%02X err=%s",
                     i, curBank, curAddr, data[i], rd, esp_err_to_name(err));
            return (err != ESP_OK) ? err : ESP_FAIL;
        }

        // Progress log every 16 bytes
        if ((i & 0x0F) == 0) {
            ESP_LOGI(TAGMPU, "DMP upload %4u / %u (bank=%u addr=%u)", i, len, curBank, curAddr);
        }

        // Advance
        i++;
        curAddr++;
        if (curAddr == 0) { // wrapped past 0xFF
            curBank++;
        }

        // Tiny breather helps some clones
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    ESP_LOGI(TAGMPU, "DMP upload done (%u bytes).", len);
    return ESP_OK;
}

static esp_err_t mpu_write_dmp_config_set(const uint8_t *cfg, uint16_t cfgSize) {
    uint16_t i = 0;
    while (i + 3 <= cfgSize) {
        uint8_t bank   = cfg[i++];
        uint8_t offset = cfg[i++];
        uint8_t length = cfg[i++];

        if (length > 0) {
            if (i + length > cfgSize) return ESP_ERR_INVALID_SIZE;
            ESP_LOGI(TAGMPU, "DMP cfg bank=%u offset=%u len=%u", bank, offset, length);
            ESP_ERROR_CHECK(mpu_write_memory_block(&cfg[i], length, bank, offset));
            i += length;
        } else {
            // SPECIAL INSTRUCTION: next byte is a command
            if (i >= cfgSize) return ESP_ERR_INVALID_SIZE;
            uint8_t special = cfg[i++];
            ESP_LOGI(TAGMPU, "DMP special=0x%02X", special);
            switch (special) {
                case 0x01: {
                    // Enable DMP-related interrupts (same as MotionApps v2.0)
                    // INT_ENABLE = 0x32 (FIFO overflow + DMP interrupt)
                    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_INT_ENABLE, 0x32));
                    break;
                }
                default:
                    ESP_LOGW(TAGMPU, "Unknown DMP special 0x%02X (skipping)", special);
                    break;
            }
        }
    }
    return ESP_OK;
}

// ---------- DMP init ----------
esp_err_t mpu_dmp_initialize(void) {
    // Make sure DMP and FIFO are off/reset
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, 0x00));
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_FIFO_RST | BIT_I2C_MST_RST | BIT_SIG_COND_RST));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Upload DMP firmware
    ESP_LOGI(TAGMPU, "Uploading DMP (%u bytes)", (unsigned)dmpMemoryLength);
    ESP_ERROR_CHECK(mpu_write_memory_block(dmpMemory, dmpMemoryLength, 0x00, 0x00));

    // Apply DMP config
    ESP_LOGI(TAGMPU, "Applying DMP config (%u bytes)", (unsigned)dmpConfigLength);
    ESP_ERROR_CHECK(mpu_write_dmp_config_set(dmpConfig, dmpConfigLength));

    // Apply DMP updates (if present)
    if (dmpUpdatesLength) {
        ESP_LOGI(TAGMPU, "Applying DMP updates (%u bytes)", (unsigned)dmpUpdatesLength);
        ESP_ERROR_CHECK(mpu_write_dmp_config_set(dmpUpdates, dmpUpdatesLength));
    }

    // Set DMP rate/output (commonly via DMP_CFG_1 / DMP_CFG_2 or via config set)
    // These magic values mirror MotionApps v2 defaults (works with 42-byte packet)
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_DMP_CFG_1, 0x03));
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_DMP_CFG_2, 0x00));

    // Enable FIFO for DMP
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_FIFO_EN, 0x00)); // DMP writes to FIFO, so no raw sensors here
    // Clear FIFO
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_FIFO_RST));
    vTaskDelay(pdMS_TO_TICKS(5));
    // Enable DMP + FIFO
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_EN));

    // Optionally enable DMP interrupt
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_INT_ENABLE, BIT_DMP_INT_EN));

    // Flush once more
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_EN | BIT_FIFO_RST));
    vTaskDelay(pdMS_TO_TICKS(5));
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_EN));

    return ESP_OK;
}

// ---------- FIFO read helpers ----------
static uint16_t mpu_fifo_count(void) {
    uint8_t hi=0, lo=0;
    i2c_read_reg(mpu_addr, MPU_REG_FIFO_COUNTH, &hi);
    i2c_read_reg(mpu_addr, MPU_REG_FIFO_COUNTL, &lo);
    return ((uint16_t)hi << 8) | lo;
}
static esp_err_t mpu_fifo_read(uint8_t *dst, size_t n) {
    // FIFO_R_W supports burst reads without sending a new reg each byte
    return i2c_master_write_read_device(I2C_PORT, mpu_addr, (uint8_t[]){MPU_REG_FIFO_RW}, 1, dst, n, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

// ---------- Math ----------
static inline int32_t be32(const uint8_t *p) {
    return (int32_t)((uint32_t)p[0] << 24 | (uint32_t)p[1] << 16 | (uint32_t)p[2] << 8 | (uint32_t)p[3]);
}

static void quat_to_ypr(float qw, float qx, float qy, float qz, float *ypr) {
    // MotionApps convention (Tait-Bryan Z-Y-X): yaw(Z), pitch(Y), roll(X)
    float ys = atan2f(2.0f*qx*qy - 2.0f*qw*qz, 2.0f*qw*qw + 2.0f*qx*qx - 1.0f);
    float ps = asinf(2.0f*qw*qy + 2.0f*qx*qz);
    float rs = atan2f(2.0f*qw*qx - 2.0f*qy*qz, 2.0f*qw*qw + 2.0f*qz*qz - 1.0f);
    const float RAD2DEG = 57.2957795f;
    ypr[0] = ys * RAD2DEG;
    ypr[1] = ps * RAD2DEG;
    ypr[2] = rs * RAD2DEG;
}

void dmp_task_polling(void *arg) {
    // Ensure DMP+FIFO are enabled and FIFO is clean
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_FIFO_EN, 0x00)); // DMP writes packets itself
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_RST));
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_EN));

    // (Optional) disable INT if you don't use it
    // ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_INT_ENABLE, 0x00));

    uint8_t buf[DMP_PACKET_SIZE * MAX_BATCH_PACKETS];

    TickType_t next = xTaskGetTickCount();
    TickType_t period = pdMS_TO_TICKS(POLL_MS);
    if (period == 0) period = 1;     // <-- ensure non-zero ticks
    
    static uint32_t counter = 0;

    for (;;) {
        next += period;
        counter++;

        // Read everything available in batches
        for (;;) {
            uint16_t cnt = mpu_fifo_count();
            if (cnt < DMP_PACKET_SIZE) break;

            // Clamp to whole packets and our buffer size
            uint16_t n_to_read = (cnt / DMP_PACKET_SIZE) * DMP_PACKET_SIZE;
            if (n_to_read > sizeof(buf)) n_to_read = sizeof(buf);

            if (mpu_fifo_read(buf, n_to_read) != ESP_OK) {
                // If a transient error, bail to next poll tick
                break;
            }

            // Parse packets; only print the LAST one to avoid serial bottleneck
            for (uint16_t i = 0; i < n_to_read; i += DMP_PACKET_SIZE) {
                const uint8_t *p = &buf[i];
                int32_t q0i = (int32_t)((uint32_t)p[0]<<24 | (uint32_t)p[1]<<16 | (uint32_t)p[2]<<8 | p[3]);
                int32_t q1i = (int32_t)((uint32_t)p[4]<<24 | (uint32_t)p[5]<<16 | (uint32_t)p[6]<<8 | p[7]);
                int32_t q2i = (int32_t)((uint32_t)p[8]<<24 | (uint32_t)p[9]<<16 | (uint32_t)p[10]<<8 | p[11]);
                int32_t q3i = (int32_t)((uint32_t)p[12]<<24 | (uint32_t)p[13]<<16 | (uint32_t)p[14]<<8 | p[15]);

                const float Q30 = 1073741824.0f;
                float qw = q0i / Q30;
                float qx = q1i / Q30;
                float qy = q2i / Q30;
                float qz = q3i / Q30;

                if (i + DMP_PACKET_SIZE >= n_to_read && counter % 10 == 0) {
                    float ypr[3];
                    quat_to_ypr(qw, qx, qy, qz, ypr);
                    printf("YPR[deg] %.2f, %.2f, %.2f\n", ypr[0], ypr[1], ypr[2]);
                }
            }
        }

        // Overflow guard (rare if you keep up)
        uint16_t cnt = mpu_fifo_count();
        if (cnt > 1000) {
            i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_EN | BIT_FIFO_RST);
            vTaskDelay(pdMS_TO_TICKS(2));
            i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, BIT_DMP_EN | BIT_FIFO_EN);
        }

        vTaskDelayUntil(&next, period);
    }
}

// ----- Raw (no DMP) YPR task with complementary filter -----

// Choose full-scale ranges to match your mpu_basic_init():
// Here we assume ±2g accel (0x00) and ±250 dps gyro (0x00).
#define ACCEL_SENS_2G     16384.0f   // LSB per g
#define GYRO_SENS_250DPS  131.0f     // LSB per deg/s
#define RAD2DEG           57.2957795f

// Burst-read 14 bytes: Ax,Ay,Az,Temp,Gx,Gy,Gz
static esp_err_t mpu_read_accel_gyro_raw(int16_t *ax, int16_t *ay, int16_t *az,
                                         int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t reg = MPU_REG_ACCEL_XOUT_H;
    uint8_t buf[14];
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, mpu_addr,
                                                 &reg, 1, buf, sizeof(buf),
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return err;

    *ax = (int16_t)((buf[0] << 8) | buf[1]);
    *ay = (int16_t)((buf[2] << 8) | buf[3]);
    *az = (int16_t)((buf[4] << 8) | buf[5]);
    // temp: buf[6..7] (unused)
    *gx = (int16_t)((buf[8] << 8) | buf[9]);
    *gy = (int16_t)((buf[10] << 8) | buf[11]);
    *gz = (int16_t)((buf[12] << 8) | buf[13]);
    return ESP_OK;
}

// Quick stationary gyro bias calibration (keep still)
static esp_err_t mpu_calibrate_gyro_bias(int samples, float *bx, float *by, float *bz)
{
    int32_t sx = 0, sy = 0, sz = 0;
    int16_t ax, ay, az, gx, gy, gz;
    for (int i = 0; i < samples; i++) {
        esp_err_t err = mpu_read_accel_gyro_raw(&ax, &ay, &az, &gx, &gy, &gz);
        if (err != ESP_OK) return err;
        sx += gx; sy += gy; sz += gz;
        vTaskDelay(pdMS_TO_TICKS(5)); // ~200 Hz sampling during biasing
    }
    *bx = (float)sx / samples;
    *by = (float)sy / samples;
    *bz = (float)sz / samples;
    return ESP_OK;
}

void ypr_task_polling(void *arg)
{
    // If DMP is enabled elsewhere, consider disabling it first to avoid confusion
    // (not strictly necessary, since we don’t touch FIFO here).

    // 1) Gyro bias (keep board still!)
    float gbx=0, gby=0, gbz=0;
    if (mpu_calibrate_gyro_bias(400, &gbx, &gby, &gbz) != ESP_OK) {
        ESP_LOGW(TAGMPU, "Gyro bias calibration failed, continuing without bias");
        gbx = gby = gbz = 0;
    }
    ESP_LOGI(TAGMPU, "Gyro bias: bx=%.2f by=%.2f bz=%.2f (LSB)", gbx, gby, gbz);

    // 2) Filter state
    float yaw=0.f, pitch=0.f, roll=0.f;     // degrees
    const float alpha = 0.98f;              // complementary blend
    const int   loop_hz = 200;              // loop rate
    TickType_t period = pdMS_TO_TICKS(1000/loop_hz);
    if(period == 0){
        period = 1;
    }
    TickType_t next = xTaskGetTickCount();

    int16_t ax_i, ay_i, az_i, gx_i, gy_i, gz_i;
    int push_div = 4;   // push YPR every 4 loops => 50 Hz to server
    int push_cnt = 0;

    for (;;) {
        next += period;

        if (mpu_read_accel_gyro_raw(&ax_i, &ay_i, &az_i, &gx_i, &gy_i, &gz_i) == ESP_OK) {
            // Convert to physical units
            float ax = (float)ax_i / ACCEL_SENS_2G;        // g
            float ay = (float)ay_i / ACCEL_SENS_2G;        // g
            float az = (float)az_i / ACCEL_SENS_2G;        // g

            float gx = ((float)gx_i - gbx) / GYRO_SENS_250DPS; // deg/s
            float gy = ((float)gy_i - gby) / GYRO_SENS_250DPS; // deg/s
            float gz = ((float)gz_i - gbz) / GYRO_SENS_250DPS; // deg/s

            // dt in seconds (from tick period)
            const float dt = (float)period / 1000.0f; // ms -> s

            // Accel tilt estimate (degrees)
            // roll = atan2(ay, az); pitch = atan2(-ax, sqrt(ay^2+az^2))
            float roll_acc  = atan2f(ay, az) * RAD2DEG;
            float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD2DEG;

            // Integrate gyro
            roll  += gx * dt;
            pitch += gy * dt;
            yaw   += gz * dt;   // will drift without magnetometer

            // Complementary blend
            roll  = alpha * roll  + (1.f - alpha) * roll_acc;
            pitch = alpha * pitch + (1.f - alpha) * pitch_acc;
            // yaw: gyro only (you can add magnetometer fusion later)

            if (++push_cnt >= push_div) {
                push_cnt = 0;
                float ypr[3] = { yaw, pitch, roll };
            }
        }

        // Keep loop timing steady
        vTaskDelayUntil(&next, period);
    }
}

// helper: (re)initialize the I2C driver at a given speed
static esp_err_t i2c_bus_reinit(uint32_t hz)
{
    // If driver already installed, remove it first
    i2c_driver_delete(I2C_PORT);

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = hz,
        .clk_flags = 0
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));
    return i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

esp_err_t mpu_dmp_init(void)
{
    // 1) I2C @ 100 kHz for robust DMP upload
    ESP_ERROR_CHECK(i2c_bus_reinit(I2C_FREQ_HZ_INIT));

    // 2) Detect + basic init
    ESP_ERROR_CHECK(mpu_detect());
    ESP_LOGI(TAGMPU, "MPU6050 detected at 0x%02X", mpu_addr);
    ESP_ERROR_CHECK(mpu_basic_init());

    // 3) DMP init/upload
    ESP_ERROR_CHECK(mpu_dmp_initialize());
    ESP_LOGI(TAGMPU, "DMP ready; packet size = %d", DMP_PACKET_SIZE);

    // 4) Switch I2C bus to 400 kHz for runtime
    ESP_ERROR_CHECK(i2c_bus_reinit(I2C_FREQ_HZ_RUN));

    return ESP_OK;
}

esp_err_t mpu_init(uint32_t bus_hz)
{
    // 1) I2C at requested speed (e.g., 400 kHz)
    ESP_ERROR_CHECK(i2c_bus_reinit(bus_hz));

    // 2) Detect + basic init
    ESP_ERROR_CHECK(mpu_detect());
    ESP_LOGI(TAGMPU, "MPU6050 detected at 0x%02X", mpu_addr);
    ESP_ERROR_CHECK(mpu_basic_init());

    // 3) Ensure DMP/FIFO/INT are OFF so raw reads are clean
    // USER_CTRL: clear DMP, FIFO; reset paths
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_USER_CTRL, 0x00));
    // Disable FIFO
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_FIFO_EN, 0x00));
    // Disable DMP interrupt
    ESP_ERROR_CHECK(i2c_write_reg(mpu_addr, MPU_REG_INT_ENABLE, 0x00));

    return ESP_OK;
}