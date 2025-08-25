#include <stdint.h>
#include "esp_log.h"
#include "esp_err.h"
#include "i2c.h"
#include "driver/i2c.h"

esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val){
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(I2C_PORT, addr, buf, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val){
    return i2c_master_write_read_device(I2C_PORT, addr, &reg, 1, val, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

esp_err_t i2c_write_bytes(uint8_t addr, uint8_t reg, const uint8_t * data, size_t len){
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