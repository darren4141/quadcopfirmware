#define I2C_SDA_PIN         22
#define I2C_SCL_PIN         23

#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ_INIT     100000   // use 100 kHz for DMP upload
#define I2C_FREQ_HZ_RUN      400000   // switch to 400 kHz after init
#define I2C_TIMEOUT_MS       5000     

/**
 * Writes one value to one register
 */
esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

/**
 * Reads one value from one register
 */
esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val);

/**
 * Writes a given amount of bytes to a register
 */
esp_err_t i2c_write_bytes(uint8_t addr, uint8_t reg, const uint8_t * data, size_t len);