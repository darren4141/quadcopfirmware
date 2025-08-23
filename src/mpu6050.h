//I2C config
#define I2C_SDA_PIN         22
#define I2C_SCL_PIN         23

#define I2C_PORT            I2C_NUM_0
#define I2C_FREQ_HZ_INIT     100000   // use 100 kHz for DMP upload
#define I2C_FREQ_HZ_RUN      400000   // switch to 400 kHz after init
#define I2C_TIMEOUT_MS       5000     

//MPU reg & addresses
#define MPU_ADDR_LOW            0x68
#define MPU_ADDR_HIGH           0x69
#define MPU_REG_WHO_AM_I        0x75
#define MPU_REG_PWR_MGMT_1      0x6B
#define MPU_REG_SMPLRT_DIV      0x19
#define MPU_REG_CONFIG          0x1A
#define MPU_REG_GYRO_CONFIG     0x1B
#define MPU_REG_ACCEL_CONFIG    0x1C
#define MPU_REG_INT_ENABLE      0x38
#define MPU_REG_INT_STATUS      0x3A
#define MPU_REG_USER_CTRL       0x6A
#define MPU_REG_FIFO_EN         0x23
#define MPU_REG_FIFO_COUNTH     0x72
#define MPU_REG_FIFO_COUNTL     0x73
#define MPU_REG_FIFO_RW         0x74
#define MPU_REG_SIG_PATH_RST    0x68

// DMP memory access
#define MPU_REG_BANK_SEL        0x6D
#define MPU_REG_MEM_START_ADDR  0x6E
#define MPU_REG_MEM_R_W         0x6F
#define MPU_REG_DMP_CFG_1       0x70
#define MPU_REG_DMP_CFG_2       0x71

// USER_CTRL bits
#define BIT_DMP_EN          0x80
#define BIT_FIFO_EN         0x40
#define BIT_I2C_MST_EN      0x20
#define BIT_I2C_IF_DIS      0x10
#define BIT_FIFO_RST        0x04
#define BIT_I2C_MST_RST     0x02
#define BIT_SIG_COND_RST    0x01

// INT_ENABLE bits
#define BIT_DMP_INT_EN      0x02

// DMP packet size
#define DMP_PACKET_SIZE     42

#define MAX_BATCH_PACKETS  8         // how many packets per burst read
#define POLL_MS            3         // for ~100 Hz DMP, poll every ~3–5 ms

#include "dmp_firmware.h"

static uint8_t mpu_addr = 0;

static esp_err_t i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val);

static esp_err_t i2c_read_reg(uint8_t addr, uint8_t reg, uint8_t *val);

static esp_err_t i2c_write_bytes(uint8_t addr, uint8_t reg, const uint8_t * data, size_t len);

static esp_err_t mpu_detect(void);

static esp_err_t mpu_basic_init(void);

static esp_err_t mpu_set_memory_bank(uint8_t bank);

static esp_err_t mpu_set_memory_start_address(uint8_t addr);

static esp_err_t mpu_mem_write_byte(uint8_t val);

static esp_err_t mpu_mem_read_byte(uint8_t *val);

static esp_err_t mpu_write_memory_block(const uint8_t *data, uint16_t len, uint8_t bank, uint8_t addr);

static esp_err_t mpu_write_dmp_config_set(const uint8_t *cfg, uint16_t cfgSize);

static esp_err_t mpu_dmp_initialize(void);

static uint16_t mpu_fifo_count(void);

static esp_err_t mpu_fifo_read(uint8_t *dst, size_t n);

static inline int32_t be32(const uint8_t *p);

static void quat_to_ypr(float qw, float qx, float qy, float qz, float *yaw, float *pitch, float *roll);

static void dmp_task_polling(void *arg);

