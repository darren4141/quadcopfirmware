#include <stdint.h>
#include <string.h>
#include "esp_err.h"

#define MAX_NUM_ELEMENTS 3
#define MAX_NUM_VALS 4
#define NAME_LENGTH 5
#define LOG_TASK_FREQ_MS 200

extern volatile int size;

typedef struct{


}log_output;

esp_err_t log_init();
void log_add_element_f(char *name, float *vals, size_t num_vals, int index);
void log_update_vals_f(uint8_t index, float *vals, size_t num_vals);
void log_add_element_i(char *name, float *vals, size_t num_vals, int index);
void log_update_vals_i(uint8_t index, float *vals, size_t num_vals);
void log_output_task(void *arg);