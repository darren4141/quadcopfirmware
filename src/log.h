#include <stdint.h>
#include <string.h>
#include "esp_err.h"

#define MAX_NUM_ELEMENTS 3
#define MAX_NUM_VALS 4
#define NAME_LENGTH 5
#define LOG_TASK_FREQ_MS 200

extern volatile int size;

typedef struct{
    char name[NAME_LENGTH];
    int val[MAX_NUM_VALS];
    size_t num_vals;
    uint8_t floating_point_presc;
}log_output_stream;

esp_err_t log_init();
void log_add_element(char *name, float *vals, size_t num_vals, int index, uint8_t floating_point_presc);
void log_update_vals(uint8_t index, float *vals, size_t num_vals);
void log_output_task(void *arg);