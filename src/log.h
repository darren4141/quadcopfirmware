#include <stdint.h>
#include <string.h>

#define MAX_NUM_ELEMENTS 3
#define MAX_NUM_VALS 4
#define NAME_LENGTH 4

extern volatile int size;

typedef struct{


}log_output;

void log_init();
void log_add_element(char *name, float *vals, size_t num_vals, uint8_t index);
void log_update_vals(uint8_t index, float *vals, size_t num_vals);
void log_output_task();