#include "log.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

volatile int log_size = 0;

static const char *TAGLOG = "LOG";

static char log_name[MAX_NUM_ELEMENTS][NAME_LENGTH];
static float log_val_f[MAX_NUM_ELEMENTS][MAX_NUM_VALS];
static int log_val_i[MAX_NUM_ELEMENTS][MAX_NUM_VALS];
static size_t log_num_vals[MAX_NUM_ELEMENTS];
static uint8_t log_type[MAX_NUM_ELEMENTS]; //0 float, 1 int

static SemaphoreHandle_t log_mutex;

esp_err_t log_init(){
    for(int i = 0; i < MAX_NUM_ELEMENTS; i++){
        log_name[i][0] = '\0';
        log_num_vals[i] = 0;
        log_type[i] = 0;
        for(int j = 0; j < MAX_NUM_VALS; j++){
            log_val_i[i][j] = 0;
            log_val_f[i][j] = 0.0f;
        }
    }
    return ESP_OK;
}

static void inline log_lock(){
    if(log_mutex != NULL){
        xSemaphoreTake(log_mutex, portMAX_DELAY);
    }
}

static void inline log_unlock(){
    if(log_mutex != NULL){
        xSemaphoreGive(log_mutex);
    }
}

void log_add_element_f(char *name, float *vals, size_t num_vals, int index){
    if(index < 0 || index > MAX_NUM_ELEMENTS){
        return;
    }
    if(num_vals > MAX_NUM_ELEMENTS){
        num_vals = MAX_NUM_ELEMENTS;
    }

    log_lock();

    size_t n = strnlen(name, NAME_LENGTH - 1); //NAME_LENGTH -1 is max length
    memcpy(log_name[index], name, n);
    log_name[index][n] = '\0';
    log_type[index] = 0;
    log_num_vals[index] = num_vals;
    
    for(int i = 0; i < num_vals; i++){
        log_val_f[index][i] = vals[i];
    }
    
    log_unlock();
    
    log_size++;
}

void log_update_vals_f(uint8_t index, float *vals, size_t num_vals){
    if(index < 0 || index > MAX_NUM_ELEMENTS){
        return;
    }
    if(num_vals > MAX_NUM_ELEMENTS){
        num_vals = MAX_NUM_ELEMENTS;
    }
    
    log_lock();
    
    log_num_vals[index] = num_vals;
    log_type[index] = 0;
    
    for(int i = 0; i < num_vals; i++){
        log_val_f[index][i] = vals[i];
    }
    
    log_unlock();
    
}

void log_add_element_i(char *name, int *vals, size_t num_vals, int index){
    if(index < 0 || index > MAX_NUM_ELEMENTS){
        return;
    }
    if(num_vals > MAX_NUM_ELEMENTS){
        num_vals = MAX_NUM_ELEMENTS;
    }
    
    log_lock();
    
    size_t n = strnlen(name, NAME_LENGTH - 1); //NAME_LENGTH -1 is max length
    memcpy(log_name[index], name, n);
    log_name[index][n] = '\0';
    log_type[index] = 1;
    log_num_vals[index] = num_vals;
    
    for(int i = 0; i < num_vals; i++){
        log_val_i[index][i] = vals[i];
    }
    
    log_unlock();
    
    log_size++;
}

void log_update_vals_i(uint8_t index, int *vals, size_t num_vals){
    if(index < 0 || index > MAX_NUM_ELEMENTS){
        return;
    }
    if(num_vals > MAX_NUM_ELEMENTS){
        num_vals = MAX_NUM_ELEMENTS;
    }
    
    log_lock();
    
    log_num_vals[index] = num_vals;
    log_type[index] = 1;
    
    for(int i = 0; i < num_vals; i++){
        log_val_i[index][i] = vals[i];
    }

    log_unlock();

}

void log_output_task(void *arg){
    (void)arg;

    char line[256];
    line[0] = '\0';

    while(1){
        log_lock();
        
        int offset = 0;
        for(int i = 0; i < MAX_NUM_ELEMENTS; i++){
            if(log_name[i][0] == '\0' || log_num_vals[i] == 0){
                continue;
            }
            int name_write_size = 0;

            if(i == 0){
                name_write_size = snprintf(line + offset, sizeof(line) - offset, "%s: ", log_name[i]);
            }else{
                name_write_size = snprintf(line + offset, sizeof(line) - offset, " || %s: ", log_name[i]);
            }

            if(name_write_size < 0 || name_write_size >= sizeof(line) - offset){
                break;
            }

            offset += name_write_size;

            for(size_t j = 0; j < log_num_vals[i]; j++){
                int data_write_size = 0;

                if(log_type[i] == 0){
                    data_write_size = snprintf(line + offset, sizeof(line) - offset, "| %.2f", log_val_f[i][j]);
                }else if(log_type[i] == 1){
                    data_write_size = snprintf(line + offset, sizeof(line) - offset, "| %d", log_val_f[i][j]);
                }
                
                if(data_write_size < 0 || data_write_size >= (int)sizeof(line) - offset){
                    break;
                }
                
                offset += data_write_size;
                
            }

        }

        log_unlock();

        if(offset == 0){
            ESP_LOGI(TAGLOG, "nothing to log");
        }else{
            ESP_LOGI(TAGLOG, "%s", line);
        }

        vTaskDelay(pdMS_TO_TICKS(LOG_TASK_FREQ_MS));
    }

}