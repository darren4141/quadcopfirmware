#include "log.h"
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

volatile int log_size = 0;

static const char *TAGLOG = "LOG";

static log_output_stream logs[MAX_NUM_ELEMENTS];

static SemaphoreHandle_t log_mutex;

int pow10(int n){
    if(n == 0){
        return 1;
    }

    int a = 1;
    for(int i = 0; i < n; i++){
        a *= 10;
    }
    return a;
}

esp_err_t log_init(){
    for(int i = 0; i < MAX_NUM_ELEMENTS; i++){
        logs[i].name[0] = '\0';
        logs[i].num_vals = 0;
        logs[i].floating_point_presc = 0;
        for(int j = 0; j < MAX_NUM_VALS; j++){
            logs[i].val[j] = 0;
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

void log_add_element(char *name, float *vals, size_t num_vals, int index, uint8_t floating_point_presc){
    if(index < 0 || index > MAX_NUM_ELEMENTS){
        return;
    }
    if(num_vals > MAX_NUM_ELEMENTS){
        num_vals = MAX_NUM_ELEMENTS;
    }

    log_lock();

    size_t n = strnlen(name, NAME_LENGTH - 1); //NAME_LENGTH -1 is max length
    memcpy(logs[index].name, name, n);
    logs[index].name[n] = '\0';
    logs[index].floating_point_presc = floating_point_presc;
    logs[index].num_vals = num_vals;
    
    for(int i = 0; i < num_vals; i++){
        logs[index].val[i] = (int)(vals[i] * pow10(floating_point_presc));
    }
    
    log_unlock();
    
    log_size++;
}

void log_update_vals(uint8_t index, float *vals, size_t num_vals){
    if(index > MAX_NUM_ELEMENTS){
        return;
    }
    if(num_vals > MAX_NUM_ELEMENTS){
        num_vals = MAX_NUM_ELEMENTS;
    }
    
    log_lock();
    
    logs[index].num_vals = num_vals;
    
    for(int i = 0; i < num_vals; i++){
        logs[index].val[i] = (int)(vals[i] * pow10(logs[index].floating_point_presc));
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
            if(logs[i].name[0] == '\0' || logs[i].num_vals == 0){
                continue;
            }
            int name_write_size = 0;

            if(i == 0){
                name_write_size = snprintf(line + offset, sizeof(line) - offset, "%s: ", logs[i].name);
            }else{
                name_write_size = snprintf(line + offset, sizeof(line) - offset, " || %s: ", logs[i].name);
            }

            if(name_write_size < 0 || name_write_size >= sizeof(line) - offset){
                break;
            }

            offset += name_write_size;

            for(size_t j = 0; j < logs[i].num_vals; j++){
                int data_write_size = 0;

                if(logs[i].floating_point_presc == 0){
                    data_write_size = snprintf(line + offset, sizeof(line) - offset, "| %d", logs[i].val[j]);
                }else{
                    data_write_size = snprintf(line + offset, sizeof(line) - offset, "| %d.%d", logs[i].val[j] / pow10(logs[i].floating_point_presc), logs[i].val[j] % pow10(logs[i].floating_point_presc));
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