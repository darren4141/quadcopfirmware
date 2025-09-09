#include "log.h"
#include "esp_log.h"

volatile int size = 0;

char name[MAX_NUM_ELEMENTS][NAME_LENGTH];
float val[MAX_NUM_ELEMENTS][MAX_NUM_VALS];
size_t num_vals[MAX_NUM_ELEMENTS];

void log_init(){
    for(int i = 0; i < MAX_NUM_ELEMENTS; i++){
        strcpy(name[i], "NULL");
    }
}

void log_add_element(char *name, float *vals, size_t num_vals, uint8_t index){
    strcpy(name[index], name);
    num_vals[index] = num_vals;

    for(int i = 0; i < num_vals; i++){
        val[index][i] = vals[i];
    }
}

void log_update_vals(uint8_t index, float *vals, size_t num_vals){
    for(int i = 0; i < num_vals; i++){
        val[index][i] = vals[i];
    }
}

void log_output_task(){


    while(1){
        for(int i = 0; i < MAX_NUM_ELEMENTS; i++){
            if(strcmp(name[i], "NULL") != 0){
                if(num_vals[i] == 3){
                    ESP_LOGI(name[i], " %f | %f | %f ", val[i][0], val[i][1], val[i][2]);
                }else if(num_vals[i] == 4){
                    ESP_LOGI(name[i], " %f | %f | %f | %f ", val[i][0], val[i][1], val[i][2], val[i][3]);
                }
            }
        }
    }
}