#include "pid.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

void PIDinitialize(pid_controller controller){
    controller.prev_time_us = esp_timer_get_time();
    controller.eI = 0;
}

void setTarget(pid_controller controller, float new_target){
    controller.target = new_target;
}

void setUpperBound(pid_controller controller, float new_bound){
    controller.upperBound = new_bound;
}

void setEIBound(pid_controller controller, float new_maxI){
    controller.maxI = new_maxI;
}

float PIDCalculate(pid_controller controller, float current){
    float dT = (esp_timer_get_time() - controller.prev_time_us) / 1e6;
    float eP = controller.target - current;
    float eI = controller.eI + (eP * dT);
    float eD = (eP - controller.errorPrevious) / dT;

    if((controller.maxI != 0) && ABS(eI) > controller.maxI){
        if(eI > 0){
            eI = controller.maxI;
        }else{
            eI = -1 * controller.maxI;
        }
    }

    float res = (controller.kP * eP) + (controller.kI * eI) + (controller.kD * eD);

    if((controller.upperBound != 0) && ABS(res) > controller.upperBound){
        if(res > 0){
            res = controller.upperBound;
        }else{
            res = -1 * controller.upperBound;
        }
    }

    return res;

}
