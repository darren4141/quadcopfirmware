#include "pid.h"
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_check.h"

void PID_initialize(pid_controller *controller){
    controller->prev_time_us = esp_timer_get_time();
    controller->eI = 0;

    //default settings, no upper bound, no max integral, no deadband
    controller->upperBound = 0;
    controller->maxI = 0;
    controller->eDeadband = 0;
    controller->integralDeadband = 0;
}

void PID_setConstants(pid_controller *controller, float new_kP, float new_kI, float new_kD){
    controller->kP = new_kP;
    controller->kI = new_kI;
    controller->kD = new_kD;
}

void PID_setTarget(pid_controller *controller, float new_target){
    controller->target = new_target;
}

void PID_setUpperBound(pid_controller *controller, float new_bound){
    controller->upperBound = new_bound;
}

void PID_setEIBound(pid_controller *controller, float new_maxI){
    controller->maxI = new_maxI;
}

void PID_setEDeadband(pid_controller *controller, float new_eDeadband){
    controller->eDeadband = new_eDeadband;
}

void PID_setEIDeadband(pid_controller *controller, float new_integralDeadband){
    controller->integralDeadband = new_integralDeadband;
}

float PID_calculate(pid_controller *controller, float current){
    float dT = (esp_timer_get_time() - controller->prev_time_us) / 1e6;
    float eP = controller->target - current;

    //deadband check
    if(fabs(eP) < controller->eDeadband){
        return 0;
    }

    float eI = controller->eI + (eP * dT);
    float eD = (eP - controller->errorPrevious) / dT;

    
    
    //check and obey integral bound
    if((controller->maxI != 0) && fabs(eI) > controller->maxI){
        if(eI > 0){
            eI = controller->maxI;
        }else{
            eI = -1 * controller->maxI;
        }
    }

    //check and obey integral deadband
    if(fabs(eP) < controller->integralDeadband){
        eI = 0;
    }

    //calculate PID
    float res = (controller->kP * eP) + (controller->kI * eI) + (controller->kD * eD);

    //check and obey upper bound
    if((controller->upperBound != 0) && fabs(res) > controller->upperBound){
        if(res > 0){
            res = controller->upperBound;
        }else{
            res = -1 * controller->upperBound;
        }
    }

    return res;

}
