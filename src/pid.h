#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define ABS(x) (x > 0) ? x : -1 * x

typedef struct{
    float kP;
    float kI;
    float kD;
    
    float maxI;         //max value of integral term eI, default value of 0 will result in no bound
    float upperBound;   //max value of pid calculation, default value of 0 will result in no bound

    float eI; //integral term
    float eDeadband; //below a certain value of eP the pid calculation will return 0

    float target;
    float errorPrevious;

    uint64_t prev_time_us;
}pid_controller;

void PIDinitialize(pid_controller controller);

void setTarget(pid_controller controller, float new_target);

void setUpperBound(pid_controller controller, float new_bound);

void setEIBound(pid_controller controller, float new_maxI);

void setEDeadband(pid_controller controller, float new_eDeadband);

float PIDCalculate(pid_controller controller, float current);