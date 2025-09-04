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

void PID_initialize(pid_controller controller);

void PID_setTarget(pid_controller controller, float new_target);

void PID_setUpperBound(pid_controller controller, float new_bound);

void PID_setEIBound(pid_controller controller, float new_maxI);

void PID_setEDeadband(pid_controller controller, float new_eDeadband);

float PID_calculate(pid_controller controller, float current);