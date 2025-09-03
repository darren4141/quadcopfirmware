#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#define ABS(x) (x > 0) ? x : -1 * x

typedef struct{
    float kP;
    float kI;
    float kD;
    
    float maxI;
    float upperBound;

    float eI;

    float target;
    float errorPrevious;

    uint64_t prev_time_us;
}pid_controller;

void PIDinitialize(pid_controller controller);

void setTarget(pid_controller controller, float new_target);

void setUpperBound(pid_controller controller, float new_bound);

void setEIBound(pid_controller controller, float new_maxI);

float PIDCalculate(pid_controller controller, float current);