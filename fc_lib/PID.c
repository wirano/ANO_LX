//
// Created by 11478 on 2021/5/3.
//

#include "PID.h"
#include <math.h>
#include <sched.h>

PID_PositionalTypeDef PID_PositionalLine_vy ={
        .Ki_Limit=0,
        .Ki_Separation=0,
        .Kp=8,
        //TODO
        .Ki=0,
        .Kd=0,
};

PID_PositionalTypeDef PID_PositionalLine_angle ={
        .Ki_Limit=0,
        .Ki_Separation=0,
        .Kp=0.03,
        .Ki=0,
        .Kd=0,
};

float PID_IncrementalRealize(PID_IncrementalTypeDef *PID, float inVal, float target) {
    float incrementSpeed;
    uint8_t Ki_enable = 1;

    PID->err = target - inVal;

    if (fabsf(PID->err) > PID->Ki_Separation) Ki_enable = 0;

    incrementSpeed =
            PID->Kp * (PID->err - PID->err_next) + Ki_enable * PID->Ki * PID->err +
            PID->Kd * (PID->err - 2 * PID->err_next + PID->err_last);

    PID->err_last = PID->err_next;
    PID->err_next = PID->err;


    return incrementSpeed;
}

float PID_PositionalRealize(PID_PositionalTypeDef *PID, float inVal, float target) {
    float incrementSpeed;

    PID->err = target - inVal;
    if (PID->err < PID->Ki_Separation && PID->err > -PID->Ki_Separation)
        PID->integral += PID->err;
    else
        PID->integral = PID->integral;

    if (PID->integral > PID->Ki_Limit)
        PID->integral = PID->Ki_Limit;
    else if (PID->integral < -PID->Ki_Limit)
        PID->integral = -PID->Ki_Limit;

    incrementSpeed = PID->Kp * PID->err + PID->Ki * PID->integral + PID->Kd * (PID->err - PID->err_last);
    PID->err_last = PID->err;

    return incrementSpeed;
}