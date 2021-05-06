//
// Created by 11478 on 2021/5/3.
//

#ifndef ANO_LX_PID_H
#define ANO_LX_PID_H

typedef struct {
    float err;//定义偏差值
    float err_next;//定义上一个偏差值
    float err_last;//定义最上前的偏差值
    float Ki_Separation;
    float Kp;
    float Ki;
    float Kd;//定义比例、积分、微分系数
} PID_IncrementalTypeDef;

typedef struct {
    float err; //定义偏差值
    float err_last; //定义上一个偏差值
    float Kp;
    float Ki;
    float Kd; //定义比例、积分、微分系数
    float integral; //定义积分值
    float Ki_Limit;
    float Ki_Separation;
} PID_PositionalTypeDef;

extern PID_PositionalTypeDef PID_PositionalLine_vy;
extern PID_PositionalTypeDef PID_PositionalLine_angle;

float PID_IncrementalRealize(PID_IncrementalTypeDef *PID, float inVal, float target);

float PID_PositionalRealize(PID_PositionalTypeDef *PID, float inVal, float target);

#endif //ANO_LX_PID_H
