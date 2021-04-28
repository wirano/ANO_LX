//
// Created by wirano on 2021/4/27.
//

#ifndef DRV_PWM_OUT_H
#define DRV_PWM_OUT_H

#include "stm32f4xx.h"

void DrvPwmOutInit(void);
void DrvMotorPWMSet(int16_t pwm[]); //范围0-1000

#endif //DRV_PWM_OUT_H
