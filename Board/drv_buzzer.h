//
// Created by wirano on 2021/5/4.
//

#ifndef DRV_BUZZER_H
#define DRV_BUZZER_H

#include "main.h"

typedef struct {
   GPIO_TypeDef *GPIO;
   uint16_t pin;
}_buzzer_hw_st;

typedef struct {
    float freq;
    _buzzer_hw_st io;
}_buzzer_st;

extern _buzzer_st buzzer;

void buzzer_drv(uint8_t dt_ms);

#endif //DRV_BUZZER_H
