//
// Created by wirano on 2021/5/4.
//

#include "drv_buzzer.h"

_buzzer_st buzzer = {
        .io.GPIO = BEEP_GPIO_Port,
        .io.pin = BEEP_Pin
};


void buzzer_drv(uint8_t dt_ms){
    float buzzer_cnt_max;
    static uint16_t buzzer_cnt;

    buzzer_cnt_max = 1000.0f / dt_ms / buzzer.freq;

    if(buzzer.freq >= 200){
        buzzer_cnt_max = 0;
    }

    if(buzzer_cnt < buzzer_cnt_max / 2){
        buzzer.io.GPIO->BSRR |= buzzer.io.pin;
    }else{
        buzzer.io.GPIO->BSRR |= buzzer.io.pin << 16U;
    }

    buzzer_cnt += dt_ms;
    if(buzzer_cnt > buzzer_cnt_max){
        buzzer_cnt = 0;
    }
}
