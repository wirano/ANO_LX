//
// Created by wirano on 2021/4/28.
//

#ifndef BOARD_H
#define BOARD_H

#include "fc_config.h"
#include "ano_lx.h"

#define USE_VL53L1X

typedef struct
{
    uint8_t sig_mode; //0==null,1==ppm,2==sbus
    //
    int16_t ppm_ch[10];
    //
    int16_t sbus_ch[16];
    uint8_t sbus_flag;
    //
    uint16_t signal_fre;
    uint8_t no_signal;
    uint8_t fail_safe;
    _rc_ch_un rc_ch;
    uint16_t signal_cnt_tmp;
    uint8_t rc_in_mode_tmp;
} _rc_input_st;

//==数据声明
extern _rc_input_st rc_in;

uint8_t All_Init(void);

void DrvRcInputInit(void);

void DrvPpmGetOneCh(uint16_t data);

//void DrvSbusGetOneByte(uint8_t data);

void DrvRcInputTask(float dT_s);

#endif //BOARD_H
