//
// Created by wirano on 2021/4/28.
//

#ifndef ANO_LX_H
#define ANO_LX_H

#include "stm32f4xx.h"

enum
{
    ch_1_rol = 0,
    ch_2_pit,
    ch_3_thr,
    ch_4_yaw,
    ch_5_aux1,
    ch_6_aux2,
    ch_7_aux3,
    ch_8_aux4,
    ch_9_aux5,
    ch_10_aux6,
};

//0x40
typedef struct
{
    int16_t ch_[10]; //

}__attribute__ ((__packed__)) _rc_ch_st;

typedef union
{
    uint8_t byte_data[20];
    _rc_ch_st st_data;
} _rc_ch_un;

//0x41
typedef struct
{
    int16_t rol;
    int16_t pit;
    int16_t thr;
    int16_t yaw_dps;
    int16_t vel_x;
    int16_t vel_y;
    int16_t vel_z;

}__attribute__ ((__packed__)) _rt_tar_st;

typedef union
{
    uint8_t byte_data[14];
    _rt_tar_st st_data;
} _rt_tar_un;

//0x0D
typedef struct
{
    uint16_t voltage_100;
    uint16_t current_100;

}__attribute__ ((__packed__)) _fc_bat_st;

typedef union
{
    uint8_t byte_data[4];
    _fc_bat_st st_data;
} _fc_bat_un;


//
typedef struct
{
    uint16_t pwm_m1;
    uint16_t pwm_m2;
    uint16_t pwm_m3;
    uint16_t pwm_m4;
    uint16_t pwm_m5;
    uint16_t pwm_m6;
    uint16_t pwm_m7;
    uint16_t pwm_m8;
} _pwm_st;

//==数据声明
extern _rt_tar_un rt_tar;
extern _fc_bat_un fc_bat;
extern _pwm_st pwm_to_esc;
//==函数声明
//static


//public
void ANO_LX_Task(void);

#endif //ANO_LX_H
