//
// Created by wirano on 2021/4/28.
//

#ifndef ANO_LX_EXT_SENSORS_H
#define ANO_LX_EXT_SENSORS_H

#include "fc_config.h"

//==定义/声明
//====通用传感器数据====
typedef struct
{
    //
    vec3_s16 hca_velocity_cmps;

} __attribute__((__packed__)) _general_vel_st;

typedef struct
{
    //
    vec3_s32 ulhca_pos_cm;

} __attribute__((__packed__)) _general_pos_st;

typedef struct
{
    //
    uint8_t direction;
    uint16_t angle_100;
    int32_t distance_cm;

} __attribute__((__packed__)) _general_dis_st;

typedef union
{
    uint8_t byte[6];
    _general_vel_st st_data;
} _general_vel_un;

typedef union
{
    uint8_t byte[12];
    _general_pos_st st_data;
} _general_pos_un;

typedef union
{
    uint8_t byte[7];
    _general_dis_st st_data;
} _general_dis_un;
//====GPS数据====
typedef struct
{
    uint8_t FIX_STA;
    uint8_t S_NUM;
    int32_t LNG;
    int32_t LAT;
    int32_t ALT_GPS;
    int16_t N_SPE;
    int16_t E_SPE;
    int16_t D_SPE;
    uint8_t PDOP_001; //0.01f
    uint8_t SACC_001; //0.01f
    uint8_t VACC_001; //0.01f

} __attribute__((__packed__)) _fc_gps_st;

typedef union
{
    uint8_t byte[23];
    _fc_gps_st st_data;
} _fc_gps_un;
//====

typedef struct
{
    //
    _general_vel_un gen_vel;
    _general_pos_un gen_pos;
    _general_dis_un gen_dis;
    _fc_gps_un fc_gps;

} _fc_ext_sensor_st;
//==数据声明
extern _fc_ext_sensor_st ext_sens;
//==函数声明
//static

//public
void LX_FC_EXT_Sensor_Task(float dT_s);

#endif //ANO_LX_EXT_SENSORS_H
