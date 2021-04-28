//
// Created by wirano on 2021/4/28.
//

#ifndef DRV_ANO_OF_H
#define DRV_ANO_OF_H

#include "fc_config.h"

//==定义/声明

typedef struct
{
    //
    uint8_t of_update_cnt;  //光流数据更新计数。
    uint8_t alt_update_cnt; //高度数据更新计数。
    //
    uint8_t link_sta; //连接状态：0，未连接。1，已连接。
    uint8_t work_sta; //工作状态：0，异常。1，正常
    //
    uint8_t of_quality;
    //
    uint8_t of0_sta;
    int8_t of0_dx;
    int8_t of0_dy;
    //
    uint8_t of1_sta;
    int16_t of1_dx;
    int16_t of1_dy;
    //
    uint8_t of2_sta;
    int16_t of2_dx;
    int16_t of2_dy;
    int16_t of2_dx_fix;
    int16_t of2_dy_fix;
    int16_t intergral_x;
    int16_t intergral_y;
    //
    uint32_t of_alt_cm;
    //
    float quaternion[4];
    //
    int16_t acc_data_x;
    int16_t acc_data_y;
    int16_t acc_data_z;
    int16_t gyr_data_x;
    int16_t gyr_data_y;
    int16_t gyr_data_z;

} _ano_of_st;

//飞控状态

//==数据声明
extern _ano_of_st ano_of;

//==函数声明
//static
static void AnoOF_DataAnl(uint8_t *data_buf, uint8_t num);

//public
void AnoOF_GetOneByte(uint8_t data);

void AnoOF_Check_State(float dT_s);

#endif //DRV_ANO_OF_H
