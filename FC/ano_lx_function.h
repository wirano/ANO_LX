//
// Created by wirano on 2021/4/28.
//

#ifndef ANO_LX_FUNCTION_H
#define ANO_LX_FUNCTION_H

#include "fc_config.h"

uint8_t FC_Unlock(void);

uint8_t FC_Lock(void);

uint8_t LX_Change_Mode(uint8_t new_mode);

uint8_t OneKey_Takeoff(uint16_t height_cm);

uint8_t OneKey_Land(void);

uint8_t OneKey_Flip(void);

uint8_t OneKey_Return_Home(void);

uint8_t Horizontal_Calibrate(void);

uint8_t Horizontal_Move(uint16_t distance_cm, uint16_t velocity_cmps, uint16_t dir_angle_0_360);

uint8_t Mag_Calibrate(void);

uint8_t ACC_Calibrate(void);

uint8_t GYR_Calibrate(void);

#endif //ANO_LX_FUNCTION_H
