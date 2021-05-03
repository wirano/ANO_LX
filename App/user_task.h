//
// Created by wirano on 2021/4/28.
//

#ifndef USER_TASK_H
#define USER_TASK_H

#include "fc_config.h"

#define process_dt_ms   20
#define Mission_finish  1
#define Mission_Unfinished  0
#define delay_finish    1
#define delay_Unfinished    0
#define LX_LED  0
#define USER_LED    1
#define NONE    0
#define RGB_R   1
#define RGB_G   2
#define RGB_B   3
#define ALL 4
#define Mission_over 99

extern void onekey_lock(void);

void one_key_takeoff_land();

void light_check(uint8_t group, uint8_t color);

uint8_t fly(uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360);

void process_control();

uint8_t user_takeoff();

uint8_t process_delay(uint16_t delay_ms);

void fly_s();

uint8_t omv_find_detection();

#endif //USER_TASK_H
