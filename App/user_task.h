//
// Created by wirano on 2021/4/28.
//

#ifndef USER_TASK_H
#define USER_TASK_H

#include "fc_config.h"

#define process_dt_ms   20
#define Mission_err 2
#define Mission_finish  1
#define Mission_Unfinished  0
#define LX_LED  0
#define USER_LED    1
#define NONE    0
#define RGB_R   1
#define RGB_G   2
#define RGB_B   3
#define ALL 4
#define Mission_over 99

typedef struct {
    uint8_t delay_star;
    uint32_t now_delay;
    uint32_t ami_delay;
    uint8_t delay_finished;
} Process_Delay;

extern void onekey_lock(void);

void one_key_takeoff_land();

void light_check(uint8_t group, uint8_t color);

uint8_t fly(uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360);

void process_control();

uint8_t user_takeoff();

void process_delay();

void fly_s();

uint8_t omv_find_detection();

uint8_t omv_find_lines();

void TestHeightSet(uint16_t Hz);   //测试HeightSet()函数的功能

#endif //USER_TASK_H
