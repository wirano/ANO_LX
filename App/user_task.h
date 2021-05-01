//
// Created by wirano on 2021/4/28.
//

#ifndef USER_TASK_H
#define USER_TASK_H

#include "fc_config.h"

#define Mission_finish 1
#define UnMission_finish 0

void one_key_takeoff_land(uint16_t dt_ms);

uint8_t fly(uint8_t dt_ms,uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360);

void process_control(uint16_t dt_ms);

uint8_t user_takeoff(uint8_t dt_ms);

void fly_s(uint16_t dt_ms);
#endif //USER_TASK_H
