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
extern Process_Delay Takeoff_delay;
extern Process_Delay Unlock_delay;
extern Process_Delay Block_delay;
extern Process_Delay Land_delay;

void process_control();

uint8_t user_takeoff();

void process_delay(Process_Delay *user_delay);

uint8_t omv_find_detection();

uint8_t omv_find_blobs();

uint8_t omv_find_lines() ;

#endif //USER_TASK_H
