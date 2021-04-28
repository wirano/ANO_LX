//
// Created by wirano on 2021/4/28.
//

#ifndef ANO_LX_STATE_H
#define ANO_LX_STATE_H

#include "fc_config.h"

typedef struct
{
    uint8_t pre_locking;
    uint8_t stick_mit_pos;

} _sticks_fun_st;

typedef struct
{
    uint8_t CID;
    uint8_t CMD_0;
    uint8_t CMD_1;
} _cmd_fun_st;
//飞控状态
typedef struct
{
    //模式
    uint8_t fc_mode_cmd;
    uint8_t fc_mode_sta;

    //解锁上锁
    uint8_t unlock_cmd;
    uint8_t unlock_sta;

    //指令功能
    _cmd_fun_st cmd_fun;

    //state
    uint8_t imu_ready;
    uint8_t take_off;
    uint8_t in_air;
    uint8_t landing;

} _fc_state_st;

//==数据声明
extern _fc_state_st fc_sta;
extern _sticks_fun_st sti_fun;
//==函数声明
//static

//public
void LX_FC_State_Task(float dT_s);

#endif //ANO_LX_STATE_H
