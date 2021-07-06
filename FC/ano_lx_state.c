//
// Created by wirano on 2021/4/28.
//

#include "ano_lx_state.h"
#include "drv_rc_in.h"
#include "ano_lx_dt.h"
#include "ano_lx_function.h"
#include "core_cm4.h"

_fc_state_st fc_sta;
_sticks_fun_st sti_fun;

//美国手，外八
#define TOE_OUT \
    (rc_in.rc_ch.st_data.ch_[ch_1_rol] > 1900 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_4_yaw] < 1100)

//美国手，内八
#define TOE_IN \
    (rc_in.rc_ch.st_data.ch_[ch_4_yaw] > 1900 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_1_rol] < 1100)

//美国手，左下+左下
#define LD_LD \
    (rc_in.rc_ch.st_data.ch_[ch_4_yaw] < 1100 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_1_rol] < 1100)

//美国手，右下+右下
#define RD_RD \
    (rc_in.rc_ch.st_data.ch_[ch_4_yaw] > 1900 && rc_in.rc_ch.st_data.ch_[ch_2_pit] < 1100 && rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1100 && rc_in.rc_ch.st_data.ch_[ch_1_rol] > 1900)

// 摇杆触发MCU复位条件
#define STICKS_MCU_RST_REQ  (RD_RD)

// 摇杆触发校准水平条件
#define STICKS_CALI_HOR_REQ (LD_LD)
//如果启用RD_RD条件MCU复位，则罗盘校准条件失效
#ifdef STICKS_MCU_RST_REQ
// 摇杆触发校准罗盘条件
#define STICKS_CALI_MAG_REQ (0)
#else
#define STICKS_CALI_MAG_REQ (RD_RD)
#endif

// 摇杆解锁条件requirement
#define STICKS_UNLOCK_REQ (TOE_OUT || TOE_IN)
// 摇杆上锁条件
#define STICKS_LOCK_REQ (STICKS_UNLOCK_REQ)
// 解锁持续时间,毫秒
#define UNLOCK_HOLD_TIME_MS (1000)
// 上锁持续时间,毫秒
#define LOCK_HOLD_TIME_MS (300)
// 当前解锁/上锁状态
#define UNLOCK_STATE (fc_sta.unlock_sta)

//
static uint16_t time_dly_cnt_ms;
static uint8_t unlock_lock_flag;

static void LX_Unlock_Lock_Check(float *dT_s)
{
    //判断yaw摇杆是否大致回中
    if ((rc_in.rc_ch.st_data.ch_[ch_4_yaw] > 1400 && rc_in.rc_ch.st_data.ch_[ch_4_yaw] < 1600)) {
        sti_fun.stick_mit_pos = 1;
        unlock_lock_flag = 1; //回中以后才能执行一次解锁或者上锁
    } else {
        sti_fun.stick_mit_pos = 0;
    }
    //标记预备上锁的动作
    if (rc_in.rc_ch.st_data.ch_[ch_3_thr] < 1200 && (sti_fun.stick_mit_pos == 0)) {
        sti_fun.pre_locking = 1;
    } else {
        sti_fun.pre_locking = 0;
    }
    //解锁
    if (unlock_lock_flag == 1) //执行条件
    {
        if (UNLOCK_STATE == 0) {
            if (STICKS_UNLOCK_REQ) {
                if (time_dly_cnt_ms < UNLOCK_HOLD_TIME_MS) {
                    time_dly_cnt_ms += *(dT_s) * 1e3f;
                } else {
                    FC_Unlock(); //解锁
                    time_dly_cnt_ms = 0;
                    unlock_lock_flag = 0; //不再执行
                }
            } else {
                time_dly_cnt_ms = 0;
            }
        } else if (UNLOCK_STATE == 1) {
            if (STICKS_LOCK_REQ) {
                if (time_dly_cnt_ms < LOCK_HOLD_TIME_MS) {
                    time_dly_cnt_ms += *(dT_s) * 1e3f;
                } else {
                    FC_Lock(); //上锁
                    time_dly_cnt_ms = 0;
                    unlock_lock_flag = 0; //不再执行
                }
            } else {
                time_dly_cnt_ms = 0;
            }
        }
    } else {
        //null
    }
}

void LX_Cali_Trig_Check()
{
    static uint8_t cali_f;
    //为上锁状态才执行
    if (UNLOCK_STATE == 0) {
        //执行条件
        if (STICKS_CALI_HOR_REQ) {
            //标记只执行一次
            if (cali_f == 0) {
                Horizontal_Calibrate();
                cali_f = 1;
            }
        } else if (STICKS_CALI_MAG_REQ) {
            if (cali_f == 0) {
                Mag_Calibrate();
                cali_f = 1;
            }
        } else {
            cali_f = 0;
        }
    }
}

void MCU_Reset_Trig_Check()
{
    if (UNLOCK_STATE == 0) {
#ifdef STICKS_MCU_RST_REQ
        if (STICKS_MCU_RST_REQ) {
            __NVIC_SystemReset(); //直接复位，无需判断执行一次
        }
#endif
    }
}

void LX_FC_State_Task(float dT_s)
{
    //有遥控信号才执行
    if (rc_in.no_signal == 0) {
        //
        LX_Unlock_Lock_Check(&dT_s);
        //
        LX_Cali_Trig_Check();

        MCU_Reset_Trig_Check();
    }
}