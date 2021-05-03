//
// Created by wirano on 2021/4/28.
//

#include "user_task.h"
#include "drv_rc_in.h"
#include "ano_lx_function.h"
#include "ano_lx_state.h"

void one_key_takeoff_land(uint16_t dt_ms)
{
    //////////////////////////////////////////////////////////////////////
    //一键起飞/降落例程
    //////////////////////////////////////////////////////////////////////
    //用静态变量记录一键起飞/降落指令已经执行。
    static uint8_t one_key_takeoff_f = 1, one_key_land_f = 1, one_key_mission_f = 0;
    static uint16_t unlock_delay = 0;
//    static uint8_t mission_step;
    //判断有遥控信号才执行
    if (rc_in.no_signal == 0) {
        //判断第6通道拨杆位置 1300<CH_6<1700
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1500) {
            //还没有执行
            if (one_key_takeoff_f == 0) {
                //标记已经执行
                if (unlock_delay == 0) {
                    FC_Unlock();
                }

                if (unlock_delay > 2000) {
                    one_key_takeoff_f =
                            //执行一键起飞
                            OneKey_Takeoff(0); //参数单位：厘米； 0：默认上位机设置的高度。
                } else {
                    unlock_delay += dt_ms;
                }
            }
        } else {
            //复位标记，以便再次执行
            one_key_takeoff_f = 0;
            unlock_delay = 0;
        }
        //
        //判断第6通道拨杆位置 800<CH_6<1200
        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] < 1200) {
            //还没有执行
            if (one_key_land_f == 0) {
                //标记已经执行
                one_key_land_f =
                        //执行一键降落
                        OneKey_Land();
            }
        } else {
            //复位标记，以便再次执行
            one_key_land_f = 0;
        }
//        //判断第6通道拨杆位置 1700<CH_6<2000
//        if (rc_in.rc_ch.st_data.ch_[ch_6_aux2] > 1700)
//        {
//            //还没有执行
//            if (one_key_mission_f == 0)
//            {
//                //标记已经执行
//                one_key_mission_f = 1;
//                //开始流程
//                mission_step = 1;
//            }
//        }
//        else
//        {
//            //复位标记，以便再次执行
//            one_key_mission_f = 0;
//        }
//        //
//        if (one_key_mission_f == 1)
//        {
//        }
//        else
//        {
//            mission_step = 0;
//        }
    }
    ////////////////////////////////////////////////////////////////////////
}

inline void onekey_lock(void)
{
    if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] < 2000) {
        if (fc_sta.unlock_sta || fc_sta.unlock_cmd) {
            FC_Lock();
        }

        fc_sta.onekey_lock_unlocked = 0;
    }else if(!fc_sta.onekey_lock_unlocked)
    {
        fc_sta.onekey_lock_unlocked = 1;
    }
}
