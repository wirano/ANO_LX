//
// Created by wirano on 2021/4/28.
//

#include "user_task.h"
#include "drv_rc_in.h"
#include "ano_lx_function.h"
#include "main.h"

extern _rt_tar_un rt_tar;

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

uint8_t fly(uint8_t dt_ms, uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360) {
    static uint16_t fly_delay = 0;
    if (fly_delay == 0) {
        Horizontal_Move(distance_cm, velocity, dir_angle_0_360);
    } else if (fly_delay >= (distance_cm / velocity * 1000 + 1000)) {
        fly_delay = 0;
        return Mission_finish;
    }
    fly_delay += dt_ms;
    return UnMission_finish;
}

uint8_t user_takeoff(uint8_t dt_ms) {
    static uint16_t unlock_delay = 0;
    if (unlock_delay == 0) {
        FC_Unlock();
    } else if (unlock_delay == 1000) {
        OneKey_Takeoff(0); //参数单位：厘米； 0：默认上位机设置的高度。
    } else if (unlock_delay == 3000) {
        return Mission_finish;
    }
    unlock_delay += dt_ms;
    return UnMission_finish;
}

void process_control(uint16_t dt_ms) {
    static uint16_t mission_flag = 0, mission_step = 0;
    static uint16_t ready = 0;
    static uint8_t mission_finish;
    if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 2000 && mission_flag == 0 && ready == 1) {
        //进入程控模式
        mission_flag = 1;
        mission_step = 1;
    } else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 1500) {
        mission_flag = 0;
        mission_step = 0;
        ready = 1;
    }
    if (mission_flag == 1) {
        if (mission_step == 0)
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        else
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        if (mission_step == 1) {
            if (mission_finish == 0) {
                mission_finish = user_takeoff(dt_ms);
            } else {
                mission_step++;
                mission_finish = 0;
            }
        } else if (mission_step == 2) {
            if (mission_finish == 0) {
                mission_finish = fly(20, 80, 40, 45);
            } else {
                mission_step++;
                mission_finish = 0;
            }
//        } else if (mission_step == 3) {
//            if (mission_finish == 0) {
//                mission_finish = fly(20, 80, 40, 90);
//            } else {
//                mission_step++;
//                mission_finish = 0;
//            }
//        } else if (mission_step == 4) {
//            if (mission_finish == 0) {
//                mission_finish = fly(20, 80, 40, 180);
//            } else {
//                mission_step++;
//                mission_finish = 0;
//            }
//        } else if (mission_step == 5) {
//            if (mission_finish == 0) {
//                mission_finish = fly(20, 80, 40, 270);
//            } else {
//                mission_step++;
//                mission_finish = 0;
//            }
        } else {
            OneKey_Land();
            mission_step = 0;
            ready = 0;
        }
    } else {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
    }
}

void fly_s(uint16_t dt_ms) {
    static uint16_t fly_s_delay = 0;
    if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] == 2000) {
        if (fly_s_delay == 0) {
            rt_tar.st_data.vel_x = 50;
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        } else if (fly_s_delay == 2000) {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            rt_tar.st_data.vel_x = -50;
        }else if (fly_s_delay == 4000) {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            rt_tar.st_data.vel_x = 0;
            fly_s_delay=0;
        }
        fly_s_delay += dt_ms;
    }
}