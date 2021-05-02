//
// Created by wirano on 2021/4/28.
//

#include "user_task.h"
//#include "drv_rc_in.h"
#include "ano_lx_function.h"
#include "main.h"
#include "open_mv.h"
#include "ano_lx.h"
#include "ano_lx_function.h"
#include "ano_math.h"

extern _rt_tar_un rt_tar;
extern uint16_t ano_mode;

void one_key_takeoff_land() {
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
                    unlock_delay += process_dt_ms;
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

void light_check(uint8_t group,uint8_t color)
{
    if (group==LX_LED){
        HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ANO_RGB_G_GPIO_Port, ANO_RGB_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_RESET);
        if (color==RGB_R){
            HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_SET);
        }
        if (color==RGB_G){
            HAL_GPIO_WritePin(ANO_RGB_G_GPIO_Port, ANO_RGB_G_Pin, GPIO_PIN_SET);
        }
        if (color==RGB_B){
            HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_SET);
        }
    } else if (group==USER_LED){
        HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ANO_RGB_G_GPIO_Port, ANO_RGB_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_RESET);
//        if (color==RGB_B){
//            HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_SET);
//        }
//        if (color==RGB_G){
//            HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_SET);
//        }
//        if (color==RGB_R){
//            HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_SET);
//        }
    }
}

uint8_t process_delay(uint16_t delay_ms) {
    static uint16_t now_delay = 0;
    if (now_delay >= delay_ms) {
        now_delay = 0;
        return delay_finish;
    } else {
        now_delay += process_dt_ms;
        return delay_Unfinished;
    }
}

uint8_t fly(uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360) {
    static uint16_t fly_f = 0;
    if (fly_f == 0) {
        fly_f = 1;
        Horizontal_Move(distance_cm, velocity, dir_angle_0_360);
        process_delay(distance_cm / velocity * 1000);
    } else if (process_delay(distance_cm / velocity * 1000) == delay_finish) {
        fly_f = 0;
        return Mission_finish;
    }
    return Mission_Unfinished;
}

uint8_t user_takeoff() {
    static uint16_t user_takeoff_f = 0, user_unlock_f = 0;
    if (user_unlock_f == 0) {
        FC_Unlock();
        user_unlock_f = 1;
    } else if (user_unlock_f == 1 && user_takeoff_f == 0) {
        if (process_delay(1000) == delay_finish) {
            OneKey_Takeoff(0); //参数单位：厘米； 0：默认上位机设置的高度。
            user_takeoff_f = 1;
        }
    } else if (user_takeoff_f == 1) {
        if (process_delay(1000) == delay_finish) {
            user_takeoff_f = 0;
            user_unlock_f = 0;
            return Mission_finish;
        }
    }
    return Mission_Unfinished;
}

void process_control() {
    static uint16_t mission_flag = 0, mission_step = 0;
    static uint16_t ready = 0;
    static uint8_t mission_finish;
    static uint8_t omv_lose;

    if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 2000 && mission_flag == 0 && ready == 1) {
        //进入程控模式
        mission_flag = 1;
        mission_step = 1;
    } else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 1500) {
        mission_flag = 0;
        mission_step = 0;
        ready = 1;
    }
    if (mission_step != 0 && mission_flag == 1) {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        light_check(LX_LED,NONE);
    }//BEEP SWITCH

    if (mission_flag == 1) {
        if (mission_step == 1) {
            if (user_takeoff() == Mission_finish) {
                mission_step++;
            }
        } //程控起飞
        else if (mission_step == 2) {
            if (omv.offline == 0 && omv.raw_data.find == 1) {
                omv_lose = 0;

                if (omv.raw_data.type == OMV_DATA_LINE) {
                    light_check(LX_LED,RGB_G);
                    if (ABS(omv.raw_data.line.angle)>10)
                    Left_Rotate(ABS(omv.raw_data.line.angle),5);
                }
                if (omv.raw_data.type == OMV_DATA_BLOCK) {
                    light_check(LX_LED,RGB_B);
                }
            } else if (omv.offline == 0 && omv.raw_data.find == 0) {
                light_check(LX_LED,RGB_R);
                omv_lose++;
            }
            if (omv_lose > (6000 / process_dt_ms)) {
                mission_step++;
            }
        } else {
            OneKey_Land();
            mission_step = 0;
            ready = 0;
            omv_lose=0;
        }
    }
}

void fly_s() {
    static uint16_t fly_s_delay = 0;
    if (rc_in.rc_ch.st_data.ch_[ch_7_aux3] == 2000) {
        if (fly_s_delay == 0) {
            rt_tar.st_data.vel_x = 50;
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        } else if (fly_s_delay == 2000) {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            rt_tar.st_data.vel_x = -50;
        } else if (fly_s_delay == 4000) {
            HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
            rt_tar.st_data.vel_x = 0;
            fly_s_delay = 0;
        }
        fly_s_delay += process_dt_ms;
    }
}