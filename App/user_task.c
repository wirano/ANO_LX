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
#include "ano_lx_state.h"
#include "drv_buzzer.h"
#include "ano_math.h"
#include "PID.h"
#include <math.h>
#include <stdio.h>

extern _rt_tar_un rt_tar;
extern uint16_t ano_mod;

uint8_t omv_find_detection() {
    static uint16_t omv_lose;
    if (omv.online == 1 && omv.raw_data.find == 0) {
        light_check(LX_LED, RGB_R);
        omv_lose++;
    } else if (omv.online == 1 && omv.raw_data.find == 1) {
        omv_lose = 0;
    }
    if (omv.online == 0) {
        light_check(USER_LED, RGB_R);
        omv_lose++;
    }
    if (omv_lose > (3000 / process_dt_ms)) {
        return Mission_over;
    }
    return 0;
}

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


void light_check(uint8_t group, uint8_t color) {
    if (group == LX_LED) {
        HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ANO_RGB_G_GPIO_Port, ANO_RGB_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_RESET);
        if (color == RGB_R) {
            HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_SET);
        }
        if (color == RGB_G) {
            HAL_GPIO_WritePin(ANO_RGB_G_GPIO_Port, ANO_RGB_G_Pin, GPIO_PIN_SET);
        }
        if (color == RGB_B) {
            HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_SET);
        }
        if (color == ALL) {
            HAL_GPIO_WritePin(ANO_RGB_R_GPIO_Port, ANO_RGB_R_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ANO_RGB_G_GPIO_Port, ANO_RGB_G_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ANO_RGB_B_GPIO_Port, ANO_RGB_B_Pin, GPIO_PIN_SET);
        }
    } else if (group == USER_LED) {
        HAL_GPIO_WritePin(USER_LED_R_GPIO_Port, USER_LED_R_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED_G_GPIO_Port, USER_LED_G_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(USER_LED_B_GPIO_Port, USER_LED_B_Pin, GPIO_PIN_RESET);
        if (color == RGB_R) {
            HAL_GPIO_WritePin(USER_LED_R_GPIO_Port, USER_LED_R_Pin, GPIO_PIN_SET);
        }
        if (color == RGB_G) {
            HAL_GPIO_WritePin(USER_LED_G_GPIO_Port, USER_LED_G_Pin, GPIO_PIN_SET);
        }
        if (color == RGB_B) {
            HAL_GPIO_WritePin(USER_LED_B_GPIO_Port, USER_LED_B_Pin, GPIO_PIN_SET);
        }
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
    static uint8_t mission_finish, block_f = 0;
    static uint16_t omv_lose, last_offset = 0;
    static float pid_angle,pid_vy;
    static uint16_t move_angle = 0;

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
        light_check(LX_LED, NONE);
        light_check(USER_LED, NONE);
    }//BEEP SWITCH
    if (mission_flag == 1) {
        if (mission_step == 1) {
            if (user_takeoff() == Mission_finish) {
                mission_step++;
            }
        } //程控起飞
        else if (mission_step == 2) {
            if (omv.online == 1) {
                if (omv.raw_data.find == 1 && omv.raw_data.data_flushed == 1) {
                    omv.raw_data.data_flushed = 0;
                    if (omv.raw_data.type == OMV_DATA_LINE) {
                        light_check(LX_LED, RGB_G);
                        if (ABS(omv.raw_data.line.angle) > 10) {
                            pid_angle= PID_PositionalRealize(&PID_PositionalLine_angle,omv.raw_data.line.angle,0);
//                            pid_vy= PID_PositionalRealize(&PID_PositionalLine_vy,,0);
                            move_angle=(int)(omv.raw_data.line.angle+ atan2(pid_vy,40)/3.14*180);
                            if (pid_angle<0) {
                                Left_Rotate(10, ABS(pid_angle));
                                Horizontal_Move(40,40,move_angle);
//                                Horizontal_Move(30, 40, 360 + omv.raw_data.line.angle+PID_PositionalRealize(&PID_PositionalLine_angle, speed[0], setSpeed[0]);
                                light_check(USER_LED, RGB_B);
                            }
                            if (pid_angle>0) {
                                Right_Rotate(10, ABS(pid_angle));
                                Horizontal_Move(40,40,move_angle);
                                light_check(USER_LED, RGB_G);
                            }
                        } else{
                            Horizontal_Move(30, 40, 0);
                        }
                    }
                    if (omv.raw_data.type == OMV_DATA_BLOCK && process_delay(10000) == delay_finish) {
                        mission_step++;
                    }
                }
                if (omv.online == 1 && (omv.raw_data.find == 0 || omv.raw_data.type == OMV_DATA_BLOCK)) {
                    Horizontal_Move(30, 40, 0);
                }
            }
            if (omv_find_detection() == Mission_over) {
                mission_step = Mission_over;
            }
        } else if (mission_step == Mission_over) {
            OneKey_Land();
            mission_step = 0;
            ready = 0;
            omv_lose = 0;
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

inline void onekey_lock(void)
{
    static uint8_t reseted = 0;

    if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] == 1000) {
        if (fc_sta.unlock_sta || fc_sta.unlock_cmd) {
            FC_Lock();
        }

        if (!reseted) {
            reseted = 1;
        }

        fc_sta.esc_output_unlocked = 0;
    } else if (!fc_sta.esc_output_unlocked && reseted) {
        fc_sta.esc_output_unlocked = 1;
    } else if(!reseted) {
        if (fc_sta.unlock_sta || fc_sta.unlock_cmd) {
            FC_Lock();
        }

        fc_sta.esc_output_unlocked = 0;
    }
}

void MyProcessTest(uint16_t Hz)
{
    static uint16_t State=0;

    if(fc_sta.unlock_cmd==1 && rc_in.rc_ch.st_data.ch_[ch_5_aux1]>1800 && rc_in.rc_ch.st_data.ch_[ch_5_aux1]<2200)  //如果解锁且处于程控模式
    {
        if(State==0)
        {
            OneKey_Takeoff(100);
            State++;
        }
        else if(State==1)
        {
            Wait(Hz,3,&State);
        }
        else if(State==2)
        {
            if( CircularMotion(Hz,50,6,720,180,1) == 1 )
            {
                State++;
            }
        }
        else if(State==3)
        {
            Wait(Hz,2,&State);
        }
        else if(State==4)
        {
            if( CircularMotion(Hz,50,6,720,180,0) == 1 )
            {
                State++;
            }
        }
        else if(State==5)
        {
            Wait(Hz,6,&State);
        }
        else if(State==6)
        {
            OneKey_Land();
            State++;
        }
        else if(State==7)
        {

        }
    }
}

//圆周运动 (任务频率，半径cm，转速_转/min，旋转角度，初相位_0到360度，转向_0逆1顺)
uint8_t CircularMotion(uint16_t Hz,uint16_t r_cm,uint16_t speed_r_min,uint16_t all_angle,uint16_t ini_phase,uint8_t direction)
{
    float w_rad=0;
    float Distance=0;
    float Speed=0;
    static uint16_t Angle=0;
    static uint16_t T_Count=0;
    static uint8_t StartFlag=1;

    if(direction==1)
    {
        if(Angle<(all_angle+ini_phase+90))
        {
            T_Count++;
            w_rad = MY_PPPIII * (float) speed_r_min / 30.0f;
            Speed = (float) r_cm * w_rad;
            Distance = Speed / (float) Hz;
            Angle = (uint16_t) (w_rad * 57.29578f * (float) T_Count / (float) Hz)+ini_phase+90;

            Horizontal_Move((uint16_t)Distance,(uint16_t)Speed,(Angle%360));
            return 0;
        }
        else
        {
            T_Count=0;
            Angle=0;
            StartFlag=1;
            return 1;
        }
    }
    else if(direction==0)
    {
        if(StartFlag==1)
        {
            StartFlag=0;
            Angle=64980+ini_phase;
        }
        else if(Angle>(64980-all_angle+ini_phase+90))
        {
            T_Count++;
            w_rad = MY_PPPIII * (float) speed_r_min / 30.0f;
            Speed = (float) r_cm * w_rad;
            Distance = Speed / (float) Hz;
            Angle =64980 - (uint16_t) (w_rad * 57.29578f * (float) T_Count / (float) Hz)+ini_phase+90;

            Horizontal_Move((uint16_t)Distance,(uint16_t)Speed,(Angle%360));
            return 0;
        }
        else
        {
            T_Count=0;
            Angle=0;
            StartFlag=1;
            return 1;
        }
    }
    else
    {
        return 0;
    }
}
