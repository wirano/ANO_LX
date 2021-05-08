//
// Created by wirano on 2021/4/28.
//

#include "user_task.h"
#include "ano_lx_function.h"
#include "main.h"
#include "open_mv.h"
#include "ano_lx.h"
#include "ano_lx_state.h"
#include "ano_math.h"
#include "PID.h"
#include <math.h>
#include "drv_led.h"
#include <stdio.h>
#include "drv_buzzer.h"

extern _rt_tar_un rt_tar;
extern uint16_t ano_mod;

Process_Delay Takeoff_delay = {
        .now_delay=0,
        .delay_star=0,
        .ami_delay=0,
        .delay_finished=0,
};

Process_Delay Unlock_delay = {
        .now_delay=0,
        .delay_star=0,
        .ami_delay=0,
        .delay_finished=0,
};

Process_Delay Block_delay = {
        .now_delay=0,
        .delay_star=0,
        .ami_delay=0,
        .delay_finished=0,
};

void process_delay() {
    if (Takeoff_delay.delay_star == 1) {
        Takeoff_delay.now_delay += process_dt_ms;
        if (Takeoff_delay.now_delay >= Takeoff_delay.ami_delay) {
            Takeoff_delay.delay_finished = 1;
        }
    } else {
        Takeoff_delay.now_delay = 0;
        Takeoff_delay.ami_delay = 0;
        Takeoff_delay.delay_finished = 0;
    }
    if (Unlock_delay.delay_star == 1) {
        Unlock_delay.now_delay += process_dt_ms;
        if (Unlock_delay.now_delay >= Unlock_delay.ami_delay) {
            Unlock_delay.delay_finished = 1;
        }
    } else {
        Unlock_delay.now_delay = 0;
        Unlock_delay.ami_delay = 0;
        Unlock_delay.delay_finished = 0;
    }
    if (Block_delay.delay_star == 1) {
        Block_delay.now_delay += process_dt_ms;
        if (Block_delay.now_delay >= Block_delay.ami_delay) {
            Block_delay.delay_finished = 1;
        }
    } else {
        Block_delay.now_delay = 0;
        Block_delay.ami_delay = 0;
        Block_delay.delay_finished = 0;
    }
}


void process_control() {
    static uint16_t mission_flag = 0, mission_step = 0;
    static uint16_t ready = 0;
    static uint8_t mission_finish, block_f = 0;

    if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 2000 && mission_flag == 0 && ready == 1) {
        //进入程控模式
        mission_flag = 1;
        mission_step = 1;
    } else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 1500) {
        mission_flag = 0;
        mission_step = 0;
        ready = 1;
        Takeoff_delay.delay_star = 0;
        Unlock_delay.delay_star = 0;
        Block_delay.delay_star = 0;
    }

    if (mission_flag == 1) {
//        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
        buzzer.freq = 5;
        omv_led_state.led_num = 4;
    } else {
//        HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
        buzzer.freq = 200;
        omv_led_state.led_num = 0;
    }//BEEP SWITCH

    if (mission_flag == 1) {
        if (mission_step == 1) {
            if (user_takeoff() == 1) {
                mission_step++;
            }
        } //程控起飞
        else if (mission_step == 2) {
            omv_find_lines();
        } else if (mission_step == 3) {
            OneKey_Land();
//            mission_step==Mission_over;
        } else if (mission_step == Mission_over) {
            OneKey_Land();
            ready = 0;
            mission_flag = 0;
            Takeoff_delay.delay_star = 0;
            Unlock_delay.delay_star = 0;
            Block_delay.delay_star = 0;
        }
    }
//    printf("mission_step:%d\r\n",mission_step);
}

uint8_t omv_find_detection() {
    static uint16_t omv_lose;
    if (omv.online == 1 && omv.raw_data.find == 0) {
        omv_lose++;
    }
    if (omv.online == 1 && omv.raw_data.find == 1) {
        omv_lose = 0;
    }
    if (omv.online == 0) {
        omv_lose++;
    }
    if (omv_lose > (1000 / process_dt_ms)) {
        return Mission_over;
    }
    return 0;
}

uint8_t omv_find_lines() {
    static float pid_angle, pid_vy;
    static uint16_t move_angle = 0;
    if (omv.online == 1 && omv.raw_data.data_flushed == 1) {
        if (omv.raw_data.find == 1) {
            omv_decoupling(20, fc_sta.fc_attitude.rol, fc_sta.fc_attitude.pit);
            omv.raw_data.data_flushed = 0;
            if (omv.raw_data.type == OMV_DATA_LINE || omv.raw_data.type == OMV_DATA_BOTH) {
                pid_angle = PID_PositionalRealize(&PID_PositionalLine_angle, omv.raw_data.line.angle, 0);
                pid_vy = PID_PositionalRealize(&PID_PositionalLine_vy,
                                               omv.line_track_data.offset_decoupled_lpf, 0);
                if ((ABS(omv.raw_data.line.angle) > 5) || (ABS(omv.line_track_data.offset_decoupled_lpf) > 5)) {
                    move_angle = (int) (ABS(omv.raw_data.line.angle) + atan2(ABS(pid_vy), 3) / 3.14 * 180);
                    if (pid_angle > 0) {
                        Left_Rotate(5, ABS(pid_angle));
                        Horizontal_Move(40, 20, 360 - move_angle);
                    }
                    if (pid_angle < 0) {
                        Right_Rotate(5, ABS(pid_angle));
                        Horizontal_Move(40, 20, move_angle);
                    }
                } else {
                    Horizontal_Move(30, 20, (int) pid_angle);
                    Left_Rotate(0, 0);
                }
                if (omv.raw_data.type == OMV_DATA_BOTH) {
                    Block_delay.delay_star = 1;
                    Block_delay.ami_delay = 3000;
                    if (Block_delay.delay_finished == 1)
                        return Mission_finish;
                }
                printf("line offset:%d angle:%d\r\n", (int) omv.raw_data.line.offset, omv.raw_data.line.angle);
                printf("de line offset:%.2f\r\n", omv.line_track_data.offset_decoupled_lpf);
                printf("pid_vy:%.2f pid_angle:%.2f\r\n",pid_vy,pid_angle);
            } else if (omv.raw_data.type == OMV_DATA_BLOCK) {
                Block_delay.delay_star = 1;
                Block_delay.ami_delay = 3000;
                if (Block_delay.delay_finished == 1) {
                    Block_delay.delay_star = 0;
                    return Mission_finish;
                } else {
                    Horizontal_Move(30, 20, 0);
                    Left_Rotate(0, 0);
                }
//                        printf("block x:%d y:%d\r\n",(int)omv.raw_data.block.center_x,(int)omv.raw_data.block.center_y);
//                        printf("de block x:%.2f y:%.2f\r\n",omv.block_track_data.offset_x_decoupled_lpf,omv.block_track_data.offset_y_decoupled_lpf);
            }
        }
        else if (omv.raw_data.find == 0) {
            Horizontal_Move(30, 20, 0);
            Left_Rotate(0, 0);
        }
    }
    if (omv_find_detection() == Mission_over) {
        return Mission_err;
    }
    return Mission_Unfinished;
}

uint8_t user_takeoff() {
    static uint16_t user_takeoff_f = 0, user_unlock_f = 0;
    if (Unlock_delay.delay_star == 0) {
        FC_Unlock();
        Unlock_delay.delay_star = 1;
        Unlock_delay.ami_delay = 1000;
    } else if (Unlock_delay.delay_finished == 1 && Takeoff_delay.delay_star == 0) {
        OneKey_Takeoff(0); //参数单位：厘米； 0：默认上位机设置的高度。
        Takeoff_delay.delay_star = 1;
        Takeoff_delay.ami_delay = 2000;
    } else if (Takeoff_delay.delay_finished == 1) {
        Unlock_delay.delay_star = 0;
        Takeoff_delay.delay_star = 0;
        return Mission_finish;
    }
    return Mission_Unfinished;
}

inline void onekey_lock(void) {
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
    } else if (!reseted) {
        if (fc_sta.unlock_sta || fc_sta.unlock_cmd) {
            FC_Lock();
        }

        fc_sta.esc_output_unlocked = 0;
    }
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

uint8_t fly(uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360) {
//    static uint16_t fly_f = 0;
//    if (fly_f == 0) {
//        fly_f = 1;
//        Horizontal_Move(distance_cm, velocity, dir_angle_0_360);
//        process_delay(distance_cm / velocity * 1000);
//    } else if (process_delay(distance_cm / velocity * 1000) == delay_finish) {
//        fly_f = 0;
//        return Mission_finish;
//    }
    return Mission_Unfinished;
}

