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

Process_Delay Land_delay = {
        .now_delay=0,
        .delay_star=0,
        .ami_delay=0,
        .delay_finished=0,
};

void process_delay(Process_Delay *user_delay) {
    if (user_delay->delay_star == 1) {
        user_delay->now_delay += process_dt_ms;
        if (user_delay->now_delay >= user_delay->ami_delay) {
            user_delay->delay_finished = 1;
        }
    } else {
        user_delay->now_delay = 0;
        user_delay->ami_delay = 0;
        user_delay->delay_finished = 0;
    }
}


void process_control() {
    static uint16_t mission_flag = 0, mission_step = 0;
    static uint16_t ready = 0;
    static uint8_t mission_finish, block_f = 0;
    static uint8_t Mission_state = Mission_Unfinished;
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
        buzzer.freq = 5;
        omv_led_state.led_num = 4;
    } else {
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
            Mission_state = omv_find_blobs();
            switch (Mission_state) {
                case Mission_finish:
                    mission_step++;
                    break;
                case Mission_err:
                    mission_step = Mission_over;
                    break;
                case Mission_Unfinished:
                    break;
            }
        } else if (mission_step == 3) {
            if (Land_delay.delay_star==0) {
                Horizontal_Move(40, 20, 0);
                Land_delay.delay_star=1;
                Land_delay.ami_delay=2000;
            }
            if (Land_delay.delay_finished==1)
                mission_step=Mission_over;
        } else if (mission_step == Mission_over) {
            OneKey_Land();
            ready = 0;
            mission_flag = 0;
            Takeoff_delay.delay_star = 0;
            Unlock_delay.delay_star = 0;
            Block_delay.delay_star = 0;
            Land_delay.delay_star=0;
        }
    }
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

uint8_t omv_find_blobs() {
    static uint8_t Unfind_time = 0;
    static uint32_t move_angle = 0, last_x = 0, last_y = 0;
    if (Block_delay.delay_star != 1) {
        Block_delay.delay_star = 1;
        Block_delay.ami_delay = 5000;
    }
    if (omv.online == 1 && omv.raw_data.data_flushed == 1) {
        if (omv.raw_data.find == 1) {
            omv_decoupling(20, fc_sta.fc_attitude.rol, fc_sta.fc_attitude.pit);
            omv.raw_data.data_flushed = 0;
            move_angle = (int) my_atan(omv.block_track_data.offset_y_decoupled_lpf,
                                       omv.block_track_data.offset_x_decoupled_lpf);
            if ((ABS(last_x - omv.block_track_data.offset_x_decoupled_lpf) < 10) &&
                ABS((last_y - omv.block_track_data.offset_y_decoupled_lpf) < 10)) {
                if (Block_delay.delay_finished == 1)
                    return Mission_finish;
            } else {
                Block_delay.now_delay = 0;
            }
            if (move_angle > 0) {
                Horizontal_Move(40, 15, move_angle);
            } else if (move_angle < 0) {
                Horizontal_Move(40, 15, 360 - move_angle);
            }
            Unfind_time = 0;
            printf("block x:%d y:%d\r\n", (int) omv.raw_data.block.center_x, (int) omv.raw_data.block.center_y);
            printf("de block x:%.2f y:%.2f\r\n", omv.block_track_data.offset_x_decoupled_lpf,
                   omv.block_track_data.offset_y_decoupled_lpf);
            printf("angle :%d \r\n", (int) move_angle);
            last_x = (int) omv.block_track_data.offset_x_decoupled_lpf;
            last_y = (int) omv.block_track_data.offset_y_decoupled_lpf;
        } else if (omv.raw_data.find == 0) {
            Unfind_time++;
            if (Unfind_time == 50)
                OneKey_Hover();
            if (Unfind_time >= 200)
                return Mission_err;
            else
                return Mission_Unfinished;
        }
        return Mission_Unfinished;
    }
    if (omv_find_detection() == Mission_over) {
        return Mission_err;
    }
    return Mission_Unfinished;
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
                printf("pid_vy:%.2f pid_angle:%.2f\r\n", pid_vy, pid_angle);
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
        } else if (omv.raw_data.find == 0) {
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

void onekey_lock(void) {
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

