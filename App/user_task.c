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
#include "ano_scheduler.h"
#include "ano_lx_dt.h"

SensorData Sensor_Data=
        {
            .Distance=0,
            .ImageX=0,
            .ImageY=0,
        };
SensorState Sensor_State=
        {
            .TofState=0,
            .ImageState=0,
        };

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
    static uint16_t mission_flag = 0, mission_step = 0, last_mission_step;
    static uint16_t ready = 0;
    static uint8_t Mission_state = Mission_Unfinished;
    if (last_mission_step != mission_step)
        printf("mission_step:%d\r\n", mission_step);
    last_mission_step = mission_step;
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
        printf("state:%d\r\n",omv.raw_data.find);
        if (mission_step == 1) {
            if (user_takeoff() == 1) {
                mission_step++;
            }
        } //程控起飞
//        else if (mission_step == 2) {
//            Horizontal_Move(200, 15, 290);
//            mission_step=6;
//        } //
        else if (mission_step == 2) {
            Mission_state = omv_find_blobs();
            switch (Mission_state) {
                case Mission_finish:
                        mission_step++;
                    break;
                case Mission_err:
                    printf("err!!!!!!!!\r\n");
                    mission_step = Mission_over;
                    break;
                case Mission_Unfinished:
                    break;
                }
        } else if (mission_step == 3) {
            if (Land_delay.delay_star == 0) {
                Horizontal_Move(40, 15, 0);
                Land_delay.delay_star = 1;
                Land_delay.ami_delay = 1500;
                }
            if (Land_delay.delay_finished == 1)
                mission_step = Mission_over;
        } else if (mission_step == Mission_over) {
            OneKey_Land();
            ready = 0;
            mission_flag = 0;
            Takeoff_delay.delay_star = 0;
            Unlock_delay.delay_star = 0;
            Block_delay.delay_star = 0;
            Land_delay.delay_star = 0;
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
    static uint8_t Unfind_time = 0, speed, distance;
    static float pid_vx, pid_vy;
    static uint32_t move_angle = 0, last_x = 0, last_y = 0;
    if (omv[0].online == 1 && omv[0].raw_data.data_flushed == 1) {
        if (omv[0].raw_data.find == 1) {
            if (Block_delay.delay_star != 1) {
                Block_delay.delay_star = 1;
                Block_delay.ami_delay = 5000;
            }
            omv_decoupling(&omv[0], 20, fc_sta.fc_attitude.rol, fc_sta.fc_attitude.pit);
            omv[0].raw_data.data_flushed = 0;
            move_angle = (int) (my_atan(omv[0].block_track_data[0].offset_y_decoupled_lpf,
                                        omv[0].block_track_data[0].offset_x_decoupled_lpf) / 3.14 * 180);
            if ((ABS(last_x - omv[0].block_track_data[0].offset_x_decoupled_lpf) < 10) &&
                ABS((last_y - omv[0].block_track_data[0].offset_y_decoupled_lpf) < 10) && distance < 25) {
                if (Block_delay.delay_finished == 1)
                    return Mission_finish;
            } else {
                Block_delay.now_delay = 0;
            }
            distance= my_2_norm(omv[0].block_track_data[0].offset_y_decoupled_lpf,omv[0].block_track_data[0].offset_x_decoupled_lpf);
            if (distance > 10) {
                pid_vy = PID_PositionalRealize(&PID_Positional_vy,
                                               omv[0].block_track_data[0].offset_y_decoupled_lpf, 0);
                pid_vx = PID_PositionalRealize(&PID_Positional_vx, omv[0].block_track_data[0].offset_x_decoupled_lpf,
                                               0);
                printf("vx:%f,vy:%f\r\n",pid_vx,pid_vy);
                speed = my_2_norm(pid_vy,pid_vx);
                if (move_angle >= 0) {
                    Horizontal_Move(0.3 * speed, speed, 360 - move_angle);
                } else if (move_angle < 0) {
                    Horizontal_Move(0.3 * speed, speed, -move_angle);
                }
    }
            Unfind_time = 0;
            printf("de block x:%.2f y:%.2f\r\n", omv[0].block_track_data[0].offset_x_decoupled_lpf,
                   omv[0].block_track_data[0].offset_y_decoupled_lpf);
            printf("angle :%d speed :%d \r\n", (int) move_angle, speed);
            last_x = (int) omv[0].block_track_data[0].offset_x_decoupled_lpf;
            last_y = (int) omv[0].block_track_data[0].offset_y_decoupled_lpf;
        } else if (omv[0].raw_data.find == 0) {
            Unfind_time++;
            printf("unfind\r\n");
            if (Unfind_time == 20)
                OneKey_Hover();
            if (Unfind_time >= 80)
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
    if (omv[0].online == 1 && omv[0].raw_data.data_flushed == 1) {
        if (omv[0].raw_data.find == 1) {
            omv_decoupling(&omv[0], 20, fc_sta.fc_attitude.rol, fc_sta.fc_attitude.pit);
            omv[0].raw_data.data_flushed = 0;
            if (omv[0].raw_data.type == OMV_DATA_LINE || omv[0].raw_data.type == OMV_DATA_BOTH) {
//                pid_angle = PID_PositionalRealize(&PID_PositionalLine_angle, omv[0].raw_data.line.angle, 0);
//                pid_vy = PID_PositionalRealize(&PID_PositionalLine_vy,
//                                               omv[0].line_track_data.offset_decoupled_lpf, 0);
                if ((ABS(omv[0].raw_data.line[0].angle) > 5) || (ABS(omv[0].line_track_data[0].offset_decoupled_lpf) > 5)) {
                    move_angle = (int) (ABS(omv[0].raw_data.line[0].angle) + atan2(ABS(pid_vy), 3) / 3.14 * 180);
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
                if (omv[0].raw_data.type == OMV_DATA_BOTH) {
                    Block_delay.delay_star = 1;
                    Block_delay.ami_delay = 3000;
                    if (Block_delay.delay_finished == 1)
                        return Mission_finish;
                }
                printf("line offset:%d angle:%d\r\n", (int) omv[0].raw_data.line[0].offset, omv[0].raw_data.line[0].angle);
                printf("de line offset:%.2f\r\n", omv[0].line_track_data[0].offset_decoupled_lpf);
                printf("pid_vy:%.2f pid_angle:%.2f\r\n", pid_vy, pid_angle);
            } else if (omv[0].raw_data.type == OMV_DATA_BLOCK) {
                Block_delay.delay_star = 1;
                Block_delay.ami_delay = 3000;
                if (Block_delay.delay_finished == 1) {
                    Block_delay.delay_star = 0;
                    return Mission_finish;
                } else {
                    Horizontal_Move(30, 20, 0);
                    Left_Rotate(0, 0);
                }
//                        printf("block x:%d y:%d\r\n",(int)omv[0].raw_data.block.center_x,(int)omv[0].raw_data.block.center_y);
//                        printf("de block x:%.2f y:%.2f\r\n",omv[0].block_track_data.offset_x_decoupled_lpf,omv[0].block_track_data.offset_y_decoupled_lpf);
            }
        } else if (omv[0].raw_data.find == 0) {
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
    if (Unlock_delay.delay_star == 0) {
        if (FC_Unlock())
            Unlock_delay.delay_star = 1;
        Unlock_delay.ami_delay = 3000;
    } else if (Unlock_delay.delay_finished == 1 && Takeoff_delay.delay_star == 0) {
        if(OneKey_Takeoff(0)) //参数单位：厘米； 0：默认上位机设置的高度。
            Takeoff_delay.delay_star = 1;
        Takeoff_delay.ami_delay = 3000;
    } else if (Takeoff_delay.delay_finished == 1) {
        Unlock_delay.delay_star = 0;
        Takeoff_delay.delay_star = 0;
        return Mission_finish;
    }
    return Mission_Unfinished;
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
            if( CircularMotion(Hz,60,6,720,180,1) == 1 )
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
            if( CircularMotion(Hz,60,6,720,180,0) == 1 )
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

//利用灵活格式帧将数据发送到匿名上位机
void DataSendToPC(uint16_t Hz)
{
    Send_User_Data(0XF1,sizeof(Sensor_Data),&Sensor_Data);
    Send_User_Data(0XF2,sizeof(Sensor_State),&Sensor_State);
//    Send_User_Data(0XF3,1,BufferU32);
//    Send_User_Data(0XF4,1,BufferS16);
//    Send_User_Data(0XF5,1,BufferS32);
}

//Y轴移动，检测到杆后停下来 (任务频率，方向_0负1正，要检测的值_当不等于0时)
uint8_t Y_axisDetect(uint16_t Hz,uint8_t direction,uint16_t detect_value,uint16_t speed)
{
    if(detect_value==0)
    {
        Horizontal_Move(speed/Hz,speed,90+direction*180);
        return 0;
    }
    else if(detect_value>0)
    {
        OneKey_Hover();
        return 1;
    }
}

//Y轴调整，根据坐标将机体对准目标，并附加偏移量（任务频率，期望坐标，反馈坐标，偏移量，允许误差,坐标变换_0:与机体坐标系相同_1:与机体坐标系相反,比例系数，积分系数）
uint8_t Y_axisAdjust(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t offset,uint16_t allow_err,uint8_t coordinate_change,float kp,float ki)
{
    float SpeedErr=0;
    static float Speed_I=0;
    float Speed=0;

    if(ABS(fb-ex)>allow_err)
    {
        if(coordinate_change==1)
        {
            SpeedErr=(float)(fb-ex-offset);
            Speed_I+=Speed/(float)Hz;
            Speed_I=LIMIT(Speed_I,-15,15);
            Speed=kp*SpeedErr+ki*Speed_I;
            Speed=LIMIT(Speed,0,15);

            if(Speed>0)
            {
                Horizontal_Move((uint16_t)(Speed/(float)Hz),(uint16_t)Speed,90);
            }
            else
            {
                Speed=ABS(Speed);
                Horizontal_Move((uint16_t)(Speed/(float)Hz),(uint16_t)Speed,270);
            }
        }
        else if(coordinate_change==0)
        {
            SpeedErr=(float)(fb-ex-offset);
            Speed_I+=Speed/(float)Hz;
            Speed_I=LIMIT(Speed_I,-15,15);
            Speed=kp*SpeedErr+ki*Speed_I;
            Speed=LIMIT(Speed,0,15);

            if(Speed>0)
            {
                Horizontal_Move((uint16_t)(Speed/(float)Hz),(uint16_t)Speed,270);
            }
            else
            {
                Speed=ABS(Speed);
                Horizontal_Move((uint16_t)(Speed/(float)Hz),(uint16_t)Speed,90);
            }
        }
        return 0;
    }
    else
    {
        Speed_I=0;
        return 1;
    }
}

//低通滤波
double LowPassFilter(float now_data,float last_data,float low_pass_coefficient)
{
    return now_data*low_pass_coefficient+last_data*(1-low_pass_coefficient);
}

//航向调整
void HeadAdjust(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t speed)
{
    if( ABS(ex-fb)<180 )
    {
        if(ex>fb)
        {
            Right_Rotate(ex-fb,LIMIT(speed,50,90));
        }
        else
        {
            Left_Rotate(fb-ex,LIMIT(speed,50,90));
        }
    }
    else
    {
        if(ex>fb)
        {
            Left_Rotate(ex-fb,LIMIT(speed,50,90));
        }
        else
        {
            Right_Rotate(fb-ex,LIMIT(speed,50,90));
        }
    }
}

//X轴移动命令，根据目标距离和期望距离计算出移动距离(任务频率，期望距离，反馈距离,移动速度)
void X_axisMove(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t speed)
{
    if( (ex-fb)<0 )
    {
        Horizontal_Move(LIMIT(ABS(ex-fb),0,40),speed,0);
    }
    else
    {
        Horizontal_Move(LIMIT(ABS(ex-fb),0,40),speed,180);
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
            OneKey_Hover();
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
            OneKey_Hover();
            return 1;
        }
    }
    else
    {
        return 0;
    }
}

//X轴移动，检测到圆后停下来 (任务频率，方向_0负1正，要检测的值_当不等于0时)
uint8_t X_axisDetect(uint16_t Hz,uint8_t direction,uint16_t detect_value,uint16_t speed)
{
    if(detect_value==0)
    {
        Horizontal_Move(speed/Hz,speed,180-180*direction);
        return 0;
    }
    else if(detect_value>0)
    {
        OneKey_Hover();
        return 1;
    }
}

//位置调节(任务频率，x轴期望值，y轴期望值，x轴反馈值，y轴反馈值，允许误差范围，坐标变换_0:与机体坐标系相同_1:与机体坐标系相反,比例系数，积分系数)
uint8_t PositionAdjust(uint16_t Hz,uint16_t x_ex,uint16_t y_ex,uint16_t x_fb,uint16_t y_fb,uint16_t allow_err,uint8_t coordinate_change,float kp,float ki)
{
    float XSpeedErr=0;
    static float XSpeed_I=0;
    float XSpeed=0;

    float YSpeedErr=0;
    static float YSpeed_I=0;
    float YSpeed=0;

    float Speed=0;
    float Angle=0;

    if( (ABS(x_fb-x_ex)>allow_err) || (ABS(y_fb-y_ex)>allow_err) )
    {
        if(coordinate_change==1)
        {
            XSpeedErr=(float)(x_ex-x_fb);
            XSpeed_I+=XSpeed/(float)Hz;
            XSpeed_I=LIMIT(XSpeed_I,-15,15);
            XSpeed=kp*XSpeedErr+ki*XSpeed_I;

            YSpeedErr=(float)(y_ex-y_fb);
            YSpeed_I+=YSpeed/(float)Hz;
            YSpeed_I=LIMIT(YSpeed_I,-15,15);
            YSpeed=kp*YSpeedErr+ki*YSpeed_I;

            Speed=LIMIT( my_2_norm(XSpeed,YSpeed) , 0,15);

            if( (XSpeed==0) && (YSpeed==0) )
            {
                OneKey_Hover();
            }
            else
            {
                Angle=(float)atan2((double)XSpeed,(double)YSpeed)*57.29578f;
                if(Angle<=0)
                {
                    Angle=-1*Angle;
                }
                else
                {
                    Angle=360-Angle;
                }
                Horizontal_Move((uint16_t)(Speed/(float)Hz),(uint16_t)Speed,(uint16_t)Angle);
            }
            return 0;
        }
        else if(coordinate_change==0)
        {
            XSpeedErr=(float)(x_fb-x_ex);
            XSpeed_I+=XSpeed/(float)Hz;
            XSpeed_I=LIMIT(XSpeed_I,-15,15);
            XSpeed=kp*XSpeedErr+ki*XSpeed_I;

            YSpeedErr=(float)(y_fb-y_ex);
            YSpeed_I+=YSpeed/(float)Hz;
            YSpeed_I=LIMIT(YSpeed_I,-15,15);
            YSpeed=kp*YSpeedErr+ki*YSpeed_I;

            Speed=LIMIT( my_2_norm(XSpeed,YSpeed) , 0,15);

            if( (XSpeed==0) && (YSpeed==0) )
            {
                OneKey_Hover();
            }
            else
            {
                Angle=(float)atan2((double)XSpeed,(double)YSpeed)*57.29578f;
                if(Angle<=0)
                {
                    Angle=-1*Angle;
                }
                else
                {
                    Angle=360-Angle;
                }
                Horizontal_Move((uint16_t)(Speed/(float)Hz),(uint16_t)Speed,(uint16_t)Angle);
            }
            return 0;
        }
        return 0;
    }
    else
    {
        XSpeed_I=0;
        YSpeed_I=0;
        return 1;
    }
}

void Task_2020(uint16_t Hz)
{
    static uint16_t State=0;
    static uint16_t Yaw0=0;

    if(fc_sta.unlock_cmd==1 && rc_in.rc_ch.st_data.ch_[ch_5_aux1]>1800 && rc_in.rc_ch.st_data.ch_[ch_5_aux1]<2200)  //如果解锁且处于程控模式
    {
        if(State==0)
        {
            OneKey_Takeoff(100);
            State++;
        }
        else if(State==1)
        {
            Yaw0=(uint16_t)fc_sta.fc_attitude.yaw;    //记录起飞后的航向角
            Wait(Hz,3,&State);
        }
        else if(State==2)
        {
//            if( Y_axisDetect(Hz,1,image_center,10) )  //向左移动，等待检测到图像
//            {
//                State++;
//            }
        }
        else if(State==3)
        {
            if( Y_axisAdjust(Hz,ImageCenter,0,0,2,0,0,0) )  //根据图像将飞机调到正对杆
            {
                State++;
            }
        }
        else if(State==4)
        {
            HeadAdjust(Hz,Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,60);  //航向归0
            State++;
        }
        else if(State==5)
        {
            Wait(Hz,3,&State);
        }
        else if(State==6)
        {
            X_axisMove(Hz,60,distance,10);    //将飞机移动到杆前方60cm处
            State++;
        }
        else if(State==7)
        {
            Wait(Hz,4,&State);
        }
        else if(State==8)
        {
            if( CircularMotion(Hz,60,4,660,180,0) == 1 ) //绕圈
            {
                State++;
            }
        }
        else if(State==9)
        {
            HeadAdjust(Hz,Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,60);  //航向归0
            State++;
        }
        else if(State==10)
        {
            Wait(Hz,3,&State);
        }
        else if(State==11)
        {
//            if( Y_axisDetect(Hz,1,image_center,10) )   //向左移动，等待检测到图像
//            {
//                State++;
//            }
        }
        else if(State==12)
        {
            if( Y_axisAdjust(Hz,ImageCenter,0,0,2,0,0,0) )  //根据图像将飞机调到正对杆
            {
                State++;
            }
        }
        else if(State==13)
        {
            HeadAdjust(Hz,Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,60);  //航向归0
            State++;
        }
        else if(State==14)
        {
            Wait(Hz,3,&State);
        }
        else if(State==15)
        {
            X_axisMove(Hz,60,distance,10);    //将飞机移动到杆前方60cm处
            State++;
        }
        else if(State==16)
        {
            Wait(Hz,4,&State);
        }
        else if(State==17)
        {
            if( CircularMotion(Hz,60,4,400,180,1) == 1 ) //绕圈
            {
                State++;
            }
        }
        else if(State==18)
        {
            HeadAdjust(Hz,Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,60);  //航向归0
            State++;
        }
        else if(State==19)
        {
            Wait(Hz,3,&State);
        }
        else if(State==20)
        {
//            if( X_axisDetect(Hz,0,Image,10) )
//            {
//                State++;
//            }
        }
        else if(State==21)
        {
//            if( PositionAdjust(Hz,ImageCenter,ImageCenter,xfb,yfb,2,0,0,0) )
//            {
//                State++;
//            }
        }
        else if(State==22)
        {
            OneKey_Land();
            State++;
        }
        else if(State==23)
        {
        }
        else if(State==24)
        {
        }
    }
}
