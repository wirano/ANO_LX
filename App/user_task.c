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

SensorDataSt SensorData=
        {
            .Distance=0,
            .ImageX=0,
            .ImageY=0,
        };
SensorStateSt SensorState=
        {
            .TofState=0,
            .ImageState=0,
        };

FlyStateSt FlyState=
        {
            .Mode=0,
            .Yaw0=0,
            .State=0,
            .FlyAngle=0,
            .FlySpeed=0,
            .Fly2Angle=0,
            .Fly2Speed=0,
        };

TempToPCSt TempToPC=
        {
            .Area=0,
            .Area2=0,
            .Area3=0,
            .ErrX=0,
            .ErrY=0,
        };

TargetMessageSt TargetMessage;

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

//void process_control() {
//    static uint16_t mission_flag = 0, mission_step = 0, last_mission_step;
//    static uint16_t ready = 0;
//    static uint8_t Mission_state = Mission_Unfinished;
//    if (last_mission_step != mission_step)
//        printf("mission_step:%d\r\n", mission_step);
//    last_mission_step = mission_step;
//    if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 2000 && mission_flag == 0 && ready == 1) {
//        //进入程控模式
//        mission_flag = 1;
//        mission_step = 1;
//    } else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 1500) {
//        mission_flag = 0;
//        mission_step = 0;
//        ready = 1;
//        Takeoff_delay.delay_star = 0;
//        Unlock_delay.delay_star = 0;
//        Block_delay.delay_star = 0;
//    }
//
//    if (mission_flag == 1) {
//        buzzer.freq = 5;
////        omv_led_state.led_num = 4;
//    } else {
//        buzzer.freq = 200;
////        omv_led_state.led_num = 0;
//    }//BEEP SWITCH
//    if (mission_flag == 1) {
//        printf("state:%d\r\n",omv.raw_data.find);
//        if (mission_step == 1) {
//            if (user_takeoff() == 1) {
//                mission_step++;
//            }
//        } //程控起飞
////        else if (mission_step == 2) {
////            Horizontal_Move(200, 15, 290);
////            mission_step=6;
////        } //
//        else if (mission_step == 2) {
////            Mission_state = omv_find_blobs();
//            switch (Mission_state) {
//                case Mission_finish:
//                        mission_step++;
//                    break;
//                case Mission_err:
//                    printf("err!!!!!!!!\r\n");
//                    mission_step = Mission_over;
//                    break;
//                case Mission_Unfinished:
//                    break;
//                }
//        } else if (mission_step == 3) {
//            if (Land_delay.delay_star == 0) {
//                Horizontal_Move(40, 15, 0);
//                Land_delay.delay_star = 1;
//                Land_delay.ami_delay = 1500;
//                }
//            if (Land_delay.delay_finished == 1)
//                mission_step = Mission_over;
//        } else if (mission_step == Mission_over) {
//            OneKey_Land();
//            ready = 0;
//            mission_flag = 0;
//            Takeoff_delay.delay_star = 0;
//            Unlock_delay.delay_star = 0;
//            Block_delay.delay_star = 0;
//            Land_delay.delay_star = 0;
//        }
//    }
//}

//uint8_t omv_find_detection() {
//    static uint16_t omv_lose;
//    if (omv.online == 1 && omv.raw_data.find == 0) {
//        omv_lose++;
//    }
//    if (omv.online == 1 && omv.raw_data.find == 1) {
//            omv_lose = 0;
//        }
//    if (omv.online == 0) {
//        omv_lose++;
//    }
//    if (omv_lose > (1000 / process_dt_ms)) {
//        return Mission_over;
//    }
//    return 0;
//}

//uint8_t omv_find_blobs() {
//    static uint8_t Unfind_time = 0, speed, distance;
//    static float pid_vx, pid_vy;
//    static uint32_t move_angle = 0, last_x = 0, last_y = 0;
//    if (omv[0].online == 1 && omv[0].raw_data.data_flushed == 1) {
//        if (omv[0].raw_data.find == 1) {
//            if (Block_delay.delay_star != 1) {
//                Block_delay.delay_star = 1;
//                Block_delay.ami_delay = 5000;
//            }
//            omv_decoupling(&omv[0], 20, fc_sta.fc_attitude.rol, fc_sta.fc_attitude.pit);
//            omv[0].raw_data.data_flushed = 0;
//            move_angle = (int) (my_atan(omv[0].block_track_data[0].offset_y_decoupled_lpf,
//                                        omv[0].block_track_data[0].offset_x_decoupled_lpf) / 3.14 * 180);
//            if ((ABS(last_x - omv[0].block_track_data[0].offset_x_decoupled_lpf) < 10) &&
//                ABS((last_y - omv[0].block_track_data[0].offset_y_decoupled_lpf) < 10) && distance < 25) {
//                if (Block_delay.delay_finished == 1)
//                    return Mission_finish;
//            } else {
//                Block_delay.now_delay = 0;
//            }
//            distance= my_2_norm(omv[0].block_track_data[0].offset_y_decoupled_lpf,omv[0].block_track_data[0].offset_x_decoupled_lpf);
//            if (distance > 10) {
//                pid_vy = PID_PositionalRealize(&PID_Positional_vy,
//                                               omv[0].block_track_data[0].offset_y_decoupled_lpf, 0);
//                pid_vx = PID_PositionalRealize(&PID_Positional_vx, omv[0].block_track_data[0].offset_x_decoupled_lpf,
//                                               0);
//                printf("vx:%f,vy:%f\r\n",pid_vx,pid_vy);
//                speed = my_2_norm(pid_vy,pid_vx);
//                if (move_angle >= 0) {
//                    Horizontal_Move(0.3 * speed, speed, 360 - move_angle);
//                } else if (move_angle < 0) {
//                    Horizontal_Move(0.3 * speed, speed, -move_angle);
//                }
//    }
//            Unfind_time = 0;
//            printf("de block x:%.2f y:%.2f\r\n", omv[0].block_track_data[0].offset_x_decoupled_lpf,
//                   omv[0].block_track_data[0].offset_y_decoupled_lpf);
//            printf("angle :%d speed :%d \r\n", (int) move_angle, speed);
//            last_x = (int) omv[0].block_track_data[0].offset_x_decoupled_lpf;
//            last_y = (int) omv[0].block_track_data[0].offset_y_decoupled_lpf;
//        } else if (omv[0].raw_data.find == 0) {
//            Unfind_time++;
//            printf("unfind\r\n");
//            if (Unfind_time == 20)
//                OneKey_Hover();
//            if (Unfind_time >= 80)
//                return Mission_err;
//            else
//                return Mission_Unfinished;
//        }
//        return Mission_Unfinished;
//    }
//    if (omv_find_detection() == Mission_over) {
//        return Mission_err;
//    }
//    return Mission_Unfinished;
//}

uint8_t user_takeoff() {
    if (Unlock_delay.delay_star == 0) {
        if (FC_Unlock())
            Unlock_delay.delay_star = 1;
        Unlock_delay.ami_delay = 1500;
    } else if (Unlock_delay.delay_finished == 1 && Takeoff_delay.delay_star == 0) {
        if(OneKey_Takeoff(120)) //参数单位：厘米； 0：默认上位机设置的高度。
            Takeoff_delay.delay_star = 1;
        Takeoff_delay.ami_delay = 1000;
    } else if (Takeoff_delay.delay_finished == 1) {
        Unlock_delay.delay_star = 0;
        Takeoff_delay.delay_star = 0;
        return Mission_finish;
    }
    return Mission_Unfinished;
}

inline void onekey_lock(uint16_t Hz)
{
    static uint8_t reseted = 0;
    static uint16_t TimeCount=0;
    static uint16_t State=0;

//    if(State==0)
//    {
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
            State=1;
        } else if(!reseted) {
            if (fc_sta.unlock_sta || fc_sta.unlock_cmd) {
                FC_Lock();
            }
            fc_sta.esc_output_unlocked = 0;
        }
//    }
//    else if(State==1)
//    {
//        if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] == 2000)
//        {
//            fc_sta.esc_output_unlocked = 1;
//        }
//        else if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] == 1000)
//        {
//            State=2;
//        }
//    }
//    else if(State==2)
//    {
//        if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] == 2000)
//        {
//            State=3;
//        }
//    }
//    else if(State==3)
//    {
//        if (rc_in.rc_ch.st_data.ch_[ch_8_aux4] == 1000)
//        {
//            State=4;
//        }
//    }
//    else if(State==4)
//    {
//        if(TimeCount<=Hz*2)
//        {
//            if (fc_sta.unlock_sta || fc_sta.unlock_cmd) {
//                FC_Lock();
//            }
//            fc_sta.esc_output_unlocked = 0;
//            State=1;
//        }
//        else
//        {
//            State=1;
//        }
//    }
//
//    if(State>1)
//    {
//        if(TimeCount<=Hz*2) {
//            TimeCount++;
//        }
//        else{
//            State=1;
//        }
//    }
//    else
//    {
//        TimeCount=0;
//    }
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
            if( CircularMotion(Hz,70,3,360,210,0) == 1 )
            {
                State++;
            }
        }
        else if(State==3)
        {
            Wait(Hz,10,&State);
        }
        else if(State==4)
        {
//            if( CircularMotion(Hz,70,3,360,210,0) == 1 )
//            {
//                State++;
//            }
            State++;
        }
        else if(State==5)
        {
            Wait(Hz,1,&State);
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
    Send_User_Data(0XF1,sizeof(SensorData),&SensorData);
    Send_User_Data(0XF2,sizeof(SensorState),&SensorState);
    Send_User_Data(0XF3,sizeof(FlyState),&FlyState);
    Send_User_Data(0XF4,sizeof(TempToPC),&TempToPC);
//    Send_User_Data(0XF5,1,BufferS32);
}

//Y轴移动，检测到杆后停下来 (任务频率，方向_0负1正，要检测的值_当不等于0时)
uint8_t Y_axisDetect(uint16_t Hz,uint8_t direction,uint16_t detect_value,uint16_t speed)
{
    if(detect_value==0)
    {
        Horizontal_Move(LIMIT(speed/Hz,5,10),speed,90+direction*180);
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
    uint16_t CenterX=0;
    static uint16_t CenterXLast=0;

    if (ABS(fb - ex) > allow_err)
    {
        CenterX = (uint16_t) LowPassFilter(fb, CenterXLast, 0.8f);
        CenterXLast = CenterX;

        if (coordinate_change == 1)
        {
            SpeedErr = (float) (CenterX - ex - offset);
            Speed_I += Speed / (float) Hz;
            Speed_I = LIMIT(Speed_I, -10, 10);
            Speed = kp * SpeedErr + ki * Speed_I;
            Speed = LIMIT(Speed, -10, 10);

            if (Speed > 0)
            {
                Horizontal_Move((uint16_t) LIMIT((Speed / (float) Hz),5,10), (uint16_t) Speed, 90);
            } else
            {
                Speed = ABS(Speed);
                Horizontal_Move((uint16_t) LIMIT((Speed / (float) Hz),5,10), (uint16_t) Speed, 270);
            }
        }
        else if (coordinate_change == 0)
        {
            SpeedErr = (float) (CenterX - ex - offset);
            Speed_I += Speed / (float) Hz;
            Speed_I = LIMIT(Speed_I, -10, 10);
            Speed = kp * SpeedErr + ki * Speed_I;
            Speed = LIMIT(Speed, -10, 10);

            if (Speed > 0)
            {
                Horizontal_Move((uint16_t) LIMIT((Speed / (float) Hz),5,10), (uint16_t) Speed, 270);
                FlyState.FlyAngle = 90;
                FlyState.FlySpeed = (uint16_t) Speed;
            }
            else
            {
                Speed = ABS(Speed);
                Horizontal_Move((uint16_t) LIMIT((Speed / (float) Hz),5,10), (uint16_t) Speed, 90);
                FlyState.FlyAngle = 270;
                FlyState.FlySpeed = (uint16_t) Speed;
            }
        }
        return 0;
    }
    else
    {
        Speed_I = 0;
        CenterXLast=0;
        OneKey_Hover();
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
    ex+=180;
    fb+=180;
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
            Left_Rotate(360-ex+fb,LIMIT(speed,50,90));
        }
        else
        {
            Right_Rotate(360-fb+ex,LIMIT(speed,50,90));
        }
    }
}

//X轴移动命令，根据目标距离和期望距离计算出移动距离(任务频率，期望距离，反馈距离，移动速度,允许误差，校验时间)
uint8_t X_axisMove(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t allow_err,float kp,float ki)
{
    float XSpeedErr=0;
    static float XSpeed_I=0;
    float XSpeed=0;

    XSpeedErr=(float)(ex-fb);
    XSpeed_I+=XSpeed/(float)Hz;
    XSpeed_I=LIMIT(XSpeed_I,-10,10);
    XSpeed=kp*XSpeedErr+ki*XSpeed_I;
    LIMIT(XSpeed,-10,10);

    if( ABS(ex-fb)>allow_err)
    {
        if( (ex-fb)<0 )
        {
            Horizontal_Move(LIMIT(ABS(XSpeed)/Hz,3,15),(uint16_t)LIMIT(ABS(XSpeed),6,10),0);
        }
        else
        {
            Horizontal_Move(LIMIT(ABS(XSpeed)/Hz,3,15),(uint16_t)LIMIT(ABS(XSpeed),6,10),180);
        }
        return 0;
    }
    else
    {
        OneKey_Hover();
        XSpeed_I=0;
        return 1;
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
    float SpeedX=0;
    float SpeedY=0;

    if(direction==1)
    {
        if(Angle<(all_angle+ini_phase+90))
        {
            w_rad = MY_PPPIII * (float) speed_r_min / 30.0f;
            Speed = (float) r_cm * w_rad;
            Distance = LIMIT(Speed / (float) Hz,5,30);
            Angle = (uint16_t) (w_rad * 57.29578f * (float) T_Count / (float) Hz)+ini_phase+90;
            T_Count++;

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
            w_rad = MY_PPPIII * (float) speed_r_min / 30.0f;
            Speed = (float) r_cm * w_rad;
            Distance = LIMIT(Speed / (float) Hz,5,30);
            Angle =64980 - (uint16_t) (w_rad * 57.29578f * (float) T_Count / (float) Hz)+ini_phase+90;
            T_Count++;

            if( ((Angle%360)>=0 && (Angle%360)<=60) || ((Angle%360)>=300 && (Angle%360)<=360) )
            {
//                SpeedX=Speed*( cos( (Angle%360)*0.0174533 ) ) + 0;
//                SpeedY=-1*(Speed*( sin( (Angle%360)*0.0174533 ) ) + 0);
//                Speed=my_2_norm(SpeedX,SpeedY);
//                Distance = LIMIT(Speed / (float) Hz,5,30);
            }

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
        Horizontal_Move(LIMIT(speed/Hz,5,10),speed,180-180*direction);
        return 0;
    }
    else if(detect_value>0)
    {
        OneKey_Hover();
        return 1;
    }
}

//位置调节_机体坐标系(任务频率，x轴期望值，y轴期望值，x轴反馈值，y轴反馈值，允许误差范围，坐标变换_0:与机体坐标系相同_1:与机体坐标系相反,比例系数，积分系数)
uint8_t PositionAdjust(uint16_t Hz,uint16_t x_ex,uint16_t y_ex,uint16_t x_fb,uint16_t y_fb,uint16_t allow_err,uint8_t coordinate_change,float kp,float ki)
{
    float XCenter=0;
    static float XCenterLast=0;
    float XSpeedErr=0;
    static float XSpeed_I=0;
    float XSpeed=0;

    float YCenter=0;
    static float YCenterLast=0;
    float YSpeedErr=0;
    static float YSpeed_I=0;
    float YSpeed=0;

    float Speed=0;
    float Angle=0;

    TempToPC.ErrX=x_fb-x_ex;
    TempToPC.ErrY=y_fb-y_ex;

    if( (((uint16_t)ABS(x_fb-x_ex)>allow_err) || ((uint16_t)ABS(y_fb-y_ex)>allow_err)) )
    {
        if( (coordinate_change==1) && omv[OMV_LAND_ID].raw_data.find )
        {
            XCenter=(uint16_t)LowPassFilter(x_fb,XCenterLast,0.7f);
            XCenterLast=XCenter;
            YCenter=(uint16_t)LowPassFilter(y_fb,YCenterLast,0.7f);
            YCenterLast=YCenter;

            XSpeedErr=(float)((float)x_ex-XCenter);
            XSpeed_I+=XSpeed/(float)Hz;
            XSpeed_I=LIMIT(XSpeed_I,-10,10);
            XSpeed=kp*XSpeedErr+ki*XSpeed_I;

            YSpeedErr=(float)((float)y_ex-YCenter);
            YSpeed_I+=YSpeed/(float)Hz;
            YSpeed_I=LIMIT(YSpeed_I,-10,10);
            YSpeed=kp*YSpeedErr+ki*YSpeed_I;

            Speed=LIMIT( my_2_norm(XSpeed,YSpeed) , 3,10);

            if( (XSpeed==0) && (YSpeed==0) )
            {
//                OneKey_Hover();
            }
            else
            {
                Angle=(float)atan2((double)YSpeed,(double)XSpeed)*57.29578f;
                if(Angle<=0)
                {
                    Angle=-1*Angle;
                }
                else
                {
                    Angle=360-Angle;
                }
                Horizontal_Move(LIMIT((uint16_t)(Speed/(float)Hz),5,10),(uint16_t)Speed,(uint16_t)Angle);
                FlyState.FlyAngle=(uint16_t)Angle;      //速度、角度信息获取后发送到上位机
                FlyState.FlySpeed=(uint16_t)Speed;
            }
            return 0;
        }
        else if( (coordinate_change==0) && omv[OMV_LAND_ID].raw_data.find )
        {
            XCenter=(uint16_t)LowPassFilter(x_fb,XCenterLast,0.7f);
            XCenterLast=XCenter;
            YCenter=(uint16_t)LowPassFilter(y_fb,YCenterLast,0.7f);
            YCenterLast=YCenter;

            XSpeedErr=(float)(XCenter-(float)x_ex);
            XSpeed_I+=XSpeed/(float)Hz;
            XSpeed_I=LIMIT(XSpeed_I,-10,10);
            XSpeed=kp*XSpeedErr+ki*XSpeed_I;

            YSpeedErr=(float)(YCenter-(float)y_ex);
            YSpeed_I+=YSpeed/(float)Hz;
            YSpeed_I=LIMIT(YSpeed_I,-10,10);
            YSpeed=-1*(kp*YSpeedErr+ki*YSpeed_I);

            Speed=LIMIT( my_2_norm(XSpeed,YSpeed) , 3,10);

            if( (XSpeed==0) && (YSpeed==0) )
            {
//                OneKey_Hover();
            }
            else
            {
                Angle=(float)atan2((double)YSpeed,(double)XSpeed)*57.29578f;
                if(Angle<0)
                {
                    Angle=360+Angle;
                }
                Horizontal_Move(LIMIT((uint16_t)(Speed/(float)Hz),5,10),(uint16_t)Speed,(uint16_t)Angle);
                FlyState.FlyAngle=(uint16_t)Angle;      //速度、角度信息获取后发送到上位机
                FlyState.FlySpeed=(uint16_t)Speed;
            }
            return 0;
        }
        return 0;
    }
    else
    {
        XSpeed_I=0;
        YSpeed_I=0;
        XCenterLast=0;
        YCenterLast=0;
        OneKey_Hover();
        return 1;
    }
}

void Task_2020(uint16_t Hz)
{
    static uint16_t State=0;
    static uint16_t mission_flag = 0;
    static uint16_t ready = 0;

    FlyState.State=State;

    if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 2000 && mission_flag == 0 && ready == 1)
    {
        //进入程控模式
        mission_flag = 1;
        State = 0;
        ready=0;
    }
    else if (rc_in.rc_ch.st_data.ch_[ch_5_aux1] == 1500)
    {
        mission_flag = 0;
        State = 0;
        ready = 1;
    }

    if(mission_flag && omv[OMV_BAR_ID].online && omv[OMV_LAND_ID].online)  //如果解锁且处于程控模式
    {
        if(State==0)
        {
            if( user_takeoff()==1 )
            {
                State++;
            }
        }
        else if(State==1)
        {
            FlyState.Yaw0=(int32_t)fc_sta.fc_attitude.yaw;    //记录起飞后的航向角
            Wait(Hz,3,&State);
        }
        else if(State==2)
        {
//            State=22;
            if( Y_axisDetect(Hz,1,omv[OMV_BAR_ID].raw_data.block_num,10) )  //向左移动，等待检测到图像
            {
                State++;
            }
        }
        else if(State==3)
        {
            FlyState.Mode=ModeJudge(Hz,omv[OMV_BAR_ID].raw_data.block,PixelsNumThr150,TargetMessage.Target1DetectDirection,8);
            if( CheckTime_s(Hz,2,0,FlyState.Mode,0)==1 )
            {
                State++;
            }
        }
        else if(State==4)
        {
            if( GoToTarget(Hz,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target1Index].area,PixelsNumThr80,ImageCenterX,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target1Index].center_x)==1 ) //运动到目标前方约80cm处
            {
                State++;
            }
        }
        else if(State==5)
        {
            if( CheckTime_s(Hz,0,0.5f,Y_axisAdjust(Hz,ImageCenterX,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target1Index].center_x,0,20,1,0.2f,0.2f),1)==1 )  //根据图像将飞机调到正对杆
            {
                State++;
            }
        }
        else if(State==6)
        {
            if( ABS(omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target1Index].center_x-ImageCenterX)>20 )
            {
                State=5;
            }
            else
            {
                if( CheckTime_s(Hz,0.3f,1,X_axisMove(Hz,57,distance,2,0.8f,0.4f),1)==1 )    //将飞机移动到杆前方57cm处
                {
                    State++;
                }
            }
        }
        else if(State==7)
        {
            HeadAdjust(Hz,FlyState.Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,50);
            State++;
        }
        else if(State==8)
        {
            Wait(Hz,2,&State);
        }
        else if(State==9)
        {
            if( CircularMotion(Hz,70,3,TargetMessage.Target1CircleAngle,180,TargetMessage.Target1CircleDirection) == 1 ) //绕圈
            {
                State++;
            }
        }
        else if(State==10)
        {
            if(FlyState.Mode==1 || FlyState.Mode==2)
            {
                State++;
            }
            else if(FlyState.Mode==3 || FlyState.Mode==4)
            {
                Left_Rotate(180,60);
            }
        }
        else if(State==11)
        {
            if(FlyState.Mode==1 || FlyState.Mode==2)
            {
                State++;
            }
            else if(FlyState.Mode==3 || FlyState.Mode==4)
            {
                Wait(Hz,4,&State);
            }
        }
        else if(State==12)
        {
            if( Y_axisDetect(Hz,TargetMessage.Target2DetectDirection,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target2Index].center_x,10) )   //移动，等待检测到图像
            {
                State++;
            }
        }
        else if(State==13)
        {
            if( GoToTarget(Hz,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target2Index].area,PixelsNumThr80,ImageCenterX,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target2Index].center_x)==1 ) //运动到目标前方约80cm处
            {
                State++;
            }
        }
        else if(State==14)
        {
            if( CheckTime_s(Hz,0,0.5f,Y_axisAdjust(Hz,ImageCenterX,omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target2Index].center_x,0,20,1,0.2f,0.2f),1)==1 )  //根据图像将飞机调到正对杆
            {
                State++;
            }
        }
        else if(State==15)
        {
            if( ABS(omv[OMV_BAR_ID].raw_data.block[TargetMessage.Target2Index].center_x-ImageCenterX)>20 )
            {
                State=14;
            }
            else
            {
                if( CheckTime_s(Hz,1,0.3f,X_axisMove(Hz,57,distance,4,0.8f,0.4f),1)==1 )    //将飞机移动到杆前方57cm处
                {
                    State++;
                }
            }
        }
        else if(State==16)
        {
            if(FlyState.Mode==1 || FlyState.Mode==2)
            {
                HeadAdjust(Hz,FlyState.Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,50);
            }
            else if(FlyState.Mode==3 || FlyState.Mode==4)
            {
                if(FlyState.Yaw0>0)
                {
                    HeadAdjust(Hz,-180+FlyState.Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,50);
                }
                else
                {
                    HeadAdjust(Hz,180+FlyState.Yaw0,(uint16_t)fc_sta.fc_attitude.yaw,50);
                }
            }
            State++;
        }
        else if(State==17)
        {
            Wait(Hz,2,&State);
        }
        else if(State==18)
        {
            if( CircularMotion(Hz,70,3,TargetMessage.Target2CircleAngle,180,TargetMessage.Target2CircleDirection) == 1 ) //绕圈
            {
                State++;
            }
        }
        else if(State==19)
        {
            Wait(Hz,2,&State);
        }
//        else if(State==20)
//        {
//            OneKey_Land();
//            State++;
//        }

        else if(State==20)
        {
            if(FlyState.Mode==1 || FlyState.Mode==2)
            {
                State++;
            }
            else if(FlyState.Mode==3 || FlyState.Mode==4)
            {
                Right_Rotate(180,60);
            }
        }
        else if(State==21)
        {
            if(FlyState.Mode==1 || FlyState.Mode==2)
            {
                State++;
            }
            else if(FlyState.Mode==3 || FlyState.Mode==4)
            {
                Wait(Hz,4,&State);
            }
        }
        else if(State==22)
        {
            if( X_axisDetect(Hz,0,omv[OMV_LAND_ID].raw_data.block[0].center_x,10) )
            {
                State++;
            }
        }
        else if(State==23)
        {
            if( CheckTime_s(Hz,0.4f,1,PositionAdjust(Hz,ImageCenterY,ImageCenterX-35,omv[OMV_LAND_ID].block_track_data->offset_y_decoupled,omv[OMV_LAND_ID].block_track_data->offset_x_decoupled,40,0,0.05f,0.35f),1) )
            {
                State++;
            }
        }
        else if(State==24)
        {
            Wait(Hz,3,&State);
        }
        else if(State==25)
        {
            OneKey_Land();
            State++;
        }
    }
    else if(omv[OMV_BAR_ID].online ==0 || omv[OMV_LAND_ID].online ==0 )
    {
        OneKey_Land();
    }

    if(State>=22)
    {
        for (int i = 0; i < 4; ++i) {
            omv[OMV_BAR_ID].raw_data.block[i].shape = 0;
            omv[OMV_BAR_ID].raw_data.block[i].center_x =0;
            omv[OMV_BAR_ID].raw_data.block[i].center_y =0;
            omv[OMV_BAR_ID].raw_data.block[i].area = 0;
            omv[OMV_BAR_ID].raw_data.block[i].color = 0;
            omv[OMV_LAND_ID].raw_data.block[i].shape = 0;
            omv[OMV_LAND_ID].raw_data.block[i].center_x =0;
            omv[OMV_LAND_ID].raw_data.block[i].center_y =0;
            omv[OMV_LAND_ID].raw_data.block[i].area = 0;
            omv[OMV_LAND_ID].raw_data.block[i].color = 0;
        }
    }
}

//根据最右方杆子的像素点个数判断两个杆的摆放位置
uint8_t ModeJudge(uint16_t Hz,_omv_block_st *block_data,uint32_t pixels_num_thr,uint8_t direction,uint16_t speed)
{
    uint8_t RightBarIndex=0;

//    printf("Y=%d\r\n",block_data->center_y);
//    if( ABS(block_data->center_y - ImageCenterY)<30 )  //判断Y坐标是否在中心
//    {
    if( (block_data->center_x) >= ((block_data+1)->center_x) )
    {
        RightBarIndex=0;
    }
    else
    {
        RightBarIndex=1;
    }

    if( ((block_data+RightBarIndex)->area) < pixels_num_thr )   //右侧杆距离较远
    {
        if( ((block_data+RightBarIndex)->color) == OMV_COLOR_RED )  //右侧杆为红色
        {
            TargetMessage.Target1Color=OMV_COLOR_RED;
            TargetMessage.Target2Color=OMV_COLOR_GREEN;
            TargetMessage.Target1CircleAngle=375;
            TargetMessage.Target2CircleAngle=450;
            TargetMessage.Target1Index=2;
            TargetMessage.Target2Index=3;
            TargetMessage.Target1CircleDirection=1;
            TargetMessage.Target2CircleDirection=0;
            TargetMessage.Target1DetectDirection=1;
            TargetMessage.Target2DetectDirection=0;
            return 3;
        }
        else if( ((block_data+RightBarIndex)->color) ==OMV_COLOR_GREEN)  //右侧杆为绿色
        {
            TargetMessage.Target1Color=OMV_COLOR_GREEN;
            TargetMessage.Target2Color=OMV_COLOR_RED;
            TargetMessage.Target1CircleAngle=375;
            TargetMessage.Target2CircleAngle=450;
            TargetMessage.Target1Index=3;
            TargetMessage.Target2Index=2;
            TargetMessage.Target1CircleDirection=0;
            TargetMessage.Target2CircleDirection=1;
            TargetMessage.Target1DetectDirection=1;
            TargetMessage.Target2DetectDirection=0;
            return 4;
        }
        else
        {
            return 0;
        }
    }
    else   //右侧杆距离较近
    {
        if( ((block_data+RightBarIndex)->color) == OMV_COLOR_RED )  //右侧杆为红色
        {
            TargetMessage.Target1Color=OMV_COLOR_RED;
            TargetMessage.Target2Color=OMV_COLOR_GREEN;
            TargetMessage.Target1CircleAngle=375;
            TargetMessage.Target2CircleAngle=375;
            TargetMessage.Target1Index=2;
            TargetMessage.Target2Index=3;
            TargetMessage.Target1CircleDirection=1;
            TargetMessage.Target2CircleDirection=0;
            TargetMessage.Target1DetectDirection=1;
            TargetMessage.Target2DetectDirection=1;
            return 1;
        }
        else if( ((block_data+RightBarIndex)->color) ==OMV_COLOR_GREEN)  //右侧杆为绿色
        {
            TargetMessage.Target1Color=OMV_COLOR_GREEN;
            TargetMessage.Target2Color=OMV_COLOR_RED;
            TargetMessage.Target1CircleAngle=375;
            TargetMessage.Target2CircleAngle=375;
            TargetMessage.Target1Index=3;
            TargetMessage.Target2Index=2;
            TargetMessage.Target1CircleDirection=0;
            TargetMessage.Target2CircleDirection=1;
            TargetMessage.Target1DetectDirection=1;
            TargetMessage.Target2DetectDirection=1;
            return 2;
        }
        else
        {
            return 0;
        }
    }
//    }
//    else
//    {
//        Horizontal_Move(LIMIT(speed/Hz,5,10),speed,90+direction*180);
//        return 0;
//    }
}

//向目标运动
uint8_t GoToTarget(uint16_t Hz,uint32_t fb_pixels_num,uint32_t pixels_num_thr,uint16_t ex_center_x,uint16_t fb_center_x)
{
    uint16_t CenterX=0;
    static uint16_t CenterXLast=0;

    CenterX=(uint16_t)LowPassFilter(fb_center_x,CenterXLast,0.8f);
    CenterXLast=CenterX;

    if( (fb_pixels_num)<pixels_num_thr )
    {
        if( ABS(CenterX-ex_center_x)<10 )   //如果中心对准，只需要向前飞行
        {
            FlyState.FlyAngle=0;
            FlyState.FlySpeed=10;
            Horizontal_Move(LIMIT(10/Hz,5,15),10,0);
            return 0;
        }
        else if( ABS(CenterX-ex_center_x)>150 )
        {
            if(CenterX>ex_center_x)
            {
                FlyState.FlyAngle=20;
                FlyState.FlySpeed=90;
                Horizontal_Move(LIMIT(15/Hz,5,15),10,90);
                return 0;
            }
            else
            {
                FlyState.FlyAngle=270;
                FlyState.FlySpeed=10;
                Horizontal_Move(LIMIT(15/Hz,5,15),10,270);
                return 0;
            }
        }
        else
        {
            if(CenterX>ex_center_x)
            {
                FlyState.FlyAngle=20;
                FlyState.FlySpeed=10;
                Horizontal_Move(LIMIT(15/Hz,5,15),10,20);
                return 0;
            }
            else
            {
                FlyState.FlyAngle=340;
                FlyState.FlySpeed=10;
                Horizontal_Move(LIMIT(15/Hz,5,15),10,340);
                return 0;
            }
        }
    }
    else
    {
        OneKey_Hover();
        CenterXLast=0;
        return 1;
    }
}

//校验某一个值是否在一段时间内等于某一个值
uint8_t CheckTime_s(uint16_t Hz,float check_s,uint16_t ex,uint8_t check_value,uint8_t direction)
{
    static uint16_t TimeCount=0;

    if(direction==1)
    {
        if( check_value==ex )
        {
            if( TimeCount<(Hz*check_s) )
            {
                TimeCount++;
                return 0;
            }
            else
            {
                TimeCount=0;
                return 1;
            }
        }
        else
        {
            TimeCount=0;
            return 0;
        }
    }
    else if(direction==0)
    {
        if( check_value!=ex )
        {
            if( TimeCount<(Hz*check_s) )
            {
                TimeCount++;
                return 0;
            }
            else
            {
                TimeCount=0;
                return 1;
            }
        }
        else
        {
            TimeCount=0;
            return 0;
        }
    }
    else
    {
        return 0;
    }
}