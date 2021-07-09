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
