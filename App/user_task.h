//
// Created by wirano on 2021/4/28.
//

#ifndef USER_TASK_H
#define USER_TASK_H

#include "fc_config.h"
#define process_dt_ms   20
#define Mission_finish  1
#define Mission_Unfinished  0
#define delay_finish    1
#define delay_Unfinished    0
#define LX_LED  0
#define USER_LED    1
#define NONE    0
#define RGB_R   1
#define RGB_G   2
#define RGB_B   3
#define ALL 4
#define Mission_over 99

#define ImageCenter 320

typedef struct
{
    uint16_t Distance;
    uint16_t ImageX;
    uint16_t ImageY;
}__attribute__ ((__packed__)) SensorData;

typedef struct
{
    uint8_t TofState;
    uint8_t ImageState;
}__attribute__ ((__packed__)) SensorState;

extern void onekey_lock(void);

void one_key_takeoff_land();

void light_check(uint8_t group, uint8_t color);

uint8_t fly(uint16_t distance_cm, uint16_t velocity, uint16_t dir_angle_0_360);

void process_control();

uint8_t user_takeoff();

uint8_t process_delay(uint16_t delay_ms);

void fly_s();

uint8_t omv_find_detection();

void MyProcessTest(uint16_t Hz);

//利用灵活格式帧将数据发送到匿名上位机
void DataSendToPC(uint16_t Hz);

//Y轴移动，检测到杆后停下来 (任务频率，方向_0负1正，要检测的值_当不等于0时)
uint8_t Y_axisDetect(uint16_t Hz,uint8_t direction,uint16_t detect_value,uint16_t speed);

//Y轴调整，根据坐标将机体对准目标，并附加偏移量（任务频率，期望坐标，反馈坐标，偏移量，允许误差,坐标变换_0:与机体坐标系相同_1:与机体坐标系相反,比例系数，积分系数）
uint8_t Y_axisAdjust(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t offset,uint16_t allow_err,uint8_t coordinate_change,float kp,float ki);

//低通滤波
double LowPassFilter(float now_data,float last_data,float low_pass_coefficient);

//航向调整
void HeadAdjust(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t speed);

//X轴移动命令，根据目标距离和期望距离计算出移动距离(任务频率，期望距离，反馈距离，移动速度)
void X_axisMove(uint16_t Hz,uint16_t ex,uint16_t fb,uint16_t speed);

//圆周运动 (任务频率，半径cm，转速_转/min，旋转角度，初相位_0到360度，转向_0逆1顺)
uint8_t CircularMotion(uint16_t Hz,uint16_t r_cm,uint16_t speed_r_min,uint16_t all_angle,uint16_t ini_phase,uint8_t direction);

//X轴移动，检测到圆后停下来 (任务频率，方向_0负1正，要检测的值_当不等于0时)
uint8_t X_axisDetect(uint16_t Hz,uint8_t direction,uint16_t detect_value,uint16_t speed);

//位置调节(任务频率，x轴期望值，y轴期望值，x轴反馈值，y轴反馈值，允许误差范围，坐标变换_0:与机体坐标系相同_1:与机体坐标系相反,比例系数，积分系数)
uint8_t PositionAdjust(uint16_t Hz,uint16_t x_ex,uint16_t y_ex,uint16_t x_fb,uint16_t y_fb,uint16_t allow_err,uint8_t coordinate_change,float kp,float ki);

void Task_2020(uint16_t Hz);
#endif //USER_TASK_H
