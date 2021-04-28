//
// Created by wirano on 2021/4/28.
//

#include "board.h"
#include "drv_pwm_out.h"
#include "drv_led.h"
#include "drv_adc.h"
#include "drv_rc_in.h"
#include "drv_tim_lx.h"
#include "ano_lx_dt.h"
#include "drv_uart.h"

uint8_t All_Init()
{
    //LED功能初始化
    MX_GPIO_Init();
    //初始化电调输出功能
    DrvPwmOutInit();
    HAL_Delay(100);
    //串口2初始化，函数参数为波特率
    DrvUart2Init();
    //串口3初始化
    DrvUart3Init();
    //接匿名光流
    DrvUart4Init();
    //串口5接imu
    DrvUart5Init();
    HAL_Delay(100);
    //SBUS输入采集初始化
    DrvRcInputInit();
    //电池电压采集初始化
    DrvAdcInit();
    HAL_Delay(100);
    //数传模块初始化
    ANO_DT_Init();
    HAL_Delay(100);
    //GPS接口初始化
//    Init_GPS();
    //初始化定时中断
    DrvTimerFcInit();
    //初始化完成，返回1
    return (1);
}

_rc_input_st rc_in;
void DrvRcInputInit(void)
{
    //任意初始化一个模式
    DrvRcPpmInit();
    //DrvRcSbusInit();

    //先标记位丢失
    rc_in.no_signal = 1;
}
void DrvPpmGetOneCh(uint16_t data)
{
    static uint8_t ch_sta = 0;
    if ((data > 2100 && ch_sta > 3) || ch_sta == 10)
    {
        ch_sta = 0;
        rc_in.signal_cnt_tmp++;
        rc_in.rc_in_mode_tmp = 1; //切换模式标记为ppm
    }
    else if (data > 300 && data < 3000) //异常的脉冲过滤掉
    {
        //
        rc_in.ppm_ch[ch_sta] = data;
        ch_sta++;
    }
}
//void DrvSbusGetOneByte(uint8_t data)
//{
///*
//sbus flags的结构如下所示：
//flags：
//bit7 = ch17 = digital channel (0x80)
//bit6 = ch18 = digital channel (0x40)
//bit5 = Frame lost, equivalent red LED on receiver (0x20)
//bit4 = failsafe activated (0x10) b: 0001 0000
//bit3 = n/a
//bit2 = n/a
//bit1 = n/a
//bit0 = n/a
//*/
//    const uint8_t frame_end[4] = {0x04, 0x14, 0x24, 0x34};
//    static uint32_t sbus_time[2];
//    static uint8_t datatmp[25];
//    static uint8_t cnt = 0;
//    static uint8_t frame_cnt;
//    //
//    sbus_time[0] = sbus_time[1];
//    sbus_time[1] = GetSysRunTimeUs();
//    if ((uint32_t)(sbus_time[1] - sbus_time[0]) > 2500)
//    {
//        cnt = 0;
//    }
//    //
//    datatmp[cnt++] = data;
//    //
//    if (cnt == 25)
//    {
//        cnt = 24;
//        //
//        //if(datatmp[0] == 0x0F && (datatmp[24] == 0x00))
//        //if(datatmp[0] == 0x0F && ((datatmp[24] == 0x00)||(datatmp[24] == 0x04)||(datatmp[24] == 0x14)||(datatmp[24] == 0x24)||(datatmp[24] == 0x34)))
//        if ((datatmp[0] == 0x0F && (datatmp[24] == 0x00 || datatmp[24] == frame_end[frame_cnt])))
//        {
//            cnt = 0;
//            rc_in.sbus_ch[0] = (int16_t)(datatmp[2] & 0x07) << 8 | datatmp[1];
//            rc_in.sbus_ch[1] = (int16_t)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
//            rc_in.sbus_ch[2] = (int16_t)(datatmp[5] & 0x01) << 10 | ((int16_t)datatmp[4] << 2) | (datatmp[3] >> 6);
//            rc_in.sbus_ch[3] = (int16_t)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
//            rc_in.sbus_ch[4] = (int16_t)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
//            rc_in.sbus_ch[5] = (int16_t)(datatmp[9] & 0x03) << 9 | ((int16_t)datatmp[8] << 1) | (datatmp[7] >> 7);
//            rc_in.sbus_ch[6] = (int16_t)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
//            rc_in.sbus_ch[7] = (int16_t)datatmp[11] << 3 | (datatmp[10] >> 5);
//
//            rc_in.sbus_ch[8] = (int16_t)(datatmp[13] & 0x07) << 8 | datatmp[12];
//            rc_in.sbus_ch[9] = (int16_t)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
//            rc_in.sbus_ch[10] = (int16_t)(datatmp[16] & 0x01) << 10 | ((int16_t)datatmp[15] << 2) | (datatmp[14] >> 6);
//            rc_in.sbus_ch[11] = (int16_t)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
//            rc_in.sbus_ch[12] = (int16_t)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
//            rc_in.sbus_ch[13] = (int16_t)(datatmp[20] & 0x03) << 9 | ((int16_t)datatmp[19] << 1) | (datatmp[18] >> 7);
//            rc_in.sbus_ch[14] = (int16_t)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
//            rc_in.sbus_ch[15] = (int16_t)datatmp[22] << 3 | (datatmp[21] >> 5);
//            rc_in.sbus_flag = datatmp[23];
//
//            //user
//            //
//            if (rc_in.sbus_flag & 0x08)
//            {
//                //如果有数据且能接收到有失控标记，则不处理，转嫁成无数据失控。
//            }
//            else
//            {
//                rc_in.signal_cnt_tmp++;
//                rc_in.rc_in_mode_tmp = 2; //切换模式标记为sbus
//            }
//            //帧尾处理
//            frame_cnt++;
//            frame_cnt %= 4;
//        }
//        else
//        {
//            for (uint8_t i = 0; i < 24; i++)
//            {
//                datatmp[i] = datatmp[i + 1];
//            }
//        }
//    }
//}
static void rcSignalCheck(float *dT_s)
{
    //
    static uint8_t cnt_tmp;
    static uint16_t time_dly;
    time_dly += (*dT_s) * 1e3f;
    //==1000ms==
    if (time_dly > 1000)
    {
        time_dly = 0;
        //
        rc_in.signal_fre = rc_in.signal_cnt_tmp;

        //==判断信号是否丢失
        if (rc_in.signal_fre < 5)
        {
            rc_in.no_signal = 1;
        }
        else
        {
            rc_in.no_signal = 0;
        }
        //==判断是否切换输入方式
        if (rc_in.no_signal)
        {
            //初始0
            if (rc_in.sig_mode == 0)
            {
                cnt_tmp++;
                cnt_tmp %= 2;
                if (cnt_tmp == 1)
                {
//                    DrvRcSbusInit();
                }
                else
                {
                    DrvRcPpmInit();
                }
            }
        }
        else
        {
            rc_in.sig_mode = rc_in.rc_in_mode_tmp;
        }
        //==
        rc_in.signal_cnt_tmp = 0;
    }
}

void DrvRcInputTask(float dT_s)
{
    //信号检测
    rcSignalCheck(&dT_s);
    //有信号
    if (rc_in.no_signal == 0)
    {
        //ppm
        if (rc_in.sig_mode == 1)
        {
            for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
            {
                rc_in.rc_ch.st_data.ch_[i] = rc_in.ppm_ch[i];
            }
        }
            //sbus
        else if (rc_in.sig_mode == 2)
        {
            for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
            {
                rc_in.rc_ch.st_data.ch_[i] = 0.644f * (rc_in.sbus_ch[i] - 1024) + 1500; //248 --1024 --1800转换到1000-2000
            }
        }
        //检查失控保护设置
        if (
                (rc_in.rc_ch.st_data.ch_[ch_5_aux1] > 1200 && rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1400) || (rc_in.rc_ch.st_data.ch_[ch_5_aux1] > 1600 && rc_in.rc_ch.st_data.ch_[ch_5_aux1] < 1800))
        {
            //满足设置，标记为失控
            rc_in.fail_safe = 1;
        }
        else
        {
            rc_in.fail_safe = 0;
        }
    }
        //无信号
    else
    {
        //失控标记置位
        rc_in.fail_safe = 1;
        //
        for (uint8_t i = 0; i < 10; i++) //注意只有10个通道
        {
            rc_in.rc_ch.st_data.ch_[i] = 0; //
        }
    }
}