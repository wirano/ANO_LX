//
// Created by wirano on 2021/4/28.
//

#include "ano_lx_function.h"
#include "ano_lx_state.h"
#include "ano_lx_dt.h"


uint8_t FC_Unlock()
{
    //
    fc_sta.unlock_cmd = 1; //解锁
    //按协议发送指令
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        dt.cmd_send.CID = 0x10;
        dt.cmd_send.CMD[0] = 0x00;
        dt.cmd_send.CMD[1] = 0x01;
        CMD_Send(0XFF, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}
//
uint8_t FC_Lock()
{
    //
    fc_sta.unlock_cmd = 0; //上锁
    //按协议发送指令
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        dt.cmd_send.CID = 0x10;
        dt.cmd_send.CMD[0] = 0x00;
        dt.cmd_send.CMD[1] = 0x02;
        CMD_Send(0XFF, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}

//改变飞控模式(模式0-3)
uint8_t LX_Change_Mode(uint8_t new_mode)
{
    static uint8_t old_mode;
    if (old_mode != new_mode)
    {
        //
        if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
        {
            old_mode = fc_sta.fc_mode_cmd = new_mode;
            //按协议发送指令
            dt.cmd_send.CID = 0X01;
            dt.cmd_send.CMD[0] = 0X01;
            dt.cmd_send.CMD[1] = 0X01;
            dt.cmd_send.CMD[2] = fc_sta.fc_mode_cmd;
            CMD_Send(0xff, &dt.cmd_send);
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else //已经在当前模式
    {
        return 1;
    }
}

//一键返航
//需要注意，程控模式下才能执行返航
uint8_t OneKey_Return_Home()
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X10;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X07;
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}

//一键起飞(高度cm)
uint8_t OneKey_Takeoff(uint16_t height_cm)
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X10;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X05;
        dt.cmd_send.CMD[2] = BYTE0(height_cm);
        dt.cmd_send.CMD[3] = BYTE1(height_cm);
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}
//一键降落
uint8_t OneKey_Land()
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X10;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X06;
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}
//平移(距离cm，速度cmps，方向角度0-360度)
uint8_t Horizontal_Move(uint16_t distance_cm, uint16_t velocity_cmps, uint16_t dir_angle_0_360)
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X10;
        dt.cmd_send.CMD[0] = 0X02;
        dt.cmd_send.CMD[1] = 0X03;
        //
        dt.cmd_send.CMD[2] = BYTE0(distance_cm);
        dt.cmd_send.CMD[3] = BYTE1(distance_cm);
        dt.cmd_send.CMD[4] = BYTE0(velocity_cmps);
        dt.cmd_send.CMD[5] = BYTE1(velocity_cmps);
        dt.cmd_send.CMD[6] = BYTE0(dir_angle_0_360);
        dt.cmd_send.CMD[7] = BYTE1(dir_angle_0_360);
        //
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}

//水平校准
uint8_t Horizontal_Calibrate()
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X01;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X03;
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}

//磁力计校准
uint8_t Mag_Calibrate()
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X01;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X04;
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}

//6面加速度校准
uint8_t ACC_Calibrate()
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X01;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X05;
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}

//陀螺仪校准
uint8_t GYR_Calibrate()
{
    //
    if (dt.wait_ck == 0) //没有其他等待校验的CMD时才发送本CMD
    {
        //按协议发送指令
        dt.cmd_send.CID = 0X01;
        dt.cmd_send.CMD[0] = 0X00;
        dt.cmd_send.CMD[1] = 0X02;
        CMD_Send(0xff, &dt.cmd_send);
        return 1;
    }
    else
    {
        return 0;
    }
}
