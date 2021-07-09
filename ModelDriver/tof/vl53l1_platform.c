
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "stm32f4xx_hal_i2c.h"
#include "main.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c2;

int8_t VL53L1_WriteMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {

    while (HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, pdata, count, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, pdata, count, 0xff);
    }
    return VL53L1_ERROR_NONE;
//    HAL_I2C_Mem_Write_IT(&hi2c1,dev,index,16,pdata,count);
//    return 0; // to be implemented
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {
    while (HAL_I2C_Mem_Read(&hi2c2, dev, index, 16, pdata, count, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Read(&hi2c2, dev, index, 16, pdata, count, 0xff);
    }
    return VL53L1_ERROR_NONE;
//    if(sign)
//    {
//        return VL53L1_ERROR_CONTROL_INTERFACE ;
//    }else return VL53L1_ERROR_NONE;

}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {

    while (HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, &data, 1, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, &data, 1, 0xff);
    }
    return VL53L1_ERROR_NONE;

//    if (HAL_I2C_Mem_Write_IT(&hi2c1,dev,index,16,&data,1))
//    {
//        return VL53L1_ERROR_CONTROL_INTERFACE ;
//    }else return VL53L1_ERROR_NONE;
}

int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {
    uint8_t pData[2];

    pData[1] = data & 0xff;
    pData[0] = data >> 8;
    while (HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, pData, 2, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, pData, 2, 0xff);
    }
    return VL53L1_ERROR_NONE;

//    if (HAL_I2C_Mem_Write_IT(&hi2c1,dev,index,16,pData,2))
//    {
//        return VL53L1_ERROR_CONTROL_INTERFACE ;
//    }else return VL53L1_ERROR_NONE;
}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {
    uint8_t pData[4];

    pData[3] = data & 0xff;
    pData[2] = (data >> 8) & 0xff;
    pData[1] = (data >> 16) & 0xff;
    pData[0] = (data >> 24) & 0xff;

    while (HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, pData, 4, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Write(&hi2c2, dev, index, 16, pData, 4, 0xff);
    }
    return VL53L1_ERROR_NONE;

//    if (HAL_I2C_Mem_Write_IT(&hi2c1,dev,index,16,pData,4))
//    {
//        return VL53L1_ERROR_CONTROL_INTERFACE ;
//    }else return VL53L1_ERROR_NONE;

}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {
    while (HAL_I2C_Mem_Read(&hi2c2, dev, index, 32, data, 1, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Read(&hi2c2, dev, index, 32, data, 1, 0xff);
    }
    return VL53L1_ERROR_NONE;
//    if(point)
//    {
//        printf("point=%d\r\n",point);
//        return VL53L1_ERROR_CONTROL_INTERFACE ;
//    }else return VL53L1_ERROR_NONE;

}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
    uint8_t ret[2];
    while (HAL_I2C_Mem_Read(&hi2c2, dev, index, 16, (uint8_t *) ret, 2, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Read(&hi2c2, dev, index, 16, (uint8_t *) ret, 2, 0xff);
    }
    *data = (((uint16_t) ret[0]) << 8) | ret[1];
    return VL53L1_ERROR_NONE;
}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {
    uint8_t ret[4];

    while (HAL_I2C_Mem_Read(&hi2c2, dev, index, 16, (uint8_t *) ret, 4, 0xff)) {
        HAL_I2C_DeInit(&hi2c2);
        HAL_I2C_Init(&hi2c2);
        HAL_I2C_Mem_Read(&hi2c2, dev, index, 16, (uint8_t *) ret, 4, 0xff);
    }
    *data = (((uint32_t) ret[0]) << 24) | (((uint32_t) ret[1]) << 16) | (((uint32_t) ret[2]) << 8) | ret[3];
    return VL53L1_ERROR_NONE;
}
//    sign=HAL_I2C_Mem_Read_IT(&hi2c1,dev,index,16,(uint8_t*)ret,4);
//    *data=(((uint32_t)ret[0])<<24) | (((uint32_t)ret[1])<<16)| (((uint32_t)ret[2])<<8) | ret[3];
//
//    if(sign)
//    {
//        return VL53L1_ERROR_CONTROL_INTERFACE ;
//    }else return VL53L1_ERROR_NONE;}

int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms) {
    HAL_Delay(wait_ms);

    return 0; // to be implemented
}
void TOF_init(uint16_t dev,uint8_t mode,uint16_t time,uint16_t IM)
{
    uint8_t sensorState=1,isDataReady;
    while(sensorState==1){
        VL53L1X_BootState(dev, &sensorState);
        HAL_Delay(2);
    }
    VL53L1X_SensorInit(dev);
    VL53L1X_StartRanging(dev);
    VL53L1X_CheckForDataReady(dev, &isDataReady);
    VL53L1X_SetDistanceMode(dev, mode); /* 1=short, 2=long */
    VL53L1X_SetTimingBudgetInMs(dev, time); /* in ms possible values [20, 50, 100, 200, 500] */
    VL53L1X_SetInterMeasurementInMs(dev, IM); /* in ms, IM must be > = TB */
    VL53L1X_StartRanging(dev);   /* This function has to be called to enable the ranging */
}
