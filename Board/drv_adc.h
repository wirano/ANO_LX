//
// Created by wirano on 2021/4/27.
//

#ifndef DRV_ADC_H
#define DRV_ADC_H

#include "stm32f4xx.h"

#define UP_R 47 //47K
#define DW_R 6.8	//6.8K


void DrvAdcInit(void);
float Drv_AdcGetBatVot(void);

#endif //DRV_ADC_H
