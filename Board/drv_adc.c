//
// Created by wirano on 2021/4/27.
//

#include "drv_adc.h"
#include "main.h"

extern ADC_HandleTypeDef hadc1;
extern DMA_HandleTypeDef hdma_adc1;

uint16_t AdcValue[50] = {0};

void DrvAdcInit(void)
{
    MX_DMA_Init();
    MX_ADC1_Init();

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *) &AdcValue, 50);
}

float Drv_AdcGetBatVot(void)
{
    float tmp = 0;
    for (uint8_t i = 0; i < 50; i++)
    {
        tmp += AdcValue[i] * 0.02f;
    }

    return tmp / 4096 * 3300 * (UP_R + DW_R) / DW_R * 0.001f;
}