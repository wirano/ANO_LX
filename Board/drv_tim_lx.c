//
// Created by wirano on 2021/4/27.
//

#include "drv_tim_lx.h"
#include "stm32f4xx.h"

extern TIM_HandleTypeDef htim7;

void DrvTimerFcInit(void)
{
    HAL_TIM_Base_Start_IT(&htim7);
}

void LX_TIM3_IRQH(void)
{
    if (__HAL_TIM_GET_ITSTATUS(&htim7,TIM_IT_UPDATE) != RESET) {
        __HAL_TIM_CLEAR_IT(&htim7,TIM_IT_UPDATE);

        ANO_LX_Task();
    }
}
