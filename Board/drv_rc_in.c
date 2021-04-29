//
// Created by wirano on 2021/4/27.
//

#include "drv_rc_in.h"
#include "board.h"
#include "main.h"

extern TIM_HandleTypeDef htim3;

void DrvRcPpmInit(void)
{
    MX_TIM3_Init();

    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_1);
}

void PPM_IRQH()
{

    static uint16_t temp_cnt[2];
    //
    if (TIM3->SR & TIM_IT_CC1)
    {
        TIM3->SR = ~TIM_IT_CC1;
        TIM3->SR = ~TIM_FLAG_CC1OF;
        //==
        if (!(GPIOB->IDR & GPIO_PIN_4))
        {
            temp_cnt[0] = TIM3->CCR1;
        }
        else
        {
            temp_cnt[1] = TIM3->CCR1;
            uint16_t _tmp;
            _tmp = temp_cnt[1] - temp_cnt[0];

            DrvPpmGetOneCh(_tmp + 400);//转换到1000-2000
        }
    }
}