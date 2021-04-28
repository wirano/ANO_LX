//
// Created by wirano on 2021/4/27.
//

#include "drv_led.h"

void LED_On_Off(uint16_t leds)
{

    if (leds & LED_R_BIT)
    {
        ANO_RGB_R_ON;
    }
    else
    {
        ANO_RGB_R_OFF;
    }
    if (leds & LED_G_BIT)
    {
        ANO_RGB_G_ON;
    }
    else
    {
        ANO_RGB_G_OFF;
    }
    if (leds & LED_B_BIT)
    {
        ANO_RGB_B_ON;
    }
    else
    {
        ANO_RGB_B_OFF;
    }
    if (leds & LED_A_BIT)
    {
        ANO_LED_OB_ON;
    }
    else
    {
        ANO_LED_OB_OFF;
    }
}

//LED的1�71ms驱动，在1ms定时中断里调用��1�7
_led_st led;
void LED_1ms_DRV() //
{
    static uint16_t led_cnt[LED_NUM];
    uint16_t led_tmp;
    for (uint8_t i = 0; i < LED_NUM; i++)
    {

        if (led_cnt[i] < (int16_t)led.brightness[i])
        {
            //ON
            led_tmp |= (1 << i);
        }
        else
        {
            //OFF
            led_tmp &= ~(1 << i);
        }

        if (++led_cnt[i] >= 20)
        {
            led_cnt[i] = 0;
        }
    }
    //
    LED_On_Off(led_tmp);
}
