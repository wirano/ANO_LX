//
// Created by wirano on 2021/4/27.
//

#ifndef DRV_LED_H
#define DRV_LED_H

#include "stm32f4xx.h"
#include "main.h"

#define LED_R_BIT 0x01
#define LED_G_BIT 0x02
#define LED_B_BIT 0x04

#define LED_A_BIT 0X08
#define LED_ALL_BIT 0x0f

#define ANO_LED_OB_OFF ANO_LED_OB_GPIO_Port->BSRR = ANO_LED_OB_Pin
#define ANO_LED_OB_ON ANO_LED_OB_GPIO_Port->BSRR = ANO_LED_OB_Pin << 16U

#define ANO_RGB_R_OFF ANO_RGB_R_GPIO_Port->BSRR = ANO_RGB_R_Pin << 16U
#define ANO_RGB_R_ON ANO_RGB_R_GPIO_Port->BSRR = ANO_RGB_R_Pin
#define ANO_RGB_G_OFF ANO_RGB_G_GPIO_Port->BSRR = ANO_RGB_G_Pin << 16U
#define ANO_RGB_G_ON ANO_RGB_G_GPIO_Port->BSRR = ANO_RGB_G_Pin
#define ANO_RGB_B_OFF ANO_RGB_B_GPIO_Port->BSRR = ANO_RGB_B_Pin << 16U
#define ANO_RGB_B_ON ANO_RGB_B_GPIO_Port->BSRR = ANO_RGB_B_Pin

#define ANO_LED_NUM 4

typedef union {
    //
    int8_t brightness[ANO_LED_NUM];

} _led_st;

extern _led_st led;

void LED_1ms_DRV(void);

#endif //DRV_LED_H
