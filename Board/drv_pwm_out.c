//
// Created by wirano on 2021/4/27.
//

#include "drv_pwm_out.h"
#include "main.h"

//21分频到 84000000/21 = 4M   0.25us
/*初始化高电平时间1000us*/
#define INIT_DUTY 4000 //u16(1000/0.25)
/*精度10000，每份0.25us*/
#define ACCURACY 10000 //总共为2500us
//电调控制协议为1000us-2000us高电平时间
/*设置飞控控制信号转换比例为4*/
#define PWM_RADIO 4    //(8000 - 4000)/1000.

extern TIM_HandleTypeDef htim1;

void DrvPwmOutInit(void){
    MX_TIM1_Init();

    HAL_TIM_Base_Start(&htim1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
}

void DrvMotorPWMSet(int16_t pwm[8])
{
    //范围0-1000，转换到4000-8000，折合1000-2000us
    TIM1->CCR4 = PWM_RADIO * (pwm[0]) + INIT_DUTY; //1
    TIM1->CCR3 = PWM_RADIO * (pwm[1]) + INIT_DUTY; //2
    TIM1->CCR2 = PWM_RADIO * (pwm[2]) + INIT_DUTY; //3
    TIM1->CCR1 = PWM_RADIO * (pwm[3]) + INIT_DUTY; //4
}