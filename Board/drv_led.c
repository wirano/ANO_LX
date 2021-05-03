//
// Created by wirano on 2021/4/27.
//

#include "drv_led.h"

static inline void LED_On_Off(uint8_t leds)
{
    if (leds & LED_R_BIT) {
        ANO_RGB_R_ON;
    } else {
        ANO_RGB_R_OFF;
    }
    if (leds & LED_G_BIT) {
        ANO_RGB_G_ON;
    } else {
        ANO_RGB_G_OFF;
    }
    if (leds & LED_B_BIT) {
        ANO_RGB_B_ON;
    } else {
        ANO_RGB_B_OFF;
    }
    if (leds & LED_A_BIT) {
        ANO_LED_OB_ON;
    } else {
        ANO_LED_OB_OFF;
    }
}

_led_st ano_leds;

void LED_1ms_DRV(void) //
{
    static uint8_t led_cnt[ANO_LED_NUM];
    uint8_t led_tmp = 0;

    for (uint8_t i = 0; i < ANO_LED_NUM; i++) {
        if (led_cnt[i] < (int16_t) ano_leds.brightness[i]) {
            //ON
            led_tmp |= (1 << i);
        } else {
            //OFF
            led_tmp &= ~(1 << i);
        }

        if (++led_cnt[i] >= 20) {
            led_cnt[i] = 0;
        }
    }
    //
    LED_On_Off(led_tmp);
}

static inline void user_rgb_switch(uint8_t leds)
{
    if (leds & LED_R_BIT) {
        USER_LED_R_ON;
    } else {
        USER_LED_R_OFF;
    }
    if (leds & LED_G_BIT) {
        USER_LED_G_ON;
    } else {
        USER_LED_G_OFF;
    }
    if (leds & LED_B_BIT) {
        USER_LED_B_ON;
    } else {
        USER_LED_B_OFF;
    }
}

_led_st user_rgb;


/**
 * @brief led软件PWM调光（20级）
 * @brief 1ms调用一次，50Hz
 */
void user_rgb_drv(void)
{
    static uint8_t led_cnt[USER_LED_NUM];
    uint8_t rgb_tmp = 0;

    for (uint8_t i = 0; i < USER_LED_NUM; i++) {
        if (led_cnt[i] < user_rgb.brightness[i]) {
            //ON
            rgb_tmp |= (1 << i);
        } else {
            //OFF
            rgb_tmp &= ~(1 << i);
        }

        if (++led_cnt[i] > 20) {
            led_cnt[i] = 0;
        }
    }

    user_rgb_switch(rgb_tmp);
}

user_led_task_st user_led;

/**
 * @brief led轮转闪烁
 * @brief led亮灭时间在此调整
 * @param dt_ms 调度间隔
 */
void user_rgb_tasks(uint16_t dt_ms)
{
    static uint8_t led_cnt;
    static uint16_t dt_cnt;

    switch (led_cnt) {
        case 0:
            if (dt_cnt < 100) {
                //on_ms
                user_rgb.brightness[0] =
                        user_led.state_list[user_led.task_type].led1_task >> 10 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[1] =
                        user_led.state_list[user_led.task_type].led1_task >> 5 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[2] =
                        user_led.state_list[user_led.task_type].led1_task >> 0 & USER_LED_BRIGHTNESS_MASK;
            } else {
                //off
                for (int i = 0; i < USER_LED_NUM; ++i) {
                    user_rgb.brightness[i] = 0;
                }
            }

            if (dt_cnt > 300) {
                dt_cnt = 0;
                led_cnt++;
            }
            break;
        case 1:
            if (dt_cnt < 100) {
                //on_ms
                user_rgb.brightness[0] =
                        user_led.state_list[user_led.task_type].led2 >> 10 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[1] =
                        user_led.state_list[user_led.task_type].led2 >> 5 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[2] =
                        user_led.state_list[user_led.task_type].led2 >> 0 & USER_LED_BRIGHTNESS_MASK;
            } else {
                //off
                for (int i = 0; i < USER_LED_NUM; ++i) {
                    user_rgb.brightness[i] = 0;
                }
            }

            if (dt_cnt > 300) {
                dt_cnt = 0;
                led_cnt++;
            }
            break;
        case 2:
            if (dt_cnt < 100) {
                //on_ms
                user_rgb.brightness[0] =
                        user_led.state_list[user_led.task_type].led3 >> 10 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[1] =
                        user_led.state_list[user_led.task_type].led3 >> 5 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[2] =
                        user_led.state_list[user_led.task_type].led3 >> 0 & USER_LED_BRIGHTNESS_MASK;
            } else {
                //off
                for (int i = 0; i < USER_LED_NUM; ++i) {
                    user_rgb.brightness[i] = 0;
                }
            }

            if (dt_cnt > 300) {
                dt_cnt = 0;
                led_cnt++;
            }
            break;
        case 3:
            if (dt_cnt < 100) {
                //on_ms
                user_rgb.brightness[0] =
                        user_led.state_list[user_led.task_type].led4 >> 10 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[1] =
                        user_led.state_list[user_led.task_type].led4 >> 5 & USER_LED_BRIGHTNESS_MASK;
                user_rgb.brightness[2] =
                        user_led.state_list[user_led.task_type].led4 >> 0 & USER_LED_BRIGHTNESS_MASK;
            } else {
                //off
                for (int i = 0; i < USER_LED_NUM; ++i) {
                    user_rgb.brightness[i] = 0;
                }
            }

            if (dt_cnt > 300) {
                dt_cnt = 0;
                led_cnt++;
            }
            break;
        case 255:
            //delay
            if (dt_cnt > 1000) {
                dt_cnt = 0;
                led_cnt = 0;
            }
            break;
    }

    if(led_cnt >= user_led.led_num - 1){
        led_cnt = 255;
    }

    dt_cnt += dt_ms;
}

_omv_state_led_st omv_led_state;

/**
 * @brief led状态更新
 * @brief 新功能的led状态表在此添加
 * @brief if顺序表示优先级，越靠后优先级越高
 * @param dt_ms 延时时间
 */
void user_rgb_update(uint16_t dt_ms)
{
    if (omv_led_state.led_num) {
        user_led.led_num = omv_led_state.led_num;

        user_led.task_type = USER_LED_OMV_TASK;

        user_led.state_list[USER_LED_OMV_TASK].led1_task = USER_LED_COLOR_YELLOW;

        if (omv_led_state.led2_omv_online) {
            user_led.state_list[USER_LED_OMV_TASK].led2 = USER_LED_COLOR_GREEN;
        } else {
            user_led.state_list[USER_LED_OMV_TASK].led2 = USER_LED_COLOR_RED;
        }

        if (omv_led_state.led3_omv_target == 0) {
            user_led.state_list[USER_LED_OMV_TASK].led3 = USER_LED_COLOR_WHILE;
        } else if (omv_led_state.led3_omv_target == 1) {
            user_led.state_list[USER_LED_OMV_TASK].led3 = USER_LED_COLOR_BLUE;
        } else if (omv_led_state.led3_omv_target == 2) {
            user_led.state_list[USER_LED_OMV_TASK].led3 = USER_LED_COLOR_GREEN;
        }

        if (omv_led_state.led4_omv_move == 0) {
            user_led.state_list[USER_LED_OMV_TASK].led4 = USER_LED_COLOR_WHILE;
        } else if (omv_led_state.led4_omv_move == 1) {
            user_led.state_list[USER_LED_OMV_TASK].led4 = USER_LED_COLOR_GREEN;
        } else if (omv_led_state.led4_omv_move == 2) {
            user_led.state_list[USER_LED_OMV_TASK].led4 = USER_LED_COLOR_BLUE;
        }
    }
}
