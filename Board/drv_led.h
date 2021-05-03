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


#define USER_LED_R_OFF USER_LED_R_GPIO_Port->BSRR = USER_LED_R_Pin << 16U
#define USER_LED_R_ON USER_LED_R_GPIO_Port->BSRR = USER_LED_R_Pin
#define USER_LED_G_OFF USER_LED_G_GPIO_Port->BSRR = USER_LED_G_Pin << 16U
#define USER_LED_G_ON USER_LED_G_GPIO_Port->BSRR = USER_LED_G_Pin
#define USER_LED_B_OFF USER_LED_B_GPIO_Port->BSRR = USER_LED_B_Pin << 16U
#define USER_LED_B_ON USER_LED_B_GPIO_Port->BSRR = USER_LED_B_Pin

#define USER_LED_NUM 3

#define USER_LED_BRIGHTNESS_MASK        0x001FU
#define USER_LED_SET_R_BRIGHTNESS(BR)   (BR << 10)
#define USER_LED_SET_G_BRIGHTNESS(BR)   (BR << 5)
#define USER_LED_SET_B_BRIGHTNESS(BR)   (BR << 0)

#define USER_LED_COLOR_WHILE    0x7fff
#define USER_LED_COLOR_RED      0x7c00
#define USER_LED_COLOR_GREEN    0x03e0
#define USER_LED_COLOR_BLUE     0x001f
#define USER_LED_COLOR_YELLOW   USER_LED_COLOR_RED | USER_LED_COLOR_GREEN

typedef union
{
    uint8_t brightness[ANO_LED_NUM];
} _led_st;

/**
 * @brief open mv 任务状态指示灯标志位
 * @brief led轮转显示对外接口
 */
typedef struct
{
    uint8_t led_num;            //任务轮转led颗数 非零表示开启此状态展示
//    uint8_t led1_state_omv: 1;  //0:off 1:on 总开关，开启有效
    uint8_t led2_omv_online: 1; //0:offline 1:online
    uint8_t led3_omv_target: 2; //0:none 1:block 2:line
    uint8_t led4_omv_move: 2;   //0:straight 1:left 2:right;
    uint8_t : 3;
} _omv_state_led_st;

/**
 * @brief 任务枚举，枚举可显示任务
 */
typedef enum {
    USER_LED_OMV_TASK,
    USER_LED_TASK_NUM,
}_user_led_task_em;

/**
 * @brief led轮转表，最大可显示颗数由此决定
 */
typedef struct {
    uint16_t led1_task; //第一颗led表示此轮展示的任务
    uint16_t led2; //由任务定义
    uint16_t led3;
    uint16_t led4;
}_user_led_state_list_st;

/**
 * @brief 轮转显示结构体
 */
typedef struct {
    _user_led_task_em task_type;
    uint8_t led_num;
    _user_led_state_list_st state_list[USER_LED_TASK_NUM];
}user_led_task_st;

extern _led_st ano_leds;

extern _omv_state_led_st omv_led_state;

void LED_1ms_DRV(void);

void user_rgb_drv(void);

void user_rgb_update(uint16_t dt_ms);

void user_rgb_tasks(uint16_t dt_ms);

#endif //DRV_LED_H
