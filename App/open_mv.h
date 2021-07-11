//
// Created by wirano on 2021/5/1.
//

#ifndef OPEN_MV_H
#define OPEN_MV_H

#include "main.h"

#define RES_WVGA    .resolution.height = 480,.resolution.width = 720
#define RES_VGA     .resolution.height = 480,.resolution.width = 640
#define RES_QVGA    .resolution.height = 240,.resolution.width = 320
#define RES_QQVGA   .resolution.height = 120,.resolution.width = 160
#define RES_QQQQVGA .resolution.height = 60,.resolution.width = 80
#define RES_NONE    .resolution.height = 0,.resolution.width = 0

#define OMV_LINE_MAX 3
#define OMV_BLOCK_MAX 4

typedef enum {
    OMV_BAR_ID,
    OMV_LAND_ID,
    OMV_INSTANCE_NUM
} _omv_instance_em;

typedef enum
{
    OMV_COLOR_WHITE,
    OMV_COLOR_BLACK,
    OMV_COLOR_RED,
    OMV_COLOR_GREEN,
    OMV_COLOR_BLUE,
    OMV_COLOR_NUM
} _omv_color_em;

typedef enum
{
    OMV_SHAPE_CIRCLE,
    OMV_SHAPE_HALF_CIRCLE,
    OMV_SHAPE_RECTANGLE,
    OMV_SHAPE_TRIANGLE,
    OMV_SHAPE_NUM
} _omv_shape_em;

typedef enum
{
    OMV_DATA_LINE,
    OMV_DATA_BLOCK,
    OMV_DATA_BOTH,
    OMV_DATA_NUM,
} _omv_data_type_em;

typedef struct
{
    uint16_t len;
    int16_t angle;
    int16_t start_x;
    int16_t start_y;
    int16_t offset;
} _omv_line_st;

typedef struct
{
    _omv_shape_em shape;
    _omv_color_em color;
    uint32_t area;
    int16_t center_x;
    int16_t center_y;
} _omv_block_st;

typedef struct {
    uint8_t target_loss;

    float offset_lpf_tmp[2];

    float offset_decoupled;
    float offset_decoupled_lpf;
}_omv_line_track_data_st;

typedef struct {
    uint8_t target_loss;

    float offset_lpf_tmp[2][2]; //[cnt][x/y]

    float offset_x_decoupled;
    float offset_y_decoupled;

    float offset_x_decoupled_lpf;
    float offset_y_decoupled_lpf;
}_omv_block_track_data_st;

typedef struct
{
    uint8_t data_flushed; //数据刷新标志
    _omv_data_type_em type;
    uint8_t block_num;
    uint8_t line_num;
    _omv_block_st block[OMV_BLOCK_MAX];
    _omv_line_st line[OMV_LINE_MAX];
    uint8_t find;//找到目标
} _omv_data_st;

typedef struct {
    uint16_t width;
    uint16_t height;
} _omv_resolution_st;

typedef struct
{
    uint8_t id;
    _omv_resolution_st resolution;

    uint8_t online;
    uint16_t offline_time_cnt;

    uint8_t data_received;
    uint8_t *rec_buffer_p;
    uint8_t rec_len;

    _omv_data_st raw_data;

    _omv_block_track_data_st block_track_data[OMV_BLOCK_MAX];
    _omv_line_track_data_st line_track_data[OMV_LINE_MAX];
} omv_st;

extern omv_st omv[OMV_INSTANCE_NUM];

void omv_data_analysis(omv_st *omv_instance, uint8_t *data, uint8_t len);

void omv_decoupling(omv_st *omv_instance, uint8_t dt_ms, float rol_deg, float pit_deg);

void omv_offline_check(omv_st *omv_instance, uint8_t dT_ms);

void omv_instance0_get_data(uint8_t byte_data);

void omv_instance1_get_data(uint8_t byte_data);

#endif //OPEN_MV_H
