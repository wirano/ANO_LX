//
// Created by wirano on 2021/5/1.
//

#ifndef OPEN_MV_H
#define OPEN_MV_H

#include "main.h"

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
    uint16_t area;
    int16_t center_x;
    int16_t center_y;
} _omv_block_st;

typedef struct {
    uint8_t target_loss;

    float offset_decoupled;
}_omv_line_track_data_st;

typedef struct {
    uint8_t target_loss;

    float offset_x_decoupled;
    float offset_y_decoupled;
}_omv_block_track_data_st;

typedef struct
{
    uint8_t data_flushed; //数据刷新标志
    _omv_data_type_em type;
    _omv_block_st block;
    _omv_line_st line;
    uint8_t find;//找到目标
} _omv_data_st;

typedef struct
{
    uint8_t online;
    uint16_t offline_time_cnt;

    uint8_t data_received;
    uint8_t *rec_buffer_p;
    uint8_t rec_len;

    _omv_data_st raw_data;

    _omv_block_track_data_st block_track_data;
    _omv_line_track_data_st line_track_data;
} omv_st;

extern omv_st omv;

void omv_data_analysis(uint8_t *data, uint8_t len);

void omv_decoupling(uint8_t dt_ms, float rol_deg, float pit_deg);

void omv_offline_check(uint8_t dT_ms);

void omv_get_data(uint8_t byte_data);

#endif //OPEN_MV_H
