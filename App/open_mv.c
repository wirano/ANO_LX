//
// Created by wirano on 2021/5/1.
//

#include "open_mv.h"
#include "ano_lx_state.h"
#include "ano_math.h"
#include <math.h>

#define OMV_OFFLINE_TIMEOUT 1000

#define PIXEL_PER_CM 0.14f  //每厘米对应像素数，与高度焦距有关，需标定

omv_st omv;

uint8_t _omv_rec_buffer[256];

_omv_line_st _tmp_line[25];
_omv_block_st _tmp_block[25];

void omv_get_data(uint8_t byte_data) {
    static uint8_t len = 0, rec_pos = 0;

    _omv_rec_buffer[rec_pos] = byte_data;

    if (rec_pos == 0) {
        if (byte_data == 0xaa) {
            rec_pos++;
        } else {
            rec_pos = 0;
        }
    } else if (rec_pos == 1) {
        len = byte_data;
        rec_pos++;
    } else if (rec_pos < len + 2) {
        rec_pos++;
    } else if (_omv_rec_buffer[rec_pos - 1] == 0x55 && rec_pos == len + 2) {
//        omv_data_analysis(_omv_rec_buffer, len + 2);
        omv.rec_buffer_p = _omv_rec_buffer;
        omv.rec_len = len + 2;
        omv.data_received = 1;

        rec_pos = 0;
    } else {
        rec_pos = 0;
    }
}

void omv_offline_check(uint8_t dT_ms)
{
    if (omv.offline_time_cnt < OMV_OFFLINE_TIMEOUT) {
        omv.offline_time_cnt += dT_ms;
    } else {
        omv.online = 0;
    }
}

void omv_data_analysis(uint8_t *data, uint8_t len)
{
    uint8_t num_line = 0;
    uint8_t num_block = 0;

    if (omv.data_received) {
        switch (data[2]) {
            case 0x01: {
                omv.raw_data.type = OMV_DATA_LINE;
                omv.raw_data.find = data[3];


                if (omv.raw_data.find) {
                    num_line = data[4];
                } else {
                    num_line = 0;
                }

                for (int i = 0; i < num_line; ++i) {
                    _tmp_line[i].start_x = ((((int16_t) data[5 + 6 * i + 0]) << 8u) | data[5 + 6 * i + 1]);
                    _tmp_line[i].start_y = ((((int16_t) data[5 + 6 * i + 2]) << 8u) | data[5 + 6 * i + 3]);
                    _tmp_line[i].len = data[5 + 6 * i + 4];
                    _tmp_line[i].angle = data[5 + 6 * i + 5];

                    if (_tmp_line[i].angle > 170 || _tmp_line[i].angle < 20) {
                        if (_tmp_line[i].angle > 90) {
                            omv.raw_data.line.angle = (180 - _tmp_line[i].angle);
                        } else {
                            omv.raw_data.line.angle = -_tmp_line[i].angle;
                        }
//                if(opmv.lt.angle > -5 && opmv.lt.angle < 5){
//                    opmv.lt.angle = 0;
//                }
                        omv.raw_data.line.offset = (_tmp_line[i].start_x +
                                                    _tmp_line[i].start_y * tan(_tmp_line[i].angle / 180.0 * 3.14159));
                    }
                }
                break;
            }
            case 0x02: {
                omv.raw_data.type = OMV_DATA_BLOCK;
                omv.raw_data.find = data[3];

                if (omv.raw_data.find) {
                    num_block = data[4];
                } else {
                    num_block = 0;
                }

                for (int i = 0; i < num_block; ++i) {
                    _tmp_block[i].shape = data[5 + 6 * i + 0];
                    _tmp_block[i].center_x = ((((int16_t) data[5 + 6 * i + 1]) << 8u) | data[5 + 6 * i + 2]);
                    _tmp_block[i].center_y = ((((int16_t) data[5 + 6 * i + 3]) << 8u) | data[5 + 6 * i + 4]);
                    _tmp_block[i].color = data[5 + 6 * i + 5];

                    if (_tmp_block[i].shape == OMV_SHAPE_CIRCLE) {
                        omv.raw_data.block.shape = _tmp_block[i].shape;
                        omv.raw_data.block.center_x = _tmp_block[i].center_x;
                        omv.raw_data.block.center_y = _tmp_block[i].center_y;
                    }
                }
                break;
            }
            case 0x03: {
                omv.raw_data.type = OMV_DATA_BOTH;
                omv.raw_data.find = data[3];

                if (omv.raw_data.find) {
                    num_block = data[4];
                    num_line = data[5];
                }

                for (int i = 0; i < num_block; ++i) {
                    _tmp_block[i].shape = data[6 + 6 * i + 0];
                    _tmp_block[i].center_x = ((((int16_t) data[6 + 6 * i + 1]) << 8u) | data[6 + 6 * i + 2]);
                    _tmp_block[i].center_y = ((((int16_t) data[6 + 6 * i + 3]) << 8u) | data[6 + 6 * i + 4]);
                    _tmp_block[i].color = data[6 + 6 * i + 5];

                    if (_tmp_block[i].shape == OMV_SHAPE_CIRCLE) {
                        omv.raw_data.block.shape = _tmp_block[i].shape;
                        omv.raw_data.block.center_x = _tmp_block[i].center_x;
                        omv.raw_data.block.center_y = _tmp_block[i].center_y;
                    }
                }

                for (int i = 0; i < num_line; ++i) {
                    _tmp_line[i].start_x = ((((int16_t) data[6 * num_block + 5 + 6 * i + 0]) << 8u) |
                                            data[6 * num_block + 5 + 6 * i + 1]);
                    _tmp_line[i].start_y = ((((int16_t) data[6 * num_block + 5 + 6 * i + 2]) << 8u) |
                                            data[6 * num_block + 5 + 6 * i + 3]);
                    _tmp_line[i].len = data[6 * num_block + 5 + 6 * i + 4];
                    _tmp_line[i].angle = data[6 * num_block + 5 + 6 * i + 5];

                    if (_tmp_line[i].angle > 160 || _tmp_line[i].angle < 30) {
                        if (_tmp_line[i].angle > 90) {
                            omv.raw_data.line.angle = _tmp_line[i].angle-180;
                        } else {
                            omv.raw_data.line.angle = _tmp_line[i].angle;
                        }
//                if(opmv.lt.angle > -5 && opmv.lt.angle < 5){
//                    opmv.lt.angle = 0;
//                }
                        omv.raw_data.line.offset = (_tmp_line[i].start_x +
                                                    _tmp_line[i].start_y * tan(_tmp_line[i].angle / 180.0 * 3.14159));
                    }
                }

                break;
            }
        }

        omv.data_received = 0;
        omv.raw_data.data_flushed = 1;

        omv.offline_time_cnt = 0;
        omv.online = 1;
    }
}

void omv_decoupling(uint8_t dt_ms, float rol_deg, float pit_deg)
{
    float alt_add_decoupled;

    alt_add_decoupled = fc_sta.fc_attitude.alt_add * cosf(rol_deg / 180.0 * 3.1415926935) *
                        cosf(pit_deg / 180.0 * 3.1415926935);

    switch (omv.raw_data.type) {
        case OMV_DATA_LINE:
            omv.line_track_data.offset_decoupled = omv.raw_data.line.offset +
                                                   PIXEL_PER_CM * tanf(rol_deg / 180.0 * 3.1415926535) *
                                                   alt_add_decoupled;
            omv.line_track_data.offset_decoupled = LIMIT(omv.line_track_data.offset_decoupled,-50,50);
            break;
        case OMV_DATA_BLOCK:
            omv.block_track_data.offset_x_decoupled = omv.raw_data.block.center_x +
            PIXEL_PER_CM * tanf(pit_deg / 180.0 * 3.1415926535) *
            alt_add_decoupled;
            omv.block_track_data.offset_y_decoupled = omv.raw_data.block.center_y +
            PIXEL_PER_CM * tanf(rol_deg / 180.0 * 3.1415926535) *
            alt_add_decoupled;
            LIMIT(omv.block_track_data.offset_x_decoupled,-80,80);
            LIMIT(omv.block_track_data.offset_y_decoupled,-90,90);
            break;
    }
}
