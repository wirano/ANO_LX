//
// Created by wirano on 2021/5/1.
//

#include "open_mv.h"
#include <math.h>

#define OMV_OFFLINE_TIMEOUT 1000

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

void omv_offline_check(uint8_t dT_ms) {
    if (omv.offline_time_cnt < OMV_OFFLINE_TIMEOUT) {
        omv.offline_time_cnt += dT_ms;
    } else{
        omv.online = 0;
    }
}

void omv_data_analysis(uint8_t *data, uint8_t len) {
    uint8_t num = 0;

    if (omv.data_received) {
        if (data[2] == 0x02) {
            omv.raw_data.type = OMV_DATA_BLOCK;
            omv.raw_data.find = data[3];

            if (omv.raw_data.find) {
                num = data[4];
            } else {
                num = 0;
            }

            for (int i = 0; i < num; ++i) {
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

        } else if (data[2] == 0x01) {
            omv.raw_data.type = OMV_DATA_LINE;
            omv.raw_data.find = data[3];


            if (omv.raw_data.find) {
                num = data[4];
            } else {
                num = 0;
            }

            for (int i = 0; i < num; ++i) {
                _tmp_line[i].start_x = ((((int16_t) data[5 + 6 * i + 0]) << 8u) | data[5 + 6 * i + 1]);
                _tmp_line[i].start_y = ((((int16_t) data[5 + 6 * i + 2]) << 8u) | data[5 + 6 * i + 3]);
                _tmp_line[i].len = data[5 + 6 * i + 4];
                _tmp_line[i].angle = data[5 + 6 * i + 5];

                if (_tmp_line[i].angle > 160 || _tmp_line[i].angle < 30) {
                    if (_tmp_line[i].angle > 90) {
                        omv.raw_data.line.angle = (_tmp_line[i].angle - 180);
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
        }

    omv.data_received = 0;
    omv.raw_data.data_flushed = 1;

    omv.offline_time_cnt = 0;
    omv.online = 1;
}
}