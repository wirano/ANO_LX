//
// Created by wirano on 2021/5/1.
//

#include "open_mv.h"
#include "ano_lx_state.h"
#include "ano_math.h"
#include "ano_filter.h"
#include <math.h>

#define OMV_OFFLINE_TIMEOUT 1000

#define PIXEL_PER_CM 1.7f  //每厘米对应像素数，与高度焦距有关，需标定

#define OMV_REC_BUFFER_LEN 200

omv_st omv[OMV_INSTANCE_NUM] = {
        {
                RES_NONE
        },
        {
                RES_NONE
        }
};

void omv_instance0_get_data(uint8_t byte_data)
{
    static uint8_t len = 0, rec_pos = 0, id;
    static uint8_t _omv_rec_buffer[OMV_REC_BUFFER_LEN];

    _omv_rec_buffer[rec_pos] = byte_data;

    if (rec_pos == 0) {
        if (byte_data == 0xaa) {
            rec_pos++;
        } else {
            rec_pos = 0;
        }
    } else if (rec_pos == 1) {
        id = byte_data;
        rec_pos++;
    } else if (rec_pos == 2) {
        len = byte_data;
        rec_pos++;
    } else if (rec_pos < len + 2) {
        rec_pos++;
    } else if (_omv_rec_buffer[rec_pos] == 0x55 && rec_pos == len + 2) {
//        omv_data_analysis(_omv_rec_buffer, len + 2);
        omv[id].id = id;
        omv[id].rec_buffer_p = _omv_rec_buffer;
        omv[id].rec_len = len + 3;
        omv[id].data_received = 1;

        rec_pos = 0;
    } else {
        rec_pos = 0;
    }
}

void omv_instance1_get_data(uint8_t byte_data)
{
    static uint8_t len = 0, rec_pos = 0, id;
    static uint8_t _omv_rec_buffer[OMV_REC_BUFFER_LEN];

    _omv_rec_buffer[rec_pos] = byte_data;

    if (rec_pos == 0) {
        if (byte_data == 0xaa) {
            rec_pos++;
        } else {
            rec_pos = 0;
        }
    } else if (rec_pos == 1) {
        id = byte_data;
        rec_pos++;
    } else if (rec_pos == 2) {
        len = byte_data;
        rec_pos++;
    } else if (rec_pos < len + 2) {
        rec_pos++;
    } else if (_omv_rec_buffer[rec_pos] == 0x55 && rec_pos == len + 2) {
//        omv_data_analysis(_omv_rec_buffer, len + 2);
        omv[id].id = id;
        omv[id].rec_buffer_p = _omv_rec_buffer;
        omv[id].rec_len = len + 3;
        omv[id].data_received = 1;

        rec_pos = 0;
    } else {
        rec_pos = 0;
    }
}

void omv_offline_check(omv_st *omv_instance, uint8_t dT_ms)
{
    if (omv_instance->offline_time_cnt < OMV_OFFLINE_TIMEOUT) {
        omv_instance->offline_time_cnt += dT_ms;
    } else {
        omv_instance->online = 0;
    }
}

void omv_data_analysis(omv_st *omv_instance, uint8_t *data, uint8_t len)
{
//    _omv_line_st _tmp_line[25];
//    _omv_block_st _tmp_block[25];

    uint8_t num_line = 0;
    uint8_t num_block = 0;

    if (omv_instance->data_received) {
        switch (data[3]) {
            case 0x01: {
                omv_instance->raw_data.type = OMV_DATA_LINE;
                omv_instance->raw_data.find = data[4];
                if (omv_instance->raw_data.find) {
                    num_line = data[5];
                } else {
                    num_line = 0;
                }

                omv_instance->raw_data.line_num = num_line;
                for (int i = 0; i < num_line; ++i) {
                    omv_instance->raw_data.line[i].start_x =
                            ((((int16_t) data[6 + 6 * i + 0]) << 8u) | data[6 + 6 * i + 1]) -
                            (omv_instance->resolution.width / 2);
                    omv_instance->raw_data.line[i].start_y =
                            ((((int16_t) data[6 + 6 * i + 2]) << 8u) | data[6 + 6 * i + 3]) -
                            (omv_instance->resolution.height / 2);
                    omv_instance->raw_data.line[i].len = data[7 + 6 * i + 4];
                    omv_instance->raw_data.line[i].angle = data[7 + 6 * i + 5];
                    omv_instance->raw_data.line[i].offset = (omv_instance->raw_data.line[i].start_x +
                                                             omv_instance->raw_data.line[i].start_y *
                                                             tan(omv_instance->raw_data.line[i].angle / 180.0 *
                                                                 3.14159));
                }
                break;
            }

            case 0x02: {
                omv_instance->raw_data.type = OMV_DATA_BLOCK;
                omv_instance->raw_data.find = data[4];

                if (omv_instance->raw_data.find) {
                    num_block = data[5];
                } else {
                    num_block = 0;
                }

                omv_instance->raw_data.block_num = num_block;
                for (int i = 0; i < num_block; ++i) {
                    omv_instance->raw_data.block[i].shape = data[6 + 10 * i + 0];
                    omv_instance->raw_data.block[i].center_x =
                            (int16_t) (data[6 + 10 * i + 1] << 8u | data[6 + 10 * i + 2]) -
                            (int16_t) (omv_instance->resolution.width / 2);
                    omv_instance->raw_data.block[i].center_y =
                            (int16_t) (data[6 + 10 * i + 3] << 8u | data[6 + 10 * i + 4]) -
                            (int16_t) (omv_instance->resolution.height / 2);
                    omv_instance->raw_data.block[i].area = (int32_t) (data[6 + 10 * i + 5] << 24u |
                                                                      data[6 + 10 * i + 6] << 16u |
                                                                      data[6 + 10 * i + 7] << 8u |
                                                                      data[6 + 10 * i + 8]);
                    omv_instance->raw_data.block[i].color = data[6 + 10 * i + 9];
                }
                break;
            }

            case 0x03: {
                omv_instance->raw_data.type = OMV_DATA_BOTH;
                omv_instance->raw_data.find = data[4];

                if (omv_instance->raw_data.find) {
                    num_block = data[5];
                    num_line = data[6];
                }

                omv_instance->raw_data.block_num = num_block;
                for (int i = 0; i < num_block; ++i) {
                    omv_instance->raw_data.block[i].shape = data[6 + 10 * i + 0];
                    omv_instance->raw_data.block[i].center_x =
                            (int16_t) (data[6 + 10 * i + 1] << 8u | data[6 + 10 * i + 2]) -
                            (int16_t) (omv_instance->resolution.width / 2);
                    omv_instance->raw_data.block[i].center_y =
                            (int16_t) (data[6 + 10 * i + 3] << 8u | data[6 + 10 * i + 4]) -
                            (int16_t) (omv_instance->resolution.height / 2);
                    omv_instance->raw_data.block[i].area = (int32_t) (data[6 + 10 * i + 5] << 24u |
                                                                      data[6 + 10 * i + 6] << 16u |
                                                                      data[6 + 10 * i + 7] << 8u |
                                                                      data[6 + 10 * i + 8]);
                    omv_instance->raw_data.block[i].color = data[6 + 10 * i + 9];
                }

                omv_instance->raw_data.line_num = num_line;
                for (int i = 0; i < num_line; ++i) {
                    omv_instance->raw_data.line[i].start_x =
                            ((((int16_t) data[6 + 6 * i + 0]) << 8u) | data[6 + 6 * i + 1]) -
                            (omv_instance->resolution.width / 2);
                    omv_instance->raw_data.line[i].start_y =
                            ((((int16_t) data[6 + 6 * i + 2]) << 8u) | data[6 + 6 * i + 3]) -
                            (omv_instance->resolution.height / 2);
                    omv_instance->raw_data.line[i].len = data[7 + 6 * i + 4];
                    omv_instance->raw_data.line[i].angle = data[7 + 6 * i + 5];
                    omv_instance->raw_data.line[i].offset = (omv_instance->raw_data.line[i].start_x +
                                                             omv_instance->raw_data.line[i].start_y *
                                                             tan(omv_instance->raw_data.line[i].angle / 180.0 *
                                                                 3.14159));
                }
            }
                break;
        }
    }

    omv_instance->data_received = 0;
    omv_instance->raw_data.data_flushed = 1;

    omv_instance->offline_time_cnt = 0;
    omv_instance->online = 1;
}

/**
 * @brief omv数据姿态解耦
 * @param dt_ms 调度时间，低通滤波需要
 * @param rol_deg rol角度
 * @param pit_deg pit角度
 */
void omv_decoupling(omv_st *omv_instance, uint8_t dt_ms, float rol_deg, float pit_deg)
{
    float alt_add_decoupled;

    alt_add_decoupled = fc_sta.fc_attitude.alt_add * cosf(rol_deg / 180.0 * 3.1415926935) *
                        cosf(pit_deg / 180.0 * 3.1415926935);

    switch (omv_instance->raw_data.type) {
        case OMV_DATA_LINE:
            for (int i = 0; i < omv_instance->raw_data.line_num; ++i) {
                omv_instance->line_track_data[i].offset_decoupled = omv_instance->raw_data.line[i].offset +
                                                                    PIXEL_PER_CM *
                                                                    tanf(rol_deg / 180.0 * 3.1415926535) *
                                                                    alt_add_decoupled;
                omv_instance->line_track_data[i].offset_decoupled = LIMIT(
                        omv_instance->line_track_data[i].offset_decoupled,
                        -50,
                        50);

                if (omv_instance->line_track_data[i].target_loss == 0) {
                    //两次低通滤波
                    omv_instance->line_track_data[i].offset_lpf_tmp[0] +=
                            0.2f * (omv_instance->line_track_data[i].offset_decoupled -
                                    omv_instance->line_track_data[i].offset_lpf_tmp[0]);
                    omv_instance->line_track_data[i].offset_lpf_tmp[1] +=
                            0.2f *
                            (omv_instance->line_track_data[i].offset_lpf_tmp[0] -
                             omv_instance->line_track_data[i].offset_lpf_tmp[1]);
                    omv_instance->line_track_data[i].offset_decoupled_lpf = omv_instance->line_track_data[i].offset_lpf_tmp[1];
                } else {
                    //丢失目标，通过低通滤波减小到0
                    LPF_1_(0.2f, dt_ms * 1e-3f, 0, omv_instance->line_track_data[i].offset_decoupled_lpf);
                }
            }
            break;

        case OMV_DATA_BLOCK:
            for (int i = 0; i < omv_instance->raw_data.block_num; ++i) {

                omv_instance->block_track_data[i].offset_x_decoupled = omv_instance->raw_data.block[i].center_x +
                                                                       PIXEL_PER_CM *
                                                                       tanf(pit_deg / 180.0 * 3.1415926535) *
                                                                       alt_add_decoupled;
                omv_instance->block_track_data[i].offset_y_decoupled = omv_instance->raw_data.block[i].center_y +
                                                                    PIXEL_PER_CM *
                                                                    tanf(rol_deg / 180.0 * 3.1415926535) *
                                                                    alt_add_decoupled;
                LIMIT(omv_instance->block_track_data[i].offset_x_decoupled, -80, 80);
                LIMIT(omv_instance->block_track_data[i].offset_y_decoupled, -90, 90);

                if (omv_instance->block_track_data[i].target_loss == 0) {
                    for (int i = 0; i < 2; ++i) {
                        omv_instance->block_track_data[i].offset_lpf_tmp[0][0] +=
                                0.2f * (omv_instance->block_track_data[i].offset_x_decoupled -
                                        omv_instance->block_track_data[i].offset_lpf_tmp[0][0]);
                        omv_instance->block_track_data[i].offset_lpf_tmp[0][1] +=
                                0.2f * (omv_instance->block_track_data[i].offset_y_decoupled -
                                        omv_instance->block_track_data[i].offset_lpf_tmp[0][1]);

                        omv_instance->block_track_data[i].offset_lpf_tmp[1][0] +=
                                0.2f * (omv_instance->block_track_data[i].offset_lpf_tmp[0][0] -
                                        omv_instance->block_track_data[i].offset_lpf_tmp[1][0]);
                        omv_instance->block_track_data[i].offset_lpf_tmp[1][1] +=
                                0.2f * (omv_instance->block_track_data[i].offset_lpf_tmp[0][1] -
                                        omv_instance->block_track_data[i].offset_lpf_tmp[1][1]);

                        omv_instance->block_track_data[i].offset_x_decoupled_lpf = omv_instance->block_track_data[i].offset_lpf_tmp[1][0];
                        omv_instance->block_track_data[i].offset_y_decoupled_lpf = omv_instance->block_track_data[i].offset_lpf_tmp[1][1];
                    }
                } else {
                    LPF_1_(0.2f, dt_ms * 1e-3f, 0, omv_instance->block_track_data[i].offset_x_decoupled_lpf);
                    LPF_1_(0.2f, dt_ms * 1e-3f, 0, omv_instance->block_track_data[i].offset_y_decoupled_lpf);
                }
            }
            break;
    }
}
