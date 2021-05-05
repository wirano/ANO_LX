//
// Created by wirano on 2021/4/28.
//

#ifndef ANO_LX_DT_H
#define ANO_LX_DT_H

#include "fc_config.h"

//==定义/声明
#define FUN_NUM_LEN 256
#define MAX_DATA_LEN 100

typedef struct
{
    uint8_t D_Addr;         //目标地址
    uint8_t WTS;             //wait to send等待发送标记
    uint16_t fre_ms;         //发送周期
    uint16_t time_cnt_ms; //计时变量
} _dt_frame_st;

//cmd
typedef struct
{
    uint8_t CID;
    uint8_t CMD[10];
} _cmd_st;

//check
typedef struct
{
    uint8_t ID;
    uint8_t SC;
    uint8_t AC;
} _ck_st;

//param
typedef struct
{
    uint16_t par_id;
    int32_t par_val;
} _par_st;

typedef struct
{
    _dt_frame_st fun[FUN_NUM_LEN];
    //
    uint8_t wait_ck;
    //
    _cmd_st cmd_send;
    _ck_st ck_send;
    _ck_st ck_back;
    _par_st par_data;
} _dt_st;

typedef struct
{
    uint8_t str[MAX_DATA_LEN];
    uint8_t str_len;
} _dt_string_st;

typedef struct {
    uint8_t data_buffer[40];
    uint8_t data_len; //byte
}_flexible_frame_st;

//==数据声明
extern _dt_st dt;

//==函数声明
//static
static void ANO_DT_LX_Send_Data(uint8_t *dataToSend, uint8_t length);

static void ANO_DT_LX_Data_Receive_Anl(uint8_t *data, uint8_t len);

//public
//
void ANO_DT_Init(void);

void ANO_LX_Data_Exchange_Task(float dT_s);

void ANO_DT_LX_Data_Receive_Prepare(uint8_t data);

//
void CMD_Send(uint8_t dest_addr, _cmd_st *cmd);

void CK_Back(uint8_t dest_addr, _ck_st *ck);

void PAR_Back(uint8_t dest_addr, _par_st *par);

void ANO_DT_String(uint8_t color, const char *str);

void Send_User_Data(uint8_t id, uint8_t len, uint8_t *data);

#endif //ANO_LX_DT_H
