//
// Created by wirano on 2021/4/27.
//

#include "drv_uart.h"
#include "ano_lx_dt.h"
#include "drv_ano_of.h"
#include "main.h"


extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;


void NoUse(uint8_t data)
{
}

//串口接收发送快速定义，直接修改此处的函数名称宏，修改成自己的串口解析和发送函数名称即可，注意函数参数格式需统一
#define U1GetOneByte    NoUse
#define U2GetOneByte    NoUse
#define U3GetOneByte    NoUse
#define U4GetOneByte    NoUse
#define U5GetOneByte    ANO_DT_LX_Data_Receive_Prepare
#define U6GetOneByte    AnoOF_GetOneByte

//====uart1
void DrvUart1Init(void)
{
    MX_USART1_UART_Init();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
}

uint8_t Tx1Buffer[256];
uint8_t Tx1Counter = 0;
uint8_t count1 = 0;

void DrvUart1SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++) {
        Tx1Buffer[count1++] = *(DataToSend + i);
    }

    if (!__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE)) {
        __HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);
    }
}

uint8_t U1RxDataTmp[100];
uint8_t U1RxInCnt = 0;
uint8_t U1RxoutCnt = 0;

void drvU1GetByte(uint8_t data)
{
    U1RxDataTmp[U1RxInCnt++] = data;
    if (U1RxInCnt >= 100)
        U1RxInCnt = 0;
}

void drvU1DataCheck(void)
{
    while (U1RxInCnt != U1RxoutCnt) {
        U1GetOneByte(U1RxDataTmp[U1RxoutCnt++]);
        if (U1RxoutCnt >= 100)
            U1RxoutCnt = 0;
    }
}

void Usart1_IRQ(void)
{
    uint8_t com_data;

    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) //ORE中断
    {
        com_data = huart1.Instance->DR;
    }
    //接收中断
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE); //清除中断标志
        com_data = huart1.Instance->DR;
        drvU1GetByte(com_data);
    }
    //发送（进入移位）中断
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE)) {
        huart1.Instance->DR = Tx1Buffer[Tx1Counter++]; //写DR清除中断标志
        if (Tx1Counter == count1) {
            __HAL_UART_DISABLE_IT(&huart1,UART_IT_TXE); //关闭TXE（发送中断）中断
        }
    }
}


//====uart2
void DrvUart2Init(void)
{
    MX_USART2_UART_Init();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

uint8_t Tx2Buffer[256];
uint8_t Tx2Counter = 0;
uint8_t count2 = 0;

void DrvUart2SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++) {
        Tx2Buffer[count2++] = *(DataToSend + i);
    }

    if (!__HAL_UART_GET_IT_SOURCE(&huart2, UART_IT_TXE)) {
        __HAL_UART_ENABLE_IT(&huart2, UART_IT_TXE);
    }
}

uint8_t U2RxDataTmp[100];
uint8_t U2RxInCnt = 0;
uint8_t U2RxoutCnt = 0;

void drvU2GetByte(uint8_t data)
{
    U2RxDataTmp[U2RxInCnt++] = data;
    if (U2RxInCnt >= 100)
        U2RxInCnt = 0;
}

void drvU2DataCheck(void)
{
    while (U2RxInCnt != U2RxoutCnt) {
        U2GetOneByte(U2RxDataTmp[U2RxoutCnt++]);
        if (U2RxoutCnt >= 100)
            U2RxoutCnt = 0;
    }
}

void Usart2_IRQ(void)
{
    uint8_t com_data;

    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_ORE)) //ORE中断
    {
        com_data = huart2.Instance->DR;
    }

    //接收中断
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE); //清除中断标志
        com_data = huart2.Instance->DR;
        drvU2GetByte(com_data);
    }
    //发送（进入移位）中断
    if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TXE)) {
        huart2.Instance->DR = Tx2Buffer[Tx2Counter++]; //写DR清除中断标志
        if (Tx2Counter == count2) {
            __HAL_UART_DISABLE_IT(&huart2,UART_IT_TXE); //关闭TXE（发送中断）中断
        }
    }
}

//====uart3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////
void DrvUart3Init(void)
{
    MX_USART3_UART_Init();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

uint8_t Tx3Buffer[256];
uint8_t Tx3Counter = 0;
uint8_t count3 = 0;

void DrvUart3SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++) {
        Tx3Buffer[count3++] = *(DataToSend + i);
    }
    if (!__HAL_UART_GET_IT_SOURCE(&huart3, UART_IT_TXE)) {
        __HAL_UART_ENABLE_IT(&huart3, UART_IT_TXE);
    }
}

uint8_t U3RxDataTmp[100];
uint8_t U3RxInCnt = 0;
uint8_t U3RxoutCnt = 0;

void drvU3GetByte(uint8_t data)
{
    U3RxDataTmp[U3RxInCnt++] = data;
    if (U3RxInCnt >= 100)
        U3RxInCnt = 0;
}

void drvU3DataCheck(void)
{
    while (U3RxInCnt != U3RxoutCnt) {
        U3GetOneByte(U3RxDataTmp[U3RxoutCnt++]);
        if (U3RxoutCnt >= 100)
            U3RxoutCnt = 0;
    }
}

void Usart3_IRQ(void)
{
    uint8_t com_data;

    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_ORE)) //ORE中断
        com_data = huart3.Instance->DR;

    //接收中断
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE); //清除中断标志
        com_data = huart3.Instance->DR;
        drvU3GetByte(com_data);
    }
    //发送（进入移位）中断
    if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TXE)) {
        huart3.Instance->DR = Tx3Buffer[Tx3Counter++]; //写DR清除中断标志
        if (Tx3Counter == count3) {
            __HAL_UART_DISABLE_IT(&huart3,UART_IT_TXE); //关闭TXE（发送中断）中断
        }
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
//====uart4
void DrvUart4Init(void)
{
    MX_UART4_Init();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_RXNE);
}

uint8_t Tx4Buffer[256];
uint8_t Tx4Counter = 0;
uint8_t count4 = 0;

void DrvUart4SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++) {
        Tx4Buffer[count4++] = *(DataToSend + i);
    }

    if (!__HAL_UART_GET_IT_SOURCE(&huart4, UART_IT_TXE)) {
        __HAL_UART_ENABLE_IT(&huart4, UART_IT_TXE);
    }
}

uint8_t U4RxDataTmp[100];
uint8_t U4RxInCnt = 0;
uint8_t U4RxoutCnt = 0;

void drvU4GetByte(uint8_t data)
{
    U4RxDataTmp[U4RxInCnt++] = data;
    if (U4RxInCnt >= 100)
        U4RxInCnt = 0;
}

void drvU4DataCheck(void)
{
    while (U4RxInCnt != U4RxoutCnt) {
        U4GetOneByte(U4RxDataTmp[U4RxoutCnt++]);
        if (U4RxoutCnt >= 100)
            U4RxoutCnt = 0;
    }
}

void Uart4_IRQ(void)
{
    uint8_t com_data;

    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_ORE)) //ORE中断
    {
        com_data = huart4.Instance->DR;
    }
    //接收中断
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart4, UART_FLAG_RXNE); //清除中断标志
        com_data = huart4.Instance->DR;
        drvU4GetByte(com_data);
    }

    //发送（进入移位）中断
    if (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TXE)) {
        huart4.Instance->DR = Tx4Buffer[Tx4Counter++]; //写DR清除中断标志
        if (Tx4Counter == count4) {
            __HAL_UART_DISABLE_IT(&huart4,UART_IT_TXE); //关闭TXE（发送中断）中断
        }
    }
}

//====uart5
void DrvUart5Init(void)
{
    MX_UART5_Init();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_RXNE);
}

uint8_t Tx5Buffer[256];
uint8_t Tx5Counter = 0;
uint8_t count5 = 0;

void DrvUart5SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++) {
        Tx5Buffer[count5++] = *(DataToSend + i);
    }

    if (!__HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_TXE)) {
        __HAL_UART_ENABLE_IT(&huart5, UART_IT_TXE);
    }
}

uint8_t U5RxDataTmp[100];
uint8_t U5RxInCnt = 0;
uint8_t U5RxoutCnt = 0;

void drvU5GetByte(uint8_t data)
{
    U5RxDataTmp[U5RxInCnt++] = data;
    if (U5RxInCnt >= 100)
        U5RxInCnt = 0;
}

void drvU5DataCheck(void)
{
    while (U5RxInCnt != U5RxoutCnt) {
        U5GetOneByte(U5RxDataTmp[U5RxoutCnt++]);
        if (U5RxoutCnt >= 100)
            U5RxoutCnt = 0;
    }
}

void Uart5_IRQ(void)
{
    uint8_t com_data;

    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_ORE)) //ORE中断
    {
        com_data = huart5.Instance->DR;
    }
    //接收中断
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart5, UART_FLAG_RXNE); //清除中断标志
        com_data = huart5.Instance->DR;
        drvU5GetByte(com_data);
    }

    //发送（进入移位）中断
    if (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TXE)) {
        huart5.Instance->DR = Tx5Buffer[Tx5Counter++]; //写DR清除中断标志
        if (Tx5Counter == count5) {
            __HAL_UART_DISABLE_IT(&huart5,UART_IT_TXE); //关闭TXE（发送中断）中断
        }
    }
}

void DrvUart6Init(void)
{
    MX_USART6_UART_Init();

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_RXNE);
}

uint8_t Tx6Buffer[256];
uint8_t Tx6Counter = 0;
uint8_t count6 = 0;

void DrvUart6SendBuf(unsigned char *DataToSend, uint8_t data_num)
{
    uint8_t i;
    for (i = 0; i < data_num; i++) {
        Tx6Buffer[count6++] = *(DataToSend + i);
    }

    if (!__HAL_UART_GET_IT_SOURCE(&huart6, UART_IT_TXE)) {
        __HAL_UART_ENABLE_IT(&huart6, UART_IT_TXE);
    }
}

uint8_t U6RxDataTmp[100];
uint8_t U6RxInCnt = 0;
uint8_t U6RxoutCnt = 0;

void drvU6GetByte(uint8_t data)
{
    U6RxDataTmp[U6RxInCnt++] = data;
    if (U6RxInCnt >= 100)
        U6RxInCnt = 0;
}

void drvU6DataCheck(void)
{
    while (U6RxInCnt != U6RxoutCnt) {
        U6GetOneByte(U6RxDataTmp[U6RxoutCnt++]);
        if (U6RxoutCnt >= 100)
            U6RxoutCnt = 0;
    }
}

void Usart6_IRQ(void)
{
    uint8_t com_data;

    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_ORE)) //ORE中断
    {
        com_data = huart6.Instance->DR;
    }
    //接收中断
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(&huart6, UART_FLAG_RXNE); //清除中断标志
        com_data = huart6.Instance->DR;
        drvU6GetByte(com_data);
    }
    //发送（进入移位）中断
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_TXE)) {
        huart6.Instance->DR = Tx6Buffer[Tx6Counter++]; //写DR清除中断标志
        if (Tx6Counter == count6) {
            __HAL_UART_DISABLE_IT(&huart6,UART_IT_TXE); //关闭TXE（发送中断）中断
        }
    }
}

void DrvUartDataCheck(void)
{
    drvU1DataCheck();
    drvU2DataCheck();
    drvU3DataCheck();
    drvU4DataCheck();
    drvU5DataCheck();
    drvU6DataCheck();
}
