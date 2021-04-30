//
// Created by wirano on 2021/4/27.
//

#ifndef DRV_UART_H
#define DRV_UART_H

#include "stm32f4xx.h"

void DrvUart1Init(void);

void DrvUart1SendBuf(uint8_t *data, uint8_t len);

void Usart1_IRQ(void);

void DrvUart2Init(void);

void DrvUart2SendBuf(uint8_t *data, uint8_t len);

void Usart2_IRQ(void);

void DrvUart3Init(void);

void DrvUart3SendBuf(uint8_t *data, uint8_t len);

void Usart3_IRQ(void);

void DrvUart4Init(void);

void DrvUart4SendBuf(uint8_t *data, uint8_t len);

void Uart4_IRQ(void);

void DrvUart5Init(void);

void DrvUart5SendBuf(uint8_t *data, uint8_t len);

void Uart5_IRQ(void);

void DrvUart6Init(void);

void DrvUart6SendBuf(uint8_t *data, uint8_t len);

void Usart6_IRQ(void);

void DrvUartDataCheck(void);

#endif //DRV_UART_H
