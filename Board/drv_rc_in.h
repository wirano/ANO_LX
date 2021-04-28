//
// Created by wirano on 2021/4/27.
//

#ifndef DRV_RC_IN_H
#define DRV_RC_IN_H

#include "stm32f4xx.h"

void DrvRcPpmInit(void);
void PPM_IRQH(void);

#endif //DRV_RC_IN_H
