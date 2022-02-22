#ifndef _CRC_H
#define _CRC_H

#include "stm32f1xx_hal.h"

uint16_t GetCrc(void *Bff, uint16_t szBff);

#endif  // _CRC_H