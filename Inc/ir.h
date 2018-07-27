#ifndef _IR_H_
#define _IR_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"


void irTX(uint8_t data[], uint8_t dataSize);
uint8_t irRX(uint8_t data[], uint8_t dataSize);

#endif
