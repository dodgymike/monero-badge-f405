#ifndef _WELCOME_H_
#define _WELCOME_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "welcome_smileys.h"

void addKonamiCodeButtonEntry(uint32_t button);
uint8_t compareKonamiCodes();

void welcome(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick);

#endif
