#ifndef _SNAKE_H_
#define _SNAKE_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct EyeGame {
	uint16_t direction;
	uint16_t dilation;
};

void eye(uint32_t brightness);

#endif
