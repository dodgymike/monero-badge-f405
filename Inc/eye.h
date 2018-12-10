#ifndef _SNAKE_H_
#define _SNAKE_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct EyeGame {
	int32_t destinationX;
	int32_t destinationY;

	int32_t currentX;
	int32_t currentY;

	int32_t rateX;
	int32_t rateY;

	uint32_t dilation;
};

void eye(uint32_t brightness, uint32_t startButtonPressed, int16_t* accData);

#endif
