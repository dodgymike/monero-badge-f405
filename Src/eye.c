#include "eye.h"

#include "buttons.h"
#include "led_panel.h"

struct EyeGame* eyeGame = NULL;
uint8_t eyeballPixelMap[24*24];

const uint32_t oneRotation = (2*314);
float xRotationLookup[628];
float yRotationLookup[628];

void initEyeGame() {
	eyeGame = malloc(sizeof(struct EyeGame));

	eyeGame->destinationX = 20;
	eyeGame->destinationY = 20;
	eyeGame->currentX = 0;
	eyeGame->currentY = 0;
	eyeGame->rateX = 5;
	eyeGame->rateY = 5;
	eyeGame->dilation = 2;

	for(uint32_t rotation = 0; rotation < oneRotation; rotation++) {
		xRotationLookup[rotation] = sin(0.01f * rotation);
		yRotationLookup[rotation] = cos(0.01f * rotation);
	}
}

void eye(uint32_t brightness, uint32_t startButtonPressed) {
	/*
	uint8_t debugText[100];
	sprintf(debugText, "dstX (%d) dstY (%d) curX (%d) curY (%d) rateX (%d) rateY (%d)\r\n",
		eyeGame->destinationX,
		eyeGame->destinationY,
		eyeGame->currentX,
		eyeGame->currentY,
		eyeGame->rateX,
		eyeGame->rateY
	);
	CDC_Transmit_FS(debugText, strnlen(debugText, 100));
	*/

	if(eyeGame == NULL) {
		initEyeGame();
	}

	for(int i = 0; i < 24 * 24; i++) {
		eyeballPixelMap[i] = 0;
	}

	//float midPoint = 24.0f/2.0f;
	uint32_t midPoint = 24/2;

	for(uint32_t rotation = 0; rotation < oneRotation; rotation += 5) {
		for(uint32_t diameter = 0; diameter <= 10; diameter++) {
			uint32_t x = (diameter * xRotationLookup[rotation]) + midPoint;
			uint32_t y = (diameter * yRotationLookup[rotation]) + midPoint;

			setPixel(x, y, brightness, 0b1111, 0b1111, 0b1111);
			eyeballPixelMap[xyToLedIndex(x, y)] = 1;
		}
	}

	for(uint32_t rotation = 0; rotation < oneRotation; rotation += 5) {
		for(uint32_t diameter = 0; diameter <= eyeGame->dilation; diameter++) {
			uint32_t x = (diameter * xRotationLookup[rotation]) + midPoint + (eyeGame->currentX / 10) + (eyeGame->currentX / 25);
			uint32_t y = (diameter * yRotationLookup[rotation]) + midPoint + (eyeGame->currentY / 10) + (eyeGame->currentY / 25);

			if(eyeballPixelMap[xyToLedIndex(x, y)]) {
				setPixel(x, y, brightness, 0, 0, 0);
			}
		}
		for(uint32_t diameter = eyeGame->dilation; diameter <= 4; diameter++) {
			uint32_t x = (diameter * xRotationLookup[rotation]) + midPoint + (eyeGame->currentX / 10);
			uint32_t y = (diameter * yRotationLookup[rotation]) + midPoint + (eyeGame->currentY / 10);

			if(eyeballPixelMap[xyToLedIndex(x, y)]) {
				setPixel(x, y, brightness, 0, 0, 0b1111);
			}
		}
	}

	if(eyeGame->currentX != eyeGame->destinationX) {
		eyeGame->currentX += eyeGame->rateX;
	}

	if(eyeGame->currentY != eyeGame->destinationY) {
		eyeGame->currentY += eyeGame->rateY;
	}

	if((eyeGame->currentX == eyeGame->destinationX) && (eyeGame->currentY == eyeGame->destinationY)) {
		eyeGame->destinationX = 10 * ((rand() % 16) - 8);
		eyeGame->destinationY = 10 * ((rand() % 16) - 8);

		eyeGame->dilation = (rand() % 3) + 1;

		eyeGame->rateX = ((eyeGame->destinationX - eyeGame->currentX) > 0) ? 10 : -10;
		eyeGame->rateY = ((eyeGame->destinationY - eyeGame->currentY) > 0) ? 10 : -10;
	}
}

