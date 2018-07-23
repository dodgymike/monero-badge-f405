#include "eye.h"

#include "buttons.h"
#include "led_panel.h"

struct EyeGame* eyeGame = NULL;
uint8_t eyeballPixelMap[24*24];

void initEyeGame() {
	eyeGame = malloc(sizeof(struct EyeGame));

	eyeGame->destinationX = 20;
	eyeGame->destinationY = 20;
	eyeGame->currentX = 0;
	eyeGame->currentY = 0;
	eyeGame->rateX = 5;
	eyeGame->rateY = 5;
	eyeGame->dilation = 2;
}

void eye(uint32_t brightness) {
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

	if(eyeGame == NULL) {
		initEyeGame();
	}

	for(int i = 0; i < 24 * 24; i++) {
		eyeballPixelMap[i] = 0;
	}

	float midPoint = 24.0f/2.0f;

	for(float rotation = 0; rotation < (2.0f * 3.14159f); rotation += 0.05) {
		for(float diameter = 0; diameter <= 10; diameter += 1) {
			float x = diameter * sin(rotation) + midPoint;
			float y = diameter * cos(rotation) + midPoint;

			setPixel(x, y, brightness, 0b1111, 0b1111, 0b1111);
			eyeballPixelMap[xyToLedIndex(x, y)] = 1;
		}
	}

	for(float rotation = 0; rotation < (2.0f * 3.14159f); rotation += 0.05) {
		for(float diameter = 0; diameter <= eyeGame->dilation; diameter += 1) {
			float x = diameter * sin(rotation) + midPoint + (eyeGame->currentX / 10);
			float y = diameter * cos(rotation) + midPoint + (eyeGame->currentY / 10);

			if(eyeballPixelMap[xyToLedIndex(x, y)]) {
				setPixel(x, y, brightness, 0, 0, 0);
			}
		}
		for(float diameter = eyeGame->dilation; diameter <= 4; diameter += 1) {
			float x = diameter * sin(rotation) + midPoint + (eyeGame->currentX / 10);
			float y = diameter * cos(rotation) + midPoint + (eyeGame->currentY / 10);

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

