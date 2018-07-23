#include "eye.h"

#include "buttons.h"
#include "led_panel.h"

struct EyeGame eyeGame;

void eye(uint32_t brightness) {
	float midPoint = 24.0f/2.0f;
	for(float rotation = 0; rotation < (2.0f * 3.14159f); rotation += 0.05) {
		for(float diameter = 0; diameter <= 10; diameter += 1) {
			float x = diameter * sin(rotation) + midPoint;
			float y = diameter * cos(rotation) + midPoint;

			setPixel(x, y, brightness, 0b1111, 0b1111, 0b1111);
		}
	}

	uint8_t pupilMidPointX = (rand() % 16) - 8;
	uint8_t pupilMidPointY = (rand() % 16) - 8;
	for(float rotation = 0; rotation < (2.0f * 3.14159f); rotation += 0.05) {
		for(float diameter = 0; diameter <= 4; diameter += 1) {
			float x = diameter * sin(rotation) + midPoint + pupilMidPointX;
			float y = diameter * cos(rotation) + midPoint + pupilMidPointY;

			setPixel(x, y, brightness, 0, 0, 0);
		}
	}
}

