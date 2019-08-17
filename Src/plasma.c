#include "plasma.h"
#include "servo.h"
#include "slave.h"

float dist(float a, float b, float c, float d) {
	return sqrt(((a - c) * (a - c) + (b - d) * (b - d)));
}

/*
int32_t dist(int32_t a, int32_t b, int32_t c, int32_t d) {
	a *= 1000;
	b *= 1000;
	c *= 1000;
	d *= 1000;

	return (int32_t)(sqrt(((a - c) * (a - c) + (b - d) * (b - d)))) / 1000;
}
*/

/*
void buildSinLookupTable(float sinLookupTable[628]) {
	for(float r = 0.0f; r <= 6.28f; r += 0.01) {
		sinLookupTable[(int)(r * 100.0f)] = sin(r);
	}
}

float lookupSin(float angle, float sinLookupTable[628]) {
	float lookupAngle = angle % 6.28f;

	int sinIndex = (int)(angle * 100.0f);
	return sinLookupTable[sinIndex];
}
*/

uint32_t timeCounterX = 0;
uint32_t timeCounterY = 0;
uint32_t frameCounter = 0;
uint32_t frameBuffer[24*24];
void plasma(uint32_t brightness, int16_t* accData) {
	if(timeCounterX == 0) {
		timeCounterX = HAL_GetTick();
		timeCounterY = HAL_GetTick();
	}

	uint8_t screenHeight = 24;
	uint8_t screenWidth = 24;

	//float time = HAL_GetTick() % 1000;
	//float time = 20;
	//float time = HAL_GetTick();
	//timeCounter += 3;

        int16_t accX = accData[1];
        int16_t accZ = -accData[2];

/*
        uint16_t timeX = -(accX / 400);
        uint16_t timeY = -(accZ / 400);
	timeX = timeX * timeX;
	timeY = timeY * timeY;
	timeCounter += sqrt(timeX + timeY);
*/
	int16_t timeCounterIncrementX = (accX / 1200);
	int16_t timeCounterIncrementY = (accZ / 600);
	/*
	if(timeCounterIncrement > 3) {
		timeCounterIncrement = 3;
	}
	*/
	timeCounterX += timeCounterIncrementX;
	timeCounterY += timeCounterIncrementY;
	//timeCounter += 3;
	frameCounter++;

	float timeX = timeCounterX;
	float timeY = timeCounterY;

	if(frameCounter % 3 == 0) {
		if(slaveModeEnabled()) {
			//user_pwm_setvalue(calculateServoAnglePwm(89));
		} else {
			//user_pwm_setvalue(calculateServoAnglePwm(-89));
		}

		for(int y = 0; y < screenHeight; y++) {
			for(int x = 0; x < screenWidth; x++) {
				float value = sin(dist(x + timeX, y, 128.0, 128.0) / 8.0)
					+ sin(dist(x, y, 64.0, 64.0) / 8.0)
					+ sin(dist(x, y + timeY / 7, 192.0, 64) / 7.0)
					+ sin(dist(x, y, 192.0, 100.0) / 8.0);
				int colour = (4 + value) * 32;
	
				uint8_t r = colour;
				uint8_t g = colour * 2;
				uint8_t b = 64 - colour;
	
				frameBuffer[(x*24)+y] = (r << 16) + (g << 8) + b;
	
				/*
				uint8_t debugText[100];
 				sprintf(debugText, "colour (%.5d) value (%.6d)\r\n", colour, ((int)(value * 1000)));
 				CDC_Transmit_FS(debugText, strlen(debugText));
				*/
	
			}
		}
	}

	for(int y = 0; y < screenHeight; y++) {
		for(int x = 0; x < screenWidth; x++) {
			setPixelColour(x, y, brightness, frameBuffer[(x*24)+y]);
		}
	}
}


/*
include '<math.h>';

#define screenWidth 24
#define screenHeight 24

// Y-coordinate first because we use horizontal scanlines
uint buffer[screenHeight][screenWidth];

float dist(a, b, c, d) {
return sqrt(((a - c) * (a - c) + (b - d) * (b - d)));
}

int main(int argc, char *argv[])
{
  //screen(24, 24, 0, "Plasma");

  float time;

  while(1)

  {

    time = 20.0;
    for(int y = 0; y < screenHeight; y++)
    for(int x = 0; x < screenHeight; x++)
    {
      float value = sin(dist(x + time, y, 128.0, 128.0) / 8.0)
             + sin(dist(x, y, 64.0, 64.0) / 8.0)
             + sin(dist(x, y + time / 7, 192.0, 64) / 7.0)
             + sin(dist(x, y, 192.0, 100.0) / 8.0);

      int color = (4 + value) * 32;
      //pset(x, y, ColorRGB(color, color * 2, 255 - color));
    }

    //redraw();
  }

  return(0);
}
*/
