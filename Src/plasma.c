#include "plasma.h"

float dist(float a, float b, float c, float d) {
	return sqrt(((a - c) * (a - c) + (b - d) * (b - d)));
}

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

uint32_t timeCounter = 0;
void plasma(uint32_t brightness) {
	uint8_t screenHeight = 24;
	uint8_t screenWidth = 24;

	//float time = HAL_GetTick() % 1000;
	//float time = 20;
	//float time = HAL_GetTick();
	timeCounter += 3;
	float time = timeCounter;

	for(int y = 0; y < screenHeight; y++) {
		for(int x = 0; x < screenWidth; x++) {
			float value = sin(dist(x + time, y, 128.0, 128.0) / 8.0)
				+ sin(dist(x, y, 64.0, 64.0) / 8.0)
				+ sin(dist(x, y + time / 7, 192.0, 64) / 7.0)
				+ sin(dist(x, y, 192.0, 100.0) / 8.0);
			//int colour = (4 + value) * 32;
			int colour = (4 + value) * 32;

			setPixel(x, y, brightness, colour, colour * 2, 64 - colour);

			uint8_t debugText[10];
 			sprintf(debugText, "colour (%.5d) value (%.6d)\r\n", colour, ((int)(value * 1000)));
 			CDC_Transmit_FS(debugText, strlen(debugText));

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
