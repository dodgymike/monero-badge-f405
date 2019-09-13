#ifndef _LED_PANEL_H_
#define _LED_PANEL_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"

void enableLedPanel(uint8_t *ledPanelEnabled);
void disableLedPanel(uint8_t *ledPanelEnabled);

//void setPixelColour(uint8_t x, uint8_t y, uint32_t brightness, uint32_t colour);
//void setPixel(uint8_t x, uint8_t y, uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue);

uint32_t rgbToPixel(uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue);
void ClearPixels();
void WriteLedPanelFrame(uint8_t ledPanelEnabled);
uint16_t xyToLedIndex(uint8_t x, uint8_t y);

void drawText(uint8_t brightness, uint8_t x, uint8_t y, char text[], uint8_t length);
void drawTextColour(uint8_t brightness, uint8_t x, uint8_t y, char text[], uint8_t length, uint8_t r, uint8_t g, uint8_t b);

static uint8_t CHAR_HEIGHT = 5;
static uint8_t characters[37 * 6] = {
	/* A */
	5,
	0b01100,
	0b10010,
	0b11110,
	0b10010,
	0b10010,
	/* B */
	5,
	0b11100,
	0b10010,
	0b11100,
	0b10010,
	0b11100,
	/* C */
	4,
	0b1110,
	0b1000,
	0b1000,
	0b1000,
	0b1110,
	/* D */
	4,
	0b1110,
	0b1001,
	0b1001,
	0b1001,
	0b1110,
	/* E */
	4,
	0b1110,
	0b1000,
	0b1110,
	0b1000,
	0b1110,
	/* F */
	4,
	0b1111,
	0b1000,
	0b1111,
	0b1000,
	0b1000,
	/* G */
	4,
	0b1111,
	0b1000,
	0b1011,
	0b1001,
	0b1111,
	/* H */
	4,
	0b1001,
	0b1001,
	0b1111,
	0b1001,
	0b1001,
	/* I */
	4,
	0b1111,
	0b0110,
	0b0110,
	0b0110,
	0b1111,
	/* J */
	4,
	0b1111,
	0b0110,
	0b0110,
	0b0110,
	0b1100,
	/* K */
	4,
	0b1001,
	0b1010,
	0b1100,
	0b1010,
	0b1001,
	/* L */
	4,
	0b1000,
	0b1000,
	0b1000,
	0b1000,
	0b1111,
	/* M */
	6,
	0b100010,
	0b110110,
	0b101010,
	0b100010,
	0b100010,
	/* N */
	4,
	0b1001,
	0b1101,
	0b1011,
	0b1001,
	0b1001,
	/* O */
	4,
	0b1110,
	0b1010,
	0b1010,
	0b1010,
	0b1110,
	/* P */
	4,
	0b1111,
	0b1001,
	0b1111,
	0b1000,
	0b1000,
	/* Q */
	4,
	0b1111,
	0b1001,
	0b1001,
	0b1111,
	0b0001,
	/* R */
	4,
	0b1110,
	0b1010,
	0b1110,
	0b1100,
	0b1010,
	/* S */
	4,
	0b1110,
	0b1000,
	0b1110,
	0b0010,
	0b1110,
	/* T */
	4,
	0b1111,
	0b0110,
	0b0110,
	0b0110,
	0b0110,
	/* U */
	4,
	0b1001,
	0b1001,
	0b1001,
	0b1001,
	0b1111,
	/* V */
	4,
	0b1001,
	0b1001,
	0b1001,
	0b0110,
	0b0000,
	/* W */
	4,
	0b1001,
	0b1001,
	0b1111,
	0b1111,
	0b0110,
	/* X */
	4,
	0b1001,
	0b1001,
	0b0110,
	0b1001,
	0b1001,
	/* Y */
	5,
	0b01010,
	0b01010,
	0b00100,
	0b00100,
	0b00100,
	/* Z */
	4,
	0b1111,
	0b0011,
	0b0110,
	0b1100,
	0b1111,
	/* 0 */
	4,
	0b1110,
	0b1010,
	0b1010,
	0b1010,
	0b1110,
	/* 1 */
	4,
	0b0100,
	0b0100,
	0b0100,
	0b0100,
	0b0100,
	/* 2 */
	4,
	0b1110,
	0b0010,
	0b1110,
	0b1000,
	0b1110,
	/* 3 */
	4,
	0b1110,
	0b0010,
	0b0110,
	0b0010,
	0b1110,
	/* 4 */
	4,
	0b1010,
	0b1010,
	0b1110,
	0b0010,
	0b0010,
	/* 5 */
	4,
	0b1110,
	0b1000,
	0b1110,
	0b0010,
	0b1110,
	/* 6 */
	4,
	0b1110,
	0b1000,
	0b1110,
	0b1010,
	0b1110,
	/* 7 */
	4,
	0b1110,
	0b0010,
	0b0010,
	0b0010,
	0b0010,
	/* 8 */
	4,
	0b1110,
	0b1010,
	0b1110,
	0b1010,
	0b1110,
	/* 9 */
	4,
	0b1110,
	0b1010,
	0b1110,
	0b0010,
	0b0010,
	/* SPACE */
	4,
	0b0000,
	0b0000,
	0b0000,
	0b0000,
	0b0000,
};

#endif
