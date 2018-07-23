#ifndef _LED_PANEL_H_
#define _LED_PANEL_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "stm32f4xx_hal.h"

void enableLedPanel(uint8_t *ledPanelEnabled);
void disableLedPanel(uint8_t *ledPanelEnabled);

void setPixelColour(uint8_t x, uint8_t y, uint32_t brightness, uint32_t colour);
void setPixel(uint8_t x, uint8_t y, uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue);

uint32_t rgbToPixel(uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue);
void ClearPixels();
void WriteLedPanelFrame();
uint16_t xyToLedIndex(uint8_t x, uint8_t y);

extern SPI_HandleTypeDef hspi2;

#endif
