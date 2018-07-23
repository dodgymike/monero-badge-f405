#include "led_panel.h"

static uint32_t pixels[576];
//static uint16_t missingLeds[] = { 460, 461 };
static uint16_t missingLeds[] = { };
static uint8_t missingLedCount = 0;

static uint8_t start_frame_data[] = { 
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
        0x00, 0x00, 0x00, 0x00, /* start frame */
};
static uint8_t end_frame_data[] = { 
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff, /* end frame */
        0xff, 0xff, 0xff, 0xff /* end frame */
};


void setPixelColour(uint8_t x, uint8_t y, uint32_t brightness, uint32_t colour) {
	pixels[xyToLedIndex(x, y)] = colour;
}

void setPixel(uint8_t x, uint8_t y, uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue) {
	pixels[xyToLedIndex(x, y)] = rgbToPixel(brightness, red, green, blue);
}

uint32_t rgbToPixel(uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue) {
	//return (((brightness << 3) | 0b00000111) << 24) + (red << 16) + (green << 8) + blue;
	return (((brightness) | 0b11100000) << 24) + (red << 16) + (green << 8) + blue;
}

void ClearPixels()
{
	for(uint16_t ledIndex = 0; ledIndex < 576; ledIndex++) {
		pixels[ledIndex] = 0;
	}
}

void WriteLedPanelFrame(uint8_t ledPanelEnabled)
{
	if(!ledPanelEnabled) {
		return;
	}

	uint8_t led_frame_size = 4;
	uint8_t offBrightness = 0b11110000;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

        HAL_SPI_Transmit(&hspi2, start_frame_data, led_frame_size * 2, HAL_MAX_DELAY);

	uint8_t off_led_data[] = {
		//0x00000111, 0x00, 0x00, 0x00
		//0x11100000, 0x00, 0x00, 0x00
		offBrightness, 0x00, 0x00, 0x00
		//0x00, 0x01, 0x01, 0x01
		//0x00, 0x00, 0x00, 0x00
	};

	for(uint16_t led_count = 0; led_count < 576; led_count++) {
		if(pixels[led_count] > 0) {
/*
			uint8_t brightness = (pixels[led_count] >> 24) & 0xff;
			uint8_t red = (pixels[led_count] >> 16) & 0xff;
			uint8_t green = (pixels[led_count] >> 8) & 0xff;
			uint8_t blue = pixels[led_count] & 0xff;
*/
			//uint8_t brightness = 0b11110000;
			uint8_t brightness = (pixels[led_count] >> 24) & 0xff;
			uint8_t red = (pixels[led_count] >> 16) & 0xff;
			uint8_t green = (pixels[led_count] >> 8) & 0xff;
			uint8_t blue = pixels[led_count] & 0xff;
/*
			uint8_t red = ((pixels[led_count] >> 16) & 0xff) > 0 ? 1 : 0;
			uint8_t green = ((pixels[led_count] >> 8) & 0xff) > 0 ? 1 : 0;
			uint8_t blue = (pixels[led_count] & 0xff) > 0 ? 1 : 0;
*/

			uint8_t led_data[] = {
				//brightness, 0x05, 0x05, 0x05
				brightness, blue, green, red
			};
			HAL_SPI_Transmit(&hspi2, led_data, led_frame_size, HAL_MAX_DELAY);
		} else {
			HAL_SPI_Transmit(&hspi2, off_led_data, led_frame_size, HAL_MAX_DELAY);
		}
	}

        HAL_SPI_Transmit(&hspi2, end_frame_data, led_frame_size * 8, HAL_MAX_DELAY);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        //HAL_Delay(10);

}

uint16_t xyToLedIndex(uint8_t x, uint8_t y) {
	uint16_t ledIndex = 0;
/*
	// for 8x8 panels in 3x3 configuration
	if(y < 8) {
		ledIndex = (x * 8) + y;
	} else if(y < 16) {
		ledIndex = 192 + (x * 8) + (y - 8);
	} else {
		ledIndex = 384 + (x * 8) + (y - 16);
	}
*/

	ledIndex = y * 24;
	ledIndex += x;
/*
	// for badge with alternating up/down rows
	if(y % 2 == 0) {
		ledIndex += x;
	} else {
		ledIndex += 23 - x;
	}
*/

	if(ledIndex >= 576) {
		ledIndex = 0;
	}

	if(missingLedCount > 0) {
		uint16_t originalLedIndex = ledIndex;

		for(int i = 0; i < missingLedCount; i++) {
			if(originalLedIndex >= missingLeds[i]) {
				ledIndex--;
			}
		}
	}

	return ledIndex;
}

void enableLedPanel(uint8_t *ledPanelEnabled) {
/*
*/
	if(*ledPanelEnabled) {
		return;
	}

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

//        ClearPixels();
//        WriteLedPanelFrame(1);

        *ledPanelEnabled = 1; 
}

void disableLedPanel(uint8_t *ledPanelEnabled) {
/*
*/
	if(!*ledPanelEnabled) {
		return;
	}

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);

        ClearPixels();
        WriteLedPanelFrame(1);

        *ledPanelEnabled = 0; 
}

