#include "welcome.h"
#include "buttons.h"

//uint32_t welcomeScreenCounter = 20 * 10;

//uint32_t* icons[16];

uint32_t* icon = mm_24;
uint32_t logoState = 0;

//uint8_t iconIndex = BUTTON_SELECT;
void welcome(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick) {

/*
	if(welcomeScreenCounter > 0) {
		welcomeScreenCounter--;
	}
*/

        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_SELECT, &lastButtonPressTick)) {
		icon = mm_24;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, &lastButtonPressTick)) {
		icon = LUSmile;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2, &lastButtonPressTick)) {
		icon = LDSad;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, &lastButtonPressTick)) {
		icon = LLAngry;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4, &lastButtonPressTick)) {
		icon = LRCrying;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1, &lastButtonPressTick)) {
		if(logoState++ % 2 == 0) {
			icon = RUDefcon;
		} else {
			icon = mm_24;
		}
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2, &lastButtonPressTick)) {
		icon = RRTongue;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3, &lastButtonPressTick)) {
		icon = RLLaugh;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4, &lastButtonPressTick)) {
		icon = RDDrunk;
        }

	for(uint32_t y = 0; y < 24; y++) {
		for(uint32_t x = 0; x < 24; x++) {
			setPixelColour(x, y, brightness, icon[y * 24 + x]);

/*
			if(welcomeScreenCounter > (20 * 5)) {
				//setPixelColour(x, y, brightness, welcomeScreenData[y * 24 + x]);
				setPixelColour(x, y, brightness, icons[y * 24 + x]);
			} else {
				setPixelColour(x, y, brightness, icons[y * 24 + x]);
				//setPixelColour(x, y, brightness, defconSmileyData[y * 24 + x]);
			}
*/
		}
	}
}

