#include "welcome.h"
#include "buttons.h"

//uint32_t welcomeScreenCounter = 20 * 10;

//uint32_t* icons[16];

uint8_t animated = 0;
uint32_t* icon = mm_24;
uint32_t logoState = 0;
uint32_t htpFrameIndex = 0;
uint32_t htpFrameCounter = 0;

uint32_t* htpFrames[18];

//uint8_t iconIndex = BUTTON_SELECT;
void welcome(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick) {

	uint8_t htpFramesLoadIndex = 0;
	htpFrames[htpFramesLoadIndex++] = frame_00;
	htpFrames[htpFramesLoadIndex++] = frame_01;
	htpFrames[htpFramesLoadIndex++] = frame_02;
	htpFrames[htpFramesLoadIndex++] = frame_03;
	htpFrames[htpFramesLoadIndex++] = frame_04;
	htpFrames[htpFramesLoadIndex++] = frame_05;
	htpFrames[htpFramesLoadIndex++] = frame_06;
	htpFrames[htpFramesLoadIndex++] = frame_07;
	htpFrames[htpFramesLoadIndex++] = frame_08;
	htpFrames[htpFramesLoadIndex++] = frame_09;
	htpFrames[htpFramesLoadIndex++] = frame_10;
	htpFrames[htpFramesLoadIndex++] = frame_11;
	htpFrames[htpFramesLoadIndex++] = frame_12;
	htpFrames[htpFramesLoadIndex++] = frame_13;
	htpFrames[htpFramesLoadIndex++] = frame_14;
	htpFrames[htpFramesLoadIndex++] = frame_15;
	htpFrames[htpFramesLoadIndex++] = frame_16;
	htpFrames[htpFramesLoadIndex++] = frame_17;

/*
	if(welcomeScreenCounter > 0) {
		welcomeScreenCounter--;
	}
*/

        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_SELECT, &lastButtonPressTick)) {
		icon = mm_24;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, &lastButtonPressTick)) {
		icon = LUSmile;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2, &lastButtonPressTick)) {
		icon = LDSad;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, &lastButtonPressTick)) {
		icon = LLAngry;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4, &lastButtonPressTick)) {
		icon = LRCrying;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1, &lastButtonPressTick)) {
		animated = 0;
		switch(logoState % 3) {
			case 0:
				icon = RUDefcon;
				break;
			case 1:
				icon = NULL;
				animated = 1;
				break;
			case 2:
			default:
				icon = mm_24;
				break;
		}
		logoState++;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2, &lastButtonPressTick)) {
		icon = RRTongue;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3, &lastButtonPressTick)) {
		icon = RLLaugh;
		animated = 0;
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4, &lastButtonPressTick)) {
		icon = RDDrunk;
		animated = 0;
        }

	if(animated) {
		if(htpFrameCounter++ % 4 == 0) {
			icon = htpFrames[htpFrameIndex++];
			if(htpFrameIndex >= 18) {
				htpFrameIndex = 0;
			}
		}
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

