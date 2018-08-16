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

// partyparrot!
uint32_t ppFrameIndex = 0;
uint32_t ppFrameCounter = 0;
uint32_t* ppFrames[10];

#define KONAMI_CODE_SIZE 10

uint8_t konamiCodeReference[] = {
	BUTTON_L1,
	BUTTON_L1,
	BUTTON_L3,
	BUTTON_L3,
	BUTTON_L4,
	BUTTON_L2,
	BUTTON_L4,
	BUTTON_L2,
	BUTTON_R2,
	BUTTON_R3
};

uint8_t konamiCodes[KONAMI_CODE_SIZE];
uint8_t konamiIndex = 0;

void addKonamiCodeButtonEntry(uint32_t button) {
	for(int i = KONAMI_CODE_SIZE - 1; i > 0; i--) {
		konamiCodes[i] = konamiCodes[i - 1];
	}

	konamiCodes[0] = button;
}

uint8_t compareKonamiCodes() {
	for(int i = 0; i < KONAMI_CODE_SIZE; i++) {
		if(konamiCodes[i] != konamiCodeReference[KONAMI_CODE_SIZE - i - 1]) {
			return 0;
		}
	}

	return 1;
}

#define PARTY_PARROT_CODE_SIZE 3

uint8_t partyParrotCodeReference[] = {
	BUTTON_L4,
	BUTTON_L4,
	BUTTON_L2,
};

uint8_t partyparrotCodes[PARTY_PARROT_CODE_SIZE];
uint8_t partyparrotIndex = 0;

void addPartyParrotCodeButtonEntry(uint32_t button) {
	for(int i = PARTY_PARROT_CODE_SIZE - 1; i > 0; i--) {
		partyparrotCodes[i] = partyparrotCodes[i - 1];
	}

	partyparrotCodes[0] = button;
}

uint8_t comparePartyParrotCodes() {
	for(int i = 0; i < PARTY_PARROT_CODE_SIZE; i++) {
		if(partyparrotCodes[i] != partyParrotCodeReference[PARTY_PARROT_CODE_SIZE - i - 1]) {
			return 0;
		}
	}

	return 1;
}

//uint8_t iconIndex = BUTTON_SELECT;
void welcome(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick, uint32_t startButtonPressed) {

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

	uint8_t ppFramesLoadIndex = 0;
	ppFrames[ppFramesLoadIndex++] = partyparrot_00;
	ppFrames[ppFramesLoadIndex++] = partyparrot_01;
	ppFrames[ppFramesLoadIndex++] = partyparrot_02;
	ppFrames[ppFramesLoadIndex++] = partyparrot_03;
	ppFrames[ppFramesLoadIndex++] = partyparrot_04;
	ppFrames[ppFramesLoadIndex++] = partyparrot_05;
	ppFrames[ppFramesLoadIndex++] = partyparrot_06;
	ppFrames[ppFramesLoadIndex++] = partyparrot_07;
	ppFrames[ppFramesLoadIndex++] = partyparrot_08;
	ppFrames[ppFramesLoadIndex++] = partyparrot_09;

/*
	if(welcomeScreenCounter > 0) {
		welcomeScreenCounter--;
	}
*/

        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_SELECT, lastButtonPressTick)) {
		icon = mm_24;
		animated = 0;

        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, lastButtonPressTick)) {
		icon = LUSmile;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_L1);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2, lastButtonPressTick)) {
		icon = LDSad;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_L2);
		addPartyParrotCodeButtonEntry(BUTTON_L2);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, lastButtonPressTick)) {
		icon = LLAngry;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_L3);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4, lastButtonPressTick)) {
		icon = LRCrying;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_L4);
		addPartyParrotCodeButtonEntry(BUTTON_L4);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1, lastButtonPressTick)) {
		animated = 0;
		switch(logoState % 2) {
			case 0:
				icon = RUDefcon;
				break;
			case 1:
			default:
				icon = mm_24;
				break;
		}
		logoState++;

		addKonamiCodeButtonEntry(BUTTON_R1);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2, lastButtonPressTick)) {
		icon = RRTongue;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_R2);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3, lastButtonPressTick)) {
		icon = RLLaugh;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_R3);
        }
        if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4, lastButtonPressTick)) {
		icon = RDDrunk;
		animated = 0;

		addKonamiCodeButtonEntry(BUTTON_R4);
        }

	if(compareKonamiCodes()) {
		if(htpFrameCounter++ % 4 == 0) {
			icon = htpFrames[htpFrameIndex++];
			if(htpFrameIndex >= 18) {
				htpFrameIndex = 0;
			}
		}
	}

	if(comparePartyParrotCodes()) {
		if(ppFrameCounter++ % 4 == 0) {
			icon = ppFrames[ppFrameIndex++];
			if(ppFrameIndex >= 10) {
				ppFrameIndex = 0;
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

