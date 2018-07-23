#ifndef _BUTTONS_H_
#define _BUTTONS_H_

#define BUTTON_SELECT ((uint16_t) 5) 
#define BUTTON_START  ((uint16_t) 6) 
#define BUTTON_R4     ((uint16_t) 7) 
#define BUTTON_R3     ((uint16_t) 8) 
#define BUTTON_R2     ((uint16_t) 9) 
#define BUTTON_R1     ((uint16_t) 10)
#define BUTTON_L4     ((uint16_t) 11)
#define BUTTON_L3     ((uint16_t) 12)
#define BUTTON_L2     ((uint16_t) 13)
#define BUTTON_L1     ((uint16_t) 14)

uint8_t buttonPressed(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t button, uint32_t* lastButtonPressTick);

#endif
