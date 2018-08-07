#include "blockchain.h"

#include "buttons.h"
#include "led_panel.h"

void initBlockchainGame(struct BlockchainGame* blockchainGame) {
}

void blockchain(struct BlockchainGame* blockchainGame, uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick) {
	uint32_t currentTick = HAL_GetTick();

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_START, lastButtonPressTick)) {
	}

/*
	for(int i = 0; i < snakeGame.snakeCount; i++) {
		if((snakeGame.snakeFood[i] == NULL) && (rand() % 10 == 1)) {
			snakeGame.snakeFood[i] = malloc(sizeof(struct SnakeFood));
			initSnakeFood(snakeGame.snakeFood[i]);
		}

		if(currentTick > snakeGame.snakeFood[i]->expiryTick) {
			free(snakeGame.snakeFood[i]);
			snakeGame.snakeFood[i] = NULL;
		}
	}
*/
		
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4, lastButtonPressTick)) {
	}

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4, lastButtonPressTick)) {
	}

}

