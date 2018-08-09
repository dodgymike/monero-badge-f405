#include "blockchain.h"

#include "buttons.h"
#include "led_panel.h"

uint32_t blockColours[] = {
	((255/5) << 16) + ((0/5) << 8) + (0/5),
	((0/5) << 16) + ((255/5) << 8) + (0/5),
	((0/5) << 16) + ((0/5) << 8) + (255/5),
	((255/5) << 16) + ((0/5) << 8) + (255/5),
	((0/5) << 16) + ((255/5) << 8) + (255/5),
	((255/5) << 16) + ((255/5) << 8) + (0/5),
	((255/5) << 16) + ((255/5) << 8) + (255/5),
};

void initBlockchain(struct Blockchain* blockchain) {
	blockchain->lastMoveTickTime = HAL_GetTick();
	blockchain->x = 6;
	blockchain->y = 0;

	blockchain->tickCount = 0;

	blockchain->rotation = rand() % 4;
	blockchain->colour = blockColours[rand() % 6];

	blockchain->blockCount = 4;

	for(int i = 0; i < 4; i += 2) {
		uint8_t blockIndex = 0;
		uint8_t rotationIndex = 0;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 0;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 0;
		blockIndex++;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 0;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 1;
		blockIndex++;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 0;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 2;
		blockIndex++;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 0;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 3;
		blockIndex++;

		rotationIndex++;
		blockIndex = 0;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = -1;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 0;
		blockIndex++;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 0;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 0;
		blockIndex++;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 1;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 0;
		blockIndex++;

		blockchain->blocks[blockIndex].xOffset[rotationIndex + i] = 2;
		blockchain->blocks[blockIndex].yOffset[rotationIndex + i] = 0;
		blockIndex++;
	}
}

void initBlockchainGame(struct BlockchainGame* blockchainGame) {
	for(int y = 0; y < 24; y++) {
		for(int x = 0; x < 24; x++) {
			blockchainGame->blocks[xyToLedIndex(x, y)] = 0;
		}
	}

	blockchainGame->gameOver = 0;
	blockchainGame->score = 0;

	initBlockchain(&(blockchainGame->currentBlockchain));
}

void blockchain(struct BlockchainGame* blockchainGame, uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick) {
	uint32_t currentTick = HAL_GetTick();

	uint8_t roundCount = 1;
	uint8_t bottomRow = 23;

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_START, lastButtonPressTick)) {
	}

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, lastButtonPressTick)) {
		(blockchainGame->currentBlockchain.rotation)++;
		if(blockchainGame->currentBlockchain.rotation >= 4) {
			blockchainGame->currentBlockchain.rotation = 0;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2, lastButtonPressTick)) {
		(blockchainGame->currentBlockchain.x)++;
		if(blockchainGame->currentBlockchain.x >= 12) {
			blockchainGame->currentBlockchain.x = 11;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, lastButtonPressTick)) {
		roundCount = 100;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4, lastButtonPressTick)) {
		(blockchainGame->currentBlockchain.x)--;
		if(blockchainGame->currentBlockchain.x < 0) {
			blockchainGame->currentBlockchain.x = 0;
		}
	}

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3, lastButtonPressTick)) {
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4, lastButtonPressTick)) {
	}

	if((roundCount > 1) || ((blockchainGame->currentBlockchain.tickCount)++ % 20 == 0)) {
		while(roundCount-- > 0) {
			(blockchainGame->currentBlockchain.y)++;

			for(int blockIndex = 0; blockIndex < blockchainGame->currentBlockchain.blockCount; blockIndex++) {
				uint8_t x = blockchainGame->currentBlockchain.x + blockchainGame->currentBlockchain.blocks[blockIndex].xOffset[blockchainGame->currentBlockchain.rotation];
				uint8_t y = blockchainGame->currentBlockchain.y + blockchainGame->currentBlockchain.blocks[blockIndex].yOffset[blockchainGame->currentBlockchain.rotation];
	
				// stop *before* the collision
				y++;
				
				if(
					(y >= bottomRow)
					|| (blockchainGame->blocks[xyToLedIndex(x,y)] != 0)
				) {
					for(int blockIndex = 0; blockIndex < blockchainGame->currentBlockchain.blockCount; blockIndex++) {
						uint8_t x = blockchainGame->currentBlockchain.x + blockchainGame->currentBlockchain.blocks[blockIndex].xOffset[blockchainGame->currentBlockchain.rotation];
						uint8_t y = blockchainGame->currentBlockchain.y + blockchainGame->currentBlockchain.blocks[blockIndex].yOffset[blockchainGame->currentBlockchain.rotation];
	
						blockchainGame->blocks[xyToLedIndex(x, y)] = blockchainGame->currentBlockchain.colour;
					}
				
					initBlockchain(&(blockchainGame->currentBlockchain));
	
					roundCount = 1;
					break;
				}
			}
		}

		uint32_t totalContiguousBlockCount = 0;
		for(int y = (bottomRow - 1); y >= 0; y--) {
			uint8_t contiguousBlockCount = 0;
			for(int x = 0; x < bottomRow; x++) {
				if(blockchainGame->blocks[xyToLedIndex(x, y)] > 0) {
					contiguousBlockCount++;
				}
			}

			if(contiguousBlockCount == 12) {
				totalContiguousBlockCount++;
				for(int copyY = y; copyY > 0; copyY--) {
					for(int x = 0; x < bottomRow; x++) {
						blockchainGame->blocks[xyToLedIndex(x, copyY)] = blockchainGame->blocks[xyToLedIndex(x, copyY-1)];
					}
				}
			}
		}
		if(totalContiguousBlockCount > 0) {
			blockchainGame->score += pow(2, totalContiguousBlockCount);
		}
	}

	for(int y = 0; y < bottomRow; y++) {
		for(int x = 0; x < 24; x++) {
			setPixelColour(x, y, brightness, blockchainGame->blocks[xyToLedIndex(x, y)]);
		}
	}

	for(int blockIndex = 0; blockIndex < blockchainGame->currentBlockchain.blockCount; blockIndex++) {
		uint8_t x = blockchainGame->currentBlockchain.x + blockchainGame->currentBlockchain.blocks[blockIndex].xOffset[blockchainGame->currentBlockchain.rotation];
		uint8_t y = blockchainGame->currentBlockchain.y + blockchainGame->currentBlockchain.blocks[blockIndex].yOffset[blockchainGame->currentBlockchain.rotation];

		setPixelColour(x, y, brightness, blockchainGame->currentBlockchain.colour);
	}

	for(int y = 0; y < 24; y++) {
		for(int x = 12; x < 24; x++) {
			setPixel(x, y, brightness, 30, 30, 30);
		}
	}

	for(int x = 0; x < 24; x++) {
		setPixel(x, bottomRow, brightness, 30, 30, 30);
	}

	uint8_t scoreText[10];
	sprintf(scoreText, "%.3d", blockchainGame->score);
	drawTextColour(brightness, 12, 0, scoreText, 3, 30, 0, 0);
}

