#ifndef _BLOCKCHAIN_H_
#define _BLOCKCHAIN_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct Block {
	uint8_t xOffset[4];
	uint8_t yOffset[4];
};

struct Blockchain {
	uint32_t tickCount;
	uint8_t x;
	uint8_t y;
	uint8_t rotation;
	uint32_t colour;

	uint8_t blockCount;
	struct Block blocks[10];

	uint32_t lastMoveTickTime;
};

struct BlockchainGame {
	uint32_t score;
	uint8_t gameOver;
	uint32_t blocks[24*24];
	struct Blockchain currentBlockchain;
	struct Blockchain nextBlockchain;
};

void initBlockchain(struct Blockchain* blockchainGame);
void blockchain(struct BlockchainGame* blockChainGame, uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick);

#endif
