#ifndef _BLOCKCHAIN_H_
#define _BLOCKCHAIN_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct Block {
	uint8_t xOffset;
	uint8_t yOffset;
};

struct Blockchain {
	uint32_t colour;
	struct Block blocks[10];
};

struct BlockchainGame {
	uint32_t blocks[24*24];
	struct Blockchain currentBlockchain;
	struct Blockchain nextBlockchain;
};

void initBlockchain(struct BlockchainGame* blockchainGame);
void blockchain(struct BlockchainGame* blockChainGame, uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick);

#endif
