#ifndef _SNAKE_H_
#define _SNAKE_H_

#include <sys/types.h>

#include <math.h>
#include <stdlib.h>
#include <string.h>

struct SnakePlayer {
	int16_t x;
	int16_t y;
     
	uint32_t colour;

	// 0 = North
	// 1 = East
	// 2 = South
	// 3 = West
	uint8_t direction;

	// 0 = left buttons on home badge
	// 1 = right buttons on home badge
	uint32_t location;

	uint8_t playing;

	uint32_t lastMovementTick;
	uint32_t speed;

	uint8_t tailSize;

	struct SnakeTail* tail;
};

struct SnakeGame {
	struct SnakePlayer players[10];
	uint8_t snakeCount;

	struct SnakeFood* snakeFood[20];

	uint32_t lastTick;
};

struct SnakeTail {
	struct SnakeTail* next;

	int16_t x;
	int16_t y;
};

struct SnakeFood {
	int16_t x;
	int16_t y;

	int32_t expiryTick;
};

#define SNAKE_DIRECTION_NORTH 0
#define SNAKE_DIRECTION_EAST  1
#define SNAKE_DIRECTION_SOUTH 2
#define SNAKE_DIRECTION_WEST  3

struct SnakeTail* findSnakePlayerTailTail(struct SnakeTail* snakeTail);
struct SnakeTail* createSnakeTail(int16_t x, int16_t y);
void addSnakeTail(struct SnakePlayer* snakePlayer);
void updateSnakeTailPositions(struct SnakeTail* snakeTail, uint16_t x, uint16_t y);
void deleteSnakeTail(struct SnakeTail* snakeTail);
uint8_t tailContainsCoords(struct SnakeTail* snakeTail, uint16_t x, uint16_t y, uint16_t depth);
void initSnakePlayer(struct SnakePlayer* snakePlayer);
void initSnakeFood(struct SnakeFood* snakeFood);
void snake(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t* lastButtonPressTick, uint32_t startButtonPressed);

#endif
