#include "snake.h"

#include "buttons.h"
#include "led_panel.h"

struct SnakeGame snakeGame;

struct SnakeTail* findSnakePlayerTailTail(struct SnakeTail* snakeTail) {
	if(snakeTail->next == NULL) {
		return snakeTail;
	}

	return findSnakePlayerTailTail(snakeTail->next);
}

struct SnakeTail* createSnakeTail(int16_t x, int16_t y) {
	struct SnakeTail* snakeTail = malloc(sizeof(struct SnakeTail));
	snakeTail->x = x;
	snakeTail->y = y;
	snakeTail->next = NULL;

	return snakeTail;
}

void addSnakeTail(struct SnakePlayer* snakePlayer) {
	struct SnakeTail* newTail = createSnakeTail(snakePlayer->x, snakePlayer->y);

	if(snakePlayer->tail == NULL) {
		snakePlayer->tail = newTail;
	} else {
		struct SnakeTail* snakeTail = findSnakePlayerTailTail(snakePlayer->tail);
		snakeTail->next = newTail;
	}
}

void updateSnakeTailPositions(struct SnakeTail* snakeTail, uint16_t x, uint16_t y) {
	if(snakeTail == NULL) {
		return;
	}

	if(snakeTail->next != NULL) {
		updateSnakeTailPositions(snakeTail->next, snakeTail->x, snakeTail->y);
	}

	snakeTail->x = x;
	snakeTail->y = y;
}

void deleteSnakeTail(struct SnakeTail* snakeTail) {
	if(snakeTail == NULL) {
		return;
	}

	if(snakeTail->next != NULL) {
		deleteSnakeTail(snakeTail->next);
	}

	free(snakeTail);
	snakeTail->next = NULL;
}

uint8_t tailContainsCoords(struct SnakeTail* snakeTail, uint16_t x, uint16_t y, uint16_t depth) {
	if(snakeTail == NULL) {
		return 0;
	}

	if((snakeTail->x == x) && (snakeTail->y == y)) {
		return 1;
	}

	if(snakeTail->next != NULL) {
		return tailContainsCoords(snakeTail->next, x, y, depth + 1);
	}

	return 0;
}

void initSnakePlayer(struct SnakePlayer* snakePlayer) {
	snakePlayer->playing = 1;
	snakePlayer->x = rand() % 24;
	snakePlayer->y = rand() % 24;
	snakePlayer->colour = rgbToPixel(20, rand() % 0b11111, rand() % 0b11111, rand() % 0b11111);
	snakePlayer->location = 0;
	snakePlayer->direction = 0;
	snakePlayer->lastMovementTick = 0;
	snakePlayer->speed = 200;

	snakePlayer->tailSize = 0;
	snakePlayer->tail = NULL;
}

void initSnakeFood(struct SnakeFood* snakeFood) {
	snakeFood->x = rand() % 24;
	snakeFood->y = rand() % 24;

	snakeFood->expiryTick = HAL_GetTick() + 5000 + (rand() % 3000);
}

void snake(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness) {
	uint32_t currentTick = HAL_GetTick();

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_START)) {
		for(int i = 0; i < snakeGame.snakeCount; i++) {
			deleteSnakeTail(snakeGame.players[i].tail);

			if(snakeGame.snakeFood[i] != NULL) {
				free(snakeGame.snakeFood[i]);
				snakeGame.snakeFood[i] = NULL;
			}
		}

		snakeGame.snakeCount = 0;
	}

	if(snakeGame.snakeCount == 0) {
		initSnakePlayer(&(snakeGame.players[0]));
		snakeGame.snakeCount++;

		initSnakePlayer(&(snakeGame.players[1]));
		snakeGame.snakeCount++;
	}

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
		
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1)) {
		snakeGame.players[0].direction = SNAKE_DIRECTION_NORTH;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2)) {
		snakeGame.players[0].direction = SNAKE_DIRECTION_EAST;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3)) {
		snakeGame.players[0].direction = SNAKE_DIRECTION_SOUTH;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4)) {
		snakeGame.players[0].direction = SNAKE_DIRECTION_WEST;
	}

	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1)) {
		snakeGame.players[1].direction = SNAKE_DIRECTION_NORTH;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2)) {
		snakeGame.players[1].direction = SNAKE_DIRECTION_EAST;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3)) {
		snakeGame.players[1].direction = SNAKE_DIRECTION_SOUTH;
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4)) {
		snakeGame.players[1].direction = SNAKE_DIRECTION_WEST;
	}

	for(int snakeIndex = 0; snakeIndex < snakeGame.snakeCount; snakeIndex++) {
		if(snakeGame.players[snakeIndex].playing == 0) {
			continue;
		}

		if((currentTick - snakeGame.players[snakeIndex].lastMovementTick) > snakeGame.players[snakeIndex].speed) {
			snakeGame.players[snakeIndex].lastMovementTick = currentTick;

			switch(snakeGame.players[snakeIndex].direction) {
				case SNAKE_DIRECTION_NORTH:
					snakeGame.players[snakeIndex].y--;
					break;
				case SNAKE_DIRECTION_EAST:
					snakeGame.players[snakeIndex].x++;
					break;
				case SNAKE_DIRECTION_WEST:
					snakeGame.players[snakeIndex].x--;
					break;
				case SNAKE_DIRECTION_SOUTH:
					snakeGame.players[snakeIndex].y++;
					break;
			}

			if(snakeGame.players[snakeIndex].x >= 24) {
				snakeGame.players[snakeIndex].x %= 24;
			}
			if(snakeGame.players[snakeIndex].x < 0) {
				snakeGame.players[snakeIndex].x = 23;
			}
			if(snakeGame.players[snakeIndex].y >= 24) {
				snakeGame.players[snakeIndex].y %= 24;
			}
			if(snakeGame.players[snakeIndex].y < 0) {
				snakeGame.players[snakeIndex].y = 23;
			}

			updateSnakeTailPositions(snakeGame.players[snakeIndex].tail, snakeGame.players[snakeIndex].x, snakeGame.players[snakeIndex].y);

			for(int snakeFoodIndex = 0; snakeFoodIndex < snakeGame.snakeCount; snakeFoodIndex++) {
				if(snakeGame.snakeFood[snakeFoodIndex] != NULL) {
					struct SnakeFood* snakeFood = snakeGame.snakeFood[snakeFoodIndex];
					if((snakeFood->x == snakeGame.players[snakeIndex].x) && (snakeFood->y == snakeGame.players[snakeIndex].y)) {
						free(snakeFood);
						snakeFood = NULL;
						snakeGame.snakeFood[snakeFoodIndex] = NULL;

						addSnakeTail(&snakeGame.players[snakeIndex]);
						snakeGame.players[snakeIndex].speed -= 3;
					}
				}
			}
		}
	}

	// check for collisions
	for(int snakeIndex = 0; snakeIndex < snakeGame.snakeCount; snakeIndex++) {
		for(int opponentSnakeIndex = 0; opponentSnakeIndex < snakeGame.snakeCount; opponentSnakeIndex++) {
			if(snakeIndex == opponentSnakeIndex) {
				continue;
			}

			if(tailContainsCoords(snakeGame.players[opponentSnakeIndex].tail, snakeGame.players[snakeIndex].x, snakeGame.players[snakeIndex].y, 0)) {
				// GAME OVER
				snakeGame.players[snakeIndex].playing = 0;
				return;
			}

			if((snakeGame.players[snakeIndex].x == snakeGame.players[opponentSnakeIndex].x) && (snakeGame.players[snakeIndex].y == snakeGame.players[opponentSnakeIndex].y)) {
				// GAME OVER
				snakeGame.players[snakeIndex].playing = 0;
				snakeGame.players[opponentSnakeIndex].playing = 0;
				return;
			}
		}
	}
	

	// DRAW SNAKE + TAIL
	for(int snakeIndex = 0; snakeIndex < snakeGame.snakeCount; snakeIndex++) {
		setPixelColour(snakeGame.players[snakeIndex].x, snakeGame.players[snakeIndex].y, brightness, snakeGame.players[snakeIndex].colour);
		struct SnakeTail* snakeTail = snakeGame.players[snakeIndex].tail;
		while(snakeTail != NULL) {
			setPixelColour(snakeTail->x, snakeTail->y, brightness, snakeGame.players[snakeIndex].colour);
			snakeTail = snakeTail->next;
		}
	}

	// DRAW SNAKE FOOD
	for(int snakeFoodIndex = 0; snakeFoodIndex < snakeGame.snakeCount; snakeFoodIndex++) {
		if(snakeGame.snakeFood[snakeFoodIndex] != NULL) {
			setPixel(snakeGame.snakeFood[snakeFoodIndex]->x, snakeGame.snakeFood[snakeFoodIndex]->y, 0b111, 0b11111, 0, 0);
		}
	}
}

