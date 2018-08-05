#include "ir.h"

/*
 * FROM: https://www.carminenoviello.com/2015/09/04/precisely-measure-microseconds-stm32/
*/
#define delayUS_ASM(us) do {\
	asm volatile (	"MOV R0,%[loops]\n\t"\
			"1: \n\t"\
			"SUB R0, #1\n\t"\
			"CMP R0, #0\n\t"\
			"BNE 1b \n\t" : : [loops] "r" (16*us) : "memory"\
		      );\
} while(0)

void irTX(uint8_t data[], uint8_t dataSize) {
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	HAL_Delay(1);

	uint8_t frameTime = 26;
	uint8_t pulseTimeZero = 8;
	uint8_t frameRemainderTimeZero = frameTime - pulseTimeZero;
	uint8_t pulseTimeOne = 16;
	uint8_t frameRemainderTimeOne = frameTime - pulseTimeOne;

	for(uint8_t dataIndex = 0; dataIndex < dataSize; dataIndex++) {
		for(uint8_t dataBit = 0; dataBit < 8; dataBit++) {
			uint8_t onTime = ((data[dataIndex] >> dataBit) & 0x01) ? pulseTimeOne : pulseTimeZero;
			uint8_t offTime = ((data[dataIndex] >> dataBit) & 0x01) ? frameRemainderTimeOne : frameRemainderTimeZero;

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

			delayUS_ASM(onTime);

			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

			delayUS_ASM(offTime);
		}
	}

	HAL_Delay(1);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

uint8_t irRX(uint8_t data[], uint8_t dataSize) {
	return 0;
}

