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

uint8_t frameTime = 26;
uint8_t pulseTimeZero = 8;
uint8_t pulseTimeOne = 16;
uint8_t pulseUnitTime = 4;

void irTX(uint8_t data[], uint8_t dataSize) {
	uint8_t frameRemainderTimeOne = frameTime - pulseTimeOne;
	uint8_t frameRemainderTimeZero = frameTime - pulseTimeZero;

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	delayUS_ASM(frameTime);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	delayUS_ASM(frameTime);

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

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

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
}

uint8_t irRX(uint8_t data[], uint8_t dataSize, uint32_t timeout) {
	uint8_t frameRemainderTimeOne = frameTime - pulseTimeOne;
	uint8_t frameRemainderTimeZero = frameTime - pulseTimeZero;

	uint8_t rxCount = 0;

	uint8_t dataIndex = 0;

	uint32_t startTickTime = HAL_GetTick();
	uint8_t gotStart = 0;
	uint8_t gotEnd = 0;
	uint8_t gotMiddle = 0;

	uint8_t bitIndex = 0;
	uint8_t currentByte = 0;

	startTickTime = HAL_GetTick();
	while(dataSize > 0) {
		if((HAL_GetTick() - startTickTime) > timeout) {
			return 0;
		}

		uint8_t unitCount = 0;
		gotStart = 0;
		gotEnd = 0;
		while(1) {
			if((HAL_GetTick() - startTickTime) > timeout) {
				return 0;
			}

			uint16_t irBit = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_11);

/*
			uint8_t irBitText[20];
			sprintf(irBitText, "irBit (%.2d)\r\n", irBit);
			CDC_Transmit_FS(irBitText, strlen(irBitText));
*/

			if(!gotStart) {
				if(irBit > 0) {
					CDC_Transmit_FS("got start\r\n", 11);
					gotStart = 1;
				}
			} else if(!gotMiddle) {
				if(irBit <= 0) {
					CDC_Transmit_FS("got middle\r\n", 12);
					gotMiddle = 1;
				}
			} else if(!gotEnd) {
				if(irBit > 0) {
					CDC_Transmit_FS("got end\r\n", 9);
					gotEnd = 1;
				}
			}

			if(gotStart && gotMiddle && gotEnd) {
					CDC_Transmit_FS("got all\r\n", 9);
				if((unitCount * pulseUnitTime) >= pulseTimeOne) {
					currentByte |= (1 << bitIndex);
				}

				bitIndex++;

				if(bitIndex > 8) {
					data[dataIndex] = currentByte;
					currentByte = 0;

					dataSize--;
				}

				rxCount++;
				
				break;
			}

			if(gotStart) {
				unitCount++;
			}

			delayUS_ASM(pulseUnitTime);
		}
	}

	return rxCount;
}

