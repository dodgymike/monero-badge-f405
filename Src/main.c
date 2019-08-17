/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mpu6000.h"
#include "snake.h"
#include "buttons.h"
#include "led_panel.h"
#include "blockchain.h"
#include "servo.h"
#include "slave.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static GPIO_InitTypeDef  GPIO_InitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#define MODE_RAIN        0b0000000000000001
#define MODE_SNAKE       0b0000000000000010
#define MODE_BLIND       0b0000000000000100
#define MODE_RANDOM      0b0000000000001000
#define MODE_DEBUG       0b0000000000010000
#define MODE_EYE         0b0000000000100000
#define MODE_WELCOME     0b0000000001000000
#define MODE_BLOCKCHAIN  0b0000000010000000
#define MODE_PLASMA      0b0000000100000000

#define SLAVE_MODE_RAIN		0
#define SLAVE_MODE_SNAKE	1
#define SLAVE_MODE_BLIND	2
#define SLAVE_MODE_RANDOM	3
#define SLAVE_MODE_EYE		4
#define SLAVE_MODE_WELCOME	5
#define SLAVE_MODE_BLOCKCHAIN	6
#define SLAVE_MODE_PLASMA	7

#define PWM_TOP 2001
#define PWM_BOTTOM 550
#define PWM_RANGE (PWM_TOP - PWM_BOTTOM)
#define PWM_RANGE_MIDPOINT (PWM_RANGE / 2.0)
const float degreesPerPwm = PWM_RANGE / 180;

uint8_t slaveModeEnabled() {
	return !HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
}

void setSlaveMode(uint8_t slaveMode) {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET); // SET OTHER BADGE TO SLAVE
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, (slaveMode & (1 << 2)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, (slaveMode & (1 << 1)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, (slaveMode & (1 << 0)) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint16_t calculateServoAnglePwm(float angle) {
	angle += 90;
	uint16_t servoPwm = PWM_BOTTOM + (angle * degreesPerPwm);

	if(servoPwm > PWM_TOP) {
		servoPwm = PWM_TOP;
	}
	if(servoPwm < PWM_BOTTOM) {
		servoPwm = PWM_BOTTOM;
	}

	return servoPwm;
}

uint8_t buttonPressed(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t button, uint32_t* lastButtonPressTick) {
	uint8_t buttonPressed = 0;

	if((buttonState[button] < 256) && (buttonAccumulators[button] > 256)) {
		buttonPressed = 1;
		buttonState[button] = 512;

		*lastButtonPressTick = HAL_GetTick();
	} else if(buttonAccumulators[button] < 256) {
		buttonState[button] = 0;
	}

	return buttonPressed;
}

struct AccRingBuffer {
	int16_t x;
	int16_t y;
	int16_t z;

	struct AccRingBuffer* next;
};
struct AccRingBuffer* accRingBuffer;

void initRingBufferEntry(struct AccRingBuffer* ringBufferEntry) {
	ringBufferEntry->x = 0;
	ringBufferEntry->y = 0;
	ringBufferEntry->z = 0;

	ringBufferEntry->next = NULL;
}

void initAccRingBuffer(uint8_t bufferSize) {
	accRingBuffer = malloc(sizeof(struct AccRingBuffer));
	struct AccRingBuffer* curAccRingBuffer = accRingBuffer;
	initRingBufferEntry(curAccRingBuffer);

	while(bufferSize-- > 1) {
		struct AccRingBuffer* nextAccRingBuffer = malloc(sizeof(struct AccRingBuffer));
		initRingBufferEntry(nextAccRingBuffer);

		nextAccRingBuffer->next = accRingBuffer;
		curAccRingBuffer->next = nextAccRingBuffer;

		curAccRingBuffer = nextAccRingBuffer;
	}
}

void moveToNextAccRingBufferEntry() {
	struct AccRingBuffer* nextAccRingBuffer = accRingBuffer->next;
	accRingBuffer = nextAccRingBuffer;
}

void calculateMeans(uint8_t bufferSize, uint16_t* ret_x, uint16_t* ret_y, uint16_t* ret_z) {
	uint32_t x = 0;
	uint32_t y = 0;
	uint32_t z = 0;

	for(int i = 0; i < bufferSize; i++) {
		printf("arb x (%d) y (%d) z (%d)\n", accRingBuffer->x, accRingBuffer->y, accRingBuffer->z);
		x += accRingBuffer->x;
		y += accRingBuffer->y;
		z += accRingBuffer->z;

		moveToNextAccRingBufferEntry();
	}

	x /= bufferSize;
	y /= bufferSize;
	z /= bufferSize;

	(*ret_x) = x;
	(*ret_y) = y;
	(*ret_z) = z;
}

void calculateStdDevs(uint8_t bufferSize, uint16_t* mean_x, uint16_t* mean_y, uint16_t* mean_z, uint16_t* ret_x, uint16_t* ret_y, uint16_t* ret_z) {
	uint32_t total_diff_x = 0;
	uint32_t total_diff_y = 0;
	uint32_t total_diff_z = 0;

	for(int i = 0; i < bufferSize; i++) {
		total_diff_x += pow(accRingBuffer->x - *mean_x, 2);
		total_diff_y += pow(accRingBuffer->y - *mean_y, 2);
		total_diff_z += pow(accRingBuffer->z - *mean_z, 2);

		moveToNextAccRingBufferEntry();
	}

	total_diff_x /= bufferSize;
	total_diff_y /= bufferSize;
	total_diff_z /= bufferSize;

	(*ret_x) = sqrt(total_diff_x);
	(*ret_y) = sqrt(total_diff_y);
	(*ret_z) = sqrt(total_diff_z);
}
 void removeOutliers(uint8_t bufferSize, uint16_t* mean_x, uint16_t* mean_y, uint16_t* mean_z, uint16_t* std_x, uint16_t* std_y, uint16_t* std_z) {
        for(int i = 0; i < bufferSize; i++) {
		if(accRingBuffer->x > (*mean_x + 2 * *std_x)) {
			accRingBuffer->x = (*mean_x + 2 * *std_x);
		} else if(accRingBuffer->x < (*mean_x - 2 * *std_x)) {
			accRingBuffer->x = (*mean_x - 2 * *std_x);
		}

		if(accRingBuffer->y > (*mean_y + 2 * *std_y)) {
			accRingBuffer->y = (*mean_y + 2 * *std_y);
		} else if(accRingBuffer->y < (*mean_y - 2 * *std_y)) {
			accRingBuffer->y = (*mean_y - 2 * *std_y);
		}

		if(accRingBuffer->z > (*mean_z + 2 * *std_z)) {
			accRingBuffer->z = (*mean_z + 2 * *std_z);
		} else if(accRingBuffer->z < (*mean_z - 2 * *std_z)) {
			accRingBuffer->z = (*mean_z - 2 * *std_z);
		}

                moveToNextAccRingBufferEntry();
        }
}

void serialSend(uint8_t* buffer) {
	//CDC_Transmit_FS(buffer, strlen(buffer));
        //HAL_UART_Receive(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart6, buffer, 2, HAL_MAX_DELAY);
//        HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
}

/*
uint8_t spiBusReadRegister(SPI_HandleTypeDef* spi, uint8_t mpuRegister) {
	uint8_t registerWithFlag = 0b10000000 | mpuRegister;
	uint8_t rxBuffer[20];

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	//HAL_Delay(1);

	//HAL_SPI_TransmitReceive(spi, &registerWithFlag, rxBuffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(spi, &registerWithFlag, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spi, &rxBuffer, 1, HAL_MAX_DELAY);

	//HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return rxBuffer[0];
}

uint8_t spiBusReadRegisterBuffer(SPI_HandleTypeDef* spi, uint8_t mpuRegister, uint8_t* rxBuffer, uint8_t bufferSize) {
	uint8_t registerWithFlag = 0b10000000 | mpuRegister;

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_Delay(1);

	//HAL_SPI_TransmitReceive(spi, &registerWithFlag, rxBuffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(spi, &registerWithFlag, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spi, rxBuffer, bufferSize, HAL_MAX_DELAY);

	HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return rxBuffer[0];
}
*/

/*
157     spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
158     spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
*/

/*
uint8_t spiBusWriteRegister(SPI_HandleTypeDef* spi, uint8_t mpuRegister, uint8_t value) {
	uint8_t writeBuffer[2] = { 0b00000000 | mpuRegister, value };

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, writeBuffer, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}
*/

/*
uint8_t initMPU6000() {
	serialSend("Initialising MPU6000");
	//spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
        HAL_Delay(200);

	uint8_t debugBuffer[100];
	
	while(1) {
	//for(int i = 0; i < 5; i++) {
		serialSend("Sending Whoami");
		uint8_t whoAmI = spiBusReadRegister(&hspi1, MPU_RA_WHO_AM_I);

		if(whoAmI == MPU6000_WHO_AM_I_CONST) {
			sprintf(debugBuffer, "Got MPU6000\r\n");
			serialSend(debugBuffer);
			break;
		}

		sprintf(debugBuffer, "WHOAMI returned (%d)\r\n", whoAmI);
		serialSend(debugBuffer);
        	//HAL_Delay(200);
        	HAL_Delay(20);
	}

       	HAL_Delay(20);
	const uint8_t productID = spiBusReadRegister(&hspi1, MPU_RA_PRODUCT_ID);
	char* productIDString;
	switch(productID) {
		case MPU6000ES_REV_C4:
			productIDString = "MPU6000ES_REV_C4";
			break;
		case MPU6000ES_REV_C5:
			productIDString = "MPU6000ES_REV_C5";
			break;
		case MPU6000ES_REV_D6:
			productIDString = "MPU6000ES_REV_D6";
			break;
		case MPU6000ES_REV_D7:
			productIDString = "MPU6000ES_REV_D7";
			break;
		case MPU6000ES_REV_D8:
			productIDString = "MPU6000ES_REV_D8";
			break;
		case MPU6000_REV_C4:
			productIDString = "MPU6000ES_REV_C4";
			break;
		case MPU6000_REV_C5:
			productIDString = "MPU6000ES_REV_C5";
			break;
		case MPU6000_REV_D6:
			productIDString = "MPU6000ES_REV_D6";
			break;
		case MPU6000_REV_D7:
			productIDString = "MPU6000ES_REV_D7";
			break;
		case MPU6000_REV_D8:
			productIDString = "MPU6000ES_REV_D8";
			break;
		case MPU6000_REV_D9:
			productIDString = "MPU6000ES_REV_D9";
			break;
		case MPU6000_REV_D10:
			productIDString = "MPU6000ES_REV_D10";
			break;
		default:
			productIDString = "UNKNOWN MPU";
	}

	sprintf(debugBuffer, "PRODUCTID returned (%d) (%s)\r\n", productID, productIDString);
	serialSend(debugBuffer);

    // Device Reset
    spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    HAL_Delay(150);

    spiBusWriteRegister(&hspi1, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    HAL_Delay(150);

    // Clock Source PPL with Z axis gyro reference
    //spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, MPU_CLK_INT_8MHZ);
    HAL_Delay(15);

    // Disable Primary I2C Interface
    spiBusWriteRegister(&hspi1, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    HAL_Delay(15);

    spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_2, 0x00);
    HAL_Delay(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    //spiBusWriteRegister(&hspi1, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
    spiBusWriteRegister(&hspi1, MPU_RA_SMPLRT_DIV, 7);
    HAL_Delay(15);

    // DLPF
    spiBusWriteRegister(&hspi1, MPU_RA_CONFIG, 4 << 0);
    HAL_Delay(15);

    // Gyro +/- 1000 DPS Full Scale
    spiBusWriteRegister(&hspi1, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    HAL_Delay(15);

    // Accel +/- 8 G Full Scale
    //spiBusWriteRegister(&hspi1, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    spiBusWriteRegister(&hspi1, MPU_RA_ACCEL_CONFIG, INV_FSR_2G << 3);
    HAL_Delay(15);

    spiBusWriteRegister(&hspi1, MPU_RA_FIFO_EN, 0 << 7 | 1 << 6 | 1 << 5 | 1 << 4 | 1 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    HAL_Delay(15);

    spiBusWriteRegister(&hspi1, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 1 << 5 | 0 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    HAL_Delay(15);

    spiBusWriteRegister(&hspi1, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    HAL_Delay(15);

    return productID;
}
*/

/*
void readAccAndGyro(int16_t* accData)
{
//    int16_t data[6];

    uint8_t mpuData[12];
    bzero(mpuData, 12);

    //spiBusReadRegisterBuffer(&hspi1, MPU_RA_ACCEL_XOUT_H, mpuData, 12);

    uint8_t mpuDataIndex = 0;
    accData[0] = (((int8_t)mpuData[0]) << 8) + ((uint8_t)mpuData[1]);
    accData[1] = (((int8_t)mpuData[2]) << 8) + ((uint8_t)mpuData[3]);
    accData[2] = (((int8_t)mpuData[4]) << 8) + ((uint8_t)mpuData[5]);

    accData[3] = (((int8_t)mpuData[6]) << 8) + ((uint8_t)mpuData[7]);
    accData[4] = (((int8_t)mpuData[8]) << 8) + ((uint8_t)mpuData[9]);
    accData[5] = (((int8_t)mpuData[10]) << 8) + ((uint8_t)mpuData[11]);

}
*/

struct Particle {
	uint8_t x;
	uint8_t y;
};

void blind(uint32_t brightness, uint32_t whiteLevel, uint32_t startButtonPressed) {
	for(uint8_t x = 0; x < 24; x++) {
		for(uint8_t y = 0; y < 24; y++) {
			setPixel(x, y, brightness, whiteLevel, whiteLevel, whiteLevel);
		}
	}
}

void lowBatteryScreen(uint32_t brightness, uint32_t* batteryFlashCounter) {
	if((*batteryFlashCounter)++ % 50 > 25) {
		for(int y = 9; y < 9 + 6; y++) {
			setPixel(10, y, brightness, 0b1111, 0, 0);
			setPixel(13, y, brightness, 0b1111, 0, 0);
		}
		
		for(int x = 11; x <= 12; x++) {
			setPixel(x, 8, brightness, 0b1111, 0, 0);
			setPixel(x, 9, brightness, 0b1111, 0, 0);
			setPixel(x, 14, brightness, 0b1111, 0, 0);
		}
	}
}

void random_pixels(uint32_t brightness, uint32_t startButtonPressed) {
	for(uint8_t x = 0; x < 24; x++) {
		for(uint8_t y = 0; y < 24; y++) {
			setPixel(x, y, brightness, rand() % 0b11111, rand() % 0b11111, rand() % 0b11111);
		}
	}
}

static const uint8_t maxParticles = 32;
struct Particle* particles[32];
uint32_t rainBrightness = 0b111;
uint32_t rainRed = 0b11111;
uint32_t rainGreen = 0b11111;
uint32_t rainBlue = 0b11111;

void rain(int xAcc, int yAcc, uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t* lastButtonPressTick, uint32_t startButtonPressed) {
}

void binToHex(uint8_t in[], uint8_t out[], uint8_t byteCount) {
	out[0] = 0;

	for(int i = 0; i < byteCount; i++) {
		sprintf(out + (i * 3), "%.3d", in[i]);
	}
}

uint32_t irDelayDebug = 0;
uint8_t irDelay = 26;
void debug(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness, uint32_t batteryAdcAverage, uint32_t batteryVoltage100, uint32_t startButtonPressed) {
	for(int i = 0; i < 16; i++) {
		setPixel(i, 0, brightness, (buttonState[i] > 256) ? 20 : 0, (buttonState[i] <= 256) ? 20 : 0, 0);
		setPixel(i, 1, brightness, (buttonAccumulators[i] > 256) ? 20 : 0, (buttonAccumulators[i] <= 256) ? 20 : 0, 0);
		setPixel(i, 2, brightness, (buttonAccumulators[i] > 128) ? 20 : 0, (buttonAccumulators[i] <= 128) ? 20 : 0, 0);
		setPixel(i, 3, brightness, (buttonAccumulators[i] > 64) ? 20 : 0, (buttonAccumulators[i] <= 64) ? 20 : 0, 0);
		setPixel(i, 4, brightness, (buttonAccumulators[i] > 32) ? 20 : 0, (buttonAccumulators[i] <= 32) ? 20 : 0, 0);
		setPixel(i, 5, brightness, (buttonAccumulators[i] > 16) ? 20 : 0, (buttonAccumulators[i] <= 16) ? 20 : 0, 0);
		setPixel(i, 6, brightness, (buttonAccumulators[i] > 0) ? 20 : 0, (buttonAccumulators[i] <= 0) ? 20 : 0, 0);
	}

	//drawText(brightness, 0, 9, "monero", 6);

	if(irDelayDebug++ % 30 > 1) {
		//irTXWithDelay("hello\r\n", 7, irDelayDebug % 100);
	}

	char batteryAdcValueText[4];
	sprintf(batteryAdcValueText, "%.4lu", batteryAdcAverage);
	drawText(brightness, 0, 7, batteryAdcValueText, 4);

	//uint8_t batteryVoltageMajor = batteryVoltage100/100;
	//uint8_t batteryVoltageMinor = (batteryVoltage100 - (batteryVoltageMajor * 100));
	char batteryVoltageText[10];
	sprintf(batteryVoltageText, "%.3d", batteryVoltage100);

	drawText(brightness, 0, 13, batteryVoltageText, 3);

	uint32_t lastButtonPressTick = 0;
	uint8_t upPressed = buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, &lastButtonPressTick);
	uint8_t downPressed = buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, &lastButtonPressTick);

	if(upPressed) {
		irDelay++;
	}
	if(downPressed) {
		irDelay--;
	}

	char irDelayText[4];
	sprintf(irDelayText, "%.4d", irDelay);
	drawText(brightness, 0, 19, irDelayText, 4);
	uint8_t irData[100];
	uint8_t receivedDataCount = irRX(irData, 100);

	uint8_t hexOut[30];
	binToHex(irData, hexOut, receivedDataCount);

	uint8_t irDebugText[100];
	sprintf(irDebugText, "Received (%d) bytes hex (%s)\r\n", receivedDataCount, hexOut);
	uint8_t irDisplayText[10];
	sprintf(irDisplayText, "%.2d", receivedDataCount);

	//drawText(brightness, 13, 15, irDisplayText, 2);
	
	//CDC_Transmit_FS(irDebugText, strlen(irDebugText));
/*
	CDC_Transmit_FS(batteryAdcValueText, strlen(batteryAdcValueText));
	CDC_Transmit_FS("\r\n", 2);
	CDC_Transmit_FS(batteryVoltageText, strlen(batteryVoltageText));
	CDC_Transmit_FS("\r\n", 2);
*/
}

uint16_t readButtons() {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        //HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        //HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        //HAL_Delay(1);

	// load   2 - PB5
	// signal 1 - PB0
	// clock  3 - PA3

       	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	// latch the input buttons
	int delayDuration = 10;
	int delayCount = 0;
	//int8_t buttonDebugString[40];
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	// WEIRD CLOCKING PROBLEM, WHICH NO LONGER APPLIES
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

	uint16_t buttonBits = 0;
        for(int bitCount = 0; bitCount < 16; bitCount++) {
        //HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_SET);
        //HAL_Delay(1);
		uint16_t buttonBit = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
		//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, GPIO_PIN_RESET);
        //HAL_Delay(1);

		buttonBits |= (buttonBit) << bitCount;
		//buttonDebugString[bitCount] = buttonBit ? '1' : '0';
	}
	/*
	buttonDebugString[16] = '\r';
	buttonDebugString[17] = '\n';
	buttonDebugString[18] = 0;
	serialSend(buttonDebugString);
	*/

	return buttonBits;
}

void enablePower() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
}

void disablePower() {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
}

/*
void byteToBits(uint8_t bits[8], uint8_t byte) {
	for(int i = 0; i < 8; i++) {
		bits[i] = ((0x01 << (8 - i - 1)) & byte);
	}
}
void byteToBits(uint8_t bits[8], uint8_t byte) {
	for(int i = 0; i < 8; i++) {
		bits[i] = ((0x01 << (i)) & byte);
	}
}
*/
void byteToBits(uint8_t bits[8], uint8_t byte);
void writeLCDDataByte(uint8_t dataByte, uint32_t delay);
void writeLCDData2Byte(uint16_t dataByte, uint32_t delay);
void writeLCDCommand(uint8_t command, uint32_t delay);
void writeLCDData(uint8_t dataBits[], uint32_t count, uint32_t delay);

void byteToBits(uint8_t bits[8], uint8_t byte) {
	bits[0] = byte & 0x80;
	bits[1] = byte & 0x40;
	bits[2] = byte & 0x20;
	bits[3] = byte & 0x10;
	bits[4] = byte & 0x08;
	bits[5] = byte & 0x04;
	bits[6] = byte & 0x02;
	bits[7] = byte & 0x01;
/*
	bits[7] = byte & 0x80;
	bits[6] = byte & 0x40;
	bits[5] = byte & 0x20;
	bits[4] = byte & 0x10;
	bits[3] = byte & 0x08;
	bits[2] = byte & 0x04;
	bits[1] = byte & 0x02;
	bits[0] = byte & 0x01;
*/
}

void writeLCDDataByte(uint8_t dataByte, uint32_t delay) {
	uint8_t dataBits[8];
	byteToBits(dataBits, dataByte);

	writeLCDData(dataBits, 8, delay);
}

/*
*/
void writeLCDData2Byte(uint16_t dataByte, uint32_t delay) {
	uint8_t dataBits[8];
	byteToBits(dataBits, ((uint8_t)((dataByte >> 8) & 0xff)));

	writeLCDData(dataBits, 8, delay);

	byteToBits(dataBits, ((uint8_t)(dataByte & 0xff)));

	writeLCDData(dataBits, 8, delay);
}

void writeLCDData3Byte(uint32_t dataByte, uint32_t delay) {
	uint8_t dataBits[8];
	byteToBits(dataBits, ((uint8_t)((dataByte >> 16) & 0xff)));

	writeLCDData(dataBits, 8, delay);

	byteToBits(dataBits, ((uint8_t)((dataByte >> 8) & 0xff)));

	writeLCDData(dataBits, 8, delay);

	byteToBits(dataBits, ((uint8_t)(dataByte & 0xff)));

	writeLCDData(dataBits, 8, delay);
}

/*
void writeLCDData2Byte(uint8_t dataByte, uint32_t delay) {
	uint8_t dataBits[8];
	byteToBits(dataBits, ((uint8_t)(dataByte & 0xff)));

	writeLCDData(dataBits, 8, delay);

	byteToBits(dataBits, ((uint8_t)((dataByte >> 8) & 0xff)));

	writeLCDData(dataBits, 8, delay);
}
*/

void writeLCDCommand(uint8_t command, uint32_t delay) {
	uint8_t commandBits[8];
	byteToBits(commandBits, command);

	//commandBits[0] = 

	// Write command
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CLK

	for(int i = 0; i < 8; i++) {
		for(int j = 0; j < delay/2; j++) { asm("NOP");}
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, commandBits[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		for(int j = 0; j < delay/2; j++) { asm("NOP");}
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);


		for(int j = 0; j < delay; j++) { asm("NOP");}

  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
	}

  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_SET);   // DC
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CLK
}

void writeLCDData(uint8_t dataBits[], uint32_t count, uint32_t delay) {
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_SET);   // DC
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CLK

	for(int i = 0; i < count; i++) {
		for(int j = 0; j < delay/2; j++) { asm("NOP");}
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, dataBits[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		for(int j = 0; j < delay/2; j++) { asm("NOP");}
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);

		for(int j = 0; j < delay; j++) { asm("NOP");}

  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
	}

	// reset all
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CLK
}


void readLCDData(uint8_t dataBits[], uint32_t count, uint32_t delay) {
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_SET);   // DC
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CLK

  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	for(int i = 0; i < count; i++) {
		for(int j = 0; j < delay/2; j++) { asm("NOP");}
  //		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, dataBits[i] ? GPIO_PIN_SET : GPIO_PIN_RESET);
		for(int j = 0; j < delay/2; j++) { asm("NOP");}
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_SET);

		for(int j = 0; j < delay; j++) { asm("NOP");}

  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,GPIO_PIN_RESET);
	}

	// reset all
  	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15,GPIO_PIN_RESET); // D1
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET); // CLK
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
// ***************************************************
//  SetSysClock();

  // Configure NVIC preempt/priority groups
  //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  // cache RCC->CSR value to use it in isMPUSoftReset() and others
//  cachedRccCsrValue = RCC->CSR;

  /* Accounts for OP Bootloader, set the Vector Table base address as specified in .ld file */
  //extern void *isr_vector_table_base;
  //NVIC_SetVectorTable((uint32_t)&isr_vector_table_base, 0x0);
  //RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, DISABLE);

  //RCC_ClearFlag();

//  enableGPIOPowerUsageAndNoiseReductions();

  // Init cycle counter
//  cycleCounterInit();

  // SysTick
  //SysTick_Config(SystemCoreClock / 1000);
// ***************************************************


  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */



/*
    //__SPI1_CLK_ENABLE();
    __SPI2_CLK_ENABLE();
    spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
    spi.Init.Direction = SPI_DIRECTION_2LINES;
    spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
    spi.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    spi.Init.NSS = SPI_NSS_SOFT;
    spi.Init.TIMode = SPI_TIMODE_DISABLED;
    spi.Init.Mode = SPI_MODE_MASTER;
*/
//	HAL_Delay(1000);
//	HAL_Delay(1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
#define ST7735_TFTWIDTH_128   128 // for 1.44 and mini
#define ST7735_TFTWIDTH_80     80 // for mini
#define ST7735_TFTHEIGHT_128  128 // for 1.44" display
#define ST7735_TFTHEIGHT_160  160 // for 1.8" and mini display

#define ST_CMD_DELAY      0x80    // special signifier for command lists

#define ST77XX_NOP        0x00
#define ST77XX_SWRESET    0x01
#define ST77XX_RDDID      0x04
#define ST77XX_RDDST      0x09

#define ST77XX_SLPIN      0x10
#define ST77XX_SLPOUT     0x11
#define ST77XX_PTLON      0x12
#define ST77XX_NORON      0x13

#define ST77XX_INVOFF     0x20
#define ST77XX_INVON      0x21
#define ST77XX_DISPOFF    0x28
#define ST77XX_DISPON     0x29
#define ST77XX_CASET      0x2A
#define ST77XX_RASET      0x2B
#define ST77XX_RAMWR      0x2C
#define ST77XX_RAMRD      0x2E

#define ST77XX_PTLAR      0x30
#define ST77XX_COLMOD     0x3A
#define ST77XX_MADCTL     0x36

#define ST77XX_MADCTL_MY  0x80
#define ST77XX_MADCTL_MX  0x40
#define ST77XX_MADCTL_MV  0x20
#define ST77XX_MADCTL_ML  0x10
#define ST77XX_MADCTL_RGB 0x00

#define ST77XX_RDID1      0xDA
#define ST77XX_RDID2      0xDB
#define ST77XX_RDID3      0xDC
#define ST77XX_RDID4      0xDD

// Some ready-made 16-bit ('565') color settings:
#define	ST77XX_BLACK      0x0000
#define ST77XX_WHITE      0xFFFF
#define	ST77XX_RED        0xF800
#define	ST77XX_GREEN      0x07E0
#define	ST77XX_BLUE       0x001F
#define ST77XX_CYAN       0x07FF
#define ST77XX_MAGENTA    0xF81F
#define ST77XX_YELLOW     0xFFE0
#define	ST77XX_ORANGE     0xFC00
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);
  	//HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_10, GPIO_PIN_RESET);
  	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);

  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);

	for(int i = 0; i < 2; i++) {
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_SET);
  		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(500);
	}

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	//for(int i = 0; i < 10000; i++) {
	//}

	// CLK  / = D11 PA7
	// DC    = D10 PB6
	// SDA00 = PA3
	// SDA01 = PA2
	// SDA02 = PA10
	// SDA03 = PB3
	// SDA04 = PB5
	// SDA05 = PB4
	// SDA06 = PB10
	// SDA07 = PA8
	// SDA08 = PA9
	// SDA09 = PC7

	//HAL_Delay(1);


/*
*/

	uint32_t delay = 1;
	/*
	while(1) {
		writeLCDCommand(ST77XX_NOP, delay);
		writeLCDCommand(ST77XX_SWRESET, delay);
	}
	*/

			
/*
*/
	HAL_Delay(5);
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(5);

  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
/*
	for(int i = 0; i < 1000; i++) {
		asm("NOP");
	}
*/
	HAL_Delay(5);
  	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
	//HAL_Delay(1000);
	HAL_Delay(150);

	writeLCDCommand(ST77XX_SWRESET, delay);
	HAL_Delay(150);
//		writeLCDCommand(ST77XX_NOP, delay);

/*
	HAL_Delay(1000);
*/

	HAL_Delay(500);
	writeLCDCommand(ST77XX_SLPIN, delay);
	HAL_Delay(500);
	writeLCDCommand(ST77XX_SLPOUT, delay);
	HAL_Delay(500);
/*
		writeLCDCommand(ST77XX_NOP, delay);
*/
	while(1) {
		// DELAY
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(100);
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);
	}

	writeLCDCommand(ST77XX_NORON, delay);

	//HAL_Delay(1);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
	HAL_Delay(10);
//		writeLCDCommand(ST77XX_NOP, delay);

	HAL_Delay(1000);

	writeLCDCommand(ST77XX_COLMOD, delay);
	writeLCDDataByte(0x55, delay);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
	HAL_Delay(10);

	writeLCDCommand(ST77XX_MADCTL, delay);
	//writeLCDDataByte(0x08);
	writeLCDDataByte(0x00, delay);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
	HAL_Delay(10);

	writeLCDCommand(ST77XX_CASET, delay);
	writeLCDDataByte(0x00, delay);
	writeLCDDataByte(0x00, delay);
	writeLCDDataByte(0x00, delay);
	writeLCDDataByte(0xF0, delay);
	HAL_Delay(10);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC

	writeLCDCommand(ST77XX_RASET, delay);
	writeLCDDataByte(0x00, delay);
	writeLCDDataByte(0x00, delay);
	writeLCDDataByte(0x00, delay);
	writeLCDDataByte(0xF0, delay);
	HAL_Delay(10);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC

	writeLCDCommand(ST77XX_INVON, delay);
	//HAL_Delay(1);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
	HAL_Delay(10);

	writeLCDCommand(ST77XX_NORON, delay);
	//HAL_Delay(1);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
	HAL_Delay(10);

	writeLCDCommand(ST77XX_DISPON, delay);
	//HAL_Delay(1);
  	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC
	HAL_Delay(500);

	//continue;

  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(100);
			
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);

/*
		while(1) {
		writeLCDCommand(ST77XX_RDDID, delay);
			uint8_t readBuffer[8];
			readLCDData(readBuffer, 8, delay);
			readLCDData(readBuffer, 8, delay);
			readLCDData(readBuffer, 8, delay);
			readLCDData(readBuffer, 8, delay);
		}
*/
		writeLCDCommand(ST77XX_RAMWR, delay);
		for(int i = 0; i < 240*(240/3); i++) {
			writeLCDData3Byte(ST77XX_RED, delay);
			writeLCDData3Byte(ST77XX_GREEN, delay);
			writeLCDData3Byte(ST77XX_BLUE, delay);
			//writeLCDData2Byte(0x0000, delay);
			//writeLCDData2Byte(0xffff, delay);

/*
			uint8_t readBuffer[8];

			readLCDData(readBuffer, 8, delay);
*/
		}
  		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13,GPIO_PIN_RESET); // DC

		// DELAY
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(100);
			
  		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(100);

	//while(1) {}
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 320;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PC0 PC1 PC2 PC3 
                           PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4 
                           PA5 PA6 PA7 PA8 
                           PA10 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4 
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8 
                          |GPIO_PIN_10|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 PC6 PC7 
                           PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB3 PB4 PB5 
                           PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5 
                          |GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA13 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/*
void user_pwm_setvalue(uint16_t value)
{
    TIM_OC_InitTypeDef sConfigOC;
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = value;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);  
}
*/
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
