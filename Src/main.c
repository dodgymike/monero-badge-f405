
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"

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

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static GPIO_InitTypeDef  GPIO_InitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

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
	CDC_Transmit_FS(buffer, strlen(buffer));
        //HAL_UART_Receive(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart6, buffer, 2, HAL_MAX_DELAY);
//        HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
}

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

/*
157     spiTransferByte(bus->busdev_u.spi.instance, reg | 0x80); // read transaction
158     spiTransfer(bus->busdev_u.spi.instance, NULL, data, length);
*/

uint8_t spiBusWriteRegister(SPI_HandleTypeDef* spi, uint8_t mpuRegister, uint8_t value) {
	uint8_t writeBuffer[2] = { 0b00000000 | mpuRegister, value };

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, writeBuffer, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

uint8_t initMPU6000() {
	serialSend("Initialising MPU6000");
	spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
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

void readAccAndGyro(int16_t* accData)
{
//    int16_t data[6];

/*
    uint16_t x_h = spiBusReadRegister(&hspi1, MPU_RA_ACCEL_XOUT_H);
    uint16_t x_l = spiBusReadRegister(&hspi1, MPU_RA_ACCEL_XOUT_L);
    uint16_t y_h = spiBusReadRegister(&hspi1, MPU_RA_ACCEL_YOUT_H);
    uint16_t y_l = spiBusReadRegister(&hspi1, MPU_RA_ACCEL_YOUT_L);
    uint16_t z_h = spiBusReadRegister(&hspi1, MPU_RA_ACCEL_ZOUT_H);
    uint16_t z_l = spiBusReadRegister(&hspi1, MPU_RA_ACCEL_ZOUT_L);

    accData[0] = (int16_t)((x_h << 8) | x_l);
    accData[1] = (int16_t)((y_h << 8) | y_l);
    accData[2] = (int16_t)((z_h << 8) | z_l);
*/

    uint8_t mpuData[12];
    bzero(mpuData, 12);

    spiBusReadRegisterBuffer(&hspi1, MPU_RA_ACCEL_XOUT_H, mpuData, 12);

/*
	uint16_t mpuDebugDataIndex = 0;
	char accDebugString[120];
	sprintf(accDebugString, "% .6d,% .6d:% .6d,% .6d:% .6d,% .6d:% .6d,% .6d:% .6d,% .6d:% .6d,% .6d\r\n", ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]), ((uint8_t)mpuData[mpuDebugDataIndex++]));
	serialSend(accDebugString);
*/

/*
    if(!mpuData[0]) {
	return;
    }
*/

    uint8_t mpuDataIndex = 0;
    accData[0] = (((int8_t)mpuData[0]) << 8) + ((uint8_t)mpuData[1]);
    accData[1] = (((int8_t)mpuData[2]) << 8) + ((uint8_t)mpuData[3]);
    accData[2] = (((int8_t)mpuData[4]) << 8) + ((uint8_t)mpuData[5]);

    accData[3] = (((int8_t)mpuData[6]) << 8) + ((uint8_t)mpuData[7]);
    accData[4] = (((int8_t)mpuData[8]) << 8) + ((uint8_t)mpuData[9]);
    accData[5] = (((int8_t)mpuData[10]) << 8) + ((uint8_t)mpuData[11]);

/*
    accData[0] = (int16_t)((mpuData[mpuDataIndex++] << 8) | mpuData[mpuDataIndex++]);
    accData[1] = (int16_t)((mpuData[mpuDataIndex++] << 8) | mpuData[mpuDataIndex++]);
    accData[2] = (int16_t)((mpuData[mpuDataIndex++] << 8) | mpuData[mpuDataIndex++]);

    accData[3] = (int16_t)((mpuData[mpuDataIndex++] << 8) | mpuData[mpuDataIndex++]);
    accData[4] = (int16_t)((mpuData[mpuDataIndex++] << 8) | mpuData[mpuDataIndex++]);
    accData[5] = (int16_t)((mpuData[mpuDataIndex++] << 8) | mpuData[mpuDataIndex++]);
*/

/*
    accData[0] = (int16_t)((data[0] << 8) | data[1]);
    accData[1] = (int16_t)((data[2] << 8) | data[3]);
    accData[2] = (int16_t)((data[4] << 8) | data[5]);
*/

/*
    const bool ack = busReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6); 
    if (!ack) {
        return false;
    }   
*/

/*
    if(accData[0] & 0xf000) {
	accData[0] *= -1;
    }
    if(accData[1] & 0xf000) {
	accData[1] *= -1;
    }
    if(accData[2] & 0xf000) {
	accData[2] *= -1;
    }
*/
}

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
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L1, lastButtonPressTick)) {
		if(rainBrightness > 0) {
			rainBrightness--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L3, lastButtonPressTick)) {
		if(rainBrightness < 0b00011111) {
			rainBrightness++;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4, lastButtonPressTick)) {
		if(rainRed > 0) {
			rainRed--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2, lastButtonPressTick)) {
		if(rainRed < 254) {
			rainRed++;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4, lastButtonPressTick)) {
		if(rainGreen > 0) {
			rainGreen--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2, lastButtonPressTick)) {
		if(rainGreen < 254) {
			rainGreen++;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1, lastButtonPressTick)) {
		if(rainBlue > 0) {
			rainBlue--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3, &lastButtonPressTick)) {
		if(rainBlue < 254) {
			rainBlue++;
		}
	}

	int makeNewParticle = rand();
	if(makeNewParticle > RAND_MAX / 4) {
		for(int i = 0; i < maxParticles; i++) {
			if(particles[i] == NULL) {
				particles[i] = malloc(sizeof(struct Particle));
				particles[i]->x = rand() % 24;
				if(xAcc >= 0) {
					particles[i]->y = 0;
				} else {
					particles[i]->y = 24;
				}

				break;
			}
		}
	}

	//uint32_t rainColour = rgbToPixel(rainBrightness, rainRed, rainGreen, rainBlue);

	for(int i = 0; i < maxParticles; i++) {
		if(particles[i] != NULL) {
			setPixel(particles[i]->x, particles[i]->y, rainBrightness, rainRed, rainGreen, rainBlue);

			if(xAcc >= 0) {
				particles[i]->y++;
			} else {
				particles[i]->y--;
			}

			if((particles[i]->y >= 24) || (particles[i]->y <= 0)) {
				free(particles[i]);
				particles[i] = NULL;
			}
		}
	}

	// DEBUG
	for(int bitIndex = 0; bitIndex < 8; bitIndex++) {
		uint32_t rainBrightnessLedLevel = ((rainBrightness >> bitIndex) & 1) ? 0b00001000 : 0;

		setPixel(bitIndex, 0, rainBrightness, ((rainBlue >> bitIndex) & 1) ? 0b00001000 : 0, 0, 0);
		setPixel(bitIndex, 1, rainBrightness, 0, ((rainGreen >> bitIndex) & 1) ? 0b00001000 : 0, 0);
		setPixel(bitIndex, 2, rainBrightness, 0, 0, ((rainRed >> bitIndex) & 1) ? 0b00001000 : 0);
		setPixel(bitIndex, 3, rainBrightness, rainBrightnessLedLevel, rainBrightnessLedLevel, rainBrightnessLedLevel);
	}
	// /DEBUG
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

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
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

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_SPI2_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_USB_DEVICE_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	uint32_t lowBatteryFlashCounter = 0;
	uint8_t ledPanelEnabled = 0;

	disablePower();
	enablePower();

/*
	for(int i = 0; i < 5; i++) {
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	        HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(500);
	}
*/

	HAL_ADC_Start(&hadc1);

	uint8_t IRHello[] = "bobmonkey";
	irTX(IRHello, strlen(IRHello));


	uint32_t batteryAdcAverage = 0;
	uint32_t batteryAdcValuesSize = 3000;
	uint32_t batteryAdcValues[batteryAdcValuesSize];
	uint32_t batteryAdcValuesIndex = 0;
	uint64_t batteryAdcTotalSampleCount = 0;
        uint32_t batteryVoltage100 = 0;
	uint8_t lowBattery = 0;

  int bufferSize = 40;
  initAccRingBuffer(bufferSize);

  int16_t mean_x = 1;
  int16_t mean_y = 0;
  int16_t mean_z = 0;

  int16_t std_x = 0;
  int16_t std_y = 0;
  int16_t std_z = 0;

	int16_t accData[6];

	uint32_t buttonAccumulators[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	uint32_t buttonState[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	uint32_t lastButtonPressTick = HAL_GetTick();

	uint16_t gameMode = MODE_WELCOME;

	uint32_t last_tick = HAL_GetTick();
	uint32_t buttonTicks = 0;

	for(uint32_t i = 0; i < batteryAdcValuesSize; i++) {
		batteryAdcValues[i] = 0;
	}

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	enableLedPanel(&ledPanelEnabled);

	struct BlockchainGame blockchainGame;
	initBlockchainGame(&blockchainGame);

	if(1) {
		HAL_Delay(2000);
		serialSend("Initialising MPU6000: ");

		uint8_t productID = 0;
		while((productID = initMPU6000()) == 0) {}
		serialSend("DONE\r\n");
	}

	//HAL_TIM_Base_Start(&htim3);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	//serialSend("Loop\r\n");



	/*
        calculateMeans(bufferSize, &mean_x, &mean_y, &mean_z);
        calculateStdDevs(bufferSize, &mean_x, &mean_y, &mean_z, &std_x, &std_y, &std_z);
	*/

/*
	if(accData[0] > (mean_x - std_x)) {
		accRingBuffer->x = mean_x - std_x;
	} else if(accData[0] < (mean_x + std_x)) {
		accRingBuffer->x = mean_x + std_x;
	} else {
		accRingBuffer->x = accData[0];
	}

	if(accData[1] > (mean_y - std_y)) {
		accRingBuffer->y = mean_y - std_y;
	} else if(accData[1] < (mean_y + std_y)) {
		accRingBuffer->y = mean_y + std_y;
	} else {
		accRingBuffer->y = accData[1];
	}

	if(accData[2] > (mean_z - std_z)) {
		accRingBuffer->z = mean_z - std_z;
	} else if(accData[2] < (mean_z + std_z)) {
		accRingBuffer->z = mean_z + std_z;
	} else {
		accRingBuffer->z = accData[2];
	}
*/

/*
	if(std_x == 0) {
		std_x = 3000;
	}
	if(std_y == 0) {
		std_y = 3000;
	}
	if(std_z == 0) {
		std_z = 3000;
	}

	if((accData[0] < (mean_x + std_x)) && ((accData[0] > (mean_x - std_x)))) {
		accRingBuffer->x = accData[0];
	} else {
		accRingBuffer->x = accData[0] * 0.1;
	}

	if((accData[1] < (mean_y + std_y)) && ((accData[1] > (mean_y - std_y)))) {
		accRingBuffer->y = accData[1];
	} else {
		accRingBuffer->y = accData[1] * 0.1;
	}

	if((accData[2] < (mean_z + std_z)) && ((accData[2] > (mean_z - std_z)))) {
		accRingBuffer->z = accData[2];
	} else {
		accRingBuffer->z = accData[2] * 0.1;
	}

	moveToNextAccRingBufferEntry();

        //removeOutliers(bufferSize, &mean_x, &mean_y, &mean_z, &std_x, &std_y, &std_z);
*/

	//uint8_t accDebugString[60];
	//sprintf(accDebugString, "x (%0.6d) y (%0.6d) z (%0.6d)\r\n", accData[0], accData[1], accData[2]);
	//sprintf(accDebugString, "tick (%lu) x (%0.6d) y (%0.6d) z (%0.6d)\r\n", HAL_GetTick(), mean_x, mean_y, mean_z);
	//serialSend(accDebugString);

	if(HAL_GetTick() - lastButtonPressTick > 1800000) {
		disableLedPanel(&ledPanelEnabled);
        	disablePower();
	} else {
		enableLedPanel(&ledPanelEnabled);
	}
/*
*/
	if(HAL_GetTick() - last_tick > 50) {
		last_tick = HAL_GetTick();
		ClearPixels();

		uint16_t mpuInt = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);
		uint8_t mpuIntFlags = spiBusReadRegister(&hspi1, MPU_RA_INT_STATUS);
		if(!mpuInt) {
			serialSend("No MPU Int\r\n");
		} else if(!mpuIntFlags) {
			serialSend("No int status\r\n");
		} else {
			readAccAndGyro(accData);
/*
			char accDebugString[120];
			sprintf(accDebugString, "acc x (% .6d) y (% .6d) z (% .6d) gyro x (% .6d) y (% .6d) z (% .6d)\r\n", accData[0], accData[1], accData[2], accData[3], accData[4], accData[5]);
			serialSend(accDebugString);
*/
		}

		uint32_t batteryAdcAccumulator = 0;
		for(uint32_t adcValueIndex = 0; adcValueIndex < batteryAdcValuesSize; adcValueIndex++) {
			if(batteryAdcValues[adcValueIndex] > 4096) {
				continue;
			}
			batteryAdcAccumulator += batteryAdcValues[adcValueIndex];
		}
		batteryAdcAverage = batteryAdcAccumulator / batteryAdcValuesSize;
		if(batteryAdcAverage > 4096) {
			batteryAdcAverage = 4096;
		} 
		//batteryVoltage100 = 2.2424 * batteryAdcAverage;
		batteryVoltage100 = 0.2419354839 * batteryAdcAverage;

		if(batteryAdcTotalSampleCount > (batteryAdcValuesSize * 20)) { // wait for ADC low-pass to have enough samples during start-up
			if(batteryVoltage100 <= 620) {
				// SHUTDOWN TIME!
				// NO FURTHER PROCESSING
				disableLedPanel(&ledPanelEnabled);
				disablePower();
				while(1) {}
/*
*/
				// NO FURTHER PROCESSING
				// NO FURTHER PROCESSING
			} else if(batteryVoltage100 < 630) {
				// LOW POWER TIME!
				lowBattery = 1;
			}
/*
			 else if(batteryVoltage > 7.5) {
				lowBattery = 0;
				enablePower();
				enableLedPanel(&ledPanelEnabled);
			}
*/
		} 


		uint8_t selectPressed = buttonPressed(buttonState, buttonAccumulators, BUTTON_SELECT, &lastButtonPressTick);
		uint8_t startPressed = buttonPressed(buttonState, buttonAccumulators, BUTTON_START, &lastButtonPressTick);

		if(slaveModeEnabled()) {
			uint8_t slaveModeBit0 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
			uint8_t slaveModeBit1 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
			uint8_t slaveModeBit2 = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);

			uint8_t slaveMode = (slaveModeBit2 << 2) + (slaveModeBit1 << 1) + slaveModeBit0;
			switch(slaveMode) {
				case SLAVE_MODE_WELCOME:
					gameMode = MODE_WELCOME;
					break;
				case SLAVE_MODE_EYE:
					gameMode = MODE_EYE;
					break;
				case SLAVE_MODE_PLASMA:
					gameMode = MODE_PLASMA;
					break;
				case SLAVE_MODE_BLIND:
					gameMode = MODE_BLIND;
					break;
				case SLAVE_MODE_RANDOM:
					gameMode = MODE_RANDOM;
					break;
				case SLAVE_MODE_RAIN:
					gameMode = MODE_RAIN;
					break;
				case SLAVE_MODE_SNAKE:
					gameMode = MODE_SNAKE;
					break;
				case SLAVE_MODE_BLOCKCHAIN:
					gameMode = MODE_BLOCKCHAIN;
					break;
			}
		}

		if(selectPressed && startPressed) {
        		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        		disablePower();
		} else if(!slaveModeEnabled() && selectPressed) {
			switch(gameMode) {
				case MODE_WELCOME:
					gameMode = MODE_RAIN;
					setSlaveMode(SLAVE_MODE_RAIN);
					break;
				case MODE_RAIN:
					gameMode = MODE_SNAKE;
					setSlaveMode(SLAVE_MODE_SNAKE);
					break;
				case MODE_SNAKE:
					gameMode = MODE_BLOCKCHAIN;
					setSlaveMode(SLAVE_MODE_BLOCKCHAIN);
					break;
				case MODE_BLOCKCHAIN:
					gameMode = MODE_EYE;
					setSlaveMode(SLAVE_MODE_EYE);
					break;
				case MODE_EYE:
					gameMode = MODE_PLASMA;
					setSlaveMode(SLAVE_MODE_PLASMA);
					break;
				case MODE_PLASMA:
					gameMode = MODE_BLIND;
					setSlaveMode(SLAVE_MODE_BLIND);
					break;
				case MODE_BLIND:
					gameMode = MODE_RANDOM;
					setSlaveMode(SLAVE_MODE_RANDOM);
					break;
				case MODE_RANDOM:
					gameMode = MODE_DEBUG;
					break;
				case MODE_DEBUG:
					gameMode = MODE_WELCOME;
					setSlaveMode(SLAVE_MODE_WELCOME);
					break;
			}
		}

		if(lowBattery) {
			lowBatteryScreen(0b111, &lowBatteryFlashCounter);
		} else if(gameMode == MODE_WELCOME) {
			//welcome(0b11111);
			welcome(buttonState, buttonAccumulators, 0b111, &lastButtonPressTick, startPressed);
		} else if(gameMode == MODE_RAIN) {
			rain(mean_x, mean_y, buttonState, buttonAccumulators, &lastButtonPressTick, startPressed);
		} else if(gameMode == MODE_BLIND) {
			blind(0b111, 0b11111, startPressed);
		} else if(gameMode == MODE_RANDOM) {
			random_pixels(0b111, startPressed);
		} else if(gameMode == MODE_BLOCKCHAIN) {
			blockchain(&blockchainGame, buttonState, buttonAccumulators, 0b111, &lastButtonPressTick, startPressed);
		} else if(gameMode == MODE_SNAKE) {
			snake(buttonState, buttonAccumulators, 0b111, &lastButtonPressTick, startPressed);
		} else if(gameMode == MODE_PLASMA) {
			plasma(0b1, accData);
		} else if(gameMode == MODE_EYE) {
			eye(0b1111, startPressed, accData);
		} else if(gameMode == MODE_DEBUG) {
			debug(buttonState, buttonAccumulators, 0b111, batteryAdcAverage, batteryVoltage100, startPressed);
		}

		WriteLedPanelFrame(ledPanelEnabled);
	}

	if (HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK) {
		batteryAdcValues[batteryAdcValuesIndex++] = HAL_ADC_GetValue(&hadc1);
		if(batteryAdcValuesIndex >= batteryAdcValuesSize) {
			batteryAdcValuesIndex = 0;
		}

		batteryAdcTotalSampleCount++;

		HAL_ADC_Start(&hadc1);
	}

	uint16_t buttonBits = readButtons();
	uint16_t populatedButtonBits = buttonBits & 0b0111111111100000;

	for(int buttonIndex = 0; buttonIndex < 16; buttonIndex++) {
		uint16_t buttonMask = (1 << buttonIndex);
		uint16_t buttonState = populatedButtonBits & buttonMask;
		if(buttonState) {
			if((buttonIndex == BUTTON_SELECT) && (gameMode == MODE_PLASMA)) {
				gameMode = MODE_BLIND;
			}
			buttonAccumulators[buttonIndex]++;
		} else {
			buttonAccumulators[buttonIndex]--;
		}

		if(buttonAccumulators[buttonIndex] <= 1) {
			buttonAccumulators[buttonIndex] = 1;
		} else if(buttonAccumulators[buttonIndex] > 512) {
			buttonAccumulators[buttonIndex] = 512;
		}
	}

/*
	HAL_Delay(2000);
*/

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 13;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 6000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1501;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 0;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BATT_BUCK_ENABLE_Pin|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, MODE0_OUT_Pin|MODE1_OUT_Pin|MODE2_OUT_Pin|MODE3_OUT_Pin 
                          |GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_POWER_ENABLE_GPIO_Port, LED_POWER_ENABLE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : MODE0_IN_Pin MODE1_IN_Pin MODE2_IN_Pin SLAVE_MODE_Pin */
  GPIO_InitStruct.Pin = MODE0_IN_Pin|MODE1_IN_Pin|MODE2_IN_Pin|SLAVE_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BATT_BUCK_ENABLE_Pin */
  GPIO_InitStruct.Pin = BATT_BUCK_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BATT_BUCK_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE0_OUT_Pin MODE1_OUT_Pin MODE2_OUT_Pin MODE3_OUT_Pin */
  GPIO_InitStruct.Pin = MODE0_OUT_Pin|MODE1_OUT_Pin|MODE2_OUT_Pin|MODE3_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_POWER_ENABLE_Pin */
  GPIO_InitStruct.Pin = LED_POWER_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_POWER_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC11 */
  GPIO_InitStruct.Pin = GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
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
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
