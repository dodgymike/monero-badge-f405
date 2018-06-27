
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "mpu6000.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

static GPIO_InitTypeDef  GPIO_InitStruct;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI1_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
static uint8_t CHAR_WIDTH = 4;
static uint8_t CHAR_HEIGHT = 5;
static uint8_t characters[37 * 5] = {
	/* A */
	0b0110,
	0b1001,
	0b1111,
	0b1001,
	0b1001,
	/* B */
	0b1111,
	0b1001,
	0b1110,
	0b1001,
	0b1111,
	/* C */
	0b1111,
	0b1000,
	0b1000,
	0b1000,
	0b1111,
	/* D */
	0b1110,
	0b1001,
	0b1001,
	0b1001,
	0b1110,
	/* E */
	0b1111,
	0b1000,
	0b1111,
	0b1000,
	0b1111,
	/* F */
	0b1111,
	0b1000,
	0b1111,
	0b1000,
	0b1000,
	/* G */
	0b1111,
	0b1000,
	0b1011,
	0b1001,
	0b1111,
	/* H */
	0b1001,
	0b1001,
	0b1111,
	0b1001,
	0b1001,
	/* I */
	0b1111,
	0b0110,
	0b0110,
	0b0110,
	0b1111,
	/* J */
	0b1111,
	0b0110,
	0b0110,
	0b0110,
	0b1100,
	/* K */
	0b1001,
	0b1010,
	0b1100,
	0b1010,
	0b1001,
	/* L */
	0b1000,
	0b1000,
	0b1000,
	0b1000,
	0b1111,
	/* M */
	0b1001,
	0b1111,
	0b1111,
	0b1001,
	0b1001,
	/* N */
	0b1001,
	0b1101,
	0b1011,
	0b1001,
	0b1001,
	/* O */
	0b1111,
	0b1001,
	0b1001,
	0b1001,
	0b1111,
	/* P */
	0b1111,
	0b1001,
	0b1111,
	0b1000,
	0b1000,
	/* Q */
	0b1111,
	0b1001,
	0b1001,
	0b1111,
	0b0001,
	/* R */
	0b1111,
	0b1001,
	0b1111,
	0b1011,
	0b1001,
	/* S */
	0b1111,
	0b1000,
	0b1111,
	0b0001,
	0b1111,
	/* T */
	0b1111,
	0b0110,
	0b0110,
	0b0110,
	0b0110,
	/* U */
	0b1001,
	0b1001,
	0b1001,
	0b1001,
	0b1111,
	/* V */
	0b1001,
	0b1001,
	0b1001,
	0b0110,
	0b0000,
	/* W */
	0b1001,
	0b1001,
	0b1111,
	0b1111,
	0b0110,
	/* X */
	0b1001,
	0b1001,
	0b0110,
	0b1001,
	0b1001,
	/* Y */
	0b1001,
	0b1001,
	0b0110,
	0b0110,
	0b0110,
	/* Z */
	0b1111,
	0b0011,
	0b0110,
	0b1100,
	0b1111,
	/* 0 */
	0b1111,
	0b1001,
	0b1001,
	0b1001,
	0b1111,
	/* 1 */
	0b0111,
	0b1101,
	0b0001,
	0b0001,
	0b0001,
	/* 2 */
	0b0111,
	0b1001,
	0b0010,
	0b0100,
	0b1111,
	/* 3 */
	0b1110,
	0b0001,
	0b0110,
	0b0001,
	0b1110,
	/* 4 */
	0b1001,
	0b1001,
	0b1111,
	0b0001,
	0b0001,
	/* 5 */
	0b0111,
	0b1000,
	0b1110,
	0b0001,
	0b1110,
	/* 6 */
	0b0111,
	0b1000,
	0b1111,
	0b1001,
	0b1111,
	/* 7 */
	0b1111,
	0b1111,
	0b0010,
	0b0100,
	0b0100,
	/* 8 */
	0b0110,
	0b1001,
	0b1111,
	0b1001,
	0b0110,
	/* 9 */
	0b0111,
	0b1001,
	0b0111,
	0b0001,
	0b0001,
	/* SPACE */
	0b0000,
	0b0000,
	0b0000,
	0b0000,
	0b0000,
};

static uint16_t missingLeds[] = { 460, 461 };
static uint8_t missingLedCount = 2;

static uint32_t pixels[576];

uint16_t BUTTON_SELECT = 5;
uint16_t BUTTON_START  = 6;
uint16_t BUTTON_R4     = 7;
uint16_t BUTTON_R3     = 8;
uint16_t BUTTON_R2     = 9;
uint16_t BUTTON_R1     = 10;
uint16_t BUTTON_L4     = 11;
uint16_t BUTTON_L3     = 12;
uint16_t BUTTON_L2     = 13;
uint16_t BUTTON_L1     = 14;

#define MODE_RAIN     0b0000000000000001
#define MODE_SNAKE    0b0000000000000010
#define MODE_BLIND    0b0000000000000100
#define MODE_RANDOM   0b0000000000001000
#define MODE_DEBUG    0b0000000000010000
#define MODE_SNAKE    0b0000000000100000


static uint8_t start_frame_data[] = {
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
	0x00, 0x00, 0x00, 0x00, /* start frame */
};
static uint8_t end_frame_data[] = {
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff, /* end frame */
	0xff, 0xff, 0xff, 0xff /* end frame */
};

uint8_t buttonPressed(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t button) {
	uint8_t buttonPressed = 0;

	if((buttonState[button] < 256) && (buttonAccumulators[button] > 256)) {
		buttonPressed = 1;
		buttonState[button] = 512;
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

void serialSend(int8_t* buffer) {
        //HAL_UART_Receive(&huart2, buffer, sizeof(buffer), HAL_MAX_DELAY);
        //HAL_UART_Transmit(&huart6, buffer, 2, HAL_MAX_DELAY);
        HAL_UART_Transmit(&huart3, buffer, strlen(buffer), HAL_MAX_DELAY);
}

uint8_t spiBusReadRegister(SPI_HandleTypeDef* spi, uint8_t mpuRegister) {
	uint8_t registerWithFlag = 0b10000000 | mpuRegister;
	uint8_t rxBuffer[20];

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(spi, &registerWithFlag, rxBuffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(spi, &registerWithFlag, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spi, &rxBuffer, 1, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return rxBuffer[0];
}

uint8_t spiBusReadRegisterBuffer(SPI_HandleTypeDef* spi, uint8_t mpuRegister, uint8_t* rxBuffer, uint8_t bufferSize) {
	uint8_t registerWithFlag = 0b10000000 | mpuRegister;

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	//HAL_SPI_TransmitReceive(spi, &registerWithFlag, rxBuffer, 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(spi, &registerWithFlag, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(spi, rxBuffer, bufferSize, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return rxBuffer[0];
}

uint8_t spiBusWriteRegister(SPI_HandleTypeDef* spi, uint8_t mpuRegister, uint8_t value) {
	uint8_t writeBuffer[2] = { 0b00000000 | mpuRegister, value };

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(spi, writeBuffer, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return 0;
}

void initMPU6000() {
	spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
        HAL_Delay(200);
	

	uint8_t debugBuffer[100];
	
	for(int i = 0; i < 5; i++) {
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
	uint8_t* productIDString;
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
    spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    HAL_Delay(15);

    // Disable Primary I2C Interface
    spiBusWriteRegister(&hspi1, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    HAL_Delay(15);

    spiBusWriteRegister(&hspi1, MPU_RA_PWR_MGMT_2, 0x00);
    HAL_Delay(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    //spiBusWriteRegister(&hspi1, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
    HAL_Delay(15);

    // Gyro +/- 1000 DPS Full Scale
    spiBusWriteRegister(&hspi1, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3);
    HAL_Delay(15);

    // Accel +/- 8 G Full Scale
    spiBusWriteRegister(&hspi1, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3);
    HAL_Delay(15);

    spiBusWriteRegister(&hspi1, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    HAL_Delay(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&hspi1, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    HAL_Delay(15);
#endif

}

void readAcc(int16_t *accData)
{
    int16_t data[6];

    spiBusReadRegisterBuffer(&hspi1, MPU_RA_ACCEL_XOUT_H, (uint8_t*)&data, 6); 
/*
    const bool ack = busReadRegisterBuffer(&acc->bus, MPU_RA_ACCEL_XOUT_H, data, 6); 
    if (!ack) {
        return false;
    }   
*/

    accData[0] = (int16_t)((data[0] << 8) | data[1]);
    accData[1] = (int16_t)((data[2] << 8) | data[3]);
    accData[2] = (int16_t)((data[4] << 8) | data[5]);

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

uint32_t rgbToPixel(uint32_t brightness, uint32_t red, uint32_t green, uint32_t blue) {
	return (((brightness << 3) | 0b00000111) << 24) + (red << 16) + (green << 8) + blue;
}

void ClearPixels()
{
	for(uint16_t ledIndex = 0; ledIndex < 576; ledIndex++) {
		pixels[ledIndex] = 0;
	}
}

void GenerateTestSPISignal()
{

	uint8_t led_frame_size = 4;
	uint8_t brightness = 0b11100001;
	//uint8_t brightness = 0b11100111;

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

        HAL_SPI_Transmit(&hspi2, start_frame_data, led_frame_size * 2, HAL_MAX_DELAY);

	uint8_t off_led_data[] = {
		//0x00000111, 0x00, 0x00, 0x00
		//0x11100000, 0x00, 0x00, 0x00
		brightness, 0x00, 0x00, 0x00
		//0x00, 0x01, 0x01, 0x01
		//0x00, 0x00, 0x00, 0x00
	};

	for(uint16_t led_count = 0; led_count < 576; led_count++) {
		if(pixels[led_count] > 0) {
			uint8_t brightness = (pixels[led_count] >> 24) & 0xff;
			uint8_t red = (pixels[led_count] >> 16) & 0xff;
			uint8_t green = (pixels[led_count] >> 8) & 0xff;
			uint8_t blue = pixels[led_count] & 0xff;

			uint8_t led_data[] = {
				//brightness, 0x05, 0x05, 0x05
				brightness, red, green, blue
			};
			HAL_SPI_Transmit(&hspi2, led_data, led_frame_size, HAL_MAX_DELAY);
		} else {
			HAL_SPI_Transmit(&hspi2, off_led_data, led_frame_size, HAL_MAX_DELAY);
		}
	}

        HAL_SPI_Transmit(&hspi2, end_frame_data, led_frame_size * 8, HAL_MAX_DELAY);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

        //HAL_Delay(10);

}

uint16_t xyToLedIndex(uint8_t x, uint8_t y) {
	uint16_t ledIndex = 0;
/*
	// for 8x8 panels in 3x3 configuration
	if(y < 8) {
		ledIndex = (x * 8) + y;
	} else if(y < 16) {
		ledIndex = 192 + (x * 8) + (y - 8);
	} else {
		ledIndex = 384 + (x * 8) + (y - 16);
	}
*/

	ledIndex = y * 24;
	if(y % 2 == 0) {
		ledIndex += x;
	} else {
		ledIndex += 23 - x;
	}

	if(ledIndex >= 576) {
		ledIndex = 0;
	}

	if(missingLedCount > 0) {
		uint16_t originalLedIndex = ledIndex;

		for(int i = 0; i < missingLedCount; i++) {
			if(originalLedIndex >= missingLeds[i]) {
				ledIndex--;
			}
		}
	}

	return ledIndex;
}

const uint8_t maxRadius = 12;

float rotation = 0;
uint8_t radius = 12;
void rotateLine() {
	uint16_t ledIndex = 0;
	uint8_t x = 0;
	uint8_t y = 0;

	for(int i = maxRadius; i >= radius; i--) {
		x = maxRadius + (sin(rotation) * i);
		y = maxRadius + (cos(rotation) * i);

		ledIndex = xyToLedIndex(x, y);
		pixels[ledIndex] = 1;
	}

	rotation += 0.1;
	if(rotation > 6.28) {
		rotation = 0;
		radius--;
		if(radius <= 1) {
			radius = maxRadius;
		}
	}
}

struct Particle {
	uint8_t x;
	uint8_t y;
};

void blind(uint32_t colour) {
	for(int i = 0; i < 576; i++) {
		pixels[i] = colour;
	}
}

void random_pixels(uint32_t brightness) {
	for(int i = 0; i < 576; i++) {
		pixels[i] = rgbToPixel(brightness, rand(), rand(), rand());
	}
}

static const uint8_t maxParticles = 32;
struct Particle* particles[32];
uint32_t rainBrightness = 1;
uint32_t rainRed = 5;
uint32_t rainGreen = 5;
uint32_t rainBlue = 5;

void rain(int xAcc, int yAcc, uint32_t buttonState[16], uint32_t buttonAccumulators[16]) {
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_START)) {
		rainBrightness++;
		if((rainBrightness < 0) || (rainBrightness >= 0b11100000)) {
			rainBrightness = 1;
		} 
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L4)) {
		if(rainRed > 0) {
			rainRed--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_L2)) {
		if(rainRed < 254) {
			rainRed++;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R4)) {
		if(rainGreen > 0) {
			rainGreen--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R2)) {
		if(rainGreen < 254) {
			rainGreen++;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R1)) {
		if(rainBlue > 0) {
			rainBlue--;
		}
	}
	if(buttonPressed(buttonState, buttonAccumulators, BUTTON_R3)) {
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

	uint32_t rainColour = rgbToPixel(rainBrightness, rainRed, rainGreen, rainBlue);

	for(int i = 0; i < maxParticles; i++) {
		if(particles[i] != NULL) {
			uint16_t ledIndex = xyToLedIndex(particles[i]->x, particles[i]->y);
			//pixels[ledIndex] = 1;
			pixels[ledIndex] = rainColour;

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
}

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
} snakeGame;

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
	snakePlayer->colour = rgbToPixel(20, rand() % 255, rand() % 255, rand() % 255);
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
		pixels[xyToLedIndex(snakeGame.players[snakeIndex].x, snakeGame.players[snakeIndex].y)] = snakeGame.players[snakeIndex].colour;
		struct SnakeTail* snakeTail = snakeGame.players[snakeIndex].tail;
		while(snakeTail != NULL) {
			pixels[xyToLedIndex(snakeTail->x, snakeTail->y)] = snakeGame.players[snakeIndex].colour;
			snakeTail = snakeTail->next;
		}
	}

	// DRAW SNAKE FOOD
	for(int snakeFoodIndex = 0; snakeFoodIndex < snakeGame.snakeCount; snakeFoodIndex++) {
		if(snakeGame.snakeFood[snakeFoodIndex] != NULL) {
			pixels[xyToLedIndex(snakeGame.snakeFood[snakeFoodIndex]->x, snakeGame.snakeFood[snakeFoodIndex]->y)] = rgbToPixel(20, 20, 0, 0);
		}
	}
}

void debug(uint32_t buttonState[16], uint32_t buttonAccumulators[16], uint32_t brightness) {
	for(int i = 0; i < (24*24); i++) {
		pixels[i] = 0;
	}

	for(int i = 0; i < 16; i++) {
		pixels[xyToLedIndex(i, 0)] = rgbToPixel(brightness, (buttonState[i] > 256) ? 20 : 0, (buttonState[i] <= 256) ? 20 : 0, 0);
		pixels[xyToLedIndex(i, 1)] = rgbToPixel(brightness, (buttonAccumulators[i] > 256) ? 20 : 0, (buttonAccumulators[i] <= 256) ? 20 : 0, 0);
		pixels[xyToLedIndex(i, 2)] = rgbToPixel(brightness, (buttonAccumulators[i] > 128) ? 20 : 0, (buttonAccumulators[i] <= 128) ? 20 : 0, 0);
		pixels[xyToLedIndex(i, 3)] = rgbToPixel(brightness, (buttonAccumulators[i] > 64) ? 20 : 0, (buttonAccumulators[i] <= 64) ? 20 : 0, 0);
		pixels[xyToLedIndex(i, 4)] = rgbToPixel(brightness, (buttonAccumulators[i] > 32) ? 20 : 0, (buttonAccumulators[i] <= 32) ? 20 : 0, 0);
		pixels[xyToLedIndex(i, 5)] = rgbToPixel(brightness, (buttonAccumulators[i] > 16) ? 20 : 0, (buttonAccumulators[i] <= 16) ? 20 : 0, 0);
		pixels[xyToLedIndex(i, 6)] = rgbToPixel(brightness, (buttonAccumulators[i] > 0) ? 20 : 0, (buttonAccumulators[i] <= 0) ? 20 : 0, 0);
	}

	drawText(brightness, 0, 9, "monkey", 6);
}

void drawText(uint8_t brightness, uint8_t x, uint8_t y, uint8_t text[], uint8_t length) {
	uint8_t xOffset = x;

	for(int textIndex = 0; textIndex < length; textIndex++) {
		uint8_t charToDraw = text[textIndex];
		uint8_t charIndex = 0;
		if(charToDraw == 32) {
			charIndex = 36;
		} else if(charToDraw >= 65 && charToDraw <= 90) {
			charIndex = charToDraw - 65;
		} else if(charToDraw >= 97 && charToDraw <= 122) {
			charIndex = charToDraw - 97;
		} else if(charToDraw >= 48 && charToDraw <= 57) {
			charIndex = charToDraw - 48 + 26;
		}

		for(uint8_t charX = 0; charX < CHAR_WIDTH; charX++) {
			for(uint8_t charY = 0; charY < CHAR_HEIGHT; charY++) {
				uint8_t charLine = characters[(charIndex * CHAR_HEIGHT) + charY];
				if((charLine >> (CHAR_WIDTH - charX - 1)) & 1) {
					pixels[xyToLedIndex(x + charX + xOffset, y + charY)] = rgbToPixel(brightness, 20, 20, 20);
				} else {
					pixels[xyToLedIndex(x + charX + xOffset, y + charY)] = rgbToPixel(brightness, 0, 0, 0);
				}
			}
		}
		pixels[xyToLedIndex(xOffset, 20)] = rgbToPixel(brightness, 20, 20, 20);
		xOffset += CHAR_WIDTH;
	}
}

uint16_t readButtons() {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
        //HAL_Delay(1);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
        //HAL_Delay(1);

	// load   2 - PB5
	// signal 1 - PB0
	// clock  3 - PA3

       	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);

	// latch the input buttons
	int delayDuration = 10;
	int delayCount = 0;
	//int8_t buttonDebugString[40];
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);

	uint16_t buttonBits = 0;
        for(int bitCount = 0; bitCount < 16; bitCount++) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
		uint16_t buttonBit = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);

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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);



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
	HAL_Delay(1000);
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_USART3_UART_Init();
  MX_SPI3_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */


	for(int i = 0; i < 5; i++) {
        	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	        HAL_Delay(500);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(500);
	}

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

  serialSend("Initialising MPU6000: ");

  initMPU6000();
  serialSend("DONE\r\n");

  int bufferSize = 40;
  initAccRingBuffer(bufferSize);

  int16_t mean_x = 1;
  int16_t mean_y = 0;
  int16_t mean_z = 0;

  int16_t std_x = 0;
  int16_t std_y = 0;
  int16_t std_z = 0;

  int16_t accData[3];

	uint32_t buttonAccumulators[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	uint32_t buttonState[16] = { 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	uint16_t gameMode = MODE_RAIN;

	uint32_t last_tick = HAL_GetTick();
	uint32_t buttonTicks = 0;

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
  //HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	//serialSend("Loop\r\n");

	//serialSend("Reading from acc: ");
	//readAcc(&accData);
	//serialSend("Done\r\n");


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


	if(HAL_GetTick() - last_tick > 50) {
		last_tick = HAL_GetTick();
		ClearPixels();

		uint8_t selectPressed = buttonPressed(buttonState, buttonAccumulators, BUTTON_SELECT);
		uint8_t startPressed = buttonPressed(buttonState, buttonAccumulators, BUTTON_START);

		if(selectPressed && startPressed) {
        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
		} else if(selectPressed) {
			switch(gameMode) {
				case MODE_RAIN:
					gameMode = MODE_BLIND;
					break;
				case MODE_BLIND:
					gameMode = MODE_RANDOM;
					break;
				case MODE_RANDOM:
					gameMode = MODE_SNAKE;
					break;
				case MODE_SNAKE:
					gameMode = MODE_DEBUG;
					break;
				case MODE_DEBUG:
					gameMode = MODE_RAIN;
					break;
			}
		}

		if(gameMode == MODE_RAIN) {
			rain(mean_x, mean_y, buttonState, buttonAccumulators);
		} else if(gameMode == MODE_BLIND) {
			blind(rgbToPixel(1, 0xff, 0xff, 0xff));
		} else if(gameMode == MODE_RANDOM) {
			random_pixels(1);
		} else if(gameMode == MODE_SNAKE) {
			snake(buttonState, buttonAccumulators, 1);
		} else if(gameMode == MODE_DEBUG) {
			debug(buttonState, buttonAccumulators, 1);
		}
		GenerateTestSPISignal();
	}

	uint16_t buttonBits = readButtons();
	uint16_t populatedButtonBits = buttonBits & 0b0111111111100000;

	for(int buttonIndex = 0; buttonIndex < 16; buttonIndex++) {
		uint16_t buttonMask = (1 << buttonIndex);
		uint16_t buttonState = populatedButtonBits & buttonMask;
		if(buttonState) {
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
	HAL_Delay(500);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

	__HAL_RCC_SPI2_CLK_ENABLE();
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
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

/* SPI3 init function */
static void MX_SPI3_Init(void)
{

  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    //GPIO_InitStruct.Alternate = GPIO_AF0_SYS;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_0;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin  = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin  = GPIO_PIN_1;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin       = GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
 
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

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
