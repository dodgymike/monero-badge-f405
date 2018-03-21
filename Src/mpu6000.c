/*
A lot of this code was taken from CleanFlight/Betaflight source:
* src/main/drivers/accgyro/accgyro_spi_mpu6000.c
*/

#include "mpu6000.h"

void discoverMPU() {
}

uint8_t mpu6000SpiDetect(const busDevice_t *bus)
{
    IOInit(bus->busdev_u.spi.csnPin, OWNER_MPU_CS, 0);
    IOConfigGPIO(bus->busdev_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(bus->busdev_u.spi.csnPin);

    spiSetDivisor(bus->busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    spiBusWriteRegister(bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);

    uint8_t attemptsRemaining = 5;
    do {
        delay(150);

        const uint8_t whoAmI = spiBusReadRegister(bus, MPU_RA_WHO_AM_I);
        if (whoAmI == MPU6000_WHO_AM_I_CONST) {
            break;
        }
        if (!attemptsRemaining) {
            return MPU_NONE;
        }
    } while (attemptsRemaining--);

    const uint8_t productID = spiBusReadRegister(bus, MPU_RA_PRODUCT_ID);

    /* look for a product ID we recognise */

    // verify product revision
    switch (productID) {
    case MPU6000ES_REV_C4:
    case MPU6000ES_REV_C5:
    case MPU6000_REV_C4:
    case MPU6000_REV_C5:
    case MPU6000ES_REV_D6:
    case MPU6000ES_REV_D7:
    case MPU6000ES_REV_D8:
    case MPU6000_REV_D6:
    case MPU6000_REV_D7:
    case MPU6000_REV_D8:
    case MPU6000_REV_D9:
    case MPU6000_REV_D10:
        return MPU_60x0_SPI;
    }

    return MPU_NONE;
}

}

void initMPU6000() {
//    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_INITIALIZATON);

    // Device Reset
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, BIT_H_RESET);
    delay(150);

    spiBusWriteRegister(&gyro->bus, MPU_RA_SIGNAL_PATH_RESET, BIT_GYRO | BIT_ACC | BIT_TEMP);
    delay(150);

    // Clock Source PPL with Z axis gyro reference
    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delayMicroseconds(15);

    // Disable Primary I2C Interface
    spiBusWriteRegister(&gyro->bus, MPU_RA_USER_CTRL, BIT_I2C_IF_DIS);
    delayMicroseconds(15);

    spiBusWriteRegister(&gyro->bus, MPU_RA_PWR_MGMT_2, 0x00);
    delayMicroseconds(15);

    // Accel Sample Rate 1kHz
    // Gyroscope Output Rate =  1kHz when the DLPF is enabled
    spiBusWriteRegister(&gyro->bus, MPU_RA_SMPLRT_DIV, gyro->mpuDividerDrops);
    delayMicroseconds(15);

    // Gyro +/- 1000 DPS Full Scale
    spiBusWriteRegister(&gyro->bus, MPU_RA_GYRO_CONFIG, INV_FSR_2000DPS << 3); 
    delayMicroseconds(15);

    // Accel +/- 8 G Full Scale
    spiBusWriteRegister(&gyro->bus, MPU_RA_ACCEL_CONFIG, INV_FSR_8G << 3); 
    delayMicroseconds(15);

    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_PIN_CFG, 0 << 7 | 0 << 6 | 0 << 5 | 1 << 4 | 0 << 3 | 0 << 2 | 0 << 1 | 0 << 0);  // INT_ANYRD_2CLEAR
    delayMicroseconds(15);

#ifdef USE_MPU_DATA_READY_SIGNAL
    spiBusWriteRegister(&gyro->bus, MPU_RA_INT_ENABLE, MPU_RF_DATA_RDY_EN);
    delayMicroseconds(15);
#endif

    spiSetDivisor(gyro->bus.busdev_u.spi.instance, SPI_CLOCK_FAST);
    delayMicroseconds(1);
}
