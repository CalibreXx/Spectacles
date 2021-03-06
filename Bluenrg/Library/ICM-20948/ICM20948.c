/*
 * ICM20948.c
 *
 *  Created on: Oct 26, 2018
 *      Author: cory
 */

//#include "main.h"
//#include "gpio.h"
//#include "spi.h"
//#include "usart.h"
//#include "dma.h"

#include "ICM20948.h"
#include <string.h>
#include "SDK_EVAL_I2C.h"

void SdkDelayMs(volatile uint32_t lTimeMs);
volatile uint32_t lSystickCounter=0;
void SdkDelayMs(volatile uint32_t lTimeMs)
{
  uint32_t nWaitPeriod = ~lSystickCounter;
  
  if(nWaitPeriod<lTimeMs)
  {
    while( lSystickCounter != 0xFFFFFFFF);
    nWaitPeriod = lTimeMs-nWaitPeriod;
  }
  else
    nWaitPeriod = lTimeMs+ ~nWaitPeriod;
  
  while( lSystickCounter != nWaitPeriod ) ;

}

/*
 *
 * SPI abstraction
 * Creates a layer between STM32 HAL and the ICM library that will allow for easy platform swap
 *
 */
void ICM_ReadBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg | 0x80;
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
//	HAL_SPI_Receive_DMA(SPI_BUS, pData, Size);
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
	
	uint8_t buffer[1];
	SdkEvalI2CRead16 ( buffer , ICM20948_SLAVE_ADD , reg, Size);
	*pData = buffer[0];
}

void ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t Size) // ***
{
	reg = reg & 0x7F;
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
//	HAL_SPI_Transmit_DMA(SPI_BUS, pData, Size);
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
		
	SdkEvalI2CWrite16 ( pData , ICM20948_SLAVE_ADD , reg, Size);

}

void ICM_ReadOneByte(uint8_t reg, uint8_t* pData) // ***
{
	reg = reg | 0x80;
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
//	while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
//		;
//	HAL_SPI_Receive_DMA(SPI_BUS, pData, 1);
//	while (HAL_SPI_GetState(SPI_BUS) != HAL_SPI_STATE_READY)
//		;
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
		uint8_t buffer[1];
	SdkEvalI2CRead16 ( buffer , ICM20948_SLAVE_ADD , reg, 1);
	*pData = buffer[0];
}

void ICM_WriteOneByte(uint8_t reg, uint8_t Data) // ***
{
	reg = reg & 0x7F;
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_RESET);
//	HAL_SPI_Transmit_DMA(SPI_BUS, &reg, 1);
//	HAL_SPI_Transmit_DMA(SPI_BUS, &Data, 1);
//	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, GPIO_PIN_SET);
	uint8_t buffer [1];
	buffer[0] = (uint8_t)(Data);
	SdkEvalI2CWrite16 ( buffer , ICM20948_SLAVE_ADD , reg, 1);
}

/*
 *
 * AUX I2C abstraction for magnetometer
 * Creates a layer between STM32 HAL and the ICM library that will allow for easy platform swap
 *
 */
void ICM_MagWrite(uint8_t reg, uint8_t value) {
	ICM_WriteOneByte(0x7F, 0x30);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x03, 0x0C);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x04, reg);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x06, value);
	SdkDelayMs (1);
}

static uint8_t ICM_MagRead(uint8_t reg) {
	uint8_t Data;
	ICM_WriteOneByte(0x7F, 0x30);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x03, 0x0C | 0x80);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x04, reg);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x06, 0xff);
	SdkDelayMs (1);
	ICM_WriteOneByte(0x7F, 0x00);
	ICM_ReadOneByte(0x3B, &Data);
	SdkDelayMs (1);
	return Data;
}

void ICM_ReadMagData(int16_t heading[3]) {
	uint8_t mag_buffer[10];
	mag_buffer[0] = ICM_MagRead(0x01);
	mag_buffer[1] = ICM_MagRead(0x11);
	mag_buffer[2] = ICM_MagRead(0x12);
	heading[0] = mag_buffer[1] | mag_buffer[2] << 8;
	mag_buffer[3] = ICM_MagRead(0x13);
	mag_buffer[4] = ICM_MagRead(0x14);
	heading[1] = mag_buffer[3] | mag_buffer[4] << 8;
	mag_buffer[5] = ICM_MagRead(0x15);
	mag_buffer[6] = ICM_MagRead(0x16);
	heading[2] = mag_buffer[5] | mag_buffer[6] << 8;
	ICM_MagWrite(0x31, 0x01);
}

/*
 *
 * Sequence to setup ICM290948 as early as possible after power on
 *
 */
void ICM_PowerOn(void) {
	char uart_buffer[200];
	uint8_t whoami = 0xEA;
	uint8_t test = ICM_WHOAMI();
	ICM_CSHigh();
	SdkDelayMs (10);
	ICM_SelectBank(USER_BANK_0);
	SdkDelayMs (10);
	ICM_Disable_I2C();
	SdkDelayMs (10);
	ICM_SetClock((uint8_t) CLK_BEST_AVAIL);
	SdkDelayMs (10);
	ICM_AccelGyroOff();
	SdkDelayMs (10);
	ICM_AccelGyroOn();
	SdkDelayMs (10);
	ICM_Initialize();
}
<<<<<<< HEAD

<<<<<<< HEAD
=======
>>>>>>> parent of 5878563... Added to PoweON in ICM20948
=======
void ICM_SelfTest(float *destination){
	uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
  uint8_t selfTest[6];
  int32_t gAvg[3] = {0}, aAvg[3] = {0}, aSTAvg[3] = {0}, gSTAvg[3] = {0};
  float factoryTrim[6];
  uint8_t FS = 0;
	
	// Get stable time source
  // Auto select clock source to be PLL gyroscope reference if ready else
  ICM_WriteOneByte(PWR_MGMT_1, 0x01);
  SdkDelayMs (200);
	
	// Switch to user bank 2
  ICM_WriteOneByte(REG_BANK_SEL, 0x20);
  // Set gyro sample rate to 1 kHz
  ICM_WriteOneByte(GYRO_SMPLRT_DIV, 0x00);
  // Set gyro sample rate to 1 kHz, DLPF to 119.5 Hz and FSR to 250 dps
  ICM_WriteOneByte( GYRO_CONFIG_1, 0x11);
  // Set accelerometer rate to 1 kHz and bandwidth to 111.4 Hz
  // Set full scale range for the accelerometer to 2 g
  ICM_WriteOneByte( ACCEL_CONFIG, 0x11);
  // Switch to user bank 0
  ICM_WriteOneByte( REG_BANK_SEL, 0x00);
	
	// Get average current values of gyro and acclerometer
  for (int ii = 0; ii < 200; ii++)
  {
		printf("BHW::ii = ");
		printf("%d\n",ii);
		
    // Read the six raw data registers into data array
    readBytes(ICM20948_ADDRESS, ACCEL_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    // Read the six raw data registers sequentially into data array
    readBytes(ICM20948_ADDRESS, GYRO_XOUT_H, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }
	
	
}
>>>>>>> parent of ae51721... For Multiranging
uint16_t ICM_Initialize(void) {
	ICM_SelectBank(USER_BANK_2);
	SdkDelayMs (10);
	ICM_SetGyroRateLPF(GYRO_RATE_250, GYRO_LPF_17HZ);
	SdkDelayMs (10);

	// Set gyroscope sample rate to 100hz (0x0A) in GYRO_SMPLRT_DIV register (0x00)
	ICM_WriteOneByte(0x00, 0x0A);
	SdkDelayMs (10);

	// Set accelerometer low pass filter to 136hz (0x11) and the rate to 8G (0x04) in register ACCEL_CONFIG (0x14)
	ICM_WriteOneByte(0x14, (0x04 | 0x11));

	// Set accelerometer sample rate to 225hz (0x00) in ACCEL_SMPLRT_DIV_1 register (0x10)
	ICM_WriteOneByte(0x10, 0x00);
	SdkDelayMs (10);

	// Set accelerometer sample rate to 100 hz (0x0A) in ACCEL_SMPLRT_DIV_2 register (0x11)
	ICM_WriteOneByte(0x11, 0x0A);
	SdkDelayMs (10);

	ICM_SelectBank(USER_BANK_2);
	SdkDelayMs (20);

	// Configure AUX_I2C Magnetometer (onboard ICM-20948)
	ICM_WriteOneByte(0x7F, 0x00);
	ICM_WriteOneByte(0x0F, 0x30);
	ICM_WriteOneByte(0x03, 0x20);
	ICM_WriteOneByte(0x7F, 0x30);
	ICM_WriteOneByte(0x01, 0x4D);
	ICM_WriteOneByte(0x02, 0x01);
	ICM_WriteOneByte(0x05, 0x81);
	ICM_MagWrite(0x32, 0x01);
	SdkDelayMs (1000);
	ICM_MagWrite(0x31, 0x02);
	return 1337;
}

void ICM_ReadAccelGyroData(void) {
	uint8_t raw_data[12];
	ICM_ReadBytes(0x2D, raw_data, 12);

	accel_data[0] = (raw_data[0] << 8) | raw_data[1];
	accel_data[1] = (raw_data[2] << 8) | raw_data[3];
	accel_data[2] = (raw_data[4] << 8) | raw_data[5];

	gyro_data[0] = (raw_data[6] << 8) | raw_data[7];
	gyro_data[1] = (raw_data[8] << 8) | raw_data[9];
	gyro_data[2] = (raw_data[10] << 8) | raw_data[11];

	accel_data[0] = accel_data[0] / 8;
	accel_data[1] = accel_data[1] / 8;
	accel_data[2] = accel_data[2] / 8;

	gyro_data[0] = gyro_data[0] / 250;
	gyro_data[1] = gyro_data[1] / 250;
	gyro_data[2] = gyro_data[2] / 250;
}
void ICM_SelectBank(uint8_t bank) {
	ICM_WriteOneByte(USER_BANK_SEL, bank);
}
void ICM_Disable_I2C(void) {
	ICM_WriteOneByte(0x03, 0x78);
}
//void ICM_CSHigh(void) {
////	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, SET);
//}
//void ICM_CSLow(void) {
////	HAL_GPIO_WritePin(ICM_CS_GPIO_Port, ICM_CS_Pin, RESET);
//}
void ICM_SetClock(uint8_t clk) {
	ICM_WriteOneByte(PWR_MGMT_1, clk);
}
void ICM_AccelGyroOff(void) {
	ICM_WriteOneByte(PWR_MGMT_2, (0x38 | 0x07));
}
void ICM_AccelGyroOn(void) {
	ICM_WriteOneByte(0x07, (0x00 | 0x00));
}
uint8_t ICM_WHOAMI(void) {
	uint8_t spiData = 0x01;
	ICM_ReadOneByte(0x00, &spiData);
	return spiData;
}
void ICM_SetGyroRateLPF(uint8_t rate, uint8_t lpf) {
	ICM_WriteOneByte(GYRO_CONFIG_1, (rate | lpf));
}
