
/******************** (C) COPYRIGHT 2018 STMicroelectronics ********************
* File Name          : SensorDemo_BlueMS_main.c
* Author             : RF Application Team
* Version            : 1.1.0
* Date               : 20-November-2017
* Description        : Sensor Demo application for interacting with ST BlueMS app
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/**
 * @file SensorDemo_BlueMS_main.c
 * @brief This application contains an example which shows how implementing the Sensor Demo application
 * tailored for interacting with the ST BlueMS smartphone app.
 * The device sends periodically, to the BlueMS APP, the data collected from the motion sensors: accelerometer and gyroscope.
 * And from the environmental sensors: pressure sensor and temperature sensor.
 * The usage is similar to the firmware example BLE_Examples/SensorDemo.
 * 

* \section ATOLLIC_project ATOLLIC project
  To use the project with ATOLLIC TrueSTUDIO for ARM, please follow the instructions below:
  -# Open the ATOLLIC TrueSTUDIO for ARM and select File->Import... Project menu. 
  -# Select Existing Projects into Workspace. 
  -# Select the ATOLLIC project
  -# Select desired configuration to build from Project->Manage Configurations
  -# Select Project->Rebuild Project. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \section KEIL_project KEIL project
  To use the project with KEIL uVision 5 for ARM, please follow the instructions below:
  -# Open the KEIL uVision 5 for ARM and select Project->Open Project menu. 
  -# Open the KEIL project
     <tt> ...\\Project\\BLE_Examples\\BLE_SensorDemo_BlueMSapp\\MDK-ARM\\BlueNRG-1\\BLE_SensorDemo_BlueMSapp.uvprojx </tt> or
     <tt> ...\\Project\\BLE_Examples\\BLE_SensorDemo_BlueMSapp\\MDK-ARM\\BlueNRG-2\\BLE_SensorDemo_BlueMSapp.uvprojx </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild all target files. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
  -# Select Project->Download to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \section IAR_project IAR project
  To use the project with IAR Embedded Workbench for ARM, please follow the instructions below:
  -# Open the Embedded Workbench for ARM and select File->Open->Workspace menu. 
  -# Open the IAR project
     <tt> ...\\Project\\BLE_Examples\\BLE_SensorDemo_BlueMSapp\\EWARM\\BlueNRG-1\\BLE_SensorDemo_BlueMSapp.eww </tt> or
     <tt> ...\\Project\\BLE_Examples\\BLE_SensorDemo_BlueMSapp\\EWARM\\BlueNRG-2\\BLE_SensorDemo_BlueMSapp.eww </tt>
  -# Select desired configuration to build
  -# Select Project->Rebuild All. This will recompile and link the entire application
  -# To download the binary image, please connect STLink to JTAG connector in your board (if available).
  -# Select Project->Download and Debug to download the related binary image.
  -# Alternatively, open the BlueNRG1 Flasher utility and download the built binary image.

* \subsection Project_configurations Project configurations
- \c Release - Release configuration


* \section Board_supported Boards supported
- \c STEVAL-IDB007V1
- \c STEVAL-IDB007V2
- \c STEVAL-IDB008V1
- \c STEVAL-IDB008V2
- \c STEVAL-IDB009V1


* \section Power_settings Power configuration settings
@table

==========================================================================================================
|                                         STEVAL-IDB00XV1                                                |
----------------------------------------------------------------------------------------------------------
| Jumper name |            |  Description                                                                |
| JP1         |   JP2      |                                                                             |
----------------------------------------------------------------------------------------------------------
| ON 1-2      | ON 2-3     | USB supply power                                                            |
| ON 2-3      | ON 1-2     | The supply voltage must be provided through battery pins.                   |
| ON 1-2      |            | USB supply power to STM32L1, JP2 pin 2 external power to BlueNRG1           |


@endtable 

* \section Jumper_settings Jumper settings
@table

========================================================================================================================================================================================
|                                                                             STEVAL-IDB00XV1                                                                                          |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| Jumper name |                                                                Description                                                                                             |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------          
| JP1         | 1-2: to provide power from USB (JP2:2-3). 2-3: to provide power from battery holder (JP2:1-2)                                                                          |          
| JP2         | 1-2: to provide power from battery holder (JP1:2-3). 2-3: to provide power from USB (JP1:1-2). Pin2 to VDD  to provide external power supply to BlueNRG-1 (JP1: 1-2)   |
| JP3         | pin 1 and 2 UART RX and TX of MCU. pin 3 GND.                                                                                                                          |          
| JP4         | Fitted: to provide VBLUE to BlueNRG1. It can be used also for current measurement.                                                                                     |
| JP5         | Fitted : TEST pin to VBLUE. Not fitted:  TEST pin to GND                                                                                                               |


@endtable 
                        
* \section Pin_settings Pin settings
@table
|  PIN name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
----------------------------------------------------------------------------------------------------------------------------
|    ADC1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    ADC2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO0    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO1    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO11    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO12    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO13    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO14    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO15    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO16    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO17    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO18    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO19    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|     IO2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    IO20    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO21    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO22    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO23    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO24    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|    IO25    |        N.A.        |        N.A.        |        N.A.        |        N.A.        |      Not Used      |
|     IO3    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO5    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO6    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO7    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     IO8    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|    TEST1   |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable 

* \section Serial_IO Serial I/O
@table
| Parameter name  | Value            | Unit      |
----------------------------------------------------
| Baudrate        | 115200 [default] | bit/sec   |
| Data bits       | 8                | bit       |
| Parity          | None             | bit       |
| Stop bits       | 1                | bit       |
@endtable

* \section LEDs_description LEDs description
@table
|  LED name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
----------------------------------------------------------------------------------------------------------------------------
|     DL1    |    Activity led    |    Activity led    |    Activity led    |    Activity led    |    Activity led    |
|     DL2    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|     DL3    |      Error led     |      Error led     |      Error led     |      Error led     |      Error led     |
|     DL4    |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |

@endtable


* \section Buttons_description Buttons description
@table
|   BUTTON name  |   STEVAL-IDB007V1  |   STEVAL-IDB007V2  |   STEVAL-IDB008V1  |   STEVAL-IDB008V2  |   STEVAL-IDB009V1  |
--------------------------------------------------------------------------------------------------------------------------------
|      PUSH1     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      PUSH2     |      Not Used      |      Not Used      |      Not Used      |      Not Used      |      Not Used      |
|      RESET     |   Reset BlueNRG1   |   Reset BlueNRG1   |   Reset BlueNRG2   |   Reset BlueNRG2   |   Reset BlueNRG2   |

@endtable

* \section Usage Usage

This is a demonstration example of the Sensor Demo application version tailored for interacting with the ST BlueMS smarthphone application. 
Refer to the related documentation for more details.

**/
   
/** @addtogroup BlueNRG1_demonstrations_applications
 * BlueNRG-1 SensorDemo with App \see SensorDemo_BlueMS_main.c for documentation.
 *
 *@{
 */

/** @} */
/** \cond DOXYGEN_SHOULD_SKIP_THIS
 */
/* Includes ------------------------------------------------------------------*/
#include <stdio.h>

#include <stdlib.h>
#include <stdio.h>

#include "BlueNRG1_it.h"
#include "BlueNRG1_conf.h"
#include "ble_const.h" 
#include "bluenrg1_stack.h"
#include "SDK_EVAL_Config.h"
#include "sleep.h"
#include "sensor.h"
#include "SensorDemo_config.h"
#include "gatt_db.h"
#include "miscutil.h"

#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "vl53l1_platform.h"

//#include "ICM20948.h"
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#ifndef DEBUG
#define DEBUG 0
#endif

#if DEBUG
#include <stdio.h>
#define PRINTF(...) printf(__VA_ARGS__)
#else
#define PRINTF(...)
#endif

#define BLE_SENSOR_VERSION_STRING "1.0.0" 

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Set the Application Service Max number of attributes records with init parameters coming from application *.config.h file */
uint8_t Services_Max_Attribute_Records[NUMBER_OF_APPLICATION_SERVICES] = {MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_1, MAX_NUMBER_ATTRIBUTES_RECORDS_SERVICE_2};

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void) 
{
  uint8_t ret;
	int status;
	uint8_t byteData,sensorState=0, dataReady;
	uint16_t wordData;
	
	uint16_t DistanceLeft;
	uint16_t DistanceCentre;
	uint16_t DistanceRight;
	
	uint8_t ToFSensor = 1;
	
	printf("Device started");
  /* System Init */
  SystemInit();
  
  /* Identify BlueNRG1 platform */
  SdkEvalIdentification();

  /* Configure I/O communication channel */
  SdkEvalComUartInit(UART_BAUDRATE);

	/* Configure I2C @ 400 kHz */
	SdkEvalI2CInit(100000);
	
	GPIO_InitType GPIO_InitStructure;
	/** Init Structure */
	GPIO_StructInit(&GPIO_InitStructure);
	
	/** Configure GPIO_Pin_7 for Proximity interrupt */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = ENABLE;
	GPIO_Init(&GPIO_InitStructure);
	
	/** Configure 10 12 13 for TOF Sensor */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Output;
	GPIO_InitStructure.GPIO_Pull = DISABLE;
	GPIO_InitStructure.GPIO_HighPwr = ENABLE;
	GPIO_Init(&GPIO_InitStructure);

	/* Put the PIN off */
  GPIO_WriteBit(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, Bit_RESET); //Turn off pins
	
	GPIO_WriteBit(GPIO_Pin_13, Bit_SET);
//	GPIO_WriteBit(GPIO_Pin_13 | GPIO_Pin_14 , Bit_RESET);
	/* Configure Gyroscope at 0x68 */
//	ICM_SelectBank(USER_BANK_0);
//	ICM_PowerOn();
	
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
  status = VL53L1_RdByte(VL53L1_I2C_SLAVE_ADDR, 0x010F, &byteData);
  printf("VL53L1X Model_ID: %X\n", byteData);
  status = VL53L1_RdByte(VL53L1_I2C_SLAVE_ADDR, 0x0110, &byteData);
  printf("VL53L1X Module_Type: %X\n", byteData);
  status = VL53L1_RdWord(VL53L1_I2C_SLAVE_ADDR, 0x0111, &wordData);
  printf("VL53L1X: %X\n", wordData);
	
	printf("Help started");
	while(sensorState==0){
		status = VL53L1X_BootState(VL53L1_I2C_SLAVE_ADDR, &sensorState);
	}
				printf("Chip booted\n");
//				sensorState = 0;
				/* This function must to be called */
				status = VL53L1X_SensorInit(VL53L1_I2C_SLAVE_ADDR);
				printf("Left Sensor Initialised\n");
				
				VL53L1X_SetI2CAddress(VL53L1_I2C_SLAVE_ADDR, VL53l1X_LEFT_ADDR);
				printf("Left Sensor address changed");
				
				status = VL53L1_RdByte(VL53l1X_LEFT_ADDR, 0x010F, &byteData);
				printf("Left VL53L1X Model_ID: %X\n", byteData);
				status = VL53L1_RdByte(VL53l1X_LEFT_ADDR, 0x0110, &byteData);
				printf("Left VL53L1X Module_Type: %X\n", byteData);
				status = VL53L1_RdWord(VL53l1X_LEFT_ADDR, 0x0111, &wordData);
				printf("Left VL53L1X: %X\n", wordData);
				
				while(sensorState==0){
					status = VL53L1X_BootState(VL53l1X_LEFT_ADDR, &sensorState);
				}
				printf("Left Chip booted\n");
				status = VL53L1X_StartRanging(VL53l1X_LEFT_ADDR);   /* This function has to be called to enable the ranging */
				
	
	
//	/* Configure Gyroscope at 0x68 */
//	ICM_SelectBank(USER_BANK_0);
//	ICM_PowerOn();
		
//  /* BlueNRG-1 stack init */
//  ret = BlueNRG_Stack_Initialization(&BlueNRG_Stack_Init_params);
//  if (ret != BLE_STATUS_SUCCESS) {
//    PRINTF("Error in BlueNRG_Stack_Initialization() 0x%02x\r\n", ret);
//    while(1);
//  }
//  
//  /* Application demo Led Init */
//  SdkEvalLedInit(LED1); //Activity led 
//  SdkEvalLedInit(LED3); //Error led 
//  SdkEvalLedOn(LED1);
//  SdkEvalLedOff(LED3);
//  
//  PRINTF("BlueNRG-1 BLE Sensor Demo Application (version: %s)\r\n", BLE_SENSOR_VERSION_STRING); 
//  
//  /* Sensor Device Init */
//  ret = Sensor_DeviceInit();
//  if (ret != BLE_STATUS_SUCCESS) {
//    SdkEvalLedOn(LED3);
//    while(1);
//  }
		
  while(1)
  {		
//	  status = VL53L1X_GetRangeStatus(newAdd, &RangeStatus);
//	  status = VL53L1X_GetDistance(newAdd, &Distance);
//	  status = VL53L1X_GetSignalRate(newAdd, &SignalRate);
//	  status = VL53L1X_GetAmbientRate(newAdd, &AmbientRate);
//	  status = VL53L1X_ClearInterrupt(newAdd); /* clear interrupt has to be called to enable next interrupt*/
		status = VL53L1X_GetDistance(VL53l1X_LEFT_ADDR, &DistanceLeft);
//		status = VL53L1X_GetDistance(VL53l1X_CENTRE_ADDR, &DistanceCentre);
//		status = VL53L1X_GetDistance(VL53l1X_RIGHT_ADDR, &DistanceRight);
		
	  printf("Data: ");
		printf("%u \n", DistanceLeft);
		
//    /* BLE Stack Tick */
//    BTLE_StackTick();

//    /* Application Tick */
//    APP_Tick();
//    
//    /* Power Save management */
//    BlueNRG_Sleep(SLEEPMODE_NOTIMER, 0, 0);
  }   
}

/****************** BlueNRG-1 Sleep Management Callback ********************************/

SleepModes App_SleepMode_Check(SleepModes sleepMode)
{
  if(SdkEvalComIOTxFifoNotEmpty() || SdkEvalComUARTBusy())
    return SLEEPMODE_RUNNING;
  
  return SLEEPMODE_NOTIMER;
}

/***************************************************************************************/

#ifdef USE_FULL_ASSERT
/*******************************************************************************
* Function Name  : assert_failed
* Description    : Reports the name of the source file and the source line number
*                  where the assert_param error has occurred.
* Input          : - file: pointer to the source file name
*                  - line: assert_param error line source number
* Output         : None
* Return         : None
*******************************************************************************/
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {}
}
#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
/** \endcond
 */
