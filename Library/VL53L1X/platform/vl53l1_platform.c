
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
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
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>
#include "SDK_EVAL_I2C.h"
#include "clock.h"

extern volatile uint32_t SystemClockTick;

uint32_t value;

int8_t VL53L1_WriteMulti( uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count) {

	int8_t status = 0;

	status = SdkEvalI2CWrite16(pdata, dev, index, count);

	return status;
}

int8_t VL53L1_ReadMulti(uint16_t dev, uint16_t index, uint8_t *pdata, uint32_t count){

	int8_t status = 0;

	status = SdkEvalI2CRead16(pdata, dev, index, count);

	return status;
}

int8_t VL53L1_WrByte(uint16_t dev, uint16_t index, uint8_t data) {

	uint8_t buffer[1];

	buffer[0] = (uint8_t) (data);

	return VL53L1_WriteMulti(dev, index, buffer, 1);

}


int8_t VL53L1_WrWord(uint16_t dev, uint16_t index, uint16_t data) {

	uint8_t buffer[2];

	// Split 16-bit word into MS and LS uint8_t
	buffer[0] = (uint8_t) (data >> 8);
	buffer[1] = (uint8_t) (data & 0x00FF);

	return VL53L1_WriteMulti(dev, index, buffer, 2);

}

int8_t VL53L1_WrDWord(uint16_t dev, uint16_t index, uint32_t data) {

	uint8_t buffer[4];

	// Split 32-bit word into MS ... LS bytes
	buffer[0] = (uint8_t) (data >> 24);
	buffer[1] = (uint8_t) ((data & 0x00FF0000) >> 16);
	buffer[2] = (uint8_t) ((data & 0x0000FF00) >> 8);
	buffer[3] = (uint8_t) (data & 0x000000FF);

	return VL53L1_WriteMulti(dev, index, buffer, 4);

}

int8_t VL53L1_RdByte(uint16_t dev, uint16_t index, uint8_t *data) {

	uint8_t status = 0;
	uint8_t buffer[1];

	 status = VL53L1_ReadMulti(dev, index, buffer, 1);
	 *data = buffer[0];

	 return status;

}

int8_t VL53L1_RdWord(uint16_t dev, uint16_t index, uint16_t *data) {
	
	uint8_t status = 0;
	uint8_t buffer[2];

	status = VL53L1_ReadMulti(dev, index, buffer, 2);
	*data = (uint16_t) (((uint16_t) (buffer[0]) << 8) + (uint16_t) buffer[1]);

	return status;

}

int8_t VL53L1_RdDWord(uint16_t dev, uint16_t index, uint32_t *data) {

	uint8_t status = 0;
	uint8_t buffer[4];

	status = VL53L1_ReadMulti(dev, index, buffer, 4);
	*data = (uint16_t)((uint32_t) buffer[0] << 24) + ((uint32_t) buffer[1] << 16) + ((uint32_t) buffer[2] << 8) + (uint32_t) buffer[3];

	return status;

}

double StartTime = 0, CurrentTime;
int8_t VL53L1_WaitMs(uint16_t dev, int32_t wait_ms){
	
	StartTime = SystemClockTick;
	CurrentTime = SystemClockTick - StartTime;
	while (CurrentTime < wait_ms) {
		CurrentTime = ((SystemClockTick) - StartTime);
	}
	
	return 0;

}
