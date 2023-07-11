/*
 * GY271.c
 *
 *  Created on: Jul 5, 2023
 *      Author: mohammed khaled
 */




#include "GY271.h"


/* Default I2C address */
#define GY271_I2C_ADDR			(0x3C)


/**
 * @defgroup  GY271 Registers defines
 * @{
 */
#define 	GY271_CONFIG_A		0x00
#define 	GY271_CONFIG_B		0x01
#define 	GY271_MODE			0x02
#define		GY271_XOUT_H		0x03
#define		GY271_XOUT_L		0x04
#define		GY271_YOUT_H		0x05
#define		GY271_YOUT_L		0x06
#define		GY271_ZOUT_H		0x07
#define		GY271_ZOUT_L		0x08
#define		GY271_STATUS		0x09
#define		GY271_CHIP_ID_A    	0x0A
#define		GY271_CHIP_ID_B    	0x0B
#define		GY271_CHIP_ID_C    	0x0C


#define 	REG_SIZE			 	(1U)   // 1 byte
/**
 * @}
 */

static 		float   	magGain;

/**
 * @defgroup  Functions Implementation
 * @{
 */


GY271_Result GY271_Init(GY271 * dataStruct){

	I2C_HandleTypeDef * I2Cx = dataStruct->I2Cx;
	uint8_t regVal = 0x01;
	uint8_t temp;
	uint8_t DevAddress = GY271_I2C_ADDR;
	//-------------------------
	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(I2Cx ,DevAddress, 5 ,1000))
	{
				return GY271_Result_DeviceNotConnected;
	}

	//-------------------------
	regVal = ((dataStruct->samplesRate << 5)  | (dataStruct->dataRate << 2)) & (0b01111100) ;

	/*Write config Register A*/
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, GY271_CONFIG_A, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
		return GY271_Result_Error;
	}

	// Assert the value
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, GY271_CONFIG_A, REG_SIZE, &temp, REG_SIZE, 1000))
	{
		return GY271_Result_Error;
	}

	if(temp != regVal){

		return GY271_Result_Error;
	}


	//-------------------------
	regVal = (dataStruct->range << 5) & (0b11100000);

	/*Write config Register B*/
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, GY271_CONFIG_B, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
		return GY271_Result_Error;
	}

	// Assert the value
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, GY271_CONFIG_B, REG_SIZE, &temp, REG_SIZE, 1000))
	{
		return GY271_Result_Error;
	}

	if(temp != regVal){

		return GY271_Result_Error;
	}

	//-------------------------
	regVal = dataStruct->mode & (0b00000011);

	/*Write Mode Register */
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, GY271_MODE, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
		return GY271_Result_Error;
	}

	// Assert the value
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, GY271_MODE, REG_SIZE, &temp, REG_SIZE, 1000))
	{
		return GY271_Result_Error;
	}

	if(temp != regVal){

		return GY271_Result_Error;
	}

	/* set mag gain*/
	switch (dataStruct->range) {
		case GY271_Range_0_9Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_0_9Ga;
			break;
		case GY271_Range_1_3Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_1_3Ga;
			break;
		case GY271_Range_1_9Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_1_9Ga;
			break;
		case GY271_Range_2_5Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_2_5Ga;
			break;
		case GY271_Range_4_0Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_4_0Ga;
			break;
		case GY271_Range_4_7Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_4_7Ga;
			break;
		case GY271_Range_5_6Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_5_6Ga;
			break;
		case GY271_Range_8_1Ga:
			magGain = (float)1.0 / GY271_GAIN_LSB_8_1Ga;
			break;

		default:
			break;
	}


	return GY271_Result_Ok;

}



GY271_Result GY271_getData( GY271* dataStruct){

	I2C_HandleTypeDef * I2Cx = dataStruct->I2Cx;
	uint8_t data[6];

	/*Read Magnemeter*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_XOUT_H, 1, data, 6, 1000)){
		return GY271_Result_Error;
	}

	/*Format*/
	dataStruct->Compass_Raw[0] = (int16_t) (data[0] << 8 | data[1]);
	dataStruct->Compass_Raw[1] = (int16_t) (data[2] << 8 | data[3]);
	dataStruct->Compass_Raw[2] = (int16_t) (data[4] << 8 | data[5]);

	dataStruct->Compass_Xyz[0] = dataStruct->Compass_Raw[0] * magGain;
	dataStruct->Compass_Xyz[1] = dataStruct->Compass_Raw[1] * magGain;
	dataStruct->Compass_Xyz[2] = dataStruct->Compass_Raw[2] * magGain;



	return GY271_Result_Ok;
}

GY271_Result GY271_getReadyFlag(GY271* dataStruct){

	I2C_HandleTypeDef * I2Cx = dataStruct->I2Cx;
	uint8_t regVal;

	/*Read status register*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_STATUS, 1, &regVal, 1, 1000)){
		return GY271_Result_Error;
	}

	dataStruct->dataReadFlag = regVal & 0x01;

	return GY271_Result_Ok;

}


/**
 * @}
 */
