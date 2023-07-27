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
#define		GY271_ZOUT_H		0x05
#define		GY271_ZOUT_L		0x06
#define		GY271_YOUT_H		0x07
#define		GY271_YOUT_L		0x08
#define		GY271_STATUS		0x09
#define		GY271_CHIP_ID_A    	0x0A
#define		GY271_CHIP_ID_B    	0x0B
#define		GY271_CHIP_ID_C    	0x0C



#define		DATA_AVG_BITS_OFFSET		5
#define		DATA_RATE_BITS_OFFSET		2
#define 	FS_RANGE_BITS_OFFSET		5






#define 	REG_SIZE			 	(1U)   // 1 byte
/**
 * @}
 */
static I2C_HandleTypeDef * I2Cx;
static 		float   	magGain;
static GY271_ModeControl mode;

/**
 * @defgroup  Functions Implementation
 * @{
 */


GY271_Result GY271_Init(GY271_HandleType * dataStruct){

	 I2Cx = dataStruct->I2Cx;
	uint8_t regVal = 0x01;
	uint8_t temp;
	uint8_t DevAddress = GY271_I2C_ADDR;
	mode = dataStruct->mode;
	//-------------------------
	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(I2Cx ,DevAddress, 5 ,1000)) return GY271_Result_DeviceNotConnected;


	//-------------------------

	/*Write config Register A*/
	if (dataStruct->mode == GY271_Mode_Continuous){
		// Write the Register
		regVal = ((dataStruct->samplesRate << DATA_AVG_BITS_OFFSET) | (dataStruct->dataRate << DATA_RATE_BITS_OFFSET)) ;
		if (HAL_I2C_Mem_Write(I2Cx, DevAddress, GY271_CONFIG_A, REG_SIZE, &regVal, REG_SIZE, 1000))  return GY271_Result_Error;


		// Assert the value
		if (HAL_I2C_Mem_Read(I2Cx, DevAddress, GY271_CONFIG_A, REG_SIZE, &temp, REG_SIZE, 1000)) return GY271_Result_Error;
		if(temp != regVal) return GY271_Result_Error;

	}



	//-------------------------

	/*Write config Register B*/
	regVal = (dataStruct->range << FS_RANGE_BITS_OFFSET);
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, GY271_CONFIG_B, REG_SIZE, &regVal, REG_SIZE, 1000)) 	return GY271_Result_Error;


	// Assert the value
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, GY271_CONFIG_B, REG_SIZE, &temp, REG_SIZE, 1000))    	return GY271_Result_Error;
	if(temp != regVal) return GY271_Result_Error;


	//-------------------------
	/*Write config Mode Register*/
		regVal = dataStruct->mode & (0b00000011);

		/*Write Mode Register */
		if (HAL_I2C_Mem_Write(I2Cx, DevAddress, GY271_MODE, REG_SIZE, &regVal, REG_SIZE, 1000)) return GY271_Result_Error;

		// Assert the value
		if (HAL_I2C_Mem_Read(I2Cx, DevAddress, GY271_MODE, REG_SIZE, &temp, REG_SIZE, 1000)) return GY271_Result_Error;
		if(temp != regVal) return GY271_Result_Error;



	/* set mag gain*/
	switch (dataStruct->range) {
		case GY271_Range_0_9Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_0_9Ga;
			break;
		case GY271_Range_1_3Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_1_3Ga;
			break;
		case GY271_Range_1_9Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_1_9Ga;
			break;
		case GY271_Range_2_5Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_2_5Ga;
			break;
		case GY271_Range_4_0Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_4_0Ga;
			break;
		case GY271_Range_4_7Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_4_7Ga;
			break;
		case GY271_Range_5_6Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_5_6Ga;
			break;
		case GY271_Range_8_1Ga:
			magGain = dataStruct->outputUnit / GY271_GAIN_LSB_8_1Ga;
			break;

		default:
			break;
	}


	return GY271_Result_Ok;

}



GY271_Result GY271_getMag( float*  mag_xyz  ){


	uint8_t data[6];
	int16_t raw[3];

	if(mode == GY271_Mode_Single){

		/*Request a single read*/
		uint8_t regVal = 0x01;
		if (HAL_I2C_Mem_Write(I2Cx, GY271_I2C_ADDR, GY271_MODE, 1, &regVal, 1, 1000)) return GY271_Result_Error;

		/*wait until data is ready*/
		while(!GY271_getReadyStatus());
	}

	/*Read Magnemeter*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_XOUT_H, 1, data, 6, 1000)) return GY271_Result_Error;

	/*Format  x-> z -> y*/
	raw[0] = (int16_t) (data[0] << 8 | data[1]) ;
	raw[2] = (int16_t) (data[2] << 8 | data[3]) ;
	raw[1] = (int16_t) (data[4] << 8 | data[5]) ;

	mag_xyz[0] = ((float) raw[0]) * magGain;
	mag_xyz[1] = ((float) raw[1]) * magGain;
	mag_xyz[2] = ((float) raw[2]) * magGain;

	return GY271_Result_Ok;
}


/**
 * @brief : get mag data (in hardware units)
 */
GY271_Result GY271_getRawMag( int16_t * mag_xyz){


	uint8_t data[6];

	if(mode == GY271_Mode_Single){

		/*Request a single read*/
		uint8_t regVal = 0x01;
		if (HAL_I2C_Mem_Write(I2Cx, GY271_I2C_ADDR, GY271_MODE, 1, &regVal, 1, 1000)) return GY271_Result_Error;

		/*wait until data is ready*/
		while(!GY271_getReadyStatus());
	}

	/*Read Magnemeter*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_XOUT_H, 1, data, 6, 1000)) return GY271_Result_Error;

	/*Format  x-> z -> y*/
	mag_xyz[0] = (int16_t) (data[0] << 8 | data[1]);
	mag_xyz[2] = (int16_t) (data[2] << 8 | data[3]);
	mag_xyz[1] = (int16_t) (data[4] << 8 | data[5]);

	return GY271_Result_Ok;
}


bool GY271_getReadyStatus(void){

	uint8_t regVal;

	/*Read status register*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_STATUS, 1, &regVal, 1, 1000)) return GY271_Result_Error;

	return (regVal & 0x01);

}

bool GY271_getLockStatus(void){

	uint8_t regVal;

	/*Read status register*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_STATUS, 1, &regVal, 1, 1000))  return GY271_Result_Error;

	return ((regVal & 0x02) >> 1);

}


/**
 * @}
 */
