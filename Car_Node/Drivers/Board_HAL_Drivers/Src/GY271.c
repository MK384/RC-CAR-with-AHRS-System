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


/**
 * @defgroup  Functions Implementation
 * @{
 */


GY271_Result GY271_Init(I2C_HandleTypeDef* I2Cx , GY271_ModeControl mode , GY271_DataRate dataRate, GY271_FieldRange range, GY271_SampleAvgRate samplesRate )
{


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
	regVal = ((samplesRate << 5)  | (dataRate << 2)) & (0b01111100) ;

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
	regVal = (range << 5) & (0b11100000);

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
	regVal = mode & (0b00000011);

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



	return GY271_Result_Ok;

}



GY271_Result GY271_ReadData(I2C_HandleTypeDef* I2Cx , GY271* dataStruct)
{

	uint8_t data[6];

	/*Read Magnemeter*/
	if (HAL_I2C_Mem_Read(I2Cx, GY271_I2C_ADDR, GY271_XOUT_H, 1, data, 6, 1000)){
		return GY271_Result_Error;
	}

	/*Format*/
	dataStruct->Compass_X = (int16_t) (data[0] << 8 | data[1]);
	dataStruct->Compass_Y = (int16_t) (data[2] << 8 | data[3]);
	dataStruct->Compass_Z = (int16_t) (data[4] << 8 | data[5]);



	return GY271_Result_Ok;
}



/**
 * @}
 */
