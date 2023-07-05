/*
 * sd_hal_mpu6050.c
 *
 *  Created on: Feb 19, 2016
 *  Author: Sina Darvishi
 */

/**
 * |----------------------------------------------------------------------
 * | Copyright (C) Sina Darvishi,2016
 * |
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * |
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */

#include "mpu6050.h"

/* Default I2C address */
#define MPU6050_I2C_ADDR			0xD0

/* Who I am register value */
#define I_AM_MPU6050				0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO			0x01
#define MPU6050_SMPLRT_DIV			0x19
#define MPU6050_CONFIG				0x1A
#define MPU6050_GYRO_CONFIG			0x1B
#define MPU6050_ACCEL_CONFIG		0x1C
#define MPU6050_MOTION_THRESH		0x1F
#define MPU6050_INT_PIN_CFG			0x37
#define MPU6050_INT_ENABLE			0x38
#define MPU6050_INT_STATUS			0x3A
#define MPU6050_ACCEL_XOUT_H		0x3B
#define MPU6050_ACCEL_XOUT_L		0x3C
#define MPU6050_ACCEL_YOUT_H		0x3D
#define MPU6050_ACCEL_YOUT_L		0x3E
#define MPU6050_ACCEL_ZOUT_H		0x3F
#define MPU6050_ACCEL_ZOUT_L		0x40
#define MPU6050_TEMP_OUT_H			0x41
#define MPU6050_TEMP_OUT_L			0x42
#define MPU6050_GYRO_XOUT_H			0x43
#define MPU6050_GYRO_XOUT_L			0x44
#define MPU6050_GYRO_YOUT_H			0x45
#define MPU6050_GYRO_YOUT_L			0x46
#define MPU6050_GYRO_ZOUT_H			0x47
#define MPU6050_GYRO_ZOUT_L			0x48
#define MPU6050_MOT_DETECT_STATUS	0x61
#define MPU6050_SIGNAL_PATH_RESET	0x68
#define MPU6050_MOT_DETECT_CTRL		0x69
#define MPU6050_USER_CTRL			0x6A
#define MPU6050_PWR_MGMT_1			0x6B
#define MPU6050_PWR_MGMT_2			0x6C
#define MPU6050_FIFO_COUNTH			0x72
#define MPU6050_FIFO_COUNTL			0x73
#define MPU6050_FIFO_R_W			0x74
#define MPU6050_WHO_AM_I			0x75

/* Gyro sensitivities in degrees/s */
#define MPU6050_GYRO_SENS_250		((float) 131.0)
#define MPU6050_GYRO_SENS_500		((float) 65.5)
#define MPU6050_GYRO_SENS_1000		((float) 32.8)
#define MPU6050_GYRO_SENS_2000		((float) 16.4)

/* Acce sensitivities in g/s */
#define MPU6050_ACCE_SENS_2			((float) 16384)
#define MPU6050_ACCE_SENS_4			((float) 8192)
#define MPU6050_ACCE_SENS_8			((float) 4096)
#define MPU6050_ACCE_SENS_16		((float) 2048)


#define 	REG_SIZE			 	(1U)   // 1 byte

/**
 * @defgroup  Functions Implementation
 * @{
 */


SD_MPU6050_Result SD_MPU6050_Init(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct,
		SD_MPU6050_Device DeviceNumber, SD_MPU6050_Accelerometer AccelerometerSensitivity,
		SD_MPU6050_Gyroscope GyroscopeSensitivity, uint8_t DataRate)
{

	uint8_t regVal;

	/* Format I2C address */
	DataStruct->Address = MPU6050_I2C_ADDR | (uint8_t)DeviceNumber;
	uint8_t DevAddress = DataStruct->Address;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(I2Cx,DevAddress, 3 ,100))
	{
				return SD_MPU6050_Result_DeviceNotConnected;
	}
	/* Check who am I */
	//------------------
		if(HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_WHO_AM_I,	 REG_SIZE, &regVal, REG_SIZE, 1000))
		{
			return SD_MPU6050_Result_Error;
		}

		/* Checking */
		if (regVal != I_AM_MPU6050)
		{
				/* Return error */
				return SD_MPU6050_Result_DeviceInvalid;
		}
	//------------------------------------------------------------------------------------

	/* Wakeup MPU6050 */
	//------------------
		/* Format array to send */
		 regVal = 0x00;

		/* Try to transmit via I2C */
		if(HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_PWR_MGMT_1, REG_SIZE, &regVal, REG_SIZE, 1000))
		{
					return SD_MPU6050_Result_Error;
		}
	//------------------


	/* Set sample rate to 1kHz */
	SD_MPU6050_SetDataRate(I2Cx,DataStruct, DataRate);

	/* Config accelerometer */
	SD_MPU6050_SetAccelerometer(I2Cx,DataStruct, AccelerometerSensitivity);

	/* Config Gyroscope */
	SD_MPU6050_SetGyroscope(I2Cx,DataStruct, GyroscopeSensitivity);

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

SD_MPU6050_Result SD_MPU6050_SetDataRate(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct, uint8_t rate)
{
	uint8_t regVal = rate;
	uint16_t DevAddress = (uint16_t) DataStruct->Address;
	/* Format array to send */

	/* Set data sample rate */
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_SMPLRT_DIV, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	/*Assert the value*/
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_SMPLRT_DIV, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	if (regVal == rate){
		/* Return OK */
		return SD_MPU6050_Result_Ok;
	}
	else{

		// return unknown error
		return SD_MPU6050_Result_Error;
	}
}

SD_MPU6050_Result SD_MPU6050_SetAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct, SD_MPU6050_Accelerometer AccelerometerSensitivity)
{
	uint8_t regVal;
	uint8_t DevAddress = DataStruct->Address;

	/* get the register content */
	if(HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	/* Config accelerometer : modify only required fields*/
	regVal = (regVal & 0b11100111) | ((uint8_t) AccelerometerSensitivity << 3);

	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_ACCEL_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}


	/*Assert the written value*/
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return SD_MPU6050_Result_Error;
	 }

	if ((regVal & 0b00011000) == ((uint8_t) AccelerometerSensitivity << 3) ){

		/* Set sensitivities for multiplying gyro and accelerometer data */
		switch (AccelerometerSensitivity) {
			case SD_MPU6050_Accelerometer_2G:
				DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_2;
				break;
			case SD_MPU6050_Accelerometer_4G:
				DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_4;
				break;
			case SD_MPU6050_Accelerometer_8G:
				DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_8;
				break;
			case SD_MPU6050_Accelerometer_16G:
				DataStruct->Acce_Mult = (float)1 / MPU6050_ACCE_SENS_16;
				break;
			default:
				break;
			}

		// Return OK
		return SD_MPU6050_Result_Ok;
	}else{
		// return unknown error
		return SD_MPU6050_Result_Error;
	}

}

SD_MPU6050_Result SD_MPU6050_SetGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct, SD_MPU6050_Gyroscope GyroscopeSensitivity)
{
	uint8_t regVal;
	uint8_t DevAddress = DataStruct->Address;


	/* get the register content */
	if(HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_GYRO_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	/* Config gyroscope : modify only required fields*/
	regVal = (regVal & 0b11100111) | ((uint8_t)GyroscopeSensitivity << 3);
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_GYRO_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	/*Assert the written value*/
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_GYRO_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return SD_MPU6050_Result_Error;
	 }

	if ((regVal & 0b00011000) == ((uint8_t) GyroscopeSensitivity << 3) ){

		/* Set sensitivities for multiplying gyro and accelerometer data */
		switch (GyroscopeSensitivity) {
				case SD_MPU6050_Gyroscope_250s:
					DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_250;
					break;
				case SD_MPU6050_Gyroscope_500s:
					DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_500;
					break;
				case SD_MPU6050_Gyroscope_1000s:
					DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_1000;
					break;
				case SD_MPU6050_Gyroscope_2000s:
					DataStruct->Gyro_Mult = (float)1 / MPU6050_GYRO_SENS_2000;
					break;
				default:
					break;
			}

		// Return OK
		return SD_MPU6050_Result_Ok;
	}else{
		// return unknown error
		return SD_MPU6050_Result_Error;
	}
}

SD_MPU6050_Result SD_MPU6050_SetLowPassFilter(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct, SD_MPU6050_Bandwidth bw){

	uint8_t regVal;
	uint8_t DevAddress = DataStruct->Address;

	/* get the register content */
	if(HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	/* Config gyroscope : modify only required fields*/
	regVal = (regVal & 0b11111000) | ((uint8_t) bw);

	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return SD_MPU6050_Result_Error;
	}

	/*Assert the written value*/
	 if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return SD_MPU6050_Result_Error;
	 }


	if ( (regVal & 0b00000111) == ((uint8_t) bw) ){
	/* Return OK */
	return SD_MPU6050_Result_Ok;}
	else{
		// return unknown error
		return SD_MPU6050_Result_Error;
	}
}

SD_MPU6050_Result SD_MPU6050_ReadAccelerometer(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
{
	uint8_t data[6];
	uint8_t DevAddress = DataStruct->Address;

	/* Read accelerometer data */
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_XOUT_H, 6, data, 6, 1000)){
		return SD_MPU6050_Result_Error;
	}

	/* Format */
	DataStruct->Accelerometer_X = ((float)(data[0] << 8 | data[1]) * DataStruct->Acce_Mult) - DataStruct->Acce_Calibrate[0];
	DataStruct->Accelerometer_Y = ((float)(data[2] << 8 | data[3]) * DataStruct->Acce_Mult) - DataStruct->Acce_Calibrate[1];
	DataStruct->Accelerometer_Z = ((float)(data[4] << 8 | data[5]) * DataStruct->Acce_Mult) - DataStruct->Acce_Calibrate[2];

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
SD_MPU6050_Result SD_MPU6050_ReadGyroscope(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
{
	uint8_t data[6];
	uint8_t DevAddress = DataStruct->Address;

	/* Read gyroscope data */
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_GYRO_XOUT_H, 6, data, 6, 1000)){
		return SD_MPU6050_Result_Error;
	}

	/* Format */
	DataStruct->Gyroscope_X = ((float)(data[0] << 8 | data[1]) * DataStruct->Gyro_Mult)  - DataStruct->Gyro_Calibrate[0];
	DataStruct->Gyroscope_Y = ((float)(data[2] << 8 | data[3]) * DataStruct->Gyro_Mult)  - DataStruct->Gyro_Calibrate[1];
	DataStruct->Gyroscope_Z = ((float)(data[4] << 8 | data[5]) * DataStruct->Gyro_Mult)  - DataStruct->Gyro_Calibrate[2];

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
SD_MPU6050_Result SD_MPU6050_ReadTemperature(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
{
	uint8_t data[2];
	int16_t temp;
	uint8_t DevAddress = DataStruct->Address;

	/* Read temperature */
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_TEMP_OUT_H, 2, data, 2, 1000)){
		return SD_MPU6050_Result_Error;
	}

	/* Format temperature */
	temp = (data[0] << 8 | data[1]);
	DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
SD_MPU6050_Result SD_MPU6050_ReadAll(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
{


	if  (SD_MPU6050_ReadAccelerometer(I2Cx, DataStruct))
	{
		return SD_MPU6050_Result_Error;
	}

	if (SD_MPU6050_ReadGyroscope(I2Cx, DataStruct))
	{
		return SD_MPU6050_Result_Error;
	}

	if (SD_MPU6050_ReadTemperature(I2Cx, DataStruct))
	{
		return SD_MPU6050_Result_Error;
	}

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}

SD_MPU6050_Result SD_MPU6050_Calibrate (I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct , uint32_t interval){


	SD_MPU6050 Calib_Struct = *DataStruct;

	for (int i = 0; i < 3; ++i) {

		Calib_Struct.Acce_Calibrate[i] = 0.0f;
		Calib_Struct.Gyro_Calibrate[i] = 0.0f;

		DataStruct->Acce_Calibrate[i] = 0.0f;
		DataStruct->Gyro_Calibrate[i] = 0.0f;
	}


	for (int var = 0; var < interval; ++var) {

		if (SD_MPU6050_ReadAll(I2Cx, &Calib_Struct) != SD_MPU6050_Result_Ok)
			return SD_MPU6050_Result_Error;

		DataStruct->Acce_Calibrate[0] += Calib_Struct.Accelerometer_X;
		DataStruct->Acce_Calibrate[1] += Calib_Struct.Accelerometer_Y;
		DataStruct->Acce_Calibrate[2] += Calib_Struct.Accelerometer_Z;


		DataStruct->Gyro_Calibrate[0] += Calib_Struct.Gyroscope_X;
		DataStruct->Gyro_Calibrate[1] += Calib_Struct.Gyroscope_Y;
		DataStruct->Gyro_Calibrate[2] += Calib_Struct.Gyroscope_Z;

		HAL_Delay(10);
	}

	for (int i = 0; i < 3; ++i) {

		DataStruct->Acce_Calibrate[i] /= (float) interval;

		DataStruct->Gyro_Calibrate[i] /= (float) interval;

	}
	return SD_MPU6050_Result_Ok;
}

SD_MPU6050_Result SD_MPU6050_EnableInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU6050_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &temp, 14, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU6050_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
SD_MPU6050_Result SD_MPU6050_DisableInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct)
{
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x00};
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	/* Disable interrupts */
	while(HAL_I2C_Master_Transmit(Handle,(uint16_t)address,reg,2,1000)!=HAL_OK);
	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
SD_MPU6050_Result SD_MPU6050_ReadInterrupts(I2C_HandleTypeDef* I2Cx,SD_MPU6050* DataStruct, SD_MPU6050_Interrupt* InterruptsStruct)
{
	uint8_t read;

	/* Reset structure */
	InterruptsStruct->Status = 0;
	uint8_t reg = MPU6050_INT_STATUS;
	I2C_HandleTypeDef* Handle = I2Cx;
	uint8_t address = DataStruct->Address;

	while(HAL_I2C_Master_Transmit(Handle, (uint16_t)address, &reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, (uint16_t)address, &read, 14, 1000) != HAL_OK);

	/* Fill value */
	InterruptsStruct->Status = read;
	/* Return OK */
	return SD_MPU6050_Result_Ok;
}
