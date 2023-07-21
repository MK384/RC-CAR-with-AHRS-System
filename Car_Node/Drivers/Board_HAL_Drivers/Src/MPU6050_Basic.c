/**
 * File : MPU6050_Basic.c
 */
/**
 *  Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
 *  Created on: Jul 18, 2023, By "Mohammed Khaled" <Mohammed.kh384@gmail.com>
 *
 * This file contains the declaration of basic sensor operations for the MPU6050 IMU (Inertial Measurement Unit)
 *  using the I2Cdev library. It provides functions for configuring the sensor and retrieving raw data from it.
 *  The original library, created by Jeff Rowberg, was primarily designed for the Arduino platform and written in C++.
 *   However, you have modified the library to work with STM32 and converted the code to C for reduced memory footprint.
 * The purpose of this file is to encapsulate the fundamental functionality required to interact with the MPU6050 sensor
 *
 * The original code in I2Cdev library was partially refactored and grouped in Three files:
 *
 * (1) MPU6050_Basic (.h /.c) :
 * 		contains the implementation of basic sensor operations for the MPU6050 IMU ,
 * 		It provides functions for configuring the sensor and retrieving raw data from it.
 *
 *
 * (2) MPU6050_Advanced (.h /.c) :
 * 		contains the implementation of more advanced features for the MPU6050 sensor using the I2Cdev library.
 * 		It includes functions for dealing with slave external sensors and MPU I2C master operations.
 *
 * 	(3) MPU6050_DMP (.h /.c) :
 * 		 contains the implementation of the Digital Motion Processing (DMP) functionality for the MPU6050 sensor using the I2Cdev library.
 *
 *  In Addition to the dependency file I2cdev based on STM32 HAL Library.
 *
 */

#include <MPU6050_Basic.h>



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

#define 	REG_SIZE			 	(1U)   // 1 byte

/**
 * @defgroup static function definition
 * @{
 */
inline static float MPU6050_getGyroScale(MPU6050* ptrConfigStruct);

inline static float MPU6050_getAccelScale(MPU6050* ptrConfigStruct);

static I2C_HandleTypeDef* I2Cx;
static uint8_t DevAddress;
/**
 * @}
 */

/**
 * @defgroup  Functions Implementation
 * @{
 */


MPU6050_Result MPU6050_Init(MPU6050* ptrConfigStruct)
{

	uint8_t regVal;
	I2Cx = ptrConfigStruct->I2Cx;
	/* Format I2C address */
	 DevAddress = ptrConfigStruct->Address;

	/* Check if device is connected */
	if(HAL_I2C_IsDeviceReady(I2Cx,DevAddress, 3 ,100))
	{
				return MPU6050_Result_DeviceNotConnected;
	}
	/* Check who am I */
	//------------------
		if(HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_WHO_AM_I,	 REG_SIZE, &regVal, REG_SIZE, 1000))
		{
			return MPU6050_Result_Error;
		}

		/* Checking */
		if (regVal != I_AM_MPU6050)
		{
				/* Return error */
				return MPU6050_Result_DeviceInvalid;
		}
	//------------------------------------------------------------------------------------

	/* Wakeup MPU6050 */
	//------------------
		/* Format array to send */
		 regVal = 0x00;

		/* Try to transmit via I2C */
		if(HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_PWR_MGMT_1, REG_SIZE, &regVal, REG_SIZE, 1000))
		{
					return MPU6050_Result_Error;
		}
//------------------
	/* Config Digital Low Pass Filter  */
	if (MPU6050_SetLowPassFilter(ptrConfigStruct, ptrConfigStruct->LowPassFilter))
	{
				return MPU6050_Result_Error;
	}
	/* config sample rate  */
	if (MPU6050_SetDataRate(ptrConfigStruct, ptrConfigStruct->DataRate))
	{
				return MPU6050_Result_Error;
	}
	/* Config accelerometer */
	if (MPU6050_setFullScaleAccelRange(ptrConfigStruct, ptrConfigStruct->AccelerometerRange))
	{
				return MPU6050_Result_Error;
	}

	/* Config Gyroscope */
	if(MPU6050_setFullScaleGyroRange(ptrConfigStruct, ptrConfigStruct->GyroscopeRange))
	{
				return MPU6050_Result_Error;
	}
	/*Config Interrupts*/
	switch (ptrConfigStruct->interruptState) {
		case MPU6050_Interrupt_Disabled:
			if(MPU6050_DisableInterrupts(ptrConfigStruct))
			{
						return MPU6050_Result_Error;
			}
			break;
		case MPU6050_Interrupt_Enabled:
			if (MPU6050_EnableInterrupts(ptrConfigStruct))
			{
						return MPU6050_Result_Error;
			}
		default:
			break;
	}

	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_SetDataRate( MPU6050* ptrConfigStruct, MPU6050_DataRate rate)
{

	/**
	 * The Sample Rate is generated by dividing the gyroscope output rate by SMPLRT_DIV:
							Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
			where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7), and 1kHz
			when the DLPF is enabled (see Register 26).
	 */
	uint8_t regVal;
	/* Format array to send */

	if( (ptrConfigStruct->LowPassFilter != MPU6050_DLPF_OFF) && (ptrConfigStruct->LowPassFilter != MPU6050_DLPF_Disable) ){

		switch (rate) {
			case MPU6050_DataRate_8KHz:
			case MPU6050_DataRate_4KHz:
			case MPU6050_DataRate_2KHz:
			case MPU6050_DataRate_1KHz:

				rate = 0x00;
				break;
			case MPU6050_DataRate_500Hz:
				rate = 0x01;
				break;
			case MPU6050_DataRate_250Hz:
				rate = 0x03;
				break;
			case MPU6050_DataRate_125Hz:
				rate = 0x07;
				break;
			case MPU6050_DataRate_100Hz:
				rate = 0x09;
				break;
			default:
				rate = 0x00;
				break;
		}
	}


		regVal =  rate;

	/* Set data sample rate */
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_SMPLRT_DIV, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return MPU6050_Result_Error;
	}

		/* Assert value*/
	regVal = 0x00;
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_SMPLRT_DIV, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return MPU6050_Result_Error;
	 }

	if ( regVal == rate ) return MPU6050_Result_Ok;
	else return MPU6050_Result_Error;

}

MPU6050_Result MPU6050_setFullScaleAccelRange(MPU6050* ptrConfigStruct, MPU6050_Accelerometer AccelerometerRange)
{
	uint8_t regVal;

	/* Config accelerometer : modify only required fields*/
	regVal = (uint8_t) (AccelerometerRange << 3);

	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_ACCEL_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return MPU6050_Result_Error;
	}


	/*Assert the written value*/
	regVal = 0x00;
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return MPU6050_Result_Error;
	 }

	if (regVal == (uint8_t) (AccelerometerRange << 3) ){
		ptrConfigStruct->AccelerometerRange = AccelerometerRange;
		ptrConfigStruct->Accel_Scale = MPU6050_getAccelScale(ptrConfigStruct);
		// Return OK
		return MPU6050_Result_Ok;
	}else{
		// return unknown error
		return MPU6050_Result_Error;
	}

}

MPU6050_Result MPU6050_setFullScaleGyroRange(MPU6050* ptrConfigStruct, MPU6050_Gyroscope GyroscopeRange)
{
	uint8_t regVal;

	/* Config gyroscope : modify only required fields*/
	regVal =  (uint8_t) (GyroscopeRange << 3);
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_GYRO_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return MPU6050_Result_Error;
	}

	/*Assert the written value*/
	regVal = 0x00;
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_GYRO_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return MPU6050_Result_Error;
	 }

	if (regVal  == (uint8_t) (GyroscopeRange << 3) ){
		ptrConfigStruct->GyroscopeRange = GyroscopeRange;
		ptrConfigStruct->Gyro_Scale = MPU6050_getGyroScale(ptrConfigStruct);
		// Return OK
		return MPU6050_Result_Ok;
	}else{
		// return unknown error
		return MPU6050_Result_Error;
	}
}

MPU6050_Result MPU6050_SetLowPassFilter(MPU6050* ptrConfigStruct, MPU6050_DLPF filter){

	uint8_t regVal;

	/* get the register content */
	if(HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return MPU6050_Result_Error;
	}

	/* Config gyroscope : modify only required fields*/
	regVal = (regVal & 0b11111000) | ((uint8_t) filter);

	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return MPU6050_Result_Error;
	}

	/*Assert the written value*/
	 if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000))
	 {
			return MPU6050_Result_Error;
	 }


	if ( (regVal & 0b00000111) == ((uint8_t) filter) ){
	ptrConfigStruct->LowPassFilter = filter;
	/* Return OK */
	return MPU6050_Result_Ok;

	}
	else{
		// return unknown error
		return MPU6050_Result_Error;
	}
}


MPU6050_Result MPU6050_getAcceleration(MPU6050* ptrConfigStruct)
{
	uint8_t data[6];

	/* Read accelerometer data */
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_XOUT_H, 6, data, 6, 1000)){
		return MPU6050_Result_Error;
	}

	/* Format */
	ptrConfigStruct->Accel_RawData[0] = (data[0] << 8 | data[1]) ;
	ptrConfigStruct->Accel_RawData[1] = (data[2] << 8 | data[3]) ;
	ptrConfigStruct->Accel_RawData[2] = (data[4] << 8 | data[5]) ;

	ptrConfigStruct->Accel_Xyz[0] = ptrConfigStruct->Accel_RawData[0] * ptrConfigStruct->Accel_Scale;
	ptrConfigStruct->Accel_Xyz[1] = ptrConfigStruct->Accel_RawData[1] * ptrConfigStruct->Accel_Scale;
	ptrConfigStruct->Accel_Xyz[2] = ptrConfigStruct->Accel_RawData[2] * ptrConfigStruct->Accel_Scale;
	/* Return OK */
	return MPU6050_Result_Ok;
}

MPU6050_Result MPU6050_getRotation(MPU6050* ptrConfigStruct)
{
	uint8_t data[6];

	/* Read gyroscope data */
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_GYRO_XOUT_H, 6, data, 6, 1000)){
		return MPU6050_Result_Error;
	}

	/* Format */
	ptrConfigStruct->Gyro_RawData[0] = (data[0] << 8 | data[1]);
	ptrConfigStruct->Gyro_RawData[1] = (data[2] << 8 | data[3]);
	ptrConfigStruct->Gyro_RawData[2] = (data[4] << 8 | data[5]);


	ptrConfigStruct->Gyro_Xyz[0] = ptrConfigStruct->Gyro_RawData[0] * ptrConfigStruct->Gyro_Scale;
	ptrConfigStruct->Gyro_Xyz[1] = ptrConfigStruct->Gyro_RawData[1] * ptrConfigStruct->Gyro_Scale;
	ptrConfigStruct->Gyro_Xyz[2] = ptrConfigStruct->Gyro_RawData[2] * ptrConfigStruct->Gyro_Scale;

	/* Return OK */
	return MPU6050_Result_Ok;
}
MPU6050_Result MPU6050_getTemperature(MPU6050* ptrConfigStruct){
	uint8_t data[2];

	/* Read temperature */
	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_TEMP_OUT_H, 2, data, 2, 1000)){
		return MPU6050_Result_Error;
	}

	/* Format temperature */
	ptrConfigStruct->Temperature = (float) (((float)(data[0] << 8 | data[1]))/(float)376.53);

	/* Return OK */
	return MPU6050_Result_Ok;
}


MPU6050_Result MPU6050_ReadAll(MPU6050* ptrConfigStruct)
{

	uint8_t data[14];

	/* Read full raw data, 14bytes */

	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_XOUT_H, 1, data, 14, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Format accelerometer data */
	ptrConfigStruct->Accel_RawData[0] = (data[0] << 8 | data[1]) ;
	ptrConfigStruct->Accel_RawData[1] = (data[2] << 8 | data[3]) ;
	ptrConfigStruct->Accel_RawData[2] = (data[4] << 8 | data[5]) ;

	ptrConfigStruct->Accel_Xyz[0] = ptrConfigStruct->Accel_RawData[0] * ptrConfigStruct->Accel_Scale;
	ptrConfigStruct->Accel_Xyz[1] = ptrConfigStruct->Accel_RawData[1] * ptrConfigStruct->Accel_Scale;
	ptrConfigStruct->Accel_Xyz[2] = ptrConfigStruct->Accel_RawData[2] * ptrConfigStruct->Accel_Scale;

	/* Format temperature */
	ptrConfigStruct->Temperature = (float) (((float)(data[6] << 8 | data[7]))/(float)376.53);

	/* Format gyroscope data */
	ptrConfigStruct->Gyro_RawData[0] = (data[8] << 8 | data[9]);
	ptrConfigStruct->Gyro_RawData[1] = (data[10] << 8 | data[11]);
	ptrConfigStruct->Gyro_RawData[2] = (data[12] << 8 | data[13]);


	ptrConfigStruct->Gyro_Xyz[0] = ptrConfigStruct->Gyro_RawData[0] * ptrConfigStruct->Gyro_Scale;
	ptrConfigStruct->Gyro_Xyz[1] = ptrConfigStruct->Gyro_RawData[1] * ptrConfigStruct->Gyro_Scale;
	ptrConfigStruct->Gyro_Xyz[2] = ptrConfigStruct->Gyro_RawData[2] * ptrConfigStruct->Gyro_Scale;

	/* Return OK */
	return MPU6050_Result_Ok;

}


MPU6050_Result MPU6050_EnableInterrupts(MPU6050* ptrConfigStruct)
{
	uint8_t temp;
	uint8_t reg[2] = {MPU6050_INT_ENABLE,0x21};
	I2C_HandleTypeDef* Handle = I2Cx;

	/* Enable interrupts for data ready and motion detect */
	while(HAL_I2C_Master_Transmit(Handle, DevAddress, reg, 2, 1000) != HAL_OK);

	uint8_t mpu_reg= MPU6050_INT_PIN_CFG;
	/* Clear IRQ flag on any read operation */
	while(HAL_I2C_Master_Transmit(Handle, DevAddress, &mpu_reg, 1, 1000) != HAL_OK);

	while(HAL_I2C_Master_Receive(Handle, DevAddress, &temp, 1, 1000) != HAL_OK);
	temp |= 0x10;
	reg[0] = MPU6050_INT_PIN_CFG;
	reg[1] = temp;
	while(HAL_I2C_Master_Transmit(Handle, DevAddress, reg, 2, 1000) != HAL_OK);

	/* Return OK */
	ptrConfigStruct->interruptState = MPU6050_Interrupt_Enabled;
	return MPU6050_Result_Ok;
}


MPU6050_Result MPU6050_DisableInterrupts(MPU6050* ptrConfigStruct)
{
	uint8_t regVal = 0x00;

	/* Disable interrupts */
	if (HAL_I2C_Mem_Write(I2Cx, DevAddress, MPU6050_INT_ENABLE, REG_SIZE, &regVal, REG_SIZE, 1000))
	{
				return MPU6050_Result_Error;
	}
	/* Return OK */
	ptrConfigStruct->interruptState = MPU6050_Interrupt_Disabled;
	return MPU6050_Result_Ok;
}



MPU6050_Result MPU6050_ReadInterrupts(MPU6050* ptrConfigStruct)
{
	uint8_t data;

	/* Reset structure */
	ptrConfigStruct->InterruptFlags.reg = 0x00;

	if (HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_INT_STATUS, 1, &data, 1, 1000) != HAL_OK)
	{
		return MPU6050_Result_Error;
	}

	/* Fill value */
	ptrConfigStruct->InterruptFlags.reg = data;

	/* Return OK */
	return MPU6050_Result_Ok;
}

inline static float MPU6050_getGyroScale(MPU6050* ptrConfigStruct){

	uint8_t regVal ,GyroscopeRange ;

	// read the gyro range

	HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_GYRO_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000);

	GyroscopeRange = (regVal >> 3) & 0x3;

	switch (GyroscopeRange) {
			case MPU6050_Gyroscope_250s:
				return ptrConfigStruct->GyrOutputUnit / MPU6050_GYRO_SENS_250;
				break;
			case MPU6050_Gyroscope_500s:
				return ptrConfigStruct->GyrOutputUnit / MPU6050_GYRO_SENS_500;
				break;
			case MPU6050_Gyroscope_1000s:
				return ptrConfigStruct->GyrOutputUnit / MPU6050_GYRO_SENS_1000;
				break;
			case MPU6050_Gyroscope_2000s:
				return ptrConfigStruct->GyrOutputUnit / MPU6050_GYRO_SENS_2000;
				break;
			default:
				break;
		}

	return (float)(0.0);
}

inline static float MPU6050_getAccelScale(MPU6050* ptrConfigStruct)
{
	uint8_t regVal ,AccelerometerRange ;

	// read the gyro range

	HAL_I2C_Mem_Read(I2Cx, DevAddress, MPU6050_ACCEL_CONFIG, REG_SIZE, &regVal, REG_SIZE, 1000);

	AccelerometerRange = (regVal >> 3) & 0x3;

	switch (AccelerometerRange) {
		case MPU6050_Accelerometer_2G:
			return ptrConfigStruct->AccOutputUnit / MPU6050_ACCE_SENS_2;
			break;
		case MPU6050_Accelerometer_4G:
			return ptrConfigStruct->AccOutputUnit / MPU6050_ACCE_SENS_4;
			break;
		case MPU6050_Accelerometer_8G:
			return ptrConfigStruct->AccOutputUnit / MPU6050_ACCE_SENS_8;
			break;
		case MPU6050_Accelerometer_16G:
			return ptrConfigStruct->AccOutputUnit / MPU6050_ACCE_SENS_16;
			break;
		default:
			break;
		}
	return (float)(0.0);
}
