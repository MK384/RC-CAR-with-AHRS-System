/*
 * GY271.h
 *
 *  Created on: Jul 5, 2023
 *      Author: mohammed khaled
 */

#ifndef BOARD_HAL_DRIVERS_INC_GY271_H_
#define BOARD_HAL_DRIVERS_INC_GY271_H_



#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"


/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif


/**
 * @defgroup GY271 result enumeration
 * @{
 */
typedef enum {

	GY271_Result_Ok,						/*!< Everything OK */
	GY271_Result_Error,						/*!< Unknown error */
	GY271_Result_DeviceNotConnected			/*!< There is no device with valid slave address */


}GY271_Result;
/**
 * @}
 */


/**
 * @defgroup GY271 mode enum
 * @{
 */
typedef enum {

	GY271_Mode_Continuous,						/*!< Continuous Mode */
	GY271_Mode_Single,					     	/*!< Single Mode */
	GY271_Mode_Standby,						    /*!< Standby Mode */



}GY271_ModeControl;

/**
 * @}
 */

/**
 * @defgroup GY271 Output data rate enum
 * @{
 */
typedef enum {

	GY271_DataRate_1_Hz,			/*!< Data generated at rate of 1 Hz*/
	GY271_DataRate_2_Hz,			/*!< Data generated at rate of 2 Hz*/
	GY271_DataRate_3_Hz,			/*!< Data generated at rate of 3 Hz*/
	GY271_DataRate_8_Hz,			/*!< Data generated at rate of 8 Hz*/
	GY271_DataRate_15_Hz,			/*!< Data generated at rate of 15 Hz*/
	GY271_DataRate_30_Hz,			/*!< Data generated at rate of 30 Hz*/
	GY271_DataRate_75_Hz,			/*!< Data generated at rate of 75 Hz*/

}GY271_DataRate;

/**
 * @}
 */

/**
 * @defgroup  GY271 Field range ,  Choose a lower  higher GN# when total field strength causes
 *  overflow in one of the data output registers(saturation)
 * @{
 */
typedef enum {

	GY271_Range_1G = 1,			/*!< For magnetic clear environment*/
	GY271_Range_2G = 2,			/*!< For magnetic clear environment*/
	GY271_Range_3G = 3,			/*!< For magnetic clear environment*/
	GY271_Range_4G = 4,			/*!< For magnetic crowded environment*/
	GY271_Range_8G = 7,			/*!< For magnetic crowded environment*/

}GY271_FieldRange;


/**
 * @}
 */

/**
 * @defgroup GY271 sample avg rate , Select numberof samples averaged (1 to 8) per measurement output
 * @{
 */

typedef enum {

	GY271_SampleAvg_1_S,			/*!< 1 sample is is averaged (no filter) */
	GY271_SampleAvg_2_S,			/*!< 2 sample is is averaged */
	GY271_SampleAvg_4_S,			/*!< 4 sample is is averaged */
	GY271_SampleAvg_8_S				/*!< 8 sample is is averaged */

}GY271_SampleAvgRate;

/**
 * @}
 */


/**
 * @defgroup  GY271 Data structure
 * @{
 */

typedef struct{

	int16_t Compass_X;
	int16_t Compass_Y;
	int16_t Compass_Z;


}GY271;

/**
 * @}
 */


/**
 * @defgroup  GY271 Functions
 * @{
 */




/**
 * @brief  GY271_Init :  Initialize the sensor with the input parameters
 * @param  mode :  Mode control for operation of the sensor
 * @param  dateRate : the rate at which data sample is generated
 * @param  range :  the gaussian range of the device
 * @param  sampleRate : the over sample ratio of the DLPF
 * @retval GY271_Result : GY271_Result_Ok if everything is went right , else if there is a problem
 * @note
 */
GY271_Result GY271_Init(I2C_HandleTypeDef* I2Cx , GY271_ModeControl mode , GY271_DataRate dataRate, GY271_FieldRange range, GY271_SampleAvgRate samplesRate );




/**
 * @brief  GY271_ReadData :  read the measured data
 * @param  dataStruct : a struct that the data will be written to.
 * @retval GY271_Result : status of the read operation.
 * @note
 */

GY271_Result GY271_ReadData(I2C_HandleTypeDef* I2Cx , GY271* dataStruct);


/**
 * @}
 */




#endif /* BOARD_HAL_DRIVERS_INC_GY271_H_ */
