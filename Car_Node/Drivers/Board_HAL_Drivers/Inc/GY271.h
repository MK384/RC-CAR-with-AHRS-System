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

	GY271_Range_0_9Ga,
	GY271_Range_1_3Ga,			/*!< For magnetic clear environment*/
	GY271_Range_1_9Ga,			/*!< For magnetic clear environment*/
	GY271_Range_2_5Ga,			/*!< For magnetic clear environment*/
	GY271_Range_4_0Ga,			/*!< For magnetic crowded environment*/
	GY271_Range_4_7Ga,			/*!< For magnetic crowded environment*/
	GY271_Range_5_6Ga,
	GY271_Range_8_1Ga

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
 * @defgroup GY271 Gain LSB define list
 * @{
 */


#define 		GY271_GAIN_LSB_0_9Ga			((float)1370)
#define 		GY271_GAIN_LSB_1_3Ga			((float)1090)
#define 		GY271_GAIN_LSB_1_9Ga			((float)820)
#define 		GY271_GAIN_LSB_2_5Ga			((float)660)
#define 		GY271_GAIN_LSB_4_0Ga			((float)440)
#define 		GY271_GAIN_LSB_4_7Ga			((float)390)
#define 		GY271_GAIN_LSB_5_6Ga			((float)330)
#define 		GY271_GAIN_LSB_8_1Ga			((float)230)





/**
 * @}
 */

/**
 * @defgroup 		Output measurement units
 * @{
 */
#define		GY271_UNIT_GAUSS				((float) 1.0    )
#define		GY271_UNIT_MILLI_GAUSS			((float) 1000.0 )
#define		GY271_UNIT_TESLA				((float) 0.0001 )
#define		GY271_UNIT_MICRO_TESLA			((float) 100.0  )
#define		GY271_UNIT_MICRO_TESLA_PER_50	((float) 2.0    )
/**
 * @}
 */



/**
 * @defgroup  GY271 Data structure
 * @{
 */

typedef struct{

	I2C_HandleTypeDef* I2Cx;
	GY271_ModeControl mode;
	GY271_DataRate dataRate;
	GY271_FieldRange range;
	GY271_SampleAvgRate samplesRate;
	uint8_t dataReadyFlag;
	float   outputUnit;

	int16_t Compass_Raw[3];
	float   Compass_Xyz[3];			/* Measurements in uT*/



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
GY271_Result GY271_Init(GY271 * dataStruct);




/**
 * @brief  GY271_ReadData :  read the measured data
 * @param  dataStruct : a struct that the data will be written to.
 * @retval GY271_Result : status of the read operation.
 * @note
 */

GY271_Result GY271_getData( GY271* dataStruct);


/**
 * @brief :	 check if data is ready
 * @param :	 GY271 struct
 * @retval GY271_Result : Result_ok if operation succeeded
 * @note
 */


GY271_Result GY271_getReadyFlag(GY271* dataStruct);

/**
 * @}
 */




#endif /* BOARD_HAL_DRIVERS_INC_GY271_H_ */
