/*
 * sd_hal_mpu6050.h
 *
 *  Created on: Feb 19, 2016
 *      Author: Sina Darvishi
 */

#ifndef DRIVERS_MYLIB_SD_HAL_MPU6050_H_
#define DRIVERS_MYLIB_SD_HAL_MPU6050_H_

/*
 C++ detection
#ifdef __cplusplus
extern "C" {
#endif
*/

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_i2c.h"

/**
 * @defgroup SD_MPU6050_Macros
 * @brief    Library defines
 * @{
 */

/* Default I2C used */
//#ifndef MPU6050_I2C
//#define	MPU6050_I2C                    I2C1              /*!< Default I2C */
//#define MPU6050_I2C_PINSPACK           SD_I2C_PinsPack_1 /*!< Default I2C pinspack. Check @ref SD_I2C for more information */
//#endif

/* Default I2C clock */
#ifndef MPU6050_I2C_CLOCK
#define MPU6050_I2C_CLOCK              400000            /*!< Default I2C clock speed */
#endif

/**
 * @brief  Data rates while LPF is OFF predefined constants
 * @{
 */
#define MPU6050_DataRate_8KHz       0   /*!< Sample rate set to 8 kHz */
#define MPU6050_DataRate_4KHz       1   /*!< Sample rate set to 4 kHz */
#define MPU6050_DataRate_2KHz       3   /*!< Sample rate set to 2 kHz */
#define MPU6050_DataRate_1KHz       7   /*!< Sample rate set to 1 kHz */
#define MPU6050_DataRate_500Hz      15  /*!< Sample rate set to 500 Hz */
#define MPU6050_DataRate_250Hz      31  /*!< Sample rate set to 250 Hz */
#define MPU6050_DataRate_125Hz      63  /*!< Sample rate set to 125 Hz */
#define MPU6050_DataRate_100Hz      79  /*!< Sample rate set to 100 Hz */
/**
 * @}
 */



/**
 * @}
 */

/**
 * @defgroup SD_MPU6050_Typedefs
 * @brief    Library Typedefs
 * @{
 */

/**
 * @brief  MPU6050 can have 2 different slave addresses, depends on it's input AD0 pin
 *         This feature allows you to use 2 different sensors with this library at the same time
 */
typedef enum  {
	MPU6050_Device_0 = 0x00, /*!< AD0 pin is set to low */
	MPU6050_Device_1 = 0x02  /*!< AD0 pin is set to high */
} MPU6050_Device;

/**
 * @brief  MPU6050 result enumeration
 */
typedef enum  {
	MPU6050_Result_Ok = 0x00,          /*!< Everything OK */
	MPU6050_Result_Error,              /*!< Unknown error */
	MPU6050_Result_DeviceNotConnected, /*!< There is no device with valid slave address */
	MPU6050_Result_DeviceInvalid       /*!< Connected device with address is not MPU6050 */
} MPU6050_Result;

/**
 * @brief  Parameters for accelerometer range
 */
typedef enum  {
	MPU6050_Accelerometer_2G = 0x00, /*!< Range is +- 2G */
	MPU6050_Accelerometer_4G = 0x01, /*!< Range is +- 4G */
	MPU6050_Accelerometer_8G = 0x02, /*!< Range is +- 8G */
	MPU6050_Accelerometer_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_Accelerometer;

/**
 * @brief  Parameters for gyroscope range
 */
typedef enum {
	MPU6050_Gyroscope_250s = 0x00,  /*!< Range is +- 250 degrees/s */
	MPU6050_Gyroscope_500s = 0x01,  /*!< Range is +- 500 degrees/s */
	MPU6050_Gyroscope_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
	MPU6050_Gyroscope_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_Gyroscope;

/**
 * @brief  Parameters for filter bandwidth
 */


typedef enum {

  MPU6050_DLPF_BW_256  = 0x00,
  MPU6050_DLPF_BW_188,
  MPU6050_DLPF_BW_98,
  MPU6050_DLPF_BW_42,
  MPU6050_DLPF_BW_20,
  MPU6050_DLPF_BW_10,
  MPU6050_DLPF_BW_5,

}MPU6050_DLPF_BW;


/**
 * @brief  Main MPU6050 structure
 */
typedef struct  {

	/* Private */

	uint8_t Address;           /*!< I2C address of device. */

	/* Public */

	int16_t Accelerometer_X; /*!< Accelerometer value X axis */
	int16_t Accelerometer_Y; /*!< Accelerometer value Y axis */
	int16_t Accelerometer_Z; /*!< Accelerometer value Z axis */

	int16_t Gyroscope_X;     /*!< Gyroscope value X axis */
	int16_t Gyroscope_Y;     /*!< Gyroscope value Y axis */
	int16_t Gyroscope_Z;     /*!< Gyroscope value Z axis */

	float   Temperature;   /*!< Temperature in degrees */

} MPU6050;

/**
 * @brief  Interrupts union and structure
 */
typedef union {
	struct {
		uint8_t DataReady:1;       /*!< Data ready interrupt */
		uint8_t reserved2:2;       /*!< Reserved bits */
		uint8_t Master:1;          /*!< Master interrupt. Not enabled with library */
		uint8_t FifoOverflow:1;    /*!< FIFO overflow interrupt. Not enabled with library */
		uint8_t reserved1:1;       /*!< Reserved bit */
		uint8_t MotionDetection:1; /*!< Motion detected interrupt */
		uint8_t reserved0:1;       /*!< Reserved bit */
	} Flags;
	uint8_t Status;
} MPU6050_Interrupt;


/**
 * @}
 */

/**
 * @defgroup SD_MPU6050_Functions
 * @brief    Library Functions
 * @{
 */

/**
 * @brief  Initializes MPU6050 and I2C peripheral
 * @param  *DataStruct: Pointer to empty @ref SD_MPU6050_t structure
 * @param  DeviceNumber: MPU6050 has one pin, AD0 which can be used to set address of device.
 *          This feature allows you to use 2 different sensors on the same board with same library.
 *          If you set AD0 pin to low, then this parameter should be SD_MPU6050_Device_0,
 *          but if AD0 pin is high, then you should use SD_MPU6050_Device_1
 *
 *          Parameter can be a value of @ref SD_MPU6050_Device_t enumeration
 * @param  AccelerometerSensitivity: Set accelerometer sensitivity. This parameter can be a value of @ref SD_MPU6050_Accelerometer_t enumeration
 * @param  GyroscopeSensitivity: Set gyroscope sensitivity. This parameter can be a value of @ref SD_MPU6050_Gyroscope_t enumeration
 * @retval Initialization status:
 *            - SD_MPU6050_Result_t: Everything OK
 *            - Other member: in other cases
 */
MPU6050_Result MPU6050_Init(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Device DeviceNumber, MPU6050_Accelerometer AccelerometerSensitivity, MPU6050_Gyroscope GyroscopeSensitivity, uint8_t DataRate);

/**
 * @brief  Sets gyroscope sensitivity
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  GyroscopeSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU6050_Gyroscope_t enumeration
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_setFullScaleGyroRange(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Gyroscope GyroscopeSensitivity);

/**
 * @brief  Sets accelerometer sensitivity
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  AccelerometerSensitivity: Gyro sensitivity value. This parameter can be a value of @ref SD_MPU6050_Accelerometer_t enumeration
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_setFullScaleAccelRange(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Accelerometer AccelerometerSensitivity);

/**
 * @brief  Sets output data rate
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  rate: Data rate value. An 8-bit value for prescaler value
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_SetDataRate(I2C_HandleTypeDef* I2Cx, MPU6050* DataStruct, uint8_t rate);

/**
 * @brief  Sets low pass filter bandwidth
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  bandwidth: Bandwidth value. A Value of SD_MPU6050_Bandwidth enum
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_SetLowPassFilter(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_DLPF_BW bw);


/**
 * @brief  Enables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_EnableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

/**
 * @brief  Disables interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_DisableInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

/**
 * @brief  Reads and clears interrupts
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure indicating MPU6050 device
 * @param  *InterruptsStruct: Pointer to @ref SD_MPU6050_Interrupt_t structure to store status in
 * @retval Member of @ref SD_MPU6050_Result_t enumeration
 */
MPU6050_Result MPU6050_ReadInterrupts(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct, MPU6050_Interrupt* InterruptsStruct);

/**
 * @brief  Reads accelerometer data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result MPU6050_getAcceleration(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

/**
 * @brief  Reads gyroscope data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result MPU6050_getRotation(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

/**
 * @brief  Reads temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result MPU6050_getTemperature(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);

/**
 * @brief  Reads accelerometer, gyroscope and temperature data from sensor
 * @param  *DataStruct: Pointer to @ref SD_MPU6050_t structure to store data to
 * @retval Member of @ref SD_MPU6050_Result_t:
 *            - SD_MPU6050_Result_Ok: everything is OK
 *            - Other: in other cases
 */
MPU6050_Result MPU6050_ReadAll(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);


/**
 * @brief  : get the scale factor that multiplied by the raw data to calculate the deg/s value
 * @retval return_type : scale factor
 * @note
 */
float    MPU6050_getGyroScale(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);
/**
 * @brief  : get the scale factor that multiplied by the raw data to calculate the (g) value
 * @retval return_type : scale factor
 * @note
 */
float    MPU6050_getAccelScale(I2C_HandleTypeDef* I2Cx,MPU6050* DataStruct);
/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */


#endif /* DRIVERS_MYLIB_SD_HAL_MPU6050_H_ */
