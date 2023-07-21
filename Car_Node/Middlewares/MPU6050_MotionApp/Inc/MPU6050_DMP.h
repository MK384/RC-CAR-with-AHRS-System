/*
 * MPU6050_DMP.h
 *
 *  	Created on: Jul 19, 2023
 *      Author: Mohammed Khaled <Mohammed.Kh384@gmail.com>
 *
 * This file contains the function declaration of the Digital Motion Processing (DMP) functionality for the MPU6050 sensor using the I2Cdev library.
 * The DMP is a utility function that assists in motion and orientation tracking by offloading some processing from the host Microcontroller.
 * As before, the original library by "Jeff Rowberg" was adapted for STM32 and rewritten in C for reduced memory usage.
 * The purpose of this file is to handle the DMP functionality provided by the MPU6050 sensor. It includes functions such as:
 * By separating the DMP-related code into its own file, it improves code organization and allows for easier maintenance and modification
 * of this specific functionality.
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

#ifndef MPU6050_MOTIONAPP_INC_MPU6050_DMP_H_
#define MPU6050_MOTIONAPP_INC_MPU6050_DMP_H_


/* Includes ------------------------------------------------------------------*/
#include	"MPU6050_Basic.h"
#include	"MPU6050_Advanced.h"
#include 	"MPU6050_Defs.h"
#include    "malloc.h"





/* User Config defines --------------------------------------------------------*/

/** @defgroup DMP_Exported_Types
 * @{
 */
/* Exported constants --------------------------------------------------------*/

#define DMP_NUM_AXES    3
#define DMP_QNUM_AXES   4

#define DMP_DEG_OF_FREEDOM 			6

/**
 * @defgroup  MPU6050_DMP struct that holds the interpreted data from the DMP packet
 * @{
 */

typedef struct {

	  MPU6050* ptrMPU;							  /* pointer to MPU6050 configuration struct that used to initialize the device */
	  float orientation[DMP_NUM_AXES];            /* 9 or 6 axes roll, pitch and yaw */
	  float quaternion[DMP_QNUM_AXES];       	  /* 9 or 6 axes quaternion */
	  float gravity[DMP_NUM_AXES];             	  /* 9 or 6 axes device frame gravity */
	  float linear_acceleration [DMP_NUM_AXES];   /* 9 or 6 axes device frame linear acceleration */
	  float heading;                              /* 9 or 6 axes heading */

} MPU6050_DMP;


/**
 * @}
 */

/**
 * @defgroup Function declaration
 * @{
 */

/**
 * @brief  DMP_Initialize : Initialize the Digital Motion Processing Unit (DMP)
 * @param   : None
 * @retval uint8_t : 0 if success operation
 * 					 1 if DMP code verification failed.
 * 					 2 if DMP configuration verification failed.
 * @note	if DMP is to be used you have to initialize the operation of MPU6050 sensors first @see MPU6050.h
 */
uint8_t MPU6050_DMP_Initialize(void);


/**
 * @brief  MPU6050_DMP_setEnable : Enable the the DMP unit in the MPU6050.
 * @param  enabled :	true  :  enable DMP.
 * 					 	false :  disable DMP.
 * @retval None :
 * @note		This function should be called after DMP_Initialize()
 */
void MPU6050_DMP_setEnable(bool enabled);


/**
 * @brief  MPU6050_DMP_setIntEnable :  Enable or disable the DMP Interrupts.
 * @param  enable :			true  :  enable DMP Interrupt,
 * 					 		false :  disable DMP Interrupt.
 * @retval None :
 * @note
 */
void MPU6050_DMP_setIntEnable(bool enable);


/**
 * @brief  MPU6050_DMP_getIntStatus :  get the current state of the DMP Interrupt.
 * @param  None :
 * @retval bool :			true  :   DMP Interrupt is set.
 * 					 		false :  disable DMP Interrupt is not set.
 * @note
 */
bool MPU6050_DMP_getIntStatus(void);


/**
 * @brief  MPU6050_DMP_IsReady : check if there is any packets ready to be processed
 * @param  None :
 * @retval return_type :			true  :   DMP Packet is ready to process.
 * 					 				false :   DMP Packet is not ready to process.
 * @note	you can check this function periodically if the interrupt is disabled
 */
bool MPU6050_DMP_IsReady(void);

/**
 * @brief  MPU6050_DMP_getOrientation :  updates the quaternion values of the passing struct
 * @param  dmpStruct   :  A pointer to DMP struct
 * @retval bool		   :  true if the value has updated , false if not
 * @note
 */
bool MPU6050_DMP_getQuaternion(MPU6050_DMP* dmpStruct);

/**
 * @brief  MPU6050_DMP_getOrientation :  updates the quaternion and orientation values of the passing struct
 * @param  dmpStruct   :  A pointer to DMP struct
 * @retval bool		   :  true if the value has updated , false if not
 * @note
 */
bool MPU6050_DMP_getOrientation(MPU6050_DMP* dmpStruct);


/**
 * @brief  MPU6050_DMP_getGravity :  updates the quaternion and gravity values of the passing struct
 * @param  dmpStruct   :  A pointer to DMP struct
 * @retval bool		   :  true if the value has updated , false if not
 * @note
 */
bool MPU6050_DMP_getGravity(MPU6050_DMP* dmpStruct);

/**
 * @brief  MPU6050_DMP_getAcceleration :  updates the quaternion and linear acceleration values of the passing struct
 * @param  dmpStruct   :  A pointer to DMP struct
 * @retval bool		   :  true if the value has updated , false if not
 * @note
 */
bool MPU6050_DMP_getAcceleration(MPU6050_DMP* dmpStruct);

/**
 * @brief  MPU6050_DMP_getHeading :  updates the quaternion and Heading values of the passing struct
 * @param  dmpStruct   :  A pointer to DMP struct
 * @retval bool		   :  true if the value has updated , false if not
 * @note
 */
bool MPU6050_DMP_getAHeading(MPU6050_DMP* dmpStruct);


/**
 * @brief  MPU6050_DMP_getHeading :  updates all the Motion type values of the passing struct
 * @param  dmpStruct   :  A pointer to DMP struct
 * @retval bool		   :  true if the value has updated , false if not
 * @note
 */
bool MPU6050_DMP_getAll(MPU6050_DMP* dmpStruct);


/**
 * @}
 */













#endif /* MPU6050_MOTIONAPP_INC_MPU6050_DMP_H_ */
