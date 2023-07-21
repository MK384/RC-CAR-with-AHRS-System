
/**
 * File : MPU6050_Advanced.h
 */
/**
 *  Based on InvenSense MPU-6050 register map document rev. 2.0, 5/19/2011 (RM-MPU-6000A-00)
 *  Created on: Jul 18, 2023, By "Mohammed Khaled" <Mohammed.kh384@gmail.com>
 *
 * This file contains the declaration of more advanced features for the MPU6050 sensor using the I2Cdev library.
 * It includes functions for dealing with slave external sensors and MPU I2C master operations.
 * Similar to the previous file, you have converted the original C++ code to C for compatibility
 *  with STM32 and reduced memory usage.
 * The purpose of this file is to provide additional functionality beyond the basic operations
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

#ifndef INV_MOTIONAPP_MOTIONAPP_6X_H_
#define INV_MOTIONAPP_MOTIONAPP_6X_H_

/**
 * @defgroup Includes
 * @{
 */
#include "stdbool.h"

#include "I2Cdev.h"
#include "MPU6050_Basic.h"
#include "MPU6050_Defs.h"


/**
 * @}
 */



/**
 * @defgroup Functions Declaration
 * @{
 */

/** Set gyroscope FIFO enabled value.
 * @param enabled New gyroscope FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
void	MPU6050_setGyroFIFOEnabled(bool enabled );

/** Set accelerometer FIFO enabled value.
 * @param enabled New accelerometer FIFO enabled value
 * @see MPU6050_RA_FIFO_EN
 */
void	MPU6050_setAccFIFOEnabled(bool enabled );

/** Set Slave [0 .. 3] FIFO enabled value.
 * @param enabled New Slave 0 FIFO enabled value
 * @param slaveNum : number of slave to be enabled
 * @see MPU6050_RA_FIFO_EN
 */
void MPU6050_setSlave0FIFOEnabled( uint8_t slaveNum ,bool enabled);

/** Set wait-for-external-sensor-data enabled value.
 * @param enabled New wait-for-external-sensor-data enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void DMP_setWaitForExternalSensorEnabled(bool enabled);

/** Set slave read/write transition enabled value.
 * @param enabled New slave read/write transition enabled value
 * @see MPU6050_RA_I2C_MST_CTRL
 */
void DMP_setSlaveReadWriteTransitionEnabled(bool enabled);

/** Set I2C master clock speed.
 * @reparam speed Current I2C master clock speed
 * @see MPU6050_RA_I2C_MST_CTRL
 *
 *  * <pre>
 * I2C_MST_CLK | I2C Master Clock Speed | 8MHz Clock Divider
 * ------------+------------------------+-------------------
 * 0           | 348kHz                 | 23
 * 1           | 333kHz                 | 24
 * 2           | 320kHz                 | 25
 * 3           | 308kHz                 | 26
 * 4           | 296kHz                 | 27
 * 5           | 286kHz                 | 28
 * 6           | 276kHz                 | 29
 * 7           | 267kHz                 | 30
 * 8           | 258kHz                 | 31
 * 9           | 500kHz                 | 16
 * 10          | 471kHz                 | 17
 * 11          | 444kHz                 | 18
 * 12          | 421kHz                 | 19
 * 13          | 400kHz                 | 20
 * 14          | 381kHz                 | 21
 * 15          | 364kHz                 | 22
 * </pre>
 */
void DMP_setMasterClockSpeed(uint8_t i2C_master_speed);

/**
* @param slaveNum Slave number (0-3)
* @return Current address for specified slave
* @see MPU6050_RA_I2C_SLV0_ADDR
*/
uint8_t DMP_getSlaveAddress(uint8_t slaveNum);

/** Set the I2C address of the specified slave (0-3).
 * @param slaveNum Slave number (0-3)
 * @param address New address for specified slave
 * @see MPU6050_RA_I2C_SLV0_ADDR
 */
void MPU6050_setSlaveAddress(uint8_t slaveNum, uint8_t slaveAddress);

/** Set the active internal register for the specified slave (0-3).
 * Read/write operations for this slave will be done to whatever internal
 * register address is stored in this MPU register.
 *
 * In write mode, the contents of I2C_SLV0_DO (Register 99) will be written to the slave device.
 *
 * The MPU-6050 supports a total of five slaves, but Slave 4 has unique
 * characteristics, and so it has its own functions.
 * @param slaveNum Slave number (0-3)
 * @param reg New active register for specified slave
 * @see MPU6050_RA_I2C_SLV0_REG
 */

void MPU6050_setSlaveRegister(uint8_t slaveNum, uint8_t regAddress);

/** Set the enabled value for the specified slave (0-3).
 * @param slaveNum Slave number (0-3)
 * @param enabled New enabled value for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
void MPU6050_setSlaveEnabled(uint8_t slaveNum, bool enabled);

/** Set word pair byte-swapping enabled for the specified slave (0-3).
 * When set to 1, this bit enables byte swapping. When byte swapping is enabled,
 * the high and low bytes of a word pair are swapped. Please refer to
 * I2C_SLV0_GRP for the pairing convention of the word pairs. When cleared to 0,
 * bytes transferred to and from Slave 0 will be written to EXT_SENS_DATA
 * registers in the order they were transferred.
 *
 *
 *  Set word pair byte-swapping enabled for the specified slave (0-3).
 * @param slaveNum Slave number (0-3)
 * @param enabled New word pair byte-swapping enabled value for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 *
 */

void DMP_setSlaveWordByteSwap(uint8_t slavenum, bool enabled);

/** Set word pair grouping order offset for the specified slave (0-3).
 * This sets specifies the grouping order of word pairs received from registers.
 * When cleared to 0, bytes from register addresses 0 and 1, 2 and 3, etc (even,
 * then odd register addresses) are paired to form a word. When set to 1, bytes
 * from register addresses are paired 1 and 2, 3 and 4, etc. (odd, then even
 * register addresses) are paired to form a word.
 *
 * Set word pair grouping order offset for the specified slave (0-3).
 * @param slaveNum Slave number (0-3)
 * @param enabled New word pair grouping order offset for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
void MPU6050_setSlaveWordGroupOffset(uint8_t slaveNum, bool enabled);

/** Set number of bytes to read for the specified slave (0-3).
 * @param slaveNum Slave number (0-3)
 * @param length Number of bytes to read for specified slave
 * @see MPU6050_RA_I2C_SLV0_CTRL
 */
void MPU6050_setSlaveDataLength(uint8_t slaveNum, uint8_t length);

/** Set I2C bypass enabled status.
 * When this bit is equal to 1 and I2C_MST_EN (Register 106 bit[5]) is equal to
 * 0, the host application processor will be able to directly access the
 * auxiliary I2C bus of the MPU-60X0. When this bit is equal to 0, the host
 * application processor will not be able to directly access the auxiliary I2C
 * bus of the MPU-60X0 regardless of the state of I2C_MST_EN (Register 106
 * bit[5]).
 * @param enabled New I2C bypass enabled status
 * @see MPU6050_RA_INT_PIN_CFG
 * @see MPU6050_INTCFG_I2C_BYPASS_EN_BIT
 */
void MPU6050_setI2CBypassEnabled(bool enabled);

/** Set FIFO Buffer Overflow interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 **/
void MPU6050_setIntFIFOBufferOverflowEnabled(bool enabled);


/** Set I2C Master interrupt enabled status.
 * @param enabled New interrupt enabled status
 * @see MPU6050_RA_INT_ENABLE
 * @see MPU6050_INTERRUPT_I2C_MST_INT_BIT
 **/
void MPU6050_setIntI2CMasterEnabled(bool enabled);


/** Get FIFO Buffer Overflow interrupt status.
 * This bit automatically sets to 1 when a Free Fall interrupt has been
 * generated. The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_FIFO_OFLOW_BIT
 */
bool MPU6050_getIntFIFOBufferOverflowStatus();


/** Get I2C Master interrupt status.
 * This bit automatically sets to 1 when an I2C Master interrupt has been
 * generated. For a list of I2C Master interrupts, please refer to Register 54.
 * The bit clears to 0 after the register has been read.
 * @return Current interrupt status
 * @see MPU6050_RA_INT_STATUS
 * @see MPU6050_INTERRUPT_I2C_MST_INT_BIT
 */
bool MPU6050_getIntI2CMasterStatus();


/** Read single byte from external sensor data register.
 * These registers store data read from external sensors by the Slave 0, 1, 2,
 * and 3 on the auxiliary I2C interface. Data read by Slave 4 is stored in
 * I2C_SLV4_DI (Register 53).
 *
 * @param position Starting position (0-23)
 * @return Byte read from register
 */

uint8_t MPU6050_getExternalSensorByte(int position);

/** Read word (2 bytes) from external sensor data registers.
 * @param position Starting position (0-21)
 * @return Word read from register
 * @see getExternalSensorByte()
 */
uint16_t MPU6050_getExternalSensorWord(int position);

/** Write byte to Data Output container for specified slave.
 * This register holds the output data written into Slave when Slave is set to
 * write mode. For further information regarding Slave control, please
 * refer to Registers 37 to 39 and immediately following.
 * @param slaveNum Slave number (0-3)
 * @param data Byte to write
 * @see MPU6050_RA_I2C_SLV0_DO
 */
void MPU6050_setSlaveOutputByte(uint8_t slaveNum, uint8_t data);


/**
 * Get slave delay enabled status.
 * When a particular slave delay is enabled, the rate of access for the that
 * slave device is reduced. When a slave's access rate is decreased relative to
 * the Sample Rate, the slave is accessed every:
 *
 *     1 / (1 + I2C_MST_DLY) Samples
 *
 * This base Sample Rate in turn is determined by SMPLRT_DIV (register  * 25)
 * and DLPF_CFG (register 26).
 *
 *  Set slave delay enabled status.
 * @param num Slave number (0-4)
 * @param enabled New slave delay enabled status.
 * @see MPU6050_RA_I2C_MST_DELAY_CTRL
 * @see MPU6050_DELAYCTRL_I2C_SLV0_DLY_EN_BIT
 */

void MPU6050_setSlaveDelayEnabled(uint8_t slaveNum, bool enabled);

/** Set FIFO enabled status.
 * @param enabled New FIFO enabled status
 * @see getFIFOEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_EN_BIT
 */
void MPU6050_setFIFOEnabled(bool enabled);

/** Set I2C Master Mode enabled status.
 * @param enabled New I2C Master Mode enabled status
 * @see getI2CMasterModeEnabled()
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_I2C_MST_EN_BIT
 */
void MPU6050_setI2CMasterModeEnabled(bool enabled);

/** Reset the FIFO.
 * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
 * bit automatically clears to 0 after the reset has been triggered.
 * @see MPU6050_RA_USER_CTRL
 * @see MPU6050_USERCTRL_FIFO_RESET_BIT
 */
void MPU6050_resetFIFO() ;

/** Trigger a full device reset.
 * A small delay of ~50ms may be desirable after triggering a reset.
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_DEVICE_RESET_BIT
 */
void MPU6050_deviceReset();

/** Set sleep mode status.
 * @param enabled New sleep mode enabled status
 * @see getSleepEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_SLEEP_BIT
 */
void MPU6050_setSleepEnabled(uint8_t enabled);

/** Set temperature sensor enabled status.
 * Note: this register stores the *disabled* value, but for consistency with the
 * rest of the code, the function is named and used with standard true/false
 * values to indicate whether the sensor is enabled or disabled, respectively.
 *
 * @param enabled New temperature sensor enabled status
 * @see getTempSensorEnabled()
 * @see MPU6050_RA_PWR_MGMT_1
 * @see MPU6050_PWR1_TEMP_DIS_BIT
 */
void MPU6050_setTempSensorEnabled(uint8_t enabled);
/**
* Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
* However, it is highly recommended that the device be configured to use one of
* the gyroscopes (or an external clock source) as the clock reference for
* improved stability. The clock source can be selected according to the following table:
*
* <pre>
* CLK_SEL | Clock Source
* --------+--------------------------------------
* 0       | Internal oscillator
* 1       | PLL with X Gyro reference
* 2       | PLL with Y Gyro reference
* 3       | PLL with Z Gyro reference
* 4       | PLL with external 32.768kHz reference
* 5       | PLL with external 19.2MHz reference
* 6       | Reserved
* 7       | Stops the clock and keeps the timing generator in reset
* </pre>
*
* @param source New clock source setting
* @see getClockSource()
* @see MPU6050_RA_PWR_MGMT_1
* @see MPU6050_PWR1_CLKSEL_BIT
* @see MPU6050_PWR1_CLKSEL_LENGTH
*/
void MPU6050_setClockSource(uint8_t source);


/**
 * @}
 */


/**
 * @defgroup Offsets and Fine gain
 * @{
 */

void MPU6050_setAxisFineGain(int8_t Xgain ,int8_t Ygain ,int8_t Zgain );
void MPU6050_getAxisFineGain(int8_t* Xgain ,int8_t* Ygain ,int8_t* Zgain );

void MPU6050_setAccelOffset(int16_t Xoffset , int16_t Yoffset ,int16_t Zoffset );
void MPU6050_getAccelOffset(int16_t* Xoffset , int16_t* Yoffset ,int16_t* Zoffset ) ;

void MPU6050_setGyroOffset(int16_t Xoffset, int16_t Yoffset , int16_t Zoffset) ;
void MPU6050_getGyroOffset(int16_t* Xoffset, int16_t* Yoffset , int16_t* Zoffset);

/**
 * @}
 */








#endif /* INV_MOTIONAPP_MOTIONAPP_6X_H_ */
