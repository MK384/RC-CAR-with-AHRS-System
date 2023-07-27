/*
 * i2c.h
 *
 *  Created on: Jul 21, 2023
 *      Author: moham
 */

#ifndef INVENSENSE_EMD_PORT_INC_I2C_H_
#define INVENSENSE_EMD_PORT_INC_I2C_H_


#include "main.h"

extern I2C_HandleTypeDef hi2c1;
#define 	I2C_Handle		(&hi2c1)


#define   MEM_REG_8BITS				1U
#define   I2C_TIMEOUT				100U


// I2C Communication Ported functions
HAL_StatusTypeDef i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data);
HAL_StatusTypeDef i2c_read (unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data );

// Time Ported Functions
void delay_ms(unsigned long ms);
void delay_us(unsigned long us);
unsigned long get_ms();
unsigned long get_us();


#endif /* INVENSENSE_EMD_PORT_INC_I2C_H_ */
