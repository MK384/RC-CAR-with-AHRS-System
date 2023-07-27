/*
 * i2c.c
 *
 *  Created on: Jul 21, 2023
 *      Author: moham
 */


#include "i2c.h"



// I2C Communication Ported functions
HAL_StatusTypeDef i2c_write(unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data){

	HAL_StatusTypeDef result =  HAL_I2C_Mem_Write(I2C_Handle, (uint16_t) slave_addr, (uint16_t) reg_addr, MEM_REG_8BITS, data,(uint16_t) length, I2C_TIMEOUT);
	return result;
}
HAL_StatusTypeDef i2c_read (unsigned char slave_addr, unsigned char reg_addr, unsigned char length, unsigned char *data ){

	HAL_StatusTypeDef result = HAL_I2C_Mem_Read(I2C_Handle, (uint16_t) slave_addr, (uint16_t) reg_addr, MEM_REG_8BITS, data,(uint16_t) length, I2C_TIMEOUT);
	return result;
}

// Time Ported Functions
void delay_ms(unsigned long ms){
	ms *= 1000;
	unsigned long start = TIM2->CNT;
	while( (TIM2->CNT - start) < ms)
	{

	}
}

void delay_us(unsigned long us)
{
	unsigned long start = TIM2->CNT;
	while( (TIM2->CNT - start) < (uint32_t) us)
	{

	}
}

unsigned long get_ms(){

	return (unsigned long) TIM2->CNT /  1000UL;

}

unsigned long get_us(){

	return (unsigned long) TIM2->CNT;

}

