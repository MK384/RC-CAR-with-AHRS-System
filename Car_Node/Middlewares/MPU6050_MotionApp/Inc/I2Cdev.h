// I2Cdev library collection - Main I2C device class
// Abstracts bit and byte I2C R/W functions into a convenient class
// RaspberryPi bcm2835 library port: bcm2835 library available at http://www.airspayce.com/mikem/bcm2835/index.html
// Based on Arduino's I2Cdev by Jeff Rowberg <jeff@rowberg.net>
//

/* ============================================
I2Cdev device library code is placed under the MIT license

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _I2CDEV_H_
#define _I2CDEV_H_

/*** Includes *******************************************************/
#include "main.h"



/**
 *  ======================================================================================
 * |					            User Configuration Section         			     	  |
 *  ======================================================================================
 */

extern    I2C_HandleTypeDef    hi2c1;

#define   I2C_Handle		  ((I2C_HandleTypeDef *) &hi2c1)


/*==========================End of User Configuration Section  ==============================*/




 void I2Cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
 void I2Cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
 void I2Cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data);
 void I2Cdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data);
 void I2Cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
 void I2Cdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);

 void I2Cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
 void I2Cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
 void I2Cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data);
 void I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
 void I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data);
 void I2Cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data);


#endif /* _I2CDEV_H_ */
