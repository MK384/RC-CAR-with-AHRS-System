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

#include "I2Cdev.h"

#include <stdio.h>

#define I2CDLY 10



/** Read a single bit from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitNum Bit position to read (0-7)
 * @param data Container for single bit value
 * @return Status of read operation (true = success)
 */
 void I2Cdev_readBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
  uint8_t buffer;
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,&buffer,1,I2CDLY) != HAL_OK);
  *data = buffer & (1 << bitNum);
}

/** Read multiple bits from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param bitStart First bit position to read (0-7)
 * @param length Number of bits to read (not more than 8)
 * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
 * @return Status of read operation (true = success)
 */
void I2Cdev_readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
  // 01101001 read byte
  // 76543210 bit numbers
  //    XXX   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted

  uint8_t buffer;
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,&buffer,1,I2CDLY) != HAL_OK){

  }

    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    buffer &= mask;
    buffer >>= (bitStart - length + 1);
    *data = buffer;
}

/** Read single byte from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for byte value read from device
 * @return Status of read operation (true = success)
 */
void I2Cdev_readByte(uint8_t devAddr, uint8_t regAddr, uint8_t *data) {

  uint8_t buffer;
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,&buffer,1,I2CDLY) != HAL_OK);
  data[0] = buffer;

}

/** Read multiple bytes from an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of bytes to read
 * @param data Buffer to store read data in
 * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
 */
void I2Cdev_readBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data) {

  uint8_t buffer[length];
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,buffer,length,I2CDLY) != HAL_OK);
  for (int i = 0; i < length ; i++) {
    data[i] = (uint8_t) buffer[i];
  }

}

/** write a single bit in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitNum Bit position to write (0-7)
 * @param value New bit value to write
 * @return Status of operation (true = success)
 */
void I2Cdev_writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data) {

  //first reading registery value
  uint8_t buffer;
  while( HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,&buffer,1,I2CDLY) != HAL_OK){

  }

    buffer = (data != 0) ? (buffer | (1 << bitNum)) : (buffer & ~(1 << bitNum));

    while(HAL_I2C_Mem_Write(I2C_Handle,devAddr,regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&buffer, 1,I2CDLY) != HAL_OK);
}

/** Write multiple bits in an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to write to
 * @param bitStart First bit position to write (0-7)
 * @param length Number of bits to write (not more than 8)
 * @param data Right-aligned value to write
 * @return Status of operation (true = success)
 */
void I2Cdev_writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value

  //first reading registery value

  uint8_t buffer;
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,&buffer,1,I2CDLY) != HAL_OK){

  }


    uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    bitStart &= ~(mask); // zero all important bits in existing byte
    buffer |= data; // combine data with existing byte

    while(HAL_I2C_Mem_Write(I2C_Handle,devAddr,regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&buffer, 1,I2CDLY) != HAL_OK);
}

/** Write single byte to an 8-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register address to write to
 * @param data New byte value to write
 * @return Status of operation (true = success)
 */
void I2Cdev_writeByte(uint8_t devAddr, uint8_t regAddr, uint8_t data) {

  while(HAL_I2C_Mem_Write(I2C_Handle,devAddr,regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&data, 1,I2CDLY) != HAL_OK);
}

/** Read single word from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr Register regAddr to read from
 * @param data Container for word value read from device
 * @return Status of read operation (true = success)
 */
void I2Cdev_readWord(uint8_t devAddr, uint8_t regAddr, uint16_t *data) {

  uint8_t buffer[2];
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,buffer,2,I2CDLY) != HAL_OK);
  data[0] = (buffer[0] << 8) | buffer[1] ;

}

/** Read multiple words from a 16-bit device register.
 * @param devAddr I2C slave device address
 * @param regAddr First register regAddr to read from
 * @param length Number of words to read
 * @param data Buffer to store read data in
 * @return Number of words read (-1 indicates failure)
 */
void I2Cdev_readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data) {

  uint8_t buffer[length << 1];
  while(HAL_I2C_Mem_Read(I2C_Handle,devAddr,regAddr,I2C_MEMADD_SIZE_8BIT,buffer,length*2,I2CDLY) != HAL_OK);

  uint8_t i;
  for (i = 0; i < length; i++) {
    data[i] = (buffer[i*2] << 8) | buffer[i*2+1] ;
  }

}

void I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data){

  uint8_t buffer[2];
  buffer[0] = (uint8_t) (data >> 8); //MSByte
  buffer[1] = (uint8_t) (data >> 0); //LSByte
  while(HAL_I2C_Mem_Write(I2C_Handle,devAddr,regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buffer, 2,I2CDLY) != HAL_OK);

}

void I2Cdev_writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data){
  uint8_t i;
  uint8_t buffer[length];
  for (i = 0; i < length; i++) {
    buffer[i] = data[i] ;
  }
  while(HAL_I2C_Mem_Write(I2C_Handle,devAddr,regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buffer, length,I2CDLY) != HAL_OK);

}

void I2Cdev_writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data){
  uint8_t i;
  uint8_t buffer[length << 1];
  for (i = 0; i < length; i++) {
    buffer[i << 1] = (uint8_t) (data[i] >> 8); //MSByte
    buffer[1 + (i << 1)] = (uint8_t) (data[i] >> 0); //LSByte
  }
  while(HAL_I2C_Mem_Write(I2C_Handle,devAddr,regAddr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)buffer, length*2,I2CDLY) != HAL_OK);
}
