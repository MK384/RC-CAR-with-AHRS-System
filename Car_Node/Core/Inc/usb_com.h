/*
 * usb_com.h
 *
 *  Created on: Jul 22, 2023
 *      Author: moham
 */

#ifndef INC_USB_COM_H_
#define INC_USB_COM_H_

/*! Includes  -----------------------------------------------------------*/
#include "usbd_cdc_if.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

/*! User Config Defines -----------------------------------------------------------*/

#define 	COM_BUF_SIZE		512

/*-------------------------------------------------------------------------------------------*/
/*! Public Function -------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/

/**
 * @brief  USB_Send : 	Transmit data over USB COM Port
 * @param  data : pointer to buffer of data to be send
 * @param  length : Length of the data
 * @retval bool : [true if succeeded] [false otherwise]
 * @note
 */
bool USB_Send(uint8_t* data, uint16_t length);

/**
 * @brief  USB_Recieve : receive data from USB COM Port if exist
 * @param  data : pointer to buffer of data to receive in
 * @param  length : Length of the data received
 * @retval return_type : true if there is a received data , false if there is no data to be received
 * @note
 */
bool USB_Recieve(uint8_t* data, uint16_t* length);

/**
 * @brief  USB_getRxStatus : get the status of RX channel
 * @param  None :
 * @retval bool : true if there is a new data to receive , false otherwise
 * @note
 */
 bool USB_getRxStatus(void);

/**
 * @brief  USB_RxCallback :  called when there is new received data
 * @param  data : pointer to buffer of recieved data
 * @param  length : Length of the data received
 * @retval None :
 * @note	User Implementation is required
 * @note 	call this function in usbd_cdc_if.c file at the end of CDC_Receive_FS() function , pass [Buff] and [Len] parameters
 */
__weak void USB_RxCallback(uint8_t* data, uint32_t len);


/*-------------------------------------------------------------------------------------------*/
/*! Private -------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/


/*
 *  FOR PRIVATE USE.! DON'T USE IN YOUR APPLICATION !!
 *
 *  [only call this this function in usbd_cdc_if.c file at the end of CDC_Receive_FS() function ]
 * */
void USB_setEngine(uint8_t* buff , uint32_t len);

#endif /* INC_USB_COM_H_ */
