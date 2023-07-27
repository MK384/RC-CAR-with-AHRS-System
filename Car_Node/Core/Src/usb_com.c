/*
 * usb_cdc_com.h
 *
 *  Created on: Jul 22, 2023
 *      Author: mohammed Khaled
 *
 *      This file facilitate the communication over the USB CDC layer.
 *
 */

#ifndef INC_USB_COM_C_
#define INC_USB_COM_C_



/*! Includes  -----------------------------------------------------------*/
#include "usb_com.h"



/*! Private Types  -----------------------------------------------------------*/

typedef struct {

	uint8_t RxBuff[COM_BUF_SIZE];
	uint16_t	data_length;
	bool 		new_data_flag;

} USB_Type;


/*! Private variables -----------------------------------------------------------*/


USB_Type   usbEngine;

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
 bool USB_Send(uint8_t* data, uint16_t length)								    	{return !CDC_Transmit_FS(data, length);}
/**
 * @brief  USB_Recieve : receive data from USB COM Port if exist
 * @param  data : pointer to buffer of data to receive in
 * @param  length : Length of the data received
 * @retval return_type : true if there is a received data , false if there is no data to be received
 * @note
 */
 bool USB_Recieve(uint8_t* data, uint16_t* length)					   		     	{if(usbEngine.new_data_flag)
																						    {memcpy(data,usbEngine.RxBuff,usbEngine.data_length);
																							usbEngine.new_data_flag = false; *length = usbEngine.data_length;
																							return true;} *length = 0; data[0] = '\0'; return false;}
/**
 * @brief  USB_getRxStatus : get the status of RX channel
 * @param  None :
 * @retval bool : true if there is a new data to receive , false otherwise
 * @note
 */
 bool USB_getRxStatus(void)												 			{return usbEngine.new_data_flag;}

/**
 * @brief  USB_setRxCallback :  sets the function to be called when usb port received new data
 * @param  userCallbackFunc : a pointer to function that accepts two parameters [pointer to received data] and [the legnth of data received]
 * @retval return_type :
 * @note
 */




/*-------------------------------------------------------------------------------------------*/
/*! Private function declaration  -----------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------*/



/*
 *  FOR PRIVATE USE.! DON'T USE IN YOUR APPLICATION !!
 *
 *  [only call this this function in usbd_cdc_if.c file at the end of CDC_Receive_FS() function ]
 * */
void USB_setEngine(uint8_t* buff , uint32_t len)
{

memcpy(usbEngine.RxBuff, buff, len);
usbEngine.data_length = len;
usbEngine.new_data_flag = true;

}


#endif /* INC_USB_COM_C_ */
