/*
 * nRF24_FD.h
 * Brief: Contains the Full nRF24 Driver APIs (Hardware independent)
 *
 *  Created on: Mar 21, 2023
 *      Author: Mohammed_Khaled
 */

#ifndef BOARD_HAL_DRIVERS_INC_NRF24_HAL_H_
#define BOARD_HAL_DRIVERS_INC_NRF24_HAL_H_



// Low level functions (hardware depended)
#include "nRF24_LL.h"



/**
 * @defgroup Default Values
 * @{
 */

#define   	nRF24_DEFAULT_RF_CHANNEL		115

#define   	nRF24_DEFAULT_RETX_DELAY		nRF24_ARD_250us

#define   	nRF24_DEFAULT_RETX_COUNT		(uint8_t)0x5

#define   	nRF24_DEFAULT_ADDR_WIDTH		(uint8_t)0x5



/**
 * @}
 */


/**
 * @defgroup Retransmit delay (nRF24_ARD_xx)
 * @{
 */

typedef enum {
	nRF24_ARD_NONE   = (uint8_t)0x00, // Dummy value for case when retransmission is not used
	nRF24_ARD_250us  = (uint8_t)0x00,
	nRF24_ARD_500us  = (uint8_t)0x01,
	nRF24_ARD_750us  = (uint8_t)0x02,
	nRF24_ARD_1000us = (uint8_t)0x03,
	nRF24_ARD_1250us = (uint8_t)0x04,
	nRF24_ARD_1500us = (uint8_t)0x05,
	nRF24_ARD_1750us = (uint8_t)0x06,
	nRF24_ARD_2000us = (uint8_t)0x07,
	nRF24_ARD_2250us = (uint8_t)0x08,
	nRF24_ARD_2500us = (uint8_t)0x09,
	nRF24_ARD_2750us = (uint8_t)0x0A,
	nRF24_ARD_3000us = (uint8_t)0x0B,
	nRF24_ARD_3250us = (uint8_t)0x0C,
	nRF24_ARD_3500us = (uint8_t)0x0D,
	nRF24_ARD_3750us = (uint8_t)0x0E,
	nRF24_ARD_4000us = (uint8_t)0x0F
}nRF24_ARDType;

/**
 * @}
 */
//


/**
 * @defgroup Data rate (nRF24_DR_xx)
 * @{
 */

typedef enum {
	nRF24_DR_250kbps = (uint8_t)0x20, // 250kbps data rate
	nRF24_DR_1Mbps   = (uint8_t)0x00, // 1Mbps data rate
	nRF24_DR_2Mbps   = (uint8_t)0x08  // 2Mbps data rate
}nRF24_DRType;

/**
 * @}
 */
//

/**
 * @defgroup RF output power in TX mode (nRF24_TXPWR_xx)
 * @{
 */

typedef enum {
	nRF24_TXPWR_18dBm = (uint8_t)0x00, // -18dBm
	nRF24_TXPWR_12dBm = (uint8_t)0x02, // -12dBm
	nRF24_TXPWR_6dBm  = (uint8_t)0x04, //  -6dBm
	nRF24_TXPWR_0dBm  = (uint8_t)0x06  //   0dBm
}nRF24_TX_PWRType;

/**
 * @}
 */
//

/**
 * @defgroup CRC encoding scheme (nRF24_CRC_xx)
 * @{
 */
typedef enum {
	nRF24_CRC_off   = (uint8_t)0x00, // CRC disabled
	nRF24_CRC_1byte = (uint8_t)0x08, // 1-byte CRC
	nRF24_CRC_2byte = (uint8_t)0x0c  // 2-byte CRC
}nRF24_CRCType;

/**
 * @}
 */
//

/**
 * @defgroup nRF24L01 power control (nRF24_PWR_xx)
 * @{
 */
typedef enum {
	nRF24_PWR_UP   = (uint8_t)0x02, // Power up
	nRF24_PWR_DOWN = (uint8_t)0x00  // Power down
}nRF24_PWR_ModeType;


/**
 * @}
 */
//

/**
 * @defgroup Transceiver mode (nRF24_MODE_xx)
 * @{
 */
typedef enum {
	nRF24_MODE_RX = (uint8_t)0x01, // PRX
	nRF24_MODE_TX = (uint8_t)0x00  // PTX
}nRF24_MODEType;

/**
 * @}
 */
//

/**
 * @defgroup Dynamic Payload lenght mode (nRF24_DPL_xx)
 * @{
 */

typedef enum {
	nRF24_DPL_ON = (uint8_t)0x01, // PRX
	nRF24_DPL_OFF = (uint8_t)0x00  // PTX
}nRF24_DPLType ;

/**
 * @}
 */

/**
 * @defgroup Enumeration of RX pipe addresses and TX address
 * @{
 */
typedef enum {
	nRF24_PIPE0  = (uint8_t)0x00, // pipe0
	nRF24_PIPE1  = (uint8_t)0x01, // pipe1
	nRF24_PIPE2  = (uint8_t)0x02, // pipe2
	nRF24_PIPE3  = (uint8_t)0x03, // pipe3
	nRF24_PIPE4  = (uint8_t)0x04, // pipe4
	nRF24_PIPE5  = (uint8_t)0x05, // pipe5
	nRF24_PIPETX = (uint8_t)0x06  // TX address (not a pipe in fact)
}nRF24_PipeType;

/**
 * @}
 */
//

/**
 * @defgroup State of auto acknowledgment for specified pipe (nRF24_AA_xx)
 * @{
 */

typedef enum {
	nRF24_AA_OFF = (uint8_t)0x00,
	nRF24_AA_ON  = (uint8_t)0x01
}nRF24_AAType;

/**
 * @}
 */
//

/**
 * @defgroup mode of the payload with Acknowledge (nRF24_PWACKType)
 * @{
 */

typedef enum{

	nRF24_PWACK_OFF = (uint8_t)0x00,
	nRF24_PWACK_ON  = (uint8_t)0x01


}nRF24_PWACKType;

/**
 * @}
 */

/**
 * @defgroup Status of the RX FIFO (nRF24_STATUS_RXFIFO_xx)
 * @{
 */

typedef enum {
	nRF24_STATUS_RXFIFO_DATA  = (uint8_t)0x00, // The RX FIFO contains data and available locations
	nRF24_STATUS_RXFIFO_EMPTY = (uint8_t)0x01, // The RX FIFO is empty
	nRF24_STATUS_RXFIFO_FULL  = (uint8_t)0x02, // The RX FIFO is full
	nRF24_STATUS_RXFIFO_ERROR = (uint8_t)0x03  // Impossible state: RX FIFO cannot be empty and full at the same time
}nRF24_STATUS_RXFIFO;

/**
 * @}
 */
//

/**
 * @defgroup Status of the TX FIFO (nRF24_STATUS_TXFIFO_xx)
 * @{
 */

typedef enum {
	nRF24_STATUS_TXFIFO_DATA  = (uint8_t)0x00, // The TX FIFO contains data and available locations
	nRF24_STATUS_TXFIFO_EMPTY = (uint8_t)0x01, // The TX FIFO is empty
	nRF24_STATUS_TXFIFO_FULL  = (uint8_t)0x02, // The TX FIFO is full
	nRF24_STATUS_TXFIFO_ERROR = (uint8_t)0x03  // Impossible state: TX FIFO cannot be empty and full at the same time
}nRF24_STATUS_TXFIFO;

/**
 * @}
 */
//

/**
 * @defgroup Result of RX FIFO reading (nRF24_RX_xx)
 * @{
 */

typedef enum {
	nRF24_RX_PIPE0  = (uint8_t)0x00, // Packet received from the PIPE#0
	nRF24_RX_PIPE1  = (uint8_t)0x01, // Packet received from the PIPE#1
	nRF24_RX_PIPE2  = (uint8_t)0x02, // Packet received from the PIPE#2
	nRF24_RX_PIPE3  = (uint8_t)0x03, // Packet received from the PIPE#3
	nRF24_RX_PIPE4  = (uint8_t)0x04, // Packet received from the PIPE#4
	nRF24_RX_PIPE5  = (uint8_t)0x05, // Packet received from the PIPE#5
	nRF24_RX_EMPTY  = (uint8_t)0xff  // The RX FIFO is empty
} nRF24_RXResult;

/**
 * @}
 */
//


/**
 * @defgroup Addresses of the RX_PW_P# registers (nRF24_REG_RX_PW_xx)
 * @{
 */
static const uint8_t nRF24_RX_PW_PIPE[6] = {
		nRF24_REG_RX_PW_P0,
		nRF24_REG_RX_PW_P1,
		nRF24_REG_RX_PW_P2,
		nRF24_REG_RX_PW_P3,
		nRF24_REG_RX_PW_P4,
		nRF24_REG_RX_PW_P5
};


/**
 * @}
 */
//

/**
 * @defgroup Addresses of the address registers (nRF24_REG_RX_ADDR_xx)
 * @{
 */

static const uint8_t nRF24_ADDR_REGS[7] = {
		nRF24_REG_RX_ADDR_P0,
		nRF24_REG_RX_ADDR_P1,
		nRF24_REG_RX_ADDR_P2,
		nRF24_REG_RX_ADDR_P3,
		nRF24_REG_RX_ADDR_P4,
		nRF24_REG_RX_ADDR_P5,
		nRF24_REG_TX_ADDR
};

/**
 * @}
 */
//

/**
 * @defgroup nRF24_HandleTypeDef
 * @{
 */


typedef struct{

	SPI_HandleTypeDef* SPI_Instance; 	/* pointer to SPI instance. */
	uint8_t RFChannel; 					/* channel - radio frequency channel, value from 0 to 125 */
	uint8_t addressWidth; 				/* RX/TX address field width, value from 3 to 5 */
	nRF24_TX_PWRType TxPwr; 			/* RF output power, one of nRF24_TXPWR_xx values */
	nRF24_DRType DataRate; 				/* data rate, one of nRF24_DR_xx values */
	nRF24_CRCType CRCScheme; 			/* CRC scheme, one of nRF24_CRC_xx values */
	nRF24_AAType AutoAck; 				/* Enable or disable the auto retransmit (a.k.a. enhanced ShockBurst) */
	nRF24_DPLType DPLMode;			    /* Enable/Disable transceiver DynamicPayloadLength feature for all the pipes*/


}nRF24_HandleTypeDef;


/**
 * @}
 */


/**
 * @defgroup  nRF24_StatusTypeDef
 * @{
 */

typedef enum
{
	nRF24_Online = (uint8_t)(0x01),
	nRF24_Offline = (uint8_t)(0x00)


}nRF24_StatusTypeDef;


/**
 * @}
 */


/**
 * @defgroup nRF24_TXResult Result of the Transmitted Packet
 * @{
 */

typedef enum {
	nRF24_TX_ERROR  = (uint8_t)0x00, // Unknown error
	nRF24_TX_SUCCESS,                // Packet has been transmitted successfully
	nRF24_TX_TIMEOUT,                // It was timeout during packet transmit
	nRF24_TX_MAXRT                   // Transmit failed with maximum auto retransmit count
} nRF24_TXResult;

/**
 * @}
 */



/****************************************************************************
 * @defgroup Initialization_functions
 * @{
 */



/**
* API_Brief: Init the nRF24 Module with the configuration pattern in the handler struct
* Args_Brief: pointer to the handler struct
* Ret_Brief:
* @note:
*/
void nRF24_Init(nRF24_HandleTypeDef* ptrHandler);

/**
* API_Brief:  Reset transceiver to it's initial state
* Args_Brief: 	none
* Ret_Brief: 	none
* @note: 	  RX/TX pipe addresses remains untouched
*/
void nRF24_Reset(void);


/**
* API_Brief: Check if the nRF24L01 present
* Args_Brief: 	none
* Ret_Brief:   1 - nRF24L01 is online and responding
*			   0 - received sequence differs from original
* @note:
*/
nRF24_StatusTypeDef nRF24_Check(void);

/**
* API_Brief: 	Control transceiver power ON/OFF mode
* Args_Brief: 	mode - new state of power mode, one of nRF24_PWR_xx values
* Ret_Brief:	None
* @note:	the power must be set on before any operation!
*/
void nRF24_SetPowerMode(nRF24_PWR_ModeType mode);

/**
 * @}
 * **************************************************************************
 */

/***************************************************************************************
 * @defgroup Status_functions
 * @{
 */

#define nRF24_FLAG_RX_DR           (uint8_t)0x40 // RX_DR bit (data ready RX FIFO interrupt)
#define nRF24_FLAG_TX_DS           (uint8_t)0x20 // TX_DS bit (data sent TX FIFO interrupt)
#define nRF24_FLAG_MAX_RT          (uint8_t)0x10 // MAX_RT bit (maximum number of TX retransmits interrupt)

/**
* API_Brief: Get pending IRQ flags
* Args_Brief:
* Ret_Brief:	current status of RX_DR, TX_DS and MAX_RT bits of the STATUS register
* @note:
*/
uint8_t nRF24_GetIRQFlags(void);

/**
* API_Brief: Clear any pending IRQ flags
* Args_Brief:
* Ret_Brief:
* @note:
*/
void nRF24_ClearIRQFlags(void);


/**
* API_Brief: Get status of the RX FIFO
* Args_Brief:
* Ret_Brief:	one of the nRF24_STATUS_RXFIFO_xx values
* @note:
*/

nRF24_STATUS_RXFIFO nRF24_GetStatus_RXFIFO(void);
/**
* API_Brief: Get status of the TX FIFO
* Args_Brief:
* Ret_Brief:	one of the nRF24_STATUS_TXFIFO_xx values
* @note: the TX_REUSE bit ignored
*/
nRF24_STATUS_TXFIFO nRF24_GetStatus_TXFIFO(void);
/**
* API_Brief: Get pipe number for the payload available for reading from RX FIFO
* Args_Brief:
* Ret_Brief:	pipe number or 0x07 if the RX FIFO is empty
* @note:
*/
nRF24_RXResult nRF24_GetRXSource(void);


/**
 * @}
***************************************************************************************
 */
/*****************************************************************************************
 * @defgroup Communication_function
 * @{
 */


/**
 * @brief  nRF24_StartListening : Set the nRF24 in RX Mode
 * @param  void :
 * @retval void :
 * @note
 */
void nRF24_StartListening(void);

/**
 * @brief  nRF24_StartListening : Set the nRF24 in TX Mode
 * @param  void :
 * @retval void :
 * @note
 */
void nRF24_StopListening(void);

/**
 * @brief  nRF24_OpenTxPipe : Open Transmitting Pipe
 * @param  uint8_t* :
 * @retval void :
 * @note
 */
void nRF24_OpenTxPipe(uint8_t* Address);


/**
 * @brief  nRF24_OpenRxPipe : Open Receiving Pipe
 * @param  uint8_t* :
 * @retval void :
 * @note
 */
void nRF24_OpenRxPipe( nRF24_PipeType pipe ,uint8_t* Address, uint8_t payloadLength);

/**
 * @brief  nRF24_IsAvailable : check if there is data available to read in Rx Buffer
 * @param  enclosing_method_arguments :
 * @retval return_type : 	true if there is available data false if else.
 * @note
 */
uint8_t nRF24_IsAvailable();


/**
 * @brief  nRF24_TransmitPacket : Function to transmit data packet
 * @param  uint8_t*, uint8_t :    pBuf - pointer to the buffer with data to transmit
	  	  	  	  	  	  	  	  	  length - length of the data buffer in bytes
 * @retval nRF24_TXResult :		  one of nRF24_TXResult values
 * @note
 */
nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length, uint32_t timeOut );


/**
 * @brief  nRF24_RecievePacket : Receive a data packet
 * @param  uint8_t*, uint8_t, uint32_t : 	 buffer for the data , length of the data ,
 * 											 time to wait if the RX FIFO is empty
 * @retval nRF24_RXResult :		one of the nRF24_RX_xx values
 * @note
 */
nRF24_RXResult nRF24_ReceivePacket(uint8_t *pBuf, uint8_t length, uint32_t timeOut);







/***************************************************************************************
 * @defgroup Auxiliary Function
 * @{
 */

/**
* API_Brief: Flush the TX FIFO
* Args_Brief:
* Ret_Brief:
* @note:
*/
void nRF24_FlushTX(void);
/**
* API_Brief: Flush the RX FIFO
* Args_Brief:
* Ret_Brief:
* @note:
*/
void nRF24_FlushRX(void);

/**
 * @}
 */


#endif /* BOARD_HAL_DRIVERS_INC_NRF24_HAL_H_ */
