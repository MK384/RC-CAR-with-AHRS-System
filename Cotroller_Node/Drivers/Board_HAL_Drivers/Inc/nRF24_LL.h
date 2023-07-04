/*
 * nRF24_LL.h
 * Brief: Header file contains the low level part of the nRF24 Driver.
 *
 *  Created on: Mar 21, 2023
 *      Author: Mohammed_khaled
 */

#ifndef BOARD_HAL_DRIVERS_INC_NRF24_LL_H_
#define BOARD_HAL_DRIVERS_INC_NRF24_LL_H_



#include "main.h"



/**
 * @defgroup nRF24L0 instruction definitions
 * @{
 */


#define nRF24_CMD_R_REGISTER       (uint8_t)0x00 // Register read
#define nRF24_CMD_W_REGISTER       (uint8_t)0x20 // Register write
#define nRF24_CMD_ACTIVATE         (uint8_t)0x50 // (De)Activates R_RX_PL_WID, W_ACK_PAYLOAD, W_TX_PAYLOAD_NOACK features
#define nRF24_CMD_R_RX_PL_WID	   (uint8_t)0x60 // Read RX-payload width for the top R_RX_PAYLOAD in the RX FIFO.
#define nRF24_CMD_R_RX_PAYLOAD     (uint8_t)0x61 // Read RX payload
#define nRF24_CMD_W_TX_PAYLOAD     (uint8_t)0xA0 // Write TX payload
#define nRF24_CMD_W_ACK_PAYLOAD    (uint8_t)0xA8 // Write ACK payload
#define nRF24_CMD_W_TX_PAYLOAD_NOACK (uint8_t) 0xB0//Write TX payload and disable AUTOACK
#define nRF24_CMD_FLUSH_TX         (uint8_t)0xE1 // Flush TX FIFO
#define nRF24_CMD_FLUSH_RX         (uint8_t)0xE2 // Flush RX FIFO
#define nRF24_CMD_REUSE_TX_PL      (uint8_t)0xE3 // Reuse TX payload
#define nRF24_CMD_LOCK_UNLOCK      (uint8_t)0x50 // Lock/unlock exclusive features
#define nRF24_CMD_NOP              (uint8_t)0xFF // No operation (used for reading status register)


/**
 * @}
 */


/**
 * @defgroup  nRF24L0 register definitions
 *
 * @{
 */
#define nRF24_REG_CONFIG           (uint8_t)0x00 // Configuration register
#define nRF24_REG_EN_AA            (uint8_t)0x01 // Enable "Auto acknowledgment"
#define nRF24_REG_EN_RXADDR        (uint8_t)0x02 // Enable RX addresses
#define nRF24_REG_SETUP_AW         (uint8_t)0x03 // Setup of address widths
#define nRF24_REG_SETUP_RETR       (uint8_t)0x04 // Setup of automatic retransmit
#define nRF24_REG_RF_CH            (uint8_t)0x05 // RF channel
#define nRF24_REG_RF_SETUP         (uint8_t)0x06 // RF setup register
#define nRF24_REG_STATUS	       (uint8_t)0x07 // Status register
#define nRF24_REG_OBSERVE_TX       (uint8_t)0x08 // Transmit observe register
#define nRF24_REG_RPD              (uint8_t)0x09 // Received power detector
#define nRF24_REG_RX_ADDR_P0       (uint8_t)0x0A // Receive address data pipe 0
#define nRF24_REG_RX_ADDR_P1       (uint8_t)0x0B // Receive address data pipe 1
#define nRF24_REG_RX_ADDR_P2       (uint8_t)0x0C // Receive address data pipe 2
#define nRF24_REG_RX_ADDR_P3       (uint8_t)0x0D // Receive address data pipe 3
#define nRF24_REG_RX_ADDR_P4       (uint8_t)0x0E // Receive address data pipe 4
#define nRF24_REG_RX_ADDR_P5       (uint8_t)0x0F // Receive address data pipe 5
#define nRF24_REG_TX_ADDR          (uint8_t)0x10 // Transmit address
#define nRF24_REG_RX_PW_P0         (uint8_t)0x11 // Number of bytes in RX payload in data pipe 0
#define nRF24_REG_RX_PW_P1         (uint8_t)0x12 // Number of bytes in RX payload in data pipe 1
#define nRF24_REG_RX_PW_P2         (uint8_t)0x13 // Number of bytes in RX payload in data pipe 2
#define nRF24_REG_RX_PW_P3         (uint8_t)0x14 // Number of bytes in RX payload in data pipe 3
#define nRF24_REG_RX_PW_P4         (uint8_t)0x15 // Number of bytes in RX payload in data pipe 4
#define nRF24_REG_RX_PW_P5         (uint8_t)0x16 // Number of bytes in RX payload in data pipe 5
#define nRF24_REG_FIFO_STATUS      (uint8_t)0x17 // FIFO status register
#define nRF24_REG_DYNPD            (uint8_t)0x1C // Enable dynamic payload length
#define nRF24_REG_FEATURE          (uint8_t)0x1D // Feature register

/**
 * @}
 */

/**
 * @defgroup Register bits definitions
 *
 * @{
 */
#define nRF24_CONFIG_PRIM_RX       (uint8_t)0x01 // PRIM_RX bit in CONFIG register
#define nRF24_CONFIG_PWR_UP        (uint8_t)0x02 // PWR_UP bit in CONFIG register

#define nRF24_FEATURE_EN_DYN_ACK   (uint8_t)0x01 // EN_DYN_ACK bit in FEATURE register
#define nRF24_FEATURE_EN_ACK_PAY   (uint8_t)0x02 // EN_ACK_PAY bit in FEATURE register
#define nRF24_FEATURE_EN_DPL       (uint8_t)0x04 // EN_DPL bit in FEATURE register



/**
 * @}
 */
//

/**
 * @defgroup Register masks definitions
 * @{
 */

#define nRF24_MASK_REG_MAP         (uint8_t)0x1F // Mask bits[4:0] for CMD_RREG and CMD_WREG commands
#define nRF24_MASK_CRC             (uint8_t)0x0C // Mask for CRC bits [3:2] in CONFIG register
#define nRF24_MASK_STATUS_IRQ      (uint8_t)0x70 // Mask for all IRQ bits in STATUS register
#define nRF24_MASK_RF_PWR          (uint8_t)0x06 // Mask RF_PWR[2:1] bits in RF_SETUP register
#define nRF24_MASK_RX_P_NO         (uint8_t)0x0E // Mask RX_P_NO[3:1] bits in STATUS register
#define nRF24_MASK_DATARATE        (uint8_t)0x28 // Mask RD_DR_[5,3] bits in RF_SETUP register
#define nRF24_MASK_EN_RX           (uint8_t)0x3F // Mask ERX_P[5:0] bits in EN_RXADDR register
#define nRF24_MASK_RX_PW           (uint8_t)0x3F // Mask [5:0] bits in RX_PW_Px register
#define nRF24_MASK_RETR_ARD        (uint8_t)0xF0 // Mask for ARD[7:4] bits in SETUP_RETR register
#define nRF24_MASK_RETR_ARC        (uint8_t)0x0F // Mask for ARC[3:0] bits in SETUP_RETR register
#define nRF24_MASK_RXFIFO          (uint8_t)0x03 // Mask for RX FIFO status bits [1:0] in FIFO_STATUS register
#define nRF24_MASK_TXFIFO          (uint8_t)0x30 // Mask for TX FIFO status bits [5:4] in FIFO_STATUS register
#define nRF24_MASK_PLOS_CNT        (uint8_t)0xF0 // Mask for PLOS_CNT[7:4] bits in OBSERVE_TX register
#define nRF24_MASK_ARC_CNT         (uint8_t)0x0F // Mask for ARC_CNT[3:0] bits in OBSERVE_TX register


/**
 * @}
 */
//

// Fake address to test transceiver presence (5 bytes long)
#define nRF24_TEST_ADDR            "nRF24"


/**
 * @defgroup spi_instance
 * @{
 */
static SPI_HandleTypeDef* SPI_Instance;

/**
 * @}
 */


static inline void nRF24_CE_L() {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CE_H() {
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

static inline void nRF24_CSN_L() {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CSN_H() {
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}


static inline uint8_t nRF24_LL_RW(uint8_t data) {
    // Wait until TX buffer is empty
    uint8_t result;
    if(HAL_SPI_TransmitReceive(SPI_Instance,&data,&result,1,2000)!=HAL_OK) {
        Error_Handler();
    };
    return result;
}

static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }



/**************************************************************************
 * @defgroup Configuration_functions
 * @{
 */


/**
* API_Brief:  Set transceiver operational mode
* Args_Brief: mode - operational mode, one of nRF24_MODE_xx values
* Ret_Brief:
* @note:
*/
static void nRF24_SetOperationalMode(uint8_t mode);
/**
* API_Brief: 	 Set frequency channel
* Args_Brief: 	 channel - radio frequency channel, value from 0 to 127
* Ret_Brief:
* @note: 		frequency will be (2400 + channel)MHz
*/
static void nRF24_SetRFChannel(uint8_t channel);
/**
* API_Brief: Set automatic retransmission parameters
* Args_Brief: 	  ard - auto retransmit delay, one of nRF24_ARD_xx values
*				  arc - count of auto retransmits, value form 0 to 15
* Ret_Brief:
* @note: 	zero arc value means that the automatic retransmission disabled
*/
static void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc);
/**
* API_Brief: 	Set of address widths
* Args_Brief: 	addr_width - RX/TX address field width, value from 3 to 5
* Ret_Brief:
* @note: 		this setting is common for all pipes
*/
static void nRF24_SetAddrWidth(uint8_t addr_width);
/**
* API_Brief: Set static RX address for a specified pipe
* Args_Brief:  pipe - pipe to configure address, one of nRF24_PIPEx values
*			   addr - pointer to the buffer with address
* Ret_Brief:
*
* note: pipe can be a number from 0 to 5 (RX pipes) and 6 (TX pipe)
* note: buffer length must be equal to current address width of transceiver
* note: for pipes[2..5] only first byte of address will be written because
*       other bytes of address equals to pipe1
* note: for pipes[2..5] only first byte of address will be written because
*       pipes 1-5 share the four most significant address bytes
*/
static void nRF24_SetAddr(uint8_t pipe, const uint8_t *addr);
/**
* API_Brief: 	Configure RF output power in TX mode
* Args_Brief: 	tx_pwr - RF output power, one of nRF24_TXPWR_xx values
* Ret_Brief:
* @note:
*/
static void nRF24_SetTXPower(uint8_t tx_pwr);
/**
* API_Brief:	 Configure transceiver data rate
* Args_Brief: 	data_rate - data rate, one of nRF24_DR_xx values
* Ret_Brief:
* @note:
*/
static void nRF24_SetDataRate(uint8_t data_rate);
/**
* API_Brief:	 Configure transceiver CRC scheme
* Args_Brief: 	 scheme - CRC scheme, one of nRF24_CRC_xx values
* Ret_Brief:
* @note: transceiver will forcibly turn on the CRC in case if auto acknowledgment
*        enabled for at least one RX pipe
*/
static void nRF24_SetCRCScheme(uint8_t scheme);
/**
* API_Brief: Configure a specified RX pipe
* Args_Brief: 	   pipe - number of the RX pipe, value from 0 to 5
*   			   aa_state - state of auto acknowledgment, one of nRF24_AA_xx values
*		  		   payload_len - payload length in bytes
* Ret_Brief:
* @note:
*/
static void nRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len);
/**
* API_Brief: Disable specified RX pipe
* Args_Brief: 	PIPE - number of RX pipe, value from 0 to 5
* Ret_Brief:
* @note:
*/
void nRF24_ClosePipe(uint8_t pipe);
/**
* API_Brief: Enable the auto retransmit
* 				(a.k.a. enhanced ShockBurst) for the specified RX pipe
* Args_Brief: 	pipe - number of the RX pipe, value from 0 to 5
* Ret_Brief:
* @note:
*/
static void nRF24_EnableAA();
/**
* API_Brief:	 Disable the auto retransmit
* 				(a.k.a. enhanced ShockBurst) for one or all RX pipes
*
* Args_Brief: 	pipe - number of the RX pipe, value from 0 to 5,
* 				any other value will disable AA for all RX pipes
* Ret_Brief:
* @note:
*/
static void nRF24_DisableAA();
/**
* API_Brief: Set transceiver DynamicPayloadLength feature for all the pipes
* Args_Brief: 	mode - status, one of nRF24_DPL_xx values
* Ret_Brief:
* @note:
*/
static void nRF24_SetDPL_Mode(uint8_t mode);
/**
* API_Brief: Enables Payload With Ack.Refer to the datasheet for proper retransmit timing.
* Args_Brief: 	mode - status, 1 or 0
* Ret_Brief:
* @note:
*/
 void nRF24_SetPayloadWithAck(uint8_t mode);



/**
 * @}
 * ************************************************************************************
 */


/**
* API_Brief:	 Write TX payload
* Args_Brief:  pBuf - pointer to the buffer with payload data
*			   length - payload length in bytes
* Ret_Brief:
* @note:
*/
void nRF24_WritePayload(uint8_t *pBuf, uint8_t length);
/**
* API_Brief:  Write TX payload in payload ack moad
* Args_Brief:  pipe : pipe number which the payload correspond
* 				pBuf - pointer to the buffer with payload data
*			   length - payload length in bytes
*
* Ret_Brief:
* @note:
*/
void nRF24_WriteAckPayload(uint8_t pipe, char *payload, uint8_t length);
/**
* API_Brief: Read top level payload available in the RX FIFO
* Args_Brief:  pBuf - pointer to the buffer with payload data
*			   length - payload length in bytes
* Ret_Brief:	one of nRF24_RX_xx values
* @note: nRF24_RX_PIPEX - packet has been received from the pipe number X
* @note: nRF24_RX_EMPTY - the RX FIFO is empty
*/
uint8_t nRF24_ReadPayload(uint8_t *pBuf, uint8_t *length);
/**
* API_Brief: Read top level payload available in the RX FIFO in dynamic payload length mode
* Args_Brief:  pBuf - pointer to the buffer with payload data
*			    length - payload length in bytes
* Ret_Brief:	one of nRF24_RX_xx values
* @note: nRF24_RX_PIPEX - packet has been received from the pipe number X
* @note: nRF24_RX_EMPTY - the RX FIFO is empty
*/
uint8_t nRF24_ReadPayloadDpl(uint8_t *pBuf, uint8_t *length);

/**
* API_Brief: Get auto retransmit statistic
* Args_Brief:
* Ret_Brief:	value of OBSERVE_TX register which contains two counters encoded in nibbles:
*			   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
*			   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
* @note:
*/
uint8_t nRF24_GetRetransmitCounters(void);
/**
* API_Brief: Get value of the FEATURE register
* Args_Brief:
* Ret_Brief: value of the FEATURE register
* @note:
*/
uint8_t nRF24_GetFeatures(void);

/**
* API_Brief: Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
* Args_Brief:
* Ret_Brief:
* @note:
*/
void nRF24_ResetPLOS(void);

/**
* API_Brief: Activiate FEATURE register
* Args_Brief:
* Ret_Brief:
* @note:
*/
void nRF24_ActivateFeatures(void);


/**
 * @}
 */

/**
 * @defgroup Tranciever_Type_Cntrl
 * @{
 */

#define nRF24_RX_ON()   nRF24_CE_H();
#define nRF24_RX_OFF()  nRF24_CE_L();

/**
 * @}
 */




#endif /* BOARD_HAL_DRIVERS_INC_NRF24_LL_H_ */
