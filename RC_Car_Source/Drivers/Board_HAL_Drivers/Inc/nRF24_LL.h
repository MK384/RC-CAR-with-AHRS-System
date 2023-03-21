/*
 * nRF24_LL.h
 * Brief: Header file contains the low level hardware dependent part of the nRF24 Driver.
 *
 *  Created on: Mar 21, 2023
 *      Author: Mohammed_khaled
 */

#ifndef BOARD_HAL_DRIVERS_INC_NRF24_LL_H_
#define BOARD_HAL_DRIVERS_INC_NRF24_LL_H_



#include "main.h"


extern SPI_HandleTypeDef hspi1;


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
    if(HAL_SPI_TransmitReceive(&hspi1,&data,&result,1,2000)!=HAL_OK) {
        Error_Handler();
    };
    return result;
}


static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }


#endif /* BOARD_HAL_DRIVERS_INC_NRF24_LL_H_ */
