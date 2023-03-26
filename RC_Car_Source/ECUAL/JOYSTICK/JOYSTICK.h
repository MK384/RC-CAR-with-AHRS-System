/*
 * JOYSTICK.h
 *
 *  Created on: Mar 24, 2023
 *      Author: moham
 */

#ifndef JOYSTICK_JOYSTICK_H_
#define JOYSTICK_JOYSTICK_H_

#define HAL_ADC_MODULE_ENABLED

#include "stm32f1xx_hal.h"



typedef enum{
	SW_PRESSED , SW_NOT_PRESSED
} JS_SwStatus;


typedef struct
{
	JS_SwStatus		JS_SWState;
    uint16_t       JS_xVal;
    uint16_t       JS_yVal;
    uint32_t       ADCx_CH;
    uint32_t       ADCy_CH;
}JoyStick_obj;



/*-----[ Prototypes For All Functions ]-----*/
void JoyStick_Init(void);
JoyStick_obj* JoyStick_getObj(uint8_t joystick_index);
void JoyStick_Read(uint8_t joystick_index);

#endif /* JOYSTICK_JOYSTICK_H_ */
