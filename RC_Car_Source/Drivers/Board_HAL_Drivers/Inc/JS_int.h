/*
 * JS_int.h
 *
 *  Created on: 21 Mar 2023
 *      Author: moham
 */

#ifndef BOARD_HAL_DRIVERS_INC_JS_INT_H_
#define BOARD_HAL_DRIVERS_INC_JS_INT_H_

typedef void (*JS_Callback_FN)(void);

typedef struct JS_HandlerType {
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	uint8_t VoltLvl;
	uint8_t ADC_Channel;
	JS_Callback_FN callback_function;

}JS_HandlerType;

typedef struct JS_PosType {
	uint16_t VX : 12;
	uint16_t VY : 12;
}JS_PosType ;

typedef enum JS_SwType{
	SW_PRESSED , SW_NOT_PRESSED
} JS_SwType;



void JS_Init(JS_HandlerType * JS_Obj);
void JS_ReadPosition(JS_HandlerType * JS_Obj, JS_PosType * JS_Pos);
JS_SwType JS_ReadSW(JS_HandlerType * JS_Obj);
JS_EnableExtInt(JS_HandlerType * JS_Obj , JS_Callback_FN callback_fn);


#endif /* BOARD_HAL_DRIVERS_INC_JS_INT_H_ */
