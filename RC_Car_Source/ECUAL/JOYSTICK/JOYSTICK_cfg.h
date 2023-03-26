/*
 * JOYSTICK_cfg.h
 *
 *  Created on: Mar 25, 2023
 *      Author: moham
 */

#ifndef JOYSTICK_JOYSTICK_CFG_H_
#define JOYSTICK_JOYSTICK_CFG_H_

#include "JOYSTICK.h"
// The Number OF JoySticks To Be Used In The Project
#define JOYSTICK_UNITS  1

const JoyStick_obj JoyStick_CfgParam[JOYSTICK_UNITS] =
{
    // JoyStick unit 1 Configurations
    {
    SW_NOT_PRESSED,
	0,
	0,
    ADC_CHANNEL_6,
    ADC_CHANNEL_7
    }
};

#endif /* JOYSTICK_JOYSTICK_CFG_H_ */
