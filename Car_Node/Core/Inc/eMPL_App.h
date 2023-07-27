/*
 * eMPL_App.h
 *
 *  Created on: Jul 22, 2023
 *      Author: moham
 */

#ifndef INC_EMPL_APP_H_
#define INC_EMPL_APP_H_


#include "usbd_cdc_if.h"
#include "stdio.h"

#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "invensense.h"
#include "invensense_adv.h"
#include "eMPL_outputs.h"
#include "mltypes.h"
#include "mpu.h"
#include "log.h"
#include "packet.h"



void eMPL_APP_Init(void);


void eMPL_APP_Process(void);


void gyro_data_ready_cb(void);


#endif /* INC_EMPL_APP_H_ */
