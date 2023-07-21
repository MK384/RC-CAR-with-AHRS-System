/*
 * MotionTrack_MATLAB.c
 *
 *  Created on: Jul 14, 2023
 *      Author: moham
 */
/**
 * This application file is created for testing purposes.
 * It has no LICENSE, permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 */
#include <MPU6050_Basic.h>
#include <OrientationTrack.h>
#include "main.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "motion_fx.h"
#include "GY271.h"
#include "string.h"
#include "usbd_cdc_if.h"





#define			USE_MATLAB				0
#define 		USE_9X					1
#define			USE_IMU_CAL				0
#define			USE_MAG_CAL				1



#if		USE_IMU_CAL	== 1
	#include "motion_ac.h"
	#include "motion_gc.h"
#endif

#if		USE_MAG_CAL == 1
	#include "motion_mc.h"
#endif


#define 	UPDATE_PERIOD_MS						 10
#define 	Timer									TIM2
#define		Base_Timer							  (&htim2)
#define 	FRAME_SIZE								 13


#define		MICRO_SEC_TO_SEC	   	    ((float)1.0f / (float)1000000.0f)
#define		MICRO_SEC_TO_MEILLI_SEC	    ((float)1.0f / (float)   1000.0f)


#define		SAMPLE_TIME					10


uint8_t		FrameBuffer[FRAME_SIZE];


static uint8_t RequestHeadingCalibration = 0;
static uint8_t RequestSensorDataRead = 0;

static uint8_t IsDataReadyForFuse = 0;
static uint8_t IsDataReadyForSend = 0;

extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t UserRxBufferFS[];


static uint32_t LastUpdateTimeStamp , LastPropagateTimeStamp;
static float deltaTimeSec , time_Stamp;

static MPU6050  mpu;
static GY271    mag;


static MFX_input_t		data_in;
static MFX_output_t    data_out;
static MFX_knobs_t		ipKnobs;

#if USE_MAG_CAL == 1

MMC_Input_t mag_cal_input;
MMC_Output_t mag_cal_output;

#endif

#if	USE_IMU_CAL == 1

MAC_input_t acc_cal_input;
MAC_output_t acc_cal_output;
MAC_knobs_t  acc_knobs;

MGC_input_t  gyr_cal_input;
MGC_output_t gyr_cal_output;
MGC_knobs_t	gyr_knobs;

#endif



float   AccOffset[3];
float   GyrOffset[3];
float   MagOffset[3];

float   HeadingOffset;


static void init_motionFX(void);
static void init_sensors(void);
static void sensor_data_read(void);
static void sensor_data_fuse(void);
static void sensor_data_send(void);

#if (USE_MATLAB == 1)
	static void FloatToArray(uint8_t *Dest, float Data);
	static void init_matlab_channel(void);
#endif

#if  USE_IMU_CAL == 1

	static void acc_calibrate(void);
	static void gyr_calibrate(void);

#endif
#if USE_MAG_CAL == 1

	static void mag_calibrate(void);

#endif

	/************* Functions Implementations ***************/

void MX_DataLogFusion_Init(void){

#if (USE_MATLAB == 1)
	init_matlab_channel();
#endif

	HAL_TIM_Base_Start(Base_Timer);

	init_sensors();

	init_motionFX();

	HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, RESET);
	HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, RESET);
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, RESET);

#if USE_IMU_CAL == 1

	gyr_calibrate();
	acc_calibrate();

#endif

#if USE_MAG_CAL == 1

	mag_calibrate();

#endif

#if (USE_9X == 1)

		  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
		  MotionFX_enable_9X(MFX_ENGINE_ENABLE);

#else
		  MotionFX_enable_9X(MFX_ENGINE_DISABLE);
		  MotionFX_enable_6X(MFX_ENGINE_ENABLE);
#endif

}


void MX_DataLogFusion_Process(void){

	sensor_data_read();
	sensor_data_fuse();
	sensor_data_send();

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{

	if (GPIO_Pin == MPUP6050_RDY_INT_Pin){

		RequestSensorDataRead = 1;
	}

	if (GPIO_Pin == GPIO_PIN_0 ){

			RequestHeadingCalibration = 1;

	}

}

static void init_motionFX(void){

	  MotionFX_initialize();

	  MotionFX_getKnobs(&ipKnobs);


	  ipKnobs.acc_orientation[0] = 'e';
	  ipKnobs.acc_orientation[1] = 'n';
	  ipKnobs.acc_orientation[2] = 'u';

	  ipKnobs.gyro_orientation[0] = 'e';
	  ipKnobs.gyro_orientation[1] = 'n';
	  ipKnobs.gyro_orientation[2] = 'u';

	  ipKnobs.mag_orientation[0] = 's';
	  ipKnobs.mag_orientation[1] = 'e';
	  ipKnobs.mag_orientation[2] = 'u';

	  ipKnobs.start_automatic_gbias_calculation = 1;
	  ipKnobs.output_type = MFX_ENGINE_OUTPUT_NED;
	  ipKnobs.LMode = 1U;
	  ipKnobs.modx = 2U;

	  MotionFX_setKnobs(&ipKnobs);


		  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
		  MotionFX_enable_9X(MFX_ENGINE_DISABLE);



		  MotionFX_MagCal_init(10, 0);
}

static void init_sensors(void){


	  mpu.I2Cx = &hi2c1;
	  mpu.Address = MPU6050_I2C_ADDR_AD0_LOW;
	  mpu.LowPassFilter = MPU6050_DLPF_BW_98Hz;
	  mpu.DataRate = MPU6050_DataRate_100Hz;
	  mpu.AccelerometerRange = MPU6050_Accelerometer_4G;
	  mpu.GyroscopeRange= MPU6050_Gyroscope_500s;
	  mpu.interruptState = MPU6050_Interrupt_Enabled;
	  mpu.AccOutputUnit = MPU6050_ACC_UNIT_G;
	  mpu.GyrOutputUnit = MPU6050_GYR_UNIT_DPS;

	  if (MPU6050_Init(&mpu))
	  {
		  while(1)
		  {
			  HAL_Delay(1000);
			  printf("Error in init MPU6050! \n\r");
		  }
	  }


	  mag.I2Cx = &hi2c1;
	  mag.dataRate = GY271_DataRate_75_Hz;
	  mag.range = GY271_Range_4_7Ga;
	  mag.mode = GY271_Mode_Continuous;
	  mag.samplesRate = GY271_SampleAvg_4_S;
	  mag.outputUnit = GY271_UNIT_MICRO_TESLA;

	  if (GY271_Init(&mag))
	  {

		  while(1)
		  {
			  HAL_Delay(1000);
			  printf("Error in init GY271! \n\r");
		  }
	  }

}

static void sensor_data_read(void){

	if (RequestSensorDataRead){

		RequestSensorDataRead = 0;


		MPU6050_ReadAll(&mpu);


#if USE_IMU_CAL == 1

		data_in.acc[0] = (mpu.Accel_Xyz[0] - acc_cal_output.AccBias[0]) * acc_cal_output.SF_Matrix[0][0];
		data_in.acc[1] = (mpu.Accel_Xyz[1] - acc_cal_output.AccBias[1]) * acc_cal_output.SF_Matrix[1][1];
		data_in.acc[2] = (mpu.Accel_Xyz[2] - acc_cal_output.AccBias[2]) * acc_cal_output.SF_Matrix[2][2];

		data_in.gyro[0] = mpu.Gyro_Xyz[0] - gyr_cal_output.GyroBiasX;
		data_in.gyro[1] = mpu.Gyro_Xyz[1] - gyr_cal_output.GyroBiasY;
		data_in.gyro[2] = mpu.Gyro_Xyz[2] - gyr_cal_output.GyroBiasZ;

#else


		data_in.acc[0] = mpu.Accel_Xyz[0];
		data_in.acc[1] = mpu.Accel_Xyz[1];
		data_in.acc[2] = mpu.Accel_Xyz[2];

		data_in.gyro[0] = mpu.Gyro_Xyz[0];
		data_in.gyro[1] = mpu.Gyro_Xyz[1];
		data_in.gyro[2] = mpu.Gyro_Xyz[2];


#endif


		GY271_getData(&mag);

#if 	USE_MAG_CAL == 1

		data_in.mag[0] = (mag.Compass_Xyz[0] - mag_cal_output.HI_Bias[0]) * mag_cal_output.SF_Matrix[0][0] +
						 (mag.Compass_Xyz[1] - mag_cal_output.HI_Bias[1]) * mag_cal_output.SF_Matrix[0][1] +
						 (mag.Compass_Xyz[2] - mag_cal_output.HI_Bias[2]) * mag_cal_output.SF_Matrix[0][2];

		data_in.mag[1] = (mag.Compass_Xyz[0] - mag_cal_output.HI_Bias[0]) * mag_cal_output.SF_Matrix[1][0] +
						 (mag.Compass_Xyz[1] - mag_cal_output.HI_Bias[1]) * mag_cal_output.SF_Matrix[1][1] +
						 (mag.Compass_Xyz[2] - mag_cal_output.HI_Bias[2]) * mag_cal_output.SF_Matrix[1][2];

		data_in.mag[2] = (mag.Compass_Xyz[0] - mag_cal_output.HI_Bias[0]) * mag_cal_output.SF_Matrix[2][0] +
						 (mag.Compass_Xyz[1] - mag_cal_output.HI_Bias[1]) * mag_cal_output.SF_Matrix[2][1] +
						 (mag.Compass_Xyz[2] - mag_cal_output.HI_Bias[2]) * mag_cal_output.SF_Matrix[2][2];

#elif

		data_in.mag[0] = mag.Compass_Xyz[0];
		data_in.mag[1] = mag.Compass_Xyz[1];
		data_in.mag[2] = mag.Compass_Xyz[2];


#endif

		data_in.mag[0] /= 50.0;
		data_in.mag[1] /= 50.0;
		data_in.mag[2] /= 50.0;


		IsDataReadyForFuse = 1;

	}

}

static void sensor_data_fuse(void){

	if (IsDataReadyForFuse){

		IsDataReadyForFuse = 0;


		deltaTimeSec = (float) (Timer->CNT - LastPropagateTimeStamp) * MICRO_SEC_TO_SEC  ;
		LastPropagateTimeStamp = Timer->CNT;
		MotionFX_propagate(&data_out, &data_in, &deltaTimeSec);

		deltaTimeSec = (float) (Timer->CNT - LastUpdateTimeStamp) * MICRO_SEC_TO_SEC  ;
		LastUpdateTimeStamp = Timer->CNT;
		MotionFX_update(&data_out, &data_in, &deltaTimeSec, NULL);

		if (RequestHeadingCalibration)
			{
				HeadingOffset = data_out.heading_9X;
				RequestHeadingCalibration = 0;
			}

		IsDataReadyForSend = 1;
	}


}

static void sensor_data_send(void){

	if (IsDataReadyForSend){

#if (USE_9X == 1)

	#if	(USE_MATLAB == 1)
						FloatToArray(&FrameBuffer[1], data_out.rotation_9X[0]);
						FloatToArray(&FrameBuffer[5], data_out.rotation_9X[1]);
						FloatToArray(&FrameBuffer[9], data_out.rotation_9X[2]);
	#else
//				printf("%.3f,%.3f,%.3f\n\r",data_out.rotation_9X[0], data_out.rotation_9X[1], data_out.rotation_9X[2]);
			    printf("%.3f\n\r",data_out.heading_9X );
	#endif


#else

	#if	(USE_MATLAB == 1)
						FloatToArray(&FrameBuffer[1], data_out.rotation_6X[0]);
						FloatToArray(&FrameBuffer[5], data_out.rotation_6X[1]);
						FloatToArray(&FrameBuffer[9], data_out.rotation_6X[2]);

	#else
				printf("%.3f,%.3f,%.3f\n\r",data_out.rotation_6X[0], data_out.rotation_6X[1], data_out.rotation_6X[2]);
	#endif

#endif


#if	(USE_MATLAB == 1)
		CDC_Transmit_FS(FrameBuffer, FRAME_SIZE);
#endif

		IsDataReadyForSend = 0;
	}

}
#if		(USE_MATLAB == 1)

	static void FloatToArray(uint8_t *Dest, float Data)
	{
	  (void)memcpy(Dest, (void *)&Data, 4);
	}
	static void init_matlab_channel(void){

		UserRxBufferFS[0] = '\0';

		while (UserRxBufferFS[0] == '\0');

		if (UserRxBufferFS[0] != '#')
			while (1){

				 printf("Communication Error !\n");
				 HAL_Delay(1000);
			}


		FrameBuffer[0] = '$';

	}
#endif



#if 	USE_MAG_CAL == 1

void  static mag_calibrate(void){

	time_Stamp = 0;
	MotionMC_Initialize(10, 1);


	while (mag_cal_output.CalQuality  != MMC_CALQSTATUSGOOD ){

		HAL_Delay(SAMPLE_TIME);

			GY271_getData(&mag);

			mag_cal_input.Mag[0] = mag.Compass_Xyz[0];
			mag_cal_input.Mag[1] = mag.Compass_Xyz[1];
			mag_cal_input.Mag[2] = mag.Compass_Xyz[2];

			mag_cal_input.TimeStamp = time_Stamp;
			time_Stamp += SAMPLE_TIME;

			MotionMC_Update(&mag_cal_input);

			MotionMC_GetCalParams(&mag_cal_output);

	}

	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, SET);

}

#endif

#if  USE_IMU_CAL == 1

void static acc_calibrate(void){


	uint8_t is_calibrated;
	MotionAC_Initialize(1);
	MotionAC_GetKnobs(&acc_knobs);
	acc_knobs.Sample_ms = SAMPLE_TIME;
	MotionAC_SetKnobs(&acc_knobs);

	time_Stamp = 0;


	while(acc_cal_output.CalQuality != MAC_CALQSTATUSGOOD ){

		if(RequestSensorDataRead){

			RequestSensorDataRead = 0;

			MPU6050_ReadAll(&mpu);
			acc_cal_input.Acc[0] = mpu.Accel_Xyz[0];
			acc_cal_input.Acc[1] = mpu.Accel_Xyz[1];
			acc_cal_input.Acc[2] = mpu.Accel_Xyz[2];

			acc_cal_input.TimeStamp = time_Stamp;
			time_Stamp += SAMPLE_TIME;

			MotionAC_Update(&acc_cal_input, &is_calibrated);
			MotionAC_GetCalParams(&acc_cal_output);

		}
	}

HAL_GPIO_WritePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin, SET);
}

void static gyr_calibrate(void){

	time_Stamp = 0;
	int  bias_update = 0;
	float freq = 100.0f;
	MotionGC_Initialize(&freq);

	MotionGC_GetKnobs(&gyr_knobs);
	gyr_knobs.AccThr = 0.008f;
	gyr_knobs.GyroThr = 0.15;
	MotionGC_SetKnobs(&gyr_knobs);

	gyr_cal_output.GyroBiasX = 0;
	gyr_cal_output.GyroBiasY = 0;
	gyr_cal_output.GyroBiasZ = 0;

	MotionGC_SetCalParams(&gyr_cal_output);

	MotionGC_SetFrequency(&freq);


	while (bias_update == 0){

		if(RequestSensorDataRead){

			RequestSensorDataRead = 0;
			MPU6050_ReadAll(&mpu);

			gyr_cal_input.Acc[0] = mpu.Accel_Xyz[0];
			gyr_cal_input.Acc[1] = mpu.Accel_Xyz[1];
			gyr_cal_input.Acc[2] = mpu.Accel_Xyz[2];


			gyr_cal_input.Gyro[0] = mpu.Gyro_Xyz[0];
			gyr_cal_input.Gyro[1] = mpu.Gyro_Xyz[1];
			gyr_cal_input.Gyro[2] = mpu.Gyro_Xyz[2];

			MotionGC_Update(&gyr_cal_input, &gyr_cal_output, &bias_update);
			MotionGC_GetCalParams(&gyr_cal_output);

		}

	}
HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, SET);
}

#endif





char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data) { return (char) 1;}
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data) { return (char) 1;}
char MotionAC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data) { return (char) 1;}
char MotionAC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data) { return (char) 1;}
char MotionMC_LoadCalFromNVM(unsigned short int dataSize, unsigned int *data) { return (char) 1;}
char MotionMC_SaveCalInNVM(unsigned short int dataSize, unsigned int *data) { return (char) 1;}

