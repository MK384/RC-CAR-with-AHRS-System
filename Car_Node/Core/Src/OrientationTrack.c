/*
 * MotionTrack_MATLAB.c
 *
 *  Created on: Jul 14, 2023
 *      Author: moham
 */
#include <OrientationTrack.h>
#include "main.h"
#include "stdio.h"
#include "stm32f4xx.h"
#include "motion_fx.h"
#include "mpu6050.h"
#include "GY271.h"
#include "string.h"
#include "usbd_cdc_if.h"



#define			USE_MATLAB				1
#define 		USE_9X					1
#define 		FRAME_SIZE				13



uint8_t		FrameBuffer[FRAME_SIZE];

static uint8_t RequestMagCalibration = 0;
static uint8_t RequestSensorDataRead = 0;

static uint8_t IsMagCalibrated    = 0;
static uint8_t IsDataReadyForFuse = 0;
static uint8_t IsDataReadyForSend = 0;

extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t UserRxBufferFS[];


static uint32_t LastUpdateTimeStamp , LastPropagateTimeStamp,  LastMagTime;
static float deltaTimeSec, MagTimeStamp;

static MPU6050  mpu;
static GY271    mag;

static MFX_MagCal_input_t mag_cal_in;
static MFX_MagCal_output_t mag_cal_out;

static MFX_input_t		data_in;
static MFX_output_t    data_out;
static MFX_knobs_t		ipKnops;

float   MagOffset[3];

#define 	UPDATE_PERIOD_MS						 10
#define 	Timer									TIM2
#define		Base_Timer							  (&htim2)


#define		MICRO_SEC_TO_SEC	   	    ((float)1.0f / (float)1000000.0f)
#define		MICRO_SEC_TO_MEILLI_SEC	    ((float)1.0f / (float)   1000.0f)


static void init_motionFX(void);
static void init_sensors(void);
static void sensor_data_read(void);
static void sensor_data_fuse(void);
static void sensor_data_send(void);

#if (USE_MATLAB == 1)
	static void FloatToArray(uint8_t *Dest, float Data);
	static void init_matlab_channel(void);
#endif



	/************* Functions Implementations ***************/

void MX_DataLogFusion_Init(void){

#if (USE_MATLAB == 1)
	init_matlab_channel();
#endif

	HAL_TIM_Base_Start(Base_Timer);

	init_sensors();

	init_motionFX();

	if (IsMagCalibrated){

		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);
		MotionFX_MagCal_init(UPDATE_PERIOD_MS, 0);
	}
	else{

		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);
		MotionFX_MagCal_init(UPDATE_PERIOD_MS, 1);
	}

}


void MX_DataLogFusion_Process(void){


	if (RequestMagCalibration){

	    while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_RESET);

	    /* Debouncing */
	    HAL_Delay(50);

	    RequestMagCalibration = 0;
	    IsMagCalibrated = 0;

	    MagOffset[0] = 0;
	    MagOffset[1] = 0;
	    MagOffset[2] = 0;

	    MotionFX_MagCal_init(UPDATE_PERIOD_MS, 1);
	    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, RESET);

	}

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

		RequestMagCalibration = 1;
	}

}

static void init_motionFX(void){

	  MotionFX_initialize();

	  MotionFX_getKnobs(&ipKnops);


	  ipKnops.acc_orientation[0] = 'e';
	  ipKnops.acc_orientation[1] = 'n';
	  ipKnops.acc_orientation[2] = 'u';

	  ipKnops.gyro_orientation[0] = 'e';
	  ipKnops.gyro_orientation[1] = 'n';
	  ipKnops.gyro_orientation[2] = 'u';

	  ipKnops.mag_orientation[0] = 's';
	  ipKnops.mag_orientation[1] = 'e';
	  ipKnops.mag_orientation[2] = 'u';

	  ipKnops.start_automatic_gbias_calculation = 1;
	  ipKnops.output_type = MFX_ENGINE_OUTPUT_NED;
	  ipKnops.LMode = 1U;
	  ipKnops.modx = 2U;

	  MotionFX_setKnobs(&ipKnops);

#if (USE_9X == 1)

		  MotionFX_enable_6X(MFX_ENGINE_DISABLE);
		  MotionFX_enable_9X(MFX_ENGINE_ENABLE);

#else
		  MotionFX_enable_9X(MFX_ENGINE_DISABLE);
		  MotionFX_enable_6X(MFX_ENGINE_ENABLE);
#endif


}

static void init_sensors(void){


	  mpu.I2Cx = &hi2c1;
	  mpu.Address = MPU6050_I2C_ADDR_AD0_LOW;
	  mpu.LowPassFilter = MPU6050_DLPF_BW_98Hz;
	  mpu.DataRate = MPU6050_DataRate_100Hz;
	  mpu.AccelerometerRange = MPU6050_Accelerometer_4G;
	  mpu.GyroscopeRange= MPU6050_Gyroscope_250s;
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
	  mag.dataRate = GY271_DataRate_30_Hz;
	  mag.range = GY271_Range_4_7Ga;
	  mag.mode = GY271_Mode_Continuous;
	  mag.samplesRate = GY271_SampleAvg_4_S;
	  mag.outputUnit = GY271_UNIT_MICRO_TESLA_PER_50;

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


		data_in.acc[0] = mpu.Accel_Xyz[0];
		data_in.acc[1] = mpu.Accel_Xyz[1];
		data_in.acc[2] = mpu.Accel_Xyz[2];

		data_in.gyro[0] = mpu.Gyro_Xyz[0];
		data_in.gyro[1] = mpu.Gyro_Xyz[1];
		data_in.gyro[2] = mpu.Gyro_Xyz[2];

		GY271_getData(&mag);

		if (!IsMagCalibrated){

			mag_cal_in.mag[0] = mag.Compass_Xyz[0];
			mag_cal_in.mag[1] = mag.Compass_Xyz[1];
			mag_cal_in.mag[2] = mag.Compass_Xyz[2];


			MagTimeStamp += ((float)(LastMagTime - Timer->CNT) * MICRO_SEC_TO_MEILLI_SEC);
			LastMagTime = Timer->CNT;
			mag_cal_in.time_stamp = (int) MagTimeStamp;

			MotionFX_MagCal_run(&mag_cal_in);

			MotionFX_MagCal_getParams(&mag_cal_out);

			if (mag_cal_out.cal_quality == MFX_MAGCALGOOD)
			{
				IsMagCalibrated = 1;

				MagOffset[0] = mag_cal_out.hi_bias[0];
				MagOffset[1] = mag_cal_out.hi_bias[1];
				MagOffset[2] = mag_cal_out.hi_bias[2];

				MotionFX_MagCal_init((unsigned short int) UPDATE_PERIOD_MS, 0);
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, SET);

			}
		}

		data_in.mag[0] = mag.Compass_Xyz[0] - MagOffset[0];
		data_in.mag[1] = mag.Compass_Xyz[1] - MagOffset[1];
		data_in.mag[2] = mag.Compass_Xyz[2] - MagOffset[2];

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
				printf("%.3f,%.3f,%.3f\n\r",data_out.rotation_9X[0], data_out.rotation_9X[1], data_out.rotation_9X[2]);
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




char MotionFX_LoadMagCalFromNVM(unsigned short int dataSize, unsigned int *data)
{

	  return (char) 1;

}


/**
 * @brief  Save calibration parameter to memory
 * @param  dataSize length ot the data
 * @param  data pointer to the data
 * @retval (1) fail, (0) success
 */
char MotionFX_SaveMagCalInNVM(unsigned short int dataSize, unsigned int *data)
{

  return (char) 1;
}
