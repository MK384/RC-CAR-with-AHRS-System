/*
 * eMPL_App.c
 *
 *  Created on: Jul 22, 2023
 *      Author: moham
 */


#include "stdio.h"
#include "eMPL_App.h"
#include "GY271.h"



 /**
  * TODO: Find the bug that prevents 9x fusion from working properly
  */
//#define USE_CAL_HW_REGISTERS
//#define HMC5883L
//#define COMPASS_ENABLED


/*! Private function declaration  -----------------------------------------------------------*/
void gyro_data_ready_cb(void);
static inline void run_self_test(void);
static void android_orient_cb(unsigned char orientation);
static void tap_cb(unsigned char direction, unsigned char count);
static void read_from_mpl(void);


/*! Private typedef -----------------------------------------------------------*/
/*! Data read from MPL. */
#define PRINT_ACCEL (0x01)
#define PRINT_GYRO (0x02)
#define PRINT_QUAT (0x04)
#define PRINT_COMPASS (0x08)
#define PRINT_EULER (0x10)
#define PRINT_ROT_MAT (0x20)
#define PRINT_HEADING (0x40)
#define PRINT_PEDO (0x80)
#define PRINT_LINEAR_ACCEL (0x100)
#define PRINT_GRAVITY_VECTOR (0x200)

volatile uint32_t hal_timestamp = 0; //!< timestamp variable initialize, its for count tick

#define ACCEL_ON (0x01)
#define GYRO_ON (0x02)
#define COMPASS_ON (0x04)

#define MOTION (0)
#define NO_MOTION (1)

/*! Starting sampling rate. */
#define DEFAULT_MPU_HZ (100)
#define DEFAULT_MAG_HZ (100)

#define MPU_READ_MS (10)
#define MAG_READ_MS (10)


#define FLASH_SIZE (512)
#define FLASH_MEM_START ((void *)0x1800)

#define PEDO_READ_MS (1000)
#define TEMP_READ_MS (500)



/*! halStruct Init */
struct hal_s
{
    unsigned char lp_accel_mode;
    unsigned char sensors;
    unsigned char dmp_on;
    unsigned char wait_for_tap;
    volatile unsigned char new_frame;
    volatile unsigned char new_data;
    unsigned char motion_int_mode;
    unsigned long no_dmp_hz;
    unsigned long next_pedo_ms;
    unsigned long next_temp_ms;
    unsigned long next_compass_ms;
    unsigned int report;
    unsigned short dmp_features;

};
static struct hal_s hal = {0};

unsigned char *mpl_key = (unsigned char *)"eMPL 5.1";

/*! Platform-specific information. Kinda like a boardfile. */
struct platform_data_s
{
    signed char orientation[9];
};

/*! The sensors can be mounted onto the board in any orientation. The mounting
 * matrix seen below tells the MPL how to rotate the raw data from the
 * driver(s).
 * boards at Invensense. If needed, please modify the matrices to match the
 * chip-to-body matrix for your particular set up.
 */
static struct platform_data_s gyro_pdata = {
    .orientation = {1, 0, 0,
                    0, 1, 0,
                    0, 0, 1}};

/*! Compass calibration for MPU9150 */
#if defined MPU9150 || defined MPU9250
static struct platform_data_s compass_pdata = {
    .orientation = {0, 1, 0,
                    1, 0, 0,
                    0, 0, -1}};
#define COMPASS_ENABLED 1
#elif defined AK8975_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                    0, 1, 0,
                    0, 0, -1}};
#define COMPASS_ENABLED 1
#elif defined AK8963_SECONDARY
static struct platform_data_s compass_pdata = {
    .orientation = {-1, 0, 0,
                    0, -1, 0,
                    0, 0, 1}};
#define COMPASS_ENABLED 1
#elif defined HMC5883
static struct platform_data_s compass_pdata = {
    .orientation = {0, 1, 0,
                   -1, 0, 0,
                    0, 0, 1}};
//#define COMPASS_ENABLED 1
#endif

/*! Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* ---------------------------------------------------------------------------*/
/* Get data from MPL.
 * between new and stale data.
 */
static void read_from_mpl(void)
{
    /*! CLocal Variable init */
    long int data[4];
    int8_t accuracy;
    inv_time_t timestamp;
    float float_data[4] = {0};

    if (hal.report & PRINT_QUAT)
    {
        /*! Sends a quaternion packet to the PC. Since this is used by the Python
         * test app to visually represent a 3D quaternion, it's sent each time
         * the MPL has new data.
         */
    	if (inv_get_sensor_type_quat(data, &accuracy, &timestamp))
    	{
    		for (int i = 0; i < 4; ++i) {float_data[i] =  inv_q30_to_float(data[i]);}

    		printf("%.3f,%.3f,%.3f,%.3f\n\r",float_data[0],float_data[1],float_data[2], float_data[3]);

    	}
    }

    else if (hal.report & PRINT_ACCEL)
    {
        if (inv_get_sensor_type_accel(data, &accuracy,&timestamp))
        {
    		for (int i = 0; i < 3; ++i) {float_data[i] =  inv_q16_to_float(data[i]);}

    		printf("%.3f,%.3f,%.3f\n\r",float_data[0],float_data[1],float_data[2]);

        }
    }
    else if (hal.report & PRINT_GYRO)
    {
        if (inv_get_sensor_type_gyro(data, &accuracy,&timestamp))
        {

        }
    }

#ifdef COMPASS_ENABLED
    else if (hal.report & PRINT_COMPASS)
    {
        if (inv_get_sensor_type_compass(data, &accuracy,
                                        (inv_time_t *)&timestamp))
            {
				for (int i = 0; i < 3; ++i) {float_data[i] =  inv_q16_to_float(data[i]);}

				printf("%.3f,%.3f,%.3f\n\r",float_data[0],float_data[1],float_data[2]);

            }
    }
#endif

    else if (hal.report & PRINT_EULER)
    {
        if (inv_get_sensor_type_euler(data, &accuracy, &timestamp))
        {
        	for (int i = 0; i < 3; ++i) float_data[i] =  inv_q16_to_float(data[i]);

        	printf("%.3f,%.3f,%.3f\n\r",float_data[0],float_data[1],float_data[2]);

        }
    }
    else if (hal.report & PRINT_ROT_MAT)
    {
        if (inv_get_sensor_type_rot_mat(data, &accuracy,&timestamp))
        {

        }

    }
    else if (hal.report & PRINT_HEADING)
    {
        if (inv_get_sensor_type_heading(data, &accuracy,&timestamp))
        {

        	for (int i = 0; i < 1; ++i) float_data[i] =  inv_q16_to_float(data[0]);

        	printf("%.3f\n\r",float_data[0]);
        }

    }
    else if (hal.report & PRINT_LINEAR_ACCEL)
    {
        if (inv_get_sensor_type_linear_acceleration(float_data, &accuracy, &timestamp))
        {

        }
    }
    else if (hal.report & PRINT_GRAVITY_VECTOR)
    {
        if (inv_get_sensor_type_gravity(float_data, &accuracy, &timestamp))
        {
        	printf("%.3f,%.3f,%.3f\n\r",float_data[0],float_data[1],float_data[2]);
        }
    }

}


#ifdef COMPASS_ENABLED
void send_status_compass()
{
    long data[3] = {0};
    int8_t accuracy = {0};
    unsigned long timestamp;
    inv_get_compass_set(data, &accuracy, (inv_time_t *)&timestamp);
    MPL_LOGI("Compass: %7.4f %7.4f %7.4f ",
             data[0] / 65536.f, data[1] / 65536.f, data[2] / 65536.f);
    MPL_LOGI("Accuracy= %d\r\n", accuracy);
}
#endif


static void tap_cb(unsigned char direction, unsigned char count)
{
    switch (direction)
    {
    case TAP_X_UP:
        MPL_LOGI("Tap X+ ");
        break;
    case TAP_X_DOWN:
        MPL_LOGI("Tap X- ");
        break;
    case TAP_Y_UP:
        MPL_LOGI("Tap Y+ ");
        break;
    case TAP_Y_DOWN:
        MPL_LOGI("Tap Y- ");
        break;
    case TAP_Z_UP:
        MPL_LOGI("Tap Z+ ");
        break;
    case TAP_Z_DOWN:
        MPL_LOGI("Tap Z- ");
        break;
    default:
        return;
    }
    MPL_LOGI("x%d\n", count);
    return;
}
/*! If you want to connect IMU
 *  with Android mobile device
 *  via bluetooth then this function
 *  should be called */
static void android_orient_cb(unsigned char orientation)
{
    switch (orientation)
    {
    case ANDROID_ORIENT_PORTRAIT:
        MPL_LOGI("Portrait\n");
        break;
    case ANDROID_ORIENT_LANDSCAPE:
        MPL_LOGI("Landscape\n");
        break;
    case ANDROID_ORIENT_REVERSE_PORTRAIT:
        MPL_LOGI("Reverse Portrait\n");
        break;
    case ANDROID_ORIENT_REVERSE_LANDSCAPE:
        MPL_LOGI("Reverse Landscape\n");
        break;
    default:
        return;
    }
}

/*! Self Test Function
 *
 */
 void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

/* self test returns biases in g's << 16. */
    result = mpu_run_self_test(gyro, accel);

    if (result == 0x7 || result == 0x3 )
    {
        printf("Passed!\n");
        printf("accel bias: %7.4f %7.4f %7.4f\n",
                 accel[0] / 65536.f,
                 accel[1] / 65536.f,
                 accel[2] / 65536.f);
        printf("gyro bias: %7.4f %7.4f %7.4f\n",
                 gyro[0] / 65536.f,
                 gyro[1] / 65536.f,
                 gyro[2] / 65536.f);
        /*! Test passed. We can trust the gyro data here, so now we need to update calibrated data*/

#ifdef USE_CAL_HW_REGISTERS
        /*!
         * This portion of the code uses the HW offset registers that are in the MPUxxxx devices
         * instead of pushing the cal data to the MPL software library
         */
        unsigned char i = 0;

        for (i = 0; i < 3; i++)
        {
        	// convert to +-1000dps
            gyro[i] = (long)(gyro[i] * 32.8f);
            gyro[i] = (long)(gyro[i] >> 16);
            // convert to +-16G
            accel[i] *= 2048.f;
            accel[i] = accel[i] >> 16;
        }

        mpu_set_gyro_bias_reg(gyro);
        mpu_set_accel_bias_6050_reg(accel);

#else
        /* Push the calibrated data to the MPL library.
         *
         * MPL expects biases in hardware units << 16, but self test returns
         * biases in g's << 16.
         */
        unsigned short accel_sens;
        float gyro_sens;

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        inv_set_accel_bias(accel, 3);
        mpu_get_gyro_sens(&gyro_sens);
        gyro[0] = (long)(gyro[0] * gyro_sens);
        gyro[1] = (long)(gyro[1] * gyro_sens);
        gyro[2] = (long)(gyro[2] * gyro_sens);
        inv_set_gyro_bias(gyro, 3);
#endif
    }
    else
    {
        if (!(result & 0x1))
        	printf("Gyro failed in self test.\n");
        if (!(result & 0x2))
        	printf("Accel failed in self test.\n");
        if (!(result & 0x4))
        	printf("Compass failed in self test.\n");
    }
}

/*! Every time new gyro data is available, this function is called in an
 * ISR context. In this example, it sets a flag protecting the FIFO read
 * function.
 */
void gyro_data_ready_cb(void)
{
    hal.new_frame = 1;
}
/*******************************************************************************/


/*! Variable Declaration */
inv_error_t result;
unsigned char accel_fsr, new_temp = 0;
unsigned short gyro_rate, gyro_fsr;
unsigned long timestamp;
struct int_param_s int_param;

#ifdef COMPASS_ENABLED
unsigned char new_compass = 0;
unsigned short compass_fsr;
#endif


void eMPL_APP_Init(void){


    result = mpu_init(&int_param);
    if (result)
    {
    	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, RESET);
    	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET);

        while(1){
        	MPL_LOGE("Could not initialize MPU6050.\n");
        	HAL_Delay(1000);
           }
    }
    else
    {
        MPL_LOGE("Initialise MPU6050 successful.\n");
    }

    /*! If you're not using an MPU9150 AND you're not using DMP features, this
     * function will place all slaves on the primary bus.
     * mpu_set_bypass(1);
     */

    result = inv_init_mpl();
    if (result)
    {
    	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, RESET);
    	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET);

        while(1){
        	MPL_LOGE("Could not initialize MPL.\n");
        	HAL_Delay(1000);
           }
    }
    else
    {
    	MPL_LOGE("Initialise MPL successful.\n");
    }


    /*! Compute 6-axis and 9-axis quaternions. */

    inv_enable_quaternion();

#if defined COMPASS_ENABLED
    inv_enable_9x_sensor_fusion();
#endif


    /*! The MPL expects compass data at a constant rate (matching the rate
     * passed to inv_set_compass_sample_rate). If this is an issue for your
     * application, call this function, and the MPL will depend on the
     * timestamps passed to inv_build_compass instead.
     *
     * inv_9x_fusion_use_timestamps(1);
     */
#if defined COMPASS_ENABLED
     inv_9x_fusion_use_timestamps(1);
#endif
    /*! This function has been deprecated.
     * inv_enable_no_gyro_fusion();
     */

    /*! Update gyro biases when not in motion.
     * WARNING: These algorithms are mutually exclusive.
     */

    inv_enable_fast_nomot();
    /*! inv_enable_motion_no_motion(); */
    /*! inv_set_no_motion_time(1000); */

    /*! Update gyro biases when temperature changes. */
    inv_enable_gyro_tc();

    /*run self test */
    run_self_test();



    /*! This algorithm updates the accel biases when in motion. A more accurate
     * bias measurement can be made when running the self-test (see case 't' in
     * handle_input), but this algorithm can be enabled if the self-test can't
     * be executed in your application.
     *
     * inv_enable_in_use_auto_calibration();
     */


#ifdef COMPASS_ENABLED
    /*! Compass calibration algorithms. */
    inv_enable_vector_compass_cal();
    inv_enable_magnetic_disturbance();
#endif
    /*! If you need to estimate your heading before the compass is calibrated,
     * enable this algorithm. It becomes useless after a good figure-eight is
     * detected, so we'll just leave it out to save memory.
     * inv_enable_heading_from_gyro();
     */

    /*! Allows use of the MPL APIs in read_from_mpl. */
//    inv_enable_hal_outputs();
    inv_enable_eMPL_outputs();

    result = inv_start_mpl();
    if (result == INV_ERROR_NOT_AUTHORIZED)
    {
        while (1)
        {
        	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, RESET);
        	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET);
            while(1){
                MPL_LOGE("Not authorized to start MPL .\n");
                HAL_Delay(1000);
               }

        }
    }
    if (result)
    {
    	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, RESET);
    	HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, SET);
        while(1){
            MPL_LOGE("Could not start the MPL.\n");
            HAL_Delay(1000);

           }
    }
    else
    {
    		MPL_LOGE("MPL started successfully.\n");
    }

    /*! Get/set hardware configuration. Start gyro. */
    /*! Wake up all sensors. */

    mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL | INV_XYZ_COMPASS);

    /*! Push both gyro and accel data into the FIFO. */
    mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    mpu_set_sample_rate(DEFAULT_MPU_HZ);


    /*! Read back configuration in case it was set improperly. */
    mpu_get_sample_rate(&gyro_rate);

    mpu_get_gyro_fsr(&gyro_fsr);
    mpu_get_accel_fsr(&accel_fsr);


    /*! Sync driver configuration with MPL. */
    /*! Sample rate expected in microseconds. */
    inv_set_gyro_sample_rate(1000000L / gyro_rate);
    inv_set_accel_sample_rate(1000000L / gyro_rate);


#ifdef COMPASS_ENABLED
    /* The compass rate is independent of the gyro and accel rates. As long as
     * inv_set_compass_sample_rate is called with the correct value, the 9-axis
     * fusion algorithm's compass correction gain will work properly.
     */
    inv_set_compass_sample_rate(DEFAULT_MAG_HZ);
    mpu_get_compass_fsr(&compass_fsr);
#endif
    /*! Set chip-to-body orientation matrix.
     * Set hardware units to dps/g's/degrees scaling factor.
     */
    inv_set_gyro_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)gyro_fsr << 15);


    inv_set_accel_orientation_and_scale(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation),
        (long)accel_fsr << 15);


#ifdef COMPASS_ENABLED
    inv_set_compass_orientation_and_scale(
        inv_orientation_matrix_to_scalar(compass_pdata.orientation),
        (long)compass_fsr << 15);
#endif


    /*! Initialize HAL state variables. */
#ifdef COMPASS_ENABLED
    hal.sensors = ACCEL_ON | GYRO_ON | COMPASS_ON;
#else
    hal.sensors = ACCEL_ON | GYRO_ON;
#endif
    hal.dmp_on = 0;
    hal.report = PRINT_EULER;
    hal.next_pedo_ms = 0;
    hal.next_compass_ms = 0;
    hal.next_temp_ms = 0;

    /*! Compass reads are handled by scheduler. */
     timestamp = get_ms();

    /*! To initialize the DMP:
     * 1. Call dmp_load_motion_driver_firmware(). This pushes the DMP image in
     *    inv_mpu_dmp_motion_driver.h into the MPU memory.
     * 2. Push the gyro and accel orientation matrix to the DMP.
     * 3. Register gesture callbacks. Don't worry, these callbacks won't be
     *    executed unless the corresponding feature is enabled.
     * 4. Call dmp_enable_feature(mask) to enable different features.
     * 5. Call dmp_set_fifo_rate(freq) to select a DMP output rate.
     * 6. Call any feature-specific control functions.
     *
     * To enable the DMP, just call mpu_set_dmp_state(1). This function can
     * be called repeatedly to enable and disable the DMP at runtime.
     *
     * The following is a short summary of the features supported in the DMP
     * image provided in inv_mpu_dmp_motion_driver.c:
     * DMP_FEATURE_LP_QUAT: Generate a gyro-only quaternion on the DMP at
     * 200Hz. Integrating the gyro data at higher rates reduces numerical
     * errors (compared to integration on the MCU at a lower sampling rate).
     * DMP_FEATURE_6X_LP_QUAT: Generate a gyro/accel quaternion on the DMP at
     * 200Hz. Cannot be used in combination with DMP_FEATURE_LP_QUAT.
     * DMP_FEATURE_TAP: Detect taps along the X, Y, and Z axes.
     * DMP_FEATURE_ANDROID_ORIENT: Google's screen rotation algorithm. Triggers
     * an event at the four orientations where the screen should rotate.
     * DMP_FEATURE_GYRO_CAL: Calibrates the gyro data after eight seconds of
     * no motion.
     * DMP_FEATURE_SEND_RAW_ACCEL: Add raw accelerometer data to the FIFO.
     * DMP_FEATURE_SEND_RAW_GYRO: Add raw gyro data to the FIFO.
     * DMP_FEATURE_SEND_CAL_GYRO: Add calibrated gyro data to the FIFO. Cannot
     * be used in combination with DMP_FEATURE_SEND_RAW_GYRO.
     */
    dmp_load_motion_driver_firmware();
    dmp_set_orientation(
        inv_orientation_matrix_to_scalar(gyro_pdata.orientation));
    dmp_register_tap_cb(tap_cb);
    dmp_register_android_orient_cb(android_orient_cb);
    /*!
     * Known Bug -
     * DMP when enabled will sample sensor data at 200Hz and output to FIFO at the rate
     * specified in the dmp_set_fifo_rate API. The DMP will then sent an interrupt once
     * a sample has been put into the FIFO. Therefore if the dmp_set_fifo_rate is at 25Hz
     * there will be a 25Hz interrupt from the MPU device.
     *
     * There is a known issue in which if you do not enable DMP_FEATURE_TAP
     * then the interrupts will be at 200Hz even if fifo rate
     * is set at a different rate. To avoid this issue include the DMP_FEATURE_TAP
     *
     * DMP sensor fusion works only with gyro at +-2000dps and accel +-2G ???!!!
     */


    hal.dmp_features = DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_SEND_RAW_ACCEL | DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_GYRO_CAL | DMP_FEATURE_TAP;
    dmp_enable_feature(hal.dmp_features);
    dmp_set_fifo_rate(DEFAULT_MPU_HZ);

    mpu_set_dmp_state(1);
    hal.dmp_on = 1;


    // reach safe point (Turn on GREEN LED)
	HAL_GPIO_WritePin(YELLOW_LED_GPIO_Port, YELLOW_LED_Pin, RESET);
	HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, SET);
}



void eMPL_APP_Process(void){


	unsigned long sensor_timestamp;

	        timestamp = get_ms();

	#ifdef COMPASS_ENABLED

	        /*! We're not using a data ready interrupt for the compass, so we'll
	         * make our compass reads timer-based instead.
	         */

	        if ((timestamp > hal.next_compass_ms)  && hal.new_frame && (hal.sensors & COMPASS_ON))
	        {
	            hal.next_compass_ms = timestamp + MAG_READ_MS;
	            new_compass = 1;
	        }

	#endif
	        /*! Temperature data doesn't need to be read with every gyro sample.
	         * Let's make them timer-based like the compass reads.
	         */
	        if (timestamp > hal.next_temp_ms)
	        {
	            hal.next_temp_ms = timestamp + TEMP_READ_MS;
	            new_temp = 1;
	        }
	        /**
	         * If there no enabled sensors or a new frame packet is not ready yet, execution mustn't proceed so far.
	         */
	        if (!hal.sensors || !hal.new_frame)		return;



	        if (hal.new_frame)
	        {

	        	hal.new_frame = 0;
	            short gyro[3], accel_short[3], sensors;
	            unsigned char more, unvalid_fifo;
	            long accel[3], quat[4], temperature;


	            /*! This function gets new data from the FIFO when the DMP is in
	             * use. The FIFO can contain any combination of gyro, accel,
	             * quaternion, and gesture data. The sensors parameter tells the
	             * caller which data fields were actually populated with new data.
	             * For example, if sensors == (INV_XYZ_GYRO | INV_WXYZ_QUAT), then
	             * the FIFO isn't being filled with accel data.
	             * The driver parses the gesture data to determine if a gesture
	             * event has occurred; on an event, the application will be notified
	             * via a callback (assuming that a callback function was properly
	             * registered). The more parameter is non-zero if there are
	             * leftover packets in the FIFO.
	             */

	            if (hal.dmp_on) unvalid_fifo = dmp_read_fifo(gyro, accel_short, quat, &sensor_timestamp, &sensors, &more);

	            else	        unvalid_fifo = mpu_read_fifo(gyro, accel_short, &sensor_timestamp,(unsigned char*) &sensors, &more);

	            if(!unvalid_fifo){

					if(more)  hal.new_frame = 1;

					if (sensors & INV_XYZ_GYRO)
					{
						/*! Push the new data to the MPL. */
						inv_build_gyro(gyro, sensor_timestamp);
						hal.new_data = 1;

						/*! Temperature only used for gyro temp comp. */
						if (new_temp)
						{
							new_temp = 0;
							mpu_get_temperature(&temperature, &sensor_timestamp);
							inv_build_temp(temperature, sensor_timestamp);
						}
					}

					if (sensors & INV_XYZ_ACCEL)
					{
						accel[0] = (long)accel_short[0];
						accel[1] = (long)accel_short[1];
						accel[2] = (long)accel_short[2];

						inv_build_accel(accel, INV_RAW_DATA ,sensor_timestamp);
						hal.new_data = 1;
					}

					if (sensors & INV_WXYZ_QUAT)
					{
						inv_build_quat(quat, INV_BIAS_APPLIED | INV_NEW_DATA, sensor_timestamp);
						hal.new_data = 1;
					}
	            }
	        }

	#ifdef COMPASS_ENABLED

	        if (new_compass)
	        {
	            short compass_short[3];
	            long compass[3];
	            new_compass = 0;
	            /*! For any MPU device with an AKM on the auxiliary I2C bus, the raw
	             * magnetometer registers are copied to special gyro registers.
	             */

	            if (!mpu_get_compass_reg(compass_short, &sensor_timestamp))
	            {
	                compass[0] = (long)compass_short[0];
	                compass[1] = (long)compass_short[1];
	                compass[2] = (long)compass_short[2];

	                /*! NOTE: If using a third-party compass calibration library,
	                 * pass in the compass data in uT * 2^16 and set the second
	                 * parameter to INV_CALIBRATED | acc, where acc is the
	                 * accuracy from 0 to 3.
	                 */
	                inv_build_compass(compass, 3, sensor_timestamp);
	                hal.new_data = 1;
	            }

	        }
	#endif

	        if (hal.new_data)
	        {
	        	hal.new_data = 0;

	            inv_execute_on_data();
	            /*! This function reads bias-compensated sensor data and sensor
	             * fusion outputs from the MPL. The outputs are formatted as seen
	             * in eMPL_outputs.c. This function only needs to be called at the
	             * rate requested by the host.
	             */
	            read_from_mpl();
	        }
}







