#include <stdlib.h>
#include <stdio.h>
#include "../MessageTypes/type_methods.h"
//#include "arm_math.h"
#include "cmsis_os.h"

#include "../config/config.h"
#include "../Commons/platform.h"
#include "sensors_calibration.h"
#include "sensors_task.h"

//#include "../MessageTypes/messages.h"
//#include "../MessageTypes/basic_types.h"
#include "../MathLib/calibration_lib.h"
#include "../Mathlib/comparison.h"
#include "../Devices/led.h"
#include "../Devices/data_link.h"
#include "../Devices/rom.h"

//extern short data2send[18];
static vec3i16_t old_mag_bias;
static void magCalTask(void* param);
#if STATIC_CAL_MEM
	float x[MAG_STORE];
	float y[MAG_STORE];
	float z[MAG_STORE];
#endif
void calibration_manager_init(void)
{
	setLed(2, 500, 500);
#if STATIC_CAL_MEM
	unsigned int calibration_stacksize = STABILIZER_TASK_STACKSIZE;
#elif ALLOC_CAL_MEM
	unsigned int calibration_stacksize = STABILIZER_TASK_STACKSIZE + 3 * MAG_STORE * sizeof(float);
#endif
	rom_get_mag_bias(&old_mag_bias);
	xTaskCreate(magCalTask, "magCal", calibration_stacksize, NULL, STABILIZER_TASK_PRI, NULL);
}
static void magCalTask(void* param)
{
#if ALLOC_CAL_MEM
	float* x;
	float* y;
	float* z;
	x = (float*)malloc(MAG_STORE*sizeof(float));
	y = (float*)malloc(MAG_STORE*sizeof(float));
	z = (float*)malloc(MAG_STORE*sizeof(float));
#endif
	//malloc cannot provide enough space
	//not enough stack
	//and will return NULL pointer
//	float gyro_scale = 7509.9f * 0.5f;
	static marg_t marg;
	vec3f_t mag_bias;
	vec3i16_t mag_bias_i;
	float radius;
	unsigned int mag_store_pt = 0;
	uint32_t tick = 0;
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 200;
	xLastWakeTime = xTaskGetTickCount ();
	data_send_start();

	while(1) {
		
		margAcquire(&marg);
	/*	for(int i=0;i<3;i++){
			data2send[i] = mag_bias.v[i];
			data2send[i+3] = marg.mag.v[i];
		}*/
		if(absolute_f(marg.gyr.x)> ROTATION_THRES|| 
			absolute_f(marg.gyr.y)> ROTATION_THRES|| 
			absolute_f(marg.gyr.z)> ROTATION_THRES){
			//take this data down
			if(vec3f_length(&marg.mag)>MAG_LEN_MIN_THRES){
				x[mag_store_pt] = marg.mag.x;
				y[mag_store_pt] = marg.mag.y;
				z[mag_store_pt] = marg.mag.z;
				mag_store_pt++;
				
				if(mag_store_pt==MAG_STORE){
					mag_store_pt = 0;
					sphere_fit_least_squares(x, y, z,
						MAG_STORE,
						100, 0.0f,
						&mag_bias.x, &mag_bias.y, &mag_bias.z,
						&radius);
					mag_bias_i.x = old_mag_bias.x-mag_bias.x;
					mag_bias_i.y = old_mag_bias.y-mag_bias.y;
					mag_bias_i.z = old_mag_bias.z-mag_bias.z;
					rom_set_mag_bias(&mag_bias_i);
					for(int i=0;i<3;i++){
						data2send[i] = mag_bias.v[i];
						data2send[i+3] = marg.mag.v[i];
					}
					setLed(0, 0, 500);
					setLed(1, 0, 500);
					setLed(2, 0, 500);
				#if ALLOC_CAL_MEM
					free(x);
					free(y);
					free(z);
					x = NULL;
					y = NULL;
					z = NULL;
				#endif
					vTaskDelete(NULL);
				}
			}
		}
		
		tick++;
		if(mag_store_pt < MAG_STORE/3){
			setLed(0, 200, 500);
			setLed(1, 0, 500);
			setLed(2, 0, 500);
		}
		else if(mag_store_pt < MAG_STORE*2/3){
			setLed(0, 0, 500);
			setLed(1, 200, 500);
			setLed(2, 0, 500);
		}
		else{
			setLed(0, 0, 500);
			setLed(1, 0, 500);
			setLed(2, 200, 500);
		}
		vTaskDelayUntil(&xLastWakeTime, timeIncreament);
	}
}

