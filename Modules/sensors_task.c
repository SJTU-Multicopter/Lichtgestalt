#include "../Devices/hmc5883l_i2c.h"
#include "../Devices/ms5611_i2c.h"
#include "../Devices/mpu6000_spi.h"
#include "../MathLib/low_pass_filter.h"
#include "sensors_task.h"

#include "cmsis_os.h"
#include "../Devices/rom.h"
#include "../config/config.h"
#include "../MessageTypes/type_methods.h"

uint8_t acc_gyr_spi_rx[15];
uint8_t mag_i2c_rx[6];
	
static vec3i16_t acc_raw;
static vec3i16_t gyr_raw;	
static vec3i16_t mag_raw;

static vec3i16_t acc_bias={0,0,0};
static vec3i16_t gyr_bias={0,0,0};	
static vec3i16_t mag_bias={0,0,0};//{0,-150,110};
static baro_t baro_data;
static marg_t marg_data;
unsigned char curr_i2c_dev;
static xQueueHandle marg_q;
static xSemaphoreHandle imuDataReady;
static bool magDataReady;
//static bool baroDataReady;
//extern short data2send[9];
IIRFilter iir_ax={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_ay={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_az={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_gx={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_gy={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
IIRFilter iir_gz={0.0,0.0,0.0,
				0.0,0.0,0.0,
				0.0,0.0};
static void sensorsTrigerTask( void *pvParameters ) ;
static void sensorsProcessTask( void *pvParameters ) ;
void acc_process(vec3i16_t* input, vec3f_t* output);
void gyr_process(vec3i16_t* input, vec3f_t* output);	
void mag_process(vec3i16_t* input, vec3f_t* output);
				
void imu_IIR_init(void)
{	
	float acc_cutoff_freq = 40.0;//, cutoff_freq2 = 40.0;
	float gyr_cutoff_freq = 40.0;
	
	float smpl_freq = 1000.0;
	float dummy;
	IIR_set_cutoff_freq(&iir_ax, acc_cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_ay, acc_cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_az, acc_cutoff_freq, smpl_freq);
	dummy = IIR_reset(&iir_ax, 0);
	dummy = IIR_reset(&iir_ay, 0);
	dummy = IIR_reset(&iir_az, 8192);
	IIR_set_cutoff_freq(&iir_gx, gyr_cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_gy, gyr_cutoff_freq, smpl_freq);
	IIR_set_cutoff_freq(&iir_gz, gyr_cutoff_freq, smpl_freq);
	dummy = IIR_reset(&iir_gx, 0);
	dummy = IIR_reset(&iir_gy, 0);
	dummy = IIR_reset(&iir_gz, 0);
	dummy = dummy;
	
}
void sensorsTaskInit(void)
{
	marg_q = xQueueCreate(1, sizeof(marg_t));
	imu_IIR_init();
	baro_data.status = baro_initializing;
	rom_get_mag_bias(&mag_bias);
	rom_get_acc_bias(&acc_bias);
	vSemaphoreCreateBinary( imuDataReady );
	xTaskCreate(sensorsTrigerTask, "sensTrigTask", SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
	xTaskCreate(sensorsProcessTask, "sensProcessTask", SENSORS_TASK_STACKSIZE, NULL, SENSORS_TASK_PRI, NULL);
}
void sensorsTrigerTask( void *pvParameters )
{
	uint32_t i2c_cnt = 0;
	uint32_t baro_cnt = 0;
	uint32_t baro_init_cnt = 0;
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 1;//1ms
	xLastWakeTime = xTaskGetTickCount();
	baro_data.refPressure = 0;
	for( ;; ){  
		mpu6000_dma_start(acc_gyr_spi_rx, 15);
		i2c_cnt ++;
		if(i2c_cnt == 14){
			i2c_cnt = 0;
			hmc5883l_dma_start(mag_i2c_rx, 6);
			curr_i2c_dev = I2C_DEV_HMC5883;
		}
		else if(i2c_cnt == 2){
			curr_i2c_dev = I2C_DEV_MS5611;
			if(baro_data.status == baro_initializing){
				if(baro_cnt == 0){
					ms5611_temp_start();
				}
				else{
					ms5611_pres_start();
				}
				baro_cnt = (!baro_cnt) & 1;
			//	baro_data.i2cStatus = baro_convert;
			}
			else{
				if(baro_cnt == 0){
					ms5611_temp_start();
				}
				else{
					ms5611_pres_start();
				}
				baro_cnt++;
				if(baro_cnt==5)
					baro_cnt = 0;
			}
		}
		else if(i2c_cnt == 11){
			curr_i2c_dev = I2C_DEV_MS5611;
			ms5611_read_trigger();
		//	baro_data.i2cStatus = baro_read;
		}
		else if(i2c_cnt == 13){
			curr_i2c_dev = I2C_DEV_MS5611;
			baroProcess(&baro_data);
			if(baro_data.status == baro_initializing){
				baro_init_cnt++;
				if(baro_init_cnt>100)
					baro_data.refPressure += baro_data.pressure;
				if(baro_init_cnt == 200){
					baro_data.refPressure /= 100;
					baro_data.status = baro_running;
				}
			}else{
				
				
			}
		}
		vTaskDelayUntil(&xLastWakeTime, timeIncreament); 
	}  
}
void sensorsProcessTask( void *pvParameters )
{
//	static int j = 0;//varify if no repetation of raw data
	for( ;; ){  
		if (pdTRUE == xSemaphoreTake(imuDataReady, portMAX_DELAY)){
			int i;
			for (i=0; i<3; i++){
				acc_raw.v[i] = ((short)acc_gyr_spi_rx[2*i+1]<<8)|(short)acc_gyr_spi_rx[2*i+2];
			}
			for (i=0; i<3; i++){
				gyr_raw.v[i] = ((short)acc_gyr_spi_rx[2*i+9]<<8)|(short)acc_gyr_spi_rx[2*i+10];
			}
			acc_process(&acc_raw, &(marg_data.acc));
			gyr_process(&gyr_raw, &(marg_data.gyr));
			if (magDataReady == true){
				magDataReady = false;
				int i;
				for (i=0; i<3; i++){
					mag_raw.v[i] = ((short)mag_i2c_rx[2*i]<<8)|(short)mag_i2c_rx[2*i+1];
				}

//				mag_raw.timestamp = xTaskGetTickCount();
				mag_process(&mag_raw, &(marg_data.mag));
				marg_data.mag_updated = true;
			}
			xQueueOverwrite(marg_q, &marg_data);
			marg_data.mag_updated = false;
		}
	}  
}
void gyro_calibrate(vec3f_t* avr_gyr)
{
	unsigned int i;
	for(i = 0; i < 3; i++){
		gyr_bias.v[i] = -avr_gyr->v[i];
	}
}
void acc_process(vec3i16_t* input, vec3f_t* output)
{
	//swap, bias, lpf
	float x,y,z;
	x = (input->x + acc_bias.x)*ACC_SCALE;
	y = (-input->y + acc_bias.y)*ACC_SCALE;
	z = (-input->z + acc_bias.z)*ACC_SCALE;
//	output->x = x;
//	output->y = y;
//	output->z = z;
	
	output->x = IIR_apply(&iir_ax, x);
	output->y = IIR_apply(&iir_ay, y);
	output->z = IIR_apply(&iir_az, z);
	
}
void gyr_process(vec3i16_t* input, vec3f_t* output)
{
	//swap, bias, lpf
	float x,y,z;
	x = (input->x + gyr_bias.x) / GYR_SCALE;
	y = (-input->y + gyr_bias.y) / GYR_SCALE;
	z = (-input->z + gyr_bias.z) / GYR_SCALE;
//	output->x = x;
//	output->y = y;
//	output->z = z;
	output->x = IIR_apply(&iir_gx, x);
	output->y = IIR_apply(&iir_gy, y);
	output->z = IIR_apply(&iir_gz, z);
}
void mag_process(vec3i16_t* input, vec3f_t* output)
{
	//swap, bias
	float x,y,z;
	x = -input->x + mag_bias.x;
	y = -input->z + mag_bias.y;
	z = input->y + mag_bias.z;
	output->x = x;
	output->y = y;
	output->z = z;
}
void mpu6000Callback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(imuDataReady, &xHigherPriorityTaskWoken);
	if (xHigherPriorityTaskWoken)
		portYIELD();
}

void hmc5883lCallback(void)
{
	magDataReady = true;
}
void ms5611Callback(void)
{
//	if(baro_data.status == 
//	baroDataReady = true;
}

void margAcquire(marg_t *marg)
{
	xQueuePeek(marg_q, marg, 0);
}
