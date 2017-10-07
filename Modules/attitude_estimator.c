#include "attitude_estimator.h"
#include "position_estimator.h"
#include "../Commons/platform.h"
#include "../MessageTypes/type_methods.h"
#include "../Mathlib/comparison.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Devices/led.h"
#include "sensors_task.h"
#include "../Devices/data_link.h"
#include "commander.h"
#include "task_manager.h"
//extern short data2send[9];


float w_mag = 0.2f;
float w_acc = 0.3f;
float w_gyr_bias = 0.031f;
float bias_max = 0.05f;

static marg_t _marg;
static vec3f_t _motion_acc;
static vec3f_t _gyr_bias;
static att_t _att;
static xQueueHandle att_q;
static void attitude_init_Task( void *pvParameters );
static void attitude_update_Task( void *pvParameters );

bool judge_steady(float* block, unsigned int len, float threshold)
{
	float std;
	arm_std_f32(block,len,&std);
	if(std < threshold)
		return true;
	else
		return false;
}
void attitude_reset(const vec3f_t* acc, const vec3f_t* mag, att_t* att)
{
/*	float xh,yh;
	float pitch,roll,yaw; 
	for(int i = 0; i<3; i++)
		_gyr_bias.v[i] = 0;
//	float q0,q1,q2,q3;
	pitch = -data_2_angle(acc->x,acc->y,acc->z);//rad
	roll = data_2_angle(acc->y,acc->x,acc->z); //rad
	xh = mag->y*arm_cos_f32(roll)+mag->x*arm_sin_f32(roll)*arm_sin_f32(pitch)-mag->z*arm_cos_f32(pitch)*arm_sin_f32(roll);		 
	yh = mag->x*arm_cos_f32(pitch)+mag->z*arm_sin_f32(pitch);
	yaw = -atan2(xh,yh)+1.57f;
	if(yaw < -PI )
		yaw = yaw + 2*PI;
	if(yaw > PI )
		yaw = yaw - 2*PI;
	stateAtt->Euler.R = roll;
	stateAtt->Euler.P = pitch;
	stateAtt->Euler.Y = yaw;
	stateAtt->rate.R = 0;
	stateAtt->rate.P = 0;
	stateAtt->rate.Y = 0;
	euler2quaternion(&att->Euler, &att->Q);
	quaternion2rotation(&att->Q, &att->R);
	*/
	vec3f_t k, I, J;
	float mag_dot_k;
	for (int i = 0; i < 3; i++) 
		k.v[i] = -acc->v[i];
	vec3f_normalize(&k);
	mag_dot_k = vec3f_dot(mag, &k);
	for (int i = 0; i < 3; i++) 
		I.v[i] = mag->v[i] - mag_dot_k * k.v[i];
	vec3f_normalize(&I);
	vec3f_cross(&k, &I, &J);
	for (int i = 0; i < 3; i++) {
		att->R.R[0][i] = I.v[i];
		att->R.R[1][i] = J.v[i];
		att->R.R[2][i] = k.v[i];
	}
	rotation2quaternion(&att->R, &att->Q);
	rotation2euler(&att->R, &att->Euler);
	
}
void attitude_update(const vec3f_t *gyr, const vec3f_t *pos_acc, const marg_t *marg, vec3f_t *gyr_bias, att_t *state, float dt)
{
	vec3f_t corr;
	vec3f_t k;
	vec3f_t grav_acc;
	vec3f_t acc_err;
//	vec3f_t rates;
	quaternion_t derQ;
	int i;
	float spinRate = vec3f_length(gyr);
	for(i=0;i<3;i++)
		corr.v[i] = 0;
	if(marg->mag_updated){
		float gainMult = 1.0f;
		const float fifty_dps = 0.873f;
		vec3f_t mag_err_earth;
		vec3f_t mag_err_body;
		vec3f_t mag_earth;
		body2earth(&state->R, &marg->mag, &mag_earth, 3);
		mag_err_earth.z = -atan2f(mag_earth.y, mag_earth.x);
		mag_err_earth.x = 0;
		mag_err_earth.y = 0;
		if (spinRate > fifty_dps) 
			gainMult = minimum_f(spinRate / fifty_dps, 10.0f);
		earth2body(&state->R, &mag_err_earth, &mag_err_body, 3);
		for(i=0;i<3;i++)
			corr.v[i] += mag_err_body.v[i] * gainMult * w_mag;
	}
	quaternion_normalize(&state->Q);
	if(1){
		k.x = 2.0f * (state->Q.q1 * state->Q.q3 - state->Q.q0 * state->Q.q2);
		k.y = 2.0f * (state->Q.q2 * state->Q.q3 + state->Q.q0 * state->Q.q1);
		k.z = 1.0f - ((state->Q.q1 * state->Q.q1 + state->Q.q2 * state->Q.q2) * 2.0f);
		for(i=0;i<3;i++)
			grav_acc.v[i] = marg->acc.v[i]-pos_acc->v[i];
		vec3f_normalize(&grav_acc);
		vec3f_cross(&k, &grav_acc, &acc_err);
		for(i=0;i<3;i++)
			corr.v[i] += acc_err.v[i] * w_acc;
	}
	if (spinRate < 0.175f) {
		for(i=0;i<3;i++)
			gyr_bias->v[i] += corr.v[i] * (w_gyr_bias * dt);
		for (int i = 0; i < 3; i++) 
			gyr_bias->v[i] = constrain_f(gyr_bias->v[i], -bias_max, bias_max);
	}
	for (int i = 0; i < 3; i++) 
		state->rate.v[i] = gyr->v[i] + gyr_bias->v[i];
	for (int i = 0; i < 3; i++) 
		corr.v[i] += state->rate.v[i];
	quaternion_derivative(&state->Q, &derQ, &corr);
	for (int i = 0; i < 4; i++) 
		state->Q.q[i] += derQ.q[i] * dt;
	quaternion_normalize(&state->Q);
	quaternion2rotation(&state->Q, &state->R);
	rotation2euler(&state->R, &state->Euler);
}

static void attitude_init_Task( void *pvParameters )
{
	#define STD_BLOCK_LEN 25
	#define AVERAGE_SAMPLES 150//2sec
	#define ACC_STEADY_STD 0.06f
	#define GYR_STEADY_STD 0.013f
	#define MAG_STEADY_STD 50.0f
	int i;
	unsigned int p_block = 0;
	bool state_steady = false;
	unsigned int usable_data_cnt = 0;
	float acc_block[3][STD_BLOCK_LEN];
	float gyr_block[3][STD_BLOCK_LEN];
	float mag_block[3][STD_BLOCK_LEN];
	vec3f_t avr_acc = {0,0,0};
	vec3f_t avr_gyr = {0,0,0};
	vec3f_t avr_mag = {0,0,0};
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(1) {
		vTaskDelayUntil(&lastWakeTime, ATT_EST_TASK_PERIOD_MS);
		margAcquire(&_marg);
		if(_marg.mag_updated){
			p_block++;
			if(p_block == STD_BLOCK_LEN)
				p_block = 0;
			state_steady = true;
			for(i = 0; i<3; i++){
				acc_block[i][p_block] = _marg.acc.v[i];
				gyr_block[i][p_block] = _marg.gyr.v[i];
				mag_block[i][p_block] = _marg.mag.v[i];
				state_steady &= judge_steady(acc_block[i], STD_BLOCK_LEN, ACC_STEADY_STD);
				state_steady &= judge_steady(gyr_block[i], STD_BLOCK_LEN, GYR_STEADY_STD);
				state_steady &= judge_steady(mag_block[i], STD_BLOCK_LEN, MAG_STEADY_STD);
			}
			if(state_steady){
				setLed(0, 1000, 1000);
				for(i = 0; i<3; i++){
					avr_acc.v[i] += _marg.acc.v[i];
					avr_gyr.v[i] += _marg.gyr.v[i];
					avr_mag.v[i] += _marg.mag.v[i];
				}
				usable_data_cnt++;
			}
			else{
				setLed(0, 0, 1000);
				for(i = 0; i<3; i++){
					avr_acc.v[i] = 0;
					avr_gyr.v[i] = 0;
					avr_mag.v[i] = 0;
				}
				usable_data_cnt = 0;
			}
			if(usable_data_cnt == AVERAGE_SAMPLES){
				setLed(0, 0, 1000);
				for(i = 0; i<3; i++){
					avr_acc.v[i] /= AVERAGE_SAMPLES;
					avr_gyr.v[i] /= AVERAGE_SAMPLES;
					avr_mag.v[i] /= AVERAGE_SAMPLES;
				}
				attitude_reset(&avr_acc, &avr_mag, &_att);
				vec3f_t acc_offset_ned;
				body2earth(&_att.R, &avr_acc, &acc_offset_ned, 3);
				acc_calibrate(&acc_offset_ned, &_att.R);
				gyro_calibrate(&avr_gyr);
				
			//	stabilizerReady2Fly();
				
				attitude_initialized_callback(&_att);
				
				vTaskDelete(NULL);
			}
		}
	}
}
static void attitude_update_Task( void *pvParameters )
{
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(1) {
		vTaskDelayUntil(&lastWakeTime, ATT_EST_TASK_PERIOD_MS);
		margAcquire(&_marg);
		#if XBEE_API
		#if INDOOR
		xbee_motionAccAcquire(&_motion_acc);
		#endif
		#endif
		vec3f_t w;
		int i;
//		float gyro_scale = 7509.9f * 0.5f;
		for(i = 0; i<3; i++){
			w.v[i] = _marg.gyr.v[i];// / gyro_scale;
		}
		attitude_update(&w, &_motion_acc, &_marg, &_gyr_bias, &_att, ATT_EST_TASK_PERIOD_S);
		_att.timestamp = xTaskGetTickCount ();
		xQueueOverwrite(att_q, &_att);
		#if XBEE_SENS_ATT
		baro_t baro;
		baroAcquire(&baro);
		for(int i=0;i<3;i++){
			data2send[i] = _att.Euler.v[i]*573.0f;
			data2send[i+3] = _att.rate.v[i]*573.0f;
			
			data2send[i+9] = _marg.mag.v[i];
			data2send[i+12] = _marg.acc.v[i]*1000;
			data2send[i+15] = _marg.gyr.v[i]*1000;
		}
		data2send[6] = baro.alt*1000;//state.Euler.v[i]*573.0f;
		#endif
	}
}

void attitude_estimator_start(void)
{
	xTaskCreate(attitude_update_Task, "attUpdate", ATT_EST_TASK_STACKSIZE, NULL, ATT_EST_TASK_PRI, NULL);
}
void attitude_init(void)
{
	att_q = xQueueCreate(1, sizeof(att_t));
	xTaskCreate(attitude_init_Task, "attInit", ATT_EST_TASK_STACKSIZE, NULL, ATT_EST_TASK_PRI, NULL);
}
void attAcquire(att_t *att)
{
	xQueuePeek(att_q, att, 0);
}
void attBlockingAcquire(att_t *att)
{
	xQueuePeek(att_q, att, portMAX_DELAY);
}
