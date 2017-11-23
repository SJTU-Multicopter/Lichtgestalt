#include "motor_mixer.h"
#include "attitude_controller.h"
#include "../Devices/battery.h"
#include "../Devices/motor_pwm.h"
#include "../MessageTypes/type_methods.h"
#include "../Commons/platform.h"
#include "../Mathlib/comparison.h"
//extern short data2send[18];
#include "cmsis_os.h"
#include "../config/config.h"
static output_t _output;
static battery_t _bat;
static void motorMixTask(void* param);
void force2output(float force[4], unsigned short duty[4], unsigned int battery)
//mNewton to 0~2400
{
	int k,i;
	if(battery == 0){
		k = 110;
	} else{
		//full4.20V(4.05V) k=100, low3.80V(3.65V) k=120
		k = constrain_int32(303-(int)battery/20,95,125);
	}
	for(i=0;i<4;i++){
		duty[i] = force[i] * k / MOTOR_POWER;// + 2750;
		duty[i] = constrain_int32(duty[i]* 5 / 12 + MOTOR_THRESHOLD + 1000, 1000,2000);;//originally 2400~4800, now 1000~2000
	}
}
/*			A								inv(A)
			[  1,  1, 1,  1]				[ 1/4, -1/(2*d),        0,  1/(4*c)]
			[ -d,  0, d,  0]				[ 1/4,        0, -1/(2*d), -1/(4*c)]
			[  0, -d, 0,  d]				[ 1/4,  1/(2*d),        0,  1/(4*c)]
			[  c, -c, c, -c]				[ 1/4,        0,  1/(2*d), -1/(4*c)]*/
/*			A													inv(A)
			[        1,        1,        1,       1]			[ 1/4, -sqrt2/(4*d),  sqrt2/(4*d),  1/(4*c)]
			[ -d/sqrt2, -d/sqrt2,  d/sqrt2, d/sqrt2]			[ 1/4, -sqrt2/(4*d), -sqrt2/(4*d), -1/(4*c)]
			[  d/sqrt2, -d/sqrt2, -d/sqrt2, d/sqrt2]			[ 1/4,  sqrt2/(4*d), -sqrt2/(4*d),  1/(4*c)]
			[        c,       -c,        c,      -c]			[ 1/4,  sqrt2/(4*d),  sqrt2/(4*d), -1/(4*c)]
		d=D/2
		torq=c*f*/
void powerDistribution(output_t* output, const battery_t * bat)
{
	float motorForce[4] = {0,0,0,0};
	unsigned short motorDuty[4] = {1000,1000,1000,1000};
	short i;
	if(output->thrust < 0.1f){
		motorForce[0] = 0;
		motorForce[1] = 0;
		motorForce[2] = 0;
		motorForce[3] = 0;
		for(i=0;i<4;i++){
			motorDuty[i]=1000;
		}
	}
	else{
	#if PLUS
		motorForce[0] = (output->thrust*0.25f) * 1000 - output->moment.P * ONE_2_ROTOR_DIST + output->moment.Y * ONE_4_F_T_RATIO;
		motorForce[1] = (output->thrust*0.25f) * 1000 - output->moment.R * ONE_2_ROTOR_DIST - output->moment.Y * ONE_4_F_T_RATIO;
		motorForce[2] = (output->thrust*0.25f) * 1000 + output->moment.P * ONE_2_ROTOR_DIST + output->moment.Y * ONE_4_F_T_RATIO;
		motorForce[3] = (output->thrust*0.25f) * 1000 + output->moment.R * ONE_2_ROTOR_DIST - output->moment.Y * ONE_4_F_T_RATIO;
	#elif CROSS

	#endif	
		if(g_statusLockRC == motorIdle)
		{
			motorForce[3] =  0.1f;
			motorForce[2] =  0.1f;
			motorForce[1] =  0.1f;
			motorForce[0] =  0.1f;
			
		}
//		force2output(motorForce, motorDuty, bat->voltage);
		force2output(motorForce, motorDuty, 4000);
		#if	XBEE_CMD
		data2send[14] = motorForce[0];
		data2send[15] = motorForce[1];
		data2send[16] = motorDuty[2];
		data2send[17] = motorDuty[3];
		#endif
	}
	if(g_statusLock == motorUnlocked && (g_statusLockRC == motorUnlocked||g_statusLockRC == motorIdle))
		motor_pwm_output(motorDuty);
	
	else
		motor_cut();
}
void motor_mixer_init(void)
{
	xTaskCreate(motorMixTask, "motormixTask", MOTORMIX_TASK_STACKSIZE, NULL, MOTORMIX_TASK_PRI, NULL);
}
static void motorMixTask(void* param)
{
	for(;;){
		outputBlockingAcquire(&_output);
		#if	XBEE_CMD
		data2send[10] = _output.thrust*100;
		#endif
		batAcquire(&_bat);
		powerDistribution(&_output, &_bat);
	}
}	
