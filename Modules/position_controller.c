#include "position_controller.h"
#include "../Devices/data_link.h"
#include "position_estimator.h"
#include "commander.h"
#include "attitude_estimator.h"
#include "../Mathlib/comparison.h"
#include "cmsis_os.h"
#include "../Commons/platform.h"
#include "../Modules/commons.h"
#define POS_CTRL_TASK_STACKSIZE (3 * configMINIMAL_STACK_SIZE)
#define POS_CTRL_TASK_PRI 2
#define POS_CTRL_TASK_PERIOD_MS 4
#define POS_CTRL_TASK_PERIOD_S 0.004f
#define POS_CTRL_TASK_FREQ 250

#define MAX_XY_VEL 5.0f
#define MAX_ALT_VEL 2.0f
#define VEL_FF_XY_P 0.1f
#define VEL_FF_Z_P 0.6f
#define ACC_FF_P 1.0f

//#define GRAVITY 9.81f
//#define VEHICLE_MASS 1.289f
#define TILT_MAX 1.0f
#define THR_MIN 4.0f
//#define TILT_COS_MAX 0.3f
static att_t _att;
static pos_t _pos;
//static pos_t _vicon;
static attsp_t _attsp;
static posCtrlsp_t _possp;
//static altCtrlsp_t _alt_ctrl_sp;
static manCtrlsp_t _man_ctrl_sp;
static xQueueHandle attsp_q;
static void position_control_Task( void *pvParameters );

PID_t altPID = {0,0,0,0,
		1.55,
		2.05, 0.16, 0.06};
PID_t pos_xPID = {0,0,0,0,
		1.027,
		5.16, 0.0,0.002};
PID_t pos_yPID = {0,0,0,0,
		1.027,
		5.16, 0.0,0.002};

void manuel_controller(attsp_t *attsp, const manCtrlsp_t * sp)
{
	quaternion2rotation(&sp->Q, &attsp->R);
	//throttle 0~1
	attsp->thrust = sp->thrust;
//	attsp->thrust = sp->throttle * GRAVITY * VEHICLE_MASS * 2.0f;
//	data2send[15] = attsp->thrust;
	
}
/*
void position_controller(const pos_t *pos, const att_t *att, const posCtrlsp_t *possp, attsp_t *attsp, float dt)
{
	float att_comp;
	float Rzz = att->R.R[2][2];
	float vel_z_sp;
	float tilt_cos_max = arm_cos_f32(TILT_MAX);
	vel_z_sp = constrain_f(external_err_pid(&altPID, possp->pos_sp.z - pos->pos.v[2]), -MAX_ALT_VEL, MAX_ALT_VEL);
	vel_z_sp +=  VEL_FF_Z_P * possp->vel_ff.z;
//	if(g_mission == Takeoff && takeoff_switch)
//	{
//		vel_z_sp = possp->vel_ff.z;
//	}
	float acc_z_sp = -GRAVITY + internal_err_pid(&altPID, vel_z_sp - pos->vel.v[2], POS_CTRL_TASK_PERIOD_S);
	
	if(Rzz > tilt_cos_max){//small angle
		if(Rzz>0.9f){
			att_comp = 1.0f / Rzz;
		}
		else if(Rzz>0.8f){
			att_comp = (1.8f - Rzz * 0.8f)/ Rzz;//1.025f / Rzz;
		}
		else if(Rzz>0.65f){
			att_comp = (1.8f - Rzz * 0.8f)/ Rzz;//att_comp = 1.05f / Rzz;
		}
	}
	else if(Rzz > 0.0f)//larger angle
		att_comp = ((1.0f / tilt_cos_max - 1.0f) / tilt_cos_max) * Rzz + 1.0f;
	else
		att_comp = 1.0f;
	attsp->thrust = -acc_z_sp * att_comp * VEHICLE_MASS;
//	if((g_statusFlight == statusLanded) && (reset_takeoff == true))
//	{
//		attsp->thrust = GRAVITY * VEHICLE_MASS*0.5f;
//	}
	
	
	vec3f_t vel_sp, acc_sp, body_acc_sp;
//	vec3f_t body_x_sp, body_y_sp, body_z_sp;
	vec3f_t euler_sp;
//	vec3f_t thrust_sp;//in earth frame

	vel_sp.v[0] = constrain_f(external_err_pid(&pos_xPID, possp->pos_sp.v[0] - pos->pos.v[0]), -MAX_XY_VEL, MAX_XY_VEL);
	vel_sp.v[1] = constrain_f(external_err_pid(&pos_yPID, possp->pos_sp.v[1] - pos->pos.v[1]), -MAX_XY_VEL, MAX_XY_VEL);

	//vel_sp.v[0] += VEL_FF_XY_P * possp->vel_ff.v[0];
	//vel_sp.v[1] += VEL_FF_XY_P * possp->vel_ff.v[1];
	acc_sp.v[0] = internal_err_pid(&pos_xPID, vel_sp.v[0] - pos->vel.v[0], POS_CTRL_TASK_PERIOD_S);
	acc_sp.v[1] = internal_err_pid(&pos_yPID, vel_sp.v[1] - pos->vel.v[1], POS_CTRL_TASK_PERIOD_S);
	earth2body(&att->R, &acc_sp, &body_acc_sp, 2);
	euler_sp.P = constrain_f(-atan2(body_acc_sp.x, GRAVITY),-0.5,0.5);
	euler_sp.R = constrain_f(atan2(body_acc_sp.y, GRAVITY),-0.5,0.5);
	euler_sp.Y = possp->yaw_sp;
	euler2rotation(&euler_sp, &attsp->R);
}
*/

void position_controller(const pos_t *pos, const att_t *att, const posCtrlsp_t *possp, attsp_t *attsp, float dt)
{
	vec3f_t vel_sp, acc_sp, z_b, y_c;
	vec3f_t body_x_sp, body_y_sp, body_z_sp;
	float t_pitch_sp;
	float t_roll_sp;
//	vec3f_t thrust_sp;//in earth frame
	int i;
	float norm_temp;
	vel_sp.v[0] = constrain_f(external_err_pid(&pos_xPID, possp->pos_sp.v[0] - pos->pos.v[0]), -MAX_XY_VEL, MAX_XY_VEL);
	vel_sp.v[1] = constrain_f(external_err_pid(&pos_yPID, possp->pos_sp.v[1] - pos->pos.v[1]), -MAX_XY_VEL, MAX_XY_VEL);
	vel_sp.v[2] = constrain_f(external_err_pid(&altPID, possp->pos_sp.v[2] - pos->pos.v[2]), -MAX_ALT_VEL, MAX_ALT_VEL);
	//vel_sp.v[0] += VEL_FF_XY_P * possp->vel_ff.v[0];
	//vel_sp.v[1] += VEL_FF_XY_P * possp->vel_ff.v[1];
	vel_sp.v[2] += VEL_FF_Z_P * possp->vel_ff.v[2];
	acc_sp.v[0] = internal_err_pid(&pos_xPID, vel_sp.v[0] - pos->vel.v[0], POS_CTRL_TASK_PERIOD_S);
	acc_sp.v[1] = internal_err_pid(&pos_yPID, vel_sp.v[1] - pos->vel.v[1], POS_CTRL_TASK_PERIOD_S);
	acc_sp.v[2] = -GRAVITY + internal_err_pid(&altPID, vel_sp.v[2] - pos->vel.v[2], POS_CTRL_TASK_PERIOD_S);
	acc_sp.v[0] += ACC_FF_P * possp->acc_ff.v[0];
	acc_sp.v[1] += ACC_FF_P * possp->acc_ff.v[1];
	acc_sp.v[2] += ACC_FF_P * possp->acc_ff.v[2];
	acc_sp.v[0] = constrain_f(acc_sp.v[0],-4,4);
	acc_sp.v[1] = constrain_f(acc_sp.v[1],-4,4);
	
	#if SEND_POS
		data2send[6] = vel_sp.v[0]*1000;
		data2send[8] = vel_sp.v[1]*1000;
	#endif
	for(i=0; i<3; i++)
		z_b.v[i] = att->R.R[i][2];
	attsp->thrust = -vec3f_dot(&acc_sp, &z_b) * VEHICLE_MASS;
	if(possp->commands == 1||possp->commands == 0){
		attsp->thrust = -10.0f;
	}
	else if (possp->commands == 2){
		;
	}
	if(att->R.R[2][2] < 0)
		attsp->thrust = -10.0f;
//	if((g_statusFlight == statusLanded) && (reset_takeoff == true))
//	{
//		attsp->thrust = GRAVITY * VEHICLE_MASS*0.5f;
//	}
//	for(i=0; i<3; i++)
//		thrust_sp.v[i] = acc_sp.v[i] * VEHICLE_MASS;
	if(fabsf(acc_sp.v[2]) < THR_MIN){
		acc_sp.v[2] = -THR_MIN;
	}
	norm_temp = vec3f_length(&acc_sp);
	if(norm_temp > 1.0f){//should be normaly 9.8
		for(i=0; i<3; i++)
			body_z_sp.v[i] = -acc_sp.v[i] / norm_temp;
	}
	else{
		body_z_sp.v[0] = 0.0f;
		body_z_sp.v[1] = 0.0f;
		body_z_sp.v[2] = 1.0f;
	}
	y_c.x = -arm_sin_f32(possp->yaw_sp);
	y_c.y = arm_cos_f32(possp->yaw_sp);
	y_c.z = 0;
	if(fabsf(body_z_sp.v[2]) > 0.05f){
		vec3f_cross(&y_c, &body_z_sp, &body_x_sp);
		if(body_z_sp.v[2] < 0.0f){
//keep nose to front while inverted upside down
			for(i=0; i<3; i++)
				body_z_sp.v[i] = -body_z_sp.v[i];
		}
		norm_temp = vec3f_length(&body_x_sp);
		for(i=0; i<3; i++)
			body_x_sp.v[i] = body_x_sp.v[i] / norm_temp;
	}
	else{
//desired thrust is in XY plane, 
//set X downside to construct correct matrix,
// but yaw component will not be used actually
		body_x_sp.v[0] = 0.0f;
		body_x_sp.v[1] = 0.0f;
		body_x_sp.v[2] = 1.0f;
	}
	vec3f_cross(&body_z_sp, &body_x_sp, &body_y_sp);
	for (int i = 0; i < 3; i++) {
		attsp->R.R[i][0] = body_x_sp.v[i];
		attsp->R.R[i][1] = body_y_sp.v[i];
		attsp->R.R[i][2] = body_z_sp.v[i];
	}

	t_pitch_sp = -asin(attsp->R.R[2][0]);
	t_roll_sp = atan2(attsp->R.R[2][1], attsp->R.R[2][2]);
	#if XBEE_POS
		data2send[15] = t_roll_sp*573;
		data2send[16] = t_pitch_sp*573;
		data2send[17] = attsp->thrust * 1000;
	#endif
}

static void position_control_Task( void *pvParameters )
{
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(1) {
		vTaskDelayUntil(&lastWakeTime, POS_CTRL_TASK_PERIOD_MS);
//		if(g_statusFlight != statusInitiating){
		attAcquire(&_att);
		//viconAcquire(&_vicon);
		posAcquire(&_pos);
			
		if(g_mode == modeMan){
			manSetpointAcquire(&_man_ctrl_sp);
			manuel_controller(&_attsp, &_man_ctrl_sp);
		}
//		else if(g_mode == modeAlt){
//			altSetpointAcquire(&_alt_ctrl_sp);
//		}
		else if(g_mode == modePos){
			posSetpointAcquire(&_possp);
			position_controller(&_pos, &_att, &_possp, &_attsp, POS_CTRL_TASK_PERIOD_S);
			#if XBEE_POS
			for(int i=0;i<3;i++){
				data2send[12+i] = _possp.pos_sp.v[i]*1000;
			}
			#endif
		}

		_attsp.timestamp = xTaskGetTickCount();
		xQueueOverwrite(attsp_q, &_attsp);
//	}
	}
}
void attsp_reset(const att_t* att, attsp_t* attsp)
{
	int i, j;
	for(i=0;i<3;i++){
		for(j=0;j<3;j++){
			attsp->R.R[i][j] = att->R.R[i][j];
		}
	}
	attsp->yaw_ff = 0;
	attsp->thrust = 0;
}
void position_controller_queue_init(void)
{
	attsp_q = xQueueCreate(1, sizeof(attsp_t));
//	param_pid_read(&pos_xPID,'x');
//	param_pid_read(&pos_yPID,'y');
//	param_pid_read(&altPID,'z');
}
void position_controller_init(const att_t* att)
{
	attsp_reset(att, &_attsp);
	xTaskCreate(position_control_Task, "posCtrl", POS_CTRL_TASK_STACKSIZE, NULL, POS_CTRL_TASK_PRI, NULL);
}
BaseType_t attspAcquire(attsp_t *attsp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(attsp_q, attsp, 0);
	return pdres;
}
BaseType_t attspBlockingAcquire(attsp_t *attsp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueueReceive(attsp_q, attsp, portMAX_DELAY);
	return pdres;
}
