#include "commander.h"
#include "commons.h"
#include "../MessageTypes/type_methods.h"
#include "../Mathlib/comparison.h"
#include "cmsis_os.h"
#include "../config/config.h"
#include "../Devices/data_link.h"
#include "../Devices/receiver.h"
#include "../Commons/platform.h"
#define MAX_ATT_MANUEL 0.5f//11437 40deg,0.698rad
#define MAX_YAW_RATE 1.4f//14303 50.0
#define MAX_YAW_RATE_MANEUL 1.4f
#define MAX_Z_RATE_MANEUL 1.0f
#define MAX_XY_RATE_MANEUL 5.0f
#define YAW_RATE_DEADZONE 0.17f
#define Z_RATE_DEADZONE 0.2f
#define XY_RATE_DEADZONE 0.3f
#define RTL_ALTITUDE 30.f
#define RTL_HOVERING_ALTITUDE 3.0f
#define BREAKING_ACC_MAX 2.0f
#define ALT_CONSTRAIN -80.0f
#define MAN_ALT_HIDE 1

static xQueueHandle mansp_q;
static manCtrlsp_t mansp;

static xQueueHandle possp_q;
static posCtrlsp_t possp;

static rc_t rc;
//static vec3f_t euler_sp;
#if CMD_XBEE
#if XBEE_API

static attCmd_t att_cmd;

static posCmd_t pos_cmd;

#endif
#else

//short rc_int[16];
#endif
static void commanderTask( void *pvParameters ) ;
void commanderInit(void)
{
	mansp_q = xQueueCreate(1,sizeof(manCtrlsp_t));
	possp_q = xQueueCreate(1,sizeof(posCtrlsp_t));

	#if XBEE_API
	xTaskCreate(commanderTask, "cmdTask", CMD_TASK_STACKSIZE, NULL, CMD_TASK_PRI, NULL);
	#endif
}
void xbee_attCmd2manCtrlsp(manCtrlsp_t* sp, const attCmd_t* cmd)
{
	for(int i=0;i<4;i++){
		sp->Q.q[i] = cmd->Q.q[i];	
	}
	sp->thrust = cmd->thrust;
}
void xbee_posCmd2posSp(posCtrlsp_t* sp, const posCmd_t* cmd)
{
	for(int i=0;i<3;i++){
		sp->pos_sp.v[i] = cmd->pos_sp.v[i];
		sp->vel_ff.v[i] = cmd->vel_ff.v[i];
		sp->acc_ff.v[i] = cmd->acc_ff.v[i];
	}
	sp->commands = cmd->commands;
	sp->yaw_sp = cmd->yaw_sp;
}
/*
void rc_channel2attsp(manCtrlsp_t* sp, const rc_t* rc, float dt)
{
	float yaw_moverate;
	float thrust;
	float throttle;//0~1.0f
//	for(int i=0;i<16;i++){
//		rc_int[i] = rc->channels[i];
//	}
	vec3f_t euler_sp;
	euler_sp.P = rc->channels[1] * MAX_ATT_MANUEL / 1024.0f;
	euler_sp.R = -rc->channels[0] * MAX_ATT_MANUEL / 1024.0f;
	yaw_moverate = dead_zone_f(rc->channels[3] * MAX_YAW_RATE_MANEUL / 1024.0f, YAWRATE_DEADZONE);
	euler_sp.Y += yaw_moverate * dt;		
	throttle = dead_zone_f((rc->channels[2] + 1024)/2048.0f, 0.05);
	thrust = VEHICLE_MASS * GRAVITY * throttle * 2.0f;
	
	euler2rotation(&euler_sp, &sp->R);
	sp->thrust = thrust;
}
*/
void commanderTask( void *pvParameters )
{
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 20;
	uint32_t last_timestamp = 0;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){

	#if XBEE_API
		if(g_mode == modeMan){
			xbee_commandBlockingAcquire(&att_cmd);
			xbee_attCmd2manCtrlsp(&mansp, &att_cmd);
			mansp.timestamp = xTaskGetTickCount ();
			xQueueOverwrite(mansp_q, &mansp);
		}
		else if (g_mode == modePos){
			if(g_statusLink != link_broken_in_air){
				xbee_outdoor_commandAcquire(&pos_cmd);//rewriten by Wade
				uint32_t this_timestamp = pos_cmd.timestamp;
				if(this_timestamp != last_timestamp){
					last_timestamp = this_timestamp;
					pos_cmd.timestamp = xTaskGetTickCount ();
					xbee_posCmd2posSp(&possp,&pos_cmd);
					possp.timestamp = xTaskGetTickCount();
					xQueueOverwrite(possp_q, &possp);
				}
			}
			else{//link_broken_in_air
				possp.pos_sp.z += 0.6f * timeIncreament / 1000;
				possp.timestamp = xTaskGetTickCount();
				xQueueOverwrite(possp_q, &possp);
			}
		}
		else if(g_mode == modeRC){
			if(g_statusLink != link_broken_in_air){
				xbee_rcAcquire(&rc);//rewriten by Wade
			//	uint32_t this_timestamp = rc.timestamp;
			//	for(int i=0;i<6;i++)
			//		data2send[6+i] = rc.channels[i]*1000;
			}
			else{//link_broken_in_air
				
				
			}
		}
	#endif
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
	}
}
/*
void attsp_reset(const att_t* state)
{
	vec3f_t euler_sp;
	euler_sp.P = state->Euler.P;
	euler_sp.R = state->Euler.R;
	euler_sp.Y = state->Euler.Y;
	euler2rotation(&euler_sp, &attsp.R);
	attsp.thrust = 0;
	xQueueOverwrite(attsp_q, &attsp);
}
void attspAcquire(attsp_t* sp)
{
	xQueuePeek(attsp_q, sp, 0);
}
*/
BaseType_t posSetpointAcquire(posCtrlsp_t *sp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(possp_q, sp, 0);
	return pdres;
}
/*
BaseType_t altSetpointAcquire(altCtrlsp_t *sp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(alt_sp_q, sp, 0);
	return pdres;
}
*/
BaseType_t manSetpointAcquire(manCtrlsp_t *sp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(mansp_q, sp, 0);
	return pdres;
}
