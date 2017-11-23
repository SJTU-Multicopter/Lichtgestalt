#include "commander.h"
#include "position_controller.h"
#include "attitude_controller.h"
#include "attitude_estimator.h"
#include "position_estimator.h"
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
bool reset_takeoff = false;
bool reset_return_pos = false;
float return_pos[2] = {0.0f, 0.0f};
mission_t g_mission;
bool armed = false;
bool takeoff_switch = true;
static mode_t _mode;
static mode_t _lastmode;
static rc_t _rc;


static xQueueHandle man_quaternion_sp_q;
static manCtrlsp_t _man_quaternion_sp;
static xQueueHandle man_euler_sp_q;
static manCtrlEulersp_t _man_euler_sp;

static xQueueHandle pos_sp_q;
static posCtrlsp_t _pos_sp;

static altCtrlsp_t _alt_sp;
static xQueueHandle alt_sp_q;
//static vec3f_t euler_sp;
#if CMD_XBEE
#if XBEE_API

static attCmd_t att_cmd;

static posCmd_t pos_cmd;

#endif
#else

//short rc_int[16];
#endif
static accProcessed_t _acc;
static att_t _att;
static pos_t _pos;
static bool _reset_position;
static float takeoff_height;
static bool takeoff_condition;
static uint16_t return_rising_cnt;
static uint16_t return_flying_cnt;
static uint16_t pos_unlock_cnt;
static bool takeoff_trigger_new;
static void commanderTask( void *pvParameters ) ;
enum statusReturn_e{
	Return_standby = 0,
	Return_rising,
	Return_flying,
	Return_hovering
}return_status;
void get_rc_mode(mode_t* mode, mode_t* lastmode, const rc_t* rc)
{
	*lastmode = *mode;
//	#if MAN_ALT_HIDE
//	
//	#else
	if(rc->channels[4]<-0.33f)
		*mode = modeMan;
	else if(rc->channels[4]<0.33f)
		*mode = modeAlt;
	else
		*mode = modePos;
//	#endif
	
	if(rc->channels[6] < -0.33f|| takeoff_trigger_new)
	{
		if(g_statusFlight == statusLanded || g_statusFlight == statusTakingoff)
		{
			g_mission = Takeoff;
		}
		else
		{
			g_mission = Normal;
		}
		if(rc->channels[6] < -0.33f)
		{
			g_mission = Normal;
			takeoff_trigger_new = false;
		}
	}
	else if(rc->channels[6] < 0.33f)
	{
		g_mission = Normal;
	}
	else
	{
		g_mission = Land;
		if(g_statusFlight == statusLanded)
		{
			g_statusFlight = statusLanded;
		}
		else
		{
			g_statusFlight = statusLanding;
		}
	}
}

int lock_detect(const rc_t *rc)
{
	if(rc->channels[0]>0.85f && rc->channels[1]<-0.85f && rc->channels[2]<-0.85f && rc->channels[3]<-0.85f)
	{
		return 1;
	}
	else if(rc->channels[0]<-0.85f && rc->channels[1]<-0.85f && rc->channels[2]<-0.85f && rc->channels[3]>0.85f)
	{
		reset_takeoff = false;
		return -1;
	}
	else
		return 0;
}
bool get_home(pos_t *pos)
{
	double distance = sqrt((pos->pos.x - return_pos[0])*(pos->pos.x - return_pos[0])+(pos->pos.y - return_pos[1])*(pos->pos.y - return_pos[1]));
	if(distance < 0.6f)
	{
		return true;
	}
	else
		return false;
}
void status_detetct(pos_t *pos,rc_t *rc,accProcessed_t *acc)
{
	if(g_statusFlight == statusLanded && g_mission == Normal)
	{
		if(pos->pos.z < -1.5f)
		{
			g_statusFlight = statusFlying;
		}
	}
	if((g_statusFlight == statusFlying || g_statusFlight == statusLanding) && g_mission == Normal)
	{
		if(rc->channels[2] < -0.9f)
		{
			if(acc->std.v[0] < 50.f&&acc->std.v[1] < 50.f&&acc->std.v[2] < 50.f)
			{
				g_statusFlight = statusLanded;
			}
		}
	}
	
	if(g_statusFlight == statusLanding && g_mission == Land)
	{
		if(acc->std.v[0] < 50.f&&acc->std.v[1] < 50.f&&acc->std.v[2] < 50.f)
		{
			g_statusFlight = statusLanded;
		}
	}
}
void generate_rc_manuel_sp(manCtrlEulersp_t* sp, const rc_t* rc, float dt)
{
	float yaw_moverate;
	sp->euler.P = -rc->channels[1] * MAX_ATT_MANUEL;
	sp->euler.R = -rc->channels[0] * MAX_ATT_MANUEL;
	yaw_moverate = -dead_zone_f(rc->channels[3] * MAX_YAW_RATE_MANEUL, YAW_RATE_DEADZONE);
	sp->euler.Y += yaw_moverate * dt;	
	sp->throttle = dead_zone_f((rc->channels[2] + 1.0f)/2.0f, 0.05);
	sp->timestamp = xTaskGetTickCount();
	#if	XBEE_CMD
		data2send[4] = sp->euler.P * 573;
		data2send[5] = sp->euler.R * 573;
		data2send[6] = sp->euler.Y * 573;
		data2send[7] = sp->throttle*1000;
	
	#endif
}
void generate_rc_altitude_sp(altCtrlsp_t* sp, const rc_t* rc, float dt)
{
	float yaw_moverate, alt_moverate;
	sp->euler.P = -rc->channels[1] * MAX_ATT_MANUEL;
	sp->euler.R = -rc->channels[0] * MAX_ATT_MANUEL;
	yaw_moverate = -dead_zone_f(rc->channels[3] * MAX_YAW_RATE_MANEUL, YAW_RATE_DEADZONE);
	sp->euler.Y += yaw_moverate * dt;		
	alt_moverate = -dead_zone_f(rc->channels[2] * MAX_Z_RATE_MANEUL, Z_RATE_DEADZONE);
	if(alt_moverate<0)
		alt_moverate +=0.1f;
	if(alt_moverate>0)
		alt_moverate -=0.1f;
	sp->vel_ff_z = alt_moverate;
	sp->pos_sp_z += alt_moverate * dt;
	#if SEND_ALT
		data2send[15] = sp->pos_sp_z *1000;
	#endif
	sp->timestamp = xTaskGetTickCount();
}
void generate_rc_position_sp(posCtrlsp_t* sp, const rc_t* rc, float dt)
{
	float yaw_moverate;
	vec3f_t body_moverate, earth_moverate;
	body_moverate.v[0] = dead_zone_f(rc->channels[1] * MAX_XY_RATE_MANEUL, XY_RATE_DEADZONE);
	body_moverate.v[1] = -dead_zone_f(rc->channels[0] * MAX_XY_RATE_MANEUL, XY_RATE_DEADZONE);
	body_moverate.v[2] = -dead_zone_f(rc->channels[2] * MAX_Z_RATE_MANEUL, Z_RATE_DEADZONE);
	yaw_moverate = -dead_zone_f(rc->channels[3] * MAX_YAW_RATE_MANEUL, YAW_RATE_DEADZONE);
	sp->yaw_sp += yaw_moverate * dt;
	body2earth(&_att.R, &body_moverate, &earth_moverate, 2);
	earth_moverate.z = body_moverate.z;

	sp->vel_ff.x = earth_moverate.v[0];
	sp->vel_ff.y = earth_moverate.v[1];
	sp->vel_ff.z = earth_moverate.v[2];
	sp->pos_sp.x += earth_moverate.v[0] * dt;
	sp->pos_sp.y += earth_moverate.v[1] * dt;
	sp->pos_sp.z += earth_moverate.v[2] * dt;
	if(sp->pos_sp.z <= ALT_CONSTRAIN)
	{
		sp->pos_sp.z = ALT_CONSTRAIN;
		sp->vel_ff.z = 0;
	}
}
void reset_yaw_sp_in_manuel(manCtrlEulersp_t* sp, const att_t * att)
{
	sp->euler.Y = att->Euler.Y;		
}
void reset_yaw_sp_in_altctrl(altCtrlsp_t* sp, const att_t * att)
{
	sp->euler.Y = att->Euler.Y;		
}
void reset_yaw_sp_in_posctrl(posCtrlsp_t* sp, const att_t * att)
{
	sp->yaw_sp = att->Euler.Y;		
}
void reset_altitude_sp(altCtrlsp_t* sp, const pos_t * pos)
{
	sp->vel_ff_z = 0;// pos->vel.z/0.6f;
	float vel_length = absolute_f(pos->vel.z);
	float delta_t = vel_length / BREAKING_ACC_MAX;
	sp->pos_sp_z = pos->pos.z + pos->vel.z * delta_t - pos->vel.z/vel_length * 0.5f *BREAKING_ACC_MAX*delta_t*delta_t;
}
void reset_position_sp(posCtrlsp_t* sp, const pos_t * pos)
{
	if(_reset_position)
	{
		sp->vel_ff.x = 0;
		sp->vel_ff.y = 0;
		sp->vel_ff.z = 0;
		sp->acc_ff.x = 0;
		sp->acc_ff.y = 0;
		sp->acc_ff.z = 0;
		float vel_length = sqrt((pos->vel.x)*(pos->vel.x) + (pos->vel.y)*(pos->vel.y));
		if(vel_length > 0.01f)
		{
			float delta_t = vel_length / BREAKING_ACC_MAX;
			sp->pos_sp.x = pos->pos.x + pos->vel.x * delta_t - pos->vel.x/vel_length * 0.5f *BREAKING_ACC_MAX*delta_t*delta_t;			
			sp->pos_sp.y = pos->pos.y + pos->vel.y * delta_t - pos->vel.y/vel_length * 0.5f *BREAKING_ACC_MAX*delta_t*delta_t;			
			sp->pos_sp.z = pos->pos.z;
		}
		else
		{
			sp->pos_sp.x = pos->pos.x;
			sp->pos_sp.y = pos->pos.y;
			sp->pos_sp.z = pos->pos.z;
		}
		_reset_position = false;
	}		
}
void commanderInit(void)
{
//	man_sp_q = xQueueCreate(1,sizeof(manCtrlsp_t));
	man_euler_sp_q = xQueueCreate(1,sizeof(manCtrlEulersp_t));
	man_quaternion_sp_q = xQueueCreate(1,sizeof(manCtrlsp_t));
	pos_sp_q = xQueueCreate(1,sizeof(posCtrlsp_t));
	alt_sp_q = xQueueCreate(1, sizeof(altCtrlsp_t));
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
	float dt;
	uint32_t lastWakeTime, currWakeTime;
	bool inited = false;
	_reset_position = true;
	int lock_cnt=0;
	return_status = Return_standby;
	takeoff_height = 0.0f;
	takeoff_condition = true;
	return_rising_cnt=0;
	return_flying_cnt=0;
	pos_unlock_cnt=0;
	takeoff_trigger_new = false;
	
	TickType_t xLastWakeTime;
	const TickType_t timeIncreament = 20;
	uint32_t last_timestamp = 0;
	xLastWakeTime = xTaskGetTickCount();
	for(;;){

	#if XBEE_API
		if(g_mode == modeMan){
			xbee_commandBlockingAcquire(&att_cmd);
			xbee_attCmd2manCtrlsp(&_man_quaternion_sp, &att_cmd);
			_man_quaternion_sp.timestamp = xTaskGetTickCount ();
			xQueueOverwrite(man_quaternion_sp_q, &_man_quaternion_sp);
		}
		else if (g_mode == modePos){
			if(g_statusLink != link_broken_in_air){
				xbee_outdoor_commandAcquire(&pos_cmd);//rewriten by Wade
				uint32_t this_timestamp = pos_cmd.timestamp;
				if(this_timestamp != last_timestamp){
					last_timestamp = this_timestamp;
					pos_cmd.timestamp = xTaskGetTickCount ();
					xbee_posCmd2posSp(&_pos_sp,&pos_cmd);
					_pos_sp.timestamp = xTaskGetTickCount();
					xQueueOverwrite(pos_sp_q, &_pos_sp);
				}
			}
			else{//link_broken_in_air
				_pos_sp.pos_sp.z += 0.6f * timeIncreament / 1000;
				_pos_sp.timestamp = xTaskGetTickCount();
				xQueueOverwrite(pos_sp_q, &_pos_sp);
			}
		}
		else if(g_mode == modeRC){
			if(g_statusLink != link_broken_in_air){
				xbee_rcAcquire(&_rc);//rewriten by Wade
				attAcquire(&_att);
				posAcquire(&_pos);
				accAcquire(&_acc);
				status_detetct(&_pos,&_rc,&_acc);
				if(!inited){
					reset_yaw_sp_in_manuel(&_man_euler_sp, &_att);
					reset_yaw_sp_in_altctrl(&_alt_sp, &_att);
					reset_yaw_sp_in_posctrl(&_pos_sp, &_att);
					inited = true;
				}
				currWakeTime = xTaskGetTickCount ();
				dt = (float)(currWakeTime - lastWakeTime) / (float)configTICK_RATE_HZ;
				if(dt > 0.2f)
					dt = 0.2f;
				if(dt < 0.01f)
					dt = 0.01f;
				lastWakeTime = currWakeTime;
				get_rc_mode(&_mode, &_lastmode, &_rc);
				if(_mode != _lastmode){
					if(_mode == modeAlt)
					{
						reset_yaw_sp_in_altctrl(&_alt_sp,&_att);
						reset_altitude_sp(&_alt_sp, &_pos);
					}
					if(_mode == modeMan)
					{
						reset_yaw_sp_in_manuel(&_man_euler_sp,&_att);
					}
					if(_mode == modePos)
					{
						if(gps_valid == false)
						{
							_mode = _lastmode;
						}
						if(g_statusFlight == statusLanded && _rc.channels[2] < -0.9f)
						{
							reset_takeoff = true;
						}
						_reset_position = true;
						reset_yaw_sp_in_posctrl(&_pos_sp, &_att);
						reset_position_sp(&_pos_sp, &_pos);
					}
				}
					if(g_statusLockRC == motorIdle)
					{
						_reset_position = true;
						reset_yaw_sp_in_posctrl(&_pos_sp, &_att);
						reset_position_sp(&_pos_sp, &_pos);
					}
					g_modeRC = _mode;
					if(_rc.channels[2]<-0.9f){
						reset_yaw_sp_in_manuel(&_man_euler_sp, &_att);
						reset_yaw_sp_in_altctrl(&_alt_sp, &_att);
						reset_yaw_sp_in_posctrl(&_pos_sp, &_att);
					}
					if(g_statusLockRC == motorIdle && _rc.channels[2]>-0.1f && _rc.channels[2]<0.1f)
					{
						pos_unlock_cnt++;
						if(pos_unlock_cnt>50)
						{
							g_statusLockRC = motorUnlocked;
							takeoff_trigger_new = true;
							pos_unlock_cnt=0;
						}
					}
					if(g_modeRC == modeMan){
						generate_rc_manuel_sp(&_man_euler_sp, &_rc, dt);
						_man_euler_sp.timestamp = xTaskGetTickCount ();
						xQueueOverwrite(man_euler_sp_q, &_man_euler_sp);
					}
					else if(g_modeRC == modeAlt){
						generate_rc_altitude_sp(&_alt_sp, &_rc, dt);
						_alt_sp.timestamp = xTaskGetTickCount ();
						xQueueOverwrite(alt_sp_q, &_alt_sp);
					}
					else if(g_modeRC == modePos){
						if(g_mission == Returnhome)
						{
							switch(return_status)
							{
								case Return_standby:
								{
									_pos_sp.pos_sp.x = _pos.pos.x;
									_pos_sp.pos_sp.y = _pos.pos.y;
									_pos_sp.pos_sp.z = _pos.pos.z;
									_pos_sp.yaw_sp = _att.Euler.Y;
									return_status = Return_rising;
									break;
								}
								case Return_rising:
								{
									if(_pos_sp.pos_sp.z > (-RTL_ALTITUDE + 0.2f))
									{
										_pos_sp.pos_sp.z -= 1.0f*dt;
									}
									else if(_pos_sp.pos_sp.z < (-RTL_ALTITUDE - 0.2f))
									{
										_pos_sp.pos_sp.z += 1.0f*dt;
									}
									else
									{
										_pos_sp.pos_sp.z = -RTL_ALTITUDE;
									}
									if(fabs(-RTL_ALTITUDE - _pos.pos.z)<0.2f)
									{
										return_rising_cnt++;
										if(return_rising_cnt>50)
										{
											return_rising_cnt=0;
											return_status = Return_flying;
										}
									}
									break;
								}
								case Return_flying:
								{
									vec3f_t dir;
									dir.v[0] = _pos_sp.pos_sp.x - return_pos[0];
									dir.v[1] = _pos_sp.pos_sp.y - return_pos[1];
									dir.v[2] = 0.0f;
									vec3f_normalize(&dir);
									
									_pos_sp.pos_sp.x -= dir.v[0]*4.0f*dt;
									_pos_sp.pos_sp.y -= dir.v[1]*4.0f*dt;
									_pos_sp.pos_sp.z = -RTL_ALTITUDE;
									if(get_home(&_pos))
									{
										return_flying_cnt++;
										if(return_flying_cnt>50)
										{
											return_flying_cnt=0;
											return_status = Return_hovering;
										}
									}
									break;
								}
								case Return_hovering:
								{
									if(_pos_sp.pos_sp.z < -RTL_HOVERING_ALTITUDE)
										_pos_sp.pos_sp.z += 1.0f*dt;
									else
									{
										_pos_sp.pos_sp.z = -RTL_HOVERING_ALTITUDE;
									}
									break;
								}
							}
							_pos_sp.timestamp = xTaskGetTickCount();
							xQueueOverwrite(pos_sp_q, &_pos_sp);
							_reset_position = true;
						}
						else if(g_mission == Land)
						{
							reset_takeoff = false;
							_pos_sp.pos_sp.z += 1.0f*dt;
							_pos_sp.timestamp = xTaskGetTickCount();
							xQueueOverwrite(pos_sp_q, &_pos_sp);
						}
						else if(g_mission == Takeoff)
						{
							reset_takeoff = false;
							g_statusFlight = statusTakingoff;
							_pos_sp.pos_sp.x = _pos.pos.x;
							_pos_sp.pos_sp.y = _pos.pos.y;
							if(_pos_sp.pos_sp.z <= takeoff_height-5.0f)
							{
								takeoff_switch = false;
								_pos_sp.pos_sp.z = takeoff_height-5.0f;
							}
							else
							{
		//						if(takeoff_condition)
		//						{
		//							_pos_sp.pos_sp.z = takeoff_height - 1.0f * dt;
		//							takeoff_condition = false;
		//						}
		//						else
		//						{
		//							_pos_sp.pos_sp.z -= 3.0f * dt;
		//						}
								if(takeoff_condition)
								{
									_pos_sp.pos_sp.z = takeoff_height + 1.0f;
									takeoff_condition = false;
								}
								else
								{
									if(_pos.pos.z > takeoff_height - 1.0f)
									{
										_pos_sp.pos_sp.z -= 1.0f * dt;
									}else
									{
										_pos_sp.pos_sp.z -= 1.2f * dt;
									}
								}
		//						takeoff_switch = true;
		//						_pos_sp.pos_sp.z = _pos.pos.z;
		//						_pos_sp.vel_ff.z = -constrain_f(0.2f * (_pos_sp.pos_sp.z - (takeoff_height-3.0f)),0.001f,1.0f);
							}
							if(fabs(takeoff_height-5.0f - _pos.pos.z)<0.2f)
							{
								g_statusFlight = statusFlying;
								takeoff_trigger_new = false;
							}
							_pos_sp.timestamp = xTaskGetTickCount();
							xQueueOverwrite(pos_sp_q, &_pos_sp);
							_reset_position = true;
						}//end of else if(g_mission == Takeoff)
						else
						{
							return_status = Return_standby;
							reset_position_sp(&_pos_sp, &_pos);
							if(g_statusLockRC == motorUnlocked)
							{
								generate_rc_position_sp(&_pos_sp, &_rc, dt);
							}
							else if(g_statusLockRC == motorIdle)
							{
								_reset_position = true;
								reset_yaw_sp_in_posctrl(&_pos_sp, &_att);
								reset_position_sp(&_pos_sp, &_pos);
							}
		//					else
		//					{
		//						_reset_position = true;
		//						reset_yaw_sp_in_posctrl(&_pos_sp, &_att);
		//						reset_position_sp(&_pos_sp, &_pos);
		//					}
								
							_pos_sp.timestamp = xTaskGetTickCount();
							xQueueOverwrite(pos_sp_q, &_pos_sp);
						}
					}
				
			//	pid_tune(&_rc);
				int lock_ret = lock_detect(&_rc);
				if(lock_ret == 1)
					lock_cnt++;
				else if(lock_ret == -1)
					lock_cnt--;
				else 
					lock_cnt=0;
				if(lock_cnt>75)
				{
					if(g_modeRC == modePos)
					{
						g_statusLockRC = motorIdle;
					}
					else
					{
						g_statusLockRC = motorUnlocked;
					}
					armed = true;
					
					if(reset_return_pos)
					{
						return_pos[0] = _pos.pos.x;
						return_pos[1] = _pos.pos.y;
						reset_return_pos = false;
					}
					takeoff_height = _pos.pos.z;
					takeoff_condition = true;
					reset_pid(&pos_xPID);
					reset_pid(&pos_yPID);
					reset_pid(&altPID);
					reset_pid(&rollPID);
					reset_pid(&pitchPID);
					reset_pid(&yawPID);
				}
				if(lock_cnt<-75)
				{
					g_statusLockRC = motorLocked;
					armed = false;
					_reset_position = true;
					reset_position_sp(&_pos_sp, &_pos);
					reset_return_pos = true;
		//			takeoff_height = _pos.pos.z;
		//			takeoff_condition = true;
				}
		
			//	uint32_t this_timestamp = rc.timestamp;
			//	for(int i=0;i<6;i++)
			//		data2send[6+i] = rc.channels[i]*1000;
			}
			else{//link_broken_in_air
				
				
			}
		}//end of modeRC
	#endif
		#if	XBEE_CMD
			data2send[0] = g_mode;
			data2send[1] = g_modeRC;
			data2send[2] = g_statusLock;
			data2send[3] = g_statusLockRC;
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
	pdres = xQueuePeek(pos_sp_q, sp, 0);
	return pdres;
}

BaseType_t altSetpointAcquire(altCtrlsp_t *sp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(alt_sp_q, sp, 0);
	return pdres;
}

BaseType_t manSetpointAcquire(manCtrlsp_t *sp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(man_quaternion_sp_q, sp, 0);
	return pdres;
}
BaseType_t manEulerSetpointAcquire(manCtrlEulersp_t *sp)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(man_euler_sp_q, sp, 0);
	return pdres;
}
