#ifndef MESSAGES_H
#define MESSAGES_H
#include "basic_types.h"

typedef struct accProcessed_s{
	uint32_t timestamp;
	vec3f_t mean;
	vec3f_t std;
}accProcessed_t;
typedef enum missionVehicle_e{
	Normal = 0,
	Takeoff,
	Land,
	Returnhome
}mission_t;
typedef struct margData_s {
	uint32_t timestamp;
	vec3f_t acc;
	vec3f_t gyr;
	vec3f_t mag;
	bool mag_updated;
} marg_t;
typedef struct calib_s {
	uint32_t timestamp;
	vec3i16_t acc_bias;
	vec3i16_t gyr_bias;	
	vec3i16_t mag_bias;
} calib_t;
typedef struct baro_s {
	uint32_t timestamp;
	float pressure;
	float temp;
	float alt;
	float offset;
	float refPressure;
//	enum status2Baro_e {
//		baro_convert = 0,
//		baro_read = 1
//	}i2cStatus;
	enum statusBaro_e {
		baro_initializing = 0,//once temp, once pres
		baro_running = 1//once temp, 5 pres
	}status;
} baro_t;
typedef struct gpsRaw_s {
	uint32_t timestamp;
	int64_t lat;
	int64_t lon;
	float azm;
	float vel;
	bool vel_valid;
	float alt;
	uint8_t sat;
	uint32_t date;
	uint32_t time;
	enum statusGPS_e {
		noGPS = 0,
		realtimeGPS = 1,
		difGPS = 2
	}status;
} gpsRaw_t;
typedef struct gpsProcessed_s {
	uint32_t timestamp;
	vec3f_t pos;
	vec3f_t vel;
	vec3f_t acc;
} gpsProcessed_t;
typedef struct rc_s {
	uint32_t timestamp;
	float channels[16];
} rc_t;
typedef struct battery_s {
	uint32_t timestamp;
	uint16_t voltage;
} battery_t;
typedef struct att_s {
	uint32_t timestamp;
	vec3f_t Euler;
	quaternion_t Q;
	rotation_t R;
	vec3f_t rate;
} att_t;
typedef struct pos_s {
	uint32_t timestamp;
	vec3f_t pos;
	vec3f_t vel;
	vec3f_t acc;
	bool xy_valid;
} pos_t;
typedef struct att_command_s {
	uint32_t timestamp;
	quaternion_t Q;
	float thrust;
} attCmd_t;
typedef struct pos_command_s {
	uint32_t timestamp;
	vec3f_t pos_sp;
	vec3f_t vel_ff;
	vec3f_t acc_ff;
	float yaw_sp;
	short commands;
} posCmd_t;
typedef struct attsp_s {
	uint32_t timestamp;
	rotation_t R;
	float thrust;
	float yaw_ff;
} attsp_t;
typedef struct possp_s {
	uint32_t timestamp;
	vec3f_t pos_sp;
	vec3f_t vel_ff;
	vec3f_t acc_ff;
	float yaw_sp;
	short commands;
} posCtrlsp_t;
typedef struct altCtrlsp_s {
	uint32_t timestamp;
	float pos_sp_z;
	float vel_ff_z;
	vec3f_t euler;
} altCtrlsp_t;
typedef struct manuelCtrlsp_s {
	uint32_t timestamp;
	float thrust;
	quaternion_t Q;
} manCtrlsp_t;

typedef struct output_s {
	uint32_t timestamp;
	vec3f_t moment;
	float thrust;
} output_t;
typedef struct motors_s {
	uint32_t timestamp;
	float thrust[8];
} motors_t;
typedef struct agent_s {
	unsigned int coord_addr_h;
	unsigned int coord_addr_l;
	unsigned int self_addr_h;
	unsigned int self_addr_l;
	unsigned int agent_addr;
} agent_t;
typedef enum modeVehicle_e {
	modeCal = 0,
	modeRate,
	modeMan,
	modeAlt,
	modePos,
	modeTrj,
	modeRC
} mode_t;
typedef enum modeRC_e {

	modeRcRate,
	modeRcMan,
	modeRcAlt,
	modeRcPos

} modeRC_t;
typedef enum statusLock_e {
	motorLocked = 0,
	motorUnlocking,
	motorUnlocked,
	motorIdle,
	motorLocking
} statusLock_t;
typedef enum statusFlight_e {
	statusLanded = 0,
	statusInitiating,
	statusTakingoff,
	statusLanding,
	statusFlying,
	statusHovering,
	stautsFreefall,
	stautsUpsidedown,
	stautsTumble
} statusFlight_t;
typedef enum statusGS_e {
	gs_normal = 0,

} statusGS_t;
typedef enum statusLink_e {
	link_idle = 0,
	link_running,
	link_broken_in_air
} statusLink_t;
typedef struct statusCheck_s{
	bool ok;
	uint8_t acc;
	uint8_t gyr;
	uint8_t mag;
	uint8_t bar;
	uint8_t gps;
	uint8_t bat;
	uint8_t rc;
	uint8_t att;
	uint8_t pos;
}statusCheck_t;
extern mode_t g_mode;
extern modeRC_t g_modeRC;
extern statusLock_t g_statusLock;
extern statusFlight_t g_statusFlight;
extern statusLink_t g_statusLink;
extern statusGS_t g_statusGS;
extern short data2send[18];
#endif
