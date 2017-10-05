#ifndef PLATFORM_H
#define PLATFORM_H

/******Board Version*****/
#define BOARD_LICHTBRD_V1 1

/******Air Frame*****/
#define AIRFRAME_LICHT160 1
#if AIRFRAME_LICHT160
	#define THRCMDRATIO 1
	#define VEHICLE_MASS 220
	#define ROTOR_DIST 160//mm
	#define ONE_2_ROTOR_DIST 3.125f//1/(2*d)
	#define D2_SQRT2 226//(2*D)/SQRT2_2D
	#define FORCE_TORQUE_RATIO 15
	#define ONE_4_F_T_RATIO 16.67f
	#define PLUS 1
	#define MOTOR_POWER 200
	#define MOTOR_THRESHOLD 150
#endif
/******Sensors Used*****/
#define IMU_MPU6000 1
#define GPS_M8N 0
#define GPS_LEA6H 0

#define COMPASS_HMC 1

#define BARO_I2C 1
#define BARO_SPI 0

/******Driver Options*****/

/******Devices Options*****/
#define XBEE_API 1
#define XBEE_TRANS 0
#define CMD_SBUS 0
#define CMD_PPM 0
#define CMD_XBEE 1
#define EEP_ROM 0
#define FLASH_ROM 1
/******Algorithm Options*****/
#define ATT_COMP 1
#define ATT_EKF 0
#define ATT_MADGWICK 0

#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25
/******Flight Modes*****/
#define INDOOR 0
#define OUTDOOR 1
#define ON_FLIGHT 1
#define OFF_FLIGHT 0

#if OUTDOOR
	#define M8N_GPS 1
#endif


#define FREQ_1000HZ 1

#define MAX_ATT_MANUEL 0.698f//11437 40deg,0.698rad
#define MAX_YAW_RATE 0.698f//14303 50.0
#define MAX_YAW_RATE_MANEUL 0.698f
#define YAWRATE_DEADZONE 0.17f
#define BAT_WARNING 3700
//#define GRAVITY 9.81f
#endif
