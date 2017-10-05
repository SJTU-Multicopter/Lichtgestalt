#ifndef GPS_H
#define GPS_H
#include "../MessageTypes/type_methods.h"
void GPSInit(void);
void gps_acquire(gpsRaw_t *gps_data);
void GPSReceive_IDLE(void);

struct _gps{
	int lat;
	int lon;
	int alt;
	int vel;
	float azm; //azimuth, rad
	short sat;
	short status;
	char gpsflag;
	int x;
	int y;
	int z;
	int vx;
	int vy;
	unsigned char v_valid;
	#define GPS_PERIOD 200
	#define GPS_DELAY 400
};

//extern void get_gps_data(void);
//extern float char_to_float(char *a);

/*****received_type define******/
#define LAT		0//in unit of min, minus means S
#define LON		1//in unit of min, minus means W
#define ALT		2//in unit of m
#define VEL		3//in unit of m/s
#define AZM		4//Azimuth, in unit of degrees
#define SAT		5//number of nevigating satellites
#define STATUS	6//
/*******frame define*******/
#define GPGGA		0
#define GPRMC		1
#define OTH_FRM		2
#endif
