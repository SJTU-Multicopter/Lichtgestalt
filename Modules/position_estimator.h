#ifndef POSITION_ESTIMATOR_H
#define POSITION_ESTIMATOR_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <float.h>
#include "../MessageTypes/type_methods.h"
//#include "../Mathlib/comparison.h"
#include "cmsis_os.h"

#define CONSTANTS_ONE_G					9.80665f		/* m/s^2		*/
#define CONSTANTS_RADIUS_OF_EARTH			6371000.0f			/* meters (m)		*/
#define M_DEG_TO_RAD 	0.01745329251994f
#define M_RAD_TO_DEG 	57.2957795130823f
#define M_DEG_TO_RAD_F 	0.01745329251994f
#define M_RAD_TO_DEG_F 	57.2957795130823f


extern bool gps_valid;
//extern xQueueHandle pos_q;
extern float cali_acc_pos_z;

struct map_projection_reference_s {
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
	bool init_done;
};

struct home_position_s {
	double lat;
	double lon;
	double alt;
};

//int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0); //lat_0, lon_0 are expected to be in correct format: -> 47.1234567 and not 471234567
//int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, double *x, double *y);

void inertial_filter_predict(float dt, float x[2], float acc);
void inertial_filter_correct(float e, float dt, float x[2], int i, float w);

void acc_calibrate(const vec3f_t* acc,const rotation_t* R);
BaseType_t posAcquire(pos_t *pos);
BaseType_t posBlockingAcquire(pos_t *pos);
BaseType_t globalposAcquire(pos_t *pos);
BaseType_t globalposBlockingAcquire(pos_t *pos);
void position_estimation_start(void);
void position_estimation_queue_init(void);
void acc_filter_start(void);
BaseType_t accAcquire(accProcessed_t *acc);
//BaseType_t acc_d_cali_BlockingAcquire(float *acc);

#endif
