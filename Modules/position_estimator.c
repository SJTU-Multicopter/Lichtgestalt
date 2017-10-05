#include "position_estimator.h"
#include "sensors_task.h"
#include "../Devices/GPS.h"
#include "attitude_estimator.h"
#include "commander.h"
#include "../Devices/data_link.h"
#include "../MessageTypes/type_methods.h"
#include "cmsis_os.h"
#define POS_EST_TASK_PERIOD_MS 10
#define EST_BUF_SIZE 30		// buffer size is 0.5s
#define MEAN_BUF 20
static xQueueHandle pos_q;
void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		x[0] += x[1] * dt + acc * dt * dt / 2.0f;
		x[1] += acc * dt;
	}
}
void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;
		if(w > 1.0f)
			w = 1.0f;
		if (i == 0) {
			x[1] += w * ewdt;
		}
	}
}
int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0) 
{
	ref->lat_rad = lat_0 * M_DEG_TO_RAD_F;
	ref->lon_rad = lon_0 * M_DEG_TO_RAD_F;
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);
	ref->init_done = true;

	return 0;
}

int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, double *x, double *y)
{
	*x = (lat * M_DEG_TO_RAD_F - ref->lat_rad) * CONSTANTS_RADIUS_OF_EARTH;
	*y = (lon * M_DEG_TO_RAD_F - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH * ref->cos_lat;

	return 0;
}



static void position_estimation( void *pvParameters )
{
	uint32_t lastWakeTime = 0;
	lastWakeTime = xTaskGetTickCount ();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, POS_EST_TASK_PERIOD_MS);
		uint32_t t = xTaskGetTickCount();
		pos_t local_pos;
		for(int i=0;i<3;i++){
			local_pos.acc.v[i] = 0;
			local_pos.vel.v[i] = 0;
			local_pos.pos.v[i] = 0;
		}
		
		xQueueOverwrite(pos_q, &local_pos);
//		marg_acquire(&imu);
//		attAcquire(&att);
//		baro_acquire(&baro);
	}
	
}
void position_estimation_start(void)
{
	xTaskCreate(position_estimation, "position_estimation", 1200, NULL, 1, NULL);
}
void position_estimation_queue_init(void)
{
	pos_q = xQueueCreate(1, sizeof(pos_t));
}
BaseType_t posAcquire(pos_t *pos)
{
	BaseType_t pdres=pdFALSE;
//	pdres = xQueuePeek(pos_q, pos, 0);
	return pdres;
}
BaseType_t posBlockingAcquire(pos_t *pos)
{
	BaseType_t pdres=pdFALSE;
//	pdres = xQueueReceive(pos_q, pos, portMAX_DELAY);
	return pdres;
}

BaseType_t globalposAcquire(pos_t *pos)
{
	BaseType_t pdres=pdFALSE;
//	xQueuePeek(global_pos_q, pos, 0);
	return pdres;
}
BaseType_t globalposBlockingAcquire(pos_t *pos)
{
	BaseType_t pdres=pdFALSE;
//	xQueueReceive(global_pos_q, pos, portMAX_DELAY);
	return pdres;
}
/*
BaseType_t accAcquire(accProcessed_t *acc)
{
	BaseType_t pdres=pdFALSE;
//	pdres = xQueuePeek(acc_processed_q, acc, 0);
	return pdres;
}

BaseType_t acc_d_cali_BlockingAcquire(float *acc)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueueReceive(acc_d_cali_q, acc, portMAX_DELAY);
	return pdres;
}

*/
