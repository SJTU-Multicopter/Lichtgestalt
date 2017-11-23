#include "position_estimator.h"
#include "sensors_task.h"
#include "../Devices/GPS.h"
#include "attitude_estimator.h"
#include "commander.h"
#include "../Devices/data_link.h"
#include "../MessageTypes/type_methods.h"
#include "../Mathlib/comparison.h"
#include "../Commons/platform.h"
#include "cmsis_os.h"
#define POS_EST_TASK_PERIOD_MS 10
#define EST_BUF_SIZE 30		// buffer size is 0.5s
#define MEAN_BUF 20
float cali_acc_pos_z = 0.0f;//hyl
const float delay_gps = 0.20f;
const float w_xy_gps_p = 0.4f;//sjtu_0.7-0.5 && 1.0//A9_1.0-1.0
const float w_z_gps_p = 0.0f;
const float w_xy_gps_v = 0.4f;
const float w_z_gps_v = 0.0f;

const float w_z_baro = 0.5f;

const float w_xy_vicon_p = 3.0f;
const float w_xy_vicon_v = 3.0f;
const float w_z_vicon_p = 2.0f;
const float w_z_vicon_v = 2.0f;

const float w_acc_xy_bias = 0.01f;//0.01f
const float w_acc_z_bias = 0.00f;//0.005f;
const float acc_bias_max = 0.1f;
bool gps_valid = false;

vec3f_t _acc_bias_body;   
vec3f_t _acc_filtered;
float yaw_bias = 0;
const float w_azm = 0.005f;
static xQueueHandle pos_q;
static xQueueHandle global_pos_q;
static xQueueHandle acc_processed_q;
static xQueueHandle acc_d_cali_q;

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


/*
static void position_estimation( void *pvParameters )
{
	baro_t  baro;
	att_t att;
	marg_t imu;
	gpsRaw_t gps_data;
	
	uint32_t lastWakeTime = 0;
	lastWakeTime = xTaskGetTickCount ();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, POS_EST_TASK_PERIOD_MS);
		uint32_t t = xTaskGetTickCount();
		pos_t local_pos;
		margAcquire(&imu);
		attAcquire(&att);
		baroAcquire(&baro);
		gps_acquire(&gps_data);
		for(int i=0;i<3;i++){
			local_pos.acc.v[i] = 0;
			local_pos.vel.v[i] = 0;
			local_pos.pos.v[i] = 0;
		}
		local_pos.timestamp = xTaskGetTickCount ();
		xQueueOverwrite(pos_q, &local_pos);
//		marg_acquire(&imu);
//		attAcquire(&att);
//		baro_acquire(&baro);
	}
	
}
*/
static void position_estimation( void *pvParameters )
{
	float x_est[2] = { 0.0f, 0.0f };	// pos, vel
	float y_est[2] = { 0.0f, 0.0f };	// pos, vel
	float z_est[2] = { 0.0f, 0.0f };	// pos, vel

	//buf for delay
	float est_buf[EST_BUF_SIZE][3][2];	// estimated position buffer
	float R_buf[EST_BUF_SIZE][3][3];	// rotation matrix buffer
	float R_gps[3][3];					// rotation matrix for GPS correction moment
	float R_vicon[3][3];
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	memset(R_vicon, 0, sizeof(R_vicon));
	int buf_ptr = 0;

	float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame
	float acc_bias_earth[3] = {0.0f,0.0f,0.0f};
	float corr_baro = 0.0f;		// D
	float corr_gps[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
/*	float corr_vicon[3][2] = {
		{ 0.0f, 0.0f },		// N (pos, vel)
		{ 0.0f, 0.0f },		// E (pos, vel)
		{ 0.0f, 0.0f },		// D (pos, vel)
	};
*/
	//baro init and offset
	int baro_init_cnt = 0;
	int baro_init_num = 200;
	int baro_reset_num = 50;
	float baro_offset = 0.0f;		// baro offset for reference altitude, initialized on start, then adjusted
	float baro_reset = 0.0f;
	float baro_reset_sum = 0.0f;
	bool reset_baro = false;
	bool baro_fly = false;
	
	//acc init and offset	
	int acc_init_cnt = 0;
	int acc_init_num = 200;
//	float acc_offset = 0.0f;
	
	int gps_init_cnt = 0;
	int gps_init_num = 20;
	int gps_avg_cnt = 0;
	int gps_avg_num = 20;
	bool gps_init = false;
	double home_lon = 0.0;
	double home_lat = 0.0;
	
	double gps_proj_x_prev = 0.0;
	double gps_proj_y_prev = 0.0;
	
//	double vicon_x_prev = 0.0;
//	double vicon_y_prev = 0.0;
//	double vicon_z_prev = 0.0;
	
	baro_t  baro;
	memset(&baro, 0, sizeof(baro));
	marg_t imu;
	memset(&imu, 0, sizeof(imu));
	///////
	marg_t imu_acc_init;
	memset(&imu_acc_init, 0, sizeof(imu_acc_init));
	uint32_t imu_acc_timestamp = 0;
	uint32_t att_acc_timestamp = 0;
	att_t att_acc_init;
	bool att_acc_ready = false;
	float cali_acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
	
	////////////
	att_t att;
	memset(&att, 0, sizeof(att));
	gpsRaw_t gps;
	memset(&gps, 0, sizeof(gps));
	pos_t vicon;
	memset(&vicon, 0, sizeof(vicon));
	pos_t local_pos;
	memset(&local_pos, 0, sizeof(local_pos));
	pos_t global_pos;
	memset(&global_pos, 0, sizeof(global_pos));
	
	//home position 
	struct map_projection_reference_s ref;
	memset(&ref, 0, sizeof(ref));
	struct home_position_s home;
	memset(&home, 0, sizeof(home));

	bool home_init = false;

	//sensor valid
//	bool gps_valid = false;
	bool vicon_valid = false;
	bool use_baro = true;
	bool att_ready = false;
	bool gps_azm_correct = false;

	/* wait for initial baro value */
	bool wait_baro = true;
	/* wait for initial acc value */
	bool wait_acc = true;
	//time_stamp
	uint32_t imu_timestamp = 0;
	uint32_t gps_timestamp = 0;
//	uint32_t vicon_timestamp = 0;
	uint32_t gps_pre_t = 0;
	uint32_t baro_timestamp = 0;
	uint32_t att_timestamp = 0;
	uint32_t t_prev = 0;
	uint32_t lastWakeTime = 0;
	lastWakeTime = xTaskGetTickCount ();

	float cali_acc_pos_z_tmp = 0.0f;
	for(int i = 0 ; i < 200; i ++)
	{
		vTaskDelayUntil(&lastWakeTime, POS_EST_TASK_PERIOD_MS);
		baroAcquire(&baro);
	}
	while (wait_baro)
	{
		vTaskDelayUntil(&lastWakeTime, POS_EST_TASK_PERIOD_MS);
		
		baroAcquire(&baro);

		/* mean calculation over several measurements */
		if (baro_init_cnt < baro_init_num) {
			if (isfinite(baro.alt)) {
				baro_offset += baro.alt;
				baro_init_cnt++;
			}
		} else {
			wait_baro = false;
			baro_init_cnt = 0;
			baro_offset /= (float)baro_init_num;
			baro_reset = baro_offset;
		}
	}

	///////////////////
	while (wait_acc)
	{
		vTaskDelayUntil(&lastWakeTime, POS_EST_TASK_PERIOD_MS);
		margAcquire(&imu_acc_init);
		attAcquire(&att_acc_init);
		if(att_acc_timestamp != att_acc_init.timestamp)
		{
			att_acc_ready = true;
		}else
		{
			continue;
		}
		if (acc_init_cnt < acc_init_num) {
			if(imu_acc_timestamp != imu_acc_init.timestamp)
			{
			//	_acc_filtered.v[0] *= CONSTANTS_ONE_G;
			//	_acc_filtered.v[1] *= CONSTANTS_ONE_G;
			//	_acc_filtered.v[2] *= CONSTANTS_ONE_G;
				vec3f_t acc_vec_0;
				body2earth(&att_acc_init.R, &_acc_filtered, &acc_vec_0, 3);
			
				cali_acc[0] = acc_vec_0.x ;
				cali_acc[1] = acc_vec_0.y ;
				cali_acc[2] = acc_vec_0.z + CONSTANTS_ONE_G;
				
				cali_acc_pos_z_tmp += cali_acc[2];
				acc_init_cnt++;
				
				imu_acc_timestamp = imu_acc_init.timestamp;
			}
		}
		else {
			wait_acc = false;
			cali_acc_pos_z_tmp /= (float)acc_init_num * CONSTANTS_ONE_G;
			cali_acc_pos_z = cali_acc_pos_z_tmp;
		}
	}
	////////////////////
	
	
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, POS_EST_TASK_PERIOD_MS);
		uint32_t t = xTaskGetTickCount();

		margAcquire(&imu);
		attAcquire(&att);
		baroAcquire(&baro);
		
		if(att_timestamp != att.timestamp)
		{
			att_ready = true;
		}else
		{
			continue;
		}
		
		if(imu_timestamp != imu.timestamp)
		{
		//	 correct accel bias 
			_acc_filtered.v[0] -= _acc_bias_body.v[0];
			_acc_filtered.v[1] -= _acc_bias_body.v[1];
			_acc_filtered.v[2] -= _acc_bias_body.v[2];
		//	_acc_filtered.v[0] *= CONSTANTS_ONE_G;
		//	_acc_filtered.v[1] *= CONSTANTS_ONE_G;
		//	_acc_filtered.v[2] *= CONSTANTS_ONE_G;
			
//			_acc_filtered.v[0] -= acc_bias[0];
//			_acc_filtered.v[1] -= acc_bias[1];
//			_acc_filtered.v[2] -= acc_bias[2];

			
	//		imu.acc.v[0] -= _acc_bias_body.v[0];
	//		imu.acc.v[1] -= _acc_bias_body.v[1];
	//		imu.acc.v[2] -= _acc_bias_body.v[2];
		
//			imu.acc.v[0] *= CONSTANTS_ONE_G;
//			imu.acc.v[1] *= CONSTANTS_ONE_G;
//			imu.acc.v[2] *= CONSTANTS_ONE_G;
//			imu.acc.v[0] -= acc_bias[0];
//			imu.acc.v[1] -= acc_bias[1];
//			imu.acc.v[2] -= acc_bias[2];

			/* transform acceleration vector from body frame to NED frame */
			vec3f_t acc_vec;
			body2earth(&att.R, &_acc_filtered, &acc_vec, 3);
			//body2earth(&att.R, &imu.acc, &acc_vec, 3);
			acc[0] = acc_vec.x ;
			acc[1] = acc_vec.y ;
			acc[2] = acc_vec.z + CONSTANTS_ONE_G;
			
			acc[0] -= acc_bias_earth[0];
			acc[1] -= acc_bias_earth[1];
			acc[2] -= acc_bias_earth[2];
					
			xQueueOverwrite(acc_d_cali_q,&acc[2]);
			
			#if XBEE_POS
				data2send[0] = acc[0] * 1000;
				data2send[1] = acc[1] * 1000;
				data2send[2] = acc[2] * 1000;	

			#endif
			//sample 200 datas, calculate the mean value, assign it to first_acc, 
			imu_timestamp = imu.timestamp;
		}

		if(baro_timestamp != baro.timestamp)
		{
			if(g_statusLock != motorLocked)
			{
				if(!reset_baro)
				{
					use_baro = false;
					if(baro_init_cnt < baro_reset_num)
					{
						if (isfinite(baro.alt)) {
							baro_reset_sum +=  baro.alt;
							baro_init_cnt++;
						}
					}else
					{
						baro_init_cnt = 0;
						baro_reset_sum /= (float)baro_reset_num;
						baro_reset = baro_reset_sum;
						baro_reset_sum = 0.0f;
						reset_baro = true;
						z_est[0] = baro_offset - baro_reset;
						z_est[1] = 0.0f;
					}
				}else
				{
					use_baro = true;
					if(((baro_offset - baro_reset - z_est[0]) < 0.8f) && !baro_fly)
					{
						corr_baro = baro_offset - baro_reset - z_est[0];
					}else
					{
						baro_fly = true;
						corr_baro = baro_offset - baro.alt - z_est[0];
					}	
				}			
			}else
			{
				reset_baro = false;
				baro_fly = false;
				corr_baro = baro_offset - baro.alt - z_est[0];
			}
			//update baro_correction
			//corr_baro = baro_offset - baro.alt - z_est[0];
			baro_timestamp = baro.timestamp;
			//data2send[15] = (baro_offset - baro_reset) * 1000;
		}else
		{
			corr_baro = 0;
		}

		
		gps_acquire(&gps);
		if(gps_timestamp != gps.timestamp)
		{
			gps_azm_correct = true;
			gps_timestamp = gps.timestamp;
			bool reset_est = false;

			//init home position
			if(!home_init)
			{
				if(gps_valid) 
				{
					if(gps.sat < 9) {
						gps_valid = false;
					}
				}else 
				{
					if(gps.sat >= 9) {
						gps_valid = true;
					}
				}

				if(gps_valid)
				{
					if(!gps_init)
					{
						gps_init_cnt++;
						if(gps_init_cnt > gps_init_num)
						{
							gps_init = true;
						}
					}else
					{
						if(gps_avg_cnt < gps_avg_num)
						{
							home_lat += gps.lat * 1e-7;
							home_lon += gps.lon * 1e-7;
							gps_avg_cnt++;
						}else
						{
							home.lat = home_lat / (double)gps_avg_num;
							home.lon = home_lon / (double)gps_avg_num;
							map_projection_init(&ref, home.lat, home.lon);
							home_init = true;
							gps_init_cnt = 0;
						}
					}
				}
			}else
			{
//				if(g_statusLock != motorLocked)
//				{
//					if(home_reset)
//					{
//						if(gps_valid) 
//						{
//							if(gps.sat < 9) {
//								gps_valid = false;
//							}
//						}else 
//						{
//							if(gps.sat >= 9) {
//								gps_valid = true;
//							}
//						}

//						if(gps_valid)
//						{
//								if(gps_avg_cnt < gps_avg_num)
//								{
//									home_lat += gps.lat * 1e-7;
//									home_lon += gps.lon * 1e-7;
//									gps_avg_cnt++;
//								}else
//								{
//									home.lat = home_lat / (double)gps_avg_num;
//									home.lon = home_lon / (double)gps_avg_num;
//									map_projection_init(&ref, home.lat, home.lon);
//									home_reset = false;
//									gps_init_cnt = 0;
//								}
//						}
//					}
//				}else
//				{
//					home_lat = 0.0f;
//					home_lon = 0.0f;
//					home_reset = true;
//				}
					
				if(gps_valid) 
				{
					if(gps.sat < 9) {
						gps_valid = false;
					}
				}else 
				{
					if(gps.sat >= 9) {
						gps_valid = true;
						reset_est = true;
					}
				}

				if (gps_valid) {
					double lat = gps.lat * 1e-7;
					double lon = gps.lon * 1e-7;

					// project GPS lat lon to plane 
					double gps_proj_x;
					double gps_proj_y;
					bool gps_data_valid;
					map_projection_project(&ref, lat, lon, &gps_proj_x, &gps_proj_y);
					
					//dibei zhuan cibei
//					double gps_proj_x_temp = 0.9848f*gps_proj_x - 0.17385f*gps_proj_y;
//					double gps_proj_y_temp = 0.17385f*gps_proj_x + 0.9848f*gps_proj_y;
//					
//					gps_proj_x = gps_proj_x_temp;
//					gps_proj_y = gps_proj_y_temp;
					
					#if XBEE_POS
						data2send[3] = gps_proj_x * 1000;
						data2send[4] = gps_proj_y * 1000;
						//data2send[15] = gps.sat;
					#endif
					
					if(fabs(gps_proj_x - gps_proj_x_prev)<10 && fabs(gps_proj_y - gps_proj_y_prev)<10)
					{
						gps_data_valid = true;
						gps_pre_t = xTaskGetTickCount ();
					}else
					{
						gps_data_valid = false;
					}

					if(gps_data_valid)
					{
						gps_proj_x_prev = gps_proj_x;
						gps_proj_y_prev = gps_proj_y;
						// reset position estimate when GPS becomes good 
						if (reset_est) {
							x_est[0] = gps_proj_x;
							x_est[1] = gps.vel * cos(gps.vel);
							y_est[0] = gps_proj_y;
							y_est[1] = gps.vel * sin(gps.vel);
						}					

						// calculate index of estimated values in buffer 
						int est_i = buf_ptr - 1 - minimum_int32(EST_BUF_SIZE - 1, delay_gps * 100);
						if (est_i < 0) {
							est_i += EST_BUF_SIZE;
						}

						// calculate correction for position 
						corr_gps[0][0] = gps_proj_x - est_buf[est_i][0][0];
						corr_gps[1][0] = gps_proj_y - est_buf[est_i][1][0];
						
						global_pos.timestamp = xTaskGetTickCount();
						global_pos.pos.x = gps_proj_x;
						global_pos.pos.y = gps_proj_y;
						global_pos.pos.z = 0.0f;
						
						//corr_gps[0][0] = gps_proj_x - x_est[0];
						//corr_gps[1][0] = gps_proj_y - y_est[0];
						

						// calculate correction for velocity 
						if (gps.vel_valid) {
							corr_gps[0][1] = gps.vel * cos((gps.azm )* M_DEG_TO_RAD) - est_buf[est_i][0][1];//+ 10
							corr_gps[1][1] = gps.vel * sin((gps.azm ) * M_DEG_TO_RAD) - est_buf[est_i][1][1];
							global_pos.vel.x = gps.vel * cos((gps.azm ) * M_DEG_TO_RAD);
							global_pos.vel.y = gps.vel * sin((gps.azm ) * M_DEG_TO_RAD);
							global_pos.vel.z = 0.0f;
							global_pos.xy_valid = true;
							
//							data2send[17] = (gps.azm + 10)*10;
							//corr_gps[0][1] = gps.vel * 0.51 * cos(gps.azm * M_DEG_TO_RAD) - x_est[1];
							//corr_gps[1][1] = gps.vel * 0.51 * sin(gps.azm * M_DEG_TO_RAD) - y_est[1];
						} else {
							global_pos.vel.x = 0.0f;
							global_pos.vel.y = 0.0f;
							global_pos.vel.z = 0.0f;
							corr_gps[0][1] = 0.0f;
							corr_gps[1][1] = 0.0f;
						}
						#if XBEE_POS
						//	data2send[10] = gps.azm*10;//yaw_bias
						#endif
						// save rotation matrix at this moment 
						memcpy(R_gps, R_buf[est_i], sizeof(R_gps));
					}

				} else {
					// no GPS lock 
					memset(corr_gps, 0, sizeof(corr_gps));
				}
			}
		}

/*	
		viconAcquire(&vicon);
		if(vicon_timestamp != vicon.timestamp)
		{
			vicon_timestamp = vicon.timestamp;
			
			bool vicon_data_valid = false;
			
			if(fabs(vicon.pos.x) > 0.0001f && fabs(vicon.pos.y) > 0.0001f && fabs(vicon.pos.z) > 0.0001f &&
				fabs(vicon.pos.x - vicon_x_prev)<10 && fabs(vicon.pos.y - vicon_y_prev)<10 &&  fabs(vicon.pos.z - vicon_z_prev)<10)
			{
				vicon_data_valid = true;
			}else
			{
				vicon_data_valid = false;
			}
			
			if(vicon_data_valid)
			{
				// calculate index of estimated values in buffer 
				int est_i = buf_ptr - 1 - minimum_int32(EST_BUF_SIZE - 1, delay_gps * 100);
				if (est_i < 0) {
					est_i += EST_BUF_SIZE;
				}
				
				//corr_vicon[0][0] = vicon.pos.x - est_buf[est_i][0][0];;
				//corr_vicon[1][0] = vicon.pos.y - est_buf[est_i][1][0];
				//corr_vicon[0][1] = vicon.vel.x - est_buf[est_i][0][1];
			//	corr_vicon[1][1] = vicon.vel.y - est_buf[est_i][1][1];;
				
			//	corr_vicon[2][0] = vicon.pos.z - est_buf[est_i][2][0];
				//corr_vicon[2][1] = vicon.vel.z - est_buf[est_i][2][1];
				
				// save rotation matrix at this moment 
				memcpy(R_vicon, R_buf[est_i], sizeof(R_vicon));
				
				corr_vicon[0][0] = vicon.pos.x - x_est[0];
			  corr_vicon[1][0] = vicon.pos.y - y_est[0];
				corr_vicon[0][1] = vicon.vel.x - x_est[1];
				corr_vicon[1][1] = vicon.vel.y - y_est[1];
				
				//corr_vicon[2][0] = vicon.pos.z - z_est[0];
				//corr_vicon[2][1] = vicon.vel.z - z_est[1];
				
				vicon_x_prev = vicon.pos.x;
				vicon_y_prev = vicon.pos.y;
				vicon_z_prev = vicon.pos.z;
			}
			
			vicon_valid = true;
		}

*/		
		float dt = 0.01f;
		if(t_prev > 0) dt = (t - t_prev) / 1000.0f ;
		else dt = 0;
		if(dt < 0.002f) dt = 0.002f;
		else if(dt > 0.02f) dt = 0.02f;
		
		t_prev = t;

		/* use GPS if it's valid and reference position initialized */
		bool use_gps_xy = home_init && gps_valid ;
		bool can_estimate_xy = use_gps_xy || vicon_valid;

		/* accelerometer bias correction for GPS (use buffered rotation matrix) */
		float accel_bias_corr[3] = { 0.0f, 0.0f, 0.0f };

		if (use_gps_xy) {
			accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
			accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
			accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
			
			if(isfinite(accel_bias_corr[0])){
				acc_bias_earth[0] += accel_bias_corr[0] * w_acc_xy_bias * dt;
			}
			if(isfinite(accel_bias_corr[1])){
				acc_bias_earth[1] += accel_bias_corr[1] * w_acc_xy_bias * dt;
			}
			
				/* transform error vector from NED frame to body frame */
			for (int i = 0; i < 3; i++) {
				float c = 0.0f;

				for (int j = 0; j < 3; j++) {
					//c += att.R.R[j][i] * accel_bias_corr[j];
					c += R_gps[j][i] * accel_bias_corr[j];
				}

				if (isfinite(c)) {
					acc_bias[i] += c * w_acc_xy_bias * dt;
					acc_bias[i] = constrain_f(acc_bias[i], -acc_bias_max, acc_bias_max);
				}
			}		
		}
		
//		if (vicon_valid) {
//			accel_bias_corr[0] -= corr_vicon[0][0] * w_xy_vicon_p * w_xy_vicon_p;
//			accel_bias_corr[0] -= corr_vicon[0][1] * w_xy_vicon_v;
//			accel_bias_corr[1] -= corr_vicon[1][0] * w_xy_vicon_p * w_xy_vicon_p;
//			accel_bias_corr[1] -= corr_vicon[1][1] * w_xy_vicon_v;
//			//accel_bias_corr[2] -= corr_vicon[2][0] * w_z_vicon_p * w_z_vicon_p;
//			//accel_bias_corr[2] -= corr_vicon[2][1] * w_z_vicon_v;
//			
//			/* transform error vector from NED frame to body frame */
//			for (int i = 0; i < 3; i++) {
//				float c = 0.0f;

//				for (int j = 0; j < 3; j++) {
//					c += att.R.R[j][i] * accel_bias_corr[j];
//					//c += R_vicon[j][i] * accel_bias_corr[j];
//				}

//				if (isfinite(c)) {
//					acc_bias[i] += c * w_acc_bias * dt;
//				}
//			}
//		}
		
		if(use_baro)
		{
			accel_bias_corr[0] = 0.0f;
			accel_bias_corr[1] = 0.0f;
			accel_bias_corr[2] = 0.0f;

			accel_bias_corr[2] -= corr_baro * w_z_baro * w_z_baro;
			
			if(isfinite(accel_bias_corr[2])){
				acc_bias_earth[2] += accel_bias_corr[2] * w_acc_z_bias * dt;
			}

			/* transform error vector from NED frame to body frame */
			for (int i = 0; i < 3; i++) {
				float c = 0.0f;

				for (int j = 0; j < 3; j++) {
					c += att.R.R[j][i] * accel_bias_corr[j];
				}

				if (isfinite(c)) {
					acc_bias[i] += c * w_acc_z_bias * dt;
					acc_bias[i] = constrain_f(acc_bias[i], -acc_bias_max, acc_bias_max);
				}
			}
		}

		/* inertial filter  for altitude */
		inertial_filter_predict(dt, z_est, acc[2]);
		if(use_baro)
		{
			inertial_filter_correct(corr_baro, dt, z_est, 0, w_z_baro);// * 2.5 / (1.5 + fabs(att.R.R[2][2])));//dongshixiong: *att.R.R[2][2] *att.R.R[2][2] *att.R.R[2][2]
		}
		
		/*if(vicon_valid)
		{
			inertial_filter_correct(corr_vicon[2][0], dt, z_est, 0, w_z_vicon_p);
			inertial_filter_correct(corr_vicon[2][1], dt, z_est, 1, w_z_vicon_v);
		}
*/
		if (can_estimate_xy) {
			/* inertial filter prediction for position */
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);
			if (use_gps_xy) {
				inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
				inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

				if (gps.vel_valid) {
					inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
					inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
				}
			}
			
/*			if (vicon_valid) {
				inertial_filter_correct(corr_vicon[0][0], dt, x_est, 0, w_xy_vicon_p);
				inertial_filter_correct(corr_vicon[1][0], dt, y_est, 0, w_xy_vicon_p);
				inertial_filter_correct(corr_vicon[0][1], dt, x_est, 1, w_xy_vicon_v);
				inertial_filter_correct(corr_vicon[1][1], dt, y_est, 1, w_xy_vicon_v);
			}*/
		}
		////////////////////////////yexin///////////////////////////////////////////////////
/*		#if 1
		if(gps_azm_correct){
			gps_azm_correct = false;
			if(gps_valid && gps.vel_valid){
				
				if(g_mode == modePos){
					int est_j = buf_ptr - 1 - minimum_int32(EST_BUF_SIZE - 1, 0.2 * 100);
					if (est_j < 0) {
						est_j += EST_BUF_SIZE;
					}
					float gps_azm = gps.azm * M_DEG_TO_RAD;
					if(gps_azm>PI)
						gps_azm-=2*PI;
					if(gps_azm<-PI)
						gps_azm+=2*PI;
					float est_azm = atan2(est_buf[est_j][1][1], est_buf[est_j][0][1]);
					float corr_azm = gps_azm - est_azm;//-yaw_bias;
					if(corr_azm > PI)
						corr_azm -= 2*PI;
					if(corr_azm < -PI)
						corr_azm += 2*PI;
					if(absolute_f(corr_azm)>0.5f)
						corr_azm = 0;
					
				//	global_pos.yaw_bias = corr_azm;
					yaw_bias += corr_azm * w_azm * 0.2f;
					yaw_bias = constrain_f(yaw_bias, -0.12f,0.12f);
					global_pos.yaw_bias = yaw_bias;
					#if SEND_AZM
					data2send[6] = global_pos.yaw_bias * 573;
					data2send[9] = gps_azm * 573;
					data2send[10] = est_azm * 573;
					data2send[12] = est_buf[est_j][0][1]*1000;
					data2send[13] = est_buf[est_j][1][1]*1000;
					data2send[15] = global_pos.vel.x*1000;
					data2send[16] = global_pos.vel.y*1000;
					#endif
				}
			}
		}
		#else
		if(gps_azm_correct){
			gps_azm_correct = false;
			if(gps_valid && gps.vel_valid){
			//	static int last_gps_time = 0;
			//	static vec3f_t gps_acc, gps_last_vel;
			//	float gps_dt = (gps.timestamp - last_gps_time)/100.0f;
			//	if( gps_dt>0 && last_gps_time>0){
			//	if(global_pos.vel.v[0] != gps_last_vel.v[0]||global_pos.vel.v[1] != gps_last_vel.v[1]){
			//		for(int i=0;i<3;i++)
			//			gps_acc.v[i] = (global_pos.vel.v[i] - gps_last_vel.v[i])/gps_dt;
			//		last_gps_time = gps.timestamp;
			//		for(int i=0;i<3;i++)
			//			gps_last_vel.v[i] = global_pos.vel.v[i];
			//	}else{
			//		for(int i=0;i<3;i++)
			//			gps_acc.v[i] = 0;
			//	}
				
				int est_j = buf_ptr - 1 - minimum_int32(EST_BUF_SIZE - 1, delay_gps * 100);
				if (est_j < 0) {
					est_j += EST_BUF_SIZE;
				}
				float gps_azm = atan2(global_pos.acc.v[1], global_pos.acc.v[0]);
				float est_azm = atan2(est_buf[est_j][1][2], est_buf[est_j][0][2]);
				float corr_azm = gps_azm - est_azm;
				if(corr_azm > PI)
					corr_azm -= 2*PI;
				if(corr_azm < -PI)
					corr_azm += 2*PI;
				
			//	global_pos.yaw_bias = corr_azm;
				yaw_bias += corr_azm * w_azm * 0.2f;
				yaw_bias = constrain_f(yaw_bias, -0.12f,0.12f);
				global_pos.yaw_bias = yaw_bias;
			//	last_gps_time = gps.timestamp;
				#if SEND_ATT
				data2send[6] = global_pos.yaw_bias * 573;
				data2send[7] = gps_azm * 573;
				data2send[8] = est_azm * 573;
				data2send[9] = global_pos.vel.x*1000;
				data2send[10] = global_pos.vel.y*1000;
			//	data2send[11]=gps_dt*1000;
				data2send[12] = global_pos.acc.x * 1000;
				data2send[13] = global_pos.acc.y * 1000;
				data2send[15] = est_buf[est_j][0][2] * 1000;
				data2send[16] = est_buf[est_j][1][2] * 1000;
				#endif
			}
		}
		#endif
		
*/		
		////////////////////////////////////////////////////////////////////////////////////
		/* push current estimate to buffer */
		est_buf[buf_ptr][0][0] = x_est[0];
		est_buf[buf_ptr][0][1] = x_est[1];
		est_buf[buf_ptr][1][0] = y_est[0];
		est_buf[buf_ptr][1][1] = y_est[1];
		est_buf[buf_ptr][2][0] = z_est[0];
		est_buf[buf_ptr][2][1] = z_est[1];
		/* push current rotation matrix to buffer */
		memcpy(R_buf[buf_ptr], att.R.R, sizeof(att.R.R));

		buf_ptr++;
		if (buf_ptr >= EST_BUF_SIZE) {
			buf_ptr = 0;
		}

		local_pos.pos.x = x_est[0];
		local_pos.pos.y = y_est[0];
		local_pos.pos.z = z_est[0];
		local_pos.vel.x = x_est[1];
		local_pos.vel.y = y_est[1];
		local_pos.vel.z = z_est[1];
		local_pos.acc.x = acc[0];
		local_pos.acc.y = acc[1];
		local_pos.acc.z = acc[2];
		
		if(can_estimate_xy)
			local_pos.xy_valid  = true;
		else
			local_pos.xy_valid  = false;
		#if XBEE_POS
			data2send[5] = (baro_offset - baro.alt) * 1000;
		
			data2send[6] = local_pos.pos.x * 1000;
			data2send[7] = local_pos.pos.y * 1000;
			data2send[8] = local_pos.pos.z * 1000;
		
			data2send[9] = local_pos.vel.x * 1000;
			data2send[10] = local_pos.vel.y * 1000;	
			data2send[11] = local_pos.vel.z * 1000;	
			
			data2send[12] = global_pos.vel.x * 1000;
			data2send[13] = global_pos.vel.y * 1000;	
	//		data2send[14] = global_pos.vel.z * 1000;	
			
		#endif

		local_pos.timestamp = xTaskGetTickCount();
		xQueueOverwrite(pos_q, &local_pos);
		xQueueOverwrite(global_pos_q, &global_pos);

	}
}
void acc_calibrate(const vec3f_t* acc, const rotation_t* R)
{
	unsigned int i;
	vec3f_t acc_earth;
	for(i=0;i<3;i++)
		acc_earth.v[i] = acc->v[i];
	acc_earth.z += CONSTANTS_ONE_G;
	earth2body(R, &acc_earth, &_acc_bias_body, 3);
	
}
static void acc_filter( void *pvParameters )
{
	marg_t imu;
	memset(&imu, 0, sizeof(imu));
	marg_t imu_filtered;
	memset(&imu_filtered, 0, sizeof(imu_filtered));
	float acc_x[MEAN_BUF];
	float acc_y[MEAN_BUF];
	float acc_z[MEAN_BUF];
	memset(acc_x, 0, sizeof(acc_x));
	memset(acc_y, 0, sizeof(acc_y));
	memset(acc_z, 0, sizeof(acc_z));
	
	int cnt = 0;
	
	uint32_t lastWakeTime = 0;
	lastWakeTime = xTaskGetTickCount ();
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, 1);
		
		margAcquire(&imu);
		acc_x[cnt] = imu.acc.v[0];
		acc_y[cnt] = imu.acc.v[1];
		acc_z[cnt] = imu.acc.v[2];
		
		cnt++;
		if(cnt >= MEAN_BUF)
		{
			cnt = 0;
		}
		
		float acc_sum_x = 0.0f;
		float acc_sum_y = 0.0f;
		float acc_sum_z = 0.0f;
		
		float acc_std_x = 0.0f;
		float acc_std_y = 0.0f;
		float acc_std_z = 0.0f;
		
		for(int i=0; i<MEAN_BUF; i++)
		{
			acc_sum_x +=  acc_x[i];
			acc_sum_y +=  acc_y[i];
			acc_sum_z +=  acc_z[i];
		}
		
		_acc_filtered.v[0] = acc_sum_x / MEAN_BUF;
		_acc_filtered.v[1] = acc_sum_y / MEAN_BUF;
		_acc_filtered.v[2] = acc_sum_z / MEAN_BUF;
		
		arm_std_f32(acc_x,MEAN_BUF,&acc_std_x);
		arm_std_f32(acc_y,MEAN_BUF,&acc_std_y);
		arm_std_f32(acc_z,MEAN_BUF,&acc_std_z);
		
		accProcessed_t acc_processed;
		acc_processed.timestamp = xTaskGetTickCount();
		acc_processed.mean.v[0] = _acc_filtered.v[0];
		acc_processed.mean.v[0] = _acc_filtered.v[0];
		acc_processed.mean.v[0] = _acc_filtered.v[0];
		acc_processed.std.v[0] = acc_std_x;
		acc_processed.std.v[0] = acc_std_y;
		acc_processed.std.v[0] = acc_std_z;
		xQueueOverwrite(acc_processed_q,&acc_processed);
			
	}
}
void position_estimation_start(void)
{
	xTaskCreate(position_estimation, "position_estimation", 1200, NULL, 2, NULL);
}
void position_estimation_queue_init(void)
{
	pos_q = xQueueCreate(1, sizeof(pos_t));
	global_pos_q = xQueueCreate(1, sizeof(pos_t));
	acc_processed_q = xQueueCreate(1, sizeof(accProcessed_t));
	acc_d_cali_q =  xQueueCreate(1, sizeof(float));
}
void acc_filter_start(void)
{
	xTaskCreate(acc_filter, "acc_filter", configMINIMAL_STACK_SIZE*2, NULL, 1, NULL);
}
BaseType_t posAcquire(pos_t *pos)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(pos_q, pos, 0);
	return pdres;
}

BaseType_t globalposAcquire(pos_t *pos)
{
	BaseType_t pdres=pdFALSE;
	xQueuePeek(global_pos_q, pos, 0);
	return pdres;
}


BaseType_t accAcquire(accProcessed_t *acc)
{
	BaseType_t pdres=pdFALSE;
	pdres = xQueuePeek(acc_processed_q, acc, 0);
	return pdres;
}


