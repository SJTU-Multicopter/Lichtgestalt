#ifndef PID_METHODS_H
#define PID_METHODS_H

typedef struct PID_s {
	float Err;
	float RateErr;
	float l_RateErr;
	float int_RateErr;
	float P;		
	float Prate;
	float Irate;
	float Drate;
	float int_constrain;
}PID_t;
float external_err_pid(PID_t *pid, float err);

float internal_err_pid(PID_t *pid, float rate_err, float dt);
void reset_pid(PID_t *pid);
void init_pid(PID_t *pid, float P, float Prate, float Irate, float Drate);

#endif
