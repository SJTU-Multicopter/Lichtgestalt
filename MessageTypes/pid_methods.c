#include "pid_methods.h"

float external_err_pid(PID_t *pid, float err)
{
 	 float retValue;								
 	 pid->Err = err;
 	 retValue = pid->P * pid->Err;
	 return retValue;
}
float internal_err_pid(PID_t *pid, float rate_err, float dt)
{
	float rateOuput;								
  	pid->RateErr = rate_err;
  	rateOuput = pid->Prate * pid->RateErr + pid->Drate * (pid->RateErr - pid->l_RateErr) / dt + pid->Irate * pid->int_RateErr;
  	pid->l_RateErr = pid->RateErr;
	pid->int_RateErr += pid->RateErr * dt;
	if(pid->int_RateErr > pid->int_constrain)
		pid->int_RateErr = pid->int_constrain;
	if(pid->int_RateErr < -pid->int_constrain)
		pid->int_RateErr = -pid->int_constrain;
  	return rateOuput;
}

void reset_pid(PID_t *pid)
{
	pid->int_RateErr = 0.0f;
	pid->Err = 0.0f;
	pid->RateErr = 0.0f;
	pid->l_RateErr = 0.0f;
}
void init_pid(PID_t *pid, float P, float Prate, float Irate, float Drate)
{
	pid->P = P;
	pid->Prate = Prate;
	pid->Irate = Irate;
	pid->Drate = Drate;
	pid->Err = 0.0f;
	pid->RateErr = 0.0f;
	pid->l_RateErr = 0.0f;
	pid->int_RateErr = 0.0f;
}
