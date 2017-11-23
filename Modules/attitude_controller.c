#include "attitude_controller.h"
#include "position_controller.h"
#include "commander.h"
#include "../Devices/battery.h"
#include "attitude_estimator.h"
#include "motor_mixer.h"
#include "../MessageTypes/type_methods.h"

#include "../Mathlib/comparison.h"
#include "cmsis_os.h"
#include "../config/config.h"
static att_t _att;
static output_t _output;
static attsp_t _attsp;
//static battery_t _bat;
static xQueueHandle output_q;
static void attitude_control_Task( void *pvParameters );

PID_t pitchPID={0,0,0,0,
			1.6,16.0,
			0.015,0.004f,
			3.0f};
PID_t rollPID={0,0,0,0,
			1.6,16.0,
			0.015,0.004f,
			3.0f};
PID_t yawPID={0,0,0,0,
			0.75,10.0,
			0.0,0.0,
			3.0f};

void attitude_controller(output_t *output, 
	const att_t *state,const attsp_t *attsp, float dt)
{
//	int i;
	float p_e_R_hat[3][3];//measured body to desired body
	float p_R_est[3][3];
	float p_R_sp[3][3];
	float p_R_est_t[3][3];
	float p_R_sp_t[3][3];
	float p_temp1[3][3];
	float p_temp2[3][3];
	float p_temp3[3][3];
	float e_R[3], e_w[3];
	float w_sp[3];
	memcpy(p_R_est,state->R.R,sizeof(p_R_est));
	memcpy(p_R_sp,attsp->R.R,sizeof(p_R_sp));
	arm_matrix_instance_f32 e_R_hat = {3,3,(float *)p_e_R_hat};
	arm_matrix_instance_f32 R_est = {3,3,(float *)p_R_est};
	arm_matrix_instance_f32 R_sp = {3,3,(float *)p_R_sp};
	arm_matrix_instance_f32 R_est_t = {3,3,(float *)p_R_est_t};
	arm_matrix_instance_f32 R_sp_t = {3,3,(float *)p_R_sp_t};
	arm_matrix_instance_f32 temp1 = {3,3,(float *)p_temp1};
	arm_matrix_instance_f32 temp2 = {3,3,(float *)p_temp2};
	arm_matrix_instance_f32 temp3 = {3,3,(float *)p_temp3};
//	e_R_hat = (R_sp.transposed() * R - R.transposed() * R_sp) * (0.5f);
	arm_mat_trans_f32(&R_est, &R_est_t);
	arm_mat_trans_f32(&R_sp, &R_sp_t);
	arm_mat_mult_f32(&R_sp_t,&R_est,&temp1);
	arm_mat_mult_f32(&R_est_t,&R_sp,&temp2);
	arm_mat_sub_f32(&temp1,&temp2,&temp3);
	arm_mat_scale_f32(&temp3,0.5f,&e_R_hat);
	e_R[0] = (p_e_R_hat[1][2] - p_e_R_hat[2][1]) / 2.0f;//roll err in rad<<14
	e_R[1] = (p_e_R_hat[2][0] - p_e_R_hat[0][2]) / 2.0f;//pitch err in rad<<14
	e_R[2] = (p_e_R_hat[0][1] - p_e_R_hat[1][0]) / 2.0f;//yaw err in rad<<14
	w_sp[0] = external_err_pid(&rollPID, e_R[0]);//RollRate_sp
	w_sp[1] = external_err_pid(&pitchPID, e_R[1]);//PitchRate_sp
	w_sp[2] = external_err_pid(&yawPID, e_R[2]);//YawRate_sp
	
	e_w[0] = w_sp[0] - state->rate.R;
	e_w[1] = w_sp[1] - state->rate.P;
	e_w[2] = w_sp[2] - state->rate.Y;
	output->moment.R = internal_err_pid(&rollPID, e_w[0], ATT_CTRL_TASK_PERIOD_S);
	output->moment.P = internal_err_pid(&pitchPID, e_w[1], ATT_CTRL_TASK_PERIOD_S);
	output->moment.Y = internal_err_pid(&yawPID, e_w[2], ATT_CTRL_TASK_PERIOD_S);
	output->thrust = attsp->thrust;
}
static void attitude_control_Task( void *pvParameters )
{
	uint32_t lastWakeTime;
	lastWakeTime = xTaskGetTickCount ();
	while(1) {
		vTaskDelayUntil(&lastWakeTime, ATT_CTRL_TASK_PERIOD_MS);
		attAcquire(&_att);
		attspAcquire(&_attsp);
		attitude_controller(&_output, &_att, &_attsp, ATT_CTRL_TASK_PERIOD_S);
		_output.timestamp = xTaskGetTickCount ();
		xQueueOverwrite(output_q, &_output);
	}
}
void attitude_controller_init(void)
{
	unsigned int i;
	for(i=0;i<3;i++){
		_output.moment.v[i] = 0;
	}
	_output.thrust = 0;
	output_q = xQueueCreate(1, sizeof(output_t));
	xTaskCreate(attitude_control_Task, "attCtrl", ATT_CTRL_TASK_STACKSIZE, NULL, ATT_CTRL_TASK_PRI, NULL);
}
void outputAcquire(output_t *output)
{
	xQueuePeek(output_q, output, 0);
}
void outputBlockingAcquire(output_t *output)
{
	xQueueReceive(output_q, output, portMAX_DELAY);
}

