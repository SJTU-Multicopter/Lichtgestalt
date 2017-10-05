#ifndef CONTROLLER_H
#define CONTROLLER_H
#include "../MessageTypes/type_methods.h"
#include "../MessageTypes/pid_methods.h"
void stateController(output_t *output, 
	const att_t *state,const attsp_t *attsp, float dt);
void outputBlockingAcquire(output_t *output);
void attitude_controller_init(void);
extern PID_t pitchPID;
extern PID_t rollPID;
extern PID_t yawPID;
#endif
