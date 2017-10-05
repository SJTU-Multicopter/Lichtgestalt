#ifndef POS_CTRL_H
#define POS_CTRL_H
#include "../MessageTypes/pid_methods.h"
#include "../MessageTypes/type_methods.h"
#include "cmsis_os.h"
void position_controller_init(const att_t* att);
void position_controller_queue_init(void);
BaseType_t attspAcquire(attsp_t *attsp);
BaseType_t attspBlockingAcquire(attsp_t *attsp);
extern PID_t altPID;
extern PID_t pos_xPID;
extern PID_t pos_yPID;
//extern xQueueHandle attsp_q;
#endif
