#ifndef COMMANDER_H
#define COMMANDER_H
#include "../MessageTypes/type_methods.h"
#include "cmsis_os.h"
//void attspAcquire(attsp_t* sp);
void commanderInit(void);
//void attsp_reset(const att_t* state);
BaseType_t  posSetpointAcquire(posCtrlsp_t *sp);

//BaseType_t  altSetpointAcquire(altCtrlsp_t *sp);

BaseType_t  manSetpointAcquire(manCtrlsp_t *sp);
#endif
