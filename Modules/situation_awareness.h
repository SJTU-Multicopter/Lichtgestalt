#ifndef SITUATION_AWARENESS
#define SITUATION_AWARENESS
#include "../MessageTypes/type_methods.h"

#define SIT_IN_AIR (1<<4)
#define SIT_ON_LAND (1<<3)
#define SIT_AT_REST (1<<2)
#define SIT_UPSIDEDOWN (1<<1)
#define SIT_FREE_FALL (1<<0)
unsigned char situAwareUpdate(attsp_t *attsp, const marg_t *marg, const att_t *state);

#endif
