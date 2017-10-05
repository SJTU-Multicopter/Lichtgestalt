#include "situation_awareness.h"
#include "../Commons/platform.h"
#include "../MessageTypes/type_methods.h"

//retVal: 0x01010101
//eight bits:
//0,0,0,on air, on land, at rest, upside down, free fall
unsigned char situAwareUpdate(attsp_t *attsp, const marg_t *marg, const att_t *state)
{
	unsigned char retVal=0x00;
	
	
	return retVal;
}
