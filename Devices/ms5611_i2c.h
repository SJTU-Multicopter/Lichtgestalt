#ifndef MS5611_I2C
#define MS5611_I2C
#include "../MessageTypes/type_methods.h"
void ms5611_init(void);
void ms5611_read_trigger(void);
void baroProcess(baro_t* data);
void ms5611_temp_start(void);
void ms5611_pres_start(void);
#endif
