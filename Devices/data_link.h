#ifndef DATA_LINK_H
#define DATA_LINK_H
#include "../MessageTypes/type_methods.h"
unsigned short crc16(void* data, unsigned short cnt);
void send_data(void *data);
void data_link_init(void);
void DataLinkReceive_IDLE(void);
void xbee_commandBlockingAcquire(attCmd_t *cmd);

void xbee_outdoor_commandBlockingAcquire(posCmd_t *cmd);//added by Wade
void xbee_outdoor_commandAcquire(posCmd_t *cmd);//added by Wade
void xbee_rcAcquire(rc_t *rc);//added by Wade

void xbee_motionAccAcquire(vec3f_t *motion_acc);
void data_send_start(void);
#endif
