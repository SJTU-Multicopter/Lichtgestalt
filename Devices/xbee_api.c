#include "xbee_api.h"
#include "string.h"
#include "cmsis_os.h"
unsigned char api_pack_decode(unsigned char * data, unsigned int pack_len)
{
	unsigned char start_delimiter;
	unsigned char api_len;
	unsigned char zero = 0;
	unsigned char checksum = 0;
	unsigned char checksum_comming;
	unsigned char i = 0;

	unsigned char frame_type;
	memcpy(&start_delimiter, data, 1);
	memcpy(&zero, data + 1, 1);
	memcpy(&api_len, data + 2, 1);
	memcpy(&checksum_comming, data + pack_len - 1, 1);
	if(start_delimiter != API_START_DELIMITER)
		return 1;//none XBee API
	for(i=0;i<api_len;i++){
		checksum += *((unsigned char *)(data)+i+3);
	}
	checksum = 0xFF-checksum;
	if(checksum != checksum_comming)
		return 2;//checksum failed
	else
		memcpy(&frame_type, data + 3, 1);
	return frame_type;
}
void api_tx_status_decode(unsigned char * data, unsigned int pack_len)
{
	unsigned char frame_id, dest_addr_high, dest_addr_low, retry_cnt;
	unsigned char delivery_status, discovery_status;
	memcpy(&frame_id, data + 4, 1);
	memcpy(&dest_addr_high, data + 5, 1);
	memcpy(&dest_addr_low, data + 6, 1);
	memcpy(&retry_cnt, data + 7, 1);
	memcpy(&delivery_status, data + 8, 1);
	memcpy(&discovery_status, data + 9, 1);
}
unsigned char  api_rx_decode(unsigned char * data, unsigned int pack_len)
{
	unsigned int src_address_high, src_address_low;
	unsigned short src_network_address;
	unsigned char rcv_options;
	unsigned char descriptor;
	memcpy(&src_address_high, data + 4, 4);
	memcpy(&src_address_low, data + 8, 4);
	memcpy(&src_network_address, data + 12, 2);
	memcpy(&rcv_options, data + 14, 1);
	memcpy(&descriptor, data + 15, 1);
	return descriptor;
}
void decode_cmd_acc(unsigned char * data, unsigned int pack_len, attCmd_t* cmd, vec3f_t* mot_acc)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x01	12-15
//	t,t,t,t,		16-19
//	q0,q0,q1,q1		20-23
//	q2,q2,q3,q3		24-27
//	thr,thr,ax,ax	28-31
//	ay,ay,az,az		32-35
//	checksum		36	
	short q[4];
	short thrust;
	short acc[3];
	int i;
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(q, data + 20, 8);
	memcpy(&thrust, data + 28, 2);
	memcpy(acc, data + 30, 6);
	cmd->Q.q0 = (float)q[0] / ATT_F;
	cmd->Q.q1 = (float)q[1] / ATT_F;
	cmd->Q.q2 = (float)q[2] / ATT_F;
	cmd->Q.q3 = (float)q[3] / ATT_F;
	cmd->thrust = (float)thrust / THR_F;
	for(i=0;i<3;i++)
		mot_acc->v[i] = (float)acc[i] / ACC_F;
}
void decode_rc(unsigned char * data, unsigned int pack_len, rc_t* rc)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x05	12-15
//	t,t,t,t,		16-19
//	c0,c0,c1,c1		20-23
//	c2,c2,c3,c3		24-27
//	c4,c4,c5,c5		28-31
//	c6,c6,c7,c7		32-35
//	c8,c8,c9,c9		36-39	
//	checksum		40	
	short channel[10];
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(channel, data + 20, 20);
	for(int i=0;i<10;i++)
		rc->channels[i] = (float)channel[i] / 1000.0f;
	rc->timestamp = timestamp;
}
/*added by Wade*/
void decode_pos_sp(unsigned char * data, unsigned int pack_len, posCmd_t* cmd)
{
	int pos[3];
	short vel[3], acc[3], yw, e;
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(pos, data + 20, 12);
	memcpy(vel, data + 32, 6);
	memcpy(acc, data + 38, 6);
	memcpy(&yw, data + 44, 2);
	memcpy(&e, data + 46, 20);
	cmd->pos_sp.v[0] = (float)pos[0] / 1000;
	cmd->pos_sp.v[1] = (float)pos[1] / 1000;
	cmd->pos_sp.v[2] = (float)pos[2] / 1000;
	cmd->vel_ff.v[0] = (float)vel[0] / 1000;
	cmd->vel_ff.v[1] = (float)vel[1] / 1000;
	cmd->vel_ff.v[2] = (float)vel[2] / 1000;
	cmd->acc_ff.v[0] = (float)acc[0] / 1000;
	cmd->acc_ff.v[1] = (float)acc[1] / 1000;
	cmd->acc_ff.v[2] = (float)acc[2] / 1000;
	cmd->yaw_sp = yw / YAW_F;
	cmd->commands = e;
}
void decode_pid(unsigned char * data, unsigned int pack_len, PID_t* PRpid1, PID_t* PRpid2, PID_t* Ypid)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x01	12-15
//	t,t,t,t,		16-19
//	P,P,P,P		20-23
//	p,p,p,p		24-27
//	i,i,i,i		28-31
//	d,d,d,d		32-35
//	P,P,P,P		36-39
//	p,p,p,p		40-43
//	i,i,i,i		44-47
//	d,d,d,d		48-51
//	checksum		52	
	float pr_P,pr_p,pr_i,pr_d;
	float y_P,y_p,y_i,y_d;
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(&pr_P, data + 20, 4);
	memcpy(&pr_p, data + 24, 4);
	memcpy(&pr_i, data + 28, 4);
	memcpy(&pr_d, data + 32, 4);
	memcpy(&y_P, data + 36, 4);
	memcpy(&y_p, data + 40, 4);
	memcpy(&y_i, data + 44, 4);
	memcpy(&y_d, data + 48, 4);
	PRpid1->P = pr_P;
	PRpid1->Prate = pr_p;
	PRpid1->Irate = pr_i;
	PRpid1->Drate = pr_d;
	PRpid2->P = pr_P;
	PRpid2->Prate = pr_p;
	PRpid2->Irate = pr_i;
	PRpid2->Drate = pr_d;
	Ypid->P = y_P;
	Ypid->Prate = y_p;
	Ypid->Irate = y_i;
	Ypid->Drate = y_d;
}
void decode_calibrate(unsigned char * data, unsigned int pack_len, calib_t* cal)
{
//	[3E,0,api_len,90,	0-3
//	add_H,H,H,H,	4-7
//	add_L,L,L,L,	8-11
//	src_net,src_net,rcv_opt,descriptor 0x02	12-15
//	t,t,t,t,		16-19
//	ax,ax,ay,ay		20-23
//	az,az,gx,gx		24-27
//	gy,gy,gz,gz		28-31
//	mx,mx,my,my		32-35
//	mz,mz,checksum	36-38	
	int i;
	short acc[3], gyr[3],mag[3];
	unsigned int timestamp;
	memcpy(&timestamp, data + 16, 4);
	memcpy(acc, data + 20, 6);
	memcpy(gyr, data + 26, 6);
	memcpy(mag, data + 32, 6);
	for(i=0;i<3;i++){
		cal->acc_bias.v[i] = acc[i];
		cal->gyr_bias.v[i] = gyr[i];
		cal->mag_bias.v[i] = mag[i];
	}
}
void api_tx_encode(unsigned char * data, unsigned int dest_addr_h, unsigned int dest_addr_l)
{
	unsigned char api_id = API_ID_TX_REQ;
	unsigned char frame_id = 0x22;
	unsigned char zero = 0;
	unsigned char unknown_16_addr_h = 0xFF;
	unsigned char unknown_16_addr_l = 0xFE;
	memcpy(data+3,&api_id,1);
	memcpy(data+4,&frame_id,1);
	memcpy(data+5,&dest_addr_h,4);
	memcpy(data+9,&dest_addr_l,4);
	memcpy(data+13,&unknown_16_addr_h,1);
	memcpy(data+14,&unknown_16_addr_l,1);
	memcpy(data+15,&zero,1);
	memcpy(data+16,&zero,1);
}
void api_pack_encode(unsigned char * data, unsigned char frame_len)
{
	unsigned char start_delimiter = API_START_DELIMITER;
	unsigned char zero = 0;
	unsigned char checksum = 0;
	unsigned char i = 0;
	for(i=0;i<frame_len;i++){
		checksum += *((unsigned char *)(data)+i+3);
	}
	checksum = 0xFF-checksum;
	memcpy(data,&start_delimiter,1);
	memcpy(data+1,&zero,1);
	memcpy(data+2,&frame_len,1);
	memcpy(data+3+frame_len,&checksum,1);
}
unsigned char encode_sens_raw(unsigned char * data, const vec3i16_t* acc, const vec3i16_t* gyr,const vec3i16_t* mag)
{
	unsigned char descriptor = DSCR_SENS_RAW;
	int i;
	short a[3], g[3],m[3];
	unsigned int timestamp = xTaskGetTickCount();
	for(i=0;i<3;i++){
		a[i] = acc->v[i];
		g[i] = gyr->v[i];
		m[i] = mag->v[i];
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, a, 6);
	memcpy(data + 28, g, 6);
	memcpy(data + 34, m, 6);
	return 23;//length from desc to last data
}
unsigned char encode_pid(unsigned char * data, float pr_P,float pr_p,float pr_i,float pr_d,float y_P,float y_p,float y_i,float y_d)
{
	unsigned char descriptor = DSCR_PID;
	unsigned int timestamp = xTaskGetTickCount();

	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, &pr_P, 4);
	memcpy(data + 26, &pr_p, 4);
	memcpy(data + 30, &pr_i, 4);
	memcpy(data + 34, &pr_d, 4);
	memcpy(data + 38, &y_P, 4);
	memcpy(data + 42, &y_p, 4);
	memcpy(data + 46, &y_i, 4);
	memcpy(data + 50, &y_d, 4);
	return 37;
}
unsigned char encode_sens(unsigned char * data, const marg_t * marg)
{
	unsigned char descriptor = DSCR_SENS;
	int i;
	short a[3], g[3],m[3];
	unsigned int timestamp = xTaskGetTickCount();
	for(i=0;i<3;i++){
		a[i] = marg->acc.v[i];
		g[i] = marg->gyr.v[i];
		m[i] = marg->mag.v[i];
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, a, 6);
	memcpy(data + 28, g, 6);
	memcpy(data + 34, m, 6);
	return 23;//length from desc to last data
}
unsigned char encode_att(unsigned char * data, const att_t* att)
{
	unsigned char descriptor = DSCR_ATT;
	short q[4];
	int i;
	unsigned int timestamp = xTaskGetTickCount();
	for(i=0;i<4;i++){
		q[i] = att->Q.q[i]*ATT_F;
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, q, 8);
	return 13;//length from desc to last data
}
unsigned char encode_yaw(unsigned char * data, const att_t* att)
{
	unsigned char descriptor = DSCR_YAW;
	unsigned int timestamp = 0;
	short yaw = att->Euler.Y * YAW_F;
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, &yaw, 2);
	return 7;//length from desc to last data
}
unsigned char encode_general_18(unsigned char * data, const void * data2send)
{
	unsigned char descriptor = DSCR_GEN;
	unsigned int timestamp = xTaskGetTickCount();
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, data2send, 36);
	return 41;//length from desc to last data
}
unsigned char encode_pos_yaw(unsigned char * data, const att_t* att, const pos_t* pos)
{
	unsigned char descriptor = DSCR_POSYAW;
	unsigned int timestamp = xTaskGetTickCount();
	short yaw = att->Euler.Y * YAW_F;
	int pos_i[3];
	for(int i=0;i<3;i++){
		pos_i[i] = pos->pos.v[i] * 1000;
	}
	memcpy(data + 17, &descriptor, 1);
	memcpy(data + 18, &timestamp, 4);
	memcpy(data + 22, pos_i, 12);
	memcpy(data + 34, &yaw, 2);
	return 19;//length from desc to last data
}
unsigned short crc_update (unsigned short crc, unsigned char data)
{
    data ^= (crc & 0xff);
    data ^= data << 4;
    return ((((unsigned short )data << 8) | ((crc>>8)&0xff)) ^ (unsigned char )(data >> 4)
         ^ ((unsigned short )data << 3));
}
unsigned short crc16(void* data, unsigned short cnt)
{
    unsigned short crc=0xff;
    unsigned char * ptr=(unsigned char *) data;
    int i;
    for (i=0;i<cnt;i++){
        crc=crc_update(crc,*ptr);
        ptr++;
    }
    return crc;
}
