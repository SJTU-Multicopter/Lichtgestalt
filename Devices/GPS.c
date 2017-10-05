#include "GPS.h"
#include "../Modules/commons.h"
#include "../Commons/platform.h"
#include "string.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdlib.h"
#include "../MessageTypes/type_methods.h"
#define GPS_BUFFER_SIZE 256
extern UART_HandleTypeDef huart1;
static gpsRaw_t gps;
int read=0;
unsigned char gps_buffer0[GPS_BUFFER_SIZE];//change bigger?
unsigned char gps_buffer1[GPS_BUFFER_SIZE];
static unsigned int data_len;
static unsigned char buffer_num=0;
static unsigned char buf2read_num=0;
//cleared at timeout, ++ during dma interrupt
static void GPSTask( void *pvParameters );
static xSemaphoreHandle gps_dataReceived;
static xQueueHandle gps_q;
void GPSReceive_IDLE(void)  
{  
  //  uint32_t temp;  
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	if((__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE) != RESET)){   
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);  
		HAL_UART_DMAStop(&huart1);  
       // temp = huart1.hdmarx->Instance->NDTR;  
		data_len =  GPS_BUFFER_SIZE - huart1.hdmarx->Instance->NDTR;             
		buf2read_num = buffer_num;
		xSemaphoreGiveFromISR(gps_dataReceived, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
			portYIELD();
		
		buffer_num = (!buffer_num) & 1;//either 0 or 1			
		if(buffer_num == 0){
			HAL_UART_Receive_DMA(&huart1,gps_buffer0,GPS_BUFFER_SIZE); 
		}
		else{
			HAL_UART_Receive_DMA(&huart1,gps_buffer1,GPS_BUFFER_SIZE); 
		}		
	}  
}  
void GPSInit(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	vSemaphoreCreateBinary( gps_dataReceived );
	gps_q = xQueueCreate(1,sizeof(gpsRaw_t));
	xTaskCreate( GPSTask, "GPS", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+3, NULL );  
	if(buffer_num == 0){
		HAL_UART_Receive_DMA(&huart1,gps_buffer0,GPS_BUFFER_SIZE); 
	}
	else{
		HAL_UART_Receive_DMA(&huart1,gps_buffer1,GPS_BUFFER_SIZE); 
	}		
}

float char_to_float(char *a)//transfer ascII code(with)
{
	float value=0.0;
	int i=0;
	int j=1;
	int sign=0;
	float decade=1;
	int point=-1;//if there is no decimal point, point==-1
	if(a[0]=='-')
		sign=1;	
	for(i=0;i<14;i++){ //find decimal position
	 	if(a[i+sign]=='.'){
		   point=i;
		   break;
		}
	}
	for(j=1;j<(point);j++){//find the decade for max bit
		decade=decade*10;
	}
	for(i=0;i<point;i++){		
	   value+=(a[i+sign]-48)*decade;
	   decade=decade*0.1f;
	}//now we have got the int part
	decade=0.1;

	for(i=point+1;i<14;i++){
	 	if(a[i+sign]!=0){
			value+=(a[i+sign]-48)*decade;
			decade=decade*0.1f;
		}
		else{
			break;
		}
	}
	if(sign)
		value=-value;
	return value;
}


void GPS_data_decode(char *gps_decode)
{
	double gps_lat_raw=0;
	double gps_lon_raw=0;
	uint8_t search_ptr=0;
	char *start_p = gps_decode;
	char *pos_p = NULL;
	gps.vel_valid = false;

		if(strncmp(gps_decode,"$GNRMC",6)==0)
		{
			pos_p = strchr(start_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			gps.time = atoi(pos_p);
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			if(*pos_p == 'A')
				gps.status = realtimeGPS;
			else
				gps.status = noGPS;
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			gps_lat_raw = strtod(pos_p,NULL);
			gps.lat = ((int64_t)gps_lat_raw/100 + (gps_lat_raw-((int64_t)gps_lat_raw/100)*100)/60)*1e7;
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			if(*pos_p == 'S')
				gps.lat = -gps.lat;
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			gps_lon_raw = strtod(pos_p,NULL);
			gps.lon = ((int64_t)gps_lon_raw/100 + (gps_lon_raw-((int64_t)gps_lon_raw/100)*100)/60)*1e7;//(*pos_p)*1e9+(*pos_p++)*1e8+(*pos_p++)*1e7+((*pos_p++)*1e7+(*pos_p++)*1e6+(*pos_p++)*1e5+(*pos_p++)*1e4+(*pos_p++)*1e3+(*pos_p++)*100 + +(*pos_p++)*10)//;
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			if(*pos_p == 'W')
				gps.lon = -gps.lon;
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			gps.vel = strtod(pos_p,NULL);
			pos_p = strchr(pos_p,',');
			pos_p++;
			if(*pos_p == ',')
			{
				gps.vel_valid = false;
			}
			else
			{
				gps.azm = atof(pos_p);
				if(gps.azm >= 0.0f && gps.azm <= 360.0f)
				{
					gps.vel_valid = true;
				}
				else
				{
					gps.vel_valid = false;
				}
				pos_p = strchr(pos_p,',');
			}
			pos_p++;
			gps.date = atoi(pos_p);	
		}
		pos_p = strchr(pos_p,'$');
		if(strncmp(pos_p,"$GNGGA",6)==0)
		{
			for(search_ptr=0;search_ptr<7;search_ptr++)
			{
				pos_p = strchr(pos_p,',');
				if(pos_p == 0)
				{
					return ;
				}
				pos_p++;
			}
			gps.sat = atoi(pos_p);
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;//shuipingxishijingdu eph
			pos_p = strchr(pos_p,',');
			if(pos_p == 0)
			{
				return ;
			}
			pos_p++;
			gps.alt = atof(pos_p);
			
		}
	
}	

void get_gps_data(void)//read the data we want in gps_buffer[160]
{
	char string[15];//to store ascII of a data between two ","
	int string_offset=0;
	int received_type=0;//the type of the data between two ","
	int frame=0;//GGA or RMC
	int gps_temp_data[2]={0,0};//to store unsigned LAT and LON, and wait for the NSEW letter
	unsigned char gps_in;
	unsigned int i,j=0;

	for(j=0;j<data_len;j++){
		if(buf2read_num == 0){
			gps_in=gps_buffer0[j]; 
		}
		else{
			gps_in=gps_buffer1[j]; 
		}		
		
		if(gps_in=='$'){
			string_offset=0;
			received_type=0;
		}
		else if(gps_in==','){//deal with one string
			string_offset=0;		
			if(received_type==0){
			#if LEA6H_GPS
				if(string[0]=='G'&&string[1]=='P'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;
				else if(string[0]=='G'&&string[1]=='P'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;
				else 
				 	frame=OTH_FRM;
			#elif M8N_GPS
				if(string[0]=='G'&&string[1]=='N'&&string[2]=='G'
				&&string[3]=='G'&&string[4]=='A')
					frame=GPGGA;	
				else if(string[0]=='G'&&string[1]=='N'&&string[2]=='R'
				&&string[3]=='M'&&string[4]=='C')
					frame=GPRMC;
				else 
				 	frame=OTH_FRM;
			#endif
			}
			if(frame==OTH_FRM)
				continue;
			else if(frame==GPGGA){
				switch(received_type){
				case 6:
				//gps status: 0 not positioning, 1 non-diff positioning, 
				//2 diff positioning, 3 invalid PPS, 6 estimating
				{
					int status=(string[0]-48);
					if(status == 0)
						gps.status = noGPS;
					else if (status == 1)
						gps.status = realtimeGPS;
					else if (status == 2)
						gps.status = difGPS;
				}					
				
					break;
				case 7://# of sat in use (00~12)
					gps.sat=(string[0]-48)*10+(string[1]-48);
					break;
				case 9://altitude -9,999.9 ~ 99,999.9
					gps.alt=char_to_float(string);//atof(string);//char_to_float(string);
					break;
				default:
					break;
				}
			}
			else if(frame==GPRMC){
				switch(received_type){
				case 3:
				//latitude ddmm.mmmmm, 0.00001m correspond to 18mm
					gps_temp_data[LAT]=
					(string[0]-48)*100000000
					+(string[1]-48)*10000000
						+((string[2]-48)*10000000
						+(string[3]-48)*1000000
						+(string[5]-48)*100000
						+(string[6]-48)*10000
						+(string[7]-48)*1000
						+(string[8]-48)*100
						+(string[9]-48)*10)/6;//10^7deg
					break;
				case 4://latitude NS
					if(string[0]=='N'){
						gps.lat=gps_temp_data[LAT];
					}
					else if(string[0]=='S'){
						gps.lat=-gps_temp_data[LAT];
					}
					break;
				case 5://longitude dddmm.mmmmm
					gps_temp_data[LON]=
					(string[0]-48)*1000000000
					+(string[1]-48)*100000000
					+(string[2]-48)*10000000
						+((string[3]-48)*10000000
						+(string[4]-48)*1000000
						+(string[6]-48)*100000
						+(string[7]-48)*10000
						+(string[8]-48)*1000
						+(string[9]-48)*100
						+(string[10]-48)*10)/6;
					break;
				case 6://longitude EW			
					if(string[0]=='E'){
						gps.lon=gps_temp_data[LON];
					}
					else if(string[0]=='W'){
						gps.lon=-gps_temp_data[LON];
					}
					break;
				case 7://velocity Knots
					gps.vel=char_to_float(string)*514.4f;//atof(string)*514.4f;//char_to_float(string)*514.4f;//1 knot = 0.5144m/s = 514.4mm/s
					break;
				case 8://azimuth deg			
					gps.azm=char_to_float(string) * DEG2RAD;;//atof(string) * DEG2RAD;//char_to_float(string) * DEG2RAD;
					break;
				default:
					break;
				}
			}
			received_type++;
			string_offset=0;
			for(i=0;i<15;i++){
				string[i]='0';
			}
		}
		else if(gps_in=='\r'||gps_in=='\n'){
		}
		else if(gps_in=='*'){
		}
		else{
			if(string_offset<15)
				string[string_offset]=gps_in;
			string_offset++;
		}
	}
}

void GPSTask( void *pvParameters )
{
//	TickType_t xLastWakeTime;
//	const TickType_t timeIncreament = 1000;
//	xLastWakeTime = xTaskGetTickCount();
	
	for( ;; )  
	{  
		if (pdTRUE == xSemaphoreTake(gps_dataReceived, portMAX_DELAY)){
		/*	if(buf2read_num == 0){
				GPS_data_decode((char*)gps_buffer0); 
				read++;
			}
			else{
				GPS_data_decode((char*)gps_buffer1); 
				read++;
			}
			data_len = data_len;
			data2send[0] = read;
			data2send[1] = buf2read_num;
		*/	
			get_gps_data();
			gps.timestamp = xTaskGetTickCount ();
			xQueueOverwrite(gps_q,&gps);
		}
//		vTaskDelayUntil( &xLastWakeTime, timeIncreament );  
	}  
}
void gps_acquire(gpsRaw_t *gps_data)
{
//	BaseType_t pdres=pdFALSE;
//	pdres = 
	xQueuePeek(gps_q,gps_data,100);
//	return pdres;
}
