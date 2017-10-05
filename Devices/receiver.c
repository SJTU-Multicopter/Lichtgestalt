#include "receiver.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../MessageTypes/type_methods.h"
#include "../config/config.h"
#include "../Commons/platform.h"
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef htim8;
#define SBUS_BUF_SIZE 25
unsigned char sbus_buffer[SBUS_BUF_SIZE];
static xSemaphoreHandle sbusReceived;
static xQueueHandle rc_q;
static rc_t rc;
static unsigned char byte_cnt = 0;
static void vSbusTask( void *pvParameters ) ;
void receiver_init(void)
{
	rc.channels[0] = 0;
	rc.channels[1] = 0;
	rc.channels[2] = -1024;
	rc.channels[3] = 0;
	#if CMD_SBUS
	sbus_init();
#elif CMD_PPM
	ppm_init();
#elif CMD_XBEE
#endif
	
}
void sbus_init(void)
{
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
	vSemaphoreCreateBinary( sbusReceived );
	rc_q = xQueueCreate(1, sizeof(rc_t));
	xTaskCreate( vSbusTask, "sbus", configMINIMAL_STACK_SIZE, NULL, SBUS_TASK_PRI, NULL ); 
	HAL_UART_Receive_IT(&huart5, sbus_buffer, 1);
}
void ppm_init(void)
{
//	__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_CC3);
	rc_q = xQueueCreate(1, sizeof(rc_t));
	HAL_TIM_IC_Start_IT (&htim8, TIM_CHANNEL_3);
}
void vSbusTask( void *pvParameters )
{
	int i;
	for( ;; ){  
		if (pdTRUE == xSemaphoreTake(sbusReceived, portMAX_DELAY)){
			if(sbus_buffer[0] == SBUS_STARTBYTE && sbus_buffer[24] == SBUS_ENDBYTE){
				rc.channels[0]  = ((sbus_buffer[1]    |sbus_buffer[2]<<8)                 & 0x07FF);
				rc.channels[1]  = ((sbus_buffer[2]>>3 |sbus_buffer[3]<<5)                 & 0x07FF);
				rc.channels[2]  = ((sbus_buffer[3]>>6 |sbus_buffer[4]<<2 |sbus_buffer[5]<<10)  & 0x07FF);
				rc.channels[3]  = ((sbus_buffer[5]>>1 |sbus_buffer[6]<<7)                 & 0x07FF);
				rc.channels[4]  = ((sbus_buffer[6]>>4 |sbus_buffer[7]<<4)                 & 0x07FF);
				rc.channels[5]  = ((sbus_buffer[7]>>7 |sbus_buffer[8]<<1 |sbus_buffer[9]<<9)   & 0x07FF);
				rc.channels[6]  = ((sbus_buffer[9]>>2 |sbus_buffer[10]<<6)                & 0x07FF);
				rc.channels[7]  = ((sbus_buffer[10]>>5|sbus_buffer[11]<<3)                & 0x07FF);
				rc.channels[8]  = ((sbus_buffer[12]   |sbus_buffer[13]<<8)                & 0x07FF);
				rc.channels[9]  = ((sbus_buffer[13]>>3|sbus_buffer[14]<<5)                & 0x07FF);
				rc.channels[10] = ((sbus_buffer[14]>>6|sbus_buffer[15]<<2|sbus_buffer[16]<<10) & 0x07FF);
				rc.channels[11] = ((sbus_buffer[16]>>1|sbus_buffer[17]<<7)                & 0x07FF);
				rc.channels[12] = ((sbus_buffer[17]>>4|sbus_buffer[18]<<4)                & 0x07FF);
				rc.channels[13] = ((sbus_buffer[18]>>7|sbus_buffer[19]<<1|sbus_buffer[20]<<9)  & 0x07FF);
				rc.channels[14] = ((sbus_buffer[20]>>2|sbus_buffer[21]<<6)                & 0x07FF);
				rc.channels[15] = ((sbus_buffer[21]>>5|sbus_buffer[22]<<3)                & 0x07FF);
				for(i=0;i<16;i++){
					rc.channels[i] -= 1024;
				}
				rc.timestamp = xTaskGetTickCount ();
				xQueueOverwrite(rc_q, &rc);
				
			}
			byte_cnt=0;
				HAL_UART_Receive_IT(&huart5, sbus_buffer, 1);
		}
	}
}
void ppmCallback(GPIO_PinState state)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	int timePPM=0;
//	static unsigned int l_time=0;
	static unsigned short channel=0;
	if(state){
		timePPM = HAL_TIM_ReadCapturedValue(&htim8, TIM_CHANNEL_3) / 2;//us
		if(timePPM >= 0 && timePPM <= 4000){
			if(channel < 9){
				rc.channels[channel] = 244*(timePPM-1220)/100;
			}
			channel++;
		}
		else{
			channel = 0;
			xQueueOverwriteFromISR(rc_q, &rc, &xHigherPriorityTaskWoken );
			if (xHigherPriorityTaskWoken)
				portYIELD();
		}
	}
	else{
		__HAL_TIM_SET_COUNTER(&htim8, 0);
	}
}

void rcBlockingAcquire(rc_t *rc)
{
	xQueueReceive(rc_q, rc, portMAX_DELAY);
}
void rcAcquire(rc_t *rc)
{
	xQueuePeek(rc_q, rc, 0);
}
void sbus_IDLE(void)
{
	byte_cnt=0;
	HAL_UART_Receive_IT(&huart5, sbus_buffer, 1);
	
}
void sbusCallback(void)
{
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	if(byte_cnt < 24){
		byte_cnt++;
		HAL_UART_Receive_IT(&huart5, sbus_buffer+byte_cnt, 1);
	}else{
		byte_cnt = 0;
		xSemaphoreGiveFromISR(sbusReceived, &xHigherPriorityTaskWoken);
		if (xHigherPriorityTaskWoken)
			portYIELD();
	}
}


