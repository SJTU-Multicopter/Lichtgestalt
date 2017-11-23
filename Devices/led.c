#include "led.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "../MessageTypes/type_methods.h"
#include "../config/config.h"
typedef struct led_s {
	unsigned int period;
	unsigned int duty;
	unsigned int cnt;
} led_t;//used in queue

#define LED1 PC3
#define LED2 PC14
#define LED3 PB15
#define BUTT PB2

#define LED1_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET)
#define LED1_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET) 

#define LED2_ON()   HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET) 
#define LED2_OFF()  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET)

#define LED3_ON()   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET) 
#define LED3_OFF()  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET)
static led_t led[3];
bool check_butt(void)
{	
	GPIO_PinState state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2);
	if(state == GPIO_PIN_RESET)//pressed down
		return true;
	else
		return false;
}

static void vLedTask(void *pvParameters);
void led_init(void)
{
	int i=0;
	for(i=0;i<3;i++){
		led[i].duty = 0;
		led[i].period = 1000;
		led[i].cnt = 0;
	}
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	xTaskCreate( vLedTask, "LED", LED_TASK_STACKSIZE, NULL, LED_TASK_PRI, NULL );  
}
void but_init(void)
{
	
	
}
void setLed(unsigned char index, unsigned int duty, unsigned int period)
{
	led[index].duty = duty;
	led[index].period = period;
	
}
void vLedTask(void *pvParameters)
{
//	bool butt_state = false;
//	int i;
	TickType_t xLastWakeTime;
//	unsigned int but_cnt=0;
	unsigned int tick = 0;
	const TickType_t timeIncreament = 1;
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
	{  
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
		tick ++;
		
		
		if(led[0].duty == led[0].period)
			LED1_ON();
		else if(led[0].duty == 0)
			LED1_OFF();
		else{
			if(led[0].cnt == led[0].duty){
				LED1_OFF();
			}
			if(led[0].cnt >= led[0].period){
				led[0].cnt = 0;
				LED1_ON();
			}
			else{
				led[0].cnt++;
			}
		}
		if(led[1].duty == led[1].period)
			LED2_ON();
		else if(led[1].duty == 0)
			LED2_OFF();
		else{
			if(led[1].cnt == led[1].duty){
				LED2_OFF();
			}
			if(led[1].cnt >= led[1].period){
				led[1].cnt = 0;
				LED2_ON();
			}
			else{
				led[1].cnt++;
			}
		}
		if(led[2].duty == led[2].period)
			LED3_ON();
		else if(led[2].duty == 0)
			LED3_OFF();
		else{
			if(led[2].cnt == led[2].duty){
				LED3_OFF();
			}
			if(led[2].cnt >= led[2].period){
				led[2].cnt = 0;
				LED3_ON();
			}
			else{
				led[2].cnt++;
			}
		}
	}  
}
