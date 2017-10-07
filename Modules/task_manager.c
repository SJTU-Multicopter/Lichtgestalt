#include "../MessageTypes/type_methods.h"

#include "cmsis_os.h"
#include "../config/config.h"

#include "commander.h"
#include "sensors_task.h"
#include "attitude_estimator.h"
#include "attitude_controller.h"
#include "motor_mixer.h"
#include "sensors_calibration.h"
#include "task_manager.h"
#include "position_controller.h"
#include "position_estimator.h"

#include "../Commons/platform.h"
#include "../MessageTypes/type_methods.h"
#include "../config/config.h"

#include "../Devices/data_link.h"
#include "../Devices/battery.h"
#include "../Devices/led.h"
#include "../Devices/rom.h"
#include "../Devices/motor_pwm.h"
#include "../Devices/mpu6000_spi.h"
#include "../Devices/hmc5883l_i2c.h"
#include "../Devices/receiver.h"
#include "../Devices/GPS.h"
#include "../Devices/ms5611_i2c.h"
mode_t g_mode;
statusLock_t g_statusLock;
statusFlight_t g_statusFlight;
statusLink_t g_statusLink;
statusGS_t g_statusGS;

//static battery_t _bat;
static void managerTask(void* param);
//static void managerInitTask(void* param);
void taskManagerInit(void)
{
	if(check_butt()){
		g_mode = modeCal;
	}
	else{
		#if INDOOR
		g_mode = modeMan;
		#elif OUTDOOR
		g_mode = modePos;
		#endif
	}
	g_statusFlight = statusInitiating;
	g_statusLock = motorLocked;
	motor_init();
	motor_cut();
	mpu6000_cfg();
	ms5611_init();
	hmc5883l_cfg();
	
	mpu_fast_init();
	hmc_fast_init();
	receiver_init();
	led_init();
	data_link_init();
	data_send_start();
	GPSInit();
	eeprom_init();
	battery_init();
	sensorsTaskInit();
	
//	motor_mixer_init();
	
	if(g_mode == modeMan ||  g_mode == modePos){
		commanderInit();
		attitude_init();
		
		position_controller_queue_init();
		position_estimation_queue_init();
	}
	else if(g_mode == modeCal){
		calibration_manager_init();
		data_send_start();
	}
}
void attitude_initialized_callback(att_t * att)
{
//	attsp_reset(att);
	g_statusFlight = statusLanded;
	start_manager();
	attitude_estimator_start();
	acc_filter_start();
	position_estimation_start();
	position_controller_init(att);
	attitude_controller_init();
	
	motor_mixer_init();
	
}
uint32_t self_check(void)
{
	attsp_t attsp;
	attspAcquire(&attsp);
	if(attsp.thrust<0){
		return 0;
	}
	else
		return 1;
}
void start_manager(void)
{
	xTaskCreate(managerTask, "managerTask", MANAGER_TASK_STACKSIZE, NULL, MANAGER_TASK_PRI, NULL);
}
static void managerTask(void* param)
{
	TickType_t xLastWakeTime;
	unsigned int tick = 0, but_cnt = 0;
	const TickType_t timeIncreament = 20;
	
	
	
	xLastWakeTime = xTaskGetTickCount();
	for( ;; )  
	{  
		vTaskDelayUntil( &xLastWakeTime, timeIncreament ); 
		tick ++;
		if(check_butt())
			but_cnt++;
		else
			but_cnt = 0;
		if(but_cnt == 25 && g_mode != modeCal){
			if(g_statusLock == motorLocked)
				g_statusLock = motorUnlocking;
			else if(g_statusLock == motorUnlocked||g_statusLock == motorUnlocking)
				g_statusLock = motorLocked;
		}
		if(g_statusLock == motorUnlocking){
			if(self_check() == 0)
				g_statusLock = motorUnlocked;
			else
				g_statusLock = motorLocked;
		}
		if(g_statusLock == motorUnlocked)
			setLed(0, 200, 500);
		else
			setLed(0, 500, 1000);		
	}
}
