#ifndef CONFIG_H_
#define CONFIG_H_


//#define QUAD_FORMATION_X
//
//#define CONFIG_BLOCK_ADDRESS    (2048 * (64-1))
//#define MCU_ID_ADDRESS          0x1FFF7A10
//#define MCU_FLASH_SIZE_ADDRESS  0x1FFF7A22
//#define FREERTOS_HEAP_SIZE      30000
//#define FREERTOS_MIN_STACK_SIZE 150       // M4-FPU register setup is bigger so stack needs to be bigger
//#define FREERTOS_MCU_CLOCK_HZ   168000000


#define ATT_EST_TASK_PRI 3
#define ATT_CTRL_TASK_PRI 3
#define MANAGER_TASK_PRI 2
#define MOTORMIX_TASK_PRI 4
// Task priorities. Higher number higher priority
#define MPU_TASK_PRI 5
#define HMC_TASK_PRI 4
#define SENSORS_TASK_PRI 5
#define STABILIZER_TASK_PRI 4
#define SYSTEM_TASK_PRI 2
#define CMD_TASK_PRI 2
#define XBEE_TX_TASK_PRI 2
#define XBEE_RX_TASK_PRI 2
#define SBUS_TASK_PRI 2
#define ADC_TASK_PRI 1
#define LED_TASK_PRI 1
#define EEPROM_TASK_PRI 1

//Task stack sizes

#define ATT_EST_TASK_STACKSIZE (3 * configMINIMAL_STACK_SIZE)
#define ATT_CTRL_TASK_STACKSIZE (3 * configMINIMAL_STACK_SIZE)
#define MANAGER_TASK_STACKSIZE (configMINIMAL_STACK_SIZE)
#define MOTORMIX_TASK_STACKSIZE (configMINIMAL_STACK_SIZE)

#define ATT_EST_TASK_PERIOD_MS 1
#define ATT_EST_TASK_PERIOD_S 0.001f
#define ATT_CTRL_TASK_PERIOD_MS 2
#define ATT_CTRL_TASK_PERIOD_S 0.002f


#define SYSTEM_TASK_STACKSIZE         (2* configMINIMAL_STACK_SIZE)
#define CMD_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define ADC_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define LED_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define PM_TASK_STACKSIZE             configMINIMAL_STACK_SIZE
#define CRTP_TX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define CRTP_RX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define CRTP_RXTX_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define LOG_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define MEM_TASK_STACKSIZE            configMINIMAL_STACK_SIZE
#define PARAM_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define SENSORS_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define STABILIZER_TASK_STACKSIZE     (3 * configMINIMAL_STACK_SIZE)
#define NRF24LINK_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define ESKYLINK_TASK_STACKSIZE       configMINIMAL_STACK_SIZE
#define SYSLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define USBLINK_TASK_STACKSIZE        configMINIMAL_STACK_SIZE
#define PROXIMITY_TASK_STACKSIZE      configMINIMAL_STACK_SIZE
#define EXTRX_TASK_STACKSIZE          configMINIMAL_STACK_SIZE
#define UART_RX_TASK_STACKSIZE        configMINIMAL_STACK_SIZE

#define M2T(X) ((unsigned int)((X)*(configTICK_RATE_HZ/1000.0)))
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))

#endif /* CONFIG_H_ */
