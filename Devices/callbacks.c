#include "callbacks.h"
#include "receiver.h"
#include "stm32f4xx_hal.h"
#include "../Modules/sensors_task.h"
#include "rom.h"
#include "mpu6000_spi.h"
extern unsigned char curr_i2c_dev;
extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;
extern UART_HandleTypeDef huart5;
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim8;
void HAL_FLASH_EndOfOperationCallback(uint32_t ReturnValue)
{
	
	
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == hi2c1.Instance){
		if(curr_i2c_dev == I2C_DEV_HMC5883)
	//	if(hi2c->Devaddress == (0x1E<<1))
			hmc5883lCallback();
		else if(curr_i2c_dev == I2C_DEV_MS5611)
	//	else if(hi2c->Devaddress == (0x77<<1))
			ms5611Callback();
	}
	if(hi2c->Instance == hi2c2.Instance)
		eeprom_readCallback();
	
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
//	if(hi2c->Instance == hi2c2.Instance)
//		eeprom_writeCallback();
	
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == hspi1.Instance){
//	static int j=0;
	
		DSEL();
		mpu6000Callback();

//	if(j%2 ==0)
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);  
//	else
//		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);  
//	j++;
//	if(j==20)
//		j=0;
//	for (i=0; i<6; i++){	
//		a_g_queue[i][j] = a_g_data[i];
//	}
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim8.Instance){
		if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC3) != RESET)
		{
			ppmCallback(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8));
		}
		if(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_CC4) != RESET)
		{
			
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == huart5.Instance){
		sbusCallback();
	}
}
