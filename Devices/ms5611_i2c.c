#include "ms5611_i2c.h"
#include "../Modules/sensors_task.h"
#include "stm32f4xx_hal.h"
//baro shares the I2C with mag
#define MS5611_ADDRESS (0x77<<1)
#define MS5611_RESET 0x1E
#define MS5611_ROM_C1 0xA2
#define MS5611_ROM_C2 0xA4
#define MS5611_ROM_C3 0xA6
#define MS5611_ROM_C4 0xA8
#define MS5611_ROM_C5 0xAA
#define MS5611_ROM_C6 0xAC
#define MS5611_CONV_D1 0x48
#define MS5611_CONV_D2 0x58//temp
#define MS5611_ADC_READ 0x00
#define STATUS_IDLE 0
#define STATUS_TEMP_CONV 1
#define STATUS_PRES_CONV 2
#define STATUS_TEMP_READ 3
#define STATUS_PRES_READ 4
extern I2C_HandleTypeDef hi2c1;
unsigned short setup,C1,C2,C3,C4,C5,C6=0;
unsigned int D2, D1;
unsigned char p_D1 = MS5611_CONV_D1;
unsigned char p_D2 = MS5611_CONV_D2;
unsigned char baro_status = STATUS_IDLE;
unsigned char res[3];
//unsigned int res_D2;
long long OFF,SENS;
void ms5611_init(void)
{
	uint8_t cmd_reset = MS5611_RESET;
	uint8_t c1_array[2]={0,0};
	uint8_t c2_array[2]={0,0};
	uint8_t c3_array[2]={0,0};
	uint8_t c4_array[2]={0,0};
	uint8_t c5_array[2]={0,0};
	uint8_t c6_array[2]={0,0};
	HAL_StatusTypeDef reset_ret = HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &cmd_reset, 1, 5);
	HAL_Delay(100);
	HAL_StatusTypeDef c1_ret = HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C1, I2C_MEMADD_SIZE_8BIT, c1_array, 2, 5);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C2, I2C_MEMADD_SIZE_8BIT, c2_array, 2, 5);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C3, I2C_MEMADD_SIZE_8BIT, c3_array, 2, 5);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C4, I2C_MEMADD_SIZE_8BIT, c4_array, 2, 5);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C5, I2C_MEMADD_SIZE_8BIT, c5_array, 2, 5);
	HAL_Delay(10);
	HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ROM_C6, I2C_MEMADD_SIZE_8BIT, c6_array, 2, 5);
	HAL_Delay(10);
	C1 = (c1_array[0]<<8)+c1_array[1];
	C2 = (c2_array[0]<<8)+c2_array[1];
	C3 = (c3_array[0]<<8)+c3_array[1];
	C4 = (c4_array[0]<<8)+c4_array[1];
	C5 = (c5_array[0]<<8)+c5_array[1];
	C6 = (c6_array[0]<<8)+c6_array[1];
	
}

void ms5611_temp_start(void)
{
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_IT(&hi2c1, MS5611_ADDRESS, &p_D2, 1);
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5611_ADDRESS, &p_D2, 1);
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &p_D2, 1, 15);
	baro_status = STATUS_TEMP_CONV;
}
void ms5611_pres_start(void)
{
	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_IT(&hi2c1, MS5611_ADDRESS, &p_D1, 1);
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit_DMA(&hi2c1, MS5611_ADDRESS, &p_D1, 1);
//	HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDRESS, &p_D1, 1, 15);
	baro_status = STATUS_PRES_CONV;
}
void ms5611_read_trigger(void)
{
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_DMA(&hi2c1, MS5611_ADDRESS, MS5611_ADC_READ, I2C_MEMADD_SIZE_8BIT, res, 3);
//	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c1, MS5611_ADDRESS, MS5611_ADC_READ, I2C_MEMADD_SIZE_8BIT, res, 3, 15);
	if(baro_status == STATUS_TEMP_CONV)
		baro_status = STATUS_TEMP_READ;
	else if (baro_status == STATUS_PRES_CONV)
		baro_status = STATUS_PRES_READ;
}
float get_altitude(float refPressure, float pressure)
{
	float tmp_float, altitude;
	tmp_float = (pressure / refPressure);
	tmp_float = pow(tmp_float, 0.190295);
	altitude = 44330 * (1.0f - tmp_float);// unit m
	return altitude;
}
void baroProcess(baro_t* baro)
{
	if(baro_status == STATUS_TEMP_READ){
		long TEMP;
		long long OFF2,SENS2,T2,AUX,AUX2,dT;
		D2 = (res[0]<<16)+(res[1]<<8)+res[2];
		dT = (long long)D2 - ((long long)C5 << 8);
		TEMP = 2000 + (((long long)C6 * dT) >> 23);
		OFF = ((long long)C2 << 16) + (((long long)C4 * dT) >> 7);
		SENS = ((long long)C1 << 15) + (((long long)C3 * dT) >> 8);
		if(TEMP < 2000){
			T2 = (dT * dT) >> 31;
			AUX = (TEMP - 2000) * (TEMP - 2000);
			OFF2 = 5 * AUX >> 1;
			SENS2 = 5 * AUX >> 2;
			if(TEMP < -1500){
				AUX2 = (TEMP + 1500) * (TEMP + 1500);
				OFF2 += 7 * AUX2;
				SENS2 += 11 * AUX2 / 2;
			}
		}
		else{
			T2 = 0;
			OFF2 = 0;
			SENS2 = 0;
		}
		TEMP -= T2;
		OFF -= OFF2;
		SENS -= SENS2;	
		baro->temp = TEMP;
	}
	else if (baro_status == STATUS_PRES_READ){
		D1 = (res[0]<<16)+(res[1]<<8)+res[2];
		baro->pressure = ((D1 * SENS >> 21) - OFF) >> 15;
		if(baro->refPressure < 130000 && baro->refPressure > 80000){
			baro->alt = get_altitude(baro->refPressure, baro->pressure);
		}
	}
	baro_status = STATUS_IDLE;
}


