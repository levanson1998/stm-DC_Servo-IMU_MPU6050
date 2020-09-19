/*
 * IMU_MPU6050.c
 *
 *  Created on: Sep 17, 2020
 *      Author: son
 */


#include "math.h"
#include <stdint.h>

#include "IMU_MPU6050.h"
#include "i2c.h"
#include "gpio.h"


void MPU6050_INIT(){

		// reset mpu6050
	TxBuffer[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, TxBuffer, 1, 1000);
	HAL_Delay(200);

	// tat sleep mode, tat cycle mode, temp_dis = 8MHZ
	TxBuffer[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 1, TxBuffer, 1, 1000);
	HAL_Delay(200);

	// disabled DLPF
	TxBuffer[0] = 0x06;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 1, TxBuffer, 1, 1000);
	HAL_Delay(200);

	// full scale range mode 3 +-2000do/s
	TxBuffer[0] = 0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, TxBuffer, 1, 1000);
	HAL_Delay(200);

	// full scale range mode 1 +-8g
	TxBuffer[0] = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, TxBuffer, 1, 1000);
	HAL_Delay(200);

	// cho phep ngat INT
	TxBuffer[0] = 0x19;
	HAL_I2C_Mem_Write(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 1, TxBuffer, 1, 1000);
	HAL_Delay(200);

	// test MPU6050
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, RxBuffer, 1, 1000);
	HAL_Delay(300);
	if (RxBuffer[0] == 0x68)
	{
		// neu MPU6050 hoat dong binh thuong thi den sang
		HAL_GPIO_WritePin(GPIOD, LED_GRE_Pin, GPIO_PIN_SET);
		HAL_Delay(2000);
	}
	else
	{
		// neu MPU6050 khong hoat dong thi den nhay 5 lan
		uint8_t j;
		for(j=0;j<=11;j++){
			HAL_GPIO_TogglePin(GPIOD, LED_GRE_Pin);
			HAL_Delay(100);
		}
	}
}
