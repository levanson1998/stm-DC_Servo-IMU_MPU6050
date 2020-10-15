/*
 * IMU_MPU6050.c
 *
 *  Created on: Sep 17, 2020
 *      Author: son
 */


#include "math.h"
#include <stdint.h>
#include <stdio.h>

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
//		HAL_GPIO_TogglePin(GPIOD, LED_GRE_Pin);
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

/*
 * value return in struct:
	struct data{
		float temp;		Nhiet do
		float gyro_x;	Goc x (gyroscope)
		float gyro_y;	Goc y
		float gyro_z;	Goc z
		float accel_x;	Toc do goc x (accelerometer)
		float accel_y;	Toc do goc y
		float accel_z;	Toc do goc z
	};
*/
struct data ReadMPU(){
	struct data mpu;
	int8_t i;
	int16_t DataBuffer16[7];
	int16_t gyro_x_temp, gyro_y_temp, gyro_z_temp, accel_x_temp, accel_y_temp, accel_z_temp;
	IMU_READ_DMA();

	for(i=0;i<7;i++)
	{
		DataBuffer16[i] = (int16_t)(((uint16_t)DataBuffer[2*i]<<8) | DataBuffer[2*i + 1]);
	}

	accel_x_temp = DataBuffer16[0];
	accel_y_temp = DataBuffer16[1];
	accel_z_temp = DataBuffer16[2];


	mpu.accel_x = roundf(((atan2((double)accel_y_temp,(double)accel_z_temp)+M_PI)*RA_TO_DEC*1000.0f));
	mpu.accel_y = roundf((float)((atan2((double)accel_x_temp,(double)accel_z_temp))*RA_TO_DEC));
	mpu.accel_z = roundf((float)((atan2((double)accel_x_temp,(double)accel_y_temp))*RA_TO_DEC));


/*
	mpu.accel_x = atan((accel_y_temp)/sqrt(pow(accel_x_temp,2)+pow(accel_z_temp,2)))*RA_TO_DEC;
	mpu.accel_x = atan(gyroAngleX);
*/

	mpu.temp = (float)DataBuffer16[3];

	gyro_x_temp = DataBuffer16[4];
	gyro_y_temp = DataBuffer16[5];
	gyro_z_temp = DataBuffer16[6];

	mpu.gyro_x = roundf((float)gyro_x_temp*1000.0f/131.0F);
	mpu.gyro_y = roundf((float)gyro_y_temp*1000.0f/131.0F);
	mpu.gyro_z = roundf((float)gyro_z_temp*1000.0f/131.0F);

	return mpu;

}

/*
DataBuffer is output
uint8_t DataBuffer[14];
*/
void IMU_READ_DMA(){
	HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, DataBuffer, 14, 1);
//	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, DataBuffer, 14);
//	HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer[0], 14);
}

