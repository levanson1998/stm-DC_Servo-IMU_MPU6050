/*
 * IMU_MPU9250.c
 *
 *  Created on: Dec 31, 2020
 *      Author: son
 */


#include "math.h"
#include <stdint.h>
#include <stdio.h>

#include "IMU_MPU9250.h"
#include "i2c.h"
#include "gpio.h"


void MPU9250_INIT(){

	// reset mpu9250
	TxBuffer9250[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	// tat sleep mode, tat cycle mode, temp_dis = 8MHZ
	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_2, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Tat interupt
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// tat FIFO
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);


//	// disabled DLPF
//	TxBuffer9250[0] = 0x06;
//	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU6050_RA_CONFIG, 1, TxBuffer9250, 1, 1000);
//	HAL_Delay(200);

//	// full scale range mode 3 +-2000do/s
//	TxBuffer9250[0] = 0x18;
//	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, TxBuffer9250, 1, 1000);
//	HAL_Delay(200);

//	// full scale range mode 1 +-8g
//	TxBuffer9250[0] = 0x10;
//	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, TxBuffer9250, 1, 1000);
//	HAL_Delay(200);

	// Set low-pass filter to 188 Hz
	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set sample rate to 1 kHz
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set gyro full-scale to 250 degrees per second, maximum sensitivity
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set accelerometer full-scale to 2 g, maximum sensitivity
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	// Enable FIFO
	TxBuffer9250[0] = 0x40;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	TxBuffer9250[0] = 0x78;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// init AK8963
/*
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);
*/

	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x0F;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x06;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

//	TxBuffer9250[0] = 0xFF;
//	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
//	HAL_Delay(200);

	// test MPU6050
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, WHO_AM_I_MPU9250, I2C_MEMADD_SIZE_8BIT, &RxBuffer9250[0], 1, 1000);
	HAL_Delay(100);
	if (RxBuffer9250[0] == 0x71)
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
			HAL_GPIO_TogglePin(GPIOD, LED_RED_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_ORG_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_BLU_Pin);
			HAL_Delay(100);
		}
	}

	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &RxBuffer9250[1], 1, 1000);
	HAL_Delay(100);

	if (RxBuffer9250[1] == 0x48)
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
			HAL_GPIO_TogglePin(GPIOD, LED_RED_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_ORG_Pin);
			HAL_GPIO_TogglePin(GPIOD, LED_BLU_Pin);
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

 * GPIO: PB9 -> IMU_SDA
 * 		 PB8 -> IMU_SCL
*/
/*
struct data_imu ReadMPU(){
	struct data_imu mpu;
	int8_t i;
	int16_t DataBuffer16[7];
	IMU_READ_DMA();

	for(i=0;i<7;i++)
	{
		DataBuffer16[i] = (int16_t)(((uint16_t)DataBuffer[2*i]<<8) | DataBuffer[2*i + 1]);
	}

	accel_x_temp = DataBuffer16[0];
	accel_y_temp = DataBuffer16[1];
	accel_z_temp = DataBuffer16[2];


//	mpu.accel_x = roundf(((atan2((double)accel_y_temp,(double)accel_z_temp)+M_PI)*RA_TO_DEC*1000.0f));
	mpu.accel_x = roundf((float)((atan2((double)accel_y_temp,(double)accel_z_temp))*RA_TO_DEC)*1000);
	mpu.accel_y = roundf((float)((atan2((double)accel_x_temp,(double)accel_z_temp))*RA_TO_DEC)*1000);
	mpu.accel_z = roundf((float)((atan2((double)accel_x_temp,(double)accel_y_temp))*RA_TO_DEC)*1000);


	mpu.accel_x = atan((accel_y_temp)/sqrt(pow(accel_x_temp,2)+pow(accel_z_temp,2)))*RA_TO_DEC;
	mpu.accel_x = atan(gyroAngleX);


	mpu.temp = (float)DataBuffer16[3];

	gyro_x_temp = DataBuffer16[4];
	gyro_y_temp = DataBuffer16[5];
	gyro_z_temp = DataBuffer16[6];

	mpu.gyro_x = roundf((float)gyro_x_temp*1000.0f/131.0F);
	mpu.gyro_y = roundf((float)gyro_y_temp*1000.0f/131.0F);
	mpu.gyro_z = roundf((float)gyro_z_temp*1000.0f/131.0F);

	imu_test[0] = mpu.accel_x/1000.0f;
	imu_test[1] = mpu.accel_y/1000.0f;
	imu_test[2] = mpu.accel_z/1000.0f;

	imu_test[3] = mpu.gyro_x/1000.0f;
	imu_test[4] = mpu.gyro_y/1000.0f;
	imu_test[5] = mpu.gyro_z/1000.0f;

	return mpu;

}

*/

uint8_t DataBuffer9250[18];
uint8_t DataBuffer9250ST;
void IMU9250_READ_DMA(){

	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[0], 6, 1);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[6], 6, 1);
	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 6, 1);

	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250ST, 1, 1);
	if (DataBuffer9250ST & 0x01){
		HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_SET);
		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 6, 1);
	}
}


