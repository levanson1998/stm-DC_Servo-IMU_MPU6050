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


float max_val[5]={-32000, -32000, -32000, -32000, -32000};
float min_val[5]={32000, 32000, 32000, 32000, 32000};
int16_t yaw_round=0;

void MPU9250_INIT(){
	// reset mpu9250
	TxBuffer9250[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
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

	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Disable I2C master
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_MST_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Reset FIFO and DMP
	TxBuffer9250[0] = 0x0C;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Configure MPU9250 gyro and accelerometer for bias calculation
	// Set low-pass filter to 188 Hz
	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set sample rate to 1 kHz
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set gyro full-scale to 1000 degrees per second, maximum sensitivity
	TxBuffer9250[0] = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, GYRO_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set accelerometer full-scale to 2 g, maximum sensitivity
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, ACCEL_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Enable FIFO
	TxBuffer9250[0] = 0x40;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	TxBuffer9250[0] = 0x78;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, FIFO_EN, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);
	/*
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x22;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_PIN_CFG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// reset mpu9250
	TxBuffer9250[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	//set clk=400khz
	TxBuffer9250[0] = 0x0D;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_MST_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	// tat sleep mode, tat cycle mode, temp1_dis = 8MHZ
	// Enable AUX
	TxBuffer9250[0] = 0x20;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, USER_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = AK8963_ADDRESS;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_ADDR, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = AK8963_CNTL;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_REG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_DO, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x81;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = AK8963_ADDRESS;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_ADDR, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x12;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_DO, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x81;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_CTRL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);
	// -----------

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



	// disabled DLPF
	TxBuffer9250[0] = 0x06;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU6050_RA_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// full scale range mode 3 +-2000do/s
	TxBuffer9250[0] = 0x18;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU6050_RA_GYRO_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// full scale range mode 1 +-8g
	TxBuffer9250[0] = 0x10;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);


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

	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = AK8963_ADDRESS;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_ADDR, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = AK8963_CNTL;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_REG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_DO, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = AK8963_CNTL;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_REG, 1, TxBuffer9250, 1, 1000);
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

	TxBuffer9250[0] = 0xFF;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);


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
*/
}

void MPU9250_Reset(){
	TxBuffer9250[0] = 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);
}

void initAK8963(){
	// Power down magnetometer
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Enter Fuse ROM access mode
	TxBuffer9250[0] = 0x0F;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ASAX, I2C_MEMADD_SIZE_8BIT, &RxBuffer9250[0], 3, 1000);

	// Power down magnetometer
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Set magnetometer data resolution and sample ODR
	TxBuffer9250[0] = 0x16;
	HAL_I2C_Mem_Write(&hi2c1, AK8963_ADDRESS, AK8963_CNTL, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// -----------------------
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ASAX, I2C_MEMADD_SIZE_8BIT, &RxBuffer9250[2], 3, 1000);

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

void initMPU9250(){
	 // Initialize MPU9250 device
	 // wake up device
	TxBuffer9250[0] = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// get stable time source
	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, PWR_MGMT_1, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x03;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, CONFIG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// Use a 200 Hz rate; the same rate set in CONFIG above
	TxBuffer9250[0] = 0x04;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, SMPLRT_DIV, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x22;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_PIN_CFG, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	TxBuffer9250[0] = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, INT_ENABLE, 1, TxBuffer9250, 1, 1000);
	HAL_Delay(200);

	// -----------------------
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
}

/*
 * value return in struct:
	struct data{
		float temp1;		Nhiet do
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
struct data_mpu9250 ReadMPU9250(){
	struct data_mpu9250 mpu;
	int8_t i;
	int16_t DataBuffer16[10];
	IMU9250_READ_DMA();

	for(i=0;i<6;i++)
	{
		DataBuffer16[i] = (int16_t)(((uint16_t)DataBuffer9250[2*i]<<8) | (uint16_t)DataBuffer9250[2*i+1]);
		DataBuffer16_test[i]=DataBuffer16[i];
	}

	for(i=6;i<9;i++)
	{
		DataBuffer16[i] = (int16_t)(((uint16_t)DataBuffer9250[2*i+1]<<8) | (uint16_t)DataBuffer9250[2*i]);
		DataBuffer16_test[i]=DataBuffer16[i];
	}

	accel_x_temp1 = DataBuffer16[0];
	accel_y_temp1 = DataBuffer16[1];
	accel_z_temp1 = DataBuffer16[2];

	mpu.accel_x = DataBuffer16[0];
	mpu.accel_y = DataBuffer16[1];
	mpu.accel_z = DataBuffer16[2];

/*
//	mpu.accel_x = roundf(((atan2((double)accel_y_temp1,(double)accel_z_temp1)+M_PI)*RA_TO_DEC*1000.0f));
	mpu.accel_x = roundf((float)((atan2((double)accel_y_temp1,(double)accel_z_temp1))*RA_TO_DEC)*1000);
	mpu.accel_y = roundf((float)((atan2((double)accel_x_temp1,(double)accel_z_temp1))*RA_TO_DEC)*1000);
	mpu.accel_z = roundf((float)((atan2((double)accel_x_temp1,(double)accel_y_temp1))*RA_TO_DEC)*1000);
*/

//	mpu.accel_x = atan((accel_y_temp1)/sqrt(pow(accel_x_temp1,2)+pow(accel_z_temp11,2)))*RA_TO_DEC;
//	mpu.accel_x = atan(gyroAngleX);

//	mpu.temp1 = (float)DataBuffer16[3];

	gyro_x_temp1 = DataBuffer16[3];
	gyro_y_temp1 = DataBuffer16[4];
	gyro_z_temp1 = DataBuffer16[5];

	mpu.gyro_x = DataBuffer16[3];
	mpu.gyro_y = DataBuffer16[4];
	mpu.gyro_z = DataBuffer16[5]/32.768F;

/*
	mpu.gyro_x = roundf((float)gyro_x_temp1*1000.0f/131.0F);
	mpu.gyro_y = roundf((float)gyro_y_temp1*1000.0f/131.0F);
	mpu.gyro_z = roundf((float)gyro_z_temp1*1000.0f/131.0F);
*/

	mpu.mag_x = DataBuffer16[6];
	mpu.mag_y = DataBuffer16[7];
	mpu.mag_z = DataBuffer16[8];

	getMaxMinValue(mpu.mag_x, 0);
	getMaxMinValue(mpu.mag_y, 1);
	getMaxMinValue(mpu.mag_z, 2);

/*
	mpu.mag_x=mpu.mag_x-(max_val[0]+min_val[0])/2;
	mpu.mag_y=mpu.mag_y-(max_val[1]+min_val[1])/2;
	mpu.mag_z=mpu.mag_z-(max_val[2]+min_val[2])/2;
*/
	mag_x_temp1 = mpu.mag_x;
	mag_y_temp1 = mpu.mag_y;
	mag_z_temp1 = mpu.mag_z;


	mpu.mag_x=mpu.mag_x+12.0F;
	mpu.mag_y=mpu.mag_y-280.0F;
	mpu.mag_z=mpu.mag_z+165.0F;


	pre_dt = current_dt;
	current_dt = HAL_GetTick();
	dt_now = (current_dt-pre_dt)/1000.0F;
	dt_led += dt_now;


	if (dt_led > 3.0){
		origin_mag_x = mpu.mag_x;
		origin_mag_y = mpu.mag_y;
	}

//	mag_norm=sqrt(pow(mpu.mag_x, 2)+pow(mpu.mag_y, 2)+pow(mpu.mag_z, 2));
	mag_norm=sqrt(mpu.mag_x*mpu.mag_x+mpu.mag_y*mpu.mag_y+mpu.mag_z*mpu.mag_z);
	mpu.mag_x=mpu.mag_x/mag_norm;
	mpu.mag_y=mpu.mag_y/mag_norm;
	mpu.mag_z=mpu.mag_z/mag_norm;

//	mpu9250_test[0] = mpu.accel_x/1000.0f;
//	mpu9250_test[1] = mpu.accel_y/1000.0f;
//	mpu9250_test[2] = mpu.accel_z/1000.0f;
//
//	mpu9250_test[3] = mpu.gyro_x/1000.0f;
//	mpu9250_test[4] = mpu.gyro_y/1000.0f;
//	mpu9250_test[5] = mpu.gyro_z/1000.0f;

//	mpu.accel_x = roundf((float)((atan2((double)accel_y_temp1,(double)accel_z_temp1))*RA_TO_DEC)*1000);
	mpu.roll=atan2(mpu.accel_y, sqrt(pow(mpu.accel_x, 2)+pow(mpu.accel_z, 2)));
	mpu.pitch=atan2(-mpu.accel_x, sqrt(pow(mpu.accel_y, 2)+pow(mpu.accel_z, 2)));
	m_x=mpu.mag_x*cos(mpu.pitch)+mpu.mag_z*sin(mpu.pitch);
	m_y=mpu.mag_x*sin(mpu.roll)*sin(mpu.pitch)+mpu.mag_y*cos(mpu.roll)-mpu.mag_z*sin(mpu.roll)*cos(mpu.pitch);

	yaw_cur=atan2(m_x, m_y);

	if(yaw_cur*pre_yaw<-M_PI_2*M_PI_2){
		if(yaw_cur>0){
			yaw_round-=1;
		}
		else{
			yaw_round+=1;
		}
	}
	pre_yaw=yaw_cur;

	mpu.yaw=yaw_round*M_PI*2+yaw_cur;

	getMaxMinValue(m_x, 3);
	getMaxMinValue(m_y, 4);

	mpu9250_test[5]=mpu.roll*RA_TO_DEC;
	mpu9250_test[6]=mpu.pitch*RA_TO_DEC;
	mpu9250_test[7] = mpu.yaw*RA_TO_DEC;

//	delta_yaw = (-pre_yaw+mpu.yaw);
//	mpu9250_test[5] = mpu.gyro_z*dt_now;
//	mpu9250_test[6] = delta_yaw;
//	mpu9250_test[7] = mpu.yaw*RA_TO_DEC;
	mpu.yaw = IMU_Kalman(-mpu.gyro_z*DEC2RAD, mpu.yaw, dt_now);

//	mpu9250_test[6] = mpu.roll*RA_TO_DEC;
//	mpu9250_test[7] = mpu.pitch*RA_TO_DEC;
	mpu9250_test[8] = mpu.yaw*RA_TO_DEC;

	return mpu;
}

void getMaxMinValue(float value, int8_t count_val){
	if(value>=max_val[count_val])
		max_val[count_val]=value;
	if(value<=min_val[count_val])
		min_val[count_val]=value;
	med_val[count_val] = (max_val[count_val]+min_val[count_val])/2;
}

float bias = 0.1F, angle=0.1;
float P[2][2]={{0, 0}, {0, 0}};
float K[2] = {0, 0};
int8_t imu_onetime = 0;
float kalman_test[10];

float IMU_Kalman(float newrate, float newangle, float dt){
	float rate, S, y, Q_angle, R_measure, Q_bias;
	Q_angle = 0.001F;
	Q_bias = 0.003F;
	R_measure = 0.03F;

	rate = newrate-bias;
/*
	if (imu_onetime == 0){
		angle = newangle;
		imu_onetime = 1;
	}
*/
//	bias = roundf(bias*100000.0f);
	angle += dt*rate;
//	angle += 0.1F;
	kalman_test[0] = (newangle-kalman_test[2])/dt;
	kalman_test[1] = newrate;
	kalman_test[2] = newangle;

	P[0][0] += dt*(dt*P[1][1]-P[0][1]-P[1][0]+Q_angle);
	P[0][1] -= dt*P[1][1];
	P[1][0] -= dt*P[1][1];
	P[1][1] += Q_bias*dt;

	S = P[0][0] + R_measure;
//	kalman_test[2] = dt;

	K[0] = P[0][0]/S;
	K[1] = P[1][0]/S;

	kalman_test[3] = K[1];

	y = newangle - angle;

	angle += K[0]*y;
	bias += K[1]*y;
	kalman_test[4] = K[0]*y;
//	bias = roundf(bias*100000.0f);
	kalman_test[5] = K[1]*y;
	kalman_test[6] = angle;
	kalman_test[7] = bias;

	P[0][0] -= K[0]*P[0][0];
	P[0][1] -= K[0]*P[0][1];
	P[1][0] -= K[1]*P[0][0];
	P[1][1] -= K[1]*P[0][1];

	return angle;
}

void IMU9250_READ_DMA(){
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &RxBuffer9250[5], 1, 1000);
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST2, I2C_MEMADD_SIZE_8BIT, &RxBuffer9250[6], 1, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[0], 6, 1000);
	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[6], 6, 1000);
	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 7, 1000);

//	IMU_9250_READ_MAG();
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 6, 1);

//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_ST1, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250ST, 1, 1);
//	if (DataBuffer9250ST & 0x01){
//		HAL_GPIO_WritePin(GPIOD, LED_RED_Pin, GPIO_PIN_SET);
//		HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 6, 1);
//	}
}

void IMU_9250_READ_MAG(){
//	HAL_I2C_Mem_Read(&hi2c1, AK8963_ADDRESS, AK8963_XOUT_L, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 6, 1);

	TxBuffer9250[0] = 0x0C | 0x80;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_ADDR, 1, &TxBuffer9250[0], 1, 1000);

	TxBuffer9250[0] = AK8963_XOUT_L;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_REG, 1, &TxBuffer9250[0], 1, 1000);

	TxBuffer9250[0] = 0x87;
	HAL_I2C_Mem_Write(&hi2c1, MPU9250_ADDRESS, I2C_SLV0_CTRL, 1, &TxBuffer9250[0], 1, 1000);

	HAL_I2C_Mem_Read(&hi2c1, MPU9250_ADDRESS, EXT_SENS_DATA_00, I2C_MEMADD_SIZE_8BIT, &DataBuffer9250[12], 7, 1000);
}


