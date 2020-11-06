/*
 * uart2pi.h
 *
 *  Created on: Oct 14, 2020
 *      Author: son
 */

#include "IMU_MPU6050.h"

#ifndef LIB_UART2PI_H_
#define LIB_UART2PI_H_


#endif /* LIB_UART2PI_H_ */

void UartTransmit(int16_t encA, int16_t encB, struct data_imu ss, uint8_t motor_dir);
// void UartTransmit(int16_t encA, int16_t encB, uint32_t accel_x, uint32_t accel_y, uint32_t accel_z, int32_t gyro_x, int32_t gyro_y, int32_t gyro_z, uint8_t motor_dir);
void Dec2Bytes(int16_t encA, int16_t encB, struct data_imu ss, uint8_t motor_dir);
void Byte2Dec();

uint8_t dataTransmit[16];
uint8_t receivebuffer[16];
float _velo[2];
float _motor_dir;
float k[4];
