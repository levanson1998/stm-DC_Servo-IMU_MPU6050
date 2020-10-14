/*
 * uart2pi.h
 *
 *  Created on: Oct 14, 2020
 *      Author: son
 */

#ifndef LIB_UART2PI_H_
#define LIB_UART2PI_H_



#endif /* LIB_UART2PI_H_ */

void Dec2Bytes(uint32_t accel, uint32_t gyro, uint16_t encA, uint16_t encB, uint8_t motor_dir);
void Byte2Dec();
void UartTransmit(uint32_t accel, uint32_t gyro, uint16_t encA, uint16_t encB, uint8_t motor_dir);

uint8_t dataTransmit[16];
uint8_t receivebuffer[16];
float k[4];
