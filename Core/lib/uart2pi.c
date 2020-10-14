/*
 * uart2pi.c
 *
 *  Created on: Oct 14, 2020
 *      Author: son
 */


#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include "usart.h"

#include "uart2pi.h"


/*
	Input:
		goc: 3 bytes
		toc do goc: 3 bytes
		encoderA: 2 bytes
		encoderB: 2 bytes
		chieu: 1 byte
	Output:
		van toc trai: 2 byte
		van toc phai: 2 byte
		chieu: 1 byte
		data: 1 byte
*/
void Dec2Bytes(uint32_t accel, uint32_t gyro, uint16_t encA, uint16_t encB, uint8_t motor_dir){
	dataTransmit[0]=(uint8_t)((((uint16_t)encA)|0x00FF)>>8); // 8 bit H
	dataTransmit[1]=(uint8_t)((((uint16_t)encA)|0xFF00)); 	      // 8 bit L

	dataTransmit[2]=(uint8_t)((((uint16_t)encB)|0x00FF)>>8); // 8 bit H
	dataTransmit[3]=(uint8_t)((((uint16_t)encB)|0xFF00)); 	      // 8 bit L

	dataTransmit[4]=(uint8_t)((((uint32_t)accel)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[5]=(uint8_t)((((uint32_t)accel)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[6]=(uint8_t)((((uint32_t)accel)|0xFFFFFF00));		// 8 bit L

	dataTransmit[7]=(uint8_t)((((uint32_t)gyro)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[8]=(uint8_t)((((uint32_t)gyro)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[9]=(uint8_t)((((uint32_t)gyro)|0xFFFFFF00));		// 8 bit L

	dataTransmit[10] = motor_dir;
}

/*
 * Input: uint8_t receivebuffer[16];

*/
void Byte2Dec(){
	k[0] = (float)(((int16_t)receivebuffer[0]<<8)|(int16_t)receivebuffer[1]);
	k[1] = (float)(((int16_t)receivebuffer[2]<<8)|(int16_t)receivebuffer[3]);
	k[2] = (float)(((int16_t)receivebuffer[4]<<8)|(int16_t)receivebuffer[5]);
	k[3] = (float)(((int16_t)receivebuffer[6]<<8)|(int16_t)receivebuffer[7]);
}

void UartTransmit(uint32_t accel, uint32_t gyro, uint16_t encA, uint16_t encB, uint8_t motor_dir){
	Dec2Bytes(accel, gyro, encA, encB, motor_dir);

	HAL_UART_Transmit(&huart2, &dataTransmit[0], sizeof(dataTransmit), 1);

	Byte2Dec();

}
