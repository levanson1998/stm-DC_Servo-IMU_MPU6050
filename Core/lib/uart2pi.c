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
void Dec2Bytes(int16_t encA, int16_t encB, struct data_imu ss, uint8_t motor_dir){
	dataTransmit[0]=(int)((((uint16_t)encA)|0x00FF)>>8); // 8 bit H
	dataTransmit[1]=(int)((((uint16_t)encA)|0xFF00)); 	      // 8 bit L

	dataTransmit[2]=(int)((((uint16_t)encB)|0x00FF)>>8); // 8 bit H
	dataTransmit[3]=(int)((((uint16_t)encB)|0xFF00)); 	      // 8 bit L

	dataTransmit[4]=(int)((((uint32_t)ss.accel_x)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[5]=(int)((((uint32_t)ss.accel_x)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[6]=(int)((((uint32_t)ss.accel_x)|0xFFFFFF00));		// 8 bit L

	dataTransmit[7]=(int)((((uint32_t)ss.accel_y)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[8]=(int)((((uint32_t)ss.accel_y)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[9]=(int)((((uint32_t)ss.accel_y)|0xFFFFFF00));		// 8 bit L

	dataTransmit[10]=(int)((((uint32_t)ss.accel_z)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[11]=(int)((((uint32_t)ss.accel_z)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[12]=(int)((((uint32_t)ss.accel_z)|0xFFFFFF00));		// 8 bit L

	dataTransmit[13]=(int)((((int32_t)ss.gyro_x)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[14]=(int)((((int32_t)ss.gyro_x)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[15]=(int)((((int32_t)ss.gyro_x)|0xFFFFFF00));		// 8 bit L

	dataTransmit[16]=(int)((((int32_t)ss.gyro_y)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[17]=(int)((((int32_t)ss.gyro_y)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[18]=(int)((((int32_t)ss.gyro_y)|0xFFFFFF00));		// 8 bit L

	dataTransmit[19]=(int)((((int32_t)ss.gyro_z)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[20]=(int)((((int32_t)ss.gyro_z)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[21]=(int)((((int32_t)ss.gyro_z)|0xFFFFFF00));		// 8 bit L

	dataTransmit[22] = (int)motor_dir;
	dataTransmit[23] = 0x0A; // new line (in python using 'serial.readline()' to read data)
}

/*
 * Input: uint8_t receivebuffer[16];
 * velo[0]: left
 * velo[1]: right
 *

*/
void Byte2Dec(){
/*
	_velo[0] = (float)receivebuffer[0] + (float)(((int16_t)receivebuffer[1]<<8)|(int16_t)receivebuffer[2])/10000.0F;
	_velo[1] = (float)receivebuffer[3] + (float)(((int16_t)receivebuffer[4]<<8)|(int16_t)receivebuffer[5])/10000.0F;
	_motor_dir = receivebuffer[6];
*/


	_motor_dir = 2;
	_velo[0] = 0.04;
	_velo[1] = 0.00;
	uart_test += 1;

}

/*
 * Transmit from STM to RP3
 * GPIO: PA2 -> TX
 * 		 PA3 -> RX
*/
void UartTransmit(int16_t encA, int16_t encB, struct data_imu ss, uint8_t motor_dir){

	Dec2Bytes(encA,encB, ss, motor_dir);
	HAL_UART_Transmit(&huart2, &dataTransmit[0], sizeof(dataTransmit), 1);
	Byte2Dec();


}
