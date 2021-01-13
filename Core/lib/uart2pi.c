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

	dataTransmit[0] = 0x7F;

	dataTransmit[1]=(int)((((int16_t)encA)|0x00FF)>>8); // 8 bit H
	dataTransmit[2]=(int)((((int16_t)encA)|0xFF00)); 	      // 8 bit L

	dataTransmit[3]=(int)((((int16_t)encB)|0x00FF)>>8); // 8 bit H
	dataTransmit[4]=(int)((((int16_t)encB)|0xFF00)); 	      // 8 bit L

	dataTransmit[5]=(int)((((int32_t)ss.accel_x)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[6]=(int)((((int32_t)ss.accel_x)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[7]=(int)((((int32_t)ss.accel_x)|0xFFFFFF00));		// 8 bit L

	dataTransmit[8]=(int)((((int32_t)ss.accel_y)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[9]=(int)((((int32_t)ss.accel_y)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[10]=(int)((((int32_t)ss.accel_y)|0xFFFFFF00));		// 8 bit L

	dataTransmit[11]=(int)((((int32_t)ss.accel_z)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[12]=(int)((((int32_t)ss.accel_z)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[13]=(int)((((int32_t)ss.accel_z)|0xFFFFFF00));		// 8 bit L

	dataTransmit[14]=(int)((((int32_t)ss.gyro_x)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[15]=(int)((((int32_t)ss.gyro_x)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[16]=(int)((((int32_t)ss.gyro_x)|0xFFFFFF00));		// 8 bit L

	dataTransmit[17]=(int)((((int32_t)ss.gyro_y)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[18]=(int)((((int32_t)ss.gyro_y)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[19]=(int)((((int32_t)ss.gyro_y)|0xFFFFFF00));		// 8 bit L

	dataTransmit[20]=(int)((((int32_t)ss.gyro_z)|0xFF00FFFF)>>16); // 8 bit H
	dataTransmit[21]=(int)((((int32_t)ss.gyro_z)|0xFFFF00FF)>>8); 	      // 8 bit M
	dataTransmit[22]=(int)((((int32_t)ss.gyro_z)|0xFFFFFF00));		// 8 bit L

	dataTransmit[23] = (int)motor_dir;
	dataTransmit[24] = 0x1B; // new line (in python using 'serial.readline(-1)' to read data)

/*
	uint8_t i;
	for (i=0; i<23; i++){
		dataTransmit[i] = i;
	}
*/
}

/*
 * Input: uint8_t receivebuffer[16];
 * velo[0]: left
 * velo[1]: right
 *

*/
void Byte2Dec(){
	_velo[0] = (float)receivebuffer[1] + (float)(((int16_t)receivebuffer[2]<<8)|(int16_t)receivebuffer[3])/10000.0F;
	_velo[1] = (float)receivebuffer[4] + (float)(((int16_t)receivebuffer[5]<<8)|(int16_t)receivebuffer[6])/10000.0F;
	_motor_dir = receivebuffer[7];

	if (!(receivebuffer[0] == 200) | !(receivebuffer[8] == 201)){
		check_error = 1;
	}
	else{
		check_error = 0;
	}

//	else check_error = 0;
//	_motor_dir = 2;
//	_velo[0] = 0.04;
//	_velo[1] = 0.00;
}

/*
 * Transmit from STM to RP3
 * GPIO: PA2 -> TX
 * 		 PA3 -> RX
*/
void UartTransmit(int16_t encA, int16_t encB, struct data_imu ss, uint8_t motor_dir){
	Byte2Dec();
	uart_test[0] = encA;
	uart_test[1] = encB;
	uart_test[2] = ss.accel_x;
	uart_test[3] = ss.accel_y;
	uart_test[4] = ss.accel_z;
	uart_test[5] = ss.gyro_x;
	uart_test[6] = ss.gyro_y;
	uart_test[7] = ss.gyro_z;

	Dec2Bytes(encA, encB, ss, motor_dir);
	HAL_UART_Transmit(&huart2, &dataTransmit[0], sizeof(dataTransmit), 1);


}
