/*
 * motor.h
 *
 *  Created on: Oct 15, 2020
 *      Author: son
 */

#ifndef LIB_MOTOR_H_
#define LIB_MOTOR_H_



#endif /* LIB_MOTOR_H_ */

void Control_Motor(int16_t duty_r,int16_t duty_l);
float * Get_Velocity();

volatile float *velo;
volatile int16_t encoder[2];
volatile float /*enc[2], */test[10];
volatile float enc[4];
