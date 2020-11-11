/*
 * motor.h
 *
 *  Created on: Oct 15, 2020
 *      Author: son
 */

#ifndef LIB_MOTOR_H_
#define LIB_MOTOR_H_



#endif /* LIB_MOTOR_H_ */

void Control_Motor(int16_t duty_r,int16_t duty_l, uint8_t dir_motor);
volatile int16_t * Get_Velocity();

volatile int16_t encoder[2];
volatile float /*enc[2], */test[10];
volatile int16_t enc[4];
