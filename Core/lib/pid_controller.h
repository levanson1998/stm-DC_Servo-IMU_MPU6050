/*
 * pid_controller.h
 *
 *  Created on: Sep 17, 2020
 *      Author: son
 */

#ifndef LIB_PID_CONTROLLER_H_
#define LIB_PID_CONTROLLER_H_

#include "stdint.h"

extern volatile float PID_current[2];
extern volatile float PID_in[2];
extern float PID_out[2];
extern float error, PID_P[2], PID_I[2], PID_D[2];
extern float PID_Kp[2], PID_Ki[2], PID_Kd[2];
extern float PID_pre_err[2], PID_ppre_err[2];
extern float PID_out_max, PID_out_min, PID_T;
extern float A0, A1, A2, Aout, Aout1, E0, E1, E2;

void PID_Init(float *Kp, float *Ki, float *Kd, float Ts);
float *PID_Calculate(float *PID_in, float *PID_current1);


#endif /* LIB_PID_CONTROLLER_H_ */
