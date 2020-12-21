/*
 * pid_controller.h
 *
 *  Created on: Sep 17, 2020
 *      Author: son
 */

#ifndef LIB_PID_CONTROLLER_H_
#define LIB_PID_CONTROLLER_H_

#endif /* LIB_PID_CONTROLLER_H_ */

//			  time   banh xe  gear encoder
// updateVel=(0.005)/(0.096/2)*30*64 (wrong)
#define  updateVel 31.83f;

//extern volatile int16_t PID_current[2];
//extern volatile float PID_in[2];
extern float PID_out[3], PID_out_[3];
extern float error, PID_P[2], PID_I[2], PID_D[2];
extern float PID_Kp[2], PID_Ki[2], PID_Kd[2];
extern float PID_pre_err[2], PID_ppre_err[2];

//extern float A0, A1, A2, Aout, Aout1, E0, E1, E2;


//volatile float PID_in[2];
float PID_out[3];
float PID_out_[3];
//float PID_P[2], PID_I[2], PID_D[2];
float PID_Kp[2], PID_Ki[2], PID_Kd[2];
float PID_Test[10];
float PID_pre_err[2], PID_ppre_err[2];
//float PID_out_max=299.0f;
//float PID_out_min=0.1f;
//float PID_T;
float PID1[2];

float error, PID_P[2], PID_I[2], PID_D[2];
float PID_Kp[2], PID_Ki[2], PID_Kd[2];

//extern float PID_T;

uint8_t TxBuffer[2], RxBuffer[7];
/*
float Kp[2] = {20.5f, 6.0f};
float Ki[2] = {2.0f, 1.0f};
float Kd[2] = {0.03f, 0.0f};
float Ts = 5; // 5ms
float vt;

*/
volatile float PID_current[2], PID_current[2], v_target[2], *duty_cycle;
float PID_in[2], PID_in_pre[2];

float A0[2], A1[2], A2[2], Aout[2], E0[2], E1[2], E2[2];
float Aout1;

void PID_Init(float *Kp, float *Ki, float *Kd, float Ts);
float * PID_Calculate(float *_PID_in, int PID_dir, volatile int16_t *_PID_current);


