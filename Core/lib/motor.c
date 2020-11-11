/*
 * motor.c
 *
 *  Created on: Oct 15, 2020
 *      Author: son
 */
#include "math.h"
#include <stdint.h>
#include <stdio.h>

#include "tim.h"

#include "motor.h"

/*
 * dir_r, dir_l:
 	 0: stop
 	 1: tien
 	 2: lui
 * GPIO: PD11 -> RPWM_L
 * 		 PD10 -> LPWM_L
 * 		 PD9  -> RPWM_R
 * 		 PD9  -> LPWM_R
 * 		 PE9  -> EN_L
 * 		 PE11 -> EN_R
 *
 * 		 dir_motor == 0000 0x2x1x0 (8 bits)
 *		 x0: motor left
 *		 x1: motor right
 *		 1: tien
 *		 0: lui
 *
*/
void Control_Motor(int16_t duty_r,int16_t duty_l, uint8_t dir_motor){

	__HAL_TIM_SET_COMPARE(&htim1, EN_R_Pin, duty_r);
	if(dir_motor&2){
//		tien
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_SET);
	}
	else{
//		lui
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_RESET);
	}

	__HAL_TIM_SET_COMPARE(&htim1, EN_L_Pin, duty_l);
	if(dir_motor&1){
//		tien
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_SET);
	}
	else{
//		lui
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_RESET);
	}
}

/*
 * return pointer *(enc+0)  *(enc+1)
 * GPIO: PB7 -> ENC-L1
 * 		 PB6 -> ENC-L2
 * 		 PA3 -> ENC-R1
 * 		 PB3 -> ENC-R2
*/
volatile int16_t * Get_Velocity(){
//	volatile float enc[2];
	enc[0]= (TIM2->CNT)-1000;
//	if ((TIM2->CNT)>=5000) enc[1]=-1;
//	else enc[1]=1;

	enc[1]= (TIM4->CNT)-1000;
//	if ((TIM4->CNT)>5000) enc[3]=1;
//	else enc[3]=-1;

	TIM4->CNT=1000;
	TIM2->CNT=1000;

	return 0;
}
