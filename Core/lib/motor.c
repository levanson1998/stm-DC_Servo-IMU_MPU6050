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
*/
void Control_Motor(int16_t duty_r,int16_t duty_l, uint8_t dir_r, uint8_t dir_l){

	__HAL_TIM_SET_COMPARE(&htim1, EN_R_Pin, duty_r);
	if(dir_r==0){
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_RESET);
	}
	else if(dir_r==1){
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_RESET);
	}
	else if(dir_r==2){
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_SET);
	}

	__HAL_TIM_SET_COMPARE(&htim1, EN_L_Pin, duty_l);
	if(dir_r==0){
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_RESET);
	}
	else if(dir_r==1){
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_SET);
	}
	else if(dir_r==2){
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_RESET);
	}
}

/*
 * return pointer *(enc+0)  *(enc+1)

*/
volatile int16_t * Get_Velocity(){
//	volatile float enc[2];
	enc[0]= (TIM2->CNT)-1000;
//	if ((TIM2->CNT)>=5000) enc[1]=-1;
//	else enc[1]=1;

	enc[2]= (TIM4->CNT)-1000;
//	if ((TIM4->CNT)>5000) enc[3]=1;
//	else enc[3]=-1;

	TIM4->CNT=1000;
	TIM2->CNT=1000;

	return enc;
}
