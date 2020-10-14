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

void Control_Motor(int16_t duty_r,int16_t duty_l){
	test[0]=duty_r;
	test[1]=duty_l;


	HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_R_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_r);

	HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_L_Pin, GPIO_PIN_RESET);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);


	HAL_GPIO_TogglePin(GPIOD, LED_ORG_Pin);

/*
	if(duty_l>=0){
		HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_L_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_L_Pin, 100);
	}
	else{
		HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_L_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_L_Pin, duty_l);
	}

	if(duty_r>=0){
		HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_R_Pin, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_R_Pin, duty_r);
	}
	else{
		HAL_GPIO_WritePin(GPIOD, MOTOR_DIR_R_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim1, MOTOR_R_Pin, duty_r);
	}
*/
}

float * Get_Velocity(){
//	volatile float enc[2];
	enc[0]= fabs((TIM2->CNT)-5000.0F);
	if ((TIM2->CNT)>=5000) enc[1]=-1;
	else enc[1]=1;

	enc[2]= fabs((TIM4->CNT)-5000.0F);
	if ((TIM4->CNT)>5000) enc[3]=1;
	else enc[3]=-1;

	TIM4->CNT=5000;
	TIM2->CNT=5000;

	return enc;
}
