/*
 * motor.c
 *
 *  Created on: Oct 15, 2020
 *      Author: son
 */
#include "math.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"

#include "tim.h"

#include "motor.h"

uint8_t mt_onetime=1;
/*
 * GPIO: PD11 -> RPWM_L
 * 		 PD10 -> LPWM_L
 * 		 PD9  -> RPWM_R
 * 		 PD8  -> LPWM_R
 * 		 PE9  -> EN_L
 * 		 PE11 -> EN_R
 *
 * 		 dir_motor == 0000 0x2x1x0 (8 bits)
 *		 x0: motor right
 *		 x1: motor left
 *		 1: tien
 *		 0: lui
 *
*/
void Control_Motor(int16_t duty_r,int16_t duty_l, uint8_t dir_motor){
	mt_test[0] = duty_r;
	mt_test[1] = duty_l;
	mt_test[2] = dir_motor;
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, duty_r);
	if(dir_motor&1){
		HAL_GPIO_TogglePin(GPIOD, LED_GRE_Pin);
//		tien
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_SET);
	}
	else{
//		lui
		HAL_GPIO_WritePin(GPIOD, RPWM_R_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LPWM_R_Pin, GPIO_PIN_RESET);
	}

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty_l);
	if(dir_motor&2){
//		tien
		HAL_GPIO_TogglePin(GPIOD, LED_RED_Pin);
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_SET);
	}
	else{
//		lui
		HAL_GPIO_WritePin(GPIOD, RPWM_L_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOD, LPWM_L_Pin, GPIO_PIN_RESET);
	}

}


//	TIM4->CNT=3000;
//	TIM2->CNT=3000;

/*
 * return pointer *(enc+0)  *(enc+1)
 * GPIO: PB7 -> ENC-L1
 * 		 PB6 -> ENC-L2
 * 		 PA5 -> ENC-R1
 * 		 PB3 -> ENC-R2
*/
volatile int16_t * Get_Velocity(){
	if (mt_onetime==1){
		TIM4->CNT=5000;
		TIM2->CNT=5000;
		mt_onetime=0;
	}
//	volatile float enc[2];
	enc[0]= (TIM2->CNT)-5000;
//	if ((TIM2->CNT)>=5000) enc[1]=-1;
//	else enc[1]=1;

	enc[1]= (TIM4->CNT)-5000;
//	if ((TIM4->CNT)>5000) enc[3]=1;
//	else enc[3]=-1;

	TIM4->CNT=5000;
	TIM2->CNT=5000;
	int i;
  	for (int i=0;i<2; i++){
		if (enc[i]>=0){
			enc[2]=enc[2]|(i+1);
		}
		else if (enc[i]<0){
			enc[2]=enc[2]&(2-i);
		}
  	}

	return 0;
}
