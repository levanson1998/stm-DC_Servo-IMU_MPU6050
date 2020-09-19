/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
//#include <stdint.h>
#include "inttypes.h"

#include "../lib/pid_controller.h"
#include "../lib/IMU_MPU6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile float *velo;
volatile int16_t encoder[2];
volatile float /*enc[2], */test[10];
volatile float enc[4];

/* UART 2*/
uint8_t receivebuffer[6], transmitData[3];
uint8_t dataTransmit[16];
float data_Receive[2];
/* PID Controller*/
float Kp[2] = {20.5f, 6.0f};
float Ki[2] = {2.0f, 1.0f};
float Kd[2] = {0.03f, 0.0f};
float Ts = 5; // 5ms
float vt;

/*  interupt*/
volatile float PID_current[2], v_target[2], *duty_cycle;

float A0, A1, A2, Aout, E0, E1, E2;
float Aout1;
/*test*/
uint32_t t1,t2,t3,t4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void Control_Motor(int16_t duty_l,int16_t duty_r);
float * Get_Velocity();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
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

// data1: x, data2: y, Kpid
void Transmit_Uart(float x, float y, float v_l, int dir_l, float v_r, int dir_r){


	dataTransmit[0]=(uint8_t)((((uint16_t)x)|0x00FF)>>8); // 8 bit H
	dataTransmit[1]=(uint8_t)((((uint16_t)x)|0xFF00)); 	      // 8 bit L
	dataTransmit[2]=(uint8_t)((((uint16_t)((x-(uint16_t)x)*10000.0f))|0x00FF)>>8); // 8 bit H
	dataTransmit[3]=(uint8_t)((((uint16_t)((x-(uint16_t)x)*10000.0f))|0xFF00));    // 8 bit L

	dataTransmit[4]=(uint8_t)((((uint16_t)y)|0x00FF)>>8); // 8 bit H
	dataTransmit[5]=(uint8_t)((((uint16_t)y)|0xFF00)); 	      // 8 bit L
	dataTransmit[6]=(uint8_t)((((uint16_t)((y-(uint16_t)y)*10000.0f))|0x00FF)>>8); // 8 bit H
	dataTransmit[7]=(uint8_t)((((uint16_t)((y-(uint16_t)y)*10000.0f))|0xFF00));    // 8 bit L

	dataTransmit[8]=(uint8_t)v_l; // 8 bit truoc dau .
	dataTransmit[9]=(uint8_t)((((uint16_t)((v_l-(uint16_t)v_l)*10000.0f))|0x00FF)>>8); // 8 bit H
	dataTransmit[10]=(uint8_t)((((uint16_t)((v_l-(uint16_t)v_l)*10000.0f))|0xFF00));    // 8 bit L

	dataTransmit[11]=(uint8_t)dir_l;

	dataTransmit[12]=(uint8_t)v_r; // 8 bit truoc dau .
	dataTransmit[13]=(uint8_t)((((uint16_t)((v_r-(uint16_t)v_r)*10000.0f))|0x00FF)>>8); // 8 bit H
	dataTransmit[14]=(uint8_t)((((uint16_t)((v_r-(uint16_t)v_r)*10000.0f))|0xFF00));    // 8 bit L

	dataTransmit[15]=(uint8_t)dir_r;
//	for(int i=0;i<14;i++){
//		dataTransmit[i]=0xF3;
//	}

	HAL_UART_Transmit(&huart2, &dataTransmit[0], sizeof(dataTransmit), 1);
//	dataTransmit[8]=(int8_t)v_l; // 8 bit truoc dau .
//	dataTransmit[9]=(int8_t)((((int16_t)((x-(int16_t)v_l)*10000))|0xF0)>>8); // 8 bit H
//	dataTransmit[10]=(int8_t)((((int16_t)((x-(int16_t)v_l)*10000))|0x0F));    // 8 bit L
}

float *Receive_Uart(){
	data_Receive[0]=(float)(receivebuffer[0]+(float)(((uint16_t)((receivebuffer[1]<<8)|receivebuffer[2])/10000.0f)));
	data_Receive[1]=(float)(receivebuffer[3]+(float)(((uint16_t)((receivebuffer[4]<<8)|receivebuffer[5])/10000.0f)));

	return data_Receive;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART2_UART_Init();
  MX_TIM5_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_Base_Start_IT(&htim9);
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_UART_Receive_DMA(&huart2 ,&receivebuffer[0], 6);

  PID_Init(Kp, Ki, Kd, Ts);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // HAL_GPIO_TogglePin(GPIOD, LED_BLU_Pin);

//	  HAL_GPIO_TogglePin(GPIOD, LED_RED_Pin);
//	  HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
//	delay 5ms
	if(htim->Instance==htim5.Instance){
		volatile float *data_Receive;
		t1++;
		HAL_GPIO_TogglePin(GPIOD, LED_GRE_Pin);
		velo = Get_Velocity();
		data_Receive = Receive_Uart();
		for(int i=0;i<2;i++){
			PID_current[i]=*(velo+i*2);
//			v_target[i]=*(data_Receive+i);
//			v_target[i] = 10.0f;
		}

		duty_cycle = PID_Calculate(v_target, PID_current);
		test[2]=*(duty_cycle);
		test[3]=*(duty_cycle+1);
		Control_Motor(*(duty_cycle), *(duty_cycle+1));
	}
//	delay 100ms
	else if(htim->Instance==htim9.Instance){
		HAL_GPIO_TogglePin(GPIOD, LED_RED_Pin);
		v_target[0] = 15.0;
/*		if(v_target[0] >= 19.0f) vt=-0.5f;
		else if (v_target[0] <= 2.0) vt = 0.5f;*/
		Transmit_Uart(523.456, 321.654,*(velo), *(velo+1), *(velo+2), *(velo+3));
//		Transmit_Uart(523.456, 321.654,12.356,1,20.214,3);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
