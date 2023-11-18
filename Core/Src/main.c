/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define M1_1_H    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET)    
#define M1_2_H    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_SET)    
#define M2_1_H    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_SET)    
#define M2_2_H    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_SET)   
#define M3_1_H    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_SET)   
#define M3_2_H    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET)   
#define M4_1_H    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)   
#define M4_2_H    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)   

#define M1_1_L    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET)    
#define M1_2_L    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_4,GPIO_PIN_RESET)    
#define M2_1_L    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,GPIO_PIN_RESET)    
#define M2_2_L    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,GPIO_PIN_RESET)   
#define M3_1_L    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_12,GPIO_PIN_RESET)   
#define M3_2_L    HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_RESET)   
#define M4_1_L    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)   
#define M4_2_L    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)   
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
float MWmotor1=0,MWmotor2=0,MWmotor3=0,MWmotor4=0;
const float K = 1;
uint8_t buf[8];
double sx,sy,oa;
uint8_t pos[10];
int qianduo = 0;
int houduo = 0;
int intake = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
void MecanumWheel(float speedx,float speedy,float omega){
	MWmotor1=-speedx-speedy+K*omega;
	MWmotor2=+speedx-speedy+K*omega;
	MWmotor3=-speedx+speedy+K*omega;
	MWmotor4=+speedx+speedy+K*omega;
}
void Motor_Speed_Control(float motor1, float motor2, float motor3, float motor4)	 
{
    if(motor1>0) {
		M1_1_H;
		M1_2_L;
	}
	else
		if(motor1<0){
			M1_1_L;
			M1_2_H;
		}
		else
			if(motor1 == 0){
				M1_1_H;
				M1_2_H;
			}
	if(motor2>0) {
		M2_1_H;
		M2_2_L;
	}
	else
		if(motor2<0){
			M2_1_L;
			M2_2_H;
		}
		else
			if(motor2 == 0){
				M2_1_H;
				M2_2_H;
			}
	if(motor3>0) {
		M3_1_H;
		M3_2_L;
	}
	else
		if(motor3<0){
			M3_1_L;
			M3_2_H;
		}
		else
			if(motor3 == 0){
				M3_1_H;
				M3_2_H;
			}
	if(motor4>0) {
		M4_1_H;
		M4_2_L;
	}
	else
		if(motor4<0){
			M4_1_L;
			M4_2_H;
		}
		else
			if(motor4 == 0){
				M4_1_H;
				M4_2_H;
			}
}
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1); 
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	
	//__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,1150);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_7,GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,GPIO_PIN_SET);
	  
	  
	  HAL_UART_Receive(&huart1,buf,1,0xFFFF);
	  if(buf[0]=='F'){
		  oa=1;
	  }
	  if(buf[0]=='D'){
		  oa=-1;
	  }
	  if(buf[0]=='E'){
		  oa=0;
	  }
	  if(buf[0]=='G'){
		  sx=1;
	  }
	  if(buf[0]=='K'){
		  sx=-1;
	  }
	  if(buf[0]=='H'){
		  sy=1;
	  }
	  if(buf[0]=='J'){
		  sy=-1;
	  }
	  if(buf[0]=='I'){
		  sx=0;
		  sy=0;
	  }
	  if(buf[0] == 'A'&&!qianduo){
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,1750);
		  qianduo = 1;
		  buf[0] = 0; 
	  }
	  if(buf[0] == 'A'&&qianduo){
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1,1500);
		  qianduo = 0;
		  buf[0] = 0;
	  }
	  if(buf[0] == 'B'&&!houduo){
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,800);
		  houduo = 1;
		  buf[0] = 0;
	  }
	  if(buf[0] == 'B'&&houduo){
		  __HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2,1150);
		  houduo = 0;
		  buf[0] = 0;
	  }
	  if(buf[0] == 'C'&&!intake){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_RESET);
		  intake = 1;
		  buf[0] = 0;
	  }
	  if(buf[0] == 'C'&&intake){
		  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
		  intake = 0;
		  buf[0] = 0;
		  
	  }
	  
	  
	  /*
	  for(int i=0;i<8;i++){
		  if(buf[i]==0x5A){
			  index = i;
		  }
	  }
	  for(int i=0;i<8;i++){
		  if(index+i<8){
			  pos[i] = buf[index+i];
		  }
		  if(index+i>=8){
			  pos[i] = buf[index+i-8];
		  }
	  }
	  sx = pos[1];
	  sy = pos[3];
	  oa = pos[5];
	  if(pos[2]==0xFF){
		  sx=-sx;
	  }
	  if(pos[4]==0xFF){
		  sy=-sy;
	  }*/
	  MecanumWheel(sx,sy,oa);
	  Motor_Speed_Control(MWmotor1, MWmotor2,MWmotor3 , MWmotor4);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
