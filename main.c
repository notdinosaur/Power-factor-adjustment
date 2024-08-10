/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "vofa.h"
#include "single_sv.h"
#include "control.h"
#include "arm_math.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float SIN[360] = {0.0000, 0.0175, 0.0349, 0.0523, 0.0698, 0.0872, 0.1045, 0.1219, 0.1392, 0.1564, 0.1736, 0.1908, 0.2079,//13个
				   0.2250, 0.2419, 0.2588, 0.2756, 0.2924, 0.3090, 0.3256, 0.3420, 0.3584, 0.3746, 0.3907, 0.4067, 0.4226, 0.4384, 0.4540,//15个
				   0.4695, 0.4848, 0.5000, 0.5150, 0.5299, 0.5446, 0.5592, 0.5736, 0.5878, 0.6018, 0.6157, 0.6293, 0.6428, 0.6561, 0.6691,
				   0.6820, 0.6947, 0.7071, 0.7193, 0.7314, 0.7431, 0.7547, 0.7660, 0.7771, 0.7880, 0.7986, 0.8090, 0.8192, 0.8290, 0.8387,
				   0.8480, 0.8572, 0.8660, 0.8746, 0.8829, 0.8910, 0.8988, 0.9063, 0.9135, 0.9205, 0.9272, 0.9336, 0.9397, 0.9455, 0.9511,
				   0.9563, 0.9613, 0.9659, 0.9703, 0.9744, 0.9781, 0.9816, 0.9848, 0.9877, 0.9903, 0.9925, 0.9945, 0.9962, 0.9976, 0.9986,
				   0.9994, 0.9998, 1.0000, 0.9998, 0.9994, 0.9986, 0.9976, 0.9962, 0.9945, 0.9925, 0.9903, 0.9877, 0.9848, 0.9816, 0.9781,
				   0.9744, 0.9703, 0.9659, 0.9613, 0.9563, 0.9511, 0.9455, 0.9397, 0.9336, 0.9272, 0.9205, 0.9135, 0.9063, 0.8988, 0.8910,
				   0.8829, 0.8746, 0.8660, 0.8572, 0.8480, 0.8387, 0.8290, 0.8192, 0.8090, 0.7986, 0.7880, 0.7771, 0.7660, 0.7547, 0.7431,
				   0.7314, 0.7193, 0.7071, 0.6947, 0.6820, 0.6691, 0.6561, 0.6428, 0.6293, 0.6157, 0.6018, 0.5878, 0.5736, 0.5592, 0.5446,
				   0.5299, 0.5150, 0.5000, 0.4848, 0.4695, 0.4540, 0.4384, 0.4226, 0.4067, 0.3907, 0.3746, 0.3584, 0.3420, 0.3256, 0.3090,
				   0.2924, 0.2756, 0.2588, 0.2419, 0.2250, 0.2079, 0.1908, 0.1737, 0.1564, 0.1392, 0.1219, 0.1045, 0.0872, 0.0698, 0.0523,
				   0.0349, 0.0175, 0.0000, -0.0174, -0.0349, -0.0523, -0.0698, -0.0872, -0.1045, -0.1219, -0.1392, -0.1564, -0.1736, -0.1908,
				   -0.2079, -0.2249, -0.2419, -0.2588, -0.2756, -0.2924, -0.3090, -0.3256, -0.3420, -0.3584, -0.3746, -0.3907, -0.4067, -0.4226,
				   -0.4384, -0.4540, -0.4695, -0.4848, -0.5000, -0.5150, -0.5299, -0.5446, -0.5592, -0.5736, -0.5878, -0.6018, -0.6157, -0.6293,
				   -0.6428, -0.6561, -0.6691, -0.6820, -0.6947, -0.7071, -0.7193, -0.7314, -0.7431, -0.7547, -0.7660, -0.7771, -0.7880, -0.7986,
				   -0.8090, -0.8191, -0.8290, -0.8387, -0.8480, -0.8572, -0.8660, -0.8746, -0.8829, -0.8910, -0.8988, -0.9063, -0.9135, -0.9205,
				   -0.9272, -0.9336, -0.9397, -0.9455, -0.9511, -0.9563, -0.9613, -0.9659, -0.9703, -0.9744, -0.9781, -0.9816, -0.9848, -0.9877,
				   -0.9903, -0.9925, -0.9945, -0.9962, -0.9976, -0.9986, -0.9994, -0.9998, -1.0000, -0.9998, -0.9994, -0.9986, -0.9976, -0.9962,
				   -0.9945, -0.9925, -0.9903, -0.9877, -0.9848, -0.9816, -0.9781, -0.9744, -0.9703, -0.9659, -0.9613, -0.9563, -0.9511, -0.9455,
				   -0.9397, -0.9336, -0.9272, -0.9205, -0.9135, -0.9063, -0.8988, -0.8910, -0.8829, -0.8746, -0.8660, -0.8572, -0.8481, -0.8387,
				   -0.8290, -0.8192, -0.8090, -0.7986, -0.7880, -0.7771, -0.7660, -0.7547, -0.7431, -0.7314, -0.7193, -0.7071, -0.6947, -0.6820,
				   -0.6691, -0.6561, -0.6428, -0.6293, -0.6157, -0.6018, -0.5878, -0.5736, -0.5592, -0.5446, -0.5299, -0.5150, -0.5000, -0.4848,
				   -0.4695, -0.4540, -0.4384, -0.4226, -0.4067, -0.3907, -0.3746, -0.3584, -0.3420, -0.3256, -0.3090, -0.2924, -0.2756, -0.2588,
				   -0.2419, -0.2250, -0.2079, -0.1908, -0.1737, -0.1564, -0.1392, -0.1219, -0.1045, -0.0872, -0.0698, -0.0523, -0.0349, -0.0175};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PAI 3.1415926f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
	SV sv_1;
	SV sv_2;
	SOGI sg_U;
  SOGI sg_I;
	Transfer tran;
  PR pr;
  PID pi_D;
	PID pi_Q;
  PID pi_DC;
	float Ui=0,Udc=0,Ii=0;
  float U_adc=0,I_adc=0,Udc_adc=0;
  float Ud=0,Uq=0,U_alpha=0,U_beta=0;
  float Id=0,Iq=0,I_alpha=0,I_beta=0;
	float sin_sg=0,cos_sg=0,L=0;
  float pi_d=0,pi_q=0,pi_dc=0;
  float pr_out=0,pi_out_D=0,pi_out_Q=0,pi_out_dc=0;
  float I = 0;
  int Times=0,i=0;
	float power_factor=0.0f;
	int i_nibian=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

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
	MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  HAL_TIM_Base_Stop_IT(&htim3);


  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */
	SOGI_init(&sg_U,1,2*PAI*50,0.02/360);
  SOGI_init(&sg_I,1,2*PAI*50,0.02/360);
	pi_init(&pi_D);
	pi_init(&pi_Q);
  PR_init(&pr,0.02f,105.0f,0.2f/360,1.0f,2*PAI*50);

  HAL_TIM_Base_Start_IT(&htim3);
  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) == GPIO_PIN_RESET) 
		{ 
			power_factor+=0.1f;
			if(power_factor>0.5f) power_factor=0;
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_8) != GPIO_PIN_SET);	
		}
		if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) == GPIO_PIN_RESET) 
		{
			power_factor-=0.1f;
			if(power_factor<-0.5f) power_factor=0;
			while(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) != GPIO_PIN_SET);	
		}
    /* USER CODE BEGIN 3 */
		vodka_JustFloat_send(&huart1);

		
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}



/*定时器3中断服务函数*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)   //输入交流峰值42.4V  负载40欧姆
{
    if (htim == (&htim3))
    {
			i++;
			i_nibian++;
      if(Times<50000)  Times++;
      else 
      {
        I = 0.00015f;
				//I = 0.0f;
      }
			
			if(i_nibian++ ==359) i_nibian=0;
			
			//采集直流侧电压
			Udc_adc = Get_Adc_Value(ADC_CHANNEL_12);
      Udc = 0.0169f*Udc_adc;
			
      //采集网侧电流电压
      U_adc = Get_Adc_Value(ADC_CHANNEL_10);
      I_adc = Get_Adc_Value(ADC_CHANNEL_11);
      Ui = 0.0269f*U_adc - 50.65f;
      Ii = 0.0028f*I_adc - 5.257f-0.15f;
			
			if(i%5==0)	pi_out_dc = CurrentPIControl(&pi_DC,0.1f,0.0001f,50.0f,Udc);
			

      SOGI_Transfer(&sg_U,Ui);	
      SOGI_Transfer(&sg_I,Ii);	

      //角度计算
      L = sqrt(sg_U.vo*sg_U.vo + sg_U.qvo * sg_U.qvo);
      sin_sg = sg_U.qvo/L;
      cos_sg = sg_U.vo/L;

      Park_transfer(&tran,sg_I.vo,sg_I.qvo,sin_sg,cos_sg);	
      Id = tran.Ud; 
      Iq = tran.Uq;

      pi_out_D = CurrentPIControl(&pi_D,0.25f,I,pi_out_dc,Id);
			//pi_out_D = CurrentPIControl(&pi_D,0.25f,I,1.98f,Id);
      pi_out_Q = CurrentPIControl(&pi_Q,0.25f,I,power_factor*Id,Iq);  

      pi_out_D = L - pi_out_D + 0.628f*Iq;
      pi_out_Q = 0.0f - pi_out_Q - 0.628f*Id;

      Inverse_Park_Transfer(&tran,pi_out_D,pi_out_Q,sin_sg,cos_sg);

      single_sv(&sv_1,tran.Ualpha,50.0f);  //  48.9v  40ou
      __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1,sv_1.CCR1);
      __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_2,sv_1.CCR2);
			
			
			single_sv(&sv_2,50.0f*SIN[i_nibian],50.0f);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,sv_2.CCR1);
      __HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,sv_2.CCR2);
			
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
