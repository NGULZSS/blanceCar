/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
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
#include "../Include/HardwareDriver/main.h"
/* USER CODE END Includes */

float pitch, roll, yaw;
float ADC_Value;
uint8_t delay_50=0,delay_flag=0; 	
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
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);//开启定时器3的编码器模式
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4的编码器模式
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);//开启定时器2通道1的PWM输出，左轮电机使用
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);//开启定时器2通道2的PWM输出，右轮电机使用
   
   oled_init();
   MPU_Init();
   mpu_dmp_init();
   HAL_NVIC_EnableIRQ(EXTI15_10_IRQn); //开启引脚外部中断
   HAL_Delay(100);
  //  oled_show_string(0,0,"Mode:",2);
  Red_LED_OFF;
  Green_LED_OFF;
  STBY_LOW
  while (1)
  {
    // delay_flag=1;
    // delay_50=0;
    // while(delay_flag);	


  if(delay_flag==1)
  {   

  oled_show(2);
      
    HAL_ADC_Start(&hadc1); 
    HAL_ADC_PollForConversion(&hadc1, 50); 
    if(HAL_IS_BIT_SET(HAL_ADC_GetState(&hadc1), HAL_ADC_STATE_REG_EOC))
    {
      ShowDataCo.Voltage = HAL_ADC_GetValue(&hadc1)/112.5;   //获取AD值
     
    }
    ShowDataCo.Temperature=MPU_Get_Temperature()/100.0;
  }

//  OLED_ShowAllFloat(48,6,delay_flag,2,16,1);
//while(delay_flag);




 OLED_ShowNum(112,2,delay_50,1,16);


     
    // BN1_HIGH;
    // BN2_LOW;
    //  AN1_LOW;
    // AN2_HIGH;
    // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 000);
    // __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, 000);
    // mpu_dmp_get_data(&PoseData.Pitch, &PoseData.Roll, &PoseData.Yaw, &PoseData.Gyro_Balance, &PoseData.Gyro_Turn, &PoseData.Acceleration_Z); // 读取加速度、角速度、倾角



    // int as=Read_Encoder(3);
    //  int asd=Read_Encoder(4);
    // OLED_ShowNum(48,2,as,4,16);
    // OLED_ShowAllFloat(48,4,ADC_Value,4,16,0);
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
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
