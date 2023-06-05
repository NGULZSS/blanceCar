/**
  ******************************************************************************
  * File Name          : gpio.h
  * Description        : This file contains all the functions prototypes for 
  *                      the gpio  
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __gpio_H
#define __gpio_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#define Red_LED_ON HAL_GPIO_WritePin(GPIOB,LED1_Pin,GPIO_PIN_RESET);
#define Red_LED_OFF HAL_GPIO_WritePin(GPIOB,LED1_Pin,GPIO_PIN_SET);
#define Green_LED_ON HAL_GPIO_WritePin(GPIOB,LED2_Pin,GPIO_PIN_RESET);
#define Green_LED_OFF HAL_GPIO_WritePin(GPIOB,LED2_Pin,GPIO_PIN_SET);

#define AN1_LOW HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_RESET);
#define AN1_HIGH HAL_GPIO_WritePin(GPIOA,AIN1_Pin,GPIO_PIN_SET);
#define AN2_LOW HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_RESET);
#define AN2_HIGH HAL_GPIO_WritePin(GPIOA,AIN2_Pin,GPIO_PIN_SET);
#define BN1_LOW HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_RESET);
#define BN1_HIGH HAL_GPIO_WritePin(GPIOA,BIN1_Pin,GPIO_PIN_SET);
#define BN2_LOW HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_RESET);
#define BN2_HIGH HAL_GPIO_WritePin(GPIOA,BIN2_Pin,GPIO_PIN_SET);

#define STBY_LOW HAL_GPIO_WritePin(GPIOB,STBY_Pin,GPIO_PIN_RESET);
#define STBY_HIGH HAL_GPIO_WritePin(GPIOB,STBY_Pin,GPIO_PIN_SET);
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_GPIO_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ pinoutConfig_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
