/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"


//C语言库相关头文件
#include "stdio.h"
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>

//MPU6050DMP库相关
// #include "inv_mpu.h"
// #include "inv_mpu_dmp_motion_driver.h"
// #include "dmpKey.h"
// #include "dmpmap.h"
// #include "MPU6050.h"
#include "filter.h"


#include "../Include/HardwareDriver/i2c.h"
#include "../Include/HardwareDriver/gpio.h"
#include "../include/OLED/oled.h"
#include "../Include/HardwareDriver/tim.h"
#include "control.h"
#include "../Include/HardwareDriver/adc.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Delay.h"
#include "Print.h"
#include "../include/MPU6050/mpu6050.h"
#include "../include/OLED/oledshow.h"

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;


struct ShowData
{
    float Angle_Balance;  //平衡角
    float AC_Angle_Balance;  //平衡角速度
    float AX_Angle_Balance;  //平衡角速度
    float AY_Angle_Balance;  //平衡角速度
    float Voltage;        //电压
    float LeftWheel_Velocity;  //左轮速度
    float RightWheel_Velocity; //右轮速度
    float Temperature;          //温度
    float LeftWheel_Position;  //左轮位置
    float RightWheel_Position; //右轮位置
}ShowDataCo ;

void Error_Handler(void);
extern uint8_t delay_50,delay_flag; 						//延时和调参相关变量
/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define STBY_Pin GPIO_PIN_0
#define STBY_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_12
#define KEY1_GPIO_Port GPIOB
#define KEY1_EXTI_IRQn EXTI15_10_IRQn
#define KEY2_Pin GPIO_PIN_13
#define KEY2_GPIO_Port GPIOB
#define KEY2_EXTI_IRQn EXTI15_10_IRQn
#define KEY3_Pin GPIO_PIN_14
#define KEY3_GPIO_Port GPIOB
#define KEY3_EXTI_IRQn EXTI15_10_IRQn
#define CO_Pin GPIO_PIN_15
#define CO_GPIO_Port GPIOB
#define CO_EXTI_IRQn EXTI15_10_IRQn
#define LED1_Pin GPIO_PIN_4
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
