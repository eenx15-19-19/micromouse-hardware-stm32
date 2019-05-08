/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOC
#define Sensor_Left_Pin GPIO_PIN_2
#define Sensor_Left_GPIO_Port GPIOA
#define Sensor_Middle_Pin GPIO_PIN_3
#define Sensor_Middle_GPIO_Port GPIOA
#define Sensor_Right_Pin GPIO_PIN_4
#define Sensor_Right_GPIO_Port GPIOA
#define AIN2_Pin GPIO_PIN_5
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_6
#define AIN1_GPIO_Port GPIOA
#define PWMA_Pin GPIO_PIN_7
#define PWMA_GPIO_Port GPIOA
#define PWMB_Pin GPIO_PIN_0
#define PWMB_GPIO_Port GPIOB
#define Btn_Back_Pin GPIO_PIN_8
#define Btn_Back_GPIO_Port GPIOA
#define Btn_Back_EXTI_IRQn EXTI9_5_IRQn
#define Btn_Front_Pin GPIO_PIN_9
#define Btn_Front_GPIO_Port GPIOA
#define Btn_Front_EXTI_IRQn EXTI9_5_IRQn
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_4
#define BIN2_GPIO_Port GPIOB
#define BIN1_Pin GPIO_PIN_5
#define BIN1_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern TIM_HandleTypeDef htim1; 
extern TIM_HandleTypeDef htim2; 
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;	

extern UART_HandleTypeDef huart3;

// To configure adc channel.
extern ADC_ChannelConfTypeDef sConfig;

extern volatile int go;
extern volatile int rot;
extern volatile int enableControlLoop;
extern uint32_t sensorValues[];

// Single command to execute while searching the maze
extern uint8_t restart[];
extern uint8_t singleCommand[2];

void move(int nrOfCells);
void rotate(int degrees); // Positive is counter clockwise, negative is clockwise
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
