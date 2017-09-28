/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define SENSOR1_Pin GPIO_PIN_0
#define SENSOR1_GPIO_Port GPIOC
#define SENSOR2_Pin GPIO_PIN_1
#define SENSOR2_GPIO_Port GPIOC
#define SENSOR3_Pin GPIO_PIN_2
#define SENSOR3_GPIO_Port GPIOC
#define SENSOR4_Pin GPIO_PIN_3
#define SENSOR4_GPIO_Port GPIOC
#define ENCODER_RIGHT1_Pin GPIO_PIN_0
#define ENCODER_RIGHT1_GPIO_Port GPIOA
#define ENCODER_RIGHT2_Pin GPIO_PIN_1
#define ENCODER_RIGHT2_GPIO_Port GPIOA
#define ENCODER_LEFT2_Pin GPIO_PIN_6
#define ENCODER_LEFT2_GPIO_Port GPIOA
#define ENCODER_LEFT1_Pin GPIO_PIN_7
#define ENCODER_LEFT1_GPIO_Port GPIOA
#define IFLED4_Pin GPIO_PIN_4
#define IFLED4_GPIO_Port GPIOC
#define IFLED3_Pin GPIO_PIN_5
#define IFLED3_GPIO_Port GPIOC
#define IFLED2_Pin GPIO_PIN_0
#define IFLED2_GPIO_Port GPIOB
#define BATTERY_Pin GPIO_PIN_1
#define BATTERY_GPIO_Port GPIOB
#define IFLED1_Pin GPIO_PIN_10
#define IFLED1_GPIO_Port GPIOB
#define MOTOR_LEFT_PWM_Pin GPIO_PIN_6
#define MOTOR_LEFT_PWM_GPIO_Port GPIOC
#define MOTOR_RIGHT_PWM_Pin GPIO_PIN_7
#define MOTOR_RIGHT_PWM_GPIO_Port GPIOC
#define MOTOR_LEFT_IN1_Pin GPIO_PIN_8
#define MOTOR_LEFT_IN1_GPIO_Port GPIOC
#define MOTOR_LEFT_IN2_Pin GPIO_PIN_9
#define MOTOR_LEFT_IN2_GPIO_Port GPIOC
#define MOTOR_STBY_Pin GPIO_PIN_11
#define MOTOR_STBY_GPIO_Port GPIOA
#define MOTOR_RIGHT_IN2_Pin GPIO_PIN_12
#define MOTOR_RIGHT_IN2_GPIO_Port GPIOA
#define SW2_Pin GPIO_PIN_10
#define SW2_GPIO_Port GPIOC
#define MOTOR_RIGHT_IN1_Pin GPIO_PIN_11
#define MOTOR_RIGHT_IN1_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_5
#define SW1_GPIO_Port GPIOB
#define SENSORLED1_Pin GPIO_PIN_6
#define SENSORLED1_GPIO_Port GPIOB
#define SENSORLED2_Pin GPIO_PIN_7
#define SENSORLED2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
