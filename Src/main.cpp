/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "gpio.h"
#include "Led.h"
#include "Switch.h"
#include "BatteryMonitor.h"
#include "Sensor.h"
#include "Encoder.h"
#include "Uart.h"
#include "stm32f1xx_hal.h"
#include "tim.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// --------------------------------------------------

void Delay(__IO uint32_t nCount) {
  for (; nCount != 0; nCount--)
    ;
}

void start_tim8_pwm(uint32_t channel, uint32_t pulse) {
  TIM_OC_InitTypeDef sConfigOC;
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = pulse;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, channel) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Start(&htim8, channel) != HAL_OK) {
    Error_Handler();
  }
}

// --- MOTOR FUNCTION ---

void Update_RightMotor_PWM(int pulse) { start_tim8_pwm(TIM_CHANNEL_2, pulse); }
void Update_LeftMotor_PWM(int pulse) { start_tim8_pwm(TIM_CHANNEL_1, pulse); }

void Stop_RightMotor_PWM() { HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2); }

void Stop_LeftMotor_PWM() { HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1); }

void Turn_Around_RightMotor() {
  HAL_GPIO_TogglePin(MOTOR_RIGHT_IN1_GPIO_Port, MOTOR_RIGHT_IN1_Pin);
  HAL_GPIO_TogglePin(MOTOR_RIGHT_IN2_GPIO_Port, MOTOR_RIGHT_IN2_Pin);
}

void Turn_Around_LeftMotor() {
  HAL_GPIO_TogglePin(MOTOR_LEFT_IN1_GPIO_Port, MOTOR_LEFT_IN1_Pin);
  HAL_GPIO_TogglePin(MOTOR_LEFT_IN2_GPIO_Port, MOTOR_LEFT_IN2_Pin);
}

void setMotorMode() {
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_GPIO_Port, MOTOR_RIGHT_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_GPIO_Port, MOTOR_RIGHT_IN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN1_GPIO_Port, MOTOR_LEFT_IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN2_GPIO_Port, MOTOR_LEFT_IN2_Pin, GPIO_PIN_RESET);
}

void unsetMotorMode() {
  // STBY -> OFF
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN1_GPIO_Port, MOTOR_RIGHT_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_RIGHT_IN2_GPIO_Port, MOTOR_RIGHT_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN1_GPIO_Port, MOTOR_LEFT_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_LEFT_IN2_GPIO_Port, MOTOR_LEFT_IN2_Pin, GPIO_PIN_RESET);
}

// ------------- ENCODER FUNCTIONS -----------------

uint32_t encoder_r, encoder_l, oldencoder_r = 0, oldencoder_l = 0;
float r_speed, l_speed;
uint32_t target_value_r = 0, target_value_l = 0;
uint32_t current_value_r = 0, current_value_l = 0;
uint32_t base_value_r = 0, base_value_l = 0;
bool is_runnning = false;
const int WALL_VALUE = 1500;
Uart *uart;
BatteryMonitor* bm;
Sensor *sensors;
Encoder *encoder;

/* USER CODE END 0 */

int main(void) {

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();
  
  /* USER CODE BEGIN 2 */
  bm = BatteryMonitor::GetInstance();
  sensors = Sensor::GetInstance();
  encoder = Encoder::GetInstance();
  uart = Uart::GetInstance();
  encoder->Start();
  // Encoder Start
  //Unset motors
  unsetMotorMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int mode = 0;
  char str[1000];
  
  Led *led = Led::GetInstance();
  Switch* sw = Switch::GetInstance();

  bool pressed = false;
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Mode Switch
    if(pressed == false && sw->IsPressed(SwitchNumber::ONE)){
      mode++;
    }
    pressed = sw->IsPressed(SwitchNumber::ONE);
    led->OnOnly(static_cast<LedNumber>(mode % 4));
    
    switch (mode % 4) {
    // Step Mode
    case 0:
      HAL_TIM_Base_Stop_IT(&htim4);
      break;
    // Sensor Mode
    case 1:{
      if (HAL_TIM_Base_GetState(&htim4) != HAL_TIM_STATE_BUSY) {
        HAL_TIM_Base_Start_IT(&htim4);
      }
      HAL_Delay(100);
      auto values = sensors->GetValue();
      uart->Transmit("test");
      sprintf(str, "SENSOR:%ld,%ld,%ld,%ld\n", values[0],values[1],values[2],values[3]);
      uart->Transmit(str);
      sprintf(str, "%ld,%ld\n", encoder->GetValue().right,encoder->GetValue().left);
      uart->Transmit(str);
      sprintf(str,"%f\n",bm->GetValue());
      uart->Transmit(str);
      break;
    }
    // RUN Mode
    case 2: {
      setMotorMode();
      Turn_Around_RightMotor();
      // Start
      if (!is_runnning) {
        HAL_Delay(3000);
        target_value_r = 150;
        target_value_l = 150;
        is_runnning = true;
      }
      /*
      float para = ((float)sensor[2] - 200.0F) / 25.0F - (float)sensor[3] / 25.0F;
      sprintf(str, "%f\n", para);
      HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
    
      target_value_r = 150 + para;
      target_value_l = 150 - para;

      // HAL_Delay(100);
      sprintf(str, "SENSOR:%ld,%ld\n", sensor->GetValue()[0], sensor->GetValue()[1]);
      HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
      sprintf(str, "TARGET:%ld,%ld\n", target_value_r, target_value_l);
      HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
      
      if (sensor[2] > WALL_VALUE && sensor[3] > WALL_VALUE) {
        mode++;
      }
      */
      break;
    }
    case 3:
      target_value_r = 0;
      target_value_l = 0;
      current_value_r = 0;
      current_value_l = 0;
      base_value_r = 0;
      base_value_l = 0;
      is_runnning = false;
      Stop_RightMotor_PWM();
      Stop_LeftMotor_PWM();
      unsetMotorMode();
      break;
    }
  }
  /* USER CODE END 3 */
}

/** System Clock Configuration
*/
void SystemClock_Config(void) {

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure the Systick interrupt time
  */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

  /**Configure the Systick
  */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == htim4.Instance) {
    bm->Scan();
    sensors->Scan();
    encoder->Scan();
    Delay(1000);
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char *file, int line) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
    number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
