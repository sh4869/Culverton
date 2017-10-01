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
  * Redistribution and use in source and binary forms, with or without
  *modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright
  *notice,
  *      this list of conditions and the following disclaimer in the
  *documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its
  *contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  *ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
  *LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  *USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

// -------------------------------------------------
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch) { HAL_UART_Transmit(&huart1, &ch, 1, 1); }
// --------------------------------------------------

void Delay(__IO uint32_t nCount) {
  for (; nCount != 0; nCount--)
    ;
}

uint32_t GetADC1(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = channel;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    return 0xffff;
  }
  uint32_t adcValue;
  HAL_ADC_Start(&hadc1);
  while (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    ;
  adcValue = HAL_ADC_GetValue(&hadc1);
  return adcValue;
}

uint32_t GetADC2(uint32_t channel) {
  ADC_ChannelConfTypeDef sConfig;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.Channel = channel;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    return 0;
  }
  uint32_t adcValue;
  HAL_ADC_Start(&hadc2);
  while (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK)
    ;
  adcValue = HAL_ADC_GetValue(&hadc2);
  return adcValue;
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

uint32_t GetRightMotorEncoderValue() { return TIM2->CNT; }

uint32_t GetLeftMotorEncoderValue() { return TIM3->CNT; }

// ------------- LED Lighting ---------------------

void LightLEDforMode(int mode) {
  GPIO_TypeDef *ports[4] = {IFLED1_GPIO_Port, IFLED2_GPIO_Port, IFLED3_GPIO_Port, IFLED4_GPIO_Port};
  uint16_t pins[4] = {IFLED1_Pin, IFLED2_Pin, IFLED3_Pin, IFLED4_Pin};
  GPIO_PinState pinstates[4] = {GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET, GPIO_PIN_RESET};
  if (mode > -1 && mode < 4) {
    pinstates[mode] = GPIO_PIN_SET;
  }
  for (int i = 0; i < 4; i++) {
    HAL_GPIO_WritePin(ports[i], pins[i], pinstates[i]);
  }
}

// Global Variables
uint32_t sensor[4] = {0};
uint32_t sensor_h[4] = {0}, sensor_l[4] = {0};
uint32_t encoder_r, encoder_l, oldencoder_r = 0, oldencoder_l = 0;
float r_speed, l_speed;
uint32_t target_value_r = 0, target_value_l = 0;
uint32_t current_value_r = 0, current_value_l = 0;
uint32_t base_value_r = 0, base_value_l = 0;
bool is_runnning = false;
uint32_t battery_value;
const int WALL_VALUE = 1500;
/* USER CODE END 0 */

int main(void) {
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU
   * Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
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
  MX_USART1_UART_Init();
  MX_TIM8_Init();
  MX_TIM3_Init();
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();

  /* USER CODE BEGIN 2 */
  // Encoder Start
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
    Error_Handler();
  }
  // Unset motors
  unsetMotorMode();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  GPIO_PinState oldState = GPIO_PIN_RESET, state;
  int mode = 0;
  char str[1000];
  while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    // Mode Switch
    state = HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin);
    if (state == GPIO_PIN_SET && oldState == GPIO_PIN_RESET) {
      mode++;
    }
    oldState = state;

    LightLEDforMode(mode % 4);
    switch (mode % 4) {
      // Step Mode
      case 0:
        HAL_TIM_Base_Stop_IT(&htim4);
        break;
      // Sensor Mode
      case 1:
        if (HAL_TIM_Base_GetState(&htim4) != HAL_TIM_STATE_BUSY) {
          HAL_TIM_Base_Start_IT(&htim4);
        }
        HAL_Delay(100);
        sprintf(str, "%ld,%ld\n", sensor[2], sensor[3]);
        HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
        break;
      // RUN Mode
      case 2:
        setMotorMode();
        Turn_Around_RightMotor();
        // Start
        if (!is_runnning) {
          HAL_Delay(3000);
          target_value_r = 150;
          target_value_l = 150;
          is_runnning = true;
        }
        //float para = (float)(sensor[2] - 200 - sensor[3]) /10.0F;
        float para = ((float)sensor[2] - 200.0F) / 25.0F - (float)sensor[3] / 25.0F;
        sprintf(str, "%f\n", para);
        HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);

        target_value_r = 150 + para;
        target_value_l = 150 - para;
        /*
        sprintf(str,"%ld\n",sensor[3] - sensor[2]);
        HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
        */
      
        //HAL_Delay(100);
        sprintf(str, "SENSOR:%ld,%ld\n", sensor[2], sensor[3]);
        HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
        sprintf(str, "TARGET:%ld,%ld\n", target_value_r, target_value_l);
        HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
        /*
        sprintf(str, "%ld,%ld\n", encoder_r, encoder_l);
        HAL_UART_Transmit(&huart1, (uint8_t *)str, strlen(str), -1);
        */
        if (sensor[2] > WALL_VALUE && sensor[3] > WALL_VALUE) {
          mode++;
        }
        break;
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

/* ADC1 init function */
static void MX_ADC1_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  /**Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* ADC2 init function */
static void MX_ADC2_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  /**Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* ADC3 init function */
static void MX_ADC3_Init(void) {
  ADC_ChannelConfTypeDef sConfig;

  /**Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  /**Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 init function */
static void MX_TIM2_Init(void) {
  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65534;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM3 init function */
static void MX_TIM3_Init(void) {
  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65534;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM4 init function */
static void MX_TIM4_Init(void) {
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 63;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM8 init function */
static void MX_TIM8_Init(void) {
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 63;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim8);
}

/* USART1 init function */
static void MX_USART1_UART_Init(void) {
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK) {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | MOTOR_STBY_Pin | MOTOR_RIGHT_IN2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IFLED4_Pin | IFLED3_Pin | MOTOR_LEFT_IN1_Pin | MOTOR_LEFT_IN2_Pin | MOTOR_RIGHT_IN1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, IFLED2_Pin | IFLED1_Pin | SENSORLED1_Pin | SENSORLED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA2 MOTOR_STBY_Pin MOTOR_RIGHT_IN2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2 | MOTOR_STBY_Pin | MOTOR_RIGHT_IN2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : IFLED4_Pin IFLED3_Pin MOTOR_LEFT_IN1_Pin
     MOTOR_LEFT_IN2_Pin
                           MOTOR_RIGHT_IN1_Pin */
  GPIO_InitStruct.Pin = IFLED4_Pin | IFLED3_Pin | MOTOR_LEFT_IN1_Pin | MOTOR_LEFT_IN2_Pin | MOTOR_RIGHT_IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IFLED2_Pin IFLED1_Pin SENSORLED1_Pin SENSORLED2_Pin */
  GPIO_InitStruct.Pin = IFLED2_Pin | IFLED1_Pin | SENSORLED1_Pin | SENSORLED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == htim4.Instance) {
    // outside Sensors
    HAL_GPIO_WritePin(SENSORLED2_GPIO_Port, SENSORLED2_Pin, GPIO_PIN_SET);
    Delay(1000);
    sensor_h[2] = GetADC1(ADC_CHANNEL_12);
    Delay(1000);
    sensor_h[3] = GetADC2(ADC_CHANNEL_13);
    HAL_GPIO_WritePin(SENSORLED2_GPIO_Port, SENSORLED2_Pin, GPIO_PIN_RESET);
    Delay(1000);
    sensor_l[2] = GetADC1(ADC_CHANNEL_12);
    Delay(1000);
    sensor_l[3] = GetADC2(ADC_CHANNEL_13);

    // Center Sensors
    HAL_GPIO_WritePin(SENSORLED1_GPIO_Port, SENSORLED1_Pin, GPIO_PIN_SET);
    Delay(1000);
    sensor_h[0] = GetADC1(ADC_CHANNEL_10);
    Delay(1000);
    sensor_h[1] = GetADC1(ADC_CHANNEL_11);
    HAL_GPIO_WritePin(SENSORLED1_GPIO_Port, SENSORLED1_Pin, GPIO_PIN_RESET);
    Delay(1000);
    sensor_l[0] = GetADC1(ADC_CHANNEL_10);
    Delay(1000);
    sensor_l[1] = GetADC1(ADC_CHANNEL_11);
    for (int i = 0; i < 4; i++) {
      sensor[i] = sensor_h[i] - sensor_l[i];
    }
    encoder_r = GetRightMotorEncoderValue();
    encoder_l = GetLeftMotorEncoderValue();
    if (target_value_r > current_value_r) {
      current_value_r = current_value_r + (target_value_r - base_value_r) / 5;
      Update_RightMotor_PWM(target_value_r);
    } else {
      base_value_r = current_value_r;
    }
    if (target_value_l > current_value_l) {
      current_value_l = current_value_l + (target_value_l - base_value_l) / 5;
      Update_LeftMotor_PWM(target_value_l);
    } else {
      base_value_l = current_value_l;
    }
    /*
    battery_value = GetADC2(ADC_CHANNEL_9);
    Delay(1000);
    */
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
