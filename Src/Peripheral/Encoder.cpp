
#include <limits>
#include "Encoder.h"

// Defines

#define ENCODER_RIGHT1_Pin GPIO_PIN_0
#define ENCODER_RIGHT1_GPIO_Port GPIOA
#define ENCODER_RIGHT2_Pin GPIO_PIN_1
#define ENCODER_RIGHT2_GPIO_Port GPIOA
#define ENCODER_LEFT2_Pin GPIO_PIN_6
#define ENCODER_LEFT2_GPIO_Port GPIOA
#define ENCODER_LEFT1_Pin GPIO_PIN_7
#define ENCODER_LEFT1_GPIO_Port GPIOA

Encoder* Encoder::instance = nullptr;

void Encoder::init() {
    // GPIO Setting
    GPIO_InitTypeDef GPIO_InitStruct;
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = ENCODER_RIGHT1_Pin | ENCODER_RIGHT2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ENCODER_LEFT2_Pin | ENCODER_LEFT1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    // Encoder Settings
    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    // Right Motor Encoder Setting
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 65534;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    HAL_TIM_Encoder_Init(&htim2, &sConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

    // Left Motor Encoder Settings
    __HAL_RCC_TIM3_CLK_ENABLE();
    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 65534;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    HAL_TIM_Encoder_Init(&htim3, &sConfig);
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
    TIM2->CNT = 0;
    TIM3->CNT = 0;
}

Encoder* Encoder::GetInstance() {
    if (instance == nullptr) {
        instance = new Encoder();
        instance->init();
    }
    return instance;
}

void Encoder::Start() {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); 
    value.right = 0;
    value.left = 0;
    TIM2->CNT = 0;
    TIM3->CNT = 0;
}

void Encoder::Stop() {
    HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);
}

void Encoder::Scan() {
    const uint16_t enc_right = TIM2->CNT;
    const uint16_t enc_left = TIM3->CNT;
    TIM2->CNT = 0;
    TIM3->CNT = 0;
    // 秒速
    if(enc_right > std::numeric_limits<int16_t>::max()){
        value.right = -(int16_t)enc_right;
    } else {
        value.right = -enc_right;
    }
    if(enc_left > std::numeric_limits<int16_t>::max()){
        value.left = (int16_t)enc_left;
    } else {
        value.left = enc_left;
    }
}

const EncoderValue& Encoder::GetValue() {
    return value;
}
