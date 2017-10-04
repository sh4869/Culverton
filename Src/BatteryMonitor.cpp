#include "BatteryMonitor.h"

#define BATTERY_Pin GPIO_PIN_1
#define BATTERY_GPIO_Port GPIOB

BatteryMonitor* BatteryMonitor::instance = nullptr;

BatteryMonitor::BatteryMonitor() {}

void BatteryMonitor::init() {
    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc2) != HAL_OK) {
        return;
    }
    sConfig.Channel = adc_channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
        return;
    }

    __HAL_RCC_ADC2_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = BATTERY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(BATTERY_GPIO_Port, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
    enable = true;
}

BatteryMonitor* BatteryMonitor::GetInstance() {
    if (instance == nullptr) {
        instance = new BatteryMonitor();
        instance->init();
    }
    return instance;
}

void BatteryMonitor::read() {
    if (enable) {
        sConfig.Channel = adc_channel;
        sConfig.Rank = 1;
        sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
        if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK) {
            return;
        }
        HAL_ADC_Start_IT(&hadc2);
        value = HAL_ADC_GetValue(&hadc2);
    }
}
float BatteryMonitor::CheckBattery() {
    read();
    return static_cast<float>(value) / 4095.0f * 3.3f * (22.0f + 10.0f) / 10.0f;
}