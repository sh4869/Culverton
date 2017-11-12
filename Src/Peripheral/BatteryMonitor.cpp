#include "BatteryMonitor.h"

#define BATTERY_Pin GPIO_PIN_1
#define BATTERY_GPIO_Port GPIOB

BatteryMonitor* BatteryMonitor::instance = nullptr;

void BatteryMonitor::init() {
    value = 0;
    enable = false;
    __HAL_RCC_ADC2_CLK_ENABLE();

    hadc2.Instance = ADC2;
    hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc2.Init.ContinuousConvMode = DISABLE;
    hadc2.Init.DiscontinuousConvMode = DISABLE;
    hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc2.Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc2);
    sConfig.Channel = adc_channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = BATTERY_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(BATTERY_GPIO_Port, &GPIO_InitStruct);
    
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
    sConfig.Channel = adc_channel;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
    HAL_ADC_ConfigChannel(&hadc2, &sConfig);
    HAL_ADC_Start(&hadc2);
    while (HAL_ADC_PollForConversion(&hadc2, 10) != HAL_OK)
        ;
    value = HAL_ADC_GetValue(&hadc2);
}

void BatteryMonitor::Scan() {
    if (enable) read();
}

const uint32_t& BatteryMonitor::GetRawValue() const {
    return value;
}

float BatteryMonitor::GetValue() const {
    return static_cast<float>(value) / 4095.0f * 3.3f * (22.0f + 10.0f) / 10.0f;
}