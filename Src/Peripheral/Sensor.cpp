#include "Sensor.h"
// Defines
#define SENSOR1_Pin GPIO_PIN_0
#define SENSOR1_GPIO_Port GPIOC
#define SENSOR2_Pin GPIO_PIN_1
#define SENSOR2_GPIO_Port GPIOC
#define SENSOR3_Pin GPIO_PIN_2
#define SENSOR3_GPIO_Port GPIOC
#define SENSOR4_Pin GPIO_PIN_3
#define SENSOR4_GPIO_Port GPIOC
#define SENSORLED1_Pin GPIO_PIN_6
#define SENSORLED1_GPIO_Port GPIOB
#define SENSORLED2_Pin GPIO_PIN_7
#define SENSORLED2_GPIO_Port GPIOB

Sensor* Sensor::instance = nullptr;

Sensor::Sensor() {}

void Sensor::init() {
    sensors = { std::make_pair(AdcNumber::ONE, ADC_CHANNEL_10),
                std::make_pair(AdcNumber::THREE, ADC_CHANNEL_11),
                std::make_pair(AdcNumber::ONE, ADC_CHANNEL_12),
                std::make_pair(AdcNumber::THREE, ADC_CHANNEL_13) };

    leds = { GPIOPinPair(SENSORLED1_GPIO_Port, SENSORLED1_Pin),
             GPIOPinPair(SENSORLED2_GPIO_Port, SENSORLED2_Pin) };

    enable = false;

    __HAL_RCC_ADC1_CLK_ENABLE();
    __HAL_RCC_ADC3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // SENSORLED Settings
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pin = SENSORLED1_Pin | SENSORLED2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // SENSOR Setting
    GPIO_InitStruct.Pin = SENSOR1_Pin | SENSOR4_Pin | SENSOR2_Pin | SENSOR3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    // SENSOR Settings
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 2;
    HAL_ADC_Init(&hadc1);
    sConfig1.Channel = ADC_CHANNEL_11;
    sConfig1.Rank = 1;
    sConfig1.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig1);

    hadc3.Instance = ADC3;
    hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc3.Init.ContinuousConvMode = DISABLE;
    hadc3.Init.DiscontinuousConvMode = DISABLE;
    hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc3.Init.NbrOfConversion = 2;
    HAL_ADC_Init(&hadc3);
    sConfig3.Channel = ADC_CHANNEL_10;
    sConfig3.Rank = 1;
    sConfig3.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc3, &sConfig3);
    enable = true;
}

uint32_t Sensor::getADCValue(std::pair<AdcNumber, uint32_t> sensor) {
    ADC_HandleTypeDef hadc;
    ADC_ChannelConfTypeDef sConfig;
    switch (sensor.first) {
        case AdcNumber::ONE:
            hadc = hadc1;
            sConfig = sConfig1;
            break;
        case AdcNumber::THREE:
            hadc = hadc3;
            sConfig = sConfig3;
            break;
    }
    sConfig.Channel = sensor.second;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);
    HAL_ADC_Start(&hadc);
    while (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK)
        ;
    uint32_t adcValue = HAL_ADC_GetValue(&hadc);
    return adcValue;
}

void Sensor::read() {
    
    HAL_GPIO_WritePin(leds[0].first, leds[0].second, GPIO_PIN_SET);
    HAL_GPIO_WritePin(leds[1].first, leds[1].second, GPIO_PIN_SET);
    Util::Delay(100);
    light_value[0] = getADCValue(sensors[0]);
    light_value[1] = getADCValue(sensors[1]);
    light_value[2] = getADCValue(sensors[2]);
    light_value[3] = getADCValue(sensors[3]);
    
    HAL_GPIO_WritePin(leds[0].first, leds[0].second, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(leds[1].first, leds[1].second, GPIO_PIN_RESET);
    Util::Delay(100);
    darkness_value[0] = getADCValue(sensors[0]);
    darkness_value[1] = getADCValue(sensors[1]);
    darkness_value[2] = getADCValue(sensors[2]);
    darkness_value[3] = getADCValue(sensors[3]);
}

Sensor* Sensor::GetInstance() {
    if (instance == nullptr) {
        instance = new Sensor();
        instance->init();
    }
    return instance;
}

std::array<uint32_t, 4> Sensor::GetValue() {
    std::array<uint32_t, 4> value = { light_value[0] - darkness_value[0],
                                      light_value[1] - darkness_value[1],
                                      light_value[2] - darkness_value[2],
                                      light_value[3] - darkness_value[3] };
    return value;
}

void Sensor::Scan() {
    if (enable) read();
}
