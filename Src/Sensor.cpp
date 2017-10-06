#include "Sensor.h"
#include "util.h"
#include <functional>

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
    sensors = { std::make_pair(AdcNumber::THREE, ADC_CHANNEL_10),
                std::make_pair(AdcNumber::ONE, ADC_CHANNEL_11),
                std::make_pair(AdcNumber::ONE, ADC_CHANNEL_12),
                std::make_pair(AdcNumber::THREE, ADC_CHANNEL_13) };

    leds = { std::make_pair(SENSORLED1_GPIO_Port, SENSORLED1_Pin),
             std::make_pair(SENSORLED2_GPIO_Port, SENSORLED2_Pin) };

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

    // SENSOR Settings
    int i= static_cast<int>(AdcNumber::ONE);
    hadc[i].Instance = ADC1;
    hadc[i].Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc[i].Init.ContinuousConvMode = DISABLE;
    hadc[i].Init.DiscontinuousConvMode = DISABLE;
    hadc[i].Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc[i].Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc[i].Init.NbrOfConversion = 2;
    HAL_ADC_Init(&hadc[i]);
    sConfig[i].Channel = ADC_CHANNEL_11;
    sConfig[i].Rank = 1;
    sConfig[i].SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig[i]);

    i = static_cast<int>(AdcNumber::THREE);
    hadc[i].Instance = ADC3;
    hadc[i].Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc[i].Init.ContinuousConvMode = DISABLE;
    hadc[i].Init.DiscontinuousConvMode = DISABLE;
    hadc[i].Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc[i].Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc[i].Init.NbrOfConversion = 1;
    HAL_ADC_Init(&hadc[i]);
    sConfig[i].Channel = ADC_CHANNEL_10;
    sConfig[i].Rank = 1;
    sConfig[i].SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig[i]);

    GPIO_InitStruct.Pin = SENSOR1_Pin | SENSOR4_Pin | SENSOR2_Pin | SENSOR3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    enable = true;
}

uint32_t Sensor::getADCValue(
    std::pair<AdcNumber,uint32_t> sensor) {
    int i = static_cast<int>(sensor.first);
    sConfig[i].Channel = sensor.second;
    sConfig[i].Rank = 1;
    sConfig[i].SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc[i], &sConfig[i]);
    HAL_ADC_Start(&hadc[i]);
    while (HAL_ADC_PollForConversion(&hadc[i], 10) != HAL_OK)
        ;
    uint32_t adcValue = HAL_ADC_GetValue(&hadc[i]);
    return adcValue;
}

void Sensor::read() {
    HAL_GPIO_WritePin(leds[0].first, leds[0].second, GPIO_PIN_SET);
    Util::Delay(1000);
    light_value[0] = getADCValue(sensors[0]);
    Util::Delay(1000);
    light_value[1] = getADCValue(sensors[1]);
    HAL_GPIO_WritePin(leds[0].first, leds[0].second, GPIO_PIN_RESET);
    Util::Delay(1000);
    darkness_value[0] = getADCValue(sensors[0]);
    Util::Delay(1000);
    darkness_value[1] = getADCValue(sensors[1]);

    HAL_GPIO_WritePin(leds[1].first, leds[1].second, GPIO_PIN_SET);
    Util::Delay(1000);
    light_value[2] = getADCValue(sensors[2]);
    Util::Delay(1000);
    light_value[3] = getADCValue(sensors[3]);
    HAL_GPIO_WritePin(leds[1].first, leds[1].second, GPIO_PIN_RESET);
    Util::Delay(1000);
    darkness_value[2] = getADCValue(sensors[2]);
    Util::Delay(1000);
    darkness_value[3] = getADCValue(sensors[3]);
}

Sensor* Sensor::GetInstance() {
    if (instance == nullptr) {
        instance = new Sensor();
        instance->init();
    }
    return instance;
}

std::array<uint32_t, 4> Sensor::GetValue(){
    std::array<uint32_t,4> value = {
        light_value[0] - darkness_value[0],
        light_value[1] - darkness_value[1],
        light_value[2] - darkness_value[2],
        light_value[3] - darkness_value[3]
    };
    return value;
}

void Sensor::Scan() {
    if (enable) read();
}
