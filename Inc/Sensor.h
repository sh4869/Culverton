/*
 * Created on Wed Oct 04 2017
 *
 * Copyright (c) 2017 Your Company
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <array>
#include <tuple>
#include <utility>
#include "stm32f1xx_hal.h"

enum class SensorNumber : int { FRONT_RIGHT, FRONT_LEFT, SIDE_RIGHT, SIDE_LEFT };
enum class SensorLedNumber : int { FRONT, SIDE };
enum class AdcNumber : int { ONE, THREE};

class Sensor {
private:
    static Sensor* instance;

    std::array<std::pair<AdcNumber, uint32_t>, 4> sensors;
    std::array<std::pair<GPIO_TypeDef*, uint16_t>, 2> leds;
    ADC_ChannelConfTypeDef sConfig[2];
    ADC_HandleTypeDef hadc[2];
    std::array<uint32_t, 4> light_value;
    std::array<uint32_t, 4> darkness_value;
    bool enable;

    Sensor();
    void init();
    void read();
    uint32_t getADCValue(std::pair<AdcNumber,uint32_t> sensor);

public:
    static Sensor* GetInstance();
    std::array<uint32_t, 4> GetValue();
    void Scan();
};

#endif
