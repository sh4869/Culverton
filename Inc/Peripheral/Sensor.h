/*
 * Created on Wed Oct 04 2017
 *
 * Copyright (c) 2017 Your Company
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include <cstdint>
#include <array>
#include <tuple>
#include <utility>
#include "Types.h"
#include "stm32f1xx_hal.h"
#include "util.h"

enum class SensorNumber : std::uint8_t { FRONT_RIGHT, FRONT_LEFT, SIDE_RIGHT, SIDE_LEFT };
enum class SensorLedNumber : std::uint8_t { FRONT, SIDE };
enum class AdcNumber : std::uint8_t { ONE, THREE };

class Sensor {
private:
    static Sensor* instance;

    std::array<std::pair<AdcNumber, uint32_t>, 4> sensors;
    std::array<GPIOPin, 2> leds;
    ADC_ChannelConfTypeDef sConfig1, sConfig3;
    ADC_HandleTypeDef hadc1, hadc3;
    std::array<uint32_t, 4> light_value;
    std::array<uint32_t, 4> darkness_value;
    bool enable;

    Sensor() = default;
    void init();
    void read();
    uint32_t getADCValue(std::pair<AdcNumber, uint32_t> sensor);

public:
    static Sensor* GetInstance();
    std::array<uint32_t, 4> GetValue();
    std::array<uint32_t, 4> GetLightValue() {
        return light_value;
    };
    std::array<uint32_t, 4> GetDarkValue() {
        return darkness_value;
    }
    void Scan();
    void LedOn(SensorLedNumber number);
    void LedOff(SensorLedNumber number);
};

#endif
