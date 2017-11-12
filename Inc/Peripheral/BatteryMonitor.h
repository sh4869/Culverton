/*
 * Created on Tue Oct 03 2017
 *
 * Copyright (c) 2017 Your sh4869
 */

#ifndef BATTERY_H_
#define BATTERY_H_

#include <utility>
#include "stm32f1xx_hal.h"

class BatteryMonitor {
private:
    static BatteryMonitor* instance;

    ADC_ChannelConfTypeDef sConfig;
    ADC_HandleTypeDef hadc2;
    static constexpr uint32_t adc_channel = ADC_CHANNEL_9;
    bool enable;
    uint32_t value;

    BatteryMonitor() = default;
    void init();
    void read();

public:
    static BatteryMonitor* GetInstance();
    void Scan();
    uint32_t GetRawValue();
    float GetValue();
};
#endif
