/*
 * Created on Mon Oct 02 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef LED_H_
#define LED_H_


#include <cstdint>
#include <array>
#include <utility>
#include "Types.h"
#include "stm32f1xx_hal.h"
#include "util.h"

enum class LedNumber : std::uint8_t { ONE, TWO, THREE, FOUR };

class Led {
private:
    std::array<GPIOPin, 4> gpio_pins;
    static Led* instance;
    Led();
    void init();

public:
    ~Led();
    static Led* GetInstance();
    void AllOn();
    void AllOff();
    void On(LedNumber num);
    void Off(LedNumber num);
    void OnOnly(LedNumber num);
};

#endif