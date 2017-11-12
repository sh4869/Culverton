/*
 * Created on Tue Oct 03 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef SWITCH_H_
#define SWITCH_H_

#include <cstdint>
#include <array>
#include <utility>
#include "stm32f1xx_hal.h"
#include "Types.h"

enum class SwitchNumber : std::uint8_t { ONE, TWO };

class Switch {
    private:
        std::array<GPIOPin, 2> gpio_pins;
        static Switch* instance;
        Switch() = default;
        void init();
    public:
        ~Switch();
        static Switch* GetInstance();
        bool IsPressed(SwitchNumber number);

};

#endif