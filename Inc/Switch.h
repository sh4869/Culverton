/*
 * Created on Tue Oct 03 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef SWITCH_H_
#define SWITCH_H_

#include "stm32f1xx_hal.h"
#include <array>
#include <utility>

enum class SwitchNumber : int { ONE, TWO };

class Switch {
    private:
        std::array<std::pair<GPIO_TypeDef*, uint16_t>, 2> gpio_pins;
        static Switch* instance;
        Switch();
        void init();
    public:
        ~Switch();
        static Switch* getInstance();
        bool isPressed(SwitchNumber number);

};

#endif