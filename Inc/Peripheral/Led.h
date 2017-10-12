/*
 * Created on Mon Oct 02 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef LED_H_
#define LED_H_

#include <array>
#include <utility>
#include "stm32f1xx_hal.h"
#include "util.h"
#include "Types.h"

enum class LedNumber : int { ONE, TWO, THREE, FOUR };

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