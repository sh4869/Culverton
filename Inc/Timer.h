#ifndef TIMER_H_
#define TIMER_H_

#include "stm32f1xx_hal.h"

class Timer {
private:
    static Timer* instance;
    TIM_HandleTypeDef htim4;
    Timer();
    void init();
public:
    static Timer* GetInstance();
    void Start();
    void Stop();
};

#endif