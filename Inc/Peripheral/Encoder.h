/*
 * Created on Wed Oct 04 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f1xx_hal.h"

struct EncoderValue {
    uint32_t right;
    uint32_t left;
};

class Encoder {
private:
    static Encoder* instance;
    Encoder();
    void init();

    TIM_HandleTypeDef htim2, htim3;
    EncoderValue value;
public:
    static Encoder* GetInstance();
    void Start();
    void Stop();
    void Scan();
    EncoderValue GetValue();
};

#endif