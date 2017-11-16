/*
 * Created on Wed Oct 04 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f1xx_hal.h"

struct EncoderValue {
    int32_t right;
    int32_t left;
    EncoderValue(int32_t _right = 0, int32_t _left = 0) : right(_right), left(_left) {}
};

struct EncoderVelocity {
    float right;
    float left;
};

class Encoder {
private:
    static uint32_t VALUE;
    static Encoder* instance;
    Encoder() = default;
    void init();

    TIM_HandleTypeDef htim2, htim3;
    EncoderValue value, lastValue;

public:
    static Encoder* GetInstance();
    void Start();
    void Stop();
    void Scan();
    const EncoderValue& GetValue();
};

#endif