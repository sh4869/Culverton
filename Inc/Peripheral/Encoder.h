/*
 * Created on Wed Oct 04 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include <array>
#include "stm32f1xx_hal.h"

struct EncoderValue {
    int32_t right;
    int32_t left;
    EncoderValue(int32_t _right = 0, int32_t _left = 0) : right(_right), left(_left) {}
};

struct EncoderVelocity {
    float right;
    float left;
    EncoderVelocity(float _right = 0, float _left = 0) : right(_right), left(_left) {}
};

class Encoder {
private:
    static uint32_t VALUE;
    static Encoder* instance;
    Encoder() = default;
    void init();

    TIM_HandleTypeDef htim2, htim3;
    EncoderValue value, lastValue;
    static constexpr int historysize = 10;
    std::array<EncoderValue, historysize> history;
    int historyindex = 0;
    EncoderVelocity velocity;

public:
    static Encoder* GetInstance();
    void Start();
    void Stop();
    void Scan();
    const EncoderValue& GetValue();
    const EncoderVelocity& GetVelocity();
};

#endif