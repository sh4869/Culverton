/*
 * Created on Wed Oct 04 2017
 *
 * Copyright (c) 2017 sh4869
 */

#ifndef ENCODER_H_
#define ENCODER_H_

#include "stm32f1xx_hal.h"

static constexpr float GearRatio = 42.0F / 8.0F;
static constexpr float WheelRadius = 27.0F / 2.0F;
static constexpr uint32_t EncoderPulse = 1024;
static constexpr float PI = 3.14159265258979F;

struct EncoderValue {
    uint32_t right;
    uint32_t left;
};

struct EncoderVelocity {
    float right;
    float left;
};

class Encoder {
private:
    static uint32_t VALUE;
    static Encoder* instance;
    Encoder();
    void init();

    TIM_HandleTypeDef htim2, htim3;
    EncoderValue value,lastValue;
    EncoderVelocity velocity;
public:
    static Encoder* GetInstance();
    void Start();
    void Stop();
    void Scan();
    EncoderValue GetValue();
    EncoderVelocity GetVelocity();
};

#endif