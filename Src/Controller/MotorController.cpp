#include "MotorController.h"
#include <cmath>

MotorController* MotorController::instance = nullptr;

MotorController::MotorController() {}

void MotorController::init() {
    motor = Motor::GetInstance();
    encoder = Encoder::GetInstance();
    reset();
}

void MotorController::reset() { rightDistance = leftDistance = 0; }

MotorController* MotorController::GetInstance() {
    if (instance == nullptr) {
        instance = new MotorController();
        instance->init();
    }
    return instance;
}

void MotorController::RampUp(MotorControlPosition pos, uint32_t target) {
    int value;
    switch (pos) {
        case MotorControlPosition::BOTH: {
            motor->Start(MotorPosition::RIGHT, 0);
            motor->Start(MotorPosition::LEFT, 0);
            break;
        }
        case MotorControlPosition::LEFT: {
            motor->Start(MotorPosition::LEFT, 0);
            break;
        }
        case MotorControlPosition::RIGHT: {
            motor->Start(MotorPosition::RIGHT, 0);
            break;
        }
    }
    for (volatile int i = 0; i < 100; i++) {
        value = static_cast<int>(std::sin(PI / 2.0F * static_cast<float>(i) / 100.0F) * target);
        switch(pos){
            case MotorControlPosition::RIGHT:
                motor->UpdatePWM(MotorPosition::RIGHT, value);
                break;
            case MotorControlPosition::LEFT:
                motor->UpdatePWM(MotorPosition::LEFT, value);
                break;
            case MotorControlPosition::BOTH:
                motor->UpdatePWM(MotorPosition::RIGHT, value);
                motor->UpdatePWM(MotorPosition::LEFT, value);
                break;
        }
    }
}

void MotorController::RampDown(MotorControlPosition pos, uint32_t start){
    int value;
    for (volatile int i = 99; i >= 0; i--) {
        value = static_cast<int>(std::sin(PI / 2.0F * static_cast<float>(i) / 100.0F) * start);
        switch(pos){
            case MotorControlPosition::RIGHT:
                motor->UpdatePWM(MotorPosition::RIGHT, value);
                break;
            case MotorControlPosition::LEFT:
                motor->UpdatePWM(MotorPosition::LEFT, value);
                break;
            case MotorControlPosition::BOTH:
                motor->UpdatePWM(MotorPosition::RIGHT, value);
                motor->UpdatePWM(MotorPosition::LEFT, value);
                break;
        }
    }
    switch(pos){
        case MotorControlPosition::RIGHT:
            motor->Stop(MotorPosition::RIGHT);
            break;
        case MotorControlPosition::LEFT:
            motor->Stop(MotorPosition::LEFT);
            break;
        case MotorControlPosition::BOTH:
            motor->Stop(MotorPosition::LEFT);
            motor->Stop(MotorPosition::RIGHT);
            break;
    }
}

void MotorController::Straight() {
    reset();
    RampUp(MotorControlPosition::BOTH, 100);
    /*
    motor->Start(MotorPosition::RIGHT, 100);
    motor->Start(MotorPosition::LEFT, 100);
    */
    while (1) {
        bool right = false, left = false;
        if (rightDistance > WallWidth) {
            RampDown(MotorControlPosition::RIGHT,100);
            right = true;
        }
        if (leftDistance > WallWidth) {
            RampDown(MotorControlPosition::LEFT,100);
            left = true;
        }
        if (right && left) break;
    }
    reset();
}

void MotorController::Scan() {
    auto value = encoder->GetValue();
    if (value.right > 0) {
        rightDistance += static_cast<float>(std::abs(value.right)) / EncoderPulse * GearRatio *
                         WheelRadius * 2 * PI;
    } else {
        rightDistance -= static_cast<float>(std::abs(value.right)) / EncoderPulse * GearRatio *
                         WheelRadius * 2 * PI;
    }
    if (value.left > 0) {
        leftDistance += static_cast<float>(std::abs(value.left)) / EncoderPulse * GearRatio *
                        WheelRadius * 2 * PI;
    } else {
        leftDistance -= static_cast<float>(std::abs(value.left)) / EncoderPulse * GearRatio *
                        WheelRadius * 2 * PI;
    }
}

MotorCurrentDistance MotorController::GetCurrentDistance() {
    MotorCurrentDistance dis;
    dis.right = rightDistance;
    dis.left = leftDistance;
    return dis;
}