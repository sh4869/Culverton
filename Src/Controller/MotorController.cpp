#include "MotorController.h"
#include <cmath>
#include "Uart.h"

MotorController* MotorController::instance = nullptr;

void MotorController::init() {
    motor = Motor::GetInstance();
    encoder = Encoder::GetInstance();
    sensorController = SensorController::GetInstance();
    prevRight = prevLeft = 0;
    reset();
}

void MotorController::reset() { rightPulse = leftPulse = 0; }

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
        value = static_cast<int>(static_cast<float>(i) / 100.0F) * target;
        switch (pos) {
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

void MotorController::RampDown(MotorControlPosition pos, uint32_t start) {
    int value;
    for (volatile int i = 99; i >= 0; i--) {
        value = static_cast<int>(static_cast<float>(i) / 100.0F * start);
        switch (pos) {
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
    switch (pos) {
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
    motor->Start(MotorPosition::RIGHT, 0);
    motor->Start(MotorPosition::LEFT, 0);
    int target = 100;
    int lefttarget = target;
    char str[1000];
    for (volatile int i = 0; i < 1000; i++) {
        motor->UpdatePWM(MotorPosition::RIGHT,
                         static_cast<int>(static_cast<float>(i) / 1000.0F * target));
        motor->UpdatePWM(MotorPosition::LEFT,
                         static_cast<int>(static_cast<float>(i) / 1000.0F * lefttarget));
    }
    int rightCount = 100, leftCount = 100;
    while (1) {
        auto cte = sensorController->GetDiffFromNormal(SensorNumber::FRONT_RIGHT) -
                   sensorController->GetDiffFromNormal(SensorNumber::FRONT_LEFT);
        float pgain = static_cast<float>(cte) * 0.8F;
        if (rightCount == 100) {
            motor->UpdatePWM(MotorPosition::RIGHT, target + static_cast<int>(pgain));
        }
        if (leftCount == 100) {
            motor->UpdatePWM(MotorPosition::LEFT, lefttarget - static_cast<int>(pgain));
        }
        if (getDistance(rightPulse) > WallWidth && rightCount > 0) {
            motor->UpdatePWM(MotorPosition::RIGHT,
                             static_cast<int>(static_cast<float>(rightCount) / 100.0F * target));
            rightCount--;
        }
        if (getDistance(leftPulse) > WallWidth && leftCount > 0) {
            motor->UpdatePWM(MotorPosition::LEFT,
                             static_cast<int>(static_cast<float>(leftCount) / 100.0F) * lefttarget);
            leftCount--;
        }
        if (rightCount <= 0 && leftCount <= 0) break;
    }
    motor->Stop(MotorPosition::RIGHT);
    motor->Stop(MotorPosition::LEFT);
    reset();
}

void MotorController::TurnRight() {
    reset();
    motor->SetDirection(MotorPosition::RIGHT, MotorDirection::BACK);
    motor->SetDirection(MotorPosition::LEFT, MotorDirection::FRONT);
    int target = 100;
    for (volatile int i = 0; i < 100; i++) {
        motor->UpdatePWM(MotorPosition::RIGHT,
                         static_cast<int>(static_cast<float>(i) / 100.0F * target));
        motor->UpdatePWM(MotorPosition::LEFT,
                         static_cast<int>(static_cast<float>(i) / 100.0F * target));
    }
    int rightCount = 100, leftCount = 100;
    while (1) {
        if (std::abs(getDistance(rightPulse)) > WheelRadius * PI && rightCount > 0) {
            motor->UpdatePWM(MotorPosition::RIGHT,
                             static_cast<int>(static_cast<float>(rightCount) / 100.0F * target));
            rightCount--;
        }
        if (getDistance(leftPulse) > WheelRadius * PI && leftCount > 0) {
            motor->UpdatePWM(MotorPosition::LEFT,
                             static_cast<int>(static_cast<float>(leftCount) / 100.0F * target));
            leftCount--;
        }
        if (rightCount <= 0 && leftCount <= 0) break;
    }
    motor->Stop(MotorPosition::RIGHT);
    motor->Stop(MotorPosition::LEFT);
}

void MotorController::Scan() {
    auto value = encoder->GetValue();
    // 変化値を保存
    variation_right = value.right - prevRight;
    variation_left = value.left - prevLeft;
    // とりあえず素直にpulseを追加する形で
    rightPulse += value.right;
    leftPulse += value.left;
    // 過去の値を追加
    prevRight = value.right;
    prevLeft = value.left;
}

MotorCurrentDistance MotorController::GetCurrentDistance() {
    MotorCurrentDistance dis;
    dis.right = rightPulse;
    dis.left = leftPulse;
    return dis;
}