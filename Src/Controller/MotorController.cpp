#include "MotorController.h"

MotorController* MotorController::instance = nullptr;

MotorController::MotorController() {}

void MotorController::init() {
    motor = Motor::GetInstance();
    encoder = Encoder::GetInstance();
}

MotorController* MotorController::GetInstance() {
    if (instance == nullptr) {
        instance = new MotorController();
        instance->init();
    }
    return instance;
}

void MotorController::Straight() {
    EncoderValue current;
    EncoderValue start = encoder->GetValue();
    bool right = true, left = true;
    motor->Start(MotorPosition::RIGHT, 200);
    motor->Start(MotorPosition::LEFT, 200);
    while (1) {
        current = encoder->GetValue();
        if (current.right - start.right > 12040) {
            motor->Stop(MotorPosition::RIGHT);
            right = false;
        }
        if (current.left - start.left > 12040) {
            motor->Stop(MotorPosition::LEFT);
            left = false;
        }
        if (!right && !left) break;
    }

}