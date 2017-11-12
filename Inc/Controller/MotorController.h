#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include <cstdint>

#include "Encoder.h"
#include "Motor.h"
#include "SensorController.h"
#include "Variables.h"

enum class MotorControlPosition : std::uint8_t { RIGHT, LEFT, BOTH };

struct MotorCurrentDistance {
    float right, left;
};

class MotorController {
private:
    static MotorController* instance;
    MotorController() = default;
    void init();
    void reset();
    static constexpr float getDistance(int32_t pulse) {
        return static_cast<float>(pulse) / EncoderPulse * GearRatio * WheelRadius * 2 * PI;
    }
    Motor* motor;
    Encoder* encoder;
    SensorController* sensorController;
    int32_t rightPulse, leftPulse;
    uint32_t prevRight, prevLeft;
    int32_t variation_right, variation_left;

public:
    static void GetMoveDistance();
    static MotorController* GetInstance();
    void RampUp(MotorControlPosition pos, uint32_t target);
    void RampDown(MotorControlPosition pos, uint32_t start);
    void Straight();
    void TurnRight();
    void TurnLeft();
    void Invert();
    void Scan();
    MotorCurrentDistance GetCurrentDistance();
};

#endif