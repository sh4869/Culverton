#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "Encoder.h"
#include "Motor.h"
#include "Variables.h"
#include "SensorController.h"

enum class MotorControlPosition : int { RIGHT, LEFT, BOTH };

struct MotorCurrentDistance {
    float right, left;
};

class MotorController {
private:
    static MotorController* instance;
    MotorController();
    void init();
    void reset();

    Motor* motor;
    Encoder* encoder;
    SensorController* sensorController;
    float rightDistance, leftDistance;
    uint32_t prevRight,prevLeft;
    int32_t variation_right,variation_left;

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