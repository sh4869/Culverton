#ifndef MOTORCONTROLLER_H_
#define MOTORCONTROLLER_H_

#include "Motor.h"
#include "Encoder.h"

class MotorController {
private:
    static MotorController* instance;
    MotorController();
    void init();
    Motor* motor;
    Encoder* encoder;
public:
    static void GetMoveDistance();
    static MotorController* GetInstance();
    void Straight();
    void TurnRight();
    void TurnLeft();
    void Invert();
};

#endif