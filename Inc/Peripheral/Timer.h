#ifndef TIMER_H_
#define TIMER_H_

#include "BatteryMonitor.h"
#include "Encoder.h"
#include "Sensor.h"
#include "stm32f1xx_hal.h"
#include "MotorController.h"

enum class TimerMode : int { NONE, SCAN };

class Timer {
private:
    static BatteryMonitor *bm;
    static Sensor *sensor;
    static Encoder *encoder;
    static MotorController *motorController;
public:
    static void Interrupt();
    static TimerMode Mode;
};

#endif