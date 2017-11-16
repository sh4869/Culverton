#ifndef TIMER_H_
#define TIMER_H_


#include <cstdint>
#include <memory>

#include "BatteryMonitor.h"
#include "Encoder.h"
#include "MotionController.h"
#include "Sensor.h"
#include "stm32f1xx_hal.h"

enum class TimerMode : std::uint8_t { NONE, BATTERY, SCAN };

class Timer {
private:
    static BatteryMonitor *bm;
    static Sensor *sensor;
    static Encoder *encoder;
public:
    static void Interrupt();
    static TimerMode Mode;
};

#endif