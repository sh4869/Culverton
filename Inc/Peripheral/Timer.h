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
    static int32_t count;
public:
    static void Interrupt();
    static TimerMode Mode;
    static const int32_t GetCount();
};

#endif