#include "Timer.h"
#include "BatteryMonitor.h"
#include "Encoder.h"
#include "Sensor.h"

BatteryMonitor* Timer::bm = BatteryMonitor::GetInstance();
Sensor* Timer::sensor = Sensor::GetInstance();
Encoder* Timer::encoder = Encoder::GetInstance();
TimerMode Timer::Mode = TimerMode::NONE;

void Timer::Interrupt() {
    switch (Timer::Mode) {
        case TimerMode::NONE: {
            break;
        }
        case TimerMode::SCAN: {
            bm->Scan();
            sensor->Scan();
            encoder->Scan();
            break;
        }
    }
}