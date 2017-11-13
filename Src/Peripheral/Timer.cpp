#include "Timer.h"

TimerMode Timer::Mode = TimerMode::NONE;

void Timer::Interrupt() {
    static BatteryMonitor* bm = BatteryMonitor::GetInstance();
    static Sensor* sensor = Sensor::GetInstance();
    static Encoder* encoder = Encoder::GetInstance();
    switch (Timer::Mode) {
        case TimerMode::NONE: {
            break;
        }
        case TimerMode::BATTERY: {
            bm->Scan();
            break;
        }
        case TimerMode::SCAN: {
            sensor->Scan();
            encoder->Scan();
            break;
        }
    }
}
