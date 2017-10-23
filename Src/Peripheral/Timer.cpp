#include "Timer.h"

BatteryMonitor* Timer::bm = BatteryMonitor::GetInstance();
Sensor* Timer::sensor = Sensor::GetInstance();
Encoder* Timer::encoder = Encoder::GetInstance();
MotorController* Timer::motorController = MotorController::GetInstance();
TimerMode Timer::Mode = TimerMode::NONE;

void Timer::Interrupt() {
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
            motorController->Scan();
            break;
        }
    }
}