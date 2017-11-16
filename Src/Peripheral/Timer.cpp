#include "Timer.h"

TimerMode Timer::Mode = TimerMode::NONE;
BatteryMonitor* Timer::bm = BatteryMonitor::GetInstance();
Sensor* Timer::sensor = Sensor::GetInstance();
Encoder* Timer::encoder = Encoder::GetInstance();

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
            /*
            if(motionCon != nullptr && motionCon->IsEnable()){
                motionCon->Update();
            }
            */
            break;
        }
    }
}