#include "Timer.h"

TimerMode Timer::Mode = TimerMode::NONE;
int32_t Timer::count = 0;

void Timer::Interrupt() {
    count++;
    static BatteryMonitor* bm = BatteryMonitor::GetInstance();
    static Sensor* sensor = Sensor::GetInstance();
    static Encoder* encoder = Encoder::GetInstance();
    static std::shared_ptr<MotionController> motionCon = MotionController::GetInstance();
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
            if(motionCon != nullptr && motionCon->IsEnable()){
                motionCon->Update();
            }
            break;
        }
    }
}

const int32_t Timer::GetCount(){
    return count;
}
