#include "Timer.h"

TimerMode Timer::Mode = TimerMode::NONE;
std::shared_ptr<MotionController> Timer::motionCon = nullptr;
int32_t Timer::count = 0;

void Timer::Interrupt() {
    count++;
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
            if(motionCon != nullptr){
                motionCon->Update();
            }
            break;
        }
    }
}

const int32_t Timer::GetCount(){
    return count;
}

void Timer::SetMotionController(std::shared_ptr<MotionController> con){
    motionCon = con;
}