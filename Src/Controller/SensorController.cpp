#include "SensorController.h"
#include <cmath>

SensorController* SensorController::instance = nullptr;

void SensorController::init(){
    sensor = Sensor::GetInstance();
    high = {};
    normal = {};
    setHigh = {false};
}

SensorController* SensorController::GetInstance(){
    if(instance == nullptr){
        instance = new SensorController();
        instance->init();
    }
    return instance;
}

void SensorController::SetSensorHigh(SensorNumber num){
    auto value = sensor->GetValue();
    high[static_cast<int>(num)] = value[static_cast<int>(num)];
    setHigh[static_cast<int>(num)] = true;
}

void SensorController::SetSensorNormal(SensorNumber num){
    auto value = sensor->GetValue();
    normal[static_cast<int>(num)] = value[static_cast<int>(num)];
}

uint32_t SensorController::GetHighValue(SensorNumber num){
    return high[static_cast<int>(num)];
}

uint32_t SensorController::GetNormalValue(SensorNumber num){
    return normal[static_cast<int>(num)];
}

int32_t SensorController::GetDiffFromNormal(SensorNumber num){
    auto value = sensor->GetValue();
    return std::sqrt(value[static_cast<int>(num)]) - std::sqrt(GetNormalValue(num));
}