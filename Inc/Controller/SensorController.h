#ifndef SENSORCONTROLLER_H_
#define SENSORCONTROLLER_H_

#include "Sensor.h"

class SensorController {
private:
    static SensorController* instance;
    SensorController();
    void init();
    std::array<uint32_t, 4> high;
    std::array<uint32_t, 4> normal;
    std::array<bool, 4> setHigh;
    Sensor* sensor;

public:
    static SensorController* GetInstance();
    void SetSensorHigh(SensorNumber num);
    void SetSensorNormal(SensorNumber num);
    uint32_t GetHighValue(SensorNumber num);
    uint32_t GetNormalValue(SensorNumber num);
    bool IsSetHighValue(SensorNumber num) { return setHigh[static_cast<int>(num)]; }
    int32_t GetDiffFromNormal(SensorNumber num);
};

#endif