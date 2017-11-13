#ifndef WALLCVGENERATOR_H_
#define WALLCVGENERATOR_H_

#include "CVGenerator.h"
#include "PIDController.h"
#include "SensorController.h"

class WallCVGenerator : public CVGenerator {
public:
    enum class WallCVStatus { RIGHT_AND_LEFT, RIGHT, LEFT, NONE };
    WallCVGenerator(PIDParams _params);
    ~WallCVGenerator() = default;
    float Update() override;

private:
    std::unique_ptr<PIDController> pidController;
    SensorController* sensorController;
    WallCVStatus GetStatus();
};

#endif