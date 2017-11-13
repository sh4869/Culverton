#ifndef SPEED_CV_GENERATOR_H_
#define SPEED_CV_GENERATOR_H_

#include <memory>
#include "CVGenerator.h"
#include "PIDController.h"
#include "StateEstimator.h"
#include "TargetGenerator.h"
#include "Velocity.h"

class SpeedCVGenerator : public CVGenerator {
public:
    SpeedCVGenerator(PIDParams _params,std::shared_ptr<StateEstimator> _estimator,std::shared_ptr<TargetGenerator> _targeter);
    ~SpeedCVGenerator() = default;
    float Update() override;
private:
    std::unique_ptr<PIDController> controller;
    std::shared_ptr<StateEstimator> estimator;
    std::shared_ptr<TargetGenerator> targeter;
    bool enabled;
};

#endif