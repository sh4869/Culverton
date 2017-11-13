#ifndef ANGULAR_VELOCITY_CVGENERATOR_H_
#define ANGULAR_VELOCITY_CVGENERATOR_H_

#include "CVGenerator.h"
#include "PIDController.h"
#include "StateEstimator.h"
#include "TargetGenerator.h"

class AngularVelocityCVGenerator : public CVGenerator {
public:
    AngularVelocityCVGenerator(PIDParams _params,
                               std::shared_ptr<StateEstimator> _estimator,
                               std::shared_ptr<TargetGenerator> _targeter);
    ~AngularVelocityCVGenerator() = default;
    float Update() override;

private:
    std::unique_ptr<PIDController> controller;
    std::shared_ptr<StateEstimator> estimator;
    std::shared_ptr<TargetGenerator> targeter;
};

#endif