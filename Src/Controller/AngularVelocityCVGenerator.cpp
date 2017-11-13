#include "AngularVelocityCVGenerator.h"

AngularVelocityCVGenerator::AngularVelocityCVGenerator(PIDParams _params,
                                                       std::shared_ptr<StateEstimator> _estimator,
                                                       std::shared_ptr<TargetGenerator> _targeter)
    : controller(std::make_unique<PIDController>(_params)),
      estimator(_estimator),
      targeter(_targeter) {}

float AngularVelocityCVGenerator::Update() {
    return controller->Update(targeter->GetTargetVelocity().omega, estimator->GetVelocity().omega);
}