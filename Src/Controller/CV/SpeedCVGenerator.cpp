#include "SpeedCVGenerator.h"

/**
 * @brief コンストラクタ
 *
 * @param estimator
 * @param _params
 */
SpeedCVGenerator::SpeedCVGenerator(PIDParams _params,
                                   std::shared_ptr<StateEstimator> _estimator,
                                   std::shared_ptr<TargetGenerator> _targeter)
    : controller(std::unique_ptr<PIDController>(new PIDController(_params))),
      estimator(_estimator),
      targeter(_targeter) {}

float SpeedCVGenerator::Update() {
    return controller->Update(estimator->GetVelocity().v, targeter->GetTargetVelocity().v);
}

