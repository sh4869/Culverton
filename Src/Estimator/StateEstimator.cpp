#include "StateEstimator.h"
#include "Velocity.h"

StateEstimator::StateEstimator(float _T)
    : T(_T),
      speedestimator(std::make_shared<SpeedEstimator>(_T)),
      odometry(std::make_shared<Odometry>(_T)) {
    encoder = Encoder::GetInstance();
}

void StateEstimator::Update() {
    auto e = encoder->GetValue();
    speedestimator->Update(e);
    odometry->Update(speedestimator->GetVelocity());
}

/**
 * @brief 現在速度を返します
 * @return Velocity&
 */
const Velocity& StateEstimator::GetVelocity() {
    return speedestimator->GetVelocity();
}

const Position& StateEstimator::GetPositon() {
    return odometry->GetPosition();
}
