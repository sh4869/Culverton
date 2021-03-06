#include "StateEstimator.h"
#include "Velocity.h"
#include "Logger.h"

StateEstimator::StateEstimator(float _T)
    : T(_T),
      speedestimator(std::unique_ptr<SpeedEstimator>(new SpeedEstimator(_T))),
      odometry(std::unique_ptr<Odometry>(new Odometry(_T))) {
    encoder = Encoder::GetInstance();
}

void StateEstimator::Update() {
    auto e = encoder->GetVelocity();
    Log::Encoder::Push(e);
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
