#include "SpeedEstimator.h"
#include "Variables.h"

SpeedEstimator::SpeedEstimator(float _T) : T(_T), prev_enc(0, 0){};

void SpeedEstimator::Update(const EncoderVelocity& enc) {
    // 角速度の算出
    float omega_r = enc.right / EncoderPulse * GearRatio * PI * 2 / T;
    float omega_l = enc.left / EncoderPulse * GearRatio * PI * 2 / T;
    ve.v = (WheelRadius / 2 * omega_r) + (WheelRadius / 2 * omega_l);
    ve.omega = (WheelRadius / T * omega_r) - (WheelRadius / T * omega_l);
    prev_enc = enc;
}

const Velocity& SpeedEstimator::GetVelocity() const {
    return ve;
}