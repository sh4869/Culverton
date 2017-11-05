#include "SpeedEstimator.h"
#include "Variables.h"

SpeedEstimator::SpeedEstimator(float _T) : T(_T),prev_enc(0,0){};

SpeedEstimator::~SpeedEstimator() {
    delete this;
}
void SpeedEstimator::Update(const EncoderValue &enc){
    float omega_r = (enc.right - prev_enc.right) / (EncoderPulse * GearRatio * T) * PI * 2;
    float omega_l = (enc.left - prev_enc.left)  / (EncoderPulse * GearRatio * T) * PI * 2;
    prev_enc = enc;
    ve.v = (WheelRadius / 2 * omega_r) + (WheelRadius / 2 * omega_l);
    ve.omega = (WheelRadius / T * omega_r) - (WheelRadius / T * omega_l);
}

const Velocity& SpeedEstimator::GetVelocity(){
    return ve;
}