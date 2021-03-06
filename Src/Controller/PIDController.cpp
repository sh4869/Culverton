#include "PIDController.h"

PIDController::PIDController() {}

/**
 * @brief コンストラクタ
 *
 * @param _params
 */
PIDController::PIDController(const PIDParams& _param) : param(_param) {}

/**
 * @brief デコンストラクタ
 */
PIDController::~PIDController() {}

/**
 * @brief
 *
 * @param measured
 * @param reference
 * @return float
 */
float PIDController::Update(float measured, float reference) {
    const float error = reference - measured;
    if (is_first) {
        prev_error = error;
        is_first = false;
    }
    float error_dif = (error - prev_error) / param.T;
    error_sum += error * param.T;
    prev_error = error;
    return param.kp * error + param.kd * error_dif + param.ki * error_sum + param.kr * reference;
}

/**
 * @brief Reset Params
 *
 */
void PIDController::Reset() {
    is_first = true;
    prev_error = 0.0f;
    error_sum = 0.0f;
}

void PIDController::SetParams(const PIDParams& _param) {
    param = _param;
}

/**
 * @brief Return PIDParams
 *
 * @return const PIDParams&
 */
const PIDParams& PIDController::GetParams() {
    return param;
}