#include "WallCVGenerator.h"

WallCVGenerator::WallCVGenerator(PIDParams _params)
    : pidController(std::unique_ptr<PIDController>(new PIDController(_params))),
      sensorController(SensorController::GetInstance()) {}

// 右の値が大きいほど(つまり壁に近いほど)大きな値が返るようになっている
float WallCVGenerator::Update() {
    switch (GetStatus()) {
        case WallCVStatus::RIGHT_AND_LEFT:
            return pidController->Update(
                    static_cast<float>(
                            sensorController->GetDiffFromNormal(SensorNumber::FRONT_RIGHT) -
                            sensorController->GetDiffFromNormal(SensorNumber::FRONT_LEFT)),
                    0);
        case WallCVStatus::RIGHT:
            return pidController->Update(
                    static_cast<float>(
                            sensorController->GetDiffFromNormal(SensorNumber::FRONT_RIGHT) * 2),
                    0);
        case WallCVStatus::LEFT:
            return pidController->Update(
                    static_cast<float>(
                            -sensorController->GetDiffFromNormal(SensorNumber::FRONT_LEFT) * 2),
                    0);
        case WallCVStatus::NONE:
            return 0.0f;
        default:
            return 0.0f;
    }
}

WallCVGenerator::WallCVStatus WallCVGenerator::GetStatus() {
    // TODO : この辺の実装(多分SensorControllerも同時に実装しなおさないと無理)
    return WallCVStatus::RIGHT_AND_LEFT;
}