#include "MotionController.h"
#include "Logger.h"

std::shared_ptr<MotionController> MotionController::instance = nullptr;
Motor* MotionController::motor = Motor::GetInstance();

MotionController::MotionController()
    : estimator(std::make_shared<StateEstimator>(T)),
      targeter(std::make_shared<TargetGenerator>()),
      enable(false) {
    // TODO : 何も考えてない
    PIDParams p(1000.0f, 3.4f, 0.0f);
    speedcvgen = std::unique_ptr<SpeedCVGenerator>(new SpeedCVGenerator(p, estimator, targeter));
    angspeedcvgen = std::unique_ptr<AngularVelocityCVGenerator>(
            new AngularVelocityCVGenerator(p, estimator, targeter));
}

std::shared_ptr<MotionController> MotionController::GetInstance() {
    if (instance == nullptr) {
        instance = std::make_shared<MotionController>();
    }
    return instance;
}

void MotionController::Update() {
    // Targetの更新
    targeter->Update();
    // 推定機の更新
    estimator->Update();
    // 処理の実行
    if (targeter->HasMotion()) {
        Log::Speed::Push(estimator->GetVelocity().v);
        Log::Target::Push(targeter->GetTargetVelocity().v);
        switch (targeter->GetTarget().variety) {
            case Target::TargetVariety::STRAIGHT: {
                const auto speedcv = speedcvgen->Update();
                const auto angspeedcv = angspeedcvgen->Update();
                Log::Velocity::Push(speedcv);
                motor->SetStandby();
                motor->SetDuty(MotorPosition::RIGHT, speedcv);
                motor->SetDuty(MotorPosition::LEFT, speedcv);
                break;
            }
            case Target::TargetVariety::PIVOTTURN:
                break;
            case Target::TargetVariety::STOP: {
                // 止める
                motor->SetDuty(MotorPosition::RIGHT, 0);
                motor->SetDuty(MotorPosition::LEFT, 0);
                break;
            }
            default:
                const MotorDuty duty = { 0, 0 };
                motor->SetDuty(duty);
                motor->Stop(MotorPosition::RIGHT);
                motor->Stop(MotorPosition::LEFT);
                break;
        }
    } else {
        enable = false;
        motor->SetStandby();
        motor->SetDuty(MotorPosition::RIGHT, 0);
        motor->SetDuty(MotorPosition::LEFT, 0);
    }
}

void MotionController::SetMotion(Motion::MotionBase* _motion) {
    targeter->SetMotion(_motion);
}

void MotionController::Enable() {
    enable = true;
}

void MotionController::Disable() {
    enable = false;
}

const bool& MotionController::IsEnable() {
    return enable;
}